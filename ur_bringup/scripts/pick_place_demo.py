#!/usr/bin/env python3
"""GT-driven pick & place state machine (plan_il_vla.md 3.2).

    /scene/object_pose ──▶ [this] ──MoveGroup(/move_action)──▶ cuMotion ──▶ JTC ──▶ Isaac
                                └───/gripper_controller/gripper_cmd───▶ 2F-85

    READY → DETECT → PRE_GRASP → GRASP → CLOSE → LIFT → TRANSFER → PLACE → OPEN → RETRACT

Why GT and not perception (plan_il_vla.md 3.2, 2026-09-07)
----------------------------------------------------------
The IL/VLA policy never sees a pose -- it maps pixels + joint states to actions
(plan_il_vla.md 2.6/7-B). This state machine only *generates the trajectories the
policy imitates*, so whether it aimed using ground truth or an estimated pose
cannot change one byte of the dataset. Using GT therefore costs nothing in data
quality and saves the whole M1/M2 perception build before data collection starts.

    *** The exit is a remap, and it must stay that way. ***
Pose arrives ONLY through a topic (--object-topic). Nothing here reaches into the
simulator. When FoundationPose publishes /target/pose (same PoseStamped type),
this runs unchanged:

    ros2 run ur_bringup pick_place_demo.py --ros-args \
        -r /scene/object_pose:=/target/pose

Honest limit: on real hardware there is no GT, so this exact script is a
SIM-DATA tool. Real demos come from freedrive / the OMY leader instead
(plan_il_vla.md 3.3/3.5). That is a different path by design, not an oversight.

Usage
-----
    # after Isaac (--scene pick_place --table --grasp-attach ...), the control
    # stack, and a MoveIt launch (cuMotion one preferred):
    #
    #   *** ur_only:=false is REQUIRED. ***
    # It DEFAULTS TO TRUE, which loads the UR-arm-only model for interactive RViz.
    # That model has no gripper_frame, and MoveIt treats a constraint on an unknown
    # link as an ALREADY-SATISFIED goal -- so every Cartesian goal here returns
    # SUCCESS while the arm never moves. Nothing in the output says anything is
    # wrong. The launch's own help calls false the "programmatic cuMotion" mode,
    # which is what this script is.
    ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py \
        use_sim:=true ur_only:=false

    ros2 run ur_bringup pick_place_demo.py
    ros2 run ur_bringup pick_place_demo.py --ros-args -p cycles:=5 -p reset_each:=true

Exit code is 0 only if every requested cycle reported SUCCESS.
"""
import math
import sys
import time

import rclpy
from rclpy.action import ActionClient
from rclpy.duration import Duration
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

from control_msgs.action import GripperCommand
from geometry_msgs.msg import PoseStamped
from moveit_msgs.action import ExecuteTrajectory, MoveGroup
from moveit_msgs.srv import GetCartesianPath, GetPositionFK
from moveit_msgs.msg import (
    CollisionObject,
    Constraints,
    JointConstraint,
    MotionPlanRequest,
    OrientationConstraint,
    PlanningOptions,
    PlanningScene,
    PositionConstraint,
)
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, Float64MultiArray
from std_srvs.srv import Trigger

from tf2_ros import Buffer, TransformListener

ARM_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
# `ready`, never `home`. home/up/zero all have elbow_joint = 0 -- the arm fully
# extended, i.e. the elbow singularity that Servo refuses outright (HISTORY.md 15).
# The planner tolerates it better than Servo does, but starting a repeatable data
# run from a singular pose is asking for trouble.
READY = [0.0, -1.5707, 1.5707, -1.5707, -1.5707, 0.0]

# The finger pads. Used to MEASURE the tool0 -> TCP offset from TF rather than
# hardcoding it: the coupling standoff differs per set (+11 mm set 2, +18 mm set 3)
# and changing it is an explicit, documented operation (CLAUDE.md pitfall 7). A
# hardcoded number would silently aim 7 mm off after such a change.
FINGER_TIPS = ["robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"]


def quat_top_down(yaw):
    """Quaternion for 'tool0 z-axis pointing straight down, rotated by yaw'.

    R = Rz(yaw) . Rx(pi):  Rx(pi) flips tool0's +z (out of the flange) to world
    -z, then yaw spins the gripper about the vertical to line the pads up with
    the part. Worked out by hand rather than pulling in scipy/tf_transformations
    -- one formula, and the ROS python env here has neither guaranteed.
    """
    c, s = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (0.0, c, s, 0.0)          # (w, x, y, z)


def yaw_of(q):
    """Yaw about z from a (w,x,y,z) quaternion."""
    w, x, y, z = q
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def wrap_grasp_yaw(yaw):
    """Fold a part yaw into [-45, +45) degrees.

    The 2F-85 is a parallel gripper and the part is square in plan, so yaw and
    yaw+90 deg are the SAME grasp. Picking the smallest-magnitude equivalent
    keeps wrist_3 near zero instead of letting a randomised +170 deg part yaw
    drive it toward its limit mid-episode.
    """
    y = math.fmod(yaw, math.pi / 2.0)
    if y >= math.pi / 4.0:
        y -= math.pi / 2.0
    elif y < -math.pi / 4.0:
        y += math.pi / 2.0
    return y


class PickPlace(Node):
    def __init__(self):
        super().__init__("pick_place_demo")
        p = self.declare_parameter
        p("group", "ur_manipulator")
        # MUST be cuMotion's declared tool frame. Its plugin rejects a task-space
        # goal for any other link ("Target link 'tool0' does not match end effector
        # 'gripper_frame'") -- see urdf/common/robotiq_2f85_macro.xacro.
        p("ee_link", "gripper_frame")
        p("object_topic", "/scene/object_pose")
        p("place_topic", "/scene/place_pose")
        # "" = whatever MoveIt's default pipeline is. The cuMotion launch already
        # makes isaac_ros_cumotion the default, so leave this empty there.
        p("pipeline_id", "")
        p("planner_id", "")
        p("cycles", 1)
        p("reset_each", False)           # call /scene/reset_episode before each cycle
        p("approach_height", 0.15)       # [m] above the part for PRE_GRASP / TRANSFER
        p("lift_height", 0.20)           # [m] to raise the part after CLOSE
        p("place_clearance", 0.02)       # [m] gap above the marker when releasing
        # Where the jaws STALL on a 35 mm part -- MEASURED, not derived.
        #
        # This used to be 0.8 (fully shut). With --grasp-attach that value is
        # reachable, because the attach disables the part's collider at 0.25 and
        # the fingers then close straight through it. The recorded gripper channel
        # therefore sat at 1.0 while "holding", which a real 2F-85 can never
        # report: it stalls on the part. The dataset had 40% of frames at 1.0 and
        # only 1.8% anywhere near the real stall, so a policy trained on it would
        # meet an unseen observation the first time it grips real hardware.
        #
        # 0.599 is where the jaws actually stop, read off /joint_states with the
        # collider left ON (Isaac started without --grasp-attach, gripper
        # commanded to 0.8, joint held 0.5988 for 20 s). The pad-gap formula
        # predicted 0.470 and was WRONG AGAIN -- it is built on inner_finger link
        # origins, the same proxy that caused the 31 mm aim error and the
        # fabricated gripper inversion (HISTORY.md 26.2, 28, 33).
        #
        # Depends on part width: re-measure if --object-size changes.
        p("grip_closed", 0.52)
        # Close in TWO stages. Commanding grip_closed directly slams the fingers
        # shut in a few physics steps and the part is EJECTED before Isaac's grasp
        # attach (D5) can fire -- measured: part at (0.597, -0.096, 0.225) after
        # GRASP, then (1.258, -0.186, 0.036) after LIFT, i.e. flung across the
        # table, and the attach fired in only 2 of 15 cycles. So: stop part-way,
        # let the attach fire, then close the rest of the way onto a part whose
        # collider is already disabled.
        #
        # 0.28 sits just above --grasp-close 0.25 (the attach threshold) and below
        # the opening where the pads reach a 35 mm part. The pad-gap figure that
        # bounds the second half of that window is a LINK-ORIGIN proxy, and link
        # origins are exactly what produced two wrong conclusions on 2026-09-09
        # (HISTORY.md 26.2, 28) -- so this value is confirmed by looking at the
        # grasp-moment wrist frames, not by the arithmetic.
        p("grip_preclose", 0.28)
        p("grip_open", 0.0)
        # Jaw opening for the approach, the descent and the release (NOT the final
        # close, and not the recovery open). Fully open, on both backends.
        #
        # This was briefly 0.30 in sim, to dodge a "descending fully open fouls on
        # the part" effect. That effect was not real: the gripper convention had
        # been mis-diagnosed as inverted, so commanding 0 was physically CLOSING
        # the jaws and the arm was descending shut. HISTORY.md 28.
        p("grip_approach", 0.0)
        p("grip_effort", 60.0)
        p("vel_scale", 0.2)
        p("acc_scale", 0.2)
        p("plan_time", 10.0)
        p("plan_attempts", 10)
        # Tight on purpose. These are the ARRIVAL accuracy at PRE_GRASP, and the
        # straight-line descent that follows preserves whatever error is left: at
        # the old ori_tol of 0.05 rad (~3 deg) a 0.15 m descent lands ~8 mm off
        # sideways, and the part was measured moving 11 mm before the gripper even
        # closed. A nudged part then gets caught corner-on, the fingers stall below
        # the attach threshold (grasp_close 0.25), and the squeeze flings it away
        # -- observed: part ended at (0.194, 0.150, 0.040), across the room.
        # Reverted to the values that plan reliably. Tightening these to
        # 0.005 / 0.01 rad did NOT stop the part being nudged (still 11 mm) and
        # started causing outright PRE_GRASP/LIFT planning failures -- so approach
        # tilt was not the cause. Keep them loose and find the real offset with
        # report_pose_error() instead of guessing.
        p("pos_tol", 0.01)               # [m] goal position tolerance
        p("ori_tol", 0.05)               # [rad] per-axis orientation tolerance
        p("tcp_offset", -1.0)            # [m] tool0->pads; <0 means "measure from TF"
        # [m] from the finger_tip_link ORIGIN to the middle of the rubber pad,
        # along the tool axis. The link origin is at the fingertip's proximal end;
        # robotiq_description's collision mesh runs -6..+51 mm from it. Applies to
        # sim and real alike -- see measure_tcp_offset().
        p("pad_offset", 0.032)
        p("settle_time", 0.7)            # [s] let physics settle before judging
        # --- arrival checking (HISTORY.md 32) --------------------------------
        # Two tolerances, because the two kinds of move need different things.
        # precise: end of a straight-line move, where the gripper then acts.
        # approach: a waypoint 150 mm away that the next absolute move re-zeros.
        # Measured: PRE_GRASP/TRANSFER plateau at 7.2-7.3 mm and never reach 3 mm,
        # so the old single 3 mm tolerance burned the full 5 s timeout twice per
        # cycle -- 10 s of 41 s, all of it recorded as stationary frames.
        p("settle_tol_precise", 0.003)   # [m]
        p("settle_tol_approach", 0.010)  # [m]
        # Give up waiting once the error stops improving for this long. What
        # remains after a joint-space move is a steady-state offset, not a
        # decaying transient: it does not shrink no matter how long you wait.
        p("settle_plateau", 0.8)         # [s]
        # 0.06 was too generous: a cycle that DROPPED the part mid-transfer still
        # passed, because the part happened to land 57 mm from the marker. That
        # episode then gets SAVED and teaches the policy that fumbling is fine.
        p("place_tol", 0.035)            # [m] how close to the marker counts as placed
        # Grasp ABOVE the part centre. Aiming the TCP at the centre of a 50 mm cube
        # sitting on the table puts the fingertips at the table surface: the descent
        # was measured ending +21.5 mm HIGH (i.e. blocked from below) while shoving
        # the part sideways. The pads are 85 mm apart open and only ~0 mm closed, so
        # gripping the upper part of the cube holds it just as well -- and it is what
        # you would do with a real 2F-85 on a part resting on a bench.
        p("grasp_z_offset", 0.015)       # [m] above the part origin (measured, HISTORY.md 34)
        # How far BELOW the real surface to put the table's collision box.
        # The table is in the planning scene to stop the ARM sweeping through the
        # work surface -- not to stop the FINGERS approaching it, which every grasp
        # of a part standing on that surface must do. Added at full height, cuMotion
        # refused every grasp (plan_failed_grasp on 3 of 3) even though the same
        # poses executed fine without it. Sinking the box a couple of centimetres
        # keeps the arm's links (all far thicker than this) blocked while leaving
        # the fingertips room.
        #   Proper fix, if this ever matters: an AllowedCollisionMatrix entry for
        #   work_table vs the gripper links only. Not done because it is unverified
        #   whether the cuMotion plugin honours the ACM.
        #   DEFAULT OFF for sim data generation, and that is a measured decision:
        #     no table   6/8 succeeded
        #     table on   5/8 succeeded, and once the layout moved it produced
        #                plan_failed_pre_grasp on poses a plan-only sweep showed
        #                are perfectly reachable WITHOUT it (y = -0.30..+0.40, both
        #                grasp and approach heights, all ok).
        #   So it costs episodes and buys nothing here. The arm hitting the table in
        #   sim just wastes a cycle that gets discarded.
        #   *** ON REAL HARDWARE SET THIS >= 0. *** There a plan through the table is
        #   a crash, not a wasted episode -- and pair it with an ACM entry for
        #   work_table vs the gripper links so grasps near the surface still plan.
        p("table_sink", -1.0)            # [m]; <0 skips the table entirely
        # Drive il_recorder so every cycle becomes a labelled LeRobot episode.
        # The state machine owns the episode lifecycle, so run the recorder with
        # auto_reset:=false -- otherwise both call /scene/reset_episode and the part
        # is re-placed twice per cycle.
        p("record", False)

        g = lambda n: self.get_parameter(n).value          # noqa: E731
        self.group = g("group")
        self.ee = g("ee_link")
        self.cycles = int(g("cycles"))
        self.reset_each = bool(g("reset_each"))
        for k in ("approach_height", "lift_height", "place_clearance", "grip_closed",
                  "grip_preclose",
                  "grip_open", "grip_approach", "grip_effort", "vel_scale", "acc_scale", "plan_time",
                  "pos_tol", "ori_tol", "tcp_offset", "pad_offset", "settle_time", "place_tol",
                  "settle_tol_precise", "settle_tol_approach", "settle_plateau",
                  "grasp_z_offset", "table_sink"):
            setattr(self, k, float(g(k)))
        self.plan_attempts = int(g("plan_attempts"))
        self.record = bool(g("record"))
        self.pipeline_id, self.planner_id = str(g("pipeline_id")), str(g("planner_id"))

        self.object_pose = None
        self.place_pose = None
        self.grasp_active = False
        self.joints = None

        sensor_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(PoseStamped, g("object_topic"), self._on_object, 10)
        # Latched by the publisher -- match it or a late start never sees the marker.
        self.create_subscription(
            PoseStamped, g("place_topic"), self._on_place,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_subscription(Bool, "/scene/grasp_active", self._on_grasp, 10)
        self.table_box = None
        self.create_subscription(
            Float64MultiArray, "/scene/table_box", self._on_table,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.scene_pub = self.create_publisher(PlanningScene, "/planning_scene", 10)
        self.create_subscription(JointState, "/joint_states", self._on_joints, sensor_qos)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.fk_cli = self.create_client(GetPositionFK, "/compute_fk")
        self.cart_cli = self.create_client(GetCartesianPath, "/compute_cartesian_path")
        self.exec_cli = ActionClient(self, ExecuteTrajectory, "/execute_trajectory")
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.grip = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")
        self.reset_cli = self.create_client(Trigger, "/scene/reset_episode")
        self.il = {n: self.create_client(Trigger, f"/il/{n}_episode")
                   for n in ("start", "stop", "discard")}

    # ---- subscriptions ----------------------------------------------------
    def _on_object(self, m): self.object_pose = m
    def _on_place(self, m): self.place_pose = m
    def _on_grasp(self, m): self.grasp_active = m.data
    def _on_table(self, m): self.table_box = list(m.data)
    def _on_joints(self, m): self.joints = m

    # ---- helpers ----------------------------------------------------------
    def wait_for_clock(self, timeout=30.0):
        """Under use_sim_time, block until /clock has actually been received.

        *** Do not remove this, and do not compute a ROS-time deadline before it. ***
        With use_sim_time the node clock reads 0 until the first /clock message
        lands. A deadline built then is `0 + timeout`, and the instant real sim
        time (thousands of seconds) arrives every such wait expires at once. The
        symptom is a bring-up wait that "times out" immediately and looks like the
        topic is dead -- which is exactly how this was first misdiagnosed.

        Bounded by the MONOTONIC clock, since ROS time is the thing we are waiting
        for and cannot be used to time its own arrival.
        """
        if not self.get_parameter("use_sim_time").value:
            return True
        end = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if self.get_clock().now().nanoseconds > 0:
                self.get_logger().info("/clock is live; using sim time")
                return True
        self.get_logger().error(
            f"use_sim_time is set but no /clock after {timeout:.0f}s -- is Isaac running?")
        return False

    def spin_until(self, pred, timeout, what):
        """Spin this node until pred() or timeout. Returns True if pred() held."""
        end = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            if pred():
                return True
        self.get_logger().error(f"timed out waiting for {what} ({timeout:.0f}s)")
        return False

    def sleep(self, seconds):
        end = self.get_clock().now() + Duration(seconds=seconds)
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.02)

    def wait_for(self, pred, timeout, what):
        """Spin until `pred()` holds, or `timeout` (sim seconds) elapses.

        Replaces the fixed sleeps that used to guard the same conditions. A blind
        sleep is wrong in both directions: too short and the check below it runs
        on stale physics, too long and the surplus is recorded as stationary
        frames -- and stationary frames are what taught ACT to hold still
        (HISTORY.md 31). Waiting for the event itself is both faster and stricter.

        Returns True if the condition was met, False on timeout. Callers keep
        their own check afterwards, so a timeout degrades to the old behaviour
        rather than skipping a guard.
        """
        end = self.get_clock().now() + Duration(seconds=timeout)
        while rclpy.ok() and self.get_clock().now() < end:
            if pred():
                return True
            rclpy.spin_once(self, timeout_sec=0.02)
        self.get_logger().debug(f"{what}: timed out after {timeout:.1f}s")
        return False

    def wait_part_still(self, timeout, what, tol=0.001):
        """Spin until the part stops moving, or `timeout`.

        Used before judging placement. The part is dropped from a couple of
        centimetres and needs to come to rest before its pose means anything --
        but it usually rests in a fraction of the old fixed 0.7 s.
        """
        end = self.get_clock().now() + Duration(seconds=timeout)
        prev = None
        still = 0
        while rclpy.ok() and self.get_clock().now() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            if self.object_pose is None:
                continue
            q = self.object_pose.pose.position
            cur = (q.x, q.y, q.z)
            if prev is not None and max(abs(a - b) for a, b in zip(cur, prev)) < tol:
                still += 1
                if still >= 5:          # ~5 consecutive quiet samples
                    return True
            else:
                still = 0
            prev = cur
        return False

    def measure_tcp_offset(self):
        """Distance along tool0's z from tool0 to the GRIPPING SURFACE.

        Two parts, and the second one used to be missing:

        1. gripper_frame -> robotiq_85_*_finger_tip_link, read from TF so it
           follows the URDF instead of duplicating it (the coupling standoff
           differs per set -- CLAUDE.md pitfall 7).
        2. that link's ORIGIN is not the pad. It sits at the proximal end of the
           fingertip: the collision mesh (robotiq_description
           meshes/collision/left_finger_tip.stl) spans z = -6..+51 mm from it, and
           the tip link's rotation relative to gripper_frame is identity, so the
           rubber pad centre is `pad_offset` further along the same axis.

        Treating the link origin as the pad aimed the whole grasp ~31 mm high.
        In Isaac that drove the fingers into the table (the arm jammed and shoved
        the part aside); on real hardware it would have done the same. This is a
        SHARED correction, not a sim workaround -- which is why pad_offset has a
        real default rather than living in pick_place_sim.yaml.

        Cross-check, two independent models agreeing to ~1 mm:
            URDF  : tip link 0.0983 + mesh    -> pad centre ~0.1303 m
            Isaac : pad mesh measured in USD  -> pad centre  0.1294 m
        """
        if self.tcp_offset >= 0.0:
            self.get_logger().info(f"tcp_offset pinned by parameter: {self.tcp_offset:.4f} m")
            return True
        if not self.spin_until(
                lambda: all(self.tf_buffer.can_transform(self.ee, t, rclpy.time.Time())
                            for t in FINGER_TIPS),
                15.0, f"TF {self.ee} -> finger tips"):
            return False

        # Wait for the fingers to actually BE open, then for the reading to stop
        # moving. A fixed sleep is not enough: the same code measured 0.0983 m one
        # run and 0.0452 m the next, purely on gripper timing, and a wrong offset
        # aims the whole grasp at the wrong height.
        def finger():
            if self.joints is None or "finger_joint" not in self.joints.name:
                return None
            return self.joints.position[self.joints.name.index("finger_joint")]

        if not self.spin_until(
                lambda: finger() is not None and abs(finger() - self.grip_open) < 0.02,
                15.0, "finger_joint to reach the open position"):
            return False

        def sample():
            zs = [self.tf_buffer.lookup_transform(self.ee, t, rclpy.time.Time())
                  .transform.translation.z for t in FINGER_TIPS]
            return sum(zs) / len(zs)

        prev = sample()
        stable = 0
        for _ in range(100):
            self.sleep(0.1)
            cur = sample()
            stable = stable + 1 if abs(cur - prev) < 5e-4 else 0
            prev = cur
            if stable >= 3:
                break
        else:
            self.get_logger().error("finger tip TF never settled; refusing to guess the TCP")
            return False
        self.tcp_offset = prev + self.pad_offset
        self.get_logger().info(
            f"measured tcp_offset ({self.ee} -> finger pad, gripper open) "
            f"= {self.tcp_offset:.4f} m  "
            f"(tip link {prev:.4f} + pad_offset {self.pad_offset:.4f})")
        return True

    def tool0_pose_for_tcp(self, x, y, z, yaw):
        """tool0 pose that puts the TCP at (x,y,z) with a top-down grasp.

        The TCP sits tcp_offset ahead of tool0 along tool0's z. Pointing straight
        down, tool0's z is world -z, so tool0 must sit that far ABOVE the TCP.
        """
        return (x, y, z + self.tcp_offset), quat_top_down(yaw)

    # ---- motion primitives ------------------------------------------------
    def _send_move(self, constraints, what):
        req = MotionPlanRequest()
        req.group_name = self.group
        req.num_planning_attempts = self.plan_attempts
        req.allowed_planning_time = self.plan_time
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale
        if self.pipeline_id:
            req.pipeline_id = self.pipeline_id
        if self.planner_id:
            req.planner_id = self.planner_id
        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req
        opts = PlanningOptions()
        opts.plan_only = False                       # plan AND execute
        opts.planning_scene_diff.is_diff = True
        opts.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options = opts

        send = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{what}: goal rejected")
            return False
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=120.0)
        res = res_fut.result()
        if res is None:
            self.get_logger().error(f"{what}: no result (timeout)")
            return False
        code = res.result.error_code.val
        if code != 1:
            self.get_logger().error(f"{what}: MoveGroup error_code={code} (1=SUCCESS)")
            return False
        self.get_logger().info(f"{what}: ok")
        return True

    def move_joints(self, q, what):
        c = Constraints()
        for name, pos in zip(ARM_JOINTS, q):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = jc.tolerance_below = 0.01
            jc.weight = 1.0
            c.joint_constraints.append(jc)
        return self._send_move(c, what)

    def move_pose(self, xyz, quat, what):
        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = self.ee
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.SPHERE
        prim.dimensions = [self.pos_tol]
        pc.constraint_region.primitives.append(prim)
        pose = PoseStamped().pose
        pose.position.x, pose.position.y, pose.position.z = xyz
        pose.orientation.w = 1.0
        pc.constraint_region.primitive_poses.append(pose)
        pc.weight = 1.0
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = self.ee
        oc.orientation.w, oc.orientation.x, oc.orientation.y, oc.orientation.z = quat
        oc.absolute_x_axis_tolerance = self.ori_tol
        oc.absolute_y_axis_tolerance = self.ori_tol
        oc.absolute_z_axis_tolerance = self.ori_tol
        oc.weight = 1.0
        c.orientation_constraints.append(oc)

        self.get_logger().info(
            f"{what}: {self.ee} -> xyz=({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})")
        ok = self._send_move(c, what)
        if ok:
            # Approach tolerance, not grasp tolerance. This is a joint-space move
            # to a waypoint 150 mm above the part; the straight-line descent that
            # follows targets ABSOLUTE coordinates and re-zeros whatever is left
            # here (measured: PRE_GRASP ends 7.4 mm off, GRASP then settles to
            # 0.2 mm). Demanding 3 mm here bought nothing and cost 5 s.
            self.wait_settled(xyz, what, tol=self.settle_tol_approach)
            self.report_pose_error(xyz, what)
        # Log where the part is after every motion. Without this a knocked-over part
        # only shows up as a generic "grasp failed" at the end, with no clue which
        # motion did it -- cuMotion plans joint-space minimum-jerk paths, so the
        # gripper does NOT travel in a straight Cartesian line between waypoints and
        # can sweep through the part on the way down.
        if self.object_pose is not None:
            q = self.object_pose.pose.position
            self.get_logger().info(
                f"{what}: part now at ({q.x:.3f}, {q.y:.3f}, {q.z:.3f})")
        return ok

    def move_linear(self, xyz, quat, what, min_fraction=0.95):
        """Straight-line Cartesian move of `ee_link`, for approach and retreat.

        *** Approach/retreat MUST NOT be planned moves. ***
        cuMotion returns joint-space minimum-jerk trajectories, so the path between
        two poses is NOT a straight Cartesian line. Descending onto the part that
        way swept a finger through it: the part was measured at (0.600, 0.000,
        0.225) after PRE_GRASP and (0.621, -0.010, 0.233) after GRASP -- knocked
        21 mm sideways before the gripper ever closed.

        compute_cartesian_path interpolates in Cartesian space instead, so the
        gripper comes straight down between the open pads. `fraction` reports how
        much of the requested line was solvable; anything short of the whole line
        means the gripper would stop short of the part, so treat it as a failure
        rather than executing a partial descent.
        """
        if not self.cart_cli.wait_for_service(timeout_sec=10.0):
            self.get_logger().error(f"{what}: /compute_cartesian_path unavailable")
            return False
        req = GetCartesianPath.Request()
        req.header.frame_id = "base_link"
        req.group_name = self.group
        req.link_name = self.ee
        target = PoseStamped().pose
        target.position.x, target.position.y, target.position.z = xyz
        target.orientation.w, target.orientation.x, target.orientation.y, target.orientation.z = quat
        req.waypoints = [target]
        req.max_step = 0.005                 # 5 mm interpolation
        # *** Do NOT leave this at 0.0. ***
        # 0.0 DISABLES joint-space jump filtering, so the solver will happily
        # return a path that flips an IK branch mid-descent and still report
        # fraction = 1.00. Measured: a descent that claimed 100% ended 50 mm off in
        # y while the part had barely moved -- the arm went somewhere else entirely,
        # not blocked, just following a discontinuous path the controller could not
        # track. With a threshold the path is TRUNCATED at the jump, fraction drops
        # below min_fraction, and the cycle fails cleanly instead of thrashing.
        req.jump_threshold = 5.0             # relative joint-space jump factor
        req.avoid_collisions = True
        req.max_velocity_scaling_factor = self.vel_scale
        req.max_acceleration_scaling_factor = self.acc_scale

        self.get_logger().info(
            f"{what}(linear): {self.ee} -> xyz=({xyz[0]:.3f}, {xyz[1]:.3f}, {xyz[2]:.3f})")
        fut = self.cart_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        res = fut.result()
        if res is None:
            self.get_logger().error(f"{what}: cartesian path service timed out")
            return False
        if res.fraction < min_fraction:
            self.get_logger().error(
                f"{what}: only {res.fraction * 100:.0f}% of the straight line was solvable "
                f"(need {min_fraction * 100:.0f}%)")
            return False
        if not self.exec_cli.wait_for_server(timeout_sec=10.0):
            self.get_logger().error(f"{what}: /execute_trajectory unavailable")
            return False
        goal = ExecuteTrajectory.Goal()
        goal.trajectory = res.solution
        send = self.exec_cli.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        h = send.result()
        if h is None or not h.accepted:
            self.get_logger().error(f"{what}: execute goal rejected")
            return False
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=120.0)
        r = rf.result()
        if r is None or r.result.error_code.val != 1:
            code = "timeout" if r is None else r.result.error_code.val
            self.get_logger().error(f"{what}: execution failed ({code})")
            return False
        self.get_logger().info(f"{what}: ok (fraction {res.fraction:.2f})")
        # Tight on purpose: the gripper acts at the end of these moves.
        self.wait_settled(xyz, what, tol=self.settle_tol_precise)
        self.report_pose_error(xyz, what)
        if self.object_pose is not None:
            q = self.object_pose.pose.position
            self.get_logger().info(f"{what}: part now at ({q.x:.3f}, {q.y:.3f}, {q.z:.3f})")
        return True

    def wait_settled(self, want, what, tol=None, timeout=5.0):
        """Wait until `ee_link` has actually REACHED `want`.

        *** The controller reports SUCCEEDED before the arm gets there. ***
        Measured at the end of a GRASP descent that MoveIt called a success:
            commanded (0.600, 0.000, 0.323)
            actual    (0.618, -0.004, 0.343)   err (+17.9, -4.2, +19.8) mm
        The sim arm trails its position command, so acting on "trajectory done"
        means closing the gripper ~18 mm off-centre -- exactly the direction the
        part was seen being pushed. Everything downstream (part knocked, corner
        grip, fingers stalling below the attach threshold, part flung across the
        room) follows from not waiting here.
        """
        tol = self.settle_tol_precise if tol is None else tol
        end = time.monotonic() + timeout
        best = None
        best_t = time.monotonic()
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)
            try:
                t = self.tf_buffer.lookup_transform(
                    "base_link", self.ee, rclpy.time.Time()).transform.translation
            except Exception:
                continue
            err = math.sqrt((t.x - want[0]) ** 2 + (t.y - want[1]) ** 2 + (t.z - want[2]) ** 2)
            if best is None or err < best - 0.0002:      # 0.2 mm = real improvement
                best, best_t = err, time.monotonic()
            if err <= tol:
                self.get_logger().info(f"{what}: settled ({err * 1000:.1f} mm)")
                return True
            # PLATEAU EXIT. What is left after a joint-space move is a STEADY-STATE
            # offset, not a decaying oscillation: the topic_based backend follows
            # the position command and whatever gap remains does not shrink with
            # time. Measured, every cycle: PRE_GRASP and TRANSFER stalled at
            # 7.2/7.3 mm and burned the full 5 s timeout -- 10 s per 41 s cycle,
            # 24% of every episode spent holding still for nothing. That idle mass
            # is what taught ACT to freeze (HISTORY.md 31).
            #
            # Waiting longer cannot help once the error stops improving, so stop.
            # The tolerance itself is NOT relaxed: the caller still learns it did
            # not reach `tol`, and the tight check still guards GRASP/PLACE, where
            # arriving late means closing the gripper off-centre.
            if time.monotonic() - best_t > self.settle_plateau:
                self.get_logger().info(
                    f"{what}: converged at {best * 1000:.1f} mm "
                    f"(no improvement for {self.settle_plateau:.1f}s; tol "
                    f"{tol * 1000:.0f} mm)")
                return best <= tol
        self.get_logger().warn(
            f"{what}: did NOT settle within {tol * 1000:.0f} mm "
            f"(best {(best or 0) * 1000:.1f} mm) after {timeout:.0f}s")
        return False

    def report_pose_error(self, want, what):
        """Where did `ee_link` ACTUALLY end up vs where we asked for?

        A systematic offset here means the model and the simulator disagree (the
        gripper is described twice -- URDF for planning, USD for physics -- and
        CLAUDE.md pitfall 7 is exactly about keeping them in sync). Without this
        the only symptom is the part being nudged during a descent that the
        planner reports as a perfect straight line.
        """
        try:
            tr = self.tf_buffer.lookup_transform("base_link", self.ee, rclpy.time.Time())
        except Exception:
            return
        t = tr.transform.translation
        d = (t.x - want[0], t.y - want[1], t.z - want[2])
        self.get_logger().info(
            f"{what}: {self.ee} actual ({t.x:.4f}, {t.y:.4f}, {t.z:.4f}) "
            f"err ({d[0] * 1000:+.1f}, {d[1] * 1000:+.1f}, {d[2] * 1000:+.1f}) mm")

    def gripper(self, position, what):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = self.grip_effort
        send = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send)
        handle = send.result()
        if handle is None or not handle.accepted:
            self.get_logger().error(f"{what}: gripper goal rejected")
            return False
        res_fut = handle.get_result_async()
        rclpy.spin_until_future_complete(self, res_fut, timeout_sec=15.0)
        # Deliberately NOT checking reached_goal: the 2F-85 stalls on the part,
        # which is exactly what a successful grasp looks like. Whether the part
        # actually came up is judged from GT after LIFT instead.
        self.get_logger().info(f"{what}: commanded {position:.2f}")
        # Wait for the jaws to ARRIVE instead of sleeping a fixed 0.5 s. Same
        # reasoning as wait_settled: a blind sleep is either too short (acting on
        # jaws still moving) or too long (recorded as stationary frames, which is
        # what taught ACT to freeze -- HISTORY.md 31). Plateau-exit covers the
        # stall-on-the-part case, where finger_joint deliberately never arrives.
        #
        # *** The plateau exit must not fire BEFORE the jaws start moving. ***
        # It did: `best` was seeded from the first sample, nothing had moved yet,
        # and 0.3 s later this returned with the jaws still open. The attach check
        # that follows then had only its own 0.8 s to cover the whole close, which
        # is enough in a warm process and not enough in a fresh one -- exactly the
        # non-monotonic attach_failed pattern seen in the grasp-depth sweep
        # (0.010 ok, 0.007 fail, 0.005 ok, 0.003 fail) that looked like a depth
        # effect and was not. So: require observed motion before allowing it.
        end = time.monotonic() + 2.5
        start_fj = None
        best, best_t = None, time.monotonic()
        moved = False
        while time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.02)
            fj = self.finger_joint()
            if fj is None:
                continue
            if start_fj is None:
                start_fj = fj
            if abs(fj - start_fj) > 0.02:
                moved = True
            err = abs(fj - position)
            if err <= 0.01:
                return True
            if best is None or err < best - 0.005:
                best, best_t = err, time.monotonic()
            # Stalled ON THE PART (a real grasp) -- but only once it has moved.
            if moved and time.monotonic() - best_t > 0.3:
                return True
        return True

    def il_call(self, which):
        """start / stop(save) / discard an il_recorder episode. No-op unless record."""
        if not self.record:
            return True
        cli = self.il[which]
        if not cli.service_is_ready() and not cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().error(f"/il/{which}_episode unavailable -- is il_recorder running?")
            return False
        fut = cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=20.0)
        r = fut.result()
        self.get_logger().info(f"il/{which}: {r.message if r else 'no response'}")
        return r is not None and r.success

    def reset_episode(self):
        if not self.reset_cli.service_is_ready() and \
           not self.reset_cli.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn("/scene/reset_episode unavailable; skipping reset")
            return False
        fut = self.reset_cli.call_async(Trigger.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        r = fut.result()
        if r is not None:
            self.get_logger().info(f"reset_episode: {r.message}")
        # The reset teleports the objects; wait for them to come to rest rather
        # than sleeping 1.0 s. This is OUTSIDE the recorded window (recording
        # starts later, deliberately, so the episode does not open with a part
        # teleporting into place) so it costs cycle time, not data quality.
        self.wait_part_still(1.0, "reset settle")
        return r is not None and r.success

    def add_table_to_scene(self):
        """Put the work surface into the PLANNING scene, not just the physics one.

        The planner otherwise has no idea the table exists: cuMotion happily routes
        a link through it, execution reports SUCCEEDED, and the arm is actually
        sitting on the table 50 mm from where it was told to be. Geometry comes from
        Isaac on /scene/table_box so the planner and the simulator cannot disagree
        -- the same numbers that spawned the slab.
        """
        if not self.spin_until(lambda: self.table_box is not None, 10.0,
                               "/scene/table_box"):
            self.get_logger().warn("no table geometry; planning WITHOUT the work surface")
            return
        if self.table_sink < 0.0:
            self.get_logger().warn("table_sink < 0: planning WITHOUT the work surface")
            return
        cx, cy, cz, sx, sy, sz = self.table_box
        cz -= self.table_sink
        co = CollisionObject()
        co.header.frame_id = "base_link"
        co.id = "work_table"
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [sx, sy, sz]
        co.primitives.append(prim)
        pose = PoseStamped().pose
        pose.position.x, pose.position.y, pose.position.z = cx, cy, cz
        pose.orientation.w = 1.0
        co.primitive_poses.append(pose)
        co.operation = CollisionObject.ADD

        scene = PlanningScene()
        scene.is_diff = True
        scene.world.collision_objects.append(co)
        for _ in range(5):                 # /planning_scene is not latched
            self.scene_pub.publish(scene)
            self.sleep(0.2)
        self.get_logger().info(
            f"added work_table to the planning scene: centre ({cx:.3f}, {cy:.3f}, {cz:.3f}) "
            f"size ({sx:.2f}, {sy:.2f}, {sz:.2f}), sunk {self.table_sink * 1000:.0f} mm")

    def finger_joint(self):
        if self.joints is None or "finger_joint" not in self.joints.name:
            return None
        return self.joints.position[self.joints.name.index("finger_joint")]

    def unjam_gripper(self, what):
        """Bring the 2F-85 linkage back inside its limits. ORDER AND REPEATS MATTER.

        Measured recovery of finger_joint = -0.558 (range [0, 0.8]):
          * teleport alone                                    -> springs back to -0.558
          * detach + reset_episode                            -> no effect
          * open command alone                                -> no effect
          * detach, reset_episode, OPEN command, then teleport -> -0.197
          * ...then two more teleports                        -> 0.000  (recovered)

        Teleporting first fails because ros2_control is still holding a CLOSED
        position target, so the drive shoves the linkage straight back out. The part
        must be moved away AND the target set to `open` BEFORE the teleport agrees
        with what the controller is asking for. One teleport is also not enough --
        the linkage unwinds over several physics steps.
        """
        def call(name, timeout=5.0):
            cli = self.create_client(Trigger, f"/scene/{name}")
            if not cli.wait_for_service(timeout_sec=timeout):
                return False
            fut = cli.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
            return fut.result() is not None

        call("detach_object")
        call("reset_episode")
        self.gripper(self.grip_open, f"OPEN({what})")   # target must be open FIRST
        for _ in range(4):
            call("reset_gripper")
            self.sleep(0.5)
            fj = self.finger_joint()
            if fj is not None and -0.01 <= fj <= 0.81:
                self.get_logger().info(f"{what}: gripper recovered (finger_joint {fj:.3f})")
                return True
        self.get_logger().error(
            f"{what}: gripper still jammed at {self.finger_joint()}; restart Isaac")
        return False

    def recover_gripper(self):
        """Start-up check: a run that died mid-grasp leaves the linkage out of range,
        and nothing downstream -- least of all the TCP measurement -- is meaningful
        from there."""
        if not self.spin_until(lambda: self.joints is not None, 15.0, "/joint_states"):
            return False
        fj = self.finger_joint()
        if fj is None or -0.01 <= fj <= 0.81:
            return True
        self.get_logger().warn(
            f"finger_joint is {fj:.3f}, outside [0, 0.8] -- a previous run probably "
            f"died mid-grasp. Unjamming.")
        return self.unjam_gripper("recover")

    def check_ee_link_known(self):
        """Refuse to run if move_group's model has no `ee_link`.

        *** This guard is not optional. ***
        MoveIt treats a goal constraint on an UNKNOWN link as an ALREADY-SATISFIED
        constraint: it logs "Constraint invalid" at WARN, then returns SUCCESS
        without planning or moving anything. The action result is error_code=1, so
        this script happily reported "PRE_GRASP: ok / GRASP: ok / LIFT: ok" through
        an entire cycle in which the arm never left its start pose. The only visible
        symptom was the part not being picked up.

        The usual cause is launching MoveIt with ur_only:=true (its DEFAULT), which
        loads the UR-arm-only model with no gripper links at all.

        Asking /compute_fk is the real question: it is answered by move_group's own
        kinematic model. Checking the `robot_description` PARAMETER is not -- with
        ur_only:=false the model arrives on the topic and the parameter still holds
        an arm-only default, so that check reports a false failure.
        """
        if not self.fk_cli.wait_for_service(timeout_sec=15.0):
            self.get_logger().error("/compute_fk unavailable; cannot verify the planning model")
            return False
        if not self.spin_until(lambda: self.joints is not None, 10.0, "/joint_states"):
            return False
        req = GetPositionFK.Request()
        req.header.frame_id = "base_link"
        req.fk_link_names = [self.ee]
        req.robot_state.joint_state = self.joints
        fut = self.fk_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        res = fut.result()
        if res is None or res.error_code.val != 1 or not res.pose_stamped:
            self.get_logger().error(
                f"move_group's model does not know link '{self.ee}'. Every Cartesian "
                f"goal would silently return SUCCESS without moving.")
            self.get_logger().error(
                "  fix: relaunch MoveIt with ur_only:=false (it defaults to true, "
                "which loads the UR-arm-only model)")
            return False
        self.get_logger().info(f"move_group's model knows '{self.ee}' (FK ok)")
        return True

    # ---- the cycle --------------------------------------------------------
    def run_cycle(self, index):
        """One pick&place, recorded as one episode when `record` is set.

        Recording starts AFTER the reset has settled, so the episode does not open
        with the part teleporting into place -- a jump the policy would otherwise
        have to explain. A failed cycle is DISCARDED rather than saved: a demo that
        knocked the part over is worse than no demo (plan_il_vla.md 6.5).
        """
        ok, reason = self._run_cycle_inner(index)
        if self.record:
            self.il_call("stop" if ok else "discard")
        if ok:
            # Outside the recorded window on purpose -- see the note in
            # _run_cycle_inner where this move used to live. The failure path does
            # the same thing in _release_after_failure().
            if not self.move_joints(READY, "READY(next cycle)"):
                return False, "plan_failed_ready"
        if not ok:
            # Let go NOW, in this process, while the scene services are still up.
            # A cycle that fails after CLOSE leaves the fingers loaded against the
            # part; the 2F-85 mimic linkage is then driven outside its limits
            # (measured -0.558, -0.974 against a range of [0, 0.8]) and NOTHING
            # recovers it short of restarting Isaac. Unattended collection cannot
            # afford one bad cycle to end the run, so unwind immediately.
            self._release_after_failure()
        return ok, reason

    def _release_after_failure(self):
        self.unjam_gripper("cleanup")
        # Put the arm back at READY. A failed cycle leaves it wherever it stopped --
        # often mid-descent over the part -- and the NEXT cycle then plans PRE_GRASP
        # from that arbitrary configuration. Measured: after one failure the rest of
        # the run collapsed into plan_failed_pre_grasp. Every cycle must start from
        # the same known-good pose, exactly as the first one does.
        self.move_joints(READY, "READY(after failure)")

    def _run_cycle_inner(self, index):
        if self.reset_each and not self.reset_episode():
            return False, "reset_failed"

        # DETECT -- read the target from the topic (never from the simulator).
        self.object_pose = None
        if not self.spin_until(lambda: self.object_pose is not None, 10.0, "object pose"):
            return False, "no_object_pose"
        op = self.object_pose.pose
        ox, oy, oz = op.position.x, op.position.y, op.position.z
        oq = (op.orientation.w, op.orientation.x, op.orientation.y, op.orientation.z)
        gyaw = wrap_grasp_yaw(yaw_of(oq))
        self.get_logger().info(
            f"[cycle {index}] DETECT object ({ox:.3f}, {oy:.3f}, {oz:.3f}) "
            f"yaw {math.degrees(yaw_of(oq)):+.1f} deg -> grasp yaw {math.degrees(gyaw):+.1f} deg")

        pp = self.place_pose.pose
        px, py, pz = pp.position.x, pp.position.y, pp.position.z

        if not self.gripper(self.grip_approach, "OPEN(pre)"):
            return False, "gripper_failed"
        if not self.il_call("start"):
            return False, "recorder_unavailable"

        xyz, q = self.tool0_pose_for_tcp(ox, oy, oz + self.approach_height, gyaw)
        if not self.move_pose(xyz, q, "PRE_GRASP"):
            return False, "plan_failed_pre_grasp"

        # Re-read the part before descending, and correct LATERALLY FIRST.
        # The DETECT reading is by now several seconds and one planned move old.
        # Descending diagonally onto a part that has shifted is what clips it, so
        # the correction is a separate move at the approach height and the descent
        # stays purely vertical.
        if self.object_pose is None:
            return False, "no_object_pose"
        cur = self.object_pose.pose.position
        drift = math.hypot(cur.x - ox, cur.y - oy)
        if drift > 0.05:
            # Too far to be a settling wobble: the yaw is probably wrong too, and a
            # demo that starts by chasing a knocked part is not one to imitate.
            self.get_logger().error(f"part moved {drift * 1000:.0f} mm since DETECT; aborting")
            return False, "part_disturbed"
        if drift > 0.002:
            self.get_logger().info(f"re-centring: part drifted {drift * 1000:.1f} mm since DETECT")
            ox, oy = cur.x, cur.y
            xyz, q = self.tool0_pose_for_tcp(ox, oy, oz + self.approach_height, gyaw)
            if not self.move_linear(xyz, q, "RECENTRE"):
                return False, "plan_failed_recentre"

        # GRASP: descend onto the part, TCP grasp_z_offset ABOVE its origin so the
        # fingertips clear the surface it is standing on.
        xyz, q = self.tool0_pose_for_tcp(ox, oy, oz + self.grasp_z_offset, gyaw)
        if not self.move_linear(xyz, q, "GRASP"):
            return False, "plan_failed_grasp"

        # CLOSE, in two stages (see grip_preclose). Isaac attaches the part
        # automatically (D5) -- do NOT call /scene/attach_object here: the
        # automatic path is what teleop and the real robot do, and calling the
        # service would make recorded demos differ from lived ones (HISTORY.md 16).
        if not self.gripper(self.grip_preclose, "CLOSE(pre)"):
            return False, "gripper_failed"
        # Wait for the attach EVENT, not a fixed 0.8 s. /scene/grasp_active tells
        # us exactly when it fired; the check below is unchanged, so a timeout
        # still fails the cycle rather than closing blind.
        self.wait_for(lambda: self.grasp_active, 0.8, "attach")
        if not self.grasp_active:
            self.get_logger().error(
                "grasp attach did not fire at the pre-close stop; closing further would "
                "eject the part")
            return False, "attach_failed"
        if not self.gripper(self.grip_closed, "CLOSE"):
            return False, "gripper_failed"

        xyz, q = self.tool0_pose_for_tcp(ox, oy, oz + self.lift_height, gyaw)
        if not self.move_linear(xyz, q, "LIFT"):
            return False, "plan_failed_lift"

        # Did the part actually come up? This is what GT is genuinely for.
        # Wait for the condition itself: a successful lift satisfies it almost
        # immediately, and only a genuine failure spends the full timeout.
        lift_z = oz + 0.5 * self.lift_height
        self.wait_for(
            lambda: self.object_pose is not None
            and self.object_pose.pose.position.z >= lift_z,
            self.settle_time, "lift check")
        if self.object_pose is None or \
                self.object_pose.pose.position.z < oz + 0.5 * self.lift_height:
            got = None if self.object_pose is None else self.object_pose.pose.position.z
            self.get_logger().error(
                f"grasp failed: part z={got} (expected > {oz + 0.5 * self.lift_height:.3f})")
            return False, "grasp_failed"

        xyz, q = self.tool0_pose_for_tcp(px, py, pz + self.approach_height, gyaw)
        if not self.move_pose(xyz, q, "TRANSFER"):
            return False, "plan_failed_transfer"

        # Still holding it? A part dropped during the carry lands somewhere on the
        # table, and if that somewhere is near enough to the marker the final check
        # calls it a success -- measured: part released mid-TRANSFER, ended 57 mm
        # out, passed a 60 mm tolerance. Checking mid-flight catches the fumble
        # regardless of where it happens to land.
        if self.object_pose is None or \
                self.object_pose.pose.position.z < oz + 0.5 * self.lift_height:
            got = None if self.object_pose is None else self.object_pose.pose.position.z
            self.get_logger().error(f"dropped during transfer: part z={got}")
            return False, "dropped_in_transfer"

        # PLACE: lower until the part's underside clears the marker. The part is
        # held at its centre, so descend to half its height plus clearance.
        half_h = max(0.0, oz - pz)
        xyz, q = self.tool0_pose_for_tcp(
            px, py, pz + half_h + self.grasp_z_offset + self.place_clearance, gyaw)
        if not self.move_linear(xyz, q, "PLACE"):
            return False, "plan_failed_place"

        # OPEN. Isaac's automatic rule releases the attach when the gripper opens.
        # Release only as far as grip_approach, NOT wide open. Fully opening is
        # what jams the descent, and it does the same damage on the way out:
        # measured, PLACE put the part within 1.2 mm of the marker and the
        # full-open release then shoved it 37 mm -- just past the 35 mm tolerance.
        # grasp_release must sit ABOVE this value or the attach never lets go.
        if not self.gripper(self.grip_approach, "OPEN"):
            return False, "gripper_failed"
        # Retract as soon as the part is released, not after a fixed delay.
        self.wait_for(lambda: not self.grasp_active, self.settle_time, "release")

        xyz, q = self.tool0_pose_for_tcp(px, py, pz + self.approach_height, gyaw)
        if not self.move_linear(xyz, q, "RETRACT"):
            return False, "plan_failed_retract"

        # NOTE: the READY return used to be here, INSIDE the recorded window. It is
        # now in run_cycle(), after the recorder stops. Going back to READY is
        # preparation for the next cycle, not part of "put the block on the marker",
        # and at ~4.5 s of motion plus its settle it was a sizeable tail of every
        # episode. The next cycle still starts from READY, so nothing the policy
        # sees at t=0 changes.

        # Judge from GT: is the part on the marker, and did we let go?
        # The part is dropped from ~20 mm and must come to rest before its pose
        # means anything -- but it usually does so well inside settle_time.
        self.wait_part_still(self.settle_time, "place judge")
        fp = self.object_pose.pose.position
        dist = math.hypot(fp.x - px, fp.y - py)
        if dist > self.place_tol:
            self.get_logger().error(
                f"place failed: part {dist:.3f} m from the marker (tol {self.place_tol})")
            return False, "place_failed"
        if self.grasp_active:
            self.get_logger().error("place failed: still holding the part")
            return False, "still_holding"
        self.get_logger().info(f"[cycle {index}] SUCCESS (part {dist:.3f} m from marker)")
        return True, "success"

    def run(self):
        if not self.wait_for_clock():
            return 1
        if not self.move.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("/move_action unavailable -- is a MoveIt launch running?")
            return 1
        if not self.grip.wait_for_server(timeout_sec=20.0):
            self.get_logger().error("/gripper_controller/gripper_cmd unavailable")
            return 1
        if not self.check_ee_link_known():
            return 1
        if not self.spin_until(lambda: self.place_pose is not None, 15.0,
                               "place pose (is Isaac running with --scene pick_place?)"):
            return 1
        if not self.recover_gripper():
            return 1
        # Open the gripper BEFORE measuring. The TCP is the midpoint of the finger
        # PADS, and the pads swing inward as the gripper closes -- measuring while
        # closed gave 0.1118 m against 0.0983 m open, a 13.5 mm error that lands the
        # grasp above the part. We always approach open, so measure open.
        if not self.gripper(self.grip_open, "OPEN(init)"):
            return 1
        self.sleep(1.0)
        if not self.measure_tcp_offset():
            return 1
        if not self.move_joints(READY, "READY(start)"):
            return 1
        # Only NOW put the table in the planning scene. Isaac starts the arm
        # stretched out at roughly work-surface height, so adding the slab first
        # makes the very first plan fail with START_STATE_IN_COLLISION (-10):
        # the robot is already inside the box it was just told about. From READY
        # the arm is folded up and clear.
        self.add_table_to_scene()

        results = []
        for i in range(1, self.cycles + 1):
            ok, reason = self.run_cycle(i)
            results.append((ok, reason))
            if not ok:
                self.get_logger().error(f"[cycle {i}] FAILED: {reason}")

        n_ok = sum(1 for ok, _ in results if ok)
        self.get_logger().info(f"=== {n_ok}/{len(results)} cycles succeeded ===")
        for i, (ok, reason) in enumerate(results, 1):
            self.get_logger().info(f"  cycle {i}: {'SUCCESS' if ok else reason}")
        return 0 if n_ok == len(results) else 1


def main():
    rclpy.init()
    node = PickPlace()
    rc = 1
    try:
        rc = node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    sys.exit(rc)


if __name__ == "__main__":
    main()
