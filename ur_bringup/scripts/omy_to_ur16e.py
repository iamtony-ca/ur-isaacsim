#!/usr/bin/env python3
"""OMY-L100 leader arm -> UR16e follower, joint-direct (no IK).

    /leader/joint_states  ->  this node  ->  /forward_position_controller/commands
                                         \\->  /gripper_controller/gripper_cmd
    /joy (optional)       ->  enable / disable / sync buttons

Step 1b of the teleop -> IL pick&place pipeline (ur_bringup/docs/plan_il_vla.md 3.5).
The DualSense path (teleop_joy.py) drives Cartesian twists through MoveIt Servo;
this one bypasses Servo entirely because the leader IS a 6R arm with the SAME axis
sequence as the UR16e, so leader joint i maps straight onto follower joint i.

WHY NO IK
---------
Measured from both robots' own URDFs (HISTORY.md 21/22): joint axes expressed in
the base frame are `+Z +Y +Y +Y +Z +Y` on BOTH arms -- yaw-pitch-pitch-pitch-yaw-pitch
with an offset wrist. That is the GELLO premise, so a per-joint affine map suffices:

    q_ur[i] = sign[i] * q_leader[i] + offset[i]

*** THE TWO NON-OBVIOUS TERMS -- do not "simplify" these away ***
  sign[4] = -1   J5: leader joint5 spins about +Z, UR wrist_2_joint about -Z.
  offset[1] = -pi/2
                 J2: the two arms have different ZERO POSES. At q=0 the L100 points
                 straight UP; the UR16e points HORIZONTALLY FORWARD. -90 deg on
                 shoulder_lift reconciles them.

*** AND THE LIMIT OF THIS MODEL ***
The L100 is the leader for the OMY-F3M, NOT a scaled UR16e. Its lateral (wrist)
offset accumulates to -46 mm where the UR16e's is +290.7 mm -- different magnitude
AND different sign. So J4/J6 have no *derivable* offset; they are tuning knobs for
operator comfort, which is why every sign/offset below is a ROS parameter. Retune
them on hardware, do not hard-code new ones here.
This mismatch does NOT affect the IL data: what gets recorded is the UR16e's own
state/action (plan_il_vla.md 2.6). The leader is an input device, nothing more.

STARTING FROM DIFFERENT POSES -- THE RENDEZVOUS (HISTORY.md 41)
---------------------------------------------------------------
If the UR16e was doing something else, the two arms start far apart and the engage
gate (below) refuses -- correctly, but unhelpfully, because matching six joints by
eye is not practical. The fix is NOT to chase each other's arbitrary pose but to
meet at a defined one:

    leader  [0, 0, +90, -90, +90, 0] deg   <- ROBOTIS OMY SRDF `home`; the pose the
                                              L100 holds when you let go of it
      maps to
    UR16e   [0, -90, +90, -90, -90, 0] deg <- reset_pose.py `ready`

That is not a coincidence: both were chosen as "elbow bent 90, wrist tidy" -- ours to
avoid the elbow/wrist singularities, ROBOTIS' so the arm stands up on its own. So the
operator PUTS THE LEADER DOWN and calls `/omy_bridge/sync`, which drives the UR16e to
`ready` THROUGH MOVEIT (collision-checked -- the arm may be sitting next to a fixture
from its previous job, and reset_pose.py's straight joint interpolation is not safe
there). Then `/omy_bridge/enable`.

`rendezvous` is a parameter. If your leader's rest pose measures differently
(CHECKLIST.md E-1 (2)), set it from the measured value -- the number that matters is
where the LEADER naturally sits, mapped through sign/offset.

SAFETY (this is a 16 kg-payload, 900 mm arm driven by a 1.46 kg toy)
--------------------------------------------------------------------
  1. Disabled at startup. Motion needs an explicit `/omy_bridge/enable`.
  2. ENGAGE GATE: enable is REFUSED unless the mapped leader pose is already close
     to where the UR16e actually is (`engage_tol`, per joint). Without this the arm
     snaps from its current pose to the leader's in one control cycle. This is the
     single most important line of defence here -- see `_enable`.
  3. Per-joint clamp to `limit_margin` x the UR16e joint limits.
  4. Per-joint velocity limit (`max_joint_speed`), applied as a slew on the
     published command, so a leader glitch cannot become a full-speed slam.
  5. Watchdog: if the leader stops publishing for `leader_timeout`, disable.
  6. enable is also refused if the STREAMING controller is not active -- otherwise
     the bridge publishes happily into a controller nobody is listening to and the
     arm just sits there, which reads as "the bridge is broken".
  7. Pad buttons that START motion (enable, sync) require the deadman held; the one
     that STOPS motion (disable) never does. A stray /joy message must not be able
     to move the arm.

USAGE
-----
    # leader (real: drop use_mock_hardware, add port_name:=/dev/ttyUSB0)
    ros2 launch open_manipulator_bringup omy_l100_leader_ai.launch.py \\
        use_mock_hardware:=true use_self_collision_avoidance:=false
    ros2 run ur_bringup omy_to_ur16e.py

    # put the leader down in its rest pose, then:
    ros2 service call /omy_bridge/sync   std_srvs/srv/Trigger   # UR16e -> rendezvous
    ros2 service call /omy_bridge/enable std_srvs/srv/Trigger   # refuses if far
    ros2 service call /omy_bridge/disable std_srvs/srv/Trigger

    # how far off am I?  (rad, leader-mapped minus follower, per joint)
    ros2 topic echo /omy_bridge/engage_error

`sync` needs move_group running (ur16e_moveit.launch.py). Without it, do the two
steps by hand -- `switch_control_mode.py trajectory` + `reset_pose.py ready` +
`switch_control_mode.py streaming` -- after checking the arm's surroundings by eye.

Recording is unchanged: il_recorder.py --action-source topic --action-topic
/leader/joint_states already handles this (plan_il_vla.md 2.5).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from control_msgs.action import GripperCommand
from controller_manager_msgs.srv import ListControllers, SwitchController
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             PlanningOptions)
from rclpy.action import ActionClient
from sensor_msgs.msg import Joy, JointState
from std_msgs.msg import Float64MultiArray, String
from std_srvs.srv import Trigger

# Order is the contract for /forward_position_controller/commands -- it must match
# `joints:` in config/common/ur16e_2f85_controllers.yaml exactly.
UR_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
LEADER_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
LEADER_GRIPPER = "rh_r1_joint"

# UR16e joint limits [rad], read off the expanded ur16e_sim.urdf.xacro.
# NOTE the elbow: +-pi, HALF of what the other five allow. The OMY-L100's J3 is
# +-pi too, so the leader can drive the UR elbow exactly to its limit with zero
# margin -- that is precisely why `limit_margin` exists and defaults below 1.0.
UR_LIMITS = [2 * math.pi, 2 * math.pi, math.pi, 2 * math.pi, 2 * math.pi, 2 * math.pi]

# reset_pose.py's `ready`. Kept in step with that file ON PURPOSE: teleop must start
# where the recorded demonstrations start, or a policy trained on them sees an
# initial state it never saw in training (HISTORY.md 36).
READY = [0.0, -math.pi / 2, math.pi / 2, -math.pi / 2, -math.pi / 2, 0.0]


class OmyToUr16e(Node):
    def __init__(self):
        super().__init__("omy_to_ur16e")
        p = self.declare_parameter

        # --- the affine map (see module docstring before changing any of these) --
        p("sign", [1.0, 1.0, 1.0, 1.0, -1.0, 1.0])
        p("offset", [0.0, -math.pi / 2, 0.0, 0.0, 0.0, 0.0])

        # --- safety -----------------------------------------------------------
        p("limit_margin", 0.95)        # fraction of the UR16e joint limits
        p("max_joint_speed", 1.0)      # [rad/s] per joint, slew on the command
        p("engage_tol", 0.15)          # [rad] per joint, checked on enable
        p("leader_timeout", 0.5)       # [s] without leader data -> disable
        p("publish_rate", 100.0)       # [Hz] to the follower

        # --- rendezvous / sync ------------------------------------------------
        p("rendezvous", READY)         # UR16e joint target for /omy_bridge/sync
        p("move_group", "ur_manipulator")
        p("sync_vel_scale", 0.15)      # deliberately slow: nobody is watching a
        p("sync_accel_scale", 0.15)    # progress bar, they are holding the leader
        p("sync_timeout", 90.0)        # [s] whole sequence, incl. both switches
        p("controller_manager", "/controller_manager")
        p("traj_controller", "scaled_joint_trajectory_controller")
        p("stream_controller", "forward_position_controller")
        p("require_stream_controller", True)

        # --- pad buttons (DualSense via hid-playstation) ----------------------
        # Indices 0..5 are taken by teleop_joy.py (Cross/Circle/Triangle/Square/L1/R1)
        # so these defaults do not collide if both nodes ever run on one pad.
        p("joy_enable", True)
        p("button_deadman", 4)         # L1 -- HELD for enable/sync, not for disable
        p("button_bridge_enable", 9)   # Options
        p("button_bridge_disable", 8)  # Create/Share
        p("button_bridge_sync", 12)    # R3 (right stick click) -- deliberate press

        p("gripper_enable", True)
        # --- gripper: leader trigger [rad] -> 2F-85 finger_joint [rad] ---------
        # L100 J7 spec range is -90..+60 deg; the useful squeeze band is small, so
        # both ends are parameters. finger_joint: 0.0 open .. 0.8 closed.
        p("gripper_in_open", 0.0)
        p("gripper_in_closed", -1.0)
        p("gripper_open", 0.0)
        p("gripper_closed", 0.8)
        p("gripper_max_effort", 100.0)
        p("gripper_deadband", 0.02)

        g = lambda n: self.get_parameter(n).value           # noqa: E731
        self.sign = [float(v) for v in g("sign")]
        self.offset = [float(v) for v in g("offset")]
        if len(self.sign) != 6 or len(self.offset) != 6:
            raise ValueError("sign and offset must each have 6 entries")
        self.margin = float(g("limit_margin"))
        self.max_speed = float(g("max_joint_speed"))
        self.engage_tol = float(g("engage_tol"))
        self.leader_timeout = float(g("leader_timeout"))
        self.rendezvous = [float(v) for v in g("rendezvous")]
        if len(self.rendezvous) != 6:
            raise ValueError("rendezvous must have 6 entries")
        self.group = g("move_group")
        self.sync_vel = float(g("sync_vel_scale"))
        self.sync_acc = float(g("sync_accel_scale"))
        self.sync_timeout = float(g("sync_timeout"))
        self.cm = str(g("controller_manager")).rstrip("/")
        self.traj_ctrl = g("traj_controller")
        self.stream_ctrl = g("stream_controller")
        self.require_stream = bool(g("require_stream_controller"))
        self.joy_on = bool(g("joy_enable"))
        self.btn_deadman = int(g("button_deadman"))
        self.btn_en = int(g("button_bridge_enable"))
        self.btn_dis = int(g("button_bridge_disable"))
        self.btn_sync = int(g("button_bridge_sync"))
        self.grip_on = bool(g("gripper_enable"))
        self.gi_open, self.gi_closed = float(g("gripper_in_open")), float(g("gripper_in_closed"))
        self.go_open, self.go_closed = float(g("gripper_open")), float(g("gripper_closed"))
        self.grip_effort = float(g("gripper_max_effort"))
        self.grip_deadband = float(g("gripper_deadband"))

        self.enabled = False
        self.leader_q = None            # latest mapped target, pre-clamp
        self.leader_grip = None
        self.leader_stamp = None
        self.ur_q = None                # follower's actual position
        self.cmd = None                 # last published command (slew origin)
        self.last_grip = None
        self.ctrl_active = {}           # controller name -> bool; empty = not polled yet
        self.sync_state = "idle"        # idle|to_traj|moving|to_stream|done|failed
        self.sync_deadline = None
        self._prev_btn = {}

        # Sensor-data QoS: joint_state_broadcaster publishes best-effort.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/leader/joint_states", self._on_leader, qos)
        self.create_subscription(JointState, "/joint_states", self._on_follower, qos)
        self.pub = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
        self.status_pub = self.create_publisher(String, "/omy_bridge/status", 10)
        # Why a separate topic instead of putting numbers in /omy_bridge/status:
        # the status string is a state name that other things match on. The operator
        # needs "how far off, per joint" WITHOUT having to call enable and read the
        # rejection -- otherwise matching the leader is call-refuse-adjust-repeat.
        self.err_pub = self.create_publisher(Float64MultiArray, "/omy_bridge/engage_error", 10)
        self.grip_cli = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")
        self.move_cli = ActionClient(self, MoveGroup, "/move_action")
        self.switch_cli = self.create_client(SwitchController, f"{self.cm}/switch_controller")
        self.list_cli = self.create_client(ListControllers, f"{self.cm}/list_controllers")

        self.create_service(Trigger, "/omy_bridge/enable", self._enable)
        self.create_service(Trigger, "/omy_bridge/disable", self._disable)
        self.create_service(Trigger, "/omy_bridge/sync", self._sync)

        if self.joy_on:
            self.create_subscription(Joy, "/joy", self._on_joy, 10)

        rate = float(g("publish_rate"))
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._tick)
        self.create_timer(0.2, self._publish_error)      # 5 Hz is plenty for a human
        self.create_timer(1.0, self._poll_controllers)

        self.get_logger().info(
            f"omy_to_ur16e up (DISABLED). sign={self.sign} offset_deg="
            f"{[round(math.degrees(o), 1) for o in self.offset]} "
            f"margin={self.margin} max_speed={self.max_speed} rad/s. "
            "Call /omy_bridge/enable to engage."
        )
        self.get_logger().info(
            "rendezvous: put the LEADER at "
            f"{[round(math.degrees(v), 1) for v in self._unmap(self.rendezvous)]} deg, "
            f"then /omy_bridge/sync moves the UR16e to "
            f"{[round(math.degrees(v), 1) for v in self.rendezvous]} deg."
        )
        if self.joy_on:
            self.get_logger().info(
                f"pad: hold button[{self.btn_deadman}] + button[{self.btn_en}]=enable / "
                f"button[{self.btn_sync}]=sync;  button[{self.btn_dis}]=disable (no deadman)."
            )

    # ------------------------------------------------------------------ inputs
    def _map(self, q):
        return [self.sign[i] * q[i] + self.offset[i] for i in range(6)]

    def _unmap(self, q_ur):
        """Follower joints -> the leader pose that maps onto them. Tells the
        operator where to physically put the L100."""
        return [(q_ur[i] - self.offset[i]) / self.sign[i] for i in range(6)]

    def _on_leader(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(n in idx for n in LEADER_JOINTS):
            return
        self.leader_q = self._map([msg.position[idx[n]] for n in LEADER_JOINTS])
        if LEADER_GRIPPER in idx:
            self.leader_grip = msg.position[idx[LEADER_GRIPPER]]
        self.leader_stamp = self.get_clock().now()

    def _on_follower(self, msg):
        idx = {n: i for i, n in enumerate(msg.name)}
        if all(n in idx for n in UR_JOINTS):
            self.ur_q = [msg.position[idx[n]] for n in UR_JOINTS]

    def _on_joy(self, msg):
        """Edge-triggered pad control. The operator's hands are on the leader, so
        reaching for a terminal to call a service is not realistic."""
        def edge(idx):
            cur = msg.buttons[idx] if 0 <= idx < len(msg.buttons) else 0
            prev = self._prev_btn.get(idx, 0)
            self._prev_btn[idx] = cur
            return cur and not prev

        deadman = (msg.buttons[self.btn_deadman]
                   if 0 <= self.btn_deadman < len(msg.buttons) else 0)
        # Order matters: read every edge before acting, so a button press is never
        # swallowed by an early return.
        want_en, want_sync, want_dis = edge(self.btn_en), edge(self.btn_sync), edge(self.btn_dis)

        if want_dis:                                   # stopping never needs a deadman
            self._disable(None, Trigger.Response())
        if (want_en or want_sync) and not deadman:
            self.get_logger().warn(
                f"pad: hold the deadman (button[{self.btn_deadman}]) to enable or sync")
            return
        if want_en:
            res = self._enable(None, Trigger.Response())
            self.get_logger().info(f"pad enable: {res.message}")
        if want_sync:
            res = self._sync(None, Trigger.Response())
            self.get_logger().info(f"pad sync: {res.message}")

    # ---------------------------------------------------------------- services
    def _engage_error(self):
        """Per-joint (mapped leader - follower). None if either side is missing."""
        if self.leader_q is None or self.ur_q is None:
            return None
        return [self.leader_q[i] - self.ur_q[i] for i in range(6)]

    def _enable(self, _req, res):
        """Refuse to engage unless the leader is already where the UR16e is.

        Skipping this check is how you get a full-speed slam the instant the
        bridge turns on: the follower would traverse the whole difference in one
        control period. The operator's job is to physically move the leader until
        it matches, then enable -- the error list below tells them which way.
        """
        if self.sync_state in ("to_traj", "moving", "to_stream"):
            res.success, res.message = False, f"sync in progress ({self.sync_state})"
            return res
        if self.leader_q is None:
            res.success, res.message = False, "no /leader/joint_states yet"
            return res
        if self.ur_q is None:
            res.success, res.message = False, "no /joint_states from the UR16e yet"
            return res
        # Engaging into an inactive streaming controller looks exactly like a broken
        # bridge: commands publish, nothing moves. Say so instead.
        if self.require_stream and self.ctrl_active.get(self.stream_ctrl) is False:
            res.success, res.message = False, (
                f"'{self.stream_ctrl}' is not active -- commands would go nowhere. "
                f"Run switch_control_mode.py streaming (or /omy_bridge/sync, which "
                f"leaves the arm in streaming mode).")
            self.get_logger().warn(res.message)
            return res
        err = self._engage_error()
        bad = [(UR_JOINTS[i], round(math.degrees(e), 1))
               for i, e in enumerate(err) if abs(e) > self.engage_tol]
        if bad:
            res.success = False
            res.message = ("REFUSED -- move the leader to match the robot first. "
                           f"Off by (deg): {bad}. Tolerance "
                           f"{math.degrees(self.engage_tol):.1f} deg/joint. "
                           "If the robot is far from where you want to start, "
                           "put the leader down and call /omy_bridge/sync.")
            self.get_logger().warn(res.message)
            return res
        # Start the slew from where the robot IS, not from a stale command.
        self.cmd = list(self.ur_q)
        self.enabled = True
        # "synced" means "a sync finished and you have not engaged since". Once the
        # operator engages, the arm is wherever the leader took it, so reporting
        # "synced" after the next disable would be a lie.
        self.sync_state = "idle"
        res.success, res.message = True, "engaged"
        self.get_logger().info("ENGAGED -- leader is now driving the UR16e")
        return res

    def _disable(self, _req, res):
        self.enabled = False
        res.success, res.message = True, "disabled"
        self.get_logger().info("disabled -- holding")
        return res

    # -------------------------------------------------------------------- sync
    def _sync(self, _req, res):
        """Drive the UR16e to the rendezvous pose through MoveIt.

        Returns as soon as the sequence is ACCEPTED, not when it finishes: this
        service is meant to be called from a pad button, and blocking a service
        callback for ~10 s of arm motion would stall the 100 Hz control timer with
        the single-threaded executor. Progress is on /omy_bridge/status.

        Why MoveIt and not reset_pose.py's straight joint interpolation: the arm may
        be next to a fixture from whatever it was doing before teleop, and a straight
        line in joint space is not a safe path through a cluttered cell.
        """
        if self.enabled:
            res.success, res.message = False, "disable the bridge first (arm is engaged)"
            return res
        if self.sync_state in ("to_traj", "moving", "to_stream"):
            res.success, res.message = False, f"already syncing ({self.sync_state})"
            return res
        if self.ur_q is None:
            res.success, res.message = False, "no /joint_states from the UR16e yet"
            return res
        if not self.move_cli.server_is_ready():
            res.success, res.message = False, (
                "/move_action not available -- start ur16e_moveit.launch.py, or do it "
                "by hand: switch_control_mode.py trajectory + reset_pose.py ready + "
                "switch_control_mode.py streaming (check the arm's surroundings first)")
            return res
        if not self.switch_cli.service_is_ready():
            res.success, res.message = False, f"{self.cm}/switch_controller not available"
            return res

        self.sync_deadline = self.get_clock().now().nanoseconds * 1e-9 + self.sync_timeout
        self._set_sync("to_traj")
        self._switch(self.traj_ctrl, self.stream_ctrl, self._sync_send_goal)
        res.success, res.message = True, (
            "sync started -- MoveIt is planning to the rendezvous pose. "
            "Watch /omy_bridge/status; call /omy_bridge/enable when it says 'synced'.")
        self.get_logger().info(res.message)
        return res

    def _set_sync(self, state, why=""):
        self.sync_state = state
        if state == "failed":
            self.get_logger().error(f"sync FAILED: {why}")
        elif state == "done":
            self.get_logger().info(
                "sync done -- UR16e at the rendezvous pose, streaming controller active. "
                "Put the leader in its rest pose and call /omy_bridge/enable.")
        else:
            self.get_logger().info(f"sync: {state}")

    def _switch(self, activate, deactivate, then):
        """Atomic controller swap. This mirrors switch_control_mode.py -- that stays
        the CLI entry point; this exists so a pad button can do it without the
        operator letting go of the leader. Keep the two in step.

        Only ask for what is actually needed: a STRICT switch is REJECTED if it names
        a controller to deactivate that is not running, or activates one that already
        is. Which of the two is active depends on what the operator was doing before
        sync, so this cannot be hard-coded."""
        active = self.ctrl_active
        if active.get(activate) is True and not active.get(deactivate, False):
            then()                                   # already in the target mode
            return
        req = SwitchController.Request()
        req.activate_controllers = [] if active.get(activate) else [activate]
        req.deactivate_controllers = [deactivate] if active.get(deactivate) else []
        if not req.activate_controllers and not req.deactivate_controllers:
            then()
            return
        # STRICT: fail loudly rather than half-switch and leave the arm uncommanded.
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout.sec = 5
        fut = self.switch_cli.call_async(req)

        def done(f):
            r = f.result()
            if r is None or not r.ok:
                self._set_sync("failed", f"controller switch to '{activate}' rejected")
                return
            # The poll is 1 Hz; the next step runs immediately. Reflect the switch now
            # so a second _switch in the same second does not act on a stale view.
            self.ctrl_active[activate] = True
            self.ctrl_active[deactivate] = False
            then()
        fut.add_done_callback(done)

    def _sync_send_goal(self):
        self._set_sync("moving")
        req = MotionPlanRequest()
        req.group_name = self.group
        req.num_planning_attempts = 10
        req.allowed_planning_time = 10.0
        req.max_velocity_scaling_factor = self.sync_vel
        req.max_acceleration_scaling_factor = self.sync_acc
        constraints = Constraints()
        for name, pos in zip(UR_JOINTS, self.rendezvous):
            jc = JointConstraint()
            jc.joint_name = name
            jc.position = float(pos)
            jc.tolerance_above = 0.01
            jc.tolerance_below = 0.01
            jc.weight = 1.0
            constraints.joint_constraints.append(jc)
        req.goal_constraints.append(constraints)

        goal = MoveGroup.Goal()
        goal.request = req
        opts = PlanningOptions()
        opts.plan_only = False               # plan AND execute
        opts.planning_scene_diff.is_diff = True
        opts.planning_scene_diff.robot_state.is_diff = True
        goal.planning_options = opts

        fut = self.move_cli.send_goal_async(goal)

        def accepted(f):
            handle = f.result()
            if handle is None or not handle.accepted:
                self._set_sync("failed", "move_group rejected the goal")
                return
            handle.get_result_async().add_done_callback(self._sync_moved)
        fut.add_done_callback(accepted)

    def _sync_moved(self, f):
        result = f.result()
        code = result.result.error_code.val if result is not None else None
        if code != 1:                        # moveit_msgs/MoveItErrorCodes.SUCCESS
            self._set_sync("failed", f"move_group error_code={code} (1 = SUCCESS)")
            return
        # Hand the arm back to streaming, else enable would engage into a controller
        # that is not running (see safety note 6).
        self._set_sync("to_stream")
        self._switch(self.stream_ctrl, self.traj_ctrl, lambda: self._set_sync("done"))

    # -------------------------------------------------------------------- polls
    def _poll_controllers(self):
        """Cache which arm controllers are active. Polled rather than queried inside
        _enable/_switch because a blocking service call from a service callback
        deadlocks the single-threaded executor."""
        if not self.list_cli.service_is_ready():
            return
        fut = self.list_cli.call_async(ListControllers.Request())

        def done(f):
            r = f.result()
            if r is None:
                return
            # Only controllers that are LOADED appear. An absent name stays absent so
            # `.get(name)` is None ("unknown"), which the guards treat as fail-open --
            # a missing controller_manager must not block teleop that would work.
            states = {c.name: c.state for c in r.controller}
            for name in (self.stream_ctrl, self.traj_ctrl):
                if name in states:
                    self.ctrl_active[name] = states[name] == "active"
        fut.add_done_callback(done)

    def _publish_error(self):
        err = self._engage_error()
        if err is not None:
            self.err_pub.publish(Float64MultiArray(data=err))

    # -------------------------------------------------------------------- loop
    def _tick(self):
        if self.sync_state in ("to_traj", "moving", "to_stream"):
            now = self.get_clock().now().nanoseconds * 1e-9
            if self.sync_deadline is not None and now > self.sync_deadline:
                self._set_sync("failed", f"timed out after {self.sync_timeout:.0f} s")
            self._status(f"sync:{self.sync_state}")
            return
        if not self.enabled:
            self._status("synced" if self.sync_state == "done" else
                         "sync_failed" if self.sync_state == "failed" else "disabled")
            return
        if self.leader_stamp is None or \
                (self.get_clock().now() - self.leader_stamp).nanoseconds * 1e-9 > self.leader_timeout:
            self.enabled = False
            self.get_logger().error("leader data stale -- DISABLED (watchdog)")
            self._status("watchdog")
            return

        step = self.max_speed * self.dt
        out = []
        for i in range(6):
            lim = UR_LIMITS[i] * self.margin
            tgt = max(-lim, min(lim, self.leader_q[i]))          # clamp
            cur = self.cmd[i]
            out.append(cur + max(-step, min(step, tgt - cur)))   # slew
        self.cmd = out
        self.pub.publish(Float64MultiArray(data=out))
        self._gripper()
        self._status("engaged")

    def _gripper(self):
        if not (self.grip_on and self.leader_grip is not None):
            return
        span = self.gi_closed - self.gi_open
        if abs(span) < 1e-6:
            return
        f = max(0.0, min(1.0, (self.leader_grip - self.gi_open) / span))
        target = self.go_open + f * (self.go_closed - self.go_open)
        if self.last_grip is not None and abs(target - self.last_grip) < self.grip_deadband:
            return
        if not self.grip_cli.server_is_ready():
            self.get_logger().warn("gripper action server not ready", throttle_duration_sec=5.0)
            return
        goal = GripperCommand.Goal()
        goal.command.position = float(target)
        goal.command.max_effort = float(self.grip_effort)
        self.grip_cli.send_goal_async(goal)
        self.last_grip = target

    def _status(self, state):
        self.status_pub.publish(String(data=state))


def main():
    rclpy.init()
    node = OmyToUr16e()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
