#!/usr/bin/env python3
"""OMY-L100 leader arm -> UR16e follower, joint-direct (no IK).

    /leader/joint_states  ->  this node  ->  /forward_position_controller/commands
                                         \\->  /gripper_controller/gripper_cmd

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

USAGE
-----
    # leader (real: drop use_mock_hardware, add port_name:=/dev/ttyUSB0)
    ros2 launch open_manipulator_bringup omy_l100_leader_ai.launch.py \\
        use_mock_hardware:=true use_self_collision_avoidance:=false
    # follower must be in STREAMING mode -- trajectory and streaming are exclusive
    python3 isaac/common/switch_control_mode.py streaming
    ros2 run ur_bringup omy_to_ur16e.py
    ros2 service call /omy_bridge/enable  std_srvs/srv/Trigger    # refuses if far
    ros2 service call /omy_bridge/disable std_srvs/srv/Trigger

Recording is unchanged: il_recorder.py --action-source topic --action-topic
/leader/joint_states already handles this (plan_il_vla.md 2.5).
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy

from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
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

        # --- gripper: leader trigger [rad] -> 2F-85 finger_joint [rad] ---------
        # L100 J7 spec range is -90..+60 deg; the useful squeeze band is small, so
        # both ends are parameters. finger_joint: 0.0 open .. 0.8 closed.
        p("gripper_enable", True)
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

        # Sensor-data QoS: joint_state_broadcaster publishes best-effort.
        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/leader/joint_states", self._on_leader, qos)
        self.create_subscription(JointState, "/joint_states", self._on_follower, qos)
        self.pub = self.create_publisher(Float64MultiArray, "/forward_position_controller/commands", 10)
        self.status_pub = self.create_publisher(String, "/omy_bridge/status", 10)
        self.grip_cli = ActionClient(self, GripperCommand, "/gripper_controller/gripper_cmd")

        self.create_service(Trigger, "/omy_bridge/enable", self._enable)
        self.create_service(Trigger, "/omy_bridge/disable", self._disable)

        rate = float(g("publish_rate"))
        self.dt = 1.0 / rate
        self.create_timer(self.dt, self._tick)

        self.get_logger().info(
            f"omy_to_ur16e up (DISABLED). sign={self.sign} offset_deg="
            f"{[round(math.degrees(o), 1) for o in self.offset]} "
            f"margin={self.margin} max_speed={self.max_speed} rad/s. "
            "Call /omy_bridge/enable to engage."
        )

    # ------------------------------------------------------------------ inputs
    def _map(self, q):
        return [self.sign[i] * q[i] + self.offset[i] for i in range(6)]

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

    # ---------------------------------------------------------------- services
    def _enable(self, _req, res):
        """Refuse to engage unless the leader is already where the UR16e is.

        Skipping this check is how you get a full-speed slam the instant the
        bridge turns on: the follower would traverse the whole difference in one
        control period. The operator's job is to physically move the leader until
        it matches, then enable -- the error list below tells them which way.
        """
        if self.leader_q is None:
            res.success, res.message = False, "no /leader/joint_states yet"
            return res
        if self.ur_q is None:
            res.success, res.message = False, "no /joint_states from the UR16e yet"
            return res
        err = [self.leader_q[i] - self.ur_q[i] for i in range(6)]
        bad = [(UR_JOINTS[i], round(math.degrees(e), 1))
               for i, e in enumerate(err) if abs(e) > self.engage_tol]
        if bad:
            res.success = False
            res.message = ("REFUSED -- move the leader to match the robot first. "
                           f"Off by (deg): {bad}. Tolerance "
                           f"{math.degrees(self.engage_tol):.1f} deg/joint.")
            self.get_logger().warn(res.message)
            return res
        # Start the slew from where the robot IS, not from a stale command.
        self.cmd = list(self.ur_q)
        self.enabled = True
        res.success, res.message = True, "engaged"
        self.get_logger().info("ENGAGED -- leader is now driving the UR16e")
        return res

    def _disable(self, _req, res):
        self.enabled = False
        res.success, res.message = True, "disabled"
        self.get_logger().info("disabled -- holding")
        return res

    # -------------------------------------------------------------------- loop
    def _tick(self):
        if not self.enabled:
            self._status("disabled")
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
