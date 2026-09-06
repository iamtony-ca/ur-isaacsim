#!/usr/bin/env python3
"""Fake OMY-L100 leader: publishes /leader/joint_states without any hardware.

Exists because the real leader stack's MOCK mode cannot move. `mock_components/
GenericSystem` mirrors *matching* command/state interfaces, and the L100 is
commanded in **effort** while position is a state-only interface -- so a mocked
L100 reports position 0.0 forever. Useful to prove the controllers load
(HISTORY.md 22), useless for driving the follower.

So: this node synthesises the leader signal instead, and the rest of the chain
(omy_to_ur16e -> forward_position_controller -> ros2_control -> Isaac) is the
REAL one. That makes the sim test cover everything except the Dynamixel read.

*** THE IMPORTANT PART: it starts matched to the follower. ***
On startup it reads the UR16e's actual /joint_states and INVERSE-maps it into
leader coordinates:

    q_leader[i] = (q_ur[i] - offset[i]) / sign[i]

so the bridge's engage gate passes immediately instead of refusing (which is what
you get if you just publish zeros -- see HISTORY.md 22). Motion then ramps in from
that pose, so nothing steps.

    sign/offset MUST match omy_to_ur16e.py. They are separate parameters on
    purpose (this node is a test fixture, not a source of truth), so if you retune
    the bridge on real hardware, retune here too or the engage gate will refuse.

Usage
-----
    ros2 run ur_bringup virtual_omy_leader.py                      # gentle sine
    ros2 run ur_bringup virtual_omy_leader.py --ros-args -p mode:=hold
    ros2 run ur_bringup virtual_omy_leader.py --ros-args \\
        -p amplitude:=0.35 -p period:=6.0 -p move_joints:="[0,1,2]"

Amplitude x 2pi/period is the peak leader speed; keep it below the bridge's
`max_joint_speed` or you are only testing the slew limiter.
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import String

UR_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
LEADER_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
LEADER_GRIPPER = "rh_r1_joint"


class VirtualLeader(Node):
    def __init__(self):
        super().__init__("virtual_omy_leader")
        p = self.declare_parameter
        p("sign", [1.0, 1.0, 1.0, 1.0, -1.0, 1.0])
        p("offset", [0.0, -math.pi / 2, 0.0, 0.0, 0.0, 0.0])
        p("rate", 300.0)             # the real leader runs at 300 Hz
        p("mode", "sine")            # sine | hold
        p("amplitude", 0.25)         # [rad] peak deviation
        p("period", 8.0)             # [s]
        p("move_joints", [0, 2])     # leader joint indices to animate
        p("ramp", 3.0)               # [s] to fade the motion in from zero
        # Hold q0 until the bridge reports "engaged". Without this the fixture
        # drifts away from the follower while you are still typing the enable
        # call, and the engage gate (correctly) refuses -- observed on the first
        # sim run: elbow had already moved 14.3 deg. It also mirrors what a human
        # must do with the real L100: hold it matched, THEN engage.
        p("wait_for_engage", True)
        p("gripper_cycle", True)     # also animate the trigger
        p("gripper_open", 0.0)
        p("gripper_closed", -1.0)

        g = lambda n: self.get_parameter(n).value          # noqa: E731
        self.sign = [float(v) for v in g("sign")]
        self.offset = [float(v) for v in g("offset")]
        self.mode = str(g("mode"))
        self.amp = float(g("amplitude"))
        self.period = float(g("period"))
        self.move = [int(v) for v in g("move_joints")]
        self.ramp = float(g("ramp"))
        self.wait_engage = bool(g("wait_for_engage"))
        self.grip_cycle = bool(g("gripper_cycle"))
        self.g_open, self.g_closed = float(g("gripper_open")), float(g("gripper_closed"))

        self.q0 = None               # leader-space home, derived from the follower
        self.t = 0.0
        self.engaged = not self.wait_engage
        self._said_go = False

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/joint_states", self._on_follower, qos)
        self.create_subscription(String, "/omy_bridge/status", self._on_status, 10)
        self.pub = self.create_publisher(JointState, "/leader/joint_states", qos)
        self.dt = 1.0 / float(g("rate"))
        self.create_timer(self.dt, self._tick)
        self.get_logger().info("virtual leader waiting for the UR16e's /joint_states ...")

    def _on_follower(self, msg):
        if self.q0 is not None:
            return                                    # latch once; never re-home
        idx = {n: i for i, n in enumerate(msg.name)}
        if not all(n in idx for n in UR_JOINTS):
            return
        ur = [msg.position[idx[n]] for n in UR_JOINTS]
        self.q0 = [(ur[i] - self.offset[i]) / self.sign[i] for i in range(6)]
        self.get_logger().info(
            "homed to the follower. leader q0 (deg) = "
            f"{[round(math.degrees(v), 1) for v in self.q0]} -- "
            "engage should be accepted now"
        )

    def _on_status(self, msg):
        if not self.wait_engage:
            return
        self.engaged = (msg.data == "engaged")
        if self.engaged and not self._said_go:
            self._said_go = True
            self.get_logger().info("bridge engaged -- starting leader motion")

    def _tick(self):
        if self.q0 is None:
            return
        if self.engaged:
            self.t += self.dt
        q = list(self.q0)
        if self.mode == "sine":
            # Ramp keeps the first published sample exactly at q0, so the bridge's
            # engage gate sees a matched pose no matter when the operator enables.
            k = min(1.0, self.t / self.ramp) if self.ramp > 0 else 1.0
            w = 2 * math.pi / self.period
            for n, j in enumerate(self.move):
                if 0 <= j < 6:
                    q[j] += k * self.amp * math.sin(w * self.t + n * math.pi / 3)
        grip = self.g_open
        if self.grip_cycle:
            f = 0.5 * (1 - math.cos(2 * math.pi * self.t / (2 * self.period)))
            grip = self.g_open + f * (self.g_closed - self.g_open)

        m = JointState()
        m.header.stamp = self.get_clock().now().to_msg()
        m.name = LEADER_JOINTS + [LEADER_GRIPPER]
        m.position = q + [grip]
        self.pub.publish(m)


def main():
    rclpy.init()
    node = VirtualLeader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
