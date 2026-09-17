#!/usr/bin/env python3
"""Teleop tracking probe (gello_comparison.md 5, BM 1): how far does the UR16e lag the leader?

Samples three things at 30 Hz for N seconds while the bridge is ENGAGED and the leader moves:
    L  the leader target, mapped with the bridge's sign/offset  (what the operator asked for)
    C  /omy_bridge/command_joint_states                          (what the bridge sent: clamp + slew)
    S  /joint_states                                             (where the arm is)
and reports, for L and C, the frame shift k that minimises |X[t] - S[t+k]| plus the error there.

How to read it:
    L->S large, C->S small   the bridge slew (max_joint_speed) is the throttle -> raise it
    L->S ~ C->S, both large  the robot itself lags (servoj / safety limits / speed)
    peak per-frame command motion == max_joint_speed/30  the cap is binding right now

Run on the REAL robot (wall clock). Under a headless Isaac the sim runs ~3.8x realtime and these
rad/s are meaningless (HISTORY.md 49.5).

    python3 teleop_lag_probe.py [seconds] [--sign 1,1,1,1,-1,1] [--offset 0,-1.5708,0,-1.5708,0,0]

The sign/offset MUST be the ones the bridge is running with (teleop_omy.launch.py defaults, or
what you set with `ros2 param set /omy_to_ur16e offset ...`).
"""
import argparse
import math

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

J = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
     "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
LJ = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]


class Probe(Node):
    def __init__(self, seconds, sign, offset):
        super().__init__("teleop_lag_probe")
        self.seconds, self.sign, self.offset = seconds, sign, offset
        self.cmd = self.st = self.ld = None
        self.rows, self.t0 = [], None
        q = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(JointState, "/omy_bridge/command_joint_states", self._c, 10)
        self.create_subscription(JointState, "/joint_states", self._s, q)
        self.create_subscription(JointState, "/leader/joint_states", self._l, q)
        self.create_timer(1 / 30, self._tick)

    def _c(self, m):
        if all(j in m.name for j in J):
            self.cmd = [m.position[m.name.index(j)] for j in J]

    def _s(self, m):
        if all(j in m.name for j in J):
            self.st = [m.position[m.name.index(j)] for j in J]

    def _l(self, m):
        if all(j in m.name for j in LJ):
            raw = [m.position[m.name.index(j)] for j in LJ]
            self.ld = [self.sign[i] * raw[i] + self.offset[i] for i in range(6)]

    def _tick(self):
        if self.cmd is None or self.st is None or self.ld is None:
            return
        if self.t0 is None:
            self.t0 = self.get_clock().now()
            self.get_logger().info("all three topics seen -- sampling")
        self.rows.append((list(self.ld), list(self.cmd), list(self.st)))
        if (self.get_clock().now() - self.t0).nanoseconds * 1e-9 > self.seconds:
            raise SystemExit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("seconds", nargs="?", type=float, default=20.0)
    ap.add_argument("--sign", default="1,1,1,1,-1,1")
    ap.add_argument("--offset", default="0,-1.5707963267948966,0,-1.5707963267948966,0,0")
    a = ap.parse_args()
    sign = [float(v) for v in a.sign.split(",")]
    offset = [float(v) for v in a.offset.split(",")]
    rclpy.init()
    p = Probe(a.seconds, sign, offset)
    try:
        rclpy.spin(p)
    except SystemExit:
        pass
    R = p.rows
    if len(R) < 60:
        raise SystemExit(f"only {len(R)} samples -- is the bridge engaged and the leader moving? "
                         "(command topic exists only while engaged)")
    print(f"samples: {len(R)} @30 Hz over {len(R) / 30:.1f} s")

    def err(src, k):
        return sum(max(abs(x - y) for x, y in zip(R[t][src], R[t + k][2]))
                   for t in range(len(R) - k)) / (len(R) - k)

    def peak(src, k):
        return max(max(abs(x - y) for x, y in zip(R[t][src], R[t + k][2])) for t in range(len(R) - k))

    for name, src in (("LEADER -> state ", 0), ("COMMAND -> state", 1)):
        res = {k: err(src, k) for k in range(0, 31)}
        best = min(res, key=res.get)
        print(f"  {name}: best k={best} frames ({best / 30 * 1000:.0f} ms)  mean err at best "
              f"{math.degrees(res[best]):.2f} deg  |  at k=0 mean {math.degrees(res[0]):.2f} deg, "
              f"peak {math.degrees(peak(src, 0)):.2f} deg")
    per = lambda src: max(max(abs(x - y) for x, y in zip(R[t][src], R[t + 1][src]))  # noqa: E731
                          for t in range(len(R) - 1))
    print(f"  peak per-frame motion: leader {per(0) * 30:.2f} rad/s, command {per(1) * 30:.2f} rad/s, "
          f"state {per(2) * 30:.2f} rad/s   (command == max_joint_speed means the slew cap is binding)")
    print("  pass line (gello_comparison.md 5-1): LEADER->state best k <= 3 frames and mean err < 0.5 deg")


if __name__ == "__main__":
    main()
