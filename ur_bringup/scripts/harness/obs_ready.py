#!/usr/bin/env python3
"""Are the observations the policy needs actually flowing? Exit 0 if yes.

Why this exists: the first V8 attempt produced NINE scored trials' worth of
"joint states are stale" and ZERO inferences. Every trial would have been recorded
as a policy failure when the policy was never asked anything -- the same class of
mistake as the three earlier harness errors (HISTORY.md 42.6): the test did not
check its own preconditions, so a broken setup read as a bad result.

Checks what UR16eROS.get_observation() checks, in the same order and with the same
2.0 s timeout, using ARRIVAL time (time.time() at callback) because that is what
the adapter uses -- not header stamps, which are sim time and would give a
different answer.

Run before each trial. A failure here means SKIP the trial, not score it.
"""
import argparse
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Image, JointState

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cameras", default="wrist=/camera/color/image_raw,"
                                         "exterior=/static_cam/color/image_raw")
    ap.add_argument("--timeout", type=float, default=2.0, help="adapter obs_timeout")
    ap.add_argument("--settle", type=float, default=3.0, help="observation window")
    ap.add_argument("--min-hz", type=float, default=10.0)
    a = ap.parse_args()

    cams = dict(p.split("=", 1) for p in a.cameras.split(",") if p)
    rclpy.init()
    n = Node("obs_ready")
    cnt = {"joints": 0, **{k: 0 for k in cams}}
    last = {k: 0.0 for k in cnt}
    names_ok = [False]

    def js(m):
        cnt["joints"] += 1
        last["joints"] = time.time()
        names_ok[0] = all(j in m.name for j in ARM)

    def mk(k):
        def cb(_m):
            cnt[k] += 1
            last[k] = time.time()
        return cb

    qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.BEST_EFFORT)
    n.create_subscription(JointState, "/joint_states", js, 10)
    for k, t in cams.items():
        n.create_subscription(Image, t, mk(k), qos)

    t0 = time.time()
    while time.time() - t0 < a.settle:
        rclpy.spin_once(n, timeout_sec=0.05)
    el = time.time() - t0
    now = time.time()

    bad = []
    for k in cnt:
        hz = cnt[k] / el
        age = (now - last[k]) if last[k] else float("inf")
        state = "fresh" if age <= a.timeout else f"STALE {age:.1f}s"
        if hz < a.min_hz or age > a.timeout:
            bad.append(k)
        print(f"   {k:9} {hz:6.1f} Hz  {state}")
    if not names_ok[0]:
        print("   joints: ARM joint names missing from /joint_states")
        bad.append("joint-names")

    rclpy.shutdown()
    if bad:
        print(f"   NOT READY: {bad}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
