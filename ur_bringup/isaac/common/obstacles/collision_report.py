#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Report WHERE the robot collides — colliding link/obstacle pairs AND the contact
POINTS (xyz in the planning frame) + penetration depth — for a given or the current
joint state. Uses MoveIt's /check_state_validity (self + planning-scene obstacles),
so it works identically in sim (Isaac) and real (RTDE).

Usage:
    # check the CURRENT robot state (reads /joint_states)
    /usr/bin/python3 collision_report.py
    # check a specific 6-joint state (rad)
    /usr/bin/python3 collision_report.py 0 -0.2 1.9 -1.7 -1.57 0
    # continuous (repeat every 0.5 s) — a light preview of the deferred monitor
    /usr/bin/python3 collision_report.py --watch
"""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
GROUP = "ur_manipulator"


def current_state(node, timeout=5.0):
    got = {}

    def cb(m):
        for i, k in enumerate(m.name):
            got[k] = m.position[i]
    sub = node.create_subscription(JointState, "/joint_states", cb, 10)
    end = time.time() + timeout
    while rclpy.ok() and time.time() < end and not all(j in got for j in ARM):
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return [got.get(j, 0.0) for j in ARM]


def report(node, cli, q):
    req = GetStateValidity.Request()
    req.group_name = GROUP
    rs = RobotState()
    js = JointState()
    js.name = ARM
    js.position = [float(x) for x in q]
    rs.joint_state = js
    req.robot_state = rs
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    res = fut.result()
    if res is None:
        print("  (no response)")
        return
    qs = "[" + ", ".join(f"{v:+.3f}" for v in q) + "]"
    if res.valid:
        print(f"VALID (collision-free)   state={qs}")
        return
    print(f"IN COLLISION  state={qs}   {len(res.contacts)} contact(s):")
    # group contacts by pair, report the deepest point per pair
    by_pair = {}
    for c in res.contacts:
        key = tuple(sorted((c.contact_body_1, c.contact_body_2)))
        p = (round(c.position.x, 4), round(c.position.y, 4), round(c.position.z, 4))
        d = round(float(c.depth), 5)
        if key not in by_pair or d > by_pair[key][1]:
            by_pair[key] = (p, d)
    for (a, b), (pt, depth) in sorted(by_pair.items()):
        print(f"   {a:26s} <-> {b:24s}  point={pt}  depth={depth*1000:.2f}mm")


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    watch = "--watch" in sys.argv
    rclpy.init()
    node = Node("collision_report")
    cli = node.create_client(GetStateValidity, "/check_state_validity")
    if not cli.wait_for_service(timeout_sec=15.0):
        print("/check_state_validity unavailable (is move_group up?)")
        return 1
    explicit = [float(x) for x in args] if len(args) >= 6 else None
    try:
        while True:
            q = explicit if explicit is not None else current_state(node)
            report(node, cli, q)
            if not watch:
                break
            time.sleep(0.5)
    except KeyboardInterrupt:
        pass
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
