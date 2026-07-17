#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Move the arm TOWARD a target joint config but STOP at the last collision-free pose
just before the first collision — so you can SEE exactly which motion/pose causes the
collision and WHAT collides WHERE. sim+real (drives scaled_joint_trajectory_controller).

How: linearly interpolate current->target, check every step with MoveIt
/check_state_validity (self + planning-scene obstacles), find the FIRST colliding step,
bisect for a precise boundary, execute the free segment via FollowJointTrajectory
(robot stops AT the boundary), then report the first-collision contacts (object, xyz,
depth). If the whole path is clear, it just moves to the target.

Usage (move_group + controllers + Isaac up; obstacles loaded):
    /usr/bin/python3 approach_to_collision.py 0 -0.2 1.9 -1.7 -1.57 0   # stop JUST BEFORE contact
    /usr/bin/python3 approach_to_collision.py <6 joints> --to-contact   # drive INTO the contact pose
    /usr/bin/python3 approach_to_collision.py <6 joints> --no-exec      # report only, don't move
"""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
GROUP = "ur_manipulator"
JTC = "/scaled_joint_trajectory_controller/follow_joint_trajectory"


def lerp(a, b, t):
    return [a[i] + (b[i] - a[i]) * t for i in range(len(a))]


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


def valid(node, cli, q):
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
    r = fut.result()
    return r.valid, (r.contacts if r else [])


def report_contacts(q, contacts):
    print(f"FIRST-COLLISION pose = [{', '.join(f'{v:+.3f}' for v in q)}]")
    by_pair = {}
    for c in contacts:
        key = tuple(sorted((c.contact_body_1, c.contact_body_2)))
        p = (round(c.position.x, 4), round(c.position.y, 4), round(c.position.z, 4))
        d = round(float(c.depth), 5)
        if key not in by_pair or d > by_pair[key][1]:
            by_pair[key] = (p, d)
    for (a, b), (pt, depth) in sorted(by_pair.items()):
        print(f"   {a:26s} <-> {b:24s}  point={pt}  depth={depth*1000:.2f}mm")


def execute(node, start, boundary, seconds=4.0, steps=60):
    ac = ActionClient(node, FollowJointTrajectory, JTC)
    if not ac.wait_for_server(timeout_sec=10.0):
        print("  (JTC action server unavailable — skipping motion)")
        return
    traj = JointTrajectory()
    traj.joint_names = ARM
    for i in range(1, steps + 1):
        t = i / steps
        pt = JointTrajectoryPoint()
        pt.positions = lerp(start, boundary, t)
        dt = seconds * t
        pt.time_from_start = Duration(sec=int(dt), nanosec=int((dt % 1) * 1e9))
        traj.points.append(pt)
    goal = FollowJointTrajectory.Goal()
    goal.trajectory = traj
    gh = ac.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, gh, timeout_sec=10.0)
    handle = gh.result()
    if not handle or not handle.accepted:
        print("  (goal rejected)")
        return
    rf = handle.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=seconds + 10.0)
    print("  moved to the collision boundary (stopped just before contact).")


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    no_exec = "--no-exec" in sys.argv
    if len(args) < 6:
        print("need 6 target joint values (rad)")
        return 1
    target = [float(x) for x in args[:6]]

    rclpy.init()
    node = Node("approach_to_collision")
    cli = node.create_client(GetStateValidity, "/check_state_validity")
    if not cli.wait_for_service(timeout_sec=15.0):
        print("/check_state_validity unavailable (move_group up?)")
        return 1

    start = current_state(node)
    print(f"start  = [{', '.join(f'{v:+.3f}' for v in start)}]")
    print(f"target = [{', '.join(f'{v:+.3f}' for v in target)}]")

    v0, _ = valid(node, cli, start)
    if not v0:
        print("START is already in collision — clear it first (report below):")
        _, c0 = valid(node, cli, start)
        report_contacts(start, c0)
        rclpy.shutdown()
        return 0

    # coarse scan for the first colliding step along the straight interpolation
    N = 120
    first_bad = None
    for i in range(1, N + 1):
        q = lerp(start, target, i / N)
        vi, _ = valid(node, cli, q)
        if not vi:
            first_bad = i
            break
    if first_bad is None:
        print("path is fully COLLISION-FREE to the target — moving all the way.")
        if not no_exec:
            execute(node, start, target)
        rclpy.shutdown()
        return 0

    # bisect between last-free (first_bad-1) and first-bad for a precise boundary
    lo, hi = (first_bad - 1) / N, first_bad / N
    for _ in range(20):
        mid = (lo + hi) / 2
        vm, _ = valid(node, cli, lerp(start, target, mid))
        if vm:
            lo = mid
        else:
            hi = mid
    boundary = lerp(start, target, lo)
    first_col = lerp(start, target, hi)
    _, contacts = valid(node, cli, first_col)

    to_contact = "--to-contact" in sys.argv
    loops = 1
    if "--loop" in sys.argv:
        i = sys.argv.index("--loop")
        loops = int(sys.argv[i + 1]) if i + 1 < len(sys.argv) and sys.argv[i + 1].isdigit() else 1000000
    frac = lo
    print(f"\nfirst collision at ~{frac*100:.1f}% of the way to the target.")
    print(f"BOUNDARY pose (last collision-free) = [{', '.join(f'{v:+.3f}' for v in boundary)}]")
    report_contacts(first_col, contacts)

    if not no_exec:
        goal_pose = first_col if to_contact else boundary
        where = "INTO the contact pose" if to_contact else "the boundary (just before)"
        if loops == 1:
            print(f"\nmoving to {where} ...")
            execute(node, start, goal_pose)
            print("robot is at " + ("the FIRST-COLLISION pose (contact realized in sim physics)."
                                     if to_contact else "the collision boundary (frozen just before).")
                  + " Inspect in Isaac/RViz.")
        else:
            # oscillate start <-> goal so you can watch the collision-causing motion repeatedly
            print(f"\nLOOPING start <-> {where} ({'∞' if loops>=1000000 else loops}x). Ctrl+C to stop.")
            k = 0
            try:
                while k < loops and rclpy.ok():
                    k += 1
                    print(f"  [{k}] start -> {where}")
                    execute(node, start, goal_pose, seconds=3.0)
                    print(f"  [{k}] back -> start")
                    execute(node, goal_pose, start, seconds=3.0)
            except KeyboardInterrupt:
                print("\nstopped.")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
