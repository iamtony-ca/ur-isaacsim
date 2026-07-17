#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""MoveJ-equivalent joint-space demo (sim or real), the counterpart to cartesian_demo.py
(MoveL). MoveJ moves in JOINT space: it interpolates the joints linearly to the target,
so the TCP traces a CURVED path (unlike MoveL's straight line). If the target is a
Cartesian pose, MoveIt's IK (KDL) resolves it to a joint goal first — exactly how a UR
MoveJ with a pose target works, except the IK lives in MoveIt, not the UR controller.

Two modes:
  * pose offset (default): target = current TCP + offset; /compute_ik -> joint goal.
      /usr/bin/python3 movej_demo.py            # +Y 0.20 m (same as the MoveL demo, for contrast)
      /usr/bin/python3 movej_demo.py 0 0.2 0
  * joint target:
      /usr/bin/python3 movej_demo.py --joints 0 -1.0 1.2 -1.75 -1.57 0
  ( --no-exec  : compute + report only, don't move )

It also FK-samples the joint-interpolated path and reports the TCP's max deviation
from the straight start→goal line — the quantitative MoveJ(curved) vs MoveL(straight) contrast.
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetPositionIK, GetPositionFK
from moveit_msgs.msg import RobotState, PositionIKRequest
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose, PoseStamped
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from tf2_ros import Buffer, TransformListener

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
GROUP, TCP, BASE = "ur_manipulator", "tool0", "base_link"
JTC = "/scaled_joint_trajectory_controller/follow_joint_trajectory"


def current_joints(node, timeout=5.0):
    got = {}
    sub = node.create_subscription(JointState, "/joint_states",
                                   lambda m: got.update(zip(m.name, m.position)), 10)
    end = time.time() + timeout
    while rclpy.ok() and time.time() < end and not all(j in got for j in ARM):
        rclpy.spin_once(node, timeout_sec=0.2)
    node.destroy_subscription(sub)
    return [got.get(j, 0.0) for j in ARM]


def tcp_pose(node, tf_buf, timeout=5.0):
    end = time.time() + timeout
    while rclpy.ok() and time.time() < end:
        try:
            t = tf_buf.lookup_transform(BASE, TCP, rclpy.time.Time())
            p = Pose(); p.position.x = t.transform.translation.x
            p.position.y = t.transform.translation.y; p.position.z = t.transform.translation.z
            p.orientation = t.transform.rotation
            return p
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.2)
    return None


def solve_ik(node, cli, pose, seed):
    req = GetPositionIK.Request()
    ik = PositionIKRequest()
    ik.group_name = GROUP
    ik.ik_link_name = TCP
    ik.robot_state = RobotState(joint_state=JointState(name=ARM, position=seed))
    ps = PoseStamped(); ps.header.frame_id = BASE; ps.pose = pose
    ik.pose_stamped = ps
    ik.avoid_collisions = True
    ik.timeout = Duration(sec=2)
    req.ik_request = ik
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    r = fut.result()
    if r is None or r.error_code.val != 1:
        return None
    d = dict(zip(r.solution.joint_state.name, r.solution.joint_state.position))
    return [d[j] for j in ARM]


def fk_tcp(node, cli, q):
    req = GetPositionFK.Request()
    req.header.frame_id = BASE
    req.fk_link_names = [TCP]
    req.robot_state = RobotState(joint_state=JointState(name=ARM, position=[float(x) for x in q]))
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
    r = fut.result()
    if r is None or not r.pose_stamped:
        return None
    p = r.pose_stamped[0].pose.position
    return [p.x, p.y, p.z]


def max_deviation(node, fk_cli, start, goal, n=20):
    """max TCP distance from the straight start->goal line over the joint interpolation."""
    p0 = fk_tcp(node, fk_cli, start); p1 = fk_tcp(node, fk_cli, goal)
    if p0 is None or p1 is None:
        return None, None, None
    seg = [p1[i] - p0[i] for i in range(3)]
    L = math.sqrt(sum(s * s for s in seg)) or 1e-9
    u = [s / L for s in seg]
    dmax = 0.0
    for k in range(1, n):
        t = k / n
        q = [start[i] + (goal[i] - start[i]) * t for i in range(6)]
        p = fk_tcp(node, fk_cli, q)
        if p is None:
            continue
        v = [p[i] - p0[i] for i in range(3)]
        proj = sum(v[i] * u[i] for i in range(3))
        foot = [p0[i] + u[i] * proj for i in range(3)]
        dmax = max(dmax, math.sqrt(sum((p[i] - foot[i]) ** 2 for i in range(3))))
    return dmax, p0, p1


def execute(node, start, goal, seconds=4.0, steps=60):
    ac = ActionClient(node, FollowJointTrajectory, JTC)
    if not ac.wait_for_server(timeout_sec=10.0):
        print("  JTC unavailable"); return
    traj = JointTrajectory(); traj.joint_names = ARM
    for i in range(1, steps + 1):
        t = i / steps
        pt = JointTrajectoryPoint()
        pt.positions = [start[j] + (goal[j] - start[j]) * t for j in range(6)]
        dt = seconds * t
        pt.time_from_start = Duration(sec=int(dt), nanosec=int((dt % 1) * 1e9))
        traj.points.append(pt)
    goal_msg = FollowJointTrajectory.Goal(); goal_msg.trajectory = traj
    gh = ac.send_goal_async(goal_msg); rclpy.spin_until_future_complete(node, gh, timeout_sec=10.0)
    h = gh.result()
    if not h or not h.accepted:
        print("  goal rejected"); return
    rf = h.get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=seconds + 10.0)
    print("  executed (MoveJ joint-space).")


def main() -> int:
    argv = sys.argv[1:]
    no_exec = "--no-exec" in argv
    rclpy.init(); node = Node("movej_demo")
    tf_buf = Buffer(); TransformListener(tf_buf, node)
    fk_cli = node.create_client(GetPositionFK, "/compute_fk")
    fk_cli.wait_for_service(timeout_sec=10.0)

    start = current_joints(node)

    if "--joints" in argv:
        i = argv.index("--joints")
        goal = [float(x) for x in argv[i + 1:i + 7]]
        print(f"MoveJ to JOINT target = [{', '.join(f'{v:+.3f}' for v in goal)}]")
    else:
        nums = [float(x) for x in argv if not x.startswith("--")]
        offset = nums[:3] if len(nums) >= 3 else [0.0, 0.2, 0.0]
        ik_cli = node.create_client(GetPositionIK, "/compute_ik")
        if not ik_cli.wait_for_service(timeout_sec=15.0):
            print("/compute_ik unavailable (move_group up?)"); return 1
        cur = tcp_pose(node, tf_buf)
        if cur is None:
            print("no current TCP pose"); return 1
        target = Pose(); target.orientation = cur.orientation
        target.position.x = cur.position.x + offset[0]
        target.position.y = cur.position.y + offset[1]
        target.position.z = cur.position.z + offset[2]
        print(f"TCP target = current + {offset} m  -> IK (MoveIt/KDL)")
        goal = solve_ik(node, ik_cli, target, start)
        if goal is None:
            print("  IK FAILED for that target (unreachable / in collision)."); return 0
        print(f"  IK joint goal = [{', '.join(f'{v:+.3f}' for v in goal)}]")

    dmax, p0, p1 = max_deviation(node, fk_cli, start, goal)
    if dmax is not None:
        print(f"TCP: start {tuple(round(x,3) for x in p0)} -> goal {tuple(round(x,3) for x in p1)}")
        print(f"MoveJ path is JOINT-space: TCP deviates up to {dmax*1000:.1f} mm from the straight "
              f"start→goal line (MoveL would be ~0). Curved TCP path = the MoveJ signature.")
    if not no_exec:
        print("executing MoveJ ...")
        execute(node, start, goal)
        print("done — joints interpolated to target; TCP took a curved path. Inspect in Isaac/RViz.")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
