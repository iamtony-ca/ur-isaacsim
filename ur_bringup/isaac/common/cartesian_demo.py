#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""MoveL-equivalent Cartesian straight-line demo (sim or real). Uses MoveIt2's
/compute_cartesian_path to move the TCP (tool0) along a straight line in the base
frame — the ROS/MoveIt equivalent of URScript MoveL — then executes the returned
joint trajectory on scaled_joint_trajectory_controller (-> topic_based -> Isaac, or
-> ur_robot_driver -> real UR). MoveIt does the IK (KDL) per waypoint; the hardware
just tracks joints. Reports the achievable fraction of the straight line.

Usage (Isaac + control + move_group up):
    /usr/bin/python3 cartesian_demo.py                 # default: TCP straight down -Z 0.20 m
    /usr/bin/python3 cartesian_demo.py 0.15 0 0        # straight +X 0.15 m
    /usr/bin/python3 cartesian_demo.py 0 0 -0.2 --no-exec
"""
from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import RobotState
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from control_msgs.action import FollowJointTrajectory
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
            p = Pose()
            p.position.x = t.transform.translation.x
            p.position.y = t.transform.translation.y
            p.position.z = t.transform.translation.z
            p.orientation = t.transform.rotation
            return p
        except Exception:
            rclpy.spin_once(node, timeout_sec=0.2)
    return None


def execute(node, traj, extra_time=8.0):
    ac = ActionClient(node, FollowJointTrajectory, JTC)
    if not ac.wait_for_server(timeout_sec=10.0):
        print("  JTC action server unavailable"); return
    goal = FollowJointTrajectory.Goal(); goal.trajectory = traj
    gh = ac.send_goal_async(goal); rclpy.spin_until_future_complete(node, gh, timeout_sec=10.0)
    h = gh.result()
    if not h or not h.accepted:
        print("  goal rejected"); return
    dur = traj.points[-1].time_from_start.sec + 1 if traj.points else 5
    rf = h.get_result_async(); rclpy.spin_until_future_complete(node, rf, timeout_sec=dur + extra_time)
    print("  executed.")


def main() -> int:
    args = [a for a in sys.argv[1:] if not a.startswith("--")]
    no_exec = "--no-exec" in sys.argv
    offset = [float(x) for x in args[:3]] if len(args) >= 3 else [0.0, 0.0, -0.20]

    rclpy.init()
    node = Node("cartesian_demo")
    tf_buf = Buffer(); TransformListener(tf_buf, node)
    cli = node.create_client(GetCartesianPath, "/compute_cartesian_path")
    if not cli.wait_for_service(timeout_sec=15.0):
        print("/compute_cartesian_path unavailable (move_group up?)"); return 1

    start_pose = tcp_pose(node, tf_buf)
    if start_pose is None:
        print("could not get current TCP pose (TF base_link->tool0)"); return 1
    goal_pose = Pose()
    goal_pose.position.x = start_pose.position.x + offset[0]
    goal_pose.position.y = start_pose.position.y + offset[1]
    goal_pose.position.z = start_pose.position.z + offset[2]
    goal_pose.orientation = start_pose.orientation           # keep TCP orientation (true MoveL)

    print(f"TCP start = ({start_pose.position.x:.3f}, {start_pose.position.y:.3f}, {start_pose.position.z:.3f})")
    print(f"MoveL     : straight line by {offset} m (orientation held)")

    req = GetCartesianPath.Request()
    req.header.frame_id = BASE
    req.group_name = GROUP
    req.link_name = TCP
    js = JointState(); js.name = ARM; js.position = current_joints(node)
    req.start_state = RobotState(joint_state=js)
    req.waypoints = [goal_pose]                              # start is implicit (current state)
    req.max_step = 0.005                                     # 5 mm Cartesian resolution
    req.jump_threshold = 0.0
    req.avoid_collisions = True

    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=15.0)
    res = fut.result()
    if res is None:
        print("service call failed"); return 1
    frac = res.fraction
    n = len(res.solution.joint_trajectory.points)
    print(f"Cartesian path: {frac*100:.1f}% of the straight line achievable, {n} waypoints "
          f"({'collision-free' if frac > 0 else 'blocked'})")
    if frac <= 0.0:
        print("  no path (blocked by collision/limits/singularity)."); rclpy.shutdown(); return 0
    if frac < 0.99:
        print(f"  NOTE: only {frac*100:.0f}% reachable (limit/collision/singularity ahead) — "
              "executing the reachable part.")
    if not no_exec:
        print("executing straight-line motion ...")
        execute(node, res.solution.joint_trajectory)
        print("done — TCP moved along the straight line (MoveL). Inspect in Isaac/RViz.")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
