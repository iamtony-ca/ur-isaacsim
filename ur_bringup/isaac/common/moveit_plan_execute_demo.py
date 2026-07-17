#!/usr/bin/env python3
"""Programmatic MoveIt2 plan+execute demo for the UR16e (sim or real).

Sends a joint-space goal to move_group's /move_action with plan_only=false, so
MoveIt plans a collision-free trajectory and executes it on
scaled_joint_trajectory_controller -> (sim) Isaac. Verifies the arm reached the
target by reading /joint_states before and after.

Run (with the ur16e + ur16e_moveit stack already up):
    source /opt/ros/jazzy/setup.bash && source /isaac-sim/ur_ws/install/setup.bash
    python3 .../isaac/moveit_plan_execute_demo.py
"""
import math

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import Constraints, JointConstraint, MotionPlanRequest, PlanningOptions

JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
          "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
TARGET = [1.0, -1.5, 1.0, -1.2, -1.57, 0.5]
GROUP = "ur_manipulator"


def latest_joint_state(node, timeout=5.0):
    got = {}
    sub = node.create_subscription(
        JointState, "/joint_states",
        lambda m: got.update({n: p for n, p in zip(m.name, m.position)}), 10)
    end = node.get_clock().now().nanoseconds + int(timeout * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end and len(got) < len(JOINTS):
        rclpy.spin_once(node, timeout_sec=0.1)
    node.destroy_subscription(sub)
    return got


def fmt(d):
    return "[" + ", ".join(f"{d.get(j, float('nan')):+.3f}" for j in JOINTS) + "]"


def main():
    rclpy.init()
    node = rclpy.create_node("moveit_plan_execute_demo")

    before = latest_joint_state(node)
    print("BEFORE /joint_states:", fmt(before))

    client = ActionClient(node, MoveGroup, "/move_action")
    if not client.wait_for_server(timeout_sec=15.0):
        print("FAIL: /move_action server not available")
        rclpy.shutdown()
        return

    req = MotionPlanRequest()
    req.group_name = GROUP
    req.num_planning_attempts = 10
    req.allowed_planning_time = 10.0
    req.max_velocity_scaling_factor = 0.2
    req.max_acceleration_scaling_factor = 0.2
    constraints = Constraints()
    for name, pos in zip(JOINTS, TARGET):
        jc = JointConstraint()
        jc.joint_name = name
        jc.position = pos
        jc.tolerance_above = 0.01
        jc.tolerance_below = 0.01
        jc.weight = 1.0
        constraints.joint_constraints.append(jc)
    req.goal_constraints.append(constraints)

    goal = MoveGroup.Goal()
    goal.request = req
    opts = PlanningOptions()
    opts.plan_only = False  # plan AND execute
    opts.planning_scene_diff.is_diff = True
    opts.planning_scene_diff.robot_state.is_diff = True
    goal.planning_options = opts

    print(f"TARGET joint goal      : {fmt(dict(zip(JOINTS, TARGET)))}")
    print("Sending goal to /move_action (plan + execute)...")

    send_future = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, send_future)
    handle = send_future.result()
    if handle is None or not handle.accepted:
        print("FAIL: goal rejected")
        rclpy.shutdown()
        return
    print("Goal accepted, planning + executing...")

    result_future = handle.get_result_async()
    rclpy.spin_until_future_complete(node, result_future, timeout_sec=60.0)
    result = result_future.result()
    if result is None:
        print("FAIL: no result (timeout)")
        rclpy.shutdown()
        return

    code = result.result.error_code.val  # 1 == SUCCESS
    print(f"MoveGroup result error_code.val = {code}  (1 = SUCCESS)")

    # let the controller settle then read final state
    end = node.get_clock().now().nanoseconds + int(2.0 * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)
    after = latest_joint_state(node)
    print("AFTER  /joint_states  :", fmt(after))

    max_err = max(abs(after.get(j, math.nan) - t) for j, t in zip(JOINTS, TARGET))
    print(f"max |reached - target| = {max_err:.4f} rad")
    print("RESULT:", "SUCCESS" if (code == 1 and max_err < 0.05) else "CHECK")

    rclpy.shutdown()


if __name__ == "__main__":
    main()
