#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""
Robotiq 2F-85 open/close demo over ROS2.

Sends GripperCommand goals to the gripper_controller (position_controllers/
GripperActionController) and verifies finger_joint actually moves in Isaac by
watching /joint_states. Mirrors moveit_plan_execute_demo.py in spirit (program-
matic, self-verifying), but for the gripper.

Prereqs (3 terminals, in order):
  1. Isaac:    /isaac-sim/python.sh \
                 /isaac-sim/ur_ws/src/ur_bringup/isaac/ur16e_isaac_ros2.py \
                 --asset-path /isaac-sim/ur_ws/src/ur_bringup/isaac/assets/ur16e_with_2f85.usd
  2. Control:  ros2 launch ur_bringup ur16e_2f85.launch.py
  3. Demo:     python3 /isaac-sim/ur_ws/src/ur_bringup/isaac/gripper_demo.py

Run with --cycles N to repeat the open/close cycle.
"""
import argparse
import sys

import rclpy
from control_msgs.action import GripperCommand
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState

JOINT = "finger_joint"
OPEN_POS = 0.0
CLOSE_POS = 0.6   # rad; ~0.8 is fully closed, 0.6 is a firm grip for the demo

# Defaults target the sim set (ur16e_2f85.launch.py, global controller_manager).
# For the REAL gripper (robotiq_2f85_real.launch.py, namespace `gripper`) pass:
#   --action /gripper/gripper_controller/gripper_cmd \
#   --joint-states-topic /gripper/joint_states
ACTION = "/gripper_controller/gripper_cmd"


class GripperDemo(Node):
    def __init__(self, action=ACTION, joint_states_topic="/joint_states"):
        super().__init__("gripper_demo")
        self._action = action
        self._client = ActionClient(self, GripperCommand, action)
        self._last = None
        self.create_subscription(JointState, joint_states_topic, self._on_js, 10)

    def _on_js(self, msg: JointState):
        if JOINT in msg.name:
            self._last = msg.position[msg.name.index(JOINT)]

    def finger_pos(self):
        return self._last

    def send(self, position, max_effort=50.0, timeout=10.0):
        if not self._client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error(f"action server {self._action} not available")
            return False
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(max_effort)
        self.get_logger().info(f"-> GripperCommand position={position:.3f}")
        send_future = self._client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_future)
        gh = send_future.result()
        if gh is None or not gh.accepted:
            self.get_logger().error("goal rejected")
            return False
        res_future = gh.get_result_async()
        rclpy.spin_until_future_complete(self, res_future, timeout_sec=timeout)
        if res_future.result() is None:
            self.get_logger().error("no result (timeout)")
            return False
        r = res_future.result().result
        self.get_logger().info(
            f"   result: position={r.position:.4f} effort={r.effort:.2f} "
            f"stalled={r.stalled} reached_goal={r.reached_goal}")
        return True


def settle(node, seconds=1.5):
    end = node.get_clock().now().nanoseconds + int(seconds * 1e9)
    while rclpy.ok() and node.get_clock().now().nanoseconds < end:
        rclpy.spin_once(node, timeout_sec=0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--cycles", type=int, default=2)
    ap.add_argument("--action", default=ACTION,
                    help="GripperCommand action (real gripper: "
                         "/gripper/gripper_controller/gripper_cmd)")
    ap.add_argument("--joint-states-topic", default="/joint_states",
                    help="JointState topic to verify finger_joint on "
                         "(real gripper: /gripper/joint_states)")
    args, _ = ap.parse_known_args()

    rclpy.init()
    node = GripperDemo(action=args.action, joint_states_topic=args.joint_states_topic)
    # wait for first /joint_states with finger_joint
    for _ in range(100):
        rclpy.spin_once(node, timeout_sec=0.1)
        if node.finger_pos() is not None:
            break
    if node.finger_pos() is None:
        node.get_logger().error(
            f"never saw '{JOINT}' on /joint_states — is Isaac running the 2F-85 "
            "scene and ur16e_2f85.launch.py up?")
        node.destroy_node(); rclpy.shutdown(); sys.exit(1)
    node.get_logger().info(f"initial finger_joint = {node.finger_pos():.4f}")

    ok = True
    for i in range(args.cycles):
        node.get_logger().info(f"===== cycle {i + 1}/{args.cycles}: CLOSE =====")
        ok &= node.send(CLOSE_POS)
        settle(node)
        node.get_logger().info(f"   finger_joint now = {node.finger_pos():.4f}")
        node.get_logger().info(f"===== cycle {i + 1}/{args.cycles}: OPEN =====")
        ok &= node.send(OPEN_POS)
        settle(node)
        node.get_logger().info(f"   finger_joint now = {node.finger_pos():.4f}")

    # verdict: did the joint track the last open command?
    final = node.finger_pos()
    reached = final is not None and abs(final - OPEN_POS) < 0.1
    node.get_logger().info(
        f"FINAL finger_joint={final:.4f}  open-reached={reached}  goals_ok={ok}")
    node.destroy_node()
    rclpy.shutdown()
    sys.exit(0 if (ok and reached) else 2)


if __name__ == "__main__":
    main()
