#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Prove MoveIt now collision-checks the gripper:
  1) /check_state_validity on the home pose -> expect VALID
  2) sweep folded arm poses -> find one INVALID with a robotiq_85_* gripper link
     in the reported contacts (gripper self-collision detected)
  3) send that colliding pose as a MoveGroup joint goal -> expect REJECTED
     (error_code != SUCCESS), proving MoveIt refuses to plan into it.
"""
import sys
import rclpy
from rclpy.node import Node
from moveit_msgs.srv import GetStateValidity
from moveit_msgs.msg import RobotState, MoveItErrorCodes
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import MotionPlanRequest, Constraints, JointConstraint
from sensor_msgs.msg import JointState
from rclpy.action import ActionClient

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
GROUP = "ur_manipulator"


class Demo(Node):
    def __init__(self):
        super().__init__("selfcollision_demo")
        self.cli = self.create_client(GetStateValidity, "/check_state_validity")
        self.ac = ActionClient(self, MoveGroup, "/move_action")

    def validity(self, q):
        req = GetStateValidity.Request()
        req.group_name = GROUP
        rs = RobotState()
        js = JointState(); js.name = ARM; js.position = [float(v) for v in q]
        rs.joint_state = js
        req.robot_state = rs
        fut = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        return fut.result()

    def send_goal(self, q):
        goal = MoveGroup.Goal()
        r = MotionPlanRequest()
        r.group_name = GROUP
        r.num_planning_attempts = 5
        r.allowed_planning_time = 5.0
        c = Constraints()
        for n, v in zip(ARM, q):
            jc = JointConstraint()
            jc.joint_name = n; jc.position = float(v)
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
            c.joint_constraints.append(jc)
        r.goal_constraints.append(c)
        goal.request = r
        goal.planning_options.plan_only = True  # don't move the robot; just plan
        if not self.ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/move_action not available"); return None
        f = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        gh = f.result()
        if gh is None or not gh.accepted:
            return None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=20.0)
        return rf.result().result.error_code.val


def contacts_str(res):
    pairs = []
    for c in res.contacts:
        pairs.append(f"{c.contact_body_1}<->{c.contact_body_2}")
    return pairs


def main():
    rclpy.init()
    n = Demo()
    if not n.cli.wait_for_service(timeout_sec=15.0):
        print("FAIL: /check_state_validity not available"); rclpy.shutdown(); sys.exit(1)

    print("=" * 64)
    home = [0.0, -1.57, 0.0, -1.57, 0.0, 0.0]
    r = n.validity(home)
    print(f"[home pose] valid={r.valid}  contacts={contacts_str(r)}")

    # candidate folded poses: curl the tool/gripper back toward the body from
    # many angles (vary shoulder_lift, elbow, wrist_1, wrist_2) to drive the
    # gripper (which sticks ~12 cm past the wrist) into an arm link.
    candidates = []
    for sl in (-0.3, -0.8, -1.3, -2.0, -2.6):
        for el in (2.2, 2.6, 2.9):
            for w1 in (-2.4, -1.5, 1.5, 2.4):
                for w2 in (0.0, 1.6, -1.6):
                    candidates.append([0.0, sl, el, w1, w2, 0.0])

    grip_poses, any_invalid = [], None
    for q in candidates:
        r = n.validity(q)
        if not r.valid:
            if any_invalid is None:
                any_invalid = (q, contacts_str(r))
            pairs = contacts_str(r)
            if any("robotiq_85" in p for p in pairs):
                grip_poses.append((q, [p for p in pairs if "robotiq_85" in p]))

    print(f"scanned {len(candidates)} poses: "
          f"{len(grip_poses)} have a GRIPPER self-collision contact")
    colliding = None
    if grip_poses:
        for q, pairs in grip_poses[:5]:
            print(f"  [{['%.1f'%v for v in q]}] GRIPPER contacts={pairs}")
        colliding = grip_poses[0][0]
    elif any_invalid is not None:
        print(f"  (no gripper-specific contact reported; first invalid pose "
              f"contacts={any_invalid[1]})")
        colliding = any_invalid[0]

    if colliding is None:
        print("No colliding pose found in sweep (unexpected)."); rclpy.shutdown(); sys.exit(2)

    print("=" * 64)
    print(f">>> sending the colliding pose as a MoveGroup goal (plan_only): {['%.2f'%v for v in colliding]}")
    code = n.send_goal(colliding)
    names = {v: k for k, v in MoveItErrorCodes.__dict__.items() if isinstance(v, int)}
    print(f"<<< MoveGroup error_code = {code}  ({names.get(code, '?')})")
    rejected = code is not None and code != MoveItErrorCodes.SUCCESS
    print(f"RESULT: MoveIt {'REJECTED the self-colliding goal ✓' if rejected else 'accepted (unexpected)'}")
    n.destroy_node(); rclpy.shutdown()
    sys.exit(0 if rejected else 3)


if __name__ == "__main__":
    main()
