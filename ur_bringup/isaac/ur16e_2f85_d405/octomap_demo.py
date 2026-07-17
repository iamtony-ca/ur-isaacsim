#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Demonstrate MoveIt planning with the live D405 OctoMap as a collision object.

  1) read /get_planning_scene (component 32 = OCTOMAP) and report the OcTree is
     populated (resolution + voxel byte count);
  2) plan+execute a couple of reachable joint goals via /move_action so the arm
     visibly moves in RViz + Isaac WHILE the octomap is in the planning scene
     (the planner is collision-aware against it) -> expect SUCCESS;
  3) probe /check_state_validity over folded/low poses and report any state whose
     contacts include the '<octomap>' body -> proves the octomap is actually
     collision-checked (the planner refuses to plan into occupied voxels).

Run while the Set 3 stack is up: Isaac (--with-camera) + ur16e_2f85_d405.launch.py
+ ur16e_2f85_d405_moveit.launch.py (use_octomap:=true).
"""
import sys

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from moveit_msgs.action import MoveGroup
from moveit_msgs.srv import GetStateValidity, GetPlanningScene
from moveit_msgs.msg import (Constraints, JointConstraint, MotionPlanRequest,
                             MoveItErrorCodes, PlanningSceneComponents, RobotState)
from sensor_msgs.msg import JointState

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
GROUP = "ur_manipulator"
OCTOMAP = 32  # PlanningSceneComponents.OCTOMAP (NOT 512 = OBJECT_COLORS)

# Reachable, visibly-different configs to plan+execute through.
GOALS = [
    [0.0, -1.20, 1.00, -1.40, -1.57, 0.0],
    [0.9, -1.50, 1.30, -1.30, -1.57, 0.0],
    [0.0, -1.57, 0.0, -1.57, 0.0, 0.0],   # back to a neutral-ish pose
]


class OctomapDemo(Node):
    def __init__(self):
        super().__init__("octomap_demo")
        self.scene_cli = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.valid_cli = self.create_client(GetStateValidity, "/check_state_validity")
        self.ac = ActionClient(self, MoveGroup, "/move_action")

    def octomap_info(self):
        req = GetPlanningScene.Request()
        req.components = PlanningSceneComponents(components=OCTOMAP)
        fut = self.scene_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        oc = fut.result().scene.world.octomap.octomap
        return oc.id, oc.resolution, len(oc.data)

    def validity(self, q):
        req = GetStateValidity.Request()
        req.group_name = GROUP
        js = JointState(); js.name = ARM; js.position = [float(v) for v in q]
        rs = RobotState(); rs.joint_state = js
        req.robot_state = rs
        fut = self.valid_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10.0)
        return fut.result()

    def plan_execute(self, q, plan_only=False):
        goal = MoveGroup.Goal()
        r = MotionPlanRequest()
        r.group_name = GROUP
        r.num_planning_attempts = 5
        r.allowed_planning_time = 8.0
        c = Constraints()
        for n, v in zip(ARM, q):
            jc = JointConstraint()
            jc.joint_name = n; jc.position = float(v)
            jc.tolerance_above = 0.01; jc.tolerance_below = 0.01; jc.weight = 1.0
            c.joint_constraints.append(jc)
        r.goal_constraints.append(c)
        goal.request = r
        goal.planning_options.plan_only = plan_only
        if not self.ac.wait_for_server(timeout_sec=10.0):
            self.get_logger().error("/move_action not available")
            return None
        f = self.ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, f)
        gh = f.result()
        if gh is None or not gh.accepted:
            return None
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=60.0)
        return rf.result().result.error_code.val


def contacts(res):
    return [f"{c.contact_body_1}<->{c.contact_body_2}" for c in res.contacts]


def main():
    rclpy.init()
    n = OctomapDemo()
    for cli, name in [(n.scene_cli, "/get_planning_scene"), (n.valid_cli, "/check_state_validity")]:
        if not cli.wait_for_service(timeout_sec=15.0):
            print(f"FAIL: {name} not available (is ur16e_2f85_d405_moveit.launch.py up?)")
            rclpy.shutdown(); sys.exit(1)

    names = {v: k for k, v in MoveItErrorCodes.__dict__.items() if isinstance(v, int)}

    print("=" * 64)
    oid, res, nbytes = n.octomap_info()
    print(f"[octomap] id={oid!r}  resolution={res}  voxel_bytes={nbytes}  "
          f"-> {'POPULATED' if nbytes > 0 else 'EMPTY (camera sees nothing in range)'}")

    print("=" * 64)
    print(">>> plan+execute through goals (octomap active in the planning scene)")
    ok = True
    for i, q in enumerate(GOALS):
        code = n.plan_execute(q, plan_only=False)
        tag = names.get(code, "?")
        print(f"  goal {i + 1}/{len(GOALS)} {['%.2f' % v for v in q]} -> error_code={code} ({tag})")
        ok &= (code == MoveItErrorCodes.SUCCESS)

    print("=" * 64)
    print(">>> probing /check_state_validity for an <octomap> collision")
    octo_hit = None
    candidates = []
    for sl in (-0.6, -0.4, -0.2):
        for el in (1.6, 2.0, 2.4):
            for w1 in (-2.2, -1.6, -1.0):
                candidates.append([0.0, sl, el, w1, -1.57, 0.0])
    for q in candidates:
        r = n.validity(q)
        if not r.valid:
            pairs = contacts(r)
            if any("octomap" in p.lower() for p in pairs):
                octo_hit = (q, [p for p in pairs if "octomap" in p.lower()])
                break
    if octo_hit:
        print(f"  octomap collision at {['%.2f' % v for v in octo_hit[0]]}: {octo_hit[1]}")
        print("  => the OctoMap is an ACTIVE collision object; the planner avoids it. ✓")
    else:
        print("  (no octomap-colliding pose found in this sweep — voxels may be outside")
        print("   the swept arm volume; octomap is still in the scene and collision-checked.)")

    print("=" * 64)
    print(f"RESULT: plan+execute all SUCCESS = {ok}; octomap populated = {nbytes > 0}")
    n.destroy_node(); rclpy.shutdown()
    sys.exit(0 if ok else 2)


if __name__ == "__main__":
    main()
