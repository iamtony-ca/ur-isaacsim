#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Demonstrate that nvblox-mapped obstacles are avoided by the cuMotion planner.

Sends task-space (pose) goals to /move_action with the cuMotion pipeline and shows
that the SAME target is reachable in clear space but refused when it falls inside
the demo obstacle that the static camera -> nvblox -> ESDF pipeline has mapped:

  FREE : a point mirrored to the obstacle-free side (-y)        -> expect SUCCESS
  OBST : a point inside the Isaac --obstacle demo box (+y)      -> expect FAILURE
         (cuMotion's collision-aware IK can't reach into the mapped obstacle)

This is the "obstacle is in the world model and avoided" check. To prove the
failure is the obstacle (not kinematics), relaunch the cuMotion MoveIt with
read_esdf_world:=false and re-run: with no ESDF the OBST point now SUCCEEDS,
because the target itself is reachable and only the mapped obstacle blocked it.

  *** cuMotion task-space goals must target the XRDF end-effector link
      ('gripper_frame'), NOT 'tool0'. A tool0 pose goal is rejected with
      "Target link 'tool0' does not match end effector 'gripper_frame'". ***

Run while the Set 3 nvblox stack is up (see README §5 / HARDWARE.md §4):
  Isaac:  ur16e_isaac_ros2.py --asset-path .../ur16e_2f85_d405.usd \
              --with-camera --with-static-cam --obstacle
  ros2 launch ur_bringup ur16e_2f85_d405.launch.py use_sim:=true
  ros2 launch ur_bringup ur16e_2f85_d405_nvblox.launch.py use_sim_time:=true
  ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py \
              use_sim_time:=true read_esdf_world:=true ur_only:=true
  python3 .../isaac/ur16e_2f85_d405/nvblox_obstacle_demo.py
"""
import sys

import rclpy
from rclpy.action import ActionClient

from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (Constraints, PositionConstraint, OrientationConstraint,
                             MotionPlanRequest, PlanningOptions, BoundingVolume)
from shape_msgs.msg import SolidPrimitive
from geometry_msgs.msg import Pose

GROUP = "ur_manipulator"
TIP = "gripper_frame"   # cuMotion's XRDF end effector (NOT tool0 -- see module docstring)
BASE = "base_link"

# Isaac --obstacle default box: center (0.5, 0.3, 0.6), size (0.12,0.12,0.5)
# -> spans x[0.44,0.56] y[0.24,0.36] z[0.35,0.85] in base_link.
# (label, x, y, z, expect_success)
CASES = [
    ("FREE (mirror of obstacle, -y side)", 0.50, -0.30, 0.60, True),
    ("OBST (inside demo obstacle box)",    0.50,  0.30, 0.60, False),
]


def plan_to(node, client, x, y, z):
    req = MotionPlanRequest()
    req.group_name = GROUP
    req.num_planning_attempts = 5
    req.allowed_planning_time = 10.0
    req.max_velocity_scaling_factor = 0.2
    req.max_acceleration_scaling_factor = 0.2

    c = Constraints()
    pc = PositionConstraint()
    pc.header.frame_id = BASE
    pc.link_name = TIP
    sphere = SolidPrimitive()
    sphere.type = SolidPrimitive.SPHERE
    sphere.dimensions = [0.05]
    bv = BoundingVolume()
    bv.primitives.append(sphere)
    p = Pose()
    p.position.x, p.position.y, p.position.z = x, y, z
    p.orientation.w = 1.0
    bv.primitive_poses.append(p)
    pc.constraint_region = bv
    pc.weight = 1.0
    c.position_constraints.append(pc)
    # any reachable orientation is fine -- we only care about reaching the point
    oc = OrientationConstraint()
    oc.header.frame_id = BASE
    oc.link_name = TIP
    oc.orientation.w = 1.0
    oc.absolute_x_axis_tolerance = 3.14
    oc.absolute_y_axis_tolerance = 3.14
    oc.absolute_z_axis_tolerance = 3.14
    oc.weight = 0.1
    c.orientation_constraints.append(oc)
    req.goal_constraints.append(c)

    goal = MoveGroup.Goal()
    goal.request = req
    opts = PlanningOptions()
    opts.plan_only = True
    opts.planning_scene_diff.is_diff = True
    opts.planning_scene_diff.robot_state.is_diff = True
    goal.planning_options = opts

    fut = client.send_goal_async(goal)
    rclpy.spin_until_future_complete(node, fut)
    h = fut.result()
    if h is None or not h.accepted:
        return None
    rf = h.get_result_async()
    rclpy.spin_until_future_complete(node, rf, timeout_sec=30.0)
    r = rf.result()
    if r is None:
        return None
    return r.result.error_code.val   # 1 == SUCCESS


def main():
    rclpy.init()
    node = rclpy.create_node("nvblox_obstacle_demo")
    client = ActionClient(node, MoveGroup, "/move_action")
    if not client.wait_for_server(timeout_sec=15.0):
        print("FAIL: /move_action not available (is the cuMotion MoveIt up?)")
        rclpy.shutdown(); sys.exit(1)

    all_ok = True
    for label, x, y, z, expect in CASES:
        code = plan_to(node, client, x, y, z)
        success = (code == 1)
        ok = (success == expect)
        all_ok = all_ok and ok
        print(f"[{'OK' if ok else 'UNEXPECTED':10}] {label}: "
              f"error_code={code} expect_success={expect} got_success={success}")

    print("\nRESULT:", "PASS - obstacle is mapped & avoided" if all_ok
          else "CHECK - did not match expectation "
               "(obstacle spawned? nvblox up? read_esdf_world:=true?)")
    rclpy.shutdown()
    sys.exit(0 if all_ok else 2)


if __name__ == "__main__":
    main()
