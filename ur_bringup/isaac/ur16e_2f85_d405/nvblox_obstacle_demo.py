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
read_esdf_world:=false and re-run with --control: with no ESDF the OBST point now
SUCCEEDS, because the target itself is reachable and only the mapped obstacle
blocked it.

  *** Before planning, the demo asks nvblox for the ESDF around the OBST point and
      WAITS (--esdf-timeout, default 180 s) until the obstacle is really in it. nvblox
      can run for minutes with an EMPTY map (its depth/camera_info ExactTime sync
      drops the late segmenter frames, HISTORY.md §56), and then the verdict lies:
      a fresh cuMotion refuses FREE too ('Failed to initialize ESDF workspace bounds'),
      an already-initialized one plans OBST (unobserved = free). If the obstacle never
      shows up the demo prints 'ESDF NOT READY' and exits 3 WITHOUT planning. ***

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
  python3 .../isaac/ur16e_2f85_d405/nvblox_obstacle_demo.py            # exit 0 = PASS
  # control: ...cumotion_moveit.launch.py read_esdf_world:=false, then
  python3 .../isaac/ur16e_2f85_d405/nvblox_obstacle_demo.py --control  # OBST must succeed
Exit codes: 0 PASS, 2 CHECK (planner verdict unexpected), 3 ESDF NOT READY,
1 /move_action missing.
"""
import argparse
import sys
import time

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

# Isaac --obstacle default box: center (0.5, 0.1, 0.6), size (0.12,0.5,0.1)
# -> spans x[0.44,0.56] y[-0.15,0.35] z[0.55,0.65] in base_link.
# (label, x, y, z, expect_success)   -- OBST's expectation flips with --control
CASES = [
    ("FREE (mirror of obstacle, -y side)", 0.50, -0.30, 0.60, True),
    ("OBST (inside demo obstacle box)",    0.50,  0.30, 0.60, False),
]

# ESDF readiness check (before any planning): a +-6 cm cube around the OBST point
# must hold >= MIN_OCCUPIED voxels with negative distance (inside the obstacle).
# A mapped box gives 56 of 343 there (2 cm voxels); an empty map gives 0 and every
# voxel == the unobserved value from nvblox_cumotion.yaml.
ESDF_SRV = "/nvblox_node/get_esdf_and_gradient"
ESDF_CHECK_CENTER = (0.50, 0.30, 0.60)
ESDF_CHECK_HALF = 0.06
MIN_OCCUPIED = 20
UNOBSERVED = 1000.0     # esdf_and_gradients_unobserved_value (nvblox_cumotion.yaml)


def esdf_obstacle_stats(node, cli):
    """One ESDF query over the check cube -> (stamp_s, n, unobserved, occupied) or None."""
    from nvblox_msgs.srv import EsdfAndGradients
    req = EsdfAndGradients.Request()
    req.update_esdf = True          # same as cuMotion's own requests
    req.use_aabb = True
    req.frame_id = BASE
    cx, cy, cz = ESDF_CHECK_CENTER
    h = ESDF_CHECK_HALF
    req.aabb_min_m.x, req.aabb_min_m.y, req.aabb_min_m.z = cx - h, cy - h, cz - h
    req.aabb_size_m.x = req.aabb_size_m.y = req.aabb_size_m.z = 2 * h
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    r = fut.result()
    if r is None or not r.success:
        return None
    d = r.esdf_and_gradients.data
    unobs = sum(1 for v in d if abs(v - UNOBSERVED) < 1e-3)
    occ = sum(1 for v in d if v < 0.0)
    return (r.header.stamp.sec + r.header.stamp.nanosec * 1e-9, len(d), unobs, occ)


def wait_for_obstacle_in_esdf(node, timeout_s):
    """True once nvblox's ESDF holds the demo obstacle; False (with the reason printed)
    if it does not within timeout_s."""
    try:
        from nvblox_msgs.srv import EsdfAndGradients
    except ImportError:
        print("ESDF NOT READY: cannot import nvblox_msgs (source the workspace install)")
        return False
    cli = node.create_client(EsdfAndGradients, ESDF_SRV)
    if not cli.wait_for_service(timeout_sec=min(15.0, timeout_s)):
        print(f"ESDF NOT READY: {ESDF_SRV} not available (is the nvblox launch up?)")
        return False
    t0 = time.time()
    last_print = -1e9
    st = None
    while True:
        st = esdf_obstacle_stats(node, cli)
        el = time.time() - t0
        if st is not None and st[3] >= MIN_OCCUPIED:
            print(f"ESDF ready after {el:.0f} s: {st[3]} occupied / {st[1]} voxels "
                  f"around OBST (stamp {st[0]:.2f})")
            return True
        if el >= timeout_s:
            break
        if el - last_print >= 10.0:
            print(f"  waiting for nvblox to map the obstacle ({el:.0f}/{timeout_s:.0f} s): "
                  + ("no ESDF response" if st is None else
                     f"stamp {st[0]:.2f}, unobserved {st[2]}/{st[1]}, occupied {st[3]}"))
            last_print = el
        time.sleep(2.0)
    if st is None:
        why = "no ESDF response"
    elif st[0] == 0.0:
        why = ("response stamp 0 -> nvblox has integrated NO depth frame "
               "(log shows only 'Last view not set', never 'Allocating space for last depth view')")
    elif st[2] == st[1]:
        why = "every voxel around OBST is unobserved (camera does not see it?)"
    else:
        why = f"only {st[3]} occupied voxels around OBST (obstacle spawned? --obstacle-pose moved?)"
    print(f"ESDF NOT READY after {timeout_s:.0f} s: {why}.\n"
          "  Not planning -- with an empty map the verdict is meaningless (a fresh cuMotion "
          "refuses FREE as well, an initialized one plans OBST). See HISTORY.md §56.")
    return False


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
    ap = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    ap.add_argument("--control", action="store_true",
                    help="cuMotion runs with read_esdf_world:=false: expect OBST to SUCCEED")
    ap.add_argument("--esdf-timeout", type=float, default=180.0,
                    help="max s to wait for the obstacle to appear in nvblox's ESDF "
                         "before planning (0 = skip the check)")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("nvblox_obstacle_demo")
    client = ActionClient(node, MoveGroup, "/move_action")
    if not client.wait_for_server(timeout_sec=15.0):
        print("FAIL: /move_action not available (is the cuMotion MoveIt up?)")
        rclpy.shutdown(); sys.exit(1)

    # also for --control: the control only means something if the obstacle IS mapped
    if args.esdf_timeout > 0 and not wait_for_obstacle_in_esdf(node, args.esdf_timeout):
        print("\nRESULT: ESDF NOT READY - no verdict")
        rclpy.shutdown()
        sys.exit(3)

    all_ok = True
    for label, x, y, z, expect in CASES:
        if args.control and label.startswith("OBST"):
            expect = True
        code = plan_to(node, client, x, y, z)
        success = (code == 1)
        ok = (success == expect)
        all_ok = all_ok and ok
        print(f"[{'OK' if ok else 'UNEXPECTED':10}] {label}: "
              f"error_code={code} expect_success={expect} got_success={success}")

    if args.control:
        print("\nRESULT:", "PASS (control) - obstacle mapped, OBST reachable without the ESDF"
              if all_ok else "CHECK - control did not match (read_esdf_world:=false?)")
    else:
        print("\nRESULT:", "PASS - obstacle is mapped & avoided" if all_ok
              else "CHECK - did not match expectation "
                   "(read_esdf_world:=true? for the =false control use --control)")
    rclpy.shutdown()
    sys.exit(0 if all_ok else 2)


if __name__ == "__main__":
    main()
