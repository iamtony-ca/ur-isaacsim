#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Drop (or remove) a small PROBE BOX at the gripped wheel's tread rim, computed
live from the gripper TF, so a gripped wheel overlaps it but the bare gripper
(fingers open, no wheel) does not. Discriminator for "is the wheel collision-checked".

  probe_box.py            # add the box at the wheel +X tread rim
  probe_box.py --remove   # remove it
"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformListener
from moveit_msgs.srv import ApplyPlanningScene
from moveit_msgs.msg import PlanningScene, CollisionObject
from geometry_msgs.msg import Pose
from shape_msgs.msg import SolidPrimitive

GRIP = "gripper_2fg14"
BASE = "base_link"
GRASP_Z = 0.179          # wheel centre in the gripper link frame (attach_wheel default)
RIM_R   = 0.062          # tread-rim radius offset along gripper +X (clear of the ±Y fingers)
BOX     = 0.020          # probe cube edge (m)


def quat_to_R(x, y, z, w):
    return np.array([
        [1-2*(y*y+z*z), 2*(x*y-z*w),   2*(x*z+y*w)],
        [2*(x*y+z*w),   1-2*(x*x+z*z), 2*(y*z-x*w)],
        [2*(x*z-y*w),   2*(y*z+x*w),   1-2*(x*x+y*y)]])


def main():
    remove = "--remove" in sys.argv
    rclpy.init()
    node = Node("probe_box")
    buf = Buffer(); TransformListener(buf, node)
    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    cli.wait_for_service(timeout_sec=15.0)

    co = CollisionObject()
    co.header.frame_id = BASE
    co.id = "probe_box"

    if remove:
        co.operation = CollisionObject.REMOVE
        where = "(removed)"
    else:
        # live gripper pose
        end = time.time() + 8
        tf = None
        while time.time() < end:
            rclpy.spin_once(node, timeout_sec=0.2)
            try:
                tf = buf.lookup_transform(BASE, GRIP, rclpy.time.Time()); break
            except Exception:
                continue
        if tf is None:
            print("no TF base_link->gripper_2fg14"); return 1
        t = tf.transform.translation; r = tf.transform.rotation
        p = np.array([t.x, t.y, t.z])
        R = quat_to_R(r.x, r.y, r.z, r.w)
        wheel_c = p + R @ np.array([0, 0, GRASP_Z])      # wheel centre
        box_c = wheel_c + RIM_R * R[:, 0]                # rim on gripper +X (away from fingers)
        co.operation = CollisionObject.ADD
        prim = SolidPrimitive(); prim.type = SolidPrimitive.BOX; prim.dimensions = [BOX, BOX, BOX]
        pose = Pose(); pose.position.x, pose.position.y, pose.position.z = map(float, box_c)
        pose.orientation.w = 1.0
        co.primitives = [prim]; co.primitive_poses = [pose]
        where = f"box@{tuple(round(v,3) for v in box_c)}  (wheel_c={tuple(round(v,3) for v in wheel_c)})"

    scene = PlanningScene(); scene.is_diff = True
    scene.world.collision_objects = [co]
    req = ApplyPlanningScene.Request(); req.scene = scene
    fut = cli.call_async(req); rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    ok = fut.result() is not None and fut.result().success
    print(f"[probe_box] {'REMOVE' if remove else 'ADD'} {where} -> {'ok' if ok else 'FAILED'}")
    rclpy.shutdown()
    return 0


if __name__ == "__main__":
    sys.exit(main())
