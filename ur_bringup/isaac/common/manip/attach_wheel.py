#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Attach / detach the WHEEL to the gripper as a MoveIt AttachedCollisionObject —
the "gripped vs not-gripped" collision-check state (to_do.md §8.3, phase-0).

WHY (collision check, not physics): our first goal is collision checking. Instead of
composing the wheel into the robot articulation USD, we attach it in the MoveIt
PLANNING SCENE only. Once attached, EVERY collision query (/check_state_validity,
plan, compute_cartesian_path) automatically treats the wheel as part of the robot —
so the existing tools (collision_report.py, approach_to_collision.py,
moveit_plan_execute_demo.py) check the carried wheel with NO change. sim == real.

TWO STATES (no runtime pick/place transitions here — that's phase-1, §8.4):
  * gripped     :  attach_wheel.py               -> wheel rides the gripper
  * not-gripped :  attach_wheel.py --detach      -> wheel removed

The wheel↔finger/gripper contact is the grasp (NOT a collision), so those links go in
`touch_links` and MoveIt ignores them; the wheel vs the rest of the arm / world IS
checked. (Same idea as adjacent-link ACM disables.)

Shape: default a CYLINDER proxy (wheel outer envelope: Ø125 x 20 mm, axis = bore = local
+Z) — simple, robust, conservative for carrying. `--shape mesh --mesh <obj>` uses a real
triangle mesh instead (export the wheel visual mesh to OBJ first). The bore hole doesn't
matter for "carrying" collision; the shaft-insertion contact is a separate (phase-1) ACM.

The grasp transform default (--grasp-xyz 0,0,0.145) is GUI-tuned via place_wheel_gui.py so
the wheel sits at the fingertips (clear of the gripper body); override for a different grasp.

Run (system python3, with move_group up):
    /usr/bin/python3 attach_wheel.py                 # gripped (cylinder proxy)
    /usr/bin/python3 attach_wheel.py --detach        # not-gripped
    /usr/bin/python3 attach_wheel.py --shape mesh --mesh <abs>/wheel.obj
"""
from __future__ import annotations

import argparse
import math
import sys

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import (PlanningScene, AttachedCollisionObject, CollisionObject)
from moveit_msgs.srv import ApplyPlanningScene
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point

WHEEL_ID = "wheel"
# gripper body link + the two fingers that hold the wheel (grasp contact -> touch_links)
DEFAULT_LINK = "gripper_2fg14"
DEFAULT_TOUCH = ["gripper_2fg14", "gripper_2fg14__finger_left", "gripper_2fg14__finger_right"]


def _quat_from_rpy(r, p, y):
    r, p, y = math.radians(r), math.radians(p), math.radians(y)
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (cr * cp * cy + sr * sp * sy,          # w
            sr * cp * cy - cr * sp * sy,          # x
            cr * sp * cy + sr * cp * sy,          # y
            cr * cp * sy - sr * sp * cy)          # z


def _pose(xyz, rpy_deg) -> Pose:
    ps = Pose()
    ps.position.x, ps.position.y, ps.position.z = [float(v) for v in xyz]
    w, x, y, z = _quat_from_rpy(*rpy_deg)
    ps.orientation.w, ps.orientation.x, ps.orientation.y, ps.orientation.z = w, x, y, z
    return ps


def _load_obj(path):
    """Minimal OBJ -> (verts, tris). Triangulates polygon faces as a fan."""
    verts, tris = [], []
    with open(path) as f:
        for line in f:
            if line.startswith("v "):
                _, x, y, z = line.split()[:4]
                verts.append((float(x), float(y), float(z)))
            elif line.startswith("f "):
                idx = [int(t.split("/")[0]) - 1 for t in line.split()[1:]]
                for k in range(1, len(idx) - 1):
                    tris.append((idx[0], idx[k], idx[k + 1]))
    return verts, tris


def _mesh_msg(path) -> Mesh:
    verts, tris = _load_obj(path)
    m = Mesh()
    m.vertices = [Point(x=v[0], y=v[1], z=v[2]) for v in verts]
    m.triangles = [MeshTriangle(vertex_indices=[int(a), int(b), int(c)]) for a, b, c in tris]
    return m


def build_aco(args) -> AttachedCollisionObject:
    aco = AttachedCollisionObject()
    aco.link_name = args.link
    aco.touch_links = args.touch_links
    co = CollisionObject()
    co.id = WHEEL_ID
    co.header.frame_id = args.link                 # grasp pose is in the gripper link frame
    pose = _pose([float(v) for v in args.grasp_xyz.split(",")],
                 [float(v) for v in args.grasp_rpy.split(",")])
    if args.detach:
        co.operation = CollisionObject.REMOVE
    elif args.shape == "mesh":
        if not args.mesh:
            raise SystemExit("--shape mesh needs --mesh <wheel.obj>")
        co.meshes = [_mesh_msg(args.mesh)]
        co.mesh_poses = [pose]
        co.operation = CollisionObject.ADD
    else:                                          # cylinder proxy (default)
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.CYLINDER
        # CYLINDER dims = [height, radius]; wheel disc thickness along +Z (bore axis)
        prim.dimensions = [float(args.height), float(args.dia) / 2.0]
        co.primitives = [prim]
        co.primitive_poses = [pose]
        co.operation = CollisionObject.ADD
    aco.object = co
    return aco


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--detach", action="store_true", help="remove the wheel (not-gripped state)")
    ap.add_argument("--link", default=DEFAULT_LINK, help="gripper link the wheel attaches to")
    ap.add_argument("--touch-links", nargs="*", default=DEFAULT_TOUCH, dest="touch_links",
                    help="links whose contact with the wheel is the grasp (ignored by MoveIt)")
    ap.add_argument("--shape", choices=["cylinder", "mesh"], default="cylinder")
    ap.add_argument("--mesh", default=None, help="OBJ path for --shape mesh")
    ap.add_argument("--dia", type=float, default=0.125, help="cylinder proxy diameter (m)")
    ap.add_argument("--height", type=float, default=0.020, help="cylinder proxy thickness along +Z (m)")
    # Grasp pose (wheel center in the gripper link frame). Default = the finger cradle
    # centre (gripper-Z 0.179) so the collision wheel matches where the physical ㄷ-channel
    # fingers grip it (finger origin Z=0.1144 + cradle centre 0.065). Tune with the fingers.
    ap.add_argument("--grasp-xyz", default="0,0,0.179", dest="grasp_xyz",
                    help="wheel center xyz in the gripper link frame (m)")
    ap.add_argument("--grasp-rpy", default="0,0,0", dest="grasp_rpy",
                    help="wheel rpy in the gripper link frame (deg); bore axis = +Z")
    args = ap.parse_args()

    rclpy.init()
    node = Node("attach_wheel")
    cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not cli.wait_for_service(timeout_sec=15.0):
        node.get_logger().error("/apply_planning_scene unavailable (is move_group up?)")
        return 1

    scene = PlanningScene()
    scene.is_diff = True
    scene.robot_state.is_diff = True
    scene.robot_state.attached_collision_objects = [build_aco(args)]
    if args.detach:
        # MoveIt detaches an attached object back INTO the world by default, so a bare
        # detach leaves a stray world "wheel" colliding with the gripper. Also REMOVE it
        # from the world so the not-gripped state is truly wheel-free.
        wr = CollisionObject()
        wr.id = WHEEL_ID
        wr.header.frame_id = args.link
        wr.operation = CollisionObject.REMOVE
        scene.world.collision_objects = [wr]

    req = ApplyPlanningScene.Request()
    req.scene = scene
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
    ok = fut.result() is not None and fut.result().success
    if args.detach:
        node.get_logger().info(f"wheel DETACHED (not-gripped) — {'ok' if ok else 'FAILED'}")
    else:
        node.get_logger().info(
            f"wheel ATTACHED to {args.link} [{args.shape}] at xyz={args.grasp_xyz} rpy={args.grasp_rpy} "
            f"touch={args.touch_links} — {'ok' if ok else 'FAILED'}")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
