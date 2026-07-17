#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""Load static CAD obstacles from obstacles.yaml into the MoveIt PLANNING SCENE.

This is the sim+real-common collision layer: move_group collision-checks against
these obstacles during planning regardless of whether the arm is Isaac (sim) or a
real UR16e (RTDE). Poses are in `fixed_frame` (robot-base-fixed) so sim == real.

Per obstacle -> moveit_msgs/CollisionObject:
  * collision: box   -> shape_msgs/SolidPrimitive BOX (size)         [no CAD needed]
  * collision: mesh|convex -> shape_msgs/Mesh loaded from the OBJ that
                        prepare_obstacles.py exports (convex = convex hull, light).
`allowed_collisions` links are written into the Allowed Collision Matrix so a
structure the robot is mounted on (the table) does not read as a constant collision.

Run (needs move_group up):
    /usr/bin/python3 load_obstacles_moveit.py [obstacles.yaml]
    # or: ros2 run ... (installed). Keeps the objects; exits after applying.
"""
from __future__ import annotations

import sys
from pathlib import Path

import yaml
import rclpy
from rclpy.node import Node

from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene
from moveit_msgs.msg import (PlanningScene, PlanningSceneComponents, CollisionObject,
                             AllowedCollisionEntry)
from shape_msgs.msg import SolidPrimitive, Mesh, MeshTriangle
from geometry_msgs.msg import Pose, Point
import math

HERE = Path(__file__).resolve().parent


def _quat_from_rpy_deg(rpy_deg):
    """ROS rpy (roll-X, pitch-Y, yaw-Z; R = Rz*Ry*Rx) degrees -> (x,y,z,w). Matches
    the EOAT pipeline convention (eoat_model / build_eoat_usd)."""
    r, p, y = [math.radians(a) for a in rpy_deg]
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return (sr * cp * cy - cr * sp * sy,     # x
            cr * sp * cy + sr * cp * sy,     # y
            cr * cp * sy - sr * sp * cy,     # z
            cr * cp * cy + sr * sp * sy)     # w


def _pose(xyz, rpy_deg) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = [float(v) for v in xyz]
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = _quat_from_rpy_deg(rpy_deg)
    return p


def _load_obj(path: Path):
    """Minimal OBJ reader -> (vertices[[x,y,z]], triangles[[i,j,k]]). Triangulates fans."""
    verts, tris = [], []
    for line in path.read_text().splitlines():
        t = line.split()
        if not t:
            continue
        if t[0] == "v":
            verts.append([float(t[1]), float(t[2]), float(t[3])])
        elif t[0] == "f":
            idx = [int(s.split("/")[0]) - 1 for s in t[1:]]
            for k in range(1, len(idx) - 1):        # fan-triangulate
                tris.append([idx[0], idx[k], idx[k + 1]])
    return verts, tris


def _mesh_msg(verts, tris) -> Mesh:
    m = Mesh()
    m.vertices = [Point(x=float(v[0]), y=float(v[1]), z=float(v[2])) for v in verts]
    m.triangles = [MeshTriangle(vertex_indices=[int(a), int(b), int(c)]) for a, b, c in tris]
    return m


def build_collision_object(o: dict, frame: str, defaults: dict, out_dir: Path,
                           level: str | None = None) -> CollisionObject:
    co = CollisionObject()
    co.header.frame_id = frame
    co.id = o["id"]
    coll = o.get("collision", defaults.get("collision", "mesh"))
    # --level overrides the fidelity for CAD obstacles (progressive checking:
    # mesh -> convexDecomposition -> convex). A box-only obstacle (no CAD) can't be a
    # mesh, so it keeps 'box'.
    if level and "cad" in o and level != "box":
        coll = level
    pose = _pose(o["pose"]["xyz"], o["pose"].get("rpy", [0, 0, 0]))
    if coll == "box":
        prim = SolidPrimitive()
        prim.type = SolidPrimitive.BOX
        prim.dimensions = [float(v) for v in o["size"]]
        co.primitives = [prim]
        co.primitive_poses = [pose]
    else:  # mesh | convex -> OBJ exported by prepare_obstacles.py
        obj = out_dir / f"{o['id']}_{coll}.obj"
        if not obj.exists():
            raise FileNotFoundError(
                f"collision mesh {obj} missing — run prepare_obstacles.py first")
        verts, tris = _load_obj(obj)
        co.meshes = [_mesh_msg(verts, tris)]
        co.mesh_poses = [pose]
    co.operation = CollisionObject.ADD
    return co


def apply_acm(scene: PlanningScene, obstacles: list):
    """Add each obstacle id to the ACM and allow collision with its listed links."""
    acm = scene.allowed_collision_matrix
    names = list(acm.entry_names)
    # ensure square matrix rows exist
    rows = [list(e.enabled) for e in acm.entry_values]
    for o in obstacles:
        oid = o["id"]
        allow = set(o.get("allowed_collisions", []) or [])
        if oid not in names:
            names.append(oid)
            for r in rows:                          # extend every existing row by 1 col
                r.append(False)
            rows.append([False] * len(names))       # new row
        oi = names.index(oid)
        for ln in allow:
            if ln in names:
                li = names.index(ln)
                rows[oi][li] = True
                rows[li][oi] = True                 # symmetric
    acm.entry_names = names
    acm.entry_values = [AllowedCollisionEntry(enabled=r) for r in rows]
    scene.allowed_collision_matrix = acm


def main() -> int:
    argv = sys.argv[1:]
    level = None
    if "--level" in argv:                              # global fidelity override
        i = argv.index("--level")
        level = argv[i + 1]
        argv = argv[:i] + argv[i + 2:]
    pos = [a for a in argv if not a.startswith("--")]
    cfg_path = Path(pos[0]).resolve() if pos else HERE / "obstacles.yaml"
    cfg = yaml.safe_load(cfg_path.read_text())
    meta = cfg.get("meta", {})
    frame = meta.get("fixed_frame", "world")
    defaults = meta.get("defaults", {})
    out_dir = (cfg_path.parent / meta.get("out_usd_dir", "../../assets/obstacles")).resolve()
    obstacles = cfg.get("obstacles", [])

    rclpy.init()
    node = Node("load_obstacles_moveit")
    get_cli = node.create_client(GetPlanningScene, "/get_planning_scene")
    app_cli = node.create_client(ApplyPlanningScene, "/apply_planning_scene")
    if not get_cli.wait_for_service(timeout_sec=15.0) or not app_cli.wait_for_service(timeout_sec=15.0):
        node.get_logger().error("planning scene services unavailable (is move_group up?)")
        return 1

    # 1) fetch current ACM so we extend (not clobber) it
    greq = GetPlanningScene.Request()
    greq.components.components = PlanningSceneComponents.ALLOWED_COLLISION_MATRIX
    gfut = get_cli.call_async(greq)
    rclpy.spin_until_future_complete(node, gfut, timeout_sec=10.0)
    scene = PlanningScene()
    scene.is_diff = True
    scene.allowed_collision_matrix = gfut.result().scene.allowed_collision_matrix

    # 2) collision objects
    if level:
        node.get_logger().info(f"--level {level}: overriding CAD-obstacle fidelity")
    for o in obstacles:
        co = build_collision_object(o, frame, defaults, out_dir, level)
        scene.world.collision_objects.append(co)
        eff = level if (level and "cad" in o and level != "box") else o.get("collision", defaults.get("collision"))
        node.get_logger().info(
            f"obstacle '{o['id']}' [{eff}] @ {o['pose']['xyz']} allow={o.get('allowed_collisions', [])}")

    # 3) ACM entries for the obstacles
    apply_acm(scene, obstacles)

    # 4) apply
    areq = ApplyPlanningScene.Request(scene=scene)
    afut = app_cli.call_async(areq)
    rclpy.spin_until_future_complete(node, afut, timeout_sec=10.0)
    ok = afut.result() is not None and afut.result().success
    node.get_logger().info(f"apply_planning_scene success={ok}  ({len(obstacles)} obstacle(s))")
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
