#!/usr/bin/env python3
"""Check a cuMotion XRDF against the UR16e collision meshes and the poses we use.

    /isaac-sim/python.sh src/ur_bringup/cumotion/check_xrdf.py [xrdf]

1. world coverage: how far each link's collision mesh sticks out of its world spheres
   (+ the XRDF collision buffer). Pass: <= 5 mm on every link.
2. self-collision (cuMotion's own check) at the named poses (reset_pose.py + the XRDF
   default): none may be in self-collision.
3. ground: world spheres (+ buffer) at the named poses stay above cuMotion's ground plane
   (ur16e_2f85_d405_cumotion_moveit.launch.py: ground_plane_z_offset -0.05).
Exit 1 on any failure. Agreement with MoveIt over random configurations was measured
separately (HISTORY.md 51) -- it needs a running move_group.
"""
import os
import sys

import numpy as np
import trimesh
import yaml

HERE = os.path.dirname(os.path.realpath(__file__))
sys.path.insert(0, HERE)
from gen_xrdf import URDF, cumotion, link_meshes  # noqa: E402

GROUND_Z = -0.05
POSES = {   # reset_pose.py + XRDF default; rad
    "ready": [3.1217, -0.0276, -2.6534, -0.3559, 1.5263, -0.0276],
    "ready_v2": [0.0, -3.1416, 2.6529, -2.6529, -1.5707, 0.0],
    "ready_v1": [0.0, -1.5707, 1.5707, -1.5707, -1.5707, 0.0],
    "home": [0.0, -1.5707, 0.0, 0.0, 0.0, 0.0],
    "up": [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0],
    "zero": [0.0] * 6,
}


def main():
    xrdf = sys.argv[1] if len(sys.argv) > 1 else os.path.join(HERE, "ur16e_2f85.xrdf")
    cumotion.set_log_level(cumotion.LogLevel.ERROR)
    X = yaml.safe_load(open(xrdf))
    POSES["xrdf_default"] = [X["default_joint_positions"][j] for j in X["cspace"]["joint_names"]]
    world = X["geometry"][X["collision"]["geometry"]]["spheres"]
    buf = X["collision"].get("buffer_distance", {}) or {}
    fails = []

    print("1. world coverage: mesh outside spheres + buffer [mm]   (pass <= 5)")
    for link, tm in link_meshes(URDF).items():
        if link not in world:
            print(f"   {link:36s}  -- no world spheres (ok only for the base, which sits on the table)")
            continue
        c = np.array([s["center"] for s in world[link]], float)
        r = np.array([s["radius"] for s in world[link]], float) + float(buf.get(link, 0.0))
        pts, _ = trimesh.sample.sample_surface(tm, 40000, seed=0)
        pts = np.vstack([pts, tm.vertices])
        gap = (np.linalg.norm(pts[:, None] - c[None], axis=-1) - r).min(1).max()
        ok = gap <= 0.005
        fails += [] if ok else [f"coverage {link}"]
        print(f"   {link:36s} {len(c):4d} spheres  {gap * 1e3:6.1f}  {'ok' if ok else 'FAIL'}")

    rd = cumotion.load_robot_from_file(xrdf, URDF)
    ins = cumotion.create_robot_world_inspector(rd, cumotion.create_world().add_world_view())
    print(f"   spheres: world {ins.num_world_collision_spheres()}, self {ins.num_self_collision_spheres()}")
    print("2./3. named poses: self-collision, lowest world sphere above ground")
    for n, q in POSES.items():
        q = np.array(q, float).reshape(-1, 1)
        pairs = ins.frames_in_self_collision(q)
        P = np.array([np.asarray(p).ravel() for p in ins.world_collision_sphere_positions(q)])
        low = (P[:, 2] - np.array(ins.world_collision_sphere_radii())).min() - 0.005
        ok = not pairs and low > GROUND_Z
        fails += [] if ok else [f"pose {n}"]
        print(f"   {n:13s} self {'free' if not pairs else pairs}  lowest {low * 1e3:7.1f} mm  {'ok' if ok else 'FAIL'}")
    print("\nPASS" if not fails else f"\nFAIL: {fails}")
    sys.exit(1 if fails else 0)


if __name__ == "__main__":
    main()
