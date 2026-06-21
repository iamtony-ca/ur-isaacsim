#!/usr/bin/env python3
"""Generate a UR16e + Robotiq 2F-85 (+GRP-ES-CPL-077 coupling) XRDF for cuMotion.

Strategy (see README/HISTORY): UR16e shares shoulder/wrist1-3/base + the whole
2F-85 collision meshes with UR10e, so we take NVIDIA's hand-tuned
ur10e_robotiq_2f_85.xrdf as the base and only REGENERATE the links whose mesh
differs or is new:
  - upper_arm_link, forearm_link  (UR16e-specific meshes, shorter than UR10e)
  - ur_to_robotiq_link            (the coupling; not present in the UR10e template)
The UR10e template puts spheres on `tool0`; our rig has the coupling there, so we
rename tool0 -> ur_to_robotiq_link in the collision/self_collision sections.

Spheres come out in each mesh's frame; we transform them into the link frame using
the URDF <collision><origin> (link_center = R(rpy) @ mesh_center + xyz).

Run with the venv that has cumotion + trimesh:
  deps/.venv-cumotion/bin/python deps/cumotion_gen/gen_xrdf.py
"""
import os
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml
import cumotion

HERE = os.path.dirname(os.path.realpath(__file__))
URDF = os.path.join(HERE, "ur16e_2f85.urdf")
BASE_XRDF = "/opt/ros/jazzy/share/isaac_ros_cumotion_robot_description/xrdf/ur10e_robotiq_2f_85.xrdf"
OUT_XRDF = os.path.join(HERE, "ur16e_2f85.xrdf")
GEOM_NAME = "ur16e_robotiq_2f_85_collision_spheres"

# Links to (re)generate from the UR16e URDF meshes: name -> (#spheres, radius_offset)
REGEN = {
    "upper_arm_link": (12, 0.0),
    "forearm_link": (10, 0.0),
    "ur_to_robotiq_link": (4, 0.0),
}


def rpy_to_matrix(rpy):
    r, p, y = rpy
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx  # URDF fixed-axis rpy


ARM_JOINTS = {"shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"}


def movable_gripper_joints(urdf):
    """The 2F-85 MASTER gripper joint(s): movable, non-arm, and NOT a <mimic>.
    cuMotion treats this as an auxiliary c-space coordinate and drives its mimic
    joints automatically — so we declare ONLY the master in default_joint_positions
    (declaring a mimic there is a FATAL error). With the master known, cuMotion
    accepts the full 12-joint start state move_group/RViz sends (6 arm cspace +
    finger_joint auxiliary + 5 mimics derived)."""
    root = ET.parse(urdf).getroot()
    return [j.get("name") for j in root.findall("joint")
            if j.get("type") in ("revolute", "continuous", "prismatic")
            and j.get("name") not in ARM_JOINTS
            and j.find("mimic") is None]


def resolve_mesh(fname):
    if fname.startswith("file://"):
        return fname[len("file://"):]
    if fname.startswith("package://"):
        rest = fname[len("package://"):]
        pkg, rel = rest.split("/", 1)
        return os.path.join("/opt/ros/jazzy/share", pkg, rel)
    return fname


def link_collisions(urdf):
    """link name -> (mesh_path, xyz, rpy) for links with a collision mesh."""
    root = ET.parse(urdf).getroot()
    out = {}
    for link in root.findall("link"):
        col = link.find("collision")
        if col is None:
            continue
        mesh = col.find("./geometry/mesh")
        if mesh is None:
            continue
        o = col.find("origin")
        xyz = [float(v) for v in ((o.get("xyz") if o is not None else None) or "0 0 0").split()]
        rpy = [float(v) for v in ((o.get("rpy") if o is not None else None) or "0 0 0").split()]
        out[link.get("name")] = (resolve_mesh(mesh.get("filename")), np.array(xyz), np.array(rpy))
    return out


def gen_spheres(mesh_path, xyz, rpy, n, offset):
    m = trimesh.load(mesh_path, force="mesh")
    verts = np.asarray(m.vertices, dtype=np.float64)
    tris = np.asarray(m.faces, dtype=np.int32)
    generator = cumotion.create_collision_sphere_generator(verts, tris)
    spheres = generator.generate_spheres(int(n), float(offset))
    R = rpy_to_matrix(rpy)
    out = []
    for s in spheres:
        c = R @ np.array([s.center[0], s.center[1], s.center[2]]) + xyz
        out.append({"center": [round(float(c[0]), 4), round(float(c[1]), 4),
                               round(float(c[2]), 4)], "radius": round(float(s.radius), 4)})
    return out


def main():
    cumotion.set_log_level(cumotion.LogLevel.ERROR)
    cols = link_collisions(URDF)
    with open(BASE_XRDF) as f:
        xrdf = yaml.safe_load(f)

    # Lock the 2F-85 gripper joints (present in the URDF, not arm cspace) at the
    # open position so cuMotion classifies them as LOCKED, not cspace. Without this
    # cuMotion rejects the 12-joint start state move_group/RViz sends with
    # "Number of c-space coordinates [12] must equal [6]". (Same pattern as
    # franka.xrdf, which locks panda_finger_joint1/2 via default_joint_positions.)
    xrdf.setdefault("default_joint_positions", {})
    for j in movable_gripper_joints(URDF):
        xrdf["default_joint_positions"][j] = 0.0
    print(f"  locked gripper joints: {movable_gripper_joints(URDF)}")

    old_geom = xrdf["collision"]["geometry"]            # ur10e_robotiq_2f_85_collision_spheres
    spheres = xrdf["geometry"][old_geom]["spheres"]

    # tool0 -> ur_to_robotiq_link (the coupling sits where the UR10e template used tool0)
    if "tool0" in spheres:
        del spheres["tool0"]

    # regenerate the size-specific arm links + the coupling
    for link, (n, off) in REGEN.items():
        if link not in cols:
            print(f"WARN: {link} has no collision mesh in URDF; skipping")
            continue
        mesh_path, xyz, rpy = cols[link]
        spheres[link] = gen_spheres(mesh_path, xyz, rpy, n, off)
        print(f"  {link:22s} {len(spheres[link])} spheres  <- {os.path.basename(mesh_path)}")

    # rename geometry + fix references
    xrdf["geometry"][GEOM_NAME] = xrdf["geometry"].pop(old_geom)
    xrdf["collision"]["geometry"] = GEOM_NAME
    xrdf["self_collision"]["geometry"] = GEOM_NAME

    # collision/self_collision buffer + ignore: swap tool0 -> ur_to_robotiq_link
    def swap_tool0(d):
        if not isinstance(d, dict):
            return
        if "tool0" in d:
            d["ur_to_robotiq_link"] = d.pop("tool0")
        for v in d.values():
            if isinstance(v, list):
                for i, e in enumerate(v):
                    if e == "tool0":
                        v[i] = "ur_to_robotiq_link"
    swap_tool0(xrdf["collision"].get("buffer_distance", {}))
    sc = xrdf["self_collision"]
    swap_tool0(sc.get("buffer_distance", {}))
    swap_tool0(sc.get("ignore", {}))

    with open(OUT_XRDF, "w") as f:
        f.write("# Auto-generated by gen_xrdf.py (UR16e + 2F-85 + coupling) for cuMotion.\n")
        f.write("# Base: ur10e_robotiq_2f_85.xrdf; regenerated upper_arm/forearm/coupling spheres.\n")
        yaml.safe_dump(xrdf, f, sort_keys=False, default_flow_style=None, width=120)
    print(f"wrote {OUT_XRDF}")


if __name__ == "__main__":
    main()
