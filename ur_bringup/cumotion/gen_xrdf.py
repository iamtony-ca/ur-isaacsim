#!/usr/bin/env python3
"""Generate a UR16e + Robotiq 2F-85 (+GRP-ES-CPL-077 coupling) XRDF for cuMotion.

Strategy (see README/HISTORY 51): start from NVIDIA's ur10e_robotiq_2f_85.xrdf (c-space,
self-collision ignore lists, buffers, tool frames) and build TWO sphere sets from the
UR16e collision meshes:

  world spheres (`collision`, obstacles / nvblox ESDF / robot_segmenter): COVER every link
    mesh with cuMotion's covering generator (`generate_collision_spheres`, `max_overshoot`
    20 mm arm + coupling, 10 mm gripper). The template's spheres sit inside the meshes:
    the meshes stuck out of spheres + 5 mm buffer by up to 39 mm (forearm), 34 (upper
    arm), 32 (coupling), 28 (shoulder), 23 (knuckles) -- an obstacle could be that deep
    in the arm while cuMotion called the path free. Covered: every link's mesh inside spheres
    + 5 mm buffer (check_xrdf.py gap <= 0; the base housing has no world spheres -- it sits on
    the table). Writes ONLY the shipped OUT_XRDF; copy the script to make an A/B candidate.
  self spheres (`self_collision`): inner (template / medial) spheres where links never
    touch, COVERING spheres (5 mm overshoot) on the links that do -- forearm, wrist_2,
    wrist_3, coupling -- and the base housing (20 mm, self only: it sits on the table).
    Covering every link at 20/30 mm would refuse 52/92 % of the 790 collision-free
    configurations (mostly base housing <-> upper arm, then forearm <-> wrist_2); covering
    every link at 5 mm is accurate (0 missed / 51) but needs 1315 spheres; inner spheres
    everywhere missed 49 of 210 MoveIt
    self-collisions in 1000 random configurations. This mix: 5 missed / 48 false alarms.

Self-collision ignore list = MoveIt's SRDF where it matters: the template also skipped
forearm <-> wrist_2 / wrist_3, which MoveIt checks and which do collide. Base housing
ignores shoulder (adjacent) and upper arm (its proximal housing turns inside the base
housing's envelope; MoveIt never reports that pair, the covering spheres always would).

Needs `cumotion` (python) + trimesh. Isaac Sim 6.1 ships both (checked on 6.1.0-rc.26 only):
  /isaac-sim/python.sh src/ur_bringup/cumotion/gen_xrdf.py      # writes ur16e_2f85.xrdf here
  /isaac-sim/python.sh src/ur_bringup/cumotion/check_xrdf.py    # coverage + named poses
(or a venv with the standalone cumotion wheel + trimesh, see README). Deterministic: the
covering generator samples with a fixed seed.
"""
import copy
import os
import sys
import xml.etree.ElementTree as ET

import numpy as np
import trimesh
import yaml

try:
    import cumotion
except ImportError:                       # Isaac Sim's bundled copy
    sys.path.insert(0, "/isaac-sim/exts/isaacsim.robot_motion.cumotion/pip_prebundle")
    import cumotion

HERE = os.path.dirname(os.path.realpath(__file__))
URDF = os.path.join(HERE, "ur16e_2f85.urdf")
BASE_XRDF = "/opt/ros/jazzy/share/isaac_ros_cumotion_robot_description/xrdf/ur10e_robotiq_2f_85.xrdf"
OUT_XRDF = os.path.join(HERE, "ur16e_2f85.xrdf")
WORLD_GEOM = "ur16e_robotiq_2f_85_world_spheres"
SELF_GEOM = "ur16e_robotiq_2f_85_self_spheres"

ARM = ["shoulder_link", "upper_arm_link", "forearm_link", "wrist_1_link", "wrist_2_link",
       "wrist_3_link", "ur_to_robotiq_link"]
OV_ARM, OV_GRIPPER = 0.020, 0.010            # world: max overshoot of the covering spheres [m]
# self: links that collide with each other in practice get covering spheres
SELF_COVER = {"forearm_link": 0.005, "wrist_2_link": 0.005, "wrist_3_link": 0.005,
              "ur_to_robotiq_link": 0.005}
SELF_BASE = ("base_link_inertia", 0.020, ["shoulder_link", "upper_arm_link"])
# self: medial (inside-the-mesh) spheres for the UR16e-specific upper arm; the rest of the
# arm and the gripper keep the template's hand-tuned spheres (shared UR10e meshes)
SELF_MEDIAL = {"upper_arm_link": 12}


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


def link_meshes(urdf):
    """link name -> trimesh in the LINK frame (collision <origin> applied)."""
    out = {}
    for link in ET.parse(urdf).getroot().findall("link"):
        col = link.find("collision")
        mesh = None if col is None else col.find("./geometry/mesh")
        if mesh is None:
            continue
        o = col.find("origin")
        xyz = [float(v) for v in ((o.get("xyz") if o is not None else None) or "0 0 0").split()]
        rpy = [float(v) for v in ((o.get("rpy") if o is not None else None) or "0 0 0").split()]
        tm = trimesh.load(resolve_mesh(mesh.get("filename")), force="mesh")
        T = np.eye(4)
        T[:3, :3] = rpy_to_matrix(rpy)
        T[:3, 3] = xyz
        tm.apply_transform(T)
        out[link.get("name")] = tm
    return out


def _spheres(sph):
    return [{"center": [round(float(v), 4) for v in np.asarray(s.center).ravel()],
             "radius": round(float(s.radius), 4)} for s in sph]


def cover(tm, overshoot):
    """Spheres covering the mesh, each poking out of it by at most ~`overshoot`."""
    return _spheres(cumotion.generate_collision_spheres(
        [v for v in np.asarray(tm.vertices, float)], [f for f in np.asarray(tm.faces, np.int32)],
        max_overshoot=float(overshoot)))


def medial(tm, n):
    """n spheres inside the mesh (medial-axis selection)."""
    gen = cumotion.create_collision_sphere_generator(
        [v for v in np.asarray(tm.vertices, float)], [f for f in np.asarray(tm.faces, np.int32)])
    return _spheres(gen.generate_spheres(int(n), 0.0))


def main():
    cumotion.set_log_level(cumotion.LogLevel.ERROR)
    meshes = link_meshes(URDF)
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

    template = copy.deepcopy(xrdf["geometry"][xrdf["collision"]["geometry"]]["spheres"])
    template.pop("tool0", None)               # our coupling sits where the template had tool0
    gripper = [l for l in meshes if l.startswith("robotiq_85_")]

    world = {l: cover(meshes[l], OV_ARM) for l in ARM}
    world.update({l: cover(meshes[l], OV_GRIPPER) for l in gripper})

    selfs = {}
    for l in ARM + gripper:
        if l in SELF_COVER:
            selfs[l] = cover(meshes[l], SELF_COVER[l])
        elif l in SELF_MEDIAL:
            selfs[l] = medial(meshes[l], SELF_MEDIAL[l])
        else:
            selfs[l] = template[l]
    base, base_ov, base_ignore = SELF_BASE
    selfs[base] = cover(meshes[base], base_ov)
    for name, s in (("world", world), ("self", selfs)):
        print(f"  {name:5s} spheres: {sum(map(len, s.values()))}  "
              + ", ".join(f"{l.replace('robotiq_85_', 'rq_')} {len(v)}" for l, v in s.items()))

    xrdf["geometry"] = {WORLD_GEOM: {"spheres": world}, SELF_GEOM: {"spheres": selfs}}
    xrdf["collision"]["geometry"] = WORLD_GEOM
    xrdf["self_collision"]["geometry"] = SELF_GEOM

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
    # MoveIt checks forearm <-> wrist_2 / wrist_3 (only adjacent pairs are disabled)
    sc["ignore"]["forearm_link"] = [l for l in sc["ignore"]["forearm_link"]
                                    if l not in ("wrist_2_link", "wrist_3_link")]
    sc["ignore"][base] = list(base_ignore)

    with open(OUT_XRDF, "w") as f:
        f.write("# Auto-generated by gen_xrdf.py (UR16e + 2F-85 + coupling) for cuMotion.\n")
        f.write("# Base: ur10e_robotiq_2f_85.xrdf. World spheres COVER the UR16e collision meshes;\n")
        f.write("# self spheres cover the links that touch (forearm/wrist_2/wrist_3/coupling/base).\n")
        yaml.safe_dump(xrdf, f, sort_keys=False, default_flow_style=None, width=120)
    print(f"wrote {OUT_XRDF}")


if __name__ == "__main__":
    main()
