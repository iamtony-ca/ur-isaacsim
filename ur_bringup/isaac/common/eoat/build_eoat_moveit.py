# SPDX-License-Identifier: Apache-2.0
"""Emit the MoveIt2/cuMotion bridge artifacts from the EOAT graph — the URDF xacro
MACRO (to fold the EOAT into the UR16e description at tool0) and the SRDF (self-
collision rules + gripper group). Same single source as the USD/URDF emitters, so
what Isaac simulates and what MoveIt collision-checks are the same links/joints
(함정 #7). Mirrors the Set-2/3 pattern (robotiq_2f85_macro.xacro + ur16e_2f85.srdf.xacro).

Outputs (default into the ur_bringup source tree so they slot into the build):
    urdf/ur16e_dualtool/<name>_eoat_macro.xacro   xacro:macro name=<name>_eoat params="parent prefix"
    srdf/common/<name>.srdf.xacro                 <xacro:ur_srdf/> + EOAT disable_collisions + gripper group

SRDF policy (the collision-discovery contract):
  * DISABLE  EOAT-internal pairs (rigid cluster hulls overlap) + EOAT-vs-{wrist_3,
             wrist_2, wrist_1, tool0, flange} (mount region) -> no false START_STATE.
  * KEEP ENABLED  EOAT-vs-arm-body (shoulder/upper_arm/forearm/base) -> MoveIt/cuMotion
             STILL reject poses that drive the tool into the arm (real self-collision).

Run:  /isaac-sim/python.sh build_eoat_moveit.py eoat_dualtool.yaml [out_root=<ur_bringup>]
      (pure python + pyyaml; no Isaac needed)
"""
from __future__ import annotations

import itertools
import sys
from pathlib import Path
from xml.dom import minidom
from xml.etree import ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eoat_model import load_graph  # noqa: E402

# UR neighbour links whose collision vs the EOAT is a false positive (mount region).
WRIST_NEIGHBORS = ["wrist_3_link", "wrist_2_link", "wrist_1_link", "tool0", "flange"]
# UR arm-body links left ENABLED so real tool-into-arm collisions are caught.
DEG = 3.141592653589793 / 180.0


def _xyz(v):
    return " ".join(f"{float(x):.6g}" for x in v)


def _box_for(link):
    return link.size or (link.physics.size if link.physics else None) or [0.05, 0.05, 0.05]


PKG = "ur_bringup"    # package shipping meshes/eoat/*.obj (exported by export_eoat_meshes.py)


def _emit_link(parent_el, l, prefix, mesh_ids):
    link = ET.SubElement(parent_el, "link", {"name": f"{prefix}{l.id}"})
    if l.physics is None:
        return
    p, box = l.physics, _box_for(l)
    inert = ET.SubElement(link, "inertial")
    ET.SubElement(inert, "origin", {"xyz": _xyz(p.com), "rpy": "0 0 0"})
    ET.SubElement(inert, "mass", {"value": f"{p.mass:.6g}"})
    ET.SubElement(inert, "inertia", {"ixx": f"{p.inertia[0]:.6g}", "iyy": f"{p.inertia[1]:.6g}",
                                     "izz": f"{p.inertia[2]:.6g}", "ixy": f"{p.inertia[3]:.6g}",
                                     "ixz": f"{p.inertia[4]:.6g}", "iyz": f"{p.inertia[5]:.6g}"})
    # Prefer the REAL part mesh (exported to meshes/eoat/) so RViz == Isaac (함정 #7);
    # the mesh is in the link (normalized) frame -> identity origin, same as the USD geo.
    # Parts without a CAD mesh (placeholder fingers) fall back to a bbox box at the COM.
    has_mesh = l.id in mesh_ids
    for tag in ("collision", "visual"):
        el = ET.SubElement(link, tag)
        if has_mesh:
            ET.SubElement(el, "origin", {"xyz": "0 0 0", "rpy": "0 0 0"})
            suffix = "_col" if tag == "collision" else ""    # collision = convex hull; visual = full
            fn = f"package://{PKG}/meshes/eoat/{l.id}{suffix}.obj"
            ET.SubElement(ET.SubElement(el, "geometry"), "mesh", {"filename": fn})
        else:
            ET.SubElement(el, "origin", {"xyz": _xyz(p.com), "rpy": "0 0 0"})
            ET.SubElement(ET.SubElement(el, "geometry"), "box", {"size": _xyz(box)})


def _emit_joint(parent_el, j, prefix, root_link):
    def name(lid):
        return "${parent}" if lid == root_link else f"{prefix}{lid}"
    joint = ET.SubElement(parent_el, "joint", {"name": f"{prefix}{j.name}", "type": j.jtype})
    ET.SubElement(joint, "parent", {"link": name(j.parent)})
    ET.SubElement(joint, "child", {"link": name(j.child)})
    ET.SubElement(joint, "origin", {"xyz": _xyz(j.xyz), "rpy": _xyz([v * DEG for v in j.rpy])})
    if j.jtype in ("revolute", "prismatic"):
        ET.SubElement(joint, "axis", {"xyz": _xyz(j.axis or [0, 0, 1])})
        lim = j.limit or {}
        ET.SubElement(joint, "limit", {"lower": f"{float(lim.get('lower', 0.0)):.6g}",
                                       "upper": f"{float(lim.get('upper', 0.0)):.6g}",
                                       "effort": f"{float(lim.get('effort', 100.0)):.6g}",
                                       "velocity": f"{float(lim.get('velocity', 1.0)):.6g}"})
        ET.SubElement(joint, "dynamics", {"damping": "0",
                                          "friction": f"{float(getattr(j, 'joint_friction', 0.0)):.6g}"})
        if j.mimic:
            ET.SubElement(joint, "mimic", {"joint": f"{prefix}{j.mimic}", "multiplier": "1", "offset": "0"})


def _pretty(el) -> str:
    return minidom.parseString(ET.tostring(el, encoding="unicode")).toprettyxml(indent="  ")


def build_macro(g, prefix="", mesh_ids=frozenset()) -> str:
    robot = ET.Element("robot", {"xmlns:xacro": "http://www.ros.org/wiki/xacro"})
    macro = ET.SubElement(robot, "xacro:macro", {"name": f"{g.name}_eoat", "params": "parent prefix"})
    for l in g.links:
        if l.id == g.root_link:
            continue
        _emit_link(macro, l, "${prefix}", mesh_ids)
    for j in g.joints:
        _emit_joint(macro, j, "${prefix}", g.root_link)
    return _pretty(robot)


def build_srdf(g) -> str:
    eoat_links = [l.id for l in g.links if l.id != g.root_link]
    driven = [j for j in g.joints if j.jtype in ("revolute", "prismatic") and not j.mimic]
    robot = ET.Element("robot", {"xmlns:xacro": "http://wiki.ros.org/xacro", "name": "$(arg name)"})
    ET.SubElement(robot, "xacro:arg", {"name": "name", "default": "ur16e"})
    ET.SubElement(robot, "xacro:include", {"filename": "$(find ur_moveit_config)/srdf/ur_macro.srdf.xacro"})
    ET.SubElement(robot, "xacro:ur_srdf")
    # gripper planning group (the single commanded DOF; mimic follows)
    if driven:
        grp = ET.SubElement(robot, "group", {"name": "gripper"})
        for j in driven:
            ET.SubElement(grp, "joint", {"name": j.name})
        lim = driven[0].limit or {}
        for state, val in (("open", lim.get("lower", 0.0)), ("closed", lim.get("upper", 0.0))):
            gs = ET.SubElement(robot, "group_state", {"name": state, "group": "gripper"})
            for j in driven:
                ET.SubElement(gs, "joint", {"name": j.name, "value": f"{float(val):.6g}"})
    # EOAT-internal pairs (rigid cluster) -> disable
    for a, b in itertools.combinations(eoat_links, 2):
        ET.SubElement(robot, "disable_collisions", {"link1": a, "link2": b, "reason": "Adjacent"})
    # EOAT vs wrist/mount neighbours -> disable (false positives). Arm body left ENABLED.
    for lk in eoat_links:
        for nb in WRIST_NEIGHBORS:
            ET.SubElement(robot, "disable_collisions", {"link1": lk, "link2": nb, "reason": "Never"})
    return _pretty(robot)


def main() -> int:
    cfg = Path(sys.argv[1]).resolve()
    g = load_graph(cfg)
    root = Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else cfg.parents[3]  # ur_bringup
    macro_path = root / "urdf" / "ur16e_dualtool" / f"{g.name}_eoat_macro.xacro"
    srdf_path = root / "srdf" / "common" / f"{g.name}.srdf.xacro"
    macro_path.parent.mkdir(parents=True, exist_ok=True)
    srdf_path.parent.mkdir(parents=True, exist_ok=True)
    # links whose real mesh was exported (export_eoat_meshes.py) -> reference <mesh>, else box
    mesh_dir = root / "meshes" / "eoat"
    mesh_ids = frozenset(p.stem for p in mesh_dir.glob("*.obj") if not p.stem.endswith("_col"))
    macro_path.write_text(build_macro(g, mesh_ids=mesh_ids))
    srdf_path.write_text(build_srdf(g))
    n_eoat = len([l for l in g.links if l.id != g.root_link])
    print(f"[build-moveit] {g.name}: {n_eoat} EOAT links  ({len(mesh_ids)} with real mesh, rest box)")
    print(f"[build-moveit]   macro -> {macro_path}")
    print(f"[build-moveit]   srdf  -> {srdf_path}")
    if not mesh_ids:
        print("[build-moveit]   NOTE: no meshes/eoat/*.obj — run export_eoat_meshes.py for RViz==Isaac")
    return 0


if __name__ == "__main__":
    sys.exit(main())
