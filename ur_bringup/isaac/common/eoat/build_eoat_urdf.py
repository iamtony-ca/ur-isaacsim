# SPDX-License-Identifier: Apache-2.0
"""Emit a URDF from the EOAT graph (eoat_model) — the MoveIt2 / cuMotion collision
model side of the single-source pipeline. Same link/joint graph as the Isaac-USD
emitter, so the planning collision model matches the sim asset by construction
(함정 #7: URDF must match USD or RViz↔Isaac diverge).

Pure python (no Isaac) — runs instantly. Per graph Link -> <link> with
<inertial> (mass/com/inertia from the model), <collision> + <visual> box sized to
the link bbox (conservative primitive collider; cuMotion sphere-ises it, MoveIt
uses it directly — swap for STL meshes when broken-out CAD lands). Per graph
Joint -> <joint> with origin/axis/limit; mimic joints get a <mimic> tag so the
parallel gripper stays a single commanded DOF in ros2_control/MoveIt.

URDF axis takes the real signed vector directly (no frame-flip needed, unlike USD).

Run:  /isaac-sim/python.sh build_eoat_urdf.py eoat_dualtool.yaml [out.urdf]
      (or any python with pyyaml — no Isaac needed)
"""
from __future__ import annotations

import sys
from pathlib import Path
from xml.dom import minidom
from xml.etree import ElementTree as ET

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eoat_model import load_graph  # noqa: E402


def _box_for(link):
    return link.size or [0.05, 0.05, 0.05]   # link.size = physics.size override or cad sidecar bbox


def _xyz(v):
    return " ".join(f"{float(x):.6g}" for x in v)


def build_urdf(cfg_path: Path) -> str:
    g = load_graph(cfg_path)
    robot = ET.Element("robot", {"name": g.name})

    for l in g.links:
        link = ET.SubElement(robot, "link", {"name": l.id})
        if l.physics is None:            # massless root frame (tool0)
            continue
        p = l.physics
        box = _box_for(l)
        com = p.com
        # inertial
        inert = ET.SubElement(link, "inertial")
        ET.SubElement(inert, "origin", {"xyz": _xyz(com), "rpy": "0 0 0"})
        ET.SubElement(inert, "mass", {"value": f"{p.mass:.6g}"})
        ix, iy, iz = p.inertia[0], p.inertia[1], p.inertia[2]
        ET.SubElement(inert, "inertia", {
            "ixx": f"{ix:.6g}", "iyy": f"{iy:.6g}", "izz": f"{iz:.6g}",
            "ixy": f"{p.inertia[3]:.6g}", "ixz": f"{p.inertia[4]:.6g}", "iyz": f"{p.inertia[5]:.6g}"})
        # collision + visual: conservative box at the link bbox (swap for mesh later)
        for tag in ("collision", "visual"):
            el = ET.SubElement(link, tag)
            ET.SubElement(el, "origin", {"xyz": _xyz(com), "rpy": "0 0 0"})
            geo = ET.SubElement(el, "geometry")
            ET.SubElement(geo, "box", {"size": _xyz(box)})

    for j in g.joints:
        joint = ET.SubElement(robot, "joint", {"name": j.name, "type": j.jtype})
        ET.SubElement(joint, "parent", {"link": j.parent})
        ET.SubElement(joint, "child", {"link": j.child})
        ET.SubElement(joint, "origin", {"xyz": _xyz(j.xyz), "rpy": _xyz([float(v) * 3.141592653589793 / 180.0 for v in j.rpy])})
        if j.jtype in ("revolute", "prismatic"):
            ET.SubElement(joint, "axis", {"xyz": _xyz(j.axis or [0, 0, 1])})
            lim = j.limit or {}
            ET.SubElement(joint, "limit", {
                "lower": f"{float(lim.get('lower', 0.0)):.6g}",
                "upper": f"{float(lim.get('upper', 0.0)):.6g}",
                "effort": f"{float(lim.get('effort', 100.0)):.6g}",
                "velocity": f"{float(lim.get('velocity', 1.0)):.6g}"})
            # joint dynamics — friction from the model (DEFAULT unless set in config).
            # Material friction/restitution are USD/sim-side (MoveIt/cuMotion do
            # collision-only, so they don't consume it); armature has no URDF field.
            ET.SubElement(joint, "dynamics", {
                "damping": "0", "friction": f"{float(getattr(j, 'joint_friction', 0.0)):.6g}"})
            if j.mimic:
                ET.SubElement(joint, "mimic", {"joint": j.mimic, "multiplier": "1", "offset": "0"})
    raw = ET.tostring(robot, encoding="unicode")
    return minidom.parseString(raw).toprettyxml(indent="  ")


def main() -> int:
    cfg = Path(sys.argv[1]).resolve()
    g = load_graph(cfg)
    out = Path(sys.argv[2]).resolve() if len(sys.argv) > 2 else g.out_urdf
    if out is None:
        out = cfg.parent / f"{g.name}.urdf"
    xml = build_urdf(cfg)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(xml)
    nlink = xml.count("<link ")
    njoint = xml.count("<joint ")
    print(f"[build-urdf] {g.name}: {nlink} links, {njoint} joints -> {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
