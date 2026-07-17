# SPDX-License-Identifier: Apache-2.0
"""Extract per-part mount poses from a hand-tuned (or full-assembly) USD back into
EOAT config values — the "GUI-tune → capture" half of the pipeline.

Workflow:
  1. Open an EOAT USD (assets/ur16e_dualtool.usd or ..._full.usd) in the Isaac GUI.
  2. Select each EOAT LINK prim (named by its chain id: damper, dual_quick_changer,
     hex_qc, gripper_2fg14, qc_tool_side_B, adapter, screwdriver, camera_adapter,
     copick, ...) and move/rotate it with the gizmo until the parts mate correctly.
     ★ Move the LINK prim, not its `geo` child.
  3. Save the stage (Ctrl+S / Save As).
  4. Run this to read each link's LOCAL transform (relative to its parent link) and
     print it as ready-to-paste `chain` mount values:
         /isaac-sim/python.sh extract_poses.py <tuned.usd> eoat_dualtool.yaml

The rotation is decomposed to ROS rpy (roll-X, pitch-Y, yaw-Z; R = Rz·Ry·Rx) so it
matches the URDF emitter and build_eoat_usd exactly (함정 #7). Same script consumes a
real full-assembly USD later — structure stays, numbers become measured.
"""
from __future__ import annotations

import math
import re
import sys
from pathlib import Path

import yaml

from isaacsim import SimulationApp
_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Gf  # noqa: E402


def _pn(name: str) -> str:
    s = re.sub(r"[^A-Za-z0-9_]", "_", name)
    return s if (s[:1].isalpha() or s[:1] == "_") else "_" + s


def _mat_to_xyz_rpy(m: Gf.Matrix4d):
    """Gf local matrix -> (xyz metres, rpy degrees) in ROS convention R=Rz·Ry·Rx."""
    t = Gf.Transform(m)
    trans = t.GetTranslation()
    # Decompose about Z,Y,X -> angles (az, ay, ax); ROS rpy = [roll=ax, pitch=ay, yaw=az].
    az, ay, ax = t.GetRotation().Decompose(Gf.Vec3d.ZAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.XAxis())
    xyz = [round(float(trans[0]), 5), round(float(trans[1]), 5), round(float(trans[2]), 5)]
    rpy = [round(float(ax), 2), round(float(ay), 2), round(float(az), 2)]
    return xyz, rpy


def main() -> int:
    argv = [a for a in sys.argv[1:] if not a.startswith("--")]
    tcp_mode = "--tcp" in sys.argv          # emit tool0/TCP-relative tcp_pose instead of parent mount
    usd = Path(argv[0]).resolve()
    cfg = yaml.safe_load(Path(argv[1]).resolve().read_text())
    stage = Usd.Stage.Open(str(usd))

    # index every prim by sanitized name for lookup
    by_name = {}
    for p in stage.Traverse():
        by_name.setdefault(p.GetName(), p)

    root_link = cfg["meta"]["root_link"]
    cache = UsdGeom.XformCache()
    tool0 = by_name.get(_pn(root_link))
    w_tool0_inv = cache.GetLocalToWorldTransform(tool0).GetInverse() if tool0 else None
    key = "tcp_pose" if tcp_mode else "mount"
    print(f"# extracted {'TCP(tool0)-relative' if tcp_mode else 'parent-relative'} poses "
          f"from {usd.name}  (rpy = ROS Rz·Ry·Rx, deg; xyz m)")
    print("# paste into chain entries; mechanical-team data (tool0 frame) uses tcp_pose.")
    print("chain:")
    for e in cfg["chain"]:
        pid = e.get("id", e["part"])
        prim = by_name.get(_pn(pid))
        if prim is None:
            print(f"  # {pid}: PRIM NOT FOUND in USD (skipped)")
            continue
        if tcp_mode:                                 # link pose in tool0 frame = M_link · inv(M_tool0)
            m = cache.GetLocalToWorldTransform(prim) * w_tool0_inv
        else:
            m = UsdGeom.Xformable(prim).GetLocalTransformation()   # relative to parent link
        xyz, rpy = _mat_to_xyz_rpy(m)
        print(f"  - {{id: {pid}, part: {e['part']}, parent: {e['parent']}, "
              f"joint: {e.get('joint','fixed')}, {key}: {{xyz: {xyz}, rpy: {rpy}}}}}")
    _app.close()
    return 0


sys.exit(main())
