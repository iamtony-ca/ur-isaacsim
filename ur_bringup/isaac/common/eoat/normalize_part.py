# SPDX-License-Identifier: Apache-2.0
"""Frame-normalize CAD part USDs: bring the mounting face to the origin and the
mounting axis to +Z, driven ENTIRELY by the EOAT yaml config (parameter tuning
only — no code edits when new parts arrive).

Problem this solves: CAD/STEP parts come in arbitrary local frames (origin not on
the mounting face, mount axis along X or Y). This authors a thin corrective wrapper
USD (references the source, adds rotate+seat translate) so downstream stacking is
trivial: every normalized part has base@z=0 and grows +Z by its own thickness.

Per-part config knobs (parts.<name>.normalize):
    rpy_deg  : [rx,ry,rz] degrees, XYZ order  -> rotate mount axis to +Z
    seat     : zmin | zmax | none             -> which rotated bbox face sits at z=0
    center_xy: bool                            -> centre x/y on the tool axis

Run (Isaac bundled python):
    /isaac-sim/python.sh normalize_part.py eoat_gripper_branch.yaml [part ...]
"""
from __future__ import annotations

import json
import os
import sys
from pathlib import Path

import yaml

from isaacsim import SimulationApp
_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Gf  # noqa: E402


def _top_prim_path(src: str):
    stg = Usd.Stage.Open(src)
    dp = stg.GetDefaultPrim()
    if dp and dp.IsValid():
        return dp.GetPath()
    for p in stg.GetPseudoRoot().GetChildren():
        return p.GetPath()
    raise RuntimeError(f"no top-level prim in {src}")


def _bbox(stage, prim):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    return cache.ComputeWorldBound(prim).ComputeAlignedRange()


def normalize(name, src_abs: Path, dst_abs: Path, norm: dict):
    rpy = norm.get("rpy_deg", [0, 0, 0])
    seat = norm.get("seat", "zmin")
    center_xy = bool(norm.get("center_xy", True))

    dst_abs.parent.mkdir(parents=True, exist_ok=True)
    src_prim = _top_prim_path(str(src_abs))
    rel = os.path.relpath(str(src_abs), str(dst_abs.parent))  # portable ref

    stage = Usd.Stage.CreateNew(str(dst_abs))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    root = UsdGeom.Xform.Define(stage, "/Part")
    stage.SetDefaultPrim(root.GetPrim())
    geo = stage.DefinePrim("/Part/geo", "Xform")
    geo.GetReferences().AddReference(assetPath=rel, primPath=src_prim)

    api = UsdGeom.XformCommonAPI(root)
    api.SetRotate(Gf.Vec3f(*[float(v) for v in rpy]))  # M = T*R ; set R first
    rng = _bbox(stage, root.GetPrim())                 # rotated bbox (T still 0)
    mn, mx = rng.GetMin(), rng.GetMax()
    tz = -mn[2] if seat == "zmin" else (-mx[2] if seat == "zmax" else 0.0)
    tx = -(mn[0] + mx[0]) / 2.0 if center_xy else 0.0
    ty = -(mn[1] + mx[1]) / 2.0 if center_xy else 0.0
    api.SetTranslate(Gf.Vec3d(tx, ty, tz))
    stage.GetRootLayer().Save()

    seated = _bbox(stage, root.GetPrim())
    s, smn, smx = seated.GetSize(), seated.GetMin(), seated.GetMax()
    # Sidecar: geometry facts the pure-python graph model (eoat_model.py) needs to
    # resolve stacking + approximate mass/inertia WITHOUT re-opening USD in Isaac.
    sidecar = {
        "size": [float(s[0]), float(s[1]), float(s[2])],          # metres
        "thickness_z": float(smx[2]),                             # top face (base@~0)
        "base_z": float(smn[2]),
        "bbox_min": [float(smn[0]), float(smn[1]), float(smn[2])],
        "bbox_max": [float(smx[0]), float(smx[1]), float(smx[2])],
    }
    dst_abs.with_suffix(".json").write_text(json.dumps(sidecar, indent=2))
    print(f"  {name:22s} rpy={rpy} seat={seat} -> base_z={smn[2]*1000:6.1f}mm "
          f"thickness_z={s[2]*1000:6.1f}mm  size=({s[0]*1000:.1f},{s[1]*1000:.1f},{s[2]*1000:.1f})mm")


def main() -> int:
    cfg_path = Path(sys.argv[1]).resolve()
    only = set(sys.argv[2:])
    cfg = yaml.safe_load(cfg_path.read_text())
    base = cfg_path.parent
    cad_dir = (base / cfg["meta"]["cad_dir"]).resolve()
    out_dir = (base / cfg["meta"]["out_normalized"]).resolve()
    print(f"normalizing -> {out_dir}")
    for name, spec in cfg["parts"].items():
        if only and name not in only:
            continue
        src = (cad_dir / spec["cad"]).resolve()
        if not src.exists():
            print(f"  {name}: MISSING source {src}")
            continue
        normalize(name, src, out_dir / f"{name}.usd", spec.get("normalize", {}))
    return 0


rc = main()
_app.close()
sys.exit(rc)
