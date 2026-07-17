# SPDX-License-Identifier: Apache-2.0
"""Quick headless dump of a USD's prim tree (names, types, mesh point counts,
per-prim bbox) so we can see whether a converted CAD blob has separable
sub-assemblies (e.g. 2FG14 base / finger_left / finger_right) that could become
distinct articulation links.

Run:  /isaac-sim/python.sh inspect_usd.py <file.usd> [max_depth]
"""
from __future__ import annotations

import sys
from pathlib import Path

from isaacsim import SimulationApp
_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom  # noqa: E402


def main() -> int:
    src = Path(sys.argv[1]).resolve()
    max_depth = int(sys.argv[2]) if len(sys.argv) > 2 else 3
    stg = Usd.Stage.Open(str(src))
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    print(f"# {src.name}  metersPerUnit={UsdGeom.GetStageMetersPerUnit(stg)} "
          f"up={UsdGeom.GetStageUpAxis(stg)}  default={stg.GetDefaultPrim().GetPath()}")
    n_mesh = 0
    for prim in stg.Traverse():
        path = prim.GetPath()
        depth = len(path.pathString.strip("/").split("/"))
        is_mesh = prim.IsA(UsdGeom.Mesh)
        if is_mesh:
            n_mesh += 1
        if depth > max_depth and not is_mesh:
            continue
        pts = ""
        if is_mesh:
            arr = UsdGeom.Mesh(prim).GetPointsAttr().Get()
            pts = f" pts={len(arr) if arr else 0}"
        indent = "  " * (depth - 1)
        # bbox only for shallow prims (cost)
        bb = ""
        if depth <= 2:
            r = cache.ComputeWorldBound(prim).ComputeAlignedRange()
            s = r.GetSize()
            bb = f"  bbox=({s[0]*1000:.0f},{s[1]*1000:.0f},{s[2]*1000:.0f})mm"
        print(f"{indent}{prim.GetName()} <{prim.GetTypeName()}>{pts}{bb}")
    print(f"# total meshes: {n_mesh}")
    return 0


rc = main()
_app.close()
sys.exit(rc)
