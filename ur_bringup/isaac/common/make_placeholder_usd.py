# SPDX-License-Identifier: Apache-2.0
"""Author simple METRE-native placeholder USD parts (no CAD kernel needed).

The container has no CAD kernel (cadquery/OCC/FreeCAD/gmsh) so we cannot emit
valid B-rep STEP for parts we don't have real CAD for. For the arbitrary
("적당히") placeholders — the cylindrical mechanical damper and the Copick camera
mount adapter — we author the pipeline's END product (USD) directly instead:
metersPerUnit=1.0, Z-up, base at z=0, a default prim, extent set. Drop-in
identical in form to convert_step_to_usd.py output, so the assembly can proceed
now and each is swappable when a real STEP arrives.

Run (Isaac bundled python, pxr only — no SimulationApp boot):
    /isaac-sim/python.sh make_placeholder_usd.py <out_dir>
"""
from __future__ import annotations

import sys
from pathlib import Path

from isaacsim import SimulationApp

_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Gf  # noqa: E402


def _new_stage(path: Path, root: str):
    path.parent.mkdir(parents=True, exist_ok=True)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    xf = UsdGeom.Xform.Define(stage, f"/{root}")
    stage.SetDefaultPrim(xf.GetPrim())
    return stage, xf


def _report(stage, label):
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = cache.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
    s = rng.GetSize()
    print(f"  {label}: size = ({s[0]*1000:.1f}, {s[1]*1000:.1f}, {s[2]*1000:.1f}) mm "
          f"min={tuple(round(v,4) for v in rng.GetMin())}")


def make_cylinder(path: Path, radius: float, height: float):
    """Cylinder along +Z, base seated at z=0 (stacks along the tool axis)."""
    stage, _ = _new_stage(path, "Damper")
    cyl = UsdGeom.Cylinder.Define(stage, "/Damper/geo")
    cyl.CreateRadiusAttr(radius)
    cyl.CreateHeightAttr(height)
    cyl.CreateAxisAttr(UsdGeom.Tokens.z)
    cyl.CreateExtentAttr([(-radius, -radius, -height / 2.0), (radius, radius, height / 2.0)])
    UsdGeom.XformCommonAPI(cyl).SetTranslate(Gf.Vec3d(0, 0, height / 2.0))
    UsdGeom.Gprim(cyl).CreateDisplayColorAttr([(0.15, 0.15, 0.17)])
    stage.GetRootLayer().Save()
    _report(stage, f"damper Ø{radius*2000:.0f}×{height*1000:.0f}mm -> {path.name}")


def make_box(path: Path, sx: float, sy: float, sz: float, root: str):
    """Box centred in x/y, base seated at z=0."""
    stage, _ = _new_stage(path, root)
    cube = UsdGeom.Cube.Define(stage, f"/{root}/geo")
    cube.CreateSizeAttr(1.0)
    cube.CreateExtentAttr([(-0.5, -0.5, -0.5), (0.5, 0.5, 0.5)])
    api = UsdGeom.XformCommonAPI(cube)
    api.SetScale(Gf.Vec3f(sx, sy, sz))
    api.SetTranslate(Gf.Vec3d(0, 0, sz / 2.0))
    UsdGeom.Gprim(cube).CreateDisplayColorAttr([(0.2, 0.22, 0.25)])
    stage.GetRootLayer().Save()
    _report(stage, f"{root} {sx*1000:.0f}×{sy*1000:.0f}×{sz*1000:.0f}mm -> {path.name}")


def main() -> int:
    out = Path(sys.argv[1] if len(sys.argv) > 1 else ".").resolve()
    # 적당히(arbitrary) but plausible dims — tweak later or replace with real STEP.
    #  damper: UR16e tool flange ~Ø63mm; a squat compliance puck.
    make_cylinder(out / "damper.usd", radius=0.0315, height=0.025)
    #  camera adapter: a plate to carry 2x Copick3D 150S on the tool-side port.
    make_box(out / "camera_adapter.usd", 0.120, 0.060, 0.012, root="CameraAdapter")
    print("done.")
    return 0


rc = main()
_app.close()
sys.exit(rc)
