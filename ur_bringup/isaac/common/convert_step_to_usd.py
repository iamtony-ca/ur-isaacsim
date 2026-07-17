# SPDX-License-Identifier: Apache-2.0
"""STEP / JT / IGES (CAD B-rep) -> USD converter for the Isaac backend.

Unlike meshes (.dae/.stl/.obj), CAD B-rep files are NOT accepted by
omni.kit.asset_converter (it returns UNSUPPORTED_IMPORT_FORMAT). They go through
the HOOPS Exchange backend, driven headless here via
omni.kit.converter.hoops_core.HoopsConverterHelper.create_import_task().

We emit METRES (dMetersPerUnit=0.001 -> STEP's native millimetres become metres)
so the result references 1:1 into the metre-based Isaac scene, and Z-up to match
USD/Isaac. After conversion the result is opened and its world bounding box is
printed in metres+mm, so scale correctness is verifiable immediately (STEP unit
mistakes otherwise surface as a 1000x model in RViz).

Batch: if <src> is a DIRECTORY, every *.step/*.stp/*.igs/*.jt inside is converted
to <dst>/<stem>.usd in a single Isaac session (one boot, many parts) -- this is
the "hand me a folder of parts and build the sim myself" entry point.

Usage (Isaac Sim bundled python):
    # one part
    /isaac-sim/python.sh convert_step_to_usd.py <src.step> <dst.usd> [opts]
    # whole folder
    /isaac-sim/python.sh convert_step_to_usd.py <src_dir> <dst_dir> [opts]
    # opts: [--up X|Y|Z] [--lod 0..4] [--keep-hidden] [--no-materials]
"""
from __future__ import annotations

import argparse
import asyncio
import sys
from pathlib import Path

_UNIT_TO_MPU = {"mm": 0.001, "cm": 0.01, "m": 1.0}
_UP_TO_INT = {"X": 0, "Y": 1, "Z": 2}

_ap = argparse.ArgumentParser()
_ap.add_argument("src")
_ap.add_argument("dst")
_ap.add_argument("--input-unit", choices=list(_UNIT_TO_MPU), default="mm",
                 help="units of the CAD file (STEP is almost always mm); output USD is always metres")
_ap.add_argument("--up", choices=list(_UP_TO_INT), default="Z", help="up axis to author (Isaac=Z)")
_ap.add_argument("--lod", type=int, default=2, help="tessellation level of detail 0(coarse)..4(fine)")
_ap.add_argument("--keep-hidden", action="store_true", help="convert hidden/suppressed bodies too")
_ap.add_argument("--no-materials", action="store_true", help="drop CAD colours/materials")
_args = _ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

sim_app = SimulationApp({"headless": True})

from isaacsim.core.utils import extensions  # noqa: E402

for _ext in ("omni.kit.converter.common", "omni.kit.converter.hoops_core"):
    extensions.enable_extension(_ext)
sim_app.update()

from omni.kit.converter.hoops_core import HoopsConverterHelper  # noqa: E402


def _report(dst: str) -> None:
    from pxr import Usd, UsdGeom
    stg = Usd.Stage.Open(dst)
    if stg is None:
        print("  WARN: could not open result for inspection")
        return
    mpu = UsdGeom.GetStageMetersPerUnit(stg)
    meshes = [p for p in stg.Traverse(Usd.TraverseInstanceProxies()) if p.IsA(UsdGeom.Mesh)]
    npts = 0
    for m in meshes:
        pts = UsdGeom.Mesh(m).GetPointsAttr().Get()
        npts += len(pts) if pts else 0
    cache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    rng = cache.ComputeWorldBound(stg.GetPseudoRoot()).ComputeAlignedRange()
    size = rng.GetSize()  # in stage units; real metres = units * metersPerUnit
    real = tuple(s * mpu for s in size)
    print(f"  metersPerUnit={mpu} upAxis={UsdGeom.GetStageUpAxis(stg)}")
    print(f"  meshes={len(meshes)} points={npts}")
    print(f"  bbox(units) min={tuple(round(v,3) for v in rng.GetMin())} "
          f"max={tuple(round(v,3) for v in rng.GetMax())}")
    print(f"  REAL size = ({real[0]*1000:.1f}, {real[1]*1000:.1f}, {real[2]*1000:.1f}) mm  "
          f"[metersPerUnit-corrected]")
    if abs(mpu - 1.0) > 1e-9:
        print(f"  WARN: metersPerUnit != 1.0 -> will NOT drop 1:1 into the metre Isaac scene; "
              f"referencing scales it {1.0/mpu:g}x. Re-run so output is metre-native.")


_CAD_EXTS = (".step", ".stp", ".stpz", ".igs", ".iges", ".jt")


def _pairs():
    """Return [(src_file, dst_usd), ...] for single-file or whole-folder input."""
    src, dst = Path(_args.src).resolve(), Path(_args.dst).resolve()
    if not src.exists():
        print(f"ERROR: source not found: {src}")
        return []
    if src.is_dir():
        files = sorted(p for p in src.iterdir()
                       if p.is_file() and p.suffix.lower() in _CAD_EXTS)
        if not files:
            print(f"ERROR: no CAD files ({', '.join(_CAD_EXTS)}) in {src}")
        dst.mkdir(parents=True, exist_ok=True)
        return [(f, dst / (f.stem + ".usd")) for f in files]
    dst.parent.mkdir(parents=True, exist_ok=True)
    return [(src, dst)]


def _convert_one(helper, src: Path, dst: Path, file_format_args) -> bool:
    print(f"\n>>> converting {src.name} -> {dst.name}")
    fut = asyncio.ensure_future(helper.create_import_task(str(src), str(dst), file_format_args))
    while not fut.done():
        sim_app.update()
    url, status = fut.result()
    code = status[0] if not isinstance(status, int) else status
    if code == 0 and dst.exists():
        print("  OK; inspecting result:")
        _report(str(dst))
        return True
    print(f"  FAILED code={code} status={status} url={url!r}")
    return False


def main() -> int:
    pairs = _pairs()
    if not pairs:
        return 1
    # Always emit METRE-native USD (metersPerUnit=1.0) so it references 1:1 into
    # the metre-based Isaac scene. HOOPS reads the STEP's own unit (usually mm)
    # and scales to metres; the output is authored Z-up to match Isaac/USD.
    file_format_args = {
        "dMetersPerUnit": "1.0",
        "iUpAxis": str(_UP_TO_INT[_args.up]),
        "tessLOD": str(_args.lod),
        "bOptimize": "true",
        "instancing": "true",
        "convertHidden": "true" if _args.keep_hidden else "false",
        "useMaterials": "false" if _args.no_materials else "true",
    }
    helper = HoopsConverterHelper()  # reused across all parts (one Isaac session)
    ok = sum(_convert_one(helper, s, d, file_format_args) for s, d in pairs)
    print(f"\n==== converted {ok}/{len(pairs)} part(s) -> {Path(_args.dst).resolve()} ====")
    return 0 if ok == len(pairs) else 1


rc = main()
sim_app.close()
sys.exit(rc)
