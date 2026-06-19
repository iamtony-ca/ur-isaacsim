# SPDX-License-Identifier: Apache-2.0
"""
Convert the PickNik UR RealSense camera adapter mesh (Collada .dae) to USD for
the Isaac backend (Set 3 only).

The eye-in-hand D405 in Isaac is a bare UsdGeom.Camera (no geometry). To show the
*real* bracket in the GUI — matching what RViz renders from the same .dae — we
bake the vendored mesh into a USD once:

    src: meshes/picknik_ur5_realsense_camera_adapter_rev2.dae   (meter, Z-up)
    dst: isaac/assets/picknik_camera_adapter.usd

ur16e_isaac_ros2.py --with-camera then references that USD under wrist_3_link at
the tool0 frame (the PickNik macro mounts the adapter on tool0 with identity), so
the bracket sits exactly where the URDF/TF place it.

Run (Isaac Sim bundled python):
    /isaac-sim/python.sh \
        /isaac-sim/ur_ws/src/ur_bringup/isaac/convert_bracket.py
"""
from __future__ import annotations

import asyncio
from pathlib import Path

from isaacsim import SimulationApp

sim_app = SimulationApp({"headless": True})

import omni.kit.asset_converter as asset_converter  # noqa: E402
from isaacsim.core.utils import extensions  # noqa: E402

THIS_DIR = Path(__file__).resolve().parent          # isaac/ur16e_2f85_d405
SRC = THIS_DIR.parent.parent / "meshes" / "picknik_ur5_realsense_camera_adapter_rev2.dae"
DST = THIS_DIR.parent / "assets" / "picknik_camera_adapter.usd"   # isaac/assets


async def _convert(src: str, dst: str) -> bool:
    ctx = asset_converter.AssetConverterContext()
    # CRITICAL: emit the USD in METERS (metersPerUnit=1.0). The converter default
    # is centimetres, which — since USD references do NOT auto-rescale across
    # metersPerUnit — makes the 0.13 m bracket come in 100x (a ~13 m "wall") when
    # referenced into the metre-based Isaac scene.
    ctx.use_meter_as_world_unit = True
    inst = asset_converter.get_instance()
    task = inst.create_converter_task(src, dst, None, ctx)
    ok = await task.wait_until_finished()
    if not ok:
        print(f"  status={task.get_status()} error={task.get_error_message()}")
    return ok


def main() -> int:
    extensions.enable_extension("omni.kit.asset_converter")
    sim_app.update()

    if not SRC.exists():
        print(f"ERROR: source mesh not found: {SRC}")
        return 1
    DST.parent.mkdir(parents=True, exist_ok=True)

    print(f"converting:\n  {SRC}\n  -> {DST}")
    fut = asyncio.ensure_future(_convert(str(SRC), str(DST)))
    while not fut.done():
        sim_app.update()
    ok = fut.result()

    # Diagnostics: confirm units + physical size are sane (~0.075 x 0.135 x 0.014 m).
    if ok and DST.exists():
        from pxr import Usd, UsdGeom
        stage = Usd.Stage.Open(str(DST))
        mpu = UsdGeom.GetStageMetersPerUnit(stage)
        bc = UsdGeom.BBoxCache(Usd.TimeCode.Default(),
                               ["default", "render", "proxy", "guide"])
        rng = bc.ComputeWorldBound(stage.GetPseudoRoot()).ComputeAlignedRange()
        size = rng.GetMax() - rng.GetMin()
        print(f"  metersPerUnit={mpu}  bbox size(units)="
              f"({size[0]:.4f}, {size[1]:.4f}, {size[2]:.4f})")
    print("OK" if ok and DST.exists() else "FAILED")
    return 0 if (ok and DST.exists()) else 2


if __name__ == "__main__":
    rc = main()
    sim_app.close()
    raise SystemExit(rc)
