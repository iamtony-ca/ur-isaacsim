# SPDX-License-Identifier: Apache-2.0
"""Convert a mesh (Collada .dae / .stl / .obj ...) to USD for the Isaac backend.

Generic one-shot converter (meters, so it references 1:1 into the metre-based
Isaac scene — the converter default is centimetres, which would scale 100x).

Usage (Isaac Sim bundled python):
    /isaac-sim/python.sh isaac/convert_dae_to_usd.py <src_mesh> <dst.usd>
"""
from __future__ import annotations

import asyncio
import sys
from pathlib import Path

from isaacsim import SimulationApp

sim_app = SimulationApp({"headless": True})

import omni.kit.asset_converter as asset_converter  # noqa: E402
from isaacsim.core.utils import extensions  # noqa: E402


async def _convert(src: str, dst: str) -> bool:
    ctx = asset_converter.AssetConverterContext()
    ctx.use_meter_as_world_unit = True   # emit metres (see convert_bracket.py)
    task = asset_converter.get_instance().create_converter_task(src, dst, None, ctx)
    ok = await task.wait_until_finished()
    if not ok:
        print(f"  status={task.get_status()} error={task.get_error_message()}")
    return ok


def main() -> int:
    if len(sys.argv) != 3:
        print("usage: convert_dae_to_usd.py <src_mesh> <dst.usd>")
        return 2
    src, dst = Path(sys.argv[1]).resolve(), Path(sys.argv[2]).resolve()
    if not src.exists():
        print(f"ERROR: source not found: {src}")
        return 1
    dst.parent.mkdir(parents=True, exist_ok=True)
    extensions.enable_extension("omni.kit.asset_converter")
    sim_app.update()
    print(f"converting {src} -> {dst}")
    fut = asyncio.ensure_future(_convert(str(src), str(dst)))
    while not fut.done():
        sim_app.update()
    ok = fut.result() and dst.exists()
    print("OK" if ok else "FAILED")
    return 0 if ok else 2


if __name__ == "__main__":
    rc = main()
    sim_app.close()
    raise SystemExit(rc)
