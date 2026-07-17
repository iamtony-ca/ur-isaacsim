#!/usr/bin/env python
# SPDX-License-Identifier: Apache-2.0
"""Generate parametric placeholder STEP parts with a real CAD kernel (cadquery).

The Isaac container has no CAD kernel, so parts we lack real CAD for (the
cylindrical mechanical damper, the Copick camera-side adapter plate) are authored
here as proper B-rep STEP — so they flow through the SAME STEP->USD pipeline as
vendor parts and can be handed back to the mechanical team / swapped for real CAD.

Runs in the isolated CAD venv (see requirements-cad.txt), NOT Isaac python:
    /isaac-sim/volume/ur_dualtool_ws/.venv-cad/bin/python make_placeholder_step.py <out_dir> \
        [--damper-dia 63 --damper-h 25] [--adapter 120x60x12]

STEP is authored in millimetres (STEP convention); convert_step_to_usd.py emits
metres. Solids are built base@z=0 (matches the normalize `seat: zmin` convention).
"""
from __future__ import annotations

import argparse
from pathlib import Path

import cadquery as cq


def make_damper(path: Path, dia_mm: float, h_mm: float):
    """Cylindrical compliance damper, axis +Z, base at z=0."""
    solid = cq.Workplane("XY").circle(dia_mm / 2.0).extrude(h_mm)
    cq.exporters.export(solid, str(path))
    print(f"  damper  Ø{dia_mm:.0f} x {h_mm:.0f} mm -> {path.name}")


def make_camera_adapter(path: Path, x_mm: float, y_mm: float, z_mm: float):
    """Flat mounting plate for the tool-side Copick cameras, base at z=0."""
    solid = (cq.Workplane("XY")
             .box(x_mm, y_mm, z_mm, centered=(True, True, False)))
    cq.exporters.export(solid, str(path))
    print(f"  camera_adapter {x_mm:.0f} x {y_mm:.0f} x {z_mm:.0f} mm -> {path.name}")


def _dims(s: str):
    return [float(v) for v in s.lower().split("x")]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir")
    ap.add_argument("--damper-dia", type=float, default=63.0, help="damper diameter (mm)")
    ap.add_argument("--damper-h", type=float, default=25.0, help="damper height (mm)")
    ap.add_argument("--adapter", default="120x60x12", help="camera adapter LxWxH (mm)")
    a = ap.parse_args()
    out = Path(a.out_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    print(f"authoring placeholder STEP -> {out}")
    make_damper(out / "damper.step", a.damper_dia, a.damper_h)
    make_camera_adapter(out / "camera_adapter.step", *_dims(a.adapter))
    print("done.")
    return 0


raise SystemExit(main())
