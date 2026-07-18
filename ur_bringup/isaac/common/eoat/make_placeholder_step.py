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


def make_gripper_finger(path: Path, mirror: bool, wheel_dia: float,
                        wheel_cy: float, wheel_z_lo: float, wheel_z_hi: float,
                        elbow=(70.0, 54.0, 0.0, 28.0), jaw=(70.0, 44.0, 74.0, 26.0, 82.0),
                        channel_clearance=0.5):
    """Custom 2FG14 finger sized to grip the Ø{wheel_dia} wheel by its outer tread.

    L-bracket in the FINGER LINK frame (origin = slider mount, +Z = tool/approach axis
    toward the wheel, +Y = outward/grip). The finger routes AROUND the wheel (it must not
    pass through it): `elbow` reaching out in +Y from the slider (base z=0) BELOW the wheel
    -> `jaw` rising in +Z along the wheel OD. The jaw's inner face is a **ㄷ (U/C) CHANNEL**:
    a CONCAVE cradle (Z-axis cylinder of the wheel radius, centred at `wheel_cy`) recessed
    ONLY over the wheel Z-band, so the material above and below stays proud as two LIPS that
    trap the wheel's flat faces axially — the tread seats radially in the cradle and can't
    slip out along the bore axis. `finger_right` = XZ mirror. (No chunky mount block.)

    `elbow`=(X_width, Y_outer, Z_lo, Z_hi); `jaw`=(X_width, Y_inner(lip), Y_outer, Z_lo, Z_hi) mm.
    `channel_clearance` = per-side gap (mm) between the lips and the wheel faces. Grasp-geometry
    defaults come from eoat_dualtool.yaml: finger origin (body Y=±13, Z=80) + tuned wheel grasp
    (Z=145, thick 20) -> wheel centre in the finger frame Y=∓13, wheel band Z=[55,75]. ROUGH
    placeholder — tune here / in the Isaac GUI, or swap for the mech team's real finger CAD.
    """
    R = wheel_dia / 2.0
    ex, ey, ezl, ezh = elbow                                    # base: reach out in +Y from z=0, below the wheel
    body = (cq.Workplane("XY").workplane(offset=ezl).center(0, ey / 2)
            .box(ex, ey, ezh - ezl, centered=(True, True, False)))
    jx, jyi, jyo, jzl, jzh = jaw                                # jaw wall Y=[inner lip, outer]
    body = body.union(cq.Workplane("XY").workplane(offset=jzl).center(0, (jyi + jyo) / 2)
                      .box(jx, jyo - jyi, jzh - jzl, centered=(True, True, False)))
    # ㄷ channel: recess the cradle ONLY over the wheel Z-band -> the lips above/below stay
    # proud (at Y=jyi) and capture the wheel faces; the tread seats on the cradle (Y=R-|cy|).
    gz_lo, gz_hi = wheel_z_lo - channel_clearance, wheel_z_hi + channel_clearance
    cyl = (cq.Workplane("XY").workplane(offset=gz_lo).center(0, wheel_cy)
           .circle(R).extrude(gz_hi - gz_lo))
    body = body.cut(cyl)
    if mirror:
        body = body.mirror("XZ")
    cq.exporters.export(body, str(path))
    bb = body.val().BoundingBox()
    print(f"  {path.stem}  bbox {bb.xlen:.0f}x{bb.ylen:.0f}x{bb.zlen:.0f} mm "
          f"(cradle Ø{wheel_dia:.0f}, {len(body.val().Solids())} solid) -> {path.name}")


def _dims(s: str):
    return [float(v) for v in s.lower().split("x")]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("out_dir")
    ap.add_argument("--damper-dia", type=float, default=63.0, help="damper diameter (mm)")
    ap.add_argument("--damper-h", type=float, default=25.0, help="damper height (mm)")
    ap.add_argument("--adapter", default="120x60x12", help="camera adapter LxWxH (mm)")
    ap.add_argument("--fingers", action="store_true",
                    help="also author the 2FG14 custom fingers (finger_left/right.step) that grip the wheel")
    ap.add_argument("--wheel-dia", type=float, default=125.0, help="wheel diameter the fingers cradle (mm)")
    ap.add_argument("--finger-cy", type=float, default=13.0, help="|finger mount Y offset| from tool axis (mm)")
    ap.add_argument("--finger-cz", type=float, default=65.0, help="wheel centre Z in the finger frame (mm)")
    ap.add_argument("--wheel-thick", type=float, default=20.0, help="wheel thickness (mm), for the cradle Z band")
    a = ap.parse_args()
    out = Path(a.out_dir).resolve()
    out.mkdir(parents=True, exist_ok=True)
    print(f"authoring placeholder STEP -> {out}")
    make_damper(out / "damper.step", a.damper_dia, a.damper_h)
    make_camera_adapter(out / "camera_adapter.step", *_dims(a.adapter))
    if a.fingers:
        z_lo, z_hi = a.finger_cz - a.wheel_thick / 2.0, a.finger_cz + a.wheel_thick / 2.0
        make_gripper_finger(out / "finger_left.step", False, a.wheel_dia, -a.finger_cy, z_lo, z_hi)
        make_gripper_finger(out / "finger_right.step", True, a.wheel_dia, -a.finger_cy, z_lo, z_hi)
    print("done.")
    return 0


raise SystemExit(main())
