#!/usr/bin/env python3
# SPDX-License-Identifier: Apache-2.0
"""PRE-convert acceptance check for delivered STEP files — reads the RAW .step text
(pure python, NO Isaac) to verify the things CAD_DELIVERY_REQUEST.md asks for on the
SOURCE side, which validate_cad.py (which runs on the CONVERTED USD) cannot see:

  * format     extension is STEP; not a faceted/tessellated STEP (must be exact B-rep)
  * AP schema  AP242 (best) / AP214 (ok) / AP203 (old) / unknown          [req §2]
  * units      length unit = mm (else scale error later)                  [req §2]
  * bodies     # of solid B-reps in the file (1 = ideal, many = fragmented) [req §2]
  * solidity   has real solids (not surface/sheet bodies only)            [req §2/§7]
  * manifest   (optional) all expected files delivered, none missing      [req §1/§4]

This is the FIRST gate: run it on the raw STEP, then convert, then run validate_cad.py
on the USD for the geometry side (scale/bbox, watertight, density, origin). The two
together cover the automatable half of the request; the rest (axis DIRECTION, clocking,
finger open/close state, defeaturing, the physics table) needs the GUI / a data sheet.

Usage (any python3 — no Isaac, no deps beyond stdlib; --config needs pyyaml):
    python3 check_step_delivery.py <file_or_dir> [more ...] [opts]
      --expect A.STEP,B.STEP   expected filenames (manifest check; reports missing/extra)
      --max-bodies N           warn if a file has > N solid bodies (default 50)
"""
from __future__ import annotations

import argparse
import re
import sys
from pathlib import Path

STEP_EXTS = (".step", ".stp", ".stpz")

_ap = argparse.ArgumentParser()
_ap.add_argument("srcs", nargs="+", help="STEP file(s) or directory(ies)")
_ap.add_argument("--expect", default=None, help="comma-separated expected filenames (manifest)")
_ap.add_argument("--max-bodies", type=int, default=50, help="warn above this solid-body count")
_args = _ap.parse_args()

_UNIT_PREFIX = {"MILLI": "mm", "CENTI": "cm", "MICRO": "um", "KILO": "km", "$": "m", "": "m"}


def _read(path: Path) -> str:
    # STEP is ASCII; decode leniently (some files carry stray high bytes in comments).
    return path.read_bytes().decode("latin-1", errors="ignore")


def ap_schema(text: str) -> tuple[str, str]:
    """(label, verdict) from the HEADER's FILE_SCHEMA/FILE_DESCRIPTION."""
    head = text[: text.upper().find("ENDSEC")] if "ENDSEC" in text.upper() else text[:8000]
    U = head.upper()
    if "AP242" in U or "10303 242" in U or "MANAGED_MODEL_BASED" in U:
        return "AP242", "best"
    if "AUTOMOTIVE_DESIGN" in U or "10303 214" in U or "AP214" in U:
        return "AP214", "ok"
    if "CONFIG_CONTROL_DESIGN" in U or "10303 203" in U or "AP203" in U:
        return "AP203", "old"
    return "unknown", "unknown"


def length_unit(text: str) -> str | None:
    """Length unit token -> 'mm'/'m'/'cm'/'inch'/... (angle units use RADIAN, so filter by METRE)."""
    m = re.search(r"SI_UNIT\s*\(\s*(\$|\.\w+\.)\s*,\s*\.METRE\.", text, re.I)
    if m:
        tok = m.group(1).strip(".").upper()
        return _UNIT_PREFIX.get(tok, tok.lower() or "m")
    if re.search(r"CONVERSION_BASED_UNIT\s*\(\s*'INCH'", text, re.I) or re.search(r"'INCH'", text, re.I):
        return "inch"
    return None


def solid_count(text: str) -> int:
    return (len(re.findall(r"MANIFOLD_SOLID_BREP\b", text, re.I))
            + len(re.findall(r"BREP_WITH_VOIDS\b", text, re.I)))


def is_faceted(text: str) -> bool:
    return bool(re.search(r"TESSELLATED|TRIANGULATED", text, re.I))


def has_surface(text: str) -> bool:
    return bool(re.search(r"SHELL_BASED_SURFACE_MODEL|MANIFOLD_SURFACE_SHAPE|OPEN_SHELL", text, re.I))


def check_one(path: Path, max_bodies: int) -> tuple[bool, bool]:
    """Print the per-file report. Return (had_error, had_warn)."""
    err = warn = False
    print(f"\n=== {path.name} ===")

    if path.suffix.lower() not in STEP_EXTS:
        print(f"    [ERROR] not a STEP file (ext {path.suffix!r}) — export to STEP (.step)")
        return True, False

    text = _read(path)
    ap, apv = ap_schema(text)
    unit = length_unit(text)
    nb = solid_count(text)
    faceted = is_faceted(text)
    surf = has_surface(text)
    size_mb = path.stat().st_size / 1e6

    print(f"    schema={ap}  units={unit or '?'}  solids={nb}  size={size_mb:.1f}MB")

    # 1) AP schema
    if apv == "best":
        print("    [PASS] AP242 (best assembly/PMI fidelity)")
    elif apv == "ok":
        print(f"    [PASS] {ap} — acceptable (AP242 preferred if the CAD tool offers it)")
    elif apv == "old":
        print(f"    [WARN] {ap} is old — request AP242/AP214 export"); warn = True
    else:
        print("    [WARN] STEP schema not recognized — confirm AP242/AP214"); warn = True

    # 2) units
    if unit == "mm":
        print("    [PASS] units = mm")
    elif unit is None:
        print("    [WARN] length unit not found in header — confirm mm"); warn = True
    else:
        print(f"    [WARN] units = {unit} (expected mm) — will mis-scale on convert; request mm re-export"); warn = True

    # 3) B-rep vs faceted / surface-only
    if faceted:
        print("    [WARN] contains TESSELLATED/TRIANGULATED data — faceted STEP, not exact B-rep; request solid B-rep"); warn = True
    if nb == 0:
        if surf:
            print("    [ERROR] no solid B-rep — surface/sheet bodies only; request closed SOLIDS"); err = True
        else:
            print("    [ERROR] no solid B-rep found — empty or unsupported; check the file"); err = True
    elif nb == 1:
        print("    [PASS] 1 solid body (ideal — single merged solid)")
    elif nb <= max_bodies:
        print(f"    [INFO] {nb} solid bodies (assembly/multi-body) — fine; merging toward fewer lowers load cost")
    else:
        print(f"    [WARN] {nb} solid bodies > {max_bodies} — heavily fragmented; ask to merge (no accuracy loss)"); warn = True

    return err, warn


def main() -> int:
    files: list[Path] = []
    for s in _args.srcs:
        p = Path(s).resolve()
        if p.is_dir():
            files += sorted(q for q in p.iterdir() if q.suffix.lower() in STEP_EXTS)
        elif p.exists():
            files.append(p)
        else:
            print(f"skip (not found): {p}")
    if not files:
        print("no STEP inputs found")
        return 1

    n_err = n_warn = 0
    for f in files:
        e, w = check_one(f, _args.max_bodies)
        n_err += 1 if e else 0
        n_warn += 1 if w else 0

    # manifest: were all expected files delivered?
    if _args.expect:
        want = {x.strip() for x in _args.expect.split(",") if x.strip()}
        got = {f.name for f in files}
        got_lower = {g.lower() for g in got}
        missing = [w for w in want if w.lower() not in got_lower]
        extra = [g for g in got if g.lower() not in {w.lower() for w in want}]
        print("\n=== manifest ===")
        if missing:
            print(f"    [ERROR] MISSING expected file(s): {missing}"); n_err += 1
        else:
            print("    [PASS] all expected files present")
        if extra:
            print(f"    [INFO] extra file(s) not in the expected list: {extra}")

    print(f"\n==== checked {len(files)} STEP file(s): {n_err} error, {n_warn} with warning(s) ====")
    print("     Next: convert (convert_step_to_usd.py) -> validate_cad.py for scale/watertight/density/origin.")
    print("     NOT checkable here (needs GUI / datasheet): axis DIRECTION, clocking, finger open-close, defeaturing, physics table.")
    return 1 if n_err else 0


if __name__ == "__main__":
    sys.exit(main())
