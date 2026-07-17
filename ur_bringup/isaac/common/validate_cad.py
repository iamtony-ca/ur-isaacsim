# SPDX-License-Identifier: Apache-2.0
"""Single-shot CAD ACCEPTANCE checker — run right after convert_step_to_usd.py to catch
the common ways a delivered CAD file differs from what the pipeline assumes, WITHOUT
having to eyeball the converter log. Validates the converted USD (metre-native, Z-up),
so it works for both EOAT parts and static obstacles.

Per USD it reports + PASS/WARN-verdicts:
  * scale        REAL bbox size (mm); vs expected_size_mm if given -> unit-error hint
                 (ratio ~1000 = m<->mm, ~25.4 = inch, ~10 = cm) — the #1 trap.
  * geometry     mesh(body) count + points/triangles — a surprise body count flags an
                 assembly delivered as a "part", or stray reference/FOV geometry in bbox.
  * watertight   welds coincident tessellation verts, then counts boundary edges
                 (used by 1 triangle) and non-manifold edges (used by >2). 0/0 = closed
                 manifold solid. Boundary>0 = open shell (bad STL / surface body).
  * density      triangle count vs a soft cap (too dense -> sim bog); degenerate/zero-
                 area triangles (bad tessellation).
  * origin       bbox min-Z: for the passthrough mounting-face convention it should be ~0.

Exit code: 1 if any file is unreadable/empty (hard error), else 0 (WARNs are advisory).

Run (Isaac bundled python):
    /isaac-sim/python.sh validate_cad.py <file_or_dir.usd> [more ...] [opts]
    opts:
      --expect X,Y,Z      expected bbox size in mm (single-file scale check)
      --config <yaml>     eoat_*.yaml / obstacles.yaml; pulls per-part expected_size_mm
                          (parts.<name>.expected_size_mm or obstacles[].expected_size_mm),
                          matched by the part's `cad` filename stem.
      --tol-pct P         scale tolerance percent (default 5)
      --max-tris N        density warn cap (default 200000)
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

_ap = argparse.ArgumentParser()
_ap.add_argument("srcs", nargs="+", help="USD file(s) or directory(ies)")
_ap.add_argument("--expect", default=None, help="expected size mm as X,Y,Z (single file)")
_ap.add_argument("--config", default=None, help="yaml with per-part expected_size_mm + cad")
_ap.add_argument("--tol-pct", type=float, default=5.0)
_ap.add_argument("--max-tris", type=int, default=200000)
_args = _ap.parse_args()

from isaacsim import SimulationApp  # noqa: E402

_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom  # noqa: E402

TOL_WELD_M = 1e-6                     # 1 micron: weld coincident tessellation verts
UNIT_HINTS = {1000.0: "m->mm (metre file read as mm, or vice-versa)",
              0.001: "mm->m", 25.4: "inch->mm", 0.03937: "mm->inch",
              10.0: "cm->mm", 0.1: "mm->cm"}


def _expected_map(cfg_path: Path) -> dict:
    """{cad_stem: (label, [x,y,z]mm)} from an eoat/obstacles yaml, for parts that
    declare expected_size_mm. Matched against the USD filename stem downstream."""
    import yaml
    cfg = yaml.safe_load(cfg_path.read_text())
    out = {}
    items = []
    parts = cfg.get("parts", {})
    if isinstance(parts, dict):
        items += [(n, s) for n, s in parts.items()]
    items += [(o.get("id", "?"), o) for o in cfg.get("obstacles", [])]
    for name, spec in items:
        exp = spec.get("expected_size_mm")
        cad = spec.get("cad")
        if exp and cad:
            out[Path(cad).stem] = (name, [float(v) for v in exp])
    return out


def _gather(usd_path: Path):
    stage = Usd.Stage.Open(str(usd_path))
    if stage is None:
        return None
    mpu = float(UsdGeom.GetStageMetersPerUnit(stage))
    up = str(UsdGeom.GetStageUpAxis(stage))
    cache = UsdGeom.XformCache()
    V, F, nmesh = [], [], 0
    # HOOPS/asset_converter geometry often lives under INSTANCEABLE prims — descend
    # into instance proxies or a plain Traverse() misses every mesh.
    for prim in stage.Traverse(Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(prim)
        pts = m.GetPointsAttr().Get()
        counts = m.GetFaceVertexCountsAttr().Get()
        idx = m.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idx:
            continue
        nmesh += 1
        M = np.array(cache.GetLocalToWorldTransform(prim), dtype=float)  # row-vector USD
        p = np.asarray(pts, dtype=float)
        world = (p @ M[:3, :3] + M[3, :3]) * mpu                        # -> metres
        base = len(V)
        V.append(world)
        counts = np.asarray(counts, dtype=int)
        idx = np.asarray(idx, dtype=int)
        o = 0
        for c in counts:                                               # fan-triangulate
            for k in range(1, c - 1):
                F.append([base + idx[o], base + idx[o + k], base + idx[o + k + 1]])
            o += c
    return dict(mpu=mpu, up=up, nmesh=nmesh,
                V=np.concatenate(V) if V else np.zeros((0, 3)),
                F=np.asarray(F, dtype=int) if F else np.zeros((0, 3), int))


def _analyze(d: dict) -> dict:
    V, F = d["V"], d["F"]
    r = dict(nmesh=d["nmesh"], mpu=d["mpu"], up=d["up"], npts=len(V), ntris=len(F))
    if len(V) == 0 or len(F) == 0:
        r["empty"] = True
        return r
    mn, mx = V.min(0), V.max(0)
    r["size_mm"] = (mx - mn) * 1000.0
    r["min_mm"] = mn * 1000.0
    # weld coincident verts (CAD tessellation duplicates verts per face -> every shared
    # edge would otherwise look like a boundary). Round to TOL_WELD_M, unique the rows.
    key = np.round(V / TOL_WELD_M).astype(np.int64)
    _, inv = np.unique(key, axis=0, return_inverse=True)
    inv = inv.reshape(-1)
    r["nverts_welded"] = int(inv.max() + 1)
    Fw = inv[F]
    deg = (Fw[:, 0] == Fw[:, 1]) | (Fw[:, 1] == Fw[:, 2]) | (Fw[:, 0] == Fw[:, 2])
    r["degenerate"] = int(deg.sum())
    Fg = Fw[~deg]
    a, b, c = V[F[~deg, 0]], V[F[~deg, 1]], V[F[~deg, 2]]
    area = 0.5 * np.linalg.norm(np.cross(b - a, c - a), axis=1)
    r["zero_area"] = int((area < 1e-12).sum())
    E = np.sort(np.concatenate([Fg[:, [0, 1]], Fg[:, [1, 2]], Fg[:, [2, 0]]]), axis=1)
    _, cnt = np.unique(E, axis=0, return_counts=True)
    r["boundary_edges"] = int((cnt == 1).sum())
    r["nonmanifold_edges"] = int((cnt > 2).sum())
    return r


def _scale_hint(ratio: float) -> str:
    for k, msg in UNIT_HINTS.items():
        if abs(ratio - k) / k < 0.05:
            return f"  <- looks like a UNIT error: {msg}"
    return ""


def _verdict(name: str, r: dict, expect, tol_pct: float, max_tris: int) -> bool:
    """Print the per-file report. Return True if any WARN was raised."""
    warn = False

    def line(tag, msg):
        nonlocal warn
        if tag == "WARN":
            warn = True
        print(f"    [{tag}] {msg}")

    print(f"\n=== {name} ===")
    if r.get("empty"):
        print("    [ERROR] no mesh geometry (unreadable / surface-only / convert failed)")
        return True
    sz = r["size_mm"]
    print(f"    size = ({sz[0]:.1f}, {sz[1]:.1f}, {sz[2]:.1f}) mm   "
          f"bodies={r['nmesh']}  tris={r['ntris']}  verts={r['npts']}(welded {r['nverts_welded']})")

    # 1) scale
    if expect is not None:
        exp = np.asarray(expect, float)
        big = np.maximum(sz, exp)
        ratios = np.where(exp > 1e-6, sz / np.where(exp > 1e-6, exp, 1.0), 1.0)
        errpct = 100.0 * np.abs(sz - exp) / np.maximum(big, 1e-6)
        if (errpct <= tol_pct).all():
            line("PASS", f"scale within {tol_pct:.0f}% of expected {tuple(exp)} mm")
        else:
            worst = float(np.max(ratios))
            line("WARN", f"scale off vs expected {tuple(exp)} mm "
                         f"(per-axis %err={tuple(round(float(e),1) for e in errpct)})"
                         + _scale_hint(worst))
    else:
        line("INFO", "no expected_size_mm -> compare REAL size above to the drawing by hand")
    if abs(r["mpu"] - 1.0) > 1e-9:
        line("WARN", f"metersPerUnit={r['mpu']} != 1.0 -> won't drop 1:1 into the metre scene "
                     f"(re-run convert so output is metre-native)")

    # 2) geometry / assembly
    if r["nmesh"] == 0:
        line("WARN", "0 mesh bodies")
    elif r["nmesh"] > 1:
        line("INFO", f"{r['nmesh']} mesh bodies — expected for an assembly; for a SINGLE part "
                     f"this may be stray geometry (reference planes / FOV / multi-solid) inflating bbox")

    # 3) watertight / manifold. Reliable only for a SINGLE body: merging many solids into
    # one triangle soup makes touching faces read as non-manifold, so for nmesh>1 this is
    # expected, not a defect -> report as INFO. Boundary edges (open shell) stay meaningful.
    b, nm = r["boundary_edges"], r["nonmanifold_edges"]
    if b == 0 and nm == 0:
        line("PASS", "watertight closed manifold (good solid)")
    elif r["nmesh"] > 1:
        tag = "WARN" if b > 0 else "INFO"
        line(tag, f"{b} boundary + {nm} non-manifold edge(s) across {r['nmesh']} merged bodies — "
                  f"non-manifold is expected where solids touch; {b} boundary edge(s) = "
                  f"{'OPEN shell/surface body present (check)' if b > 0 else 'none, good'}. "
                  f"convex/convexDecomp collision unaffected")
    else:
        line("WARN", f"NOT watertight: {b} boundary + {nm} non-manifold edge(s) — open shell / "
                     f"surface body / bad STL. Mesh collision may leak; convex/convexDecomp still ok")

    # 4) density / tessellation quality
    if r["ntris"] > max_tris:
        line("WARN", f"{r['ntris']} tris > {max_tris} cap — dense; will bog sim. Lower --lod (STEP) "
                     f"or decimate. (obstacles/EOAT exporters decimate, but check the source)")
    else:
        line("PASS", f"{r['ntris']} tris within density cap")
    if r["degenerate"] or r["zero_area"]:
        line("WARN", f"{r['degenerate']} degenerate + {r['zero_area']} zero-area triangle(s) "
                     f"(bad tessellation; may break convex hull / CoACD)")

    # 5) origin convention (passthrough)
    minz = r["min_mm"][2]
    if abs(minz) <= 0.5:
        line("PASS", f"bbox min-Z = {minz:.2f} mm ~ 0 (mounting face at origin — passthrough-ready)")
    else:
        line("INFO", f"bbox min-Z = {minz:.1f} mm != 0 — fine if origin isn't the mounting face "
                     f"(use rpy_deg/seat) ; for passthrough the mounting face should be at z=0")
    return warn


def main() -> int:
    expect_cli = None
    if _args.expect:
        expect_cli = [float(v) for v in _args.expect.replace(" ", "").split(",")]
    exp_map = _expected_map(Path(_args.config).resolve()) if _args.config else {}

    usds = []
    for s in _args.srcs:
        p = Path(s).resolve()
        if p.is_dir():
            usds += sorted(p.glob("*.usd"))
        elif p.exists():
            usds.append(p)
        else:
            print(f"skip (not found): {p}")
    if not usds:
        print("no USD inputs")
        return 1

    single = len(usds) == 1
    n_err = n_warn = 0
    for u in usds:
        d = _gather(u)
        if d is None:
            print(f"\n=== {u.name} ===\n    [ERROR] could not open stage")
            n_err += 1
            continue
        r = _analyze(d)
        exp = None
        if u.stem in exp_map:
            exp = exp_map[u.stem][1]
        elif single and expect_cli:
            exp = expect_cli
        if r.get("empty"):
            n_err += 1
        warn = _verdict(u.name, r, exp, _args.tol_pct, _args.max_tris)
        n_warn += 1 if warn else 0

    print(f"\n==== checked {len(usds)} file(s): {n_err} error, {n_warn} with warning(s) ====")
    if n_err:
        print("     ERROR = unreadable/empty (surface-only or failed convert) — must fix before use.")
    if n_warn:
        print("     WARN  = review in Isaac GUI / confirm with the mechanical team.")
    return 1 if n_err else 0


rc = main()
_app.close()
sys.exit(rc)
