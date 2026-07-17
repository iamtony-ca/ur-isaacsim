# SPDX-License-Identifier: Apache-2.0
"""Prepare CAD obstacles for the collision pipeline: convert each obstacle's CAD to
(1) a USD for the Isaac scene and (2) a LIGHTWEIGHT collision mesh OBJ for the MoveIt
planning scene. Driven by obstacles.yaml — same single-source pattern as EOAT.

Per obstacle with a `cad:` field:
  * STEP/IGES/JT (B-rep) -> HOOPS backend (convert_step_to_usd) -> USD
    STL/OBJ/DAE (mesh)   -> asset_converter (convert_dae_to_usd) -> USD
  * then extract the tessellated mesh from that USD (metres) and write:
      <id>_mesh.obj    (decimated full mesh)      when collision: mesh
      <id>_convex.obj  (single convex hull, light) when collision: convex
    box obstacles need no CAD/mesh (primitive is authored directly by the loaders).

The convex hull / decimation keeps the MoveIt (and Isaac) collider LIGHT — dense CAD
tessellation otherwise bogs down the sim (esp. under RL). Isaac itself still uses its
native convexDecomposition at scene-load; this OBJ is what MoveIt/FCL consumes.

Run (Isaac bundled python):
    /isaac-sim/python.sh prepare_obstacles.py [obstacles.yaml]
"""
from __future__ import annotations

import subprocess
import sys
from pathlib import Path

import numpy as np

from isaacsim import SimulationApp
_app = SimulationApp({"headless": True})

import yaml  # noqa: E402
from pxr import Usd, UsdGeom, Gf  # noqa: E402

HERE = Path(__file__).resolve().parent
COMMON = HERE.parent                                   # isaac/common
CAD_EXTS_BREP = (".step", ".stp", ".stpz", ".igs", ".iges", ".jt")
MAX_TRIS = 4000                                        # decimation cap for collision mesh


def info(m: str) -> None:
    print(f"[prep-obs] {m}", flush=True)


def cad_to_usd(src: Path, dst: Path, lod: int) -> bool:
    """Route B-rep -> HOOPS, mesh -> asset_converter, both via the existing scripts."""
    dst.parent.mkdir(parents=True, exist_ok=True)
    if src.suffix.lower() in CAD_EXTS_BREP:
        script = COMMON / "convert_step_to_usd.py"
        cmd = ["/isaac-sim/python.sh", str(script), str(src), str(dst), "--lod", str(lod)]
    else:                                              # mesh (.stl/.obj/.dae/.fbx)
        script = COMMON / "convert_dae_to_usd.py"
        cmd = ["/isaac-sim/python.sh", str(script), str(src), str(dst)]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if not dst.exists():
        info(f"  CONVERT FAILED {src.name}: {r.stderr.strip().splitlines()[-1] if r.stderr else '?'}")
        return False
    return True


def usd_world_mesh(usd: Path):
    """Gather all mesh triangles from a USD in its (metre) world frame -> (verts, tris)."""
    stage = Usd.Stage.Open(str(usd))
    all_v, all_f = [], []
    cache = UsdGeom.XformCache()
    # HOOPS/asset_converter USDs often store geometry under INSTANCEABLE prims, which
    # a plain Traverse() skips -> descend into instance proxies to reach the meshes.
    for prim in stage.Traverse(Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)):
        if not prim.IsA(UsdGeom.Mesh):
            continue
        m = UsdGeom.Mesh(prim)
        pts = m.GetPointsAttr().Get()
        counts = m.GetFaceVertexCountsAttr().Get()
        idxs = m.GetFaceVertexIndicesAttr().Get()
        if not pts or not counts or not idxs:
            continue
        xf = cache.GetLocalToWorldTransform(prim)
        base = len(all_v)
        for p in pts:
            wp = xf.Transform(Gf.Vec3d(p[0], p[1], p[2]))
            all_v.append([wp[0], wp[1], wp[2]])
        o = 0
        for c in counts:                               # fan-triangulate each face
            for k in range(1, c - 1):
                all_f.append([base + idxs[o], base + idxs[o + k], base + idxs[o + k + 1]])
            o += c
    return np.array(all_v, dtype=float), np.array(all_f, dtype=int)


def write_obj(path: Path, verts, tris) -> None:
    lines = [f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}" for v in verts]
    lines += [f"f {t[0]+1} {t[1]+1} {t[2]+1}" for t in tris]
    path.write_text("\n".join(lines) + "\n")


def convex_hull(verts):
    from scipy.spatial import ConvexHull
    h = ConvexHull(verts)
    # reindex hull vertices compactly
    used = sorted(set(int(i) for s in h.simplices for i in s))
    remap = {old: new for new, old in enumerate(used)}
    hv = verts[used]
    hf = [[remap[int(a)], remap[int(b)], remap[int(c)]] for a, b, c in h.simplices]
    return hv, hf


def convex_decomposition(verts, tris):
    """CoACD -> list of convex parts [(verts, tris), ...]. Follows concavity closely
    (unlike a single convex hull, which fills concave regions and is over-conservative)
    while staying light (few convex pieces). Falls back to a single hull if CoACD fails."""
    try:
        import coacd
        m = coacd.Mesh(np.asarray(verts, dtype=np.float64), np.asarray(tris, dtype=np.int32))
        parts = coacd.run_coacd(m)                 # [(vertices, faces), ...]
        return [(np.asarray(v), np.asarray(f)) for v, f in parts]
    except Exception as e:
        info(f"  CoACD failed ({e}); falling back to single convex hull")
        return [convex_hull(verts)]


def decimate(verts, tris, max_tris):
    if len(tris) <= max_tris:
        return verts, tris
    try:
        import trimesh
        mesh = trimesh.Trimesh(vertices=verts, faces=tris, process=True)
        dec = mesh.simplify_quadric_decimation(face_count=max_tris)   # kw (needs fast_simplification)
        return np.asarray(dec.vertices), np.asarray(dec.faces)
    except Exception as e:
        info(f"  decimation skipped ({e}); keeping {len(tris)} tris")
        return verts, tris


def main() -> int:
    cfg_path = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else HERE / "obstacles.yaml"
    cfg = yaml.safe_load(cfg_path.read_text())
    meta = cfg.get("meta", {})
    defaults = meta.get("defaults", {})
    lod = int(defaults.get("lod", 1))
    cad_dir = (cfg_path.parent / meta.get("cad_dir", "../../assets/cad/obstacles")).resolve()
    out_dir = (cfg_path.parent / meta.get("out_usd_dir", "../../assets/obstacles")).resolve()
    out_dir.mkdir(parents=True, exist_ok=True)

    for o in cfg.get("obstacles", []):
        oid = o["id"]
        coll = o.get("collision", defaults.get("collision", "convex"))
        if "cad" not in o:
            info(f"{oid}: primitive ({coll}) — no CAD, skip")
            continue
        src = cad_dir / o["cad"]
        if not src.exists():
            info(f"{oid}: CAD MISSING {src} — skip")
            continue
        usd = out_dir / f"{oid}.usd"
        if usd.exists() and usd.stat().st_mtime >= src.stat().st_mtime:
            info(f"{oid}: USD up-to-date ({usd.name}) — skip conversion")
        elif not cad_to_usd(src, usd, lod):
            continue
        verts, tris = usd_world_mesh(usd)
        info(f"{oid}: {src.name} -> {usd.name}  ({len(verts)} v, {len(tris)} f)")
        if coll == "box":
            continue                                   # primitive; loaders use size
        # Emit ALL THREE collision representations so the level can be switched
        # (per-obstacle `collision:` or the loader's --level) with NO re-prepare:
        #   mesh   = true surface, decimated (accurate, NOT inflated)
        #   convexDecomposition = CoACD convex parts merged (follows concavity, light)
        #   convex = single convex hull (lightest, but fills concavity -> conservative)
        dv, df = decimate(verts, tris, MAX_TRIS)
        write_obj(out_dir / f"{oid}_mesh.obj", dv, df)
        info(f"  mesh   -> {oid}_mesh.obj  ({len(dv)} v, {len(df)} f)")

        parts = convex_decomposition(verts, tris)
        mv, mf, off = [], [], 0                        # merge parts into one triangle soup
        for pv, pf in parts:
            mv.extend(pv.tolist())
            mf.extend([[a + off, b + off, c + off] for a, b, c in pf])
            off += len(pv)
        write_obj(out_dir / f"{oid}_convexDecomposition.obj", np.array(mv), mf)
        info(f"  convexDecomp -> {oid}_convexDecomposition.obj  "
             f"({len(parts)} parts, {len(mv)} v, {len(mf)} f)")

        hv, hf = convex_hull(verts)
        write_obj(out_dir / f"{oid}_convex.obj", hv, hf)
        info(f"  convex -> {oid}_convex.obj  ({len(hv)} v, {len(hf)} f)")
    _app.close()
    return 0


sys.exit(main())
