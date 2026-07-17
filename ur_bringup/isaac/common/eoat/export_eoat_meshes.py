# SPDX-License-Identifier: Apache-2.0
"""Export each EOAT part's NORMALIZED mesh to OBJ so the URDF/RViz shows the REAL part
geometry (matching the Isaac USD) instead of a bounding box. Without this the URDF
emitter falls back to box primitives and RViz looks different from Isaac (함정 #7 gap).

Per graph link with a real `geom` (normalized USD stem):
    meshes/eoat/<id>.obj       full (decimated) mesh -> URDF <visual>   (always the REAL shape)
    meshes/eoat/<id>_col.obj   collision proxy       -> URDF <collision>
The collision proxy fidelity is chosen PER PART by `physics.collision` (the SAME single-source
knob the USD emitter reads), mirroring the obstacle pipeline's three levels:
    convex (=convexHull, DEFAULT) -> single convex hull       (lightest; fills concavities)
    convexDecomposition           -> CoACD convex parts merged (follows concavity, still light)
    mesh   (=meshSimplification)  -> decimated real surface    (exact; grasp/insertion contact)
Links with no CAD (placeholder fingers) get no OBJ -> the URDF keeps a box for them.
(convexDecomposition needs `pip install coacd`; falls back to a single hull if unavailable.)
The mesh is in the link (normalized) frame, so the URDF references it at identity —
same as build_eoat_usd references the normalized USD as the link's `geo` child.

Run (Isaac bundled python):
    /isaac-sim/python.sh export_eoat_meshes.py eoat_dualtool.yaml
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

from isaacsim import SimulationApp
_app = SimulationApp({"headless": True})

from pxr import Usd, UsdGeom, Gf  # noqa: E402

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
from eoat_model import load_graph  # noqa: E402


def usd_world_mesh(usd: Path):
    stage = Usd.Stage.Open(str(usd))
    V, F = [], []
    cache = UsdGeom.XformCache()
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
        base = len(V)
        for p in pts:
            wp = xf.Transform(Gf.Vec3d(p[0], p[1], p[2]))
            V.append([wp[0], wp[1], wp[2]])
        o = 0
        for c in counts:
            for k in range(1, c - 1):
                F.append([base + idxs[o], base + idxs[o + k], base + idxs[o + k + 1]])
            o += c
    return np.array(V, dtype=float), np.array(F, dtype=int)


def write_obj(path: Path, V, F):
    lines = [f"v {v[0]:.6f} {v[1]:.6f} {v[2]:.6f}" for v in V]
    lines += [f"f {t[0]+1} {t[1]+1} {t[2]+1}" for t in F]
    path.write_text("\n".join(lines) + "\n")


def convex_hull(V):
    from scipy.spatial import ConvexHull
    h = ConvexHull(V)
    used = sorted({int(i) for s in h.simplices for i in s})
    remap = {o: n for n, o in enumerate(used)}
    return V[used], [[remap[int(a)], remap[int(b)], remap[int(c)]] for a, b, c in h.simplices]


def convex_decomposition(V, F):
    """CoACD -> convex parts merged into one triangle soup (follows concavity, unlike a
    single hull, while staying light). Falls back to a single convex hull if CoACD missing."""
    try:
        import coacd
        m = coacd.Mesh(np.asarray(V, dtype=np.float64), np.asarray(F, dtype=np.int32))
        parts = coacd.run_coacd(m)                       # [(vertices, faces), ...]
        mv, mf, off = [], [], 0
        for pv, pf in parts:
            mv.extend(np.asarray(pv).tolist())
            mf.extend([[a + off, b + off, c + off] for a, b, c in np.asarray(pf)])
            off += len(pv)
        return np.asarray(mv, dtype=float), mf
    except Exception as e:
        print(f"  convexDecomposition unavailable ({e}); using single convex hull")
        return convex_hull(V)


VIS_MAX_TRIS = 20000        # decimate the VISUAL mesh (RViz render only) to keep OBJs small
COL_MAX_TRIS = 10000        # decimate the 'mesh' collision proxy (still needs to be light for FCL)


def decimate(V, F, max_tris):
    if len(F) <= max_tris:
        return V, F
    try:
        import trimesh
        m = trimesh.Trimesh(vertices=V, faces=F, process=True)
        d = m.simplify_quadric_decimation(face_count=max_tris)   # kw: signature is (percent, face_count, ...)
        return np.asarray(d.vertices), np.asarray(d.faces)
    except Exception as e:
        print(f"  decimation skipped ({e}); keeping {len(F)} tris")
        return V, F


def main() -> int:
    cfg = Path(sys.argv[1]).resolve() if len(sys.argv) > 1 else HERE / "eoat_dualtool.yaml"
    g = load_graph(cfg)
    # meshes/eoat under ur_bringup (installed via symlink -> package:// resolves)
    out = (HERE.parents[2] / "meshes" / "eoat")     # isaac/common/eoat -> ur_bringup/meshes/eoat
    out.mkdir(parents=True, exist_ok=True)
    n = 0
    for l in g.links:
        if not l.geom or "#" in str(l.geom):
            print(f"[export-mesh] {l.id}: no CAD mesh (box fallback in URDF)")
            continue
        usd = g.norm_dir / f"{l.geom}.usd"
        if not usd.exists():
            print(f"[export-mesh] {l.id}: {usd} missing — skip")
            continue
        V, F = usd_world_mesh(usd)
        if len(V) == 0:
            print(f"[export-mesh] {l.id}: 0 verts extracted — skip")
            continue
        dv, df = decimate(V, F, VIS_MAX_TRIS)          # visual: real shape, capped tri count
        write_obj(out / f"{l.id}.obj", dv, df)
        # collision proxy: fidelity per part from physics.collision (same knob as USD).
        # DEFAULT convexHull -> convex hull (current behavior). See eoat_model._COLL_ALIAS.
        approx = l.physics.collision if l.physics else "convexHull"
        if approx == "convexDecomposition":
            cv, cf = convex_decomposition(V, F)         # CoACD parts merged (concave-aware)
        elif approx == "meshSimplification":            # 'mesh': exact (decimated) real surface
            cv, cf = decimate(V, F, COL_MAX_TRIS)
        else:                                           # convexHull / boundingCube / none -> hull
            cv, cf = convex_hull(V)
        write_obj(out / f"{l.id}_col.obj", cv, cf)
        print(f"[export-mesh] {l.id}: {usd.name} -> {l.id}.obj ({len(df)}f, from {len(F)}) "
              f"+ {l.id}_col.obj [{approx}] ({len(cf)}f)")
        n += 1
    print(f"[export-mesh] exported {n} part mesh(es) -> {out}")
    _app.close()
    return 0


sys.exit(main())
