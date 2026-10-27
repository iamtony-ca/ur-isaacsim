#!/usr/bin/env python3
"""Choose a UR16e base pose for a fixed set of TCP targets in a confined cell.

    source /opt/ros/jazzy/setup.bash            # xacro + ur_description
    python3 place_base.py examples/confined_cell.yaml --out out/confined

Pipeline (details: README.md)
  1. grid over each mount candidate in the YAML (floor/pedestal heights, ceiling,
     walls) on (u, v) and -- only if J1 is narrower than +-360 deg -- yaw
  2. hard constraints: footprint, IK within the joint limits for every target,
     ONE common IK branch, sphere-model collision vs. cell + self at every target
  3. soft score: manipulability, UR singularity margins, joint-limit margin,
     velocity-limited cycle time, clearance, straight joint-move collision sampling
  4. local refinement: a finer grid around the best spots
  5. robustness: base perturbed by the installation tolerance; same branch must hold
Outputs: results.json, report.md, top_candidates.csv, grid_feasible.csv (+ PNG with --plot)

CPU: single process and ONE BLAS thread by default (this machine shares its CPU with
other containers). `--workers N` spreads the grid, the refinement and the robustness
checks over N processes.
"""
import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS", "NUMEXPR_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import argparse                      # noqa: E402
import csv                           # noqa: E402
import json                          # noqa: E402
import time                          # noqa: E402
import warnings                      # noqa: E402

import numpy as np                   # noqa: E402

import cell as cellmod               # noqa: E402
import evaluator                     # noqa: E402
import ur16e_kin as kin              # noqa: E402

warnings.filterwarnings("ignore", "All-NaN slice")
HERE = os.path.dirname(os.path.abspath(__file__))

_P = None          # per-process Problem (multiprocessing initializer)


def _init(cell_path, cache_dir):
    global _P
    _P = evaluator.Problem(cellmod.Cell(cell_path), cache_dir)


def _eval_job(job):
    mi, U, V, Y, with_path = job
    m = _P.cell.mounts[mi]
    T = _P.cell.mount_pose(m, U, V, Y)
    r = _P.evaluate(m, T, with_path=with_path)
    r.pop("Q")
    return r


def _run(pool, jobs):
    if pool is None:
        return [_eval_job(j) for j in jobs]
    return pool.map(_eval_job, jobs)


def _cat(parts):
    out = {}
    for k in parts[0]:
        if k == "n_path_samples":
            out[k] = int(sum(p[k] for p in parts))
        else:
            out[k] = np.concatenate([p[k] for p in parts])
    return out


def yaw_values(P, yaw_step):
    """J1 at +-360 deg makes base yaw redundant: rotating the base about its own z is
    the same as offsetting q1 and the base housing is round, so feasibility, FK,
    collisions and travel are unchanged. Only the joint-limit-margin term can move
    (q1 sits closer to or further from +-360); with the alias centring in
    unwrap_cycle that spread is small (measured <= 0.006 in score; 0.029 before the
    centring was added). Yaw then matters for the cable exit -- choose it on site.
    Only a narrowed J1 (cable / safety config) makes yaw a search variable."""
    j1_full = (P.upper[0] - P.lower[0]) >= 4 * np.pi - 1e-6      # +-360 deg
    if yaw_step in (None, "auto"):
        return (np.array([0.0]), True) if j1_full else (np.radians(np.arange(0, 360, 30.0)), False)
    yaw_step = float(yaw_step)
    if yaw_step <= 0:
        return np.array([0.0]), j1_full
    return np.radians(np.arange(0, 360, yaw_step)), False


def _axis(lo, hi, step):
    """Grid points from lo with spacing `step`, plus hi when the step does not land on it
    (np.arange alone dropped the high end; with a step larger than the range it tested a
    single corner and could miss a feasible base)."""
    a = np.arange(lo, hi + 1e-9, step)
    return a if hi - a[-1] <= 1e-6 else np.append(a, hi)


def grid_search(P, pool, step, yaws, with_path, chunk, sel):
    """sel: indices into P.cell.mounts (the full YAML list) to evaluate."""
    res = {}
    for mi in sel:
        m = P.cell.mounts[mi]
        lo_u, hi_u = map(float, m["u"])
        lo_v, hi_v = map(float, m["v"])
        if step > hi_u - lo_u > 0 or step > hi_v - lo_v > 0:
            print(f"  note: mount {m['name']}: grid {step} m is larger than its u/v range -- sampling the ends only")
        us, vs = _axis(lo_u, hi_u, step), _axis(lo_v, hi_v, step)
        U, V, Y = [a.ravel() for a in np.meshgrid(us, vs, yaws, indexing="ij")]
        jobs = [(mi, U[a:a + chunk], V[a:a + chunk], Y[a:a + chunk], with_path) for a in range(0, len(U), chunk)]
        t0 = time.time()
        r = _cat(_run(pool, jobs))
        r.update(U=U, V=V, Y=Y, us=us, vs=vs, ys=yaws, runtime_s=time.time() - t0, mount_index=mi)
        res[m["name"]] = r
        best = np.max(np.where(r["feasible"], r["score"], -1))
        print(f"  grid {m['name']:14s} {len(U):6d} cand  feasible {int(r['feasible'].sum()):6d}  "
              f"best {best:.3f}  {r['runtime_s']:.1f}s", flush=True)
    return res


def refine(P, pool, grid, step, n_seeds, with_path, chunk):
    """Finer grid (step/5, radius = step) around the best distinct spots per mount.
    Seeds closer than 2*step (ties included) count as one spot. All seeds of a mount go
    out as one job list so --workers > 1 spreads them."""
    out = []
    for name, g in grid.items():
        if not g["feasible"].any():
            continue
        mi = g["mount_index"]
        o = np.argsort(-np.where(g["feasible"], g["score"], -9))
        seeds = []
        for i in o:
            if not g["feasible"][i] or len(seeds) >= n_seeds:
                break
            if all(np.hypot(g["U"][i] - g["U"][j], g["V"][i] - g["V"][j]) > 2 * step + 1e-6 for j in seeds):
                seeds.append(i)
        m = P.cell.mounts[mi]
        f = np.linspace(-step, step, 11)
        du, dv = [a.ravel() for a in np.meshgrid(f, f, indexing="ij")]
        U = np.concatenate([np.clip(g["U"][i] + du, *map(float, m["u"])) for i in seeds])
        V = np.concatenate([np.clip(g["V"][i] + dv, *map(float, m["v"])) for i in seeds])
        Y = np.concatenate([np.full(len(du), g["Y"][i]) for i in seeds])
        r = _cat(_run(pool, [(mi, U[a:a + chunk], V[a:a + chunk], Y[a:a + chunk], with_path)
                             for a in range(0, len(U), chunk)]))
        for k in np.where(r["feasible"])[0]:
            out.append(dict(mount=name, u=round(float(U[k]), 6), v=round(float(V[k]), 6),
                            yaw=round(float(Y[k]), 9), score=float(r["score"][k]), src="refined"))
    return out


def _robust_job(job):
    mi, u, v, yaw, branch, rb, with_path = job
    return robustness(_P, _P.cell.mounts[mi], u, v, yaw, branch, rb, with_path)


def robustness(P, mount, u, v, yaw, branch, rb, with_path=True):
    """Same-branch feasibility / score under the installation tolerance.

    with_path follows --no-path: the hard-constraint fractions do not depend on it,
    only robust_score does (it then carries the same optimistic path term as score)."""
    dxy, dyaw, n_rand = float(rb.get("dxy", 0.02)), np.radians(float(rb.get("dyaw_deg", 3.0))), int(rb.get("n_random", 200))
    g = np.linspace(-dxy, dxy, 5)
    du, dv, dy = [a.ravel() for a in np.meshgrid(g, g, np.array([-dyaw, 0, dyaw]), indexing="ij")]
    TA = P.cell.mount_pose(mount, u + du, v + dv, yaw + dy)
    T0 = P.cell.mount_pose(mount, np.array([u]), np.array([v]), np.array([yaw]))[0]
    rng = np.random.default_rng(0)
    TB = np.repeat(T0[None], n_rand, 0)
    dR = np.array([kin.rpy_to_R(*a) for a in rng.uniform(-dyaw, dyaw, (n_rand, 3))])
    TB[:, :3, :3] = TB[:, :3, :3] @ dR
    TB[:, :3, 3] += rng.uniform(-dxy, dxy, (n_rand, 3))
    out = {}
    # random_6dof only feeds the feasibility fractions (hard constraints do not depend on
    # the straight-move check), so its 200 samples skip path sampling -- that was ~70 %
    # of a default run. in_plane feeds robust_score and keeps the path term.
    for lab, T, wp in (("in_plane", TA, with_path), ("random_6dof", TB, False)):
        r = P.evaluate(mount, T, with_path=wp)
        sb = r["branch_scores"][:, branch]
        same = np.isfinite(sb)
        out[lab] = dict(n=int(len(T)), frac_same_branch_feasible=float(same.mean()),
                        frac_any_branch_feasible=float(r["feasible"].mean()),
                        score_same_branch_min=float(sb[same].min()) if same.any() else None,
                        score_same_branch_mean=float(np.where(same, sb, 0).mean()))
    return out


def distinct(lst, n, step):
    """First n entries of the score-sorted list `lst` that are not the same spot as an
    earlier one. Spots one grid step apart or closer (max(4 cm, step), ties included,
    1e-6 tolerance so float noise cannot decide) are the same spot -- the installation
    tolerance is +-2 cm."""
    out = []
    for c in lst:
        if all(not (c["mount"] == t["mount"] and
                    np.hypot(c["u"] - t["u"], c["v"] - t["v"]) <= max(0.04, step) + 1e-6)
               for t in out):
            out.append(c)
        if len(out) == n:
            break
    return out


def rank(P, grid, refined, step, top_n):
    """Distinct top candidates, ALWAYS scored with the straight moves checked.

    The shortlist (3*top_n distinct spots) is re-scored with the straight moves checked,
    so the scores shown and the order WITHIN the list are exact. With --no-path, which
    spots were refined, which spot represents a cluster and which spots made the
    shortlist were decided on optimistic scores (path term assumed full), so best/top can
    differ from a default run -- explore with --no-path, decide without it.
    Returns (top, seconds)."""
    t0 = time.time()
    cands = []
    for n, g in grid.items():
        for i in np.where(g["feasible"])[0]:
            cands.append(dict(mount=n, mount_index=int(g["mount_index"]), u=round(float(g["U"][i]), 6),
                              v=round(float(g["V"][i]), 6), yaw=round(float(g["Y"][i]), 9),
                              score=float(g["score"][i]), src="grid"))
    mi_of = {n: int(g["mount_index"]) for n, g in grid.items()}
    for c in refined:
        c["mount_index"] = mi_of[c["mount"]]
    cands += refined
    cands.sort(key=lambda d: -d["score"])

    shortlist = distinct(cands, 3 * top_n, step)
    for c in shortlist:
        m = P.cell.mounts[c["mount_index"]]
        T = P.cell.mount_pose(m, np.array([c["u"]]), np.array([c["v"]]), np.array([c["yaw"]]))
        r = P.evaluate(m, T, with_path=True)
        c["branch"] = int(r["branch"][0])
        c["branch_name"] = kin.BRANCH_NAMES[c["branch"]] if c["branch"] >= 0 else None
        c["n_feasible_branches"] = int(r["n_feasible_branches"][0])
        for k in ["score", "manip", "ur", "jlim", "tcycle", "clear", "clear_all", "path_frac", "path_clear", "path_wmin"]:
            c[k] = float(r[k][0])
        c["T_cell_base_link"] = np.round(T[0], 6).tolist()
        c["mount_spec"] = dict(name=m["name"], kind=m["kind"], height=float(m.get("height", 0.0)))
        c["placement"] = placement_snippets(T[0])
        c["targets"] = P.detail(m, T[0], c["branch"])[0] if c["branch"] >= 0 else []
    shortlist = [c for c in shortlist if c["branch"] >= 0]
    shortlist.sort(key=lambda d: -d["score"])
    return shortlist[:top_n], time.time() - t0


def diagnose(f, blk, nt):
    """Why a mount has no feasible base, by funnel stage (None if it has some)."""
    if f["feasible"]:
        return None
    if not f["footprint_ok"]:
        return ("the base/pedestal/plate never fits: check the u/v range, the pedestal height and the "
                "obstacle boxes near the mount")
    def worst(key):
        w = sorted(blk.items(), key=lambda kv: kv[1][key])[:3]
        return ", ".join(f"{t} {v[key] * 100:.0f} %" for t, v in w)
    if not f["every_target_has_ik"]:
        return f"some target has no IK within the joint limits from any fitting base; least reachable: {worst('reachable')}"
    if not f["one_branch_has_ik_for_all"]:
        return "every target is reachable, but never on one common IK branch (the arm would have to reconfigure)"
    if not f["every_target_collision_free_some_branch"]:
        return f"some target collides on every branch; least collision-free: {worst('collision_free')}"
    return (f"each target is collision-free on some branch, but no single branch serves all of them "
            f"(best branch: {f['best_branch_covers']}/{nt} targets)")


def _clean(o):
    """JSON-safe: numpy -> python, NaN/inf -> None (RFC 8259 has no NaN)."""
    if isinstance(o, dict):
        return {k: _clean(v) for k, v in o.items()}
    if isinstance(o, (list, tuple)):
        return [_clean(v) for v in o]
    if isinstance(o, (np.floating, float)):
        return float(o) if np.isfinite(o) else None
    if isinstance(o, np.integer):
        return int(o)
    if isinstance(o, np.bool_):
        return bool(o)
    if isinstance(o, np.ndarray):
        return _clean(o.tolist())
    return o


def placement_snippets(T):
    """Snippets for putting the robot at T (cell -> base_link), cell frame == world."""
    r, p, y = kin.R_to_rpy(T[:3, :3])
    q = kin.R_to_quat_wxyz(T[:3, :3])
    x = T[:3, 3]
    return dict(
        xyz=[float(v) for v in x], rpy_rad=[float(r), float(p), float(y)],
        quat_wxyz=[float(v) for v in q],
        xacro_origin=f'<origin xyz="{x[0]:.4f} {x[1]:.4f} {x[2]:.4f}" rpy="{r:.6f} {p:.6f} {y:.6f}"/>',
        isaac=dict(prim="/UR16e", position=[round(float(v), 4) for v in x],
                   orientation_wxyz=[round(float(v), 6) for v in q]),
    )


def plot_maps(P, grid, best, path, recommended=None):
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        from matplotlib.patches import Rectangle
    except ImportError:
        print("  (matplotlib not available -- skipping plots)")
        return
    names = list(grid)
    ncol = min(3, len(names))
    nrow = int(np.ceil(len(names) / ncol))
    fig, axs = plt.subplots(nrow, ncol, figsize=(5 * ncol, 4.3 * nrow), squeeze=False, layout="constrained")
    vmax = max([np.max(np.where(g["feasible"], g["score"], 0)) for g in grid.values()] + [1e-6])
    im = None
    for k, name in enumerate(names):
        ax = axs[k // ncol, k % ncol]
        g = grid[name]
        m = P.cell.mounts[g["mount_index"]]
        uv = cellmod.MOUNT_UV[m["kind"]]
        ia, ib = "xyz".index(uv[0]), "xyz".index(uv[1])
        nu, nv, ny = len(g["us"]), len(g["vs"]), len(g["ys"])
        sc = np.where(g["feasible"], g["score"], np.nan).reshape(nu, nv, ny)
        img = np.nanmax(sc, 2) if np.isfinite(sc).any() else np.full((nu, nv), np.nan)
        foot = g["foot"].reshape(nu, nv, ny).any(2)
        reach = g["reach_any"].reshape(nu, nv, ny).any(2)
        su = (g["us"][1] - g["us"][0]) if nu > 1 else 0.05
        sv = (g["vs"][1] - g["vs"][0]) if nv > 1 else 0.05
        ext = [g["us"][0] - su / 2, g["us"][-1] + su / 2, g["vs"][0] - sv / 2, g["vs"][-1] + sv / 2]
        bg = np.ones((nu, nv, 3))
        bg[foot] = (0.93, 0.93, 0.90)
        bg[reach] = (0.77, 0.77, 0.74)
        ax.imshow(np.swapaxes(bg, 0, 1), origin="lower", extent=ext, interpolation="nearest")
        im = ax.imshow(img.T, origin="lower", extent=ext, cmap="Blues", vmin=0, vmax=vmax, interpolation="nearest")
        for bn, bl, bh in zip(P.cell.box_names, P.cell.box_lo, P.cell.box_hi):
            ax.add_patch(Rectangle((bl[ia], bl[ib]), bh[ia] - bl[ia], bh[ib] - bl[ib], fill=False, ls="--", lw=0.8))
            ax.text(bl[ia] + 0.01, bh[ib] - 0.04, bn, fontsize=7)
        for t, n in enumerate(P.names):
            pp = P.T_w_tcp[t, :3, 3]
            ax.plot(pp[ia], pp[ib], "kx", ms=6)
            ax.text(pp[ia] + 0.015, pp[ib] + 0.015, n, fontsize=7)
        if best is not None and best["mount"] == name:
            ax.plot(best["u"], best["v"], "*", color="#eb6834", ms=15, mec="k")
        if recommended is not None and recommended["mount"] == name:
            ax.plot(recommended["u"], recommended["v"], "D", color="#eda100", ms=9, mec="k")
        lo, hi = P.cell.lo, P.cell.hi
        ax.add_patch(Rectangle((lo[ia], lo[ib]), hi[ia] - lo[ia], hi[ib] - lo[ib], fill=False, lw=1.5))
        ax.set_title(f"{name}: {int(np.isfinite(sc).any(2).sum())} feasible cells", fontsize=9)
        ax.set_xlabel(f"{uv[0]} [m]")
        ax.set_ylabel(f"{uv[1]} [m]")
        ax.set_aspect("equal")
    for k in range(len(names), nrow * ncol):
        axs[k // ncol, k % ncol].axis("off")
    if im is not None:
        fig.colorbar(im, ax=axs.ravel().tolist(), shrink=0.6, label="score (feasible)")
    fig.suptitle("white: base does not fit | light gray: fits, a target has no IK | "
                 "mid gray: IK everywhere but no single collision-free branch | blue: feasible\n"
                 "star: best score | diamond: recommended (best under the installation tolerance)", fontsize=9)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main():
    global _P
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("cell", help="cell YAML (see examples/confined_cell.yaml)")
    ap.add_argument("--out", default=None, help="output dir (default: out/<cell name>)")
    ap.add_argument("--grid", type=float, default=None, help="u/v grid step [m] (default: YAML search.grid_step or 0.05)")
    ap.add_argument("--yaw-step", default=None, help="'auto' (default), 0 = fixed, or degrees")
    ap.add_argument("--mounts", default=None, help="comma list of mount names to evaluate (default: all)")
    ap.add_argument("--workers", type=int, default=1, help="processes (default 1 -- shared CPU)")
    ap.add_argument("--chunk", type=int, default=None,
                    help="candidates per job (default 2000; 250 with --workers > 1 so the work spreads)")
    ap.add_argument("--no-path", action="store_true",
                    help="skip straight-move collision sampling in the grid, refine and robustness stages "
                         "(those scores assume every move is free). The final shortlist is re-scored with "
                         "paths checked, but which spots reach it is decided optimistically, so best/top can "
                         "differ from a default run. Use it to explore; decide without it.")
    ap.add_argument("--no-refine", action="store_true", help="skip the finer local grid around the best spots")
    ap.add_argument("--no-robust", action="store_true",
                    help="skip the installation-tolerance study (the slowest stage); recommended = best")
    ap.add_argument("--top", type=int, default=None, help="distinct candidates to report (default 10)")
    ap.add_argument("--plot", action="store_true", help="write score_maps.png (needs matplotlib)")
    ap.add_argument("--cache-dir", default=None, help="where generated URDFs are cached")
    args = ap.parse_args()

    cell_path = os.path.abspath(args.cell)
    try:
        C = cellmod.Cell(cell_path)
    except (ValueError, KeyError) as e:
        raise SystemExit(f"cell YAML {cell_path}: {e}")
    # Mount indices always refer to the FULL YAML list (results.json, pool workers and
    # moveit_validate all see the full list); --mounts only selects which ones to evaluate.
    sel = list(range(len(C.mounts)))
    if args.mounts is not None:
        keep = [k.strip() for k in args.mounts.split(",") if k.strip()]
        if not keep:
            raise SystemExit("--mounts: give at least one mount name")
        unknown = sorted(set(keep) - {m["name"] for m in C.mounts})
        if unknown:
            raise SystemExit(f"--mounts: unknown {unknown}; the cell has {[m['name'] for m in C.mounts]}")
        sel = [i for i, m in enumerate(C.mounts) if m["name"] in keep]
    S = C.search
    step = float(args.grid if args.grid is not None else S.get("grid_step", 0.05))
    top_n = int(args.top if args.top is not None else S.get("top_k", 10))
    if step <= 0 or top_n <= 0 or (args.chunk is not None and args.chunk <= 0) or args.workers < 1:
        raise SystemExit("--grid, --top, --chunk and --workers must be positive")
    with_path = not args.no_path
    out_dir = args.out or os.path.join(HERE, "out", os.path.splitext(os.path.basename(cell_path))[0])
    os.makedirs(out_dir, exist_ok=True)

    T_ALL = time.time()
    try:
        P = evaluator.Problem(C, args.cache_dir)
    except (ValueError, KeyError) as e:
        raise SystemExit(f"configuration error: {e}")
    try:
        yaws, yaw_free = yaw_values(P, args.yaw_step if args.yaw_step is not None else S.get("yaw_step_deg", "auto"))
    except ValueError:
        raise SystemExit(f"--yaw-step / search.yaw_step_deg must be 'auto' or a number of degrees")
    print(f"targets ({len(P.names)}): {P.names}   order: {[P.names[i] for i in P.order]}  cyclic={P.cyclic}")
    print(f"set {P.set_id}, tool0->TCP z = {P.T_tool0_tcp[2, 3]:.4f} m, spheres {len(P.rad)}, "
          f"grid {step} m, yaw {'free (J1 +-360)' if yaw_free else np.degrees(yaws).round(1).tolist()}")
    print(f"joint limits [deg] {np.degrees(P.lower).round(0).tolist()} .. {np.degrees(P.upper).round(0).tolist()}")

    pool = None
    _P = P
    if args.workers > 1:
        from multiprocessing import Pool
        pool = Pool(args.workers, initializer=_init, initargs=(cell_path, args.cache_dir))
    chunk = args.chunk or (250 if pool is not None else 2000)
    timing = {}
    try:
        t0 = time.time()
        grid = grid_search(P, pool, step, yaws, with_path, chunk, sel)
        timing["grid_s"] = time.time() - t0
        refined = []
        if not args.no_refine:
            t0 = time.time()
            refined = refine(P, pool, grid, step, int(S.get("refine_seeds", 3)), with_path, chunk)
            timing["refine_s"] = time.time() - t0
        top, timing["rank_s"] = rank(P, grid, refined, step, top_n)
        if not args.no_robust and top:
            t0 = time.time()
            jobs = [(c["mount_index"], c["u"], c["v"], c["yaw"], c["branch"], S.get("robustness", {}) or {},
                     with_path) for c in top]
            robs = [_robust_job(j) for j in jobs] if pool is None else pool.map(_robust_job, jobs)
            for c, rb in zip(top, robs):
                c["robustness"] = rb
                ip = rb["in_plane"]
                c["robust_score"] = ip["score_same_branch_mean"]
                if not with_path:
                    # in_plane ran without the path check (term assumed full). Use the
                    # candidate's own path-checked path_frac instead, so robust_score is on
                    # the same footing as score (assumes a 2 cm shift keeps the blocked moves).
                    c["robust_score"] -= P.W["path"] * ip["frac_same_branch_feasible"] * (1.0 - c["path_frac"])
            timing["robust_s"] = time.time() - t0
        else:
            for c in top:
                c["robust_score"] = None
    finally:
        if pool is not None:
            pool.close()
            pool.join()

    # ------------------------------------------------------------ funnel ---
    funnel, blockers = {}, {}
    for n, g in grid.items():
        funnel[n] = dict(total=int(len(g["U"])), footprint_ok=int(g["foot"].sum()),
                         every_target_has_ik=int(g["reach_any"].sum()),
                         one_branch_has_ik_for_all=int(g["common_ik"].sum()),
                         every_target_collision_free_some_branch=int(g["collfree_any"].sum()),
                         feasible=int(g["feasible"].sum()),
                         best_score=float(np.max(g["score"])) if g["feasible"].any() else None,
                         runtime_s=round(g["runtime_s"], 2))
        f = g["foot"]
        if f.any():
            blockers[n] = {P.names[t]: dict(reachable=float(g["tgt_reach"][f, t].mean()),
                                            collision_free=float(g["tgt_cfree"][f, t].mean()))
                           for t in range(len(P.names))}
        funnel[n]["best_branch_covers"] = int(g["best_cov"].max()) if len(g["best_cov"]) else 0
        funnel[n]["diagnosis"] = diagnose(funnel[n], blockers.get(n), len(P.names))

    best = top[0] if top else None
    recommended = max(top, key=lambda c: c["robust_score"]) if top and not args.no_robust else best
    details = {lab: c["targets"] for lab, c in (("best", best), ("recommended", recommended)) if c is not None}
    if args.plot:
        plot_maps(P, grid, best, os.path.join(out_dir, "score_maps.png"), recommended)
    timing["total_s"] = time.time() - T_ALL

    # ------------------------------------------------------------ outputs ---
    with open(os.path.join(out_dir, "grid_feasible.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["mount", "u", "v", "yaw_deg", "branch", "score", "manip", "ur", "jlim", "tcycle_s",
                    "clear_arm_m", "path_frac", "path_clear_m"])
        for n, g in grid.items():
            for i in np.where(g["feasible"])[0]:
                w.writerow([n, f"{g['U'][i]:.3f}", f"{g['V'][i]:.3f}", f"{np.degrees(g['Y'][i]):.1f}",
                            kin.BRANCH_NAMES[g["branch"][i]], f"{g['score'][i]:.4f}", f"{g['manip'][i]:.4f}",
                            f"{g['ur'][i]:.4f}", f"{g['jlim'][i]:.4f}", f"{g['tcycle'][i]:.3f}",
                            f"{g['clear'][i]:.4f}", f"{g['path_frac'][i]:.3f}", f"{g['path_clear'][i]:.4f}"])
    with open(os.path.join(out_dir, "top_candidates.csv"), "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["rank", "mount", "u", "v", "yaw_deg", "src", "branch", "score", "robust_score", "manip", "ur",
                    "jlim", "tcycle_s", "clear_arm_m", "path_frac", "x", "y", "z", "roll", "pitch", "yaw"])
        for i, c in enumerate(top):
            pl = c["placement"]
            w.writerow([i + 1, c["mount"], f"{c['u']:.4f}", f"{c['v']:.4f}", f"{np.degrees(c['yaw']):.1f}", c["src"],
                        c["branch_name"], f"{c['score']:.4f}",
                        "" if c["robust_score"] is None else f"{c['robust_score']:.4f}",
                        f"{c['manip']:.3f}", f"{c['ur']:.3f}", f"{c['jlim']:.3f}", f"{c['tcycle']:.2f}",
                        f"{c['clear']:.3f}", f"{c['path_frac']:.3f}",
                        *[f"{v:.4f}" for v in pl["xyz"]], *[f"{v:.6f}" for v in pl["rpy_rad"]]])
    result = dict(cell=cell_path, cell_sha256=C.sha256, mounts_evaluated=[C.mounts[i]["name"] for i in sel],
                  targets=P.names, order=[P.names[i] for i in P.order], cyclic=P.cyclic,
                  set=P.set_id, tool0_to_tcp=np.round(P.T_tool0_tcp, 6).tolist(),
                  joint_limits_rad=dict(lower=P.lower.tolist(), upper=P.upper.tolist(), vmax=P.vmax.tolist()),
                  weights=P.W, constants=P.K, wmax_abs_detJ=P.wmax, grid_step=step,
                  yaw=dict(free=bool(yaw_free), values_deg=np.degrees(yaws).tolist()),
                  funnel=funnel, target_blockers=blockers, top=top, best=best, recommended=recommended,
                  details=details, timing=timing,
                  stages=dict(path_checked_in_grid=with_path, refine=not args.no_refine,
                              robustness=not args.no_robust, final_ranking_path_checked=True),
                  urdf=dict(kin.URDF_PROVENANCE),
                  n_candidates=int(sum(v["total"] for v in funnel.values())),
                  n_feasible=int(sum(v["feasible"] for v in funnel.values())))
    with open(os.path.join(out_dir, "results.json"), "w") as f:
        json.dump(_clean(result), f, indent=1, allow_nan=False)
    write_report(os.path.join(out_dir, "report.md"), result)

    print("\nfunnel:")
    for n, f in funnel.items():
        print(f"  {n:14s} total {f['total']:6d}  fits {f['footprint_ok']:6d}  all-IK {f['every_target_has_ik']:6d}  "
              f"one-branch-IK {f['one_branch_has_ik_for_all']:6d}  coll-free {f['every_target_collision_free_some_branch']:6d}  "
              f"FEASIBLE {f['feasible']:6d}")
    if not top:
        print("\nNO feasible base. Why, per mount (fix the first stage that fails):")
        for n, f in funnel.items():
            print(f"  {n:14s} {f['diagnosis']}")
    else:
        print(f"\ntop {len(top)}:")
        for i, c in enumerate(top):
            rs = "" if c["robust_score"] is None else f" robust {c['robust_score']:.3f}"
            print(f" {i + 1:2d}. {c['mount']:14s} u={c['u']:.3f} v={c['v']:.3f} yaw={np.degrees(c['yaw']):6.1f}  "
                  f"{c['branch_name']}  score {c['score']:.3f}{rs}  T {c['tcycle']:.2f}s  path {c['path_frac']:.2f}")
        rec = recommended
        print(f"\nrecommended: {rec['mount']}  {rec['placement']['xacro_origin']}")
    print(f"\nwrote {out_dir}/results.json, report.md, top_candidates.csv, grid_feasible.csv   "
          f"({timing['total_s']:.1f} s)")


def write_report(path, R):
    L = [f"# Base placement report", "",
         f"cell: `{R['cell']}`  set {R['set']}  targets {len(R['targets'])}  order {R['order']}  cyclic {R['cyclic']}",
         f"candidates {R['n_candidates']}, feasible {R['n_feasible']}, grid {R['grid_step']} m, "
         f"yaw {'free (J1 +-360 deg: choose for cable exit)' if R['yaw']['free'] else R['yaw']['values_deg']}", "",
         ("> grid/refine/robustness ran with `--no-path`: those scores assumed every straight move free. "
          "The scores below were re-checked with the moves, but which spots made the list was decided on the "
          "optimistic scores -- best/top can differ from a default run. Decide on a default run."
          if not R["stages"]["path_checked_in_grid"]
          else "> straight joint moves were checked in every stage."),
         "> The sweep's collision model is spheres, a filter: the cuMotion XRDF's world spheres (they cover the "
         "meshes, <= 20 mm overshoot) for the cell and self checks; agreement with MoveIt depends on the margins "
         "(HISTORY.md 51.6, still being re-tuned). "
         "Confirm the chosen candidate with `moveit_validate.py results.json --pick all` before building.", ""]
    L += ["## Funnel", "", "| mount | total | fits | every target IK | one branch IK | collision free | feasible | why not |",
          "|---|---|---|---|---|---|---|---|"]
    for n, f in R["funnel"].items():
        L.append(f"| {n} | {f['total']} | {f['footprint_ok']} | {f['every_target_has_ik']} | "
                 f"{f['one_branch_has_ik_for_all']} | {f['every_target_collision_free_some_branch']} | {f['feasible']} | "
                 f"{f.get('diagnosis') or ''} |")
    L += ["", "## Per-target blockers (fraction of fitting bases: reachable / collision free)", ""]
    for n, b in R["target_blockers"].items():
        L.append(f"- **{n}**: " + ", ".join(f"{t} {v['reachable']:.2f}/{v['collision_free']:.2f}" for t, v in b.items()))
    for lab in ("recommended", "best"):
        c = R.get(lab)
        if not c:
            continue
        L += ["", f"## {lab}: {c['mount']} u={c['u']:.3f} v={c['v']:.3f} yaw={np.degrees(c['yaw']):.1f} deg, "
                  f"branch {c['branch_name']}, score {c['score']:.3f}", "",
              f"- xacro (`world` = cell frame): `{c['placement']['xacro_origin']}`",
              f"- Isaac prim `/UR16e`: position {c['placement']['isaac']['position']}, "
              f"orientation wxyz {c['placement']['isaac']['orientation_wxyz']}"]
        if c.get("robustness"):
            rb = c["robustness"]
            L.append(f"- robustness: same branch feasible {rb['in_plane']['frac_same_branch_feasible']:.2f} (in-plane), "
                     f"{rb['random_6dof']['frac_same_branch_feasible']:.2f} (random 6-DoF)")
        L += ["", "| target | q [deg] | clear [m] | w | abs sin q5 | abs sin q3 | shoulder [m] | next | t [s] | straight |",
              "|---|---|---|---|---|---|---|---|---|---|"]
        for d in R["details"].get(lab, []):
            mv = d["move_to_next"] or {}
            L.append(f"| {d['target']} | {d['q_deg']} | {d['clearance_m']:.3f} | {d['w_norm']:.3f} | "
                     f"{d['abs_sin_q5']:.2f} | {d['abs_sin_q3']:.2f} | {d['shoulder_dist_m']:.3f} | "
                     f"{mv.get('to', '-')} | {mv.get('t_s', 0):.2f} | "
                     f"{'free' if mv.get('straight_free') else ('-' if not mv else 'BLOCKED')} |")
    with open(path, "w") as f:
        f.write("\n".join(L) + "\n")


if __name__ == "__main__":
    main()
