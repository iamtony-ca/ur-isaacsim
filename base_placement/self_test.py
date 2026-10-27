#!/usr/bin/env python3
"""Regression self-test (no ROS nodes, single thread, ~1 s).

    source /opt/ros/jazzy/setup.bash
    python3 self_test.py [--n 1000]

1. Kinematics: analytic IK / DH FK against the xacro-generated UR16e URDF.
   Pass: FK < 1e-8 m / 1e-6 rad, IK round trip < 1e-8 m, generating q recovered for
   > 99 % of samples (misses are ill-conditioned samples next to a singularity).
2. Regressions for the defects found on 2026-10-16 (HISTORY.md 50): 180-degree
   quaternions, unknown URDF links, set-1 TCP, footprint vs boxes inside the base /
   under the pedestal, 2pi-alias centring, duplicate names, SRDF self-collision pairs.
3. Regressions for the second review (HISTORY.md 50.2): YAML key typos, the mounting
   face is solid even if the YAML opens it, grid axes keep their high end, candidate
   dedup is not decided by float noise, xacro leaves no temp prefix behind, MoveIt
   object ids of obstacles cannot replace generated ones.
"""
import os

for _v in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
    os.environ.setdefault(_v, "1")

import argparse          # noqa: E402
import itertools         # noqa: E402
import tempfile          # noqa: E402

import numpy as np       # noqa: E402
import yaml              # noqa: E402

import cell as cellmod   # noqa: E402
import evaluator         # noqa: E402
import place_base        # noqa: E402
import ur16e_kin as kin  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
FAILS = []


def check(name, cond, detail=""):
    print(f"  {'ok  ' if cond else 'FAIL'} {name}" + (f"  ({detail})" if detail else ""))
    if not cond:
        FAILS.append(name)


def kinematics(n):
    print("== kinematics vs xacro URDF")
    arm = kin.URDFArm(kin.generate_urdf(1))
    T_bl_dh0, T_dh6_tool0 = kin.reconcile(arm)
    check("base_link -> DH base is Rz(pi)", np.allclose(T_bl_dh0, kin.make_T(kin.rpy_to_R(0, 0, np.pi)), atol=1e-12))
    check("DH frame 6 == tool0", np.allclose(T_dh6_tool0, np.eye(4), atol=1e-9))
    rng = np.random.default_rng(1)
    q = rng.uniform(arm.lower, arm.upper, (n, 6))
    T_urdf = arm.fk_tool0(q)
    T_dh = T_bl_dh0 @ kin.dh_fk(q) @ T_dh6_tool0
    dp = np.linalg.norm(T_urdf[:, :3, 3] - T_dh[:, :3, 3], axis=1).max()
    dr = kin.rot_angle(T_urdf[:, :3, :3], T_dh[:, :3, :3]).max()
    check("FK URDF vs DH", dp < 1e-8 and dr < 1e-6, f"{dp:.2e} m, {dr:.2e} rad")
    Td = kin.inv_T(T_bl_dh0) @ T_urdf @ kin.inv_T(T_dh6_tool0)
    Q, V = kin.dh_ik(Td)
    errs, found = [], 0
    for i in range(n):
        qs = Q[i][V[i]]
        errs.append(np.linalg.norm(arm.fk_tool0(qs)[:, :3, 3] - T_urdf[i, :3, 3], axis=1).max())
        found += bool((np.abs(kin.wrap(qs - q[i])).max(1) < 1e-6).any())
    check("IK round trip", max(errs) < 1e-8 and found / n > 0.99,
          f"{max(errs):.2e} m, branches/pose {V.sum(1).min()}..{V.sum(1).max()}, recovered {found / n:.4f}")


def quaternions():
    print("== rotation conversions")
    rng = np.random.default_rng(2)
    worst = 0.0
    mats = []
    for _ in range(2000):                                   # exact 180-degree rotations, mixed-sign axes
        n = rng.normal(size=3)
        n /= np.linalg.norm(n)
        mats.append(2 * np.outer(n, n) - np.eye(3))
    for kind, R0 in cellmod.MOUNT_R.items():                # every mount at every yaw
        for yaw in np.radians(np.arange(0, 360, 5)):
            mats.append(R0 @ cellmod._Rz(yaw))
    for _ in range(2000):
        q = rng.normal(size=4)
        mats.append(kin.quat_xyzw_to_R(q))
    for R in mats:
        w, x, y, z = kin.R_to_quat_wxyz(R)
        worst = max(worst, np.abs(kin.quat_xyzw_to_R([x, y, z, w]) - R).max())
    check("R -> quat -> R (180 deg, mounts x yaw, random)", worst < 1e-9, f"max {worst:.1e}")
    rpy_worst = max(np.abs(kin.rpy_to_R(*kin.R_to_rpy(R)) - R).max() for R in mats)
    check("R -> rpy -> R", rpy_worst < 1e-6, f"max {rpy_worst:.1e}")


def links_and_tcp():
    print("== URDF links / TCP")
    u1, u2 = kin.generate_urdf(1), kin.generate_urdf(2)
    m = kin.UrdfModel(u1)
    try:
        m.fixed_T("tool0", "no_such_link")
        check("unknown link raises", False)
    except KeyError:
        check("unknown link raises", True)
    try:
        kin.tcp_transform(u1, "pad")
        check("tcp pad on set 1 raises", False)
    except ValueError:
        check("tcp pad on set 1 raises", True)
    try:
        kin.tcp_transform(u1, "custom", custom=None)
        check("tcp custom without a dict raises", False)
    except ValueError:
        check("tcp custom without a dict raises", True)
    gf = kin.tcp_transform(u2, "gripper_frame")
    pad = kin.tcp_transform(u2, "pad")
    check("set 2 gripper_frame = tool0 + 11 mm", np.allclose(gf, kin.make_T(np.eye(3), [0, 0, 0.011]), atol=1e-9))
    check("set 2 pad = tool0 + 141.3 mm", abs(pad[2, 3] - 0.141326) < 1e-5, f"{pad[2, 3]:.6f}")


def footprint_and_alias(tmp):
    print("== footprint / aliases / names")
    C = yaml.safe_load(open(os.path.join(HERE, "examples", "confined_cell.yaml")))
    C["cell"]["obstacles"] = [{"name": "inside_base", "lo": [0.48, 0.48, 0.0], "hi": [0.52, 0.52, 0.05]},
                              {"name": "under_ped", "lo": [0.92, 0.42, 0.0], "hi": [0.98, 0.48, 0.10]}]
    p = os.path.join(tmp, "fp.yaml")
    yaml.safe_dump(C, open(p, "w"))
    P = evaluator.Problem(cellmod.Cell(p))
    floor = next(m for m in P.cell.mounts if m["kind"] == "floor" and float(m.get("height", 0)) == 0)
    ped = next(m for m in P.cell.mounts if m["kind"] == "floor" and float(m.get("height", 0)) > 0)
    T = P.cell.mount_pose(floor, np.array([0.5, 0.7]), np.array([0.5, 0.7]), np.zeros(2))
    ok = P.footprint_ok(floor, T)
    check("box inside the base disk rejected / clear spot kept", (not ok[0]) and ok[1], str(ok.tolist()))
    T = P.cell.mount_pose(ped, np.array([0.95]), np.array([0.45]), np.zeros(1))
    check("box under the pedestal rejected", not P.footprint_ok(ped, T)[0])

    rng = np.random.default_rng(3)
    worst_gap = 0.0
    for nt in (1, 2, 3, 5):
        for cyc in (True, False):
            P.order = list(range(nt))
            P.cyclic = cyc
            Qp = kin.wrap(rng.uniform(-np.pi, np.pi, (40, nt, 6)))
            Qa = P.unwrap_cycle(Qp)
            for f in range(40):
                for j in range(6):
                    v = Qp[f, :, j]
                    opts = [(a, a - 2 * np.pi * np.sign(a) if a != 0 else a) for a in v]
                    best = np.inf
                    for combo in itertools.product(*opts):
                        c = np.array(combo)
                        if ((c < P.lower[j] - 1e-9) | (c > P.upper[j] + 1e-9)).any():
                            continue
                        mv = np.abs(np.diff(c)).sum() + (abs(c[-1] - c[0]) if cyc else 0)
                        best = min(best, mv)
                    got = Qa[f, :, j]
                    mv = np.abs(np.diff(got)).sum() + (abs(got[-1] - got[0]) if cyc else 0)
                    worst_gap = max(worst_gap, mv - best)
    check("alias DP travel == brute force optimum", worst_gap < 1e-9, f"max excess {worst_gap:.1e} rad")
    P.order, P.cyclic = [0, 1, 2], True
    q = np.zeros((1, 3, 6))
    q[0, :, 4] = np.radians([90, 32, 90])                   # all-negative-alias tie case
    Qa = P.unwrap_cycle(q)
    check("2pi tie centred (q5 near +90, not -270)", np.all(np.abs(Qa[0, :, 4]) < np.pi), str(np.degrees(Qa[0, :, 4]).round(1)))

    for key in ("mounts", "obstacles"):
        C2 = yaml.safe_load(open(os.path.join(HERE, "examples", "confined_cell.yaml")))
        if key == "mounts":
            C2["mounts"][1]["name"] = C2["mounts"][0]["name"]
        else:
            C2["cell"]["obstacles"][1]["name"] = C2["cell"]["obstacles"][0]["name"]
        p2 = os.path.join(tmp, f"dup_{key}.yaml")
        yaml.safe_dump(C2, open(p2, "w"))
        try:
            cellmod.Cell(p2)
            check(f"duplicate {key} names rejected", False)
        except ValueError:
            check(f"duplicate {key} names rejected", True)


def self_pairs():
    print("== self-collision pairs follow MoveIt's SRDF")
    arm = kin.URDFArm(kin.generate_urdf(1))
    sph = kin.load_spheres(arm, kin.generate_urdf(2))
    ii, jj, _ = kin.self_pairs(sph)
    pairs = {frozenset((sph["name"][i], sph["name"][j])) for i, j in zip(ii, jj)}
    check("forearm <-> wrist_2 checked", frozenset(("forearm_link", "wrist_2_link")) in pairs)
    check("forearm <-> wrist_3 checked", frozenset(("forearm_link", "wrist_3_link")) in pairs)
    check("forearm <-> wrist_1 (adjacent) skipped", frozenset(("forearm_link", "wrist_1_link")) not in pairs)


def second_review(tmp):
    print("== second review (YAML keys, mounting face, grid axes, dedup, xacro prefix, scene ids)")
    base = yaml.safe_load(open(os.path.join(HERE, "examples", "confined_cell.yaml")))

    def load(mod, name):
        C2 = yaml.safe_load(yaml.safe_dump(base))
        mod(C2)
        p2 = os.path.join(tmp, name)
        yaml.safe_dump(C2, open(p2, "w"))
        return cellmod.Cell(p2)

    for what, mod in (("cell 'obstacle' typo", lambda C: C["cell"].update(obstacle=[])),
                      ("mount 'heigth' typo", lambda C: C["mounts"][0].update(heigth=0.1)),
                      ("walls 'zmx' typo", lambda C: C["cell"]["walls"].update(zmx=False)),
                      ("walls value 'no' (not a bool)", lambda C: C["cell"]["walls"].update(zmax="no"))):
        try:
            load(mod, "keys.yaml")
            check(f"{what} rejected", False)
        except ValueError as e:
            check(f"{what} rejected", True, str(e).split(";")[0][:60])

    def open_all(C):
        C["cell"]["walls"] = {k: False for k in cellmod.WALL_KEYS}
    P = evaluator.Problem(load(open_all, "open.yaml"))
    ceil = next(m for m in P.cell.mounts if m["kind"] == "ceiling")
    floor = next(m for m in P.cell.mounts if m["kind"] == "floor")
    check("open ceiling is solid for a ceiling mount", P.cell.walls_for(ceil)["zmax"] and not P.cell.walls_for(floor)["zmax"])
    P.evaluate(ceil, P.cell.mount_pose(ceil, np.array([0.7]), np.array([0.5]), np.zeros(1)), with_path=False)
    above = np.array([[[0.7, 0.5, P.cell.hi[2] + 0.05]]])
    check("evaluate() applies it (point above the ceiling is inside the wall)", P.env_sdf(above, None)[0, 0] < 0)

    ax = [place_base._axis(0.0, 1.0, 0.3), place_base._axis(0.0, 0.9, 0.3), place_base._axis(0.0, 0.1, 0.5)]
    check("grid axis keeps the high end, no duplicate", np.allclose(ax[0], [0, .3, .6, .9, 1.0]) and
          np.allclose(ax[1], [0, .3, .6, .9]) and np.allclose(ax[2], [0, .1]), str([a.round(2).tolist() for a in ax]))

    rng = np.random.default_rng(4)
    step = 0.04
    lst = [dict(mount="m", u=0.10 + step * k, v=0.50, score=1.0 - 0.01 * k) for k in range(6)]
    lst += [dict(mount="m", u=0.10 + 0.05 * k, v=0.80, score=0.5 - 0.01 * k) for k in range(4)]
    ref = [(c["u"], c["v"]) for c in place_base.distinct(lst, 99, step)]
    same = True
    for _ in range(50):
        jit = [dict(c, u=c["u"] + rng.uniform(-1e-12, 1e-12), v=c["v"] + rng.uniform(-1e-12, 1e-12)) for c in lst]
        got = [(round(c["u"], 9), round(c["v"], 9)) for c in place_base.distinct(jit, 99, step)]
        same &= got == [(round(u, 9), round(v, 9)) for u, v in ref]
    check("dedup: exactly-one-step neighbours merge, 5 cm ones do not, jitter-proof",
          same and len(ref) == 3 + 4, f"{len(ref)} spots")

    cache = os.path.join(tmp, "urdf_cache")
    kin.generate_urdf(2, cache_dir=cache)
    left = [f for f in os.listdir(cache) if f.startswith("_src_prefix")]
    check("xacro leaves no _src_prefix behind", not left, str(left))

    try:
        import moveit_validate as mv
    except ImportError as e:                      # moveit_msgs not sourced: skip, do not fail
        print(f"  skip planning-scene ids ({e})")
        return
    def clash(C):
        C["cell"]["obstacles"][0]["name"] = "pedestal"
        C["cell"]["obstacles"][1]["name"] = "wall_xmin"
    C2 = load(clash, "clash.yaml")
    ped = next(m for m in C2.mounts if m["kind"] == "floor" and float(m.get("height", 0)) > 0)
    T = C2.mount_pose(ped, np.array([0.3]), np.array([0.3]), np.zeros(1))[0]
    ids = [o.id for o in mv.cell_objects(C2, ped, T)]
    check("obstacles named 'pedestal'/'wall_xmin' keep separate scene ids",
          len(ids) == len(set(ids)) and "bp_obs_pedestal" in ids and "bp_cell_pedestal" in ids, f"{len(ids)} ids")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--n", type=int, default=1000)
    a = ap.parse_args()
    kinematics(a.n)
    quaternions()
    links_and_tcp()
    with tempfile.TemporaryDirectory() as tmp:
        footprint_and_alias(tmp)
        self_pairs()
        second_review(tmp)
    print("\nPASS" if not FAILS else f"\nFAIL: {FAILS}")
    raise SystemExit(0 if not FAILS else 1)


if __name__ == "__main__":
    main()
