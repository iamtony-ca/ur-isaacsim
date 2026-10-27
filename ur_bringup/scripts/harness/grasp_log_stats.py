#!/usr/bin/env python3
"""Summarise one grasp_physical.sh run: <prefix>.steps.csv + .demo.log + .isaac.log.

    grasp_log_stats.py outputs/harness_logs/gp_<tag> [--object red]

Prints, against the physical-grasp acceptance criteria (HISTORY.md 53):
  * per-cycle outcome from the state machine
  * finger_joint range (must stay in [-0.02, 0.82]) and every gripper DOF range
  * guard trips / "finger forced open" lines from Isaac
  * arm: peak |velocity| per joint and the largest one-step jump (a wrist spin
    shows up here as tens of rad/s)
  * each hold (grasp_active run): duration, peak part height, and how far the
    part slid relative to the pad TCP after the first 0.3 s
"""
import argparse
import csv
import math
import re
import sys

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("prefix")
    ap.add_argument("--object", default="red")
    a = ap.parse_args()

    try:
        demo = open(a.prefix + ".demo.log", errors="replace").read()
    except OSError:
        demo = ""
    outcomes = re.findall(r"\[cycle (\d+)\] (SUCCESS[^\n]*|FAIL[^\n]*)", demo)
    summary = re.findall(r"=== (\d+)/(\d+) cycles succeeded", demo)
    print(f"== {a.prefix}")
    print(f"   demo: {summary[-1][0] + '/' + summary[-1][1] if summary else '?'} succeeded")
    for c, o in outcomes:
        print(f"     cycle {c}: {o[:70]}")

    try:
        isaac = open(a.prefix + ".isaac.log", errors="replace").read()
    except OSError:
        isaac = ""
    print(f"   isaac: guard trips {len(re.findall('gripper guard #', isaac))}, "
          f"'finger forced open' {len(re.findall('finger forced open', isaac))}")

    rows = list(csv.DictReader(open(a.prefix + ".steps.csv")))
    if not rows:
        print("   (no steps logged)")
        return 1
    f = lambda r, k: float(r[k])                                   # noqa: E731
    qcols = [k for k in rows[0] if k.startswith("q_")]
    grip = [k for k in qcols if k[2:] not in ARM]
    t = [f(r, "t") for r in rows]
    print(f"   steps {len(rows)}, sim {t[0]:.1f}..{t[-1]:.1f} s")
    for k in grip:
        v = [f(r, k) for r in rows]
        flag = "  <-- OUT" if k == "q_finger_joint" and (min(v) < -0.02 or max(v) > 0.82) else ""
        print(f"     {k[2:]:34s} {min(v):+8.3f} .. {max(v):+8.3f}{flag}")
    # Judge motion from POSITIONS. PhysX's reported joint velocity is not
    # trustworthy under contact: finger_joint reads its 0.995 rad/s cap while
    # standing still on the part, and the wrists read 3-4 rad/s while their
    # positions move at 0.5 rad/s. A spin is a position running away.
    # Skip the first 20 s: controller start-up swings the arm off its home pose.
    k0 = next((i for i, x in enumerate(t) if x >= 20.0), 0)
    print("   arm (t >= 20 s): peak |dq/dt| from positions [rad/s] / reported |v| / range")
    for j in ARM:
        q = [f(r, "q_" + j) for r in rows]
        v = [abs(f(r, "v_" + j)) for r in rows[k0:]]
        fd = max((abs(q[i + 1] - q[i]) / max(t[i + 1] - t[i], 1e-6)
                  for i in range(k0, len(q) - 1)), default=0.0)
        flag = "  <-- SPIN" if fd > math.pi or max(q) - min(q) > 2 * math.pi + 0.5 else ""
        print(f"     {j:22s} {fd:6.2f} / {max(v, default=0.0):5.2f} / "
              f"{min(q):+7.2f}..{max(q):+7.2f}{flag}")

    # Holds: contiguous grasp_active == 1
    ox, oy, oz = (a.object + s for s in ("_x", "_y", "_z"))
    holds, cur = [], None
    for i, r in enumerate(rows):
        on = r["grasp_active"] == "1"
        if on and cur is None:
            cur = [i, i]
        elif on:
            cur[1] = i
        elif cur is not None:
            holds.append(cur); cur = None
    if cur is not None:
        holds.append(cur)
    print(f"   holds ({a.object}): {len(holds)}")
    for n, (i0, i1) in enumerate(holds, 1):
        seg = rows[i0:i1 + 1]
        rel = [(f(r, ox) - f(r, "tcp_x"), f(r, oy) - f(r, "tcp_y"), f(r, oz) - f(r, "tcp_z"))
               for r in seg]
        ts = [f(r, "t") for r in seg]
        fj = [f(r, "q_finger_joint") for r in seg]
        # Slip only while the jaw is shut: the closing and opening ends of a hold
        # move the TCP relative to a part that is not gripped yet / any more.
        shut = [k for k, x in enumerate(fj) if x >= max(fj) - 0.02]
        k0 = shut[0] if shut else 0
        k1 = shut[-1] if shut else len(seg) - 1
        base = rel[k0]
        slip = max(math.dist(base, p) for p in rel[k0:k1 + 1])
        zmax = max(f(r, oz) for r in seg)
        vfj = sorted(abs(f(r, "v_finger_joint")) for r in seg[k0:k1 + 1])
        print(f"        shut {ts[k1] - ts[k0]:5.1f} s, |v_finger| median {vfj[len(vfj) // 2]:.3f} rad/s")
        print(f"     #{n}: {ts[-1] - ts[0]:5.1f} s  part z max {zmax:.3f}  slip {slip * 1000:5.1f} mm  "
              f"fj {min(fj):.3f}..{max(fj):.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
