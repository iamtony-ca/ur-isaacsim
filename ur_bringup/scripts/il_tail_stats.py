#!/usr/bin/env python3
"""How still is the END of each raw episode? Decides whether il_recorder.py's
`trim_tail_frames` is worth turning on for a dataset (GELLO drops its last 5 frames).

For every episode_*/data.json: count trailing frames whose arm state moved less than
--eps [rad] (max over joints) from the previous frame, and the gripper too. Prints a
per-episode line and a summary. If the median still tail is ~0, trimming does nothing;
if it is 10-30 frames at 30 Hz, that is 0.3-1 s of "arm parked while the operator
reaches for the stop button" that every episode teaches the policy to imitate.

    python3 il_tail_stats.py <raw_dir> [--eps 0.002] [--grip-eps 0.01]

Read-only. Runs with ROS python (json only).
"""
import argparse
import glob
import json
import os
import statistics


def still_tail(frames, eps, geps):
    n = 0
    for i in range(len(frames) - 1, 0, -1):
        a, b = frames[i], frames[i - 1]
        darm = max(abs(x - y) for x, y in zip(a["state.single_arm"], b["state.single_arm"]))
        dgr = abs(a["state.gripper"][0] - b["state.gripper"][0])
        if darm < eps and dgr < geps:
            n += 1
        else:
            break
    return n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("raw_dir")
    ap.add_argument("--eps", type=float, default=0.002, help="[rad] per-frame arm motion below this = still")
    ap.add_argument("--grip-eps", type=float, default=0.01, help="normalised gripper motion below this = still")
    a = ap.parse_args()
    eps_files = sorted(glob.glob(os.path.join(a.raw_dir, "episode_*", "data.json")))
    if not eps_files:
        raise SystemExit(f"no episode_*/data.json under {a.raw_dir}")
    tails, lens = [], []
    for f in eps_files:
        frames = json.load(open(f))["frames"]
        meta = json.load(open(os.path.join(os.path.dirname(f), "meta.json")))
        t = still_tail(frames, a.eps, a.grip_eps)
        tails.append(t)
        lens.append(len(frames))
        print(f"{os.path.basename(os.path.dirname(f))}: {len(frames)} frames, still tail {t} "
              f"({t / meta.get('rate_hz', 30):.2f} s), trim_tail_frames was {meta.get('trim_tail_frames', 0)}")
    print(f"\n{len(tails)} episodes: still tail median {statistics.median(tails)} frames, "
          f"max {max(tails)}, mean {statistics.mean(tails):.1f}; "
          f"episode length median {statistics.median(lens)}")
    print("-> a median near 0 means trim_tail_frames buys nothing; 10+ means try it.")


if __name__ == "__main__":
    main()
