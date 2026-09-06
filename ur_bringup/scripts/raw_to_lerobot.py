#!/usr/bin/env python3
"""Convert raw episodes recorded by il_recorder.py into a LeRobot dataset.

*** RUN THIS IN THE ML ENVIRONMENT, NOT IN ROS. ***
It needs `lerobot` (and therefore torch), which deliberately do not exist in the
ROS 2 Jazzy Python environment -- see ur_bringup/docs/plan_il_vla.md 2.2 for why
that boundary is kept. il_recorder.py writes a plain, self-describing raw format
(JPEG + JSON) precisely so that this half can live somewhere else.

    python3 raw_to_lerobot.py --raw <recorder out_dir> --repo-id <user>/<name>
    python3 raw_to_lerobot.py --raw <dir> --repo-id x/y --dry-run   # inspect only

The schema written here is ur_bringup/docs/plan_il_vla.md 2.6:
    video.exterior, video.wrist, state.single_arm(6), state.gripper(1),
    action.single_arm(6), action.gripper(1), task
Keep it stable: re-collecting data is far more expensive than re-running code.

Note on LeRobot versions: the dataset API has changed shape across releases
(`LeRobotDataset.create`, feature dicts, `save_episode` signature). This script
targets the API where `create(...)` + `add_frame(...)` + `save_episode()` exist.
If your installed lerobot differs, adapt HERE -- never by changing the recorder,
because that would invalidate data you have already collected.
"""
import argparse
import json
import os
import sys


def load_episodes(raw_dir):
    eps = []
    for name in sorted(os.listdir(raw_dir)):
        d = os.path.join(raw_dir, name)
        if not (name.startswith("episode_") and os.path.isdir(d)):
            continue
        meta_p, data_p = os.path.join(d, "meta.json"), os.path.join(d, "data.json")
        if not (os.path.isfile(meta_p) and os.path.isfile(data_p)):
            print(f"  skip {name}: missing meta.json/data.json (recording interrupted?)")
            continue
        with open(meta_p) as f:
            meta = json.load(f)
        with open(data_p) as f:
            frames = json.load(f)["frames"]
        if len(frames) != meta.get("num_frames", len(frames)):
            print(f"  warn {name}: meta says {meta.get('num_frames')} frames, data has {len(frames)}")
        eps.append((d, meta, frames))
    return eps


def summarise(eps):
    print(f"\nepisodes : {len(eps)}")
    total = sum(len(f) for _, _, f in eps)
    print(f"frames   : {total}")
    tasks = {}
    for _, m, f in eps:
        tasks.setdefault(m.get("task", ""), 0)
        tasks[m["task"]] = tasks.get(m["task"], 0) + 1
    print("tasks    :")
    for t, n in tasks.items():
        print(f"   {n:4d} ep  '{t}'" + ("   <-- EMPTY task string" if not t else ""))
    if len([t for t in tasks if t]) < 2:
        print("\n  ! Fewer than 2 distinct task strings. A VLA trained on this will ignore")
        print("    language and you get an expensive ACT (plan_il_vla.md 2.8).")
    srcs = {m.get("action_source") for _, m, _ in eps}
    if len(srcs) > 1:
        print(f"\n  ! Mixed action_source across episodes: {srcs}. Actions do not mean the")
        print("    same thing in every episode -- convert them separately or re-record.")
    return total


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--raw", required=True, help="il_recorder.py out_dir")
    ap.add_argument("--repo-id", help="LeRobot repo id, e.g. tony/ur16e_pick_place")
    ap.add_argument("--root", default=None, help="output root (default: LeRobot cache)")
    ap.add_argument("--fps", type=int, default=None, help="override fps (default: from meta)")
    ap.add_argument("--robot-type", default="ur16e_2f85")
    ap.add_argument("--dry-run", action="store_true", help="inspect the raw set and exit")
    args = ap.parse_args()

    if not os.path.isdir(args.raw):
        print(f"no such directory: {args.raw}"); return 2
    eps = load_episodes(args.raw)
    if not eps:
        print(f"no episodes found under {args.raw}"); return 1
    summarise(eps)
    if args.dry_run:
        return 0
    if not args.repo_id:
        print("\n--repo-id is required unless --dry-run"); return 2

    try:
        import numpy as np
        from PIL import Image
    except ImportError as e:
        print(f"\nImportError: {e}"); return 3
    # lerobot moved the dataset module: <=0.5 `lerobot.common.datasets`,
    # 0.6+ `lerobot.datasets`. Try new first, fall back to old.
    LeRobotDataset = None
    for mod in ("lerobot.datasets.lerobot_dataset", "lerobot.common.datasets.lerobot_dataset"):
        try:
            LeRobotDataset = __import__(mod, fromlist=["LeRobotDataset"]).LeRobotDataset
            print(f"\nusing {mod}")
            break
        except ImportError as e:
            last = e
    if LeRobotDataset is None:
        print(f"\nImportError: {last}")
        print("Run this in the ML environment (deps/.venv-ml), NOT in ROS. See the docstring.")
        print("If it complains about `datasets`, install the extra:  pip install 'lerobot[dataset]'")
        return 3

    meta0 = eps[0][1]
    fps = args.fps or int(round(meta0.get("rate_hz", 30)))
    video_keys = meta0["video_keys"]

    # Probe one frame for the image shape rather than assuming 640x480.
    probe = os.path.join(eps[0][0], "frames", video_keys[0], "000000.jpg")
    h, w = np.asarray(Image.open(probe)).shape[:2]

    features = {
        "observation.state": {"dtype": "float32", "shape": (7,),
                              "names": meta0["arm_joints"] + ["gripper"]},
        "action": {"dtype": "float32", "shape": (7,),
                   "names": meta0["arm_joints"] + ["gripper"]},
    }
    for k in video_keys:
        features[f"observation.images.{k.split('.')[-1]}"] = {
            "dtype": "video", "shape": (h, w, 3), "names": ["height", "width", "channel"]}

    ds = LeRobotDataset.create(repo_id=args.repo_id, fps=fps, root=args.root,
                               robot_type=args.robot_type, features=features,
                               use_videos=True)

    for d, meta, frames in eps:
        for i, fr in enumerate(frames):
            frame = {
                "observation.state": np.asarray(
                    fr["state.single_arm"] + fr["state.gripper"], dtype=np.float32),
                "action": np.asarray(
                    fr["action.single_arm"] + fr["action.gripper"], dtype=np.float32),
            }
            for k in video_keys:
                img = np.asarray(Image.open(
                    os.path.join(d, "frames", k, f"{i:06d}.jpg")).convert("RGB"))
                frame[f"observation.images.{k.split('.')[-1]}"] = img
            # lerobot 0.6.x: add_frame(frame) only -- the task rides INSIDE the
            # frame dict. (0.5.x took task= as a keyword; passing it here raises.)
            frame["task"] = meta["task"]
            ds.add_frame(frame)
        ds.save_episode()
        print(f"  converted {os.path.basename(d)} ({len(frames)} frames)")

    print(f"\ndone -> {args.repo_id}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
