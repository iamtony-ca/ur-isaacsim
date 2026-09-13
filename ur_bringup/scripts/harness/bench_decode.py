#!/usr/bin/env python3
"""Random-access decode cost of a LeRobot dataset's videos, per dataset root.

This is what the DataLoader pays per sample (GR00T: 1 frame x N cameras, random
timestamp), measured with lerobot's own decode_video_frames + pyav -- the exact
path training uses -- so a number here transfers to data_s in the train log.
Also reports the codec each dataset was encoded with (from meta/info.json), so
"which one is faster" and "which one is AV1" cannot be confused.

usage: bench_decode.py <root> [<root> ...]   (run with deps/.venv-ml/bin/python)
"""
import glob, json, random, sys, time
from lerobot.datasets.video_utils import decode_video_frames

N = int(__import__("os").environ.get("N", "150"))
random.seed(0)
for root in sys.argv[1:]:
    info = json.load(open(f"{root}/meta/info.json"))
    cams = [k for k, v in info["features"].items() if v["dtype"] == "video"]
    codec = info["features"][cams[0]]["info"].get("video.codec")
    fps = info["fps"]
    print(f"== {root}\n   codec={codec}  cams={len(cams)}  frames={info['total_frames']}")
    for cam in cams:
        files = sorted(glob.glob(f"{root}/videos/{cam}/**/*.mp4", recursive=True))
        # timestamps within the first video file; decode ONE frame per call like
        # the loader does for a 1-frame observation window.
        f = files[0]
        n_frames = info["total_frames"] if len(files) == 1 else 1000
        dur = n_frames / fps
        ts = [random.uniform(0.5, dur - 0.5) for _ in range(N)]
        decode_video_frames(f, [ts[0]], 1 / fps, backend="pyav")  # warm
        t0 = time.perf_counter()
        for t in ts:
            fr = decode_video_frames(f, [t], 1 / fps, backend="pyav")
        dt = (time.perf_counter() - t0) / N
        print(f"   {cam.split('.')[-1]:9} {dt*1000:7.1f} ms/frame   ({tuple(fr.shape)})")
