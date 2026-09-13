#!/usr/bin/env python3
"""PSNR of a converted dataset's frames against the raw JPEG source.

Both AV1 and h264 are lossy on top of the recorder's JPEG. Switching codec for
decode speed is only acceptable if the pixels the policy sees are not visibly
worse, so compare each candidate against the SAME source frames. Reports mean
PSNR per camera over a fixed sample of (episode, frame) pairs.

usage: bench_quality.py <raw_dir> <root> [<root> ...]
"""
import glob, json, os, random, sys
import numpy as np
from PIL import Image
from lerobot.datasets.video_utils import decode_video_frames

raw = sys.argv[1]; roots = sys.argv[2:]
random.seed(1)
eps = sorted(glob.glob(f"{raw}/episode_*"))[:20]
pairs = []
for e in eps:
    m = json.load(open(f"{e}/meta.json"))
    n = m.get("num_frames") or len(glob.glob(f"{e}/*/*.jpg")) // max(1, len(m.get("video_keys", [1])))
    if n: pairs.append((e, random.randrange(5, max(6, n - 5))))
def psnr(a, b):
    mse = np.mean((a.astype(np.float64) - b.astype(np.float64)) ** 2)
    return 99.0 if mse == 0 else 10 * np.log10(255.0 ** 2 / mse)
for root in roots:
    info = json.load(open(f"{root}/meta/info.json")); fps = info["fps"]
    cams = [k for k, v in info["features"].items() if v["dtype"] == "video"]
    codec = info["features"][cams[0]]["info"].get("video.codec")
    ep_meta = {}
    import pyarrow.parquet as pq
    for p in glob.glob(f"{root}/meta/episodes/**/*.parquet", recursive=True):
        t = pq.read_table(p).to_pydict()
        for i, ei in enumerate(t["episode_index"]):
            ep_meta[ei] = {k: t[k][i] for k in t}
    print(f"== {root}  codec={codec}")
    for cam in cams:
        name = cam.split(".")[-1]; vals = []
        for ei, (e, fi) in enumerate(pairs):
            jp = sorted(glob.glob(f"{e}/frames/video.{name}/*.jpg"))
            if fi >= len(jp): continue
            src = np.asarray(Image.open(jp[fi]).convert("RGB"))
            em = ep_meta.get(ei)
            if em is None: continue
            vf = sorted(glob.glob(f"{root}/videos/{cam}/**/*.mp4", recursive=True))[em[f"videos/{cam}/file_index"]]
            t = em[f"videos/{cam}/from_timestamp"] + fi / fps
            fr = decode_video_frames(vf, [t], 1 / fps, backend="pyav")[0]
            dec = (fr.permute(1, 2, 0).numpy() * 255).round().astype(np.uint8)
            if dec.shape != src.shape: continue
            vals.append(psnr(src, dec))
        if vals: print(f"   {name:9} PSNR vs raw JPEG: mean {np.mean(vals):5.2f} dB  min {np.min(vals):5.2f}  (n={len(vals)})")
        else: print(f"   {name:9} (no comparable frames found)")
