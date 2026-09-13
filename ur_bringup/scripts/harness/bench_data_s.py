#!/usr/bin/env python3
"""Where does GR00T's data_s go?  lerobot's data_s = DataLoader wait + uint8->float
+ preprocessor(batch), reported as the MAX over the log window.  This times the
three pieces separately, with the same dataset/config/DataLoader as lerobot-train,
and repeats with the main process pinned to 1 torch thread (the DataLoader workers
already run with 1) to expose intra-op thread oversubscription.

usage: bench_data_s.py <dataset_root> [batch] [workers] [iters]   (deps/.venv-ml/bin/python)
"""
import os, sys, time, torch
from pathlib import Path
from huggingface_hub import snapshot_download
from lerobot.configs.train import TrainPipelineConfig
from lerobot.configs.default import DatasetConfig
from lerobot.policies.groot.configuration_groot import GrootConfig
from lerobot.datasets.factory import make_dataset
from lerobot.policies.factory import make_policy, make_pre_post_processors
from lerobot.utils.collate import lerobot_collate_fn

root = sys.argv[1]; B = int(sys.argv[2]) if len(sys.argv) > 2 else 32
W = int(sys.argv[3]) if len(sys.argv) > 3 else 2; IT = int(sys.argv[4]) if len(sys.argv) > 4 else 12
base = snapshot_download("nvidia/GR00T-N1.7-3B")
cfg = TrainPipelineConfig(
    dataset=DatasetConfig(repo_id="tony/x", root=root),
    policy=GrootConfig(base_model_path=base, model_params_fp32=False, push_to_hub=False),
    batch_size=B, num_workers=W, output_dir=Path(os.environ["S"]) / "bench_data_s_out", steps=10,
)
ds = make_dataset(cfg)
policy = make_policy(cfg=cfg.policy, ds_meta=ds.meta, rename_map=cfg.rename_map)
pre, _ = make_pre_post_processors(
    policy_cfg=cfg.policy, pretrained_path=None, dataset_stats=ds.meta.stats,
    preprocessor_overrides={"device_processor": {"device": "cuda"}})
del policy; torch.cuda.empty_cache()
collate = lerobot_collate_fn if ds.meta.has_language_columns else None
codec = ds.meta.info["features"]["observation.images.wrist"]["info"].get("video.codec")

def run(threads):
    torch.set_num_threads(threads)
    dl = torch.utils.data.DataLoader(ds, num_workers=W, batch_size=B, shuffle=True, pin_memory=True,
                                     drop_last=False, collate_fn=collate,
                                     prefetch_factor=2 if W else None, persistent_workers=bool(W))
    it = iter(dl); next(it)  # warm up workers
    tw = tf = tp = 0.0
    for _ in range(IT):
        a = time.perf_counter(); batch = next(it); b = time.perf_counter()
        for k in ds.meta.camera_keys:
            if k in batch and batch[k].dtype == torch.uint8:
                batch[k] = batch[k].to(dtype=torch.float32) / 255.0
        c = time.perf_counter(); batch = pre(batch); torch.cuda.synchronize(); d = time.perf_counter()
        tw += b - a; tf += c - b; tp += d - c
    print(f"{codec:5} batch={B} workers={W} main_threads={threads:2d}:  "
          f"loader_wait {tw/IT:.3f}s  to_float {tf/IT:.3f}s  preprocessor {tp/IT:.3f}s  "
          f"(sum {(tw+tf+tp)/IT:.3f}s = lerobot data_s, mean not max)", flush=True)
    del it, dl

run(torch.get_num_threads())
run(1)
