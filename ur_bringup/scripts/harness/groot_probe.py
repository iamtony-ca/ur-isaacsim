#!/usr/bin/env python3
"""What can be verified WITHOUT the gated Cosmos-Reason2-2B tokenizer.

The gate blocks exactly one thing: _build_n1_7_processor() (tokenizer + image/video
processor). Everything else -- backbone architecture, backbone weights, modality
resolution, stats -- is local. So V1 (do both cameras get in?) and V5 (does the
relative-action path build?) can be answered now, using upstream functions rather
than by reading the source and hoping.

Also loads the 3B model to get a floor on V2 (VRAM), which is NOT the training peak.
"""

import json
import sys

import torch

from lerobot.datasets.lerobot_dataset import LeRobotDatasetMetadata
from lerobot.policies.groot.configuration_groot import GrootConfig
from lerobot.policies.groot import processor_groot as pg
from lerobot.utils.constants import ACTION, OBS_STATE

from huggingface_hub import snapshot_download

# base_model_path MUST be a local directory: _load_n1_7_checkpoint_processor_assets()
# does Path(...).is_dir() and returns None for a bare repo id, silently discarding the
# checkpoint's preprocessing settings (albumentations, state dropout, percentiles, crop).
BASE = snapshot_download("nvidia/GR00T-N1.7-3B")

import os, sys
ROOT = sys.argv[1] if len(sys.argv) > 1 else os.path.join(os.environ.get("UR_WS", os.path.abspath(os.path.join(os.path.dirname(__file__), "../../../.."))), "outputs/lerobot_ds_240_v2")
REPO = "tony/ur16e_pick_place_240_v2"


def build_cfg(relative: bool) -> GrootConfig:
    from lerobot.configs.types import FeatureType, PolicyFeature

    cfg = GrootConfig(
        base_model_path=BASE,
        use_relative_actions=relative,
        relative_exclude_joints=["gripper"] if relative else [],
    )
    cfg.input_features = {
        "observation.images.exterior": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 240, 320)),
        "observation.images.wrist": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 240, 320)),
        OBS_STATE: PolicyFeature(type=FeatureType.STATE, shape=(7,)),
    }
    cfg.output_features = {ACTION: PolicyFeature(type=FeatureType.ACTION, shape=(7,))}
    return cfg


meta = LeRobotDatasetMetadata(REPO, root=ROOT)
print(f"dataset: {meta.total_episodes} ep, {meta.total_frames} frames, {len(meta.tasks)} tasks")
print(f"  tasks: {list(meta.tasks.index)[:5]}")

# ---- V1: which cameras does GR00T actually consume? -------------------------
keys = pg._resolve_visual_modality_keys_from_dataset_meta(meta)
print(f"\n[V1] dataset-meta visual keys      -> {keys}")

cfg = build_cfg(relative=False)
assets = pg._load_n1_7_checkpoint_processor_assets(cfg)
print(f"[V1] checkpoint video_modality_keys -> {assets.video_modality_keys}  (None => dataset fallback)")
print(f"[V1] checkpoint has stats for '{cfg.embodiment_tag}' -> {pg.has_modality_stats(assets.stats)}")
print(f"[V1] checkpoint use_relative_action -> {assets.use_relative_action}")
print(f"[V1] image target/crop              -> {assets.image_target_size} / {assets.image_crop_size}")

# _ordered_image_keys is the function that decides. Ask it directly.
step = pg.GrootN17PackInputsStep(video_modality_keys=assets.video_modality_keys)
obs = {f"observation.images.{k}": None for k in keys}
print(f"[V1] _ordered_image_keys(obs)       -> {step._ordered_image_keys(obs)}")

# ---- V5: does the relative-action path build from our dataset? --------------
cfg_rel = build_cfg(relative=True)
try:
    stats = pg._make_relative_action_training_stats_from_dataset_meta(cfg_rel, meta)
    rel = pg._build_n1_7_relative_action_processor_assets(cfg_rel, stats, meta, base_assets=assets)
    if rel is None:
        print("\n[V5] relative assets -> None  (would raise)")
    else:
        mc = rel.modality_config
        print(f"\n[V5] relative build OK")
        print(f"     video keys      {rel.video_modality_keys}")
        print(f"     state groups    {mc['state']['modality_keys']}")
        print(f"     action reps     {[c['rep'] for c in mc['action']['action_configs']]}")
        print(f"     action horizon  {rel.max_action_horizon} (valid {rel.valid_action_horizon})")
        print(f"     has stats       {pg.has_modality_stats(rel.stats)}")
except Exception as e:  # noqa: BLE001
    print(f"\n[V5] relative build FAILED: {type(e).__name__}: {e}")

# ---- V2 floor: load the 3B and measure resident VRAM ------------------------
if "--model" in sys.argv:
    from lerobot.policies.groot.modeling_groot import GrootPolicy

    torch.cuda.reset_peak_memory_stats()
    policy = GrootPolicy(cfg)
    policy.to("cuda")
    n_all = sum(p.numel() for p in policy.parameters())
    n_tr = sum(p.numel() for p in policy.parameters() if p.requires_grad)
    print(f"\n[V2] params total {n_all/1e9:.2f} B, trainable {n_tr/1e6:.0f} M ({100*n_tr/n_all:.1f}%)")
    print(f"[V2] weights resident {torch.cuda.max_memory_allocated()/2**20:.0f} MiB "
          f"(floor only -- training peak adds activations+optimizer)")
