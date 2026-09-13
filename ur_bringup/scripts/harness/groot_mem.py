#!/usr/bin/env python3
"""V2 detail: where the 1.62 B trainable parameters actually are, and what that
costs in optimizer memory on a 32 GB card.

51.5% trainable contradicts the "backbone frozen, only projector + diffusion head"
reading of the config, so the split has to be measured per module rather than assumed.
"""

import sys

import torch
from huggingface_hub import snapshot_download

from lerobot.configs.types import FeatureType, PolicyFeature
from lerobot.policies.groot.configuration_groot import GrootConfig
from lerobot.policies.groot.modeling_groot import GrootPolicy
from lerobot.utils.constants import ACTION, OBS_STATE

BASE = snapshot_download("nvidia/GR00T-N1.7-3B")
FP32 = "--bf16" not in sys.argv

cfg = GrootConfig(base_model_path=BASE, model_params_fp32=FP32)
cfg.input_features = {
    "observation.images.exterior": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 240, 320)),
    "observation.images.wrist": PolicyFeature(type=FeatureType.VISUAL, shape=(3, 240, 320)),
    OBS_STATE: PolicyFeature(type=FeatureType.STATE, shape=(7,)),
}
cfg.output_features = {ACTION: PolicyFeature(type=FeatureType.ACTION, shape=(7,))}
print(f"model_params_fp32={cfg.model_params_fp32}  tune_llm={cfg.tune_llm} "
      f"tune_visual={cfg.tune_visual} tune_projector={cfg.tune_projector} "
      f"tune_diffusion_model={cfg.tune_diffusion_model} tune_vlln={cfg.tune_vlln}")

policy = GrootPolicy(cfg)

groups: dict[str, list[int]] = {}
dtypes: dict[str, int] = {}
for name, p in policy.named_parameters():
    parts = name.split(".")
    key = ".".join(parts[1:4]) if parts[0] == "model" else ".".join(parts[:3])
    key = ".".join(key.split(".")[:2])
    g = groups.setdefault(key, [0, 0])
    g[0] += p.numel()
    if p.requires_grad:
        g[1] += p.numel()
    dtypes[str(p.dtype)] = dtypes.get(str(p.dtype), 0) + p.numel()

print(f"\n{'module':40} {'total':>10} {'trainable':>10}")
tot = tr = 0
for k, (a, b) in sorted(groups.items(), key=lambda x: -x[1][0]):
    if a < 1e6:
        continue
    print(f"{k:40} {a/1e6:9.0f}M {b/1e6:9.0f}M")
    tot += a
    tr += b
print(f"{'TOTAL (>1M modules)':40} {tot/1e6:9.0f}M {tr/1e6:9.0f}M")
print(f"param dtypes: { {k: f'{v/1e6:.0f}M' for k, v in dtypes.items()} }")

# AdamW keeps two fp32-ish moments per trainable parameter, in the parameter dtype.
bpp = 4 if FP32 else 2
n_all = sum(p.numel() for p in policy.parameters())
n_tr = sum(p.numel() for p in policy.parameters() if p.requires_grad)
est = (n_all * bpp + n_tr * bpp + 2 * n_tr * bpp) / 2**30
print(f"\nstatic estimate: params {n_all*bpp/2**30:.1f} + grads {n_tr*bpp/2**30:.1f} "
      f"+ AdamW {2*n_tr*bpp/2**30:.1f} = {est:.1f} GiB  (activations NOT included; card is 31.8 GiB)")
