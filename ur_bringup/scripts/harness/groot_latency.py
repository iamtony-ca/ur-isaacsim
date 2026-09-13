#!/usr/bin/env python3
"""V7: GR00T N1.7 chunk latency. Run with deps/.venv-ml/bin/python.

PASS CRITERION (plan_groot_n17.md 4): one 40-action chunk must be produced in
under 40/30 = 1.333 s, because the chunk is consumed at the dataset's 30 Hz. Miss
it and the robot runs out of actions before the next chunk lands -- which does not
look like "slow inference", it looks like the arm stuttering mid-motion, and it
would be easy to blame the policy instead of the clock.

Uses the SAME call chain as lerobot.async_inference.policy_server:

    policy_class.from_pretrained(path)
    make_pre_post_processors(config, pretrained_path=path, ...)
    preprocessor(obs) -> policy.predict_action_chunk(obs) -> postprocessor(...)

My first attempt called policy.select_action() after a bare from_pretrained, which
would have measured a DIFFERENT pipeline than the one that actually serves the
robot -- no preprocessor means no resize/crop, no tokenization, no percentile
normalization, and for this checkpoint no relative-action reconstruction either.
The number would have been optimistic and meaningless.

What this does NOT include, on purpose: gRPC transport, pickling, and the client's
queueing. Those are measured end-to-end from the server's own
"Preprocessing and inference took" line during the V8 rollouts. This isolates the
model so a failure can be attributed.

Timing hygiene:
  * torch.cuda.synchronize() around each call -- CUDA is async, so without it we
    time the kernel launch, not the work.
  * warmup calls are discarded: cudnn autotune and lazy CUDA module loads land
    there and are not steady state.
  * MAX is reported next to the median. A median under budget with a p100 over it
    still stutters, and a chunked policy stutters visibly.
"""
import argparse
import statistics
import time

import numpy as np
import torch


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ckpt", required=True, help="…/checkpoints/<n>/pretrained_model")
    ap.add_argument("--warmup", type=int, default=5)
    ap.add_argument("--iters", type=int, default=30)
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--actions-per-chunk", type=int, default=40)
    ap.add_argument("--task", default="put the red block on the left marker")
    a = ap.parse_args()

    from lerobot.policies.factory import get_policy_class, make_pre_post_processors

    print(f"loading {a.ckpt}")
    t0 = time.perf_counter()
    cls = get_policy_class("groot")
    policy = cls.from_pretrained(a.ckpt)
    dev = "cuda"
    policy.to(dev)
    policy.eval()
    pre, post = make_pre_post_processors(
        policy.config,
        pretrained_path=a.ckpt,
        preprocessor_overrides={"device_processor": {"device": dev}},
        postprocessor_overrides={"device_processor": {"device": dev}},
    )
    print(f"  loaded in {time.perf_counter() - t0:.1f}s")

    cfg = policy.config
    budget = a.actions_per_chunk / a.fps
    print(f"  chunk_size={cfg.chunk_size} n_action_steps={cfg.n_action_steps} "
          f"-> budget {budget:.3f}s for {a.actions_per_chunk} actions at {a.fps:g} Hz")

    # Build a raw observation from the policy's own declared features, so this
    # cannot drift from what the checkpoint expects. float32 HWC in [0,1] and a
    # plain state vector is what raw_observation_to_observation hands over.
    obs = {}
    for key, ft in cfg.input_features.items():
        if key.startswith("observation.images"):
            c, h, w = (ft.shape if ft.shape[0] in (1, 3)
                       else (ft.shape[2], ft.shape[0], ft.shape[1]))
            obs[key] = torch.from_numpy(
                np.random.rand(1, c, h, w).astype("float32"))
        else:
            obs[key] = torch.zeros(1, *ft.shape, dtype=torch.float32)
    obs["task"] = a.task
    print(f"  inputs: {[(k, tuple(v.shape)) for k, v in obs.items() if torch.is_tensor(v)]}")

    def one():
        policy.reset()
        torch.cuda.synchronize()
        t = time.perf_counter()
        with torch.no_grad():
            o = pre(dict(obs))
            chunk = policy.predict_action_chunk(o)
            if chunk.ndim != 3:
                chunk = chunk.unsqueeze(0)
            chunk = chunk[:, : a.actions_per_chunk, :]
        torch.cuda.synchronize()
        dt = time.perf_counter() - t
        return dt, tuple(chunk.shape)

    for _ in range(a.warmup):
        one()
    res = [one() for _ in range(a.iters)]
    ts = [r[0] for r in res]
    shape = res[0][1]

    med, mx, mn = statistics.median(ts), max(ts), min(ts)
    print(f"\n  action chunk shape: {shape}")
    print(f"  latency over {a.iters} calls (preprocess + inference):")
    print(f"    min {mn * 1000:7.1f} ms")
    print(f"    med {med * 1000:7.1f} ms")
    print(f"    max {mx * 1000:7.1f} ms")
    print(f"    budget {budget * 1000:.0f} ms")
    print(f"    headroom: median {budget / med:.1f}x   worst {budget / mx:.1f}x")
    if mx < budget:
        verdict = "PASS"
    elif med < budget:
        verdict = "MARGINAL -- median inside budget but worst case over; expect stutter"
    else:
        verdict = "FAIL"
    print(f"\n  V7 {verdict}")
    return 0 if med < budget else 1


if __name__ == "__main__":
    raise SystemExit(main())
