#!/usr/bin/env bash
# ACT on the v3 set: 50 episodes collected with the optimised state machine, and
# converted with NO idle thinning (raw idle ~9.8%).
#
# What this checks: act_red_left_thin got 8/8 from post-processed data. If v3
# matches that WITHOUT post-processing, the fix lives in the state machine and
# transfers to real teleop data, where post-hoc thinning is not an option.
#
# Loss is not comparable across v2/thin/v3 -- different frame mixes, different
# floors. Judge by the rollout (HISTORY.md 32.5).
#
# ONE task on purpose: ACT has no language input (HISTORY.md 30), so a mixed set
# is contradictory supervision. This is the first run where "did it learn the
# task?" is a question the setup can actually answer -- the wide randomisation
# means a memorised trajectory will not score well.
#
# 60000 steps at the measured ~8 step/s = ~2 h. ALOHA trained ACT for 100k+ on
# 50 episodes/task; 60k is the point where the loss curve has usually flattened,
# and the checkpoint is what gets rolled out, not the loss.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(dirname "$0")"
DS="$WS/outputs/lerobot_ds_wrist_only"
cd "$WS"
for _ in $(seq 1 160); do
  [ -f "$DS/meta/info.json" ] && ! ps -eo args --no-headers | grep -q "[r]aw_to_lerobot" && break
  sleep 15
done
ps -eo args --no-headers | grep -q "[r]aw_to_lerobot" && { echo "FAIL: conversion still running"; exit 1; }
[ -f "$DS/meta/info.json" ] || { echo "FAIL: no dataset"; exit 1; }

deps/.venv-ml/bin/python - "$DS" <<'PY'
import json, sys
i = json.load(open(sys.argv[1] + "/meta/info.json"))
print(f"dataset: {i['total_episodes']} ep, {i['total_frames']} frames, fps {i['fps']}")
for k, v in i["features"].items():
    if k.startswith(("observation", "action")):
        print(f"   {k:32} {v['dtype']:6} {tuple(v['shape'])}")
PY

# num_workers=4, measured (bench_workers.sh, Isaac stopped, 400 steps each):
#     8 -> died on /dev/shm      6 -> died on /dev/shm
#     4 -> 12.52 step/s          2 -> 7.88 step/s        0 -> 1.36 step/s
# The ladder still falls back, because the /dev/shm failure is intermittent and
# depends on what else holds shared memory: Isaac's ROS stack alone parks ~14 MiB
# of Fast-DDS segments in the 64 MiB /dev/shm, which is what pushed an earlier
# num_workers=2 run over the edge. Stop Isaac before training.
rm -rf "$WS/outputs/act_wrist_only"
for W in 4 2 0; do
  echo "== num_workers=$W"
  UR_WS_TORCH_SHM_FIX=1 deps/.venv-ml/bin/lerobot-train \
    --policy.type=act --policy.push_to_hub=false \
    --dataset.repo_id=tony/ur16e_pick_place_wrist_only --dataset.root="$DS" \
    --steps=60000 --batch_size=32 --num_workers="$W" \
    --output_dir="$WS/outputs/act_wrist_only" > "$LOG/train_w1_w$W.log" 2>&1
  if find "$WS/outputs/act_wrist_only" -path "*pretrained_model/config.json" 2>/dev/null | grep -q .; then
    echo "   DONE (workers=$W)"
    grep -oE "[0-9]+/60000 \[[0-9:]+<[0-9:]+, *[0-9.]+s/step\]" "$LOG/train_w1_w$W.log" | tail -1
    grep -oE "loss:[0-9.]+" "$LOG/train_w1_w$W.log" | tail -1
    break
  fi
  echo "   failed: $(grep -oE '(RuntimeError|OSError|ValueError).*' "$LOG/train_w1_w$W.log" | tail -1 | cut -c1-70)"
  rm -rf "$WS/outputs/act_wrist_only"
done
