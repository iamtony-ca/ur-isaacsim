#!/usr/bin/env bash
# GR00T N1.7 finetune -- the real run. Every non-default argument below is here
# because a measurement or a source read said the default is wrong for us.
#
#   batch_size=32     throughput sweep: samples/s rises 16.6 -> 22.9 from batch
#                     8 -> 32 while VRAM moves only 25.4 -> 27.3 GB, because the
#                     25 GB floor is model + AdamW state, not activations. Small
#                     batches are pure loss. 48 is faster still (25.2 samples/s)
#                     but 29.3/32.6 GB leaves too little on a SHARED GPU -- another
#                     project already holds ~1.4 GB and an OOM 3 h in costs the run.
#   num_workers=2     sweep: 2/4/6 workers = 2.08/1.97/1.99 step/s (flat -> compute
#                     bound, not loader bound), and 8 workers DIES on /dev/shm
#                     ("unable to allocate shared memory", step 24) -- the exact
#                     failure that killed ACT (HISTORY.md 24). 2 is fastest and safest.
#   steps=10000       NVIDIA Isaac-GR00T reference finetune length (the deprecated
#                     max_steps/batch_size defaults in configuration_groot.py are
#                     10000/32). Measured 0.758 step/s -> 3.7 h, inside the 6 h budget.
#                     lerobot's own default of 100_000 would be 36.6 h.
#   max_steps=10000   *** num_warmup_steps = ceil(max_steps * warmup_ratio) is taken
#                     from THIS field, not from --steps. Measured: a 4000-step run
#                     with the default 10000 peaks the LR at 12.5% in instead of 5%.
#                     It happens to equal --steps here, but state it so the recipe
#                     stays correct if either number is ever changed.
#   model_params_fp32=false   fp32 is 29.8/31.8 GiB STATIC, i.e. fails regardless of
#                     batch size (HISTORY.md 40.5).
#   base_model_path=<local dir>   a repo id makes is_raw_groot_n1_7_checkpoint()
#                     return None and silently swaps the checkpoint's albumentations /
#                     state dropout / percentile / crop settings for lerobot defaults.
#                     Training still runs, just not the way N1.7 was pretrained.
#   use_relative_actions=true + exclude ["gripper"]
#                     N1.7 was PRETRAINED on relative action chunks (checkpoint
#                     processor_kwargs has use_relative_action: True) while the
#                     lerobot default is False -- plan_groot_n17.md 3 picks relative.
#                     Gripper stays absolute so grasp/release, a discrete event,
#                     does not dissolve into accumulated delta error.
#                     *** Pass exactly ["gripper"]. Matching is substring-based, so
#                     ["joint"] would invert the intent -- all 6 arm joints absolute
#                     and the gripper relative, with no warning. Verified by calling
#                     _infer_n1_7_action_groups directly.
#   save_freq=5000    default 20000 > our 10000, so only the final checkpoint would
#                     exist. 5000 buys one mid-run resume point.
#   push_to_hub=false / wandb.enable=false   [training] extra turns on hub validation;
#                     GrootConfig.report_to defaults to wandb.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(cd "$(dirname "$0")" && pwd)"
cd "$WS"
source "$WS/src/setup/ml_env.sh"

DS="$WS/outputs/lerobot_ds_240_v2"
REPO=tony/ur16e_pick_place_240_v2
OUT="$WS/outputs/groot_240_v2_abs"

[ -f "$DS/meta/info.json" ] || { echo "FAIL: no dataset at $DS"; exit 1; }

BASE=$(deps/.venv-ml/bin/python -c \
  "from huggingface_hub import snapshot_download; print(snapshot_download('nvidia/GR00T-N1.7-3B'))" 2>/dev/null)
[ -d "$BASE" ] || { echo "FAIL: could not resolve GR00T snapshot (HF token? gate?)"; exit 1; }
echo "== base:    $BASE"
echo "== dataset: $DS"
echo "== output:  $OUT"

# Isaac parks ~14 MiB of Fast-DDS in the 64 MiB /dev/shm and competes for the GPU.
pkill -f ur16e_isaac_ros2.py 2>/dev/null
sleep 2
echo "== start:   GPU $(nvidia-smi --query-gpu=memory.used --format=csv,noheader), shm $(df -h /dev/shm | tail -1 | awk '{print $3"/"$2}')"

( while :; do
    printf '%s %s\n' "$(nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader,nounits | tr -d ' ')" \
                     "$(df -k /dev/shm | tail -1 | awk '{print $3}')"
    sleep 30
  done ) > "$LOG/groot_train_abs_res.txt" &
RESPID=$!
trap 'kill $RESPID 2>/dev/null' EXIT

# OMP_NUM_THREADS=8: HISTORY.md 45.4 -- data_s is the main-process preprocessor, 7x slower at the default 20 threads.
OMP_NUM_THREADS=8 deps/.venv-ml/bin/lerobot-train \
  --policy.type=groot \
  --policy.base_model_path="$BASE" \
  --policy.model_params_fp32=false \
  --policy.max_steps=10000 \
  --policy.push_to_hub=false \
  --dataset.repo_id="$REPO" --dataset.root="$DS" \
  --steps=10000 --batch_size=32 --num_workers=2 \
  --save_freq=5000 --wandb.enable=false \
  --output_dir="$OUT"
rc=$?

echo "== rc=$rc"
echo "== peak VRAM $(awk -F, '{print $1}' "$LOG/groot_train_abs_res.txt" | sort -n | tail -1) MiB"
echo "== peak shm  $(awk '{print $2}' "$LOG/groot_train_abs_res.txt" | sort -n | tail -1) KiB"
ls -la "$OUT/checkpoints" 2>/dev/null
exit $rc
