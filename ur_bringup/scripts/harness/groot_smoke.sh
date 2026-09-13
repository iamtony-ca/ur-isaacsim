#!/usr/bin/env bash
# GR00T N1.7 smoke: V1-V5 of docs/plan_groot_n17.md 4.
#
# Answers, before committing to a long run:
#   V1 does it build, and does it take BOTH cameras (no drop/mismatch warning)
#   V2 peak VRAM  -- 3B partial finetune on a single 32 GB card is unverified
#   V3 step/s     -- feeds the 6-hour budget gate the user set
#   V4 /dev/shm   -- the failure mode that killed ACT DataLoader workers
#   V5 absolute vs relative actions (4 of the plan) -- does the relative path build
#
# No checkpoint is written (--save_checkpoint=false): a 3B save is GBs and the
# smoke is about numbers, not weights.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(cd "$(dirname "$0")" && pwd)"
BATCH=${1:-8}
STEPS=${2:-200}
WORKERS=${3:-2}
DS="$WS/outputs/lerobot_ds_240_v2"
cd "$WS"
source "$WS/src/setup/ml_env.sh"

[ -f "$DS/meta/info.json" ] || { echo "FAIL: no dataset at $DS"; exit 1; }

# The backbone tokenizer lives in the GATED nvidia/Cosmos-Reason2-2B repo (plan_groot_n17.md 8.1).
# Fail here with the fix rather than 60 s into a weight load with a 401 traceback.
if [ -z "${HF_TOKEN:-}" ] && [ ! -f "${HF_HOME:?}/token" ] && [ ! -f "${HF_HOME}/stored_tokens" ]; then
  echo "FAIL: no HF credentials. nvidia/Cosmos-Reason2-2B is gated."
  echo "      1) accept the licence at https://huggingface.co/nvidia/Cosmos-Reason2-2B"
  echo "      2) export HF_TOKEN=hf_...   (read token)"
  exit 1
fi

# base_model_path must be a LOCAL DIR: a bare repo id makes
# _load_n1_7_checkpoint_processor_assets() return None and the checkpoint's own
# preprocessing (albumentations, state dropout, percentiles, crop) is silently
# replaced by lerobot defaults -- training still runs, just not the way N1.7 was
# pretrained (plan_groot_n17.md 2.0). snapshot_download returns the cached path without refetching.
BASE=$(deps/.venv-ml/bin/python -c \
  "from huggingface_hub import snapshot_download; print(snapshot_download('nvidia/GR00T-N1.7-3B'))" 2>/dev/null)
[ -d "$BASE" ] || { echo "FAIL: could not resolve GR00T snapshot"; exit 1; }
echo "== base: $BASE"

# Isaac parks ~14 MiB of Fast-DDS in the 64 MiB /dev/shm and competes for the GPU.
pkill -f ur16e_isaac_ros2.py 2>/dev/null
sleep 2
echo "== free: GPU $(nvidia-smi --query-gpu=memory.used --format=csv,noheader), shm $(df -h /dev/shm | tail -1 | awk '{print $3"/"$2}')"

run_one() {  # $1=tag  $2...=extra policy args
  local tag=$1; shift
  local log="$LOG/groot_smoke_$tag.log"
  local vram="$LOG/groot_vram_$tag.txt"
  echo "== [$tag] batch=$BATCH steps=$STEPS workers=$WORKERS  $*"
  rm -rf "$WS/outputs/groot_smoke_$tag"

  ( while :; do nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits; sleep 2; done ) > "$vram" &
  local vpid=$!
  local t0=$(date +%s)
  deps/.venv-ml/bin/lerobot-train \
    --policy.type=groot --policy.push_to_hub=false \
    --policy.base_model_path="$BASE" \
    --policy.model_params_fp32=false \
    --dataset.repo_id=tony/ur16e_pick_place_240_v2 --dataset.root="$DS" \
    --steps="$STEPS" --batch_size="$BATCH" --num_workers="$WORKERS" \
    --save_checkpoint=false --wandb.enable=false \
    --output_dir="$WS/outputs/groot_smoke_$tag" \
    "$@" > "$log" 2>&1
  local rc=$?
  local t1=$(date +%s)
  kill "$vpid" 2>/dev/null

  local peak; peak=$(sort -n "$vram" 2>/dev/null | tail -1)
  local elapsed=$((t1 - t0))
  if [ "$rc" -ne 0 ]; then
    echo "   BUILD/RUN FAILED (rc=$rc) after ${elapsed}s"
    grep -oE "(RuntimeError|OSError|ValueError|TypeError|KeyError|AssertionError)[:.].*" "$log" | tail -3 | cut -c1-160
    tail -3 "$log" | cut -c1-160
    return 1
  fi
  # step/s from lerobot's own progress line, not from wall clock: wall clock
  # includes ~1 min of 3B weight loading, which is not per-step cost.
  local last; last=$(grep -oE "[0-9.]+ ?s/step|[0-9.]+ ?step/s" "$log" | tail -1)
  echo "   OK  wall ${elapsed}s   peak VRAM ${peak:-?} MiB   rate ${last:-?}"
  grep -oE "loss:[0-9.eE+-]+" "$log" | head -1 | sed 's/^/   first /'
  grep -oE "loss:[0-9.eE+-]+" "$log" | tail -1 | sed 's/^/   last  /'
  # V1: the camera question. These are warnings, so a silent log is the pass.
  if grep -qiE "video modality keys|Dropping camera|falling back to feeding all cameras" "$log"; then
    echo "   !! CAMERA WARNING:"; grep -iE "video modality keys|Dropping camera|falling back" "$log" | head -3 | cut -c1-200
  else
    echo "   cameras: no drop/mismatch warning"
  fi
  echo "   shm now $(df -h /dev/shm | tail -1 | awk '{print $3"/"$2}')"
  return 0
}

run_one abs
echo
run_one rel --policy.use_relative_actions=true --policy.relative_exclude_joints='["gripper"]'
echo
echo "== logs: $LOG/groot_smoke_{abs,rel}.log"
