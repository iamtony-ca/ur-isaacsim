#!/usr/bin/env bash
# GR00T N1.7 throughput sweep -- feeds the 6-hour budget gate (plan_groot_n17.md 4.1).
#
# WHY: the V3 smoke measured 2.24 step/s at batch 8 / 2 workers, but GPU util
# sampled 32/2/96/3/2/77% -- bursty, i.e. the 3B model is NOT the bottleneck, the
# DataLoader is. So step/s is a tunable here, not a constant, and the honest
# input to the 6-hour gate is the BEST rate we can reach, not the first one.
#
# THE CONSTRAINT is /dev/shm = 64 MiB (shared container, cannot be enlarged).
# float32 240x320x3 = 0.92 MiB; x2 cameras x batch = per-sample-batch cost, and
# workers prefetch 2 batches each. ACT died exactly here (HISTORY.md 24).
# So this sweep does NOT trust the arithmetic -- CLAUDE.md 2-C records that the
# arithmetic predicted failure at 320x240 and was WRONG. It measures, and it
# watches /dev/shm while measuring.
#
# 200 steps is enough: the rate stabilises by ~step 20 (weight load and cudnn
# autotune are done), and a dead DataLoader shows up in the first few steps.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(cd "$(dirname "$0")" && pwd)"
BATCH=${BATCH:-8}
STEPS=${STEPS:-200}
DS="$WS/outputs/lerobot_ds_240_v2"
REPO=tony/ur16e_pick_place_240_v2
cd "$WS"
source "$WS/src/setup/ml_env.sh"

BASE=$(deps/.venv-ml/bin/python -c \
  "from huggingface_hub import snapshot_download; print(snapshot_download('nvidia/GR00T-N1.7-3B'))" 2>/dev/null)
[ -d "$BASE" ] || { echo "FAIL: could not resolve GR00T snapshot"; exit 1; }

OUT="$LOG/groot_sweep_results.txt"
: > "$OUT"
printf '%-4s %-8s %-10s %-12s %-10s %-12s %s\n' \
  workers batch "step/s" "peak VRAM" "peak shm" "100k steps" note | tee -a "$OUT"

for W in "$@"; do
  log="$LOG/groot_sweep_w${W}.log"
  vram="$LOG/groot_sweep_vram_w${W}.txt"
  shm="$LOG/groot_sweep_shm_w${W}.txt"
  rm -rf "$WS/outputs/groot_sweep_w$W"

  # Sample shm too: the failure we care about is "worker killed", and the
  # evidence for how close we came is the shm high-water mark, not the exit code.
  ( while :; do nvidia-smi --query-gpu=memory.used --format=csv,noheader,nounits; sleep 2; done ) > "$vram" &
  vpid=$!
  ( while :; do df -k /dev/shm | tail -1 | awk '{print $3}'; sleep 1; done ) > "$shm" &
  spid=$!

  deps/.venv-ml/bin/lerobot-train \
    --policy.type=groot --policy.push_to_hub=false \
    --policy.base_model_path="$BASE" \
    --policy.model_params_fp32=false \
    --dataset.repo_id="$REPO" --dataset.root="$DS" \
    --steps="$STEPS" --batch_size="$BATCH" --num_workers="$W" \
    --save_checkpoint=false --wandb.enable=false \
    --output_dir="$WS/outputs/groot_sweep_w$W" > "$log" 2>&1
  rc=$?
  kill "$vpid" "$spid" 2>/dev/null

  pv=$(sort -n "$vram" | tail -1)
  ps=$(sort -n "$shm" | tail -1)
  ps_mib=$(( ${ps:-0} / 1024 ))

  if [ "$rc" -ne 0 ]; then
    # Name the shm death explicitly -- it is the one failure that looks like a
    # random crash but is actually a capacity limit.
    note="rc=$rc"
    grep -qiE "unable to allocate|shared memory|DataLoader worker.*killed|bus error" "$log" && note="SHM DEATH"
    grep -qiE "out of memory|CUDA out of memory" "$log" && note="CUDA OOM"
    printf '%-4s %-8s %-10s %-12s %-10s %-12s %s\n' \
      "$W" "$BATCH" "-" "${pv:-?} MiB" "${ps_mib} MiB" "-" "$note" | tee -a "$OUT"
    continue
  fi

  # Take the LAST progress-bar rate: the first few steps include cudnn autotune.
  r=$(tr '\r' '\n' < "$log" | grep -oE "[0-9.]+ ?step/s|[0-9.]+ ?s/step" | tail -1)
  sps=$(printf '%s' "$r" | grep -oE "^[0-9.]+")
  case "$r" in
    *s/step) sps=$(deps/.venv-ml/bin/python -c "print(f'{1/$sps:.3f}')") ;;
  esac
  h100k=$(deps/.venv-ml/bin/python -c "print(f'{100000/$sps/3600:.1f} h')" 2>/dev/null || echo "?")
  printf '%-4s %-8s %-10s %-12s %-10s %-12s %s\n' \
    "$W" "$BATCH" "${sps:-?}" "${pv:-?} MiB" "${ps_mib} MiB" "$h100k" "ok" | tee -a "$OUT"
done

echo
echo "results: $OUT"
