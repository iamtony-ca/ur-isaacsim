#!/usr/bin/env bash
# Does OMP_NUM_THREADS (= torch intra-op threads in the main process) cut data_s
# in the real lerobot-train loop?  bench_data_s.py says preprocessor 0.86 s at 20
# threads vs 0.32 s at 1; this checks the number the user waits on, incl. updt_s.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...); S="$(cd "$(dirname "$0")" && pwd)"
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
STEPS=${1:-60}; cd "$WS"; source "$WS/src/setup/ml_env.sh"
BASE=$(deps/.venv-ml/bin/python -c "from huggingface_hub import snapshot_download; print(snapshot_download('nvidia/GR00T-N1.7-3B'))" 2>/dev/null)
for T in ${THREADS:-1 4}; do
  log="$LOG/groot_threads_$T.log"; rm -rf "$WS/outputs/groot_threads_smoke"; t0=$(date +%s)
  OMP_NUM_THREADS=$T deps/.venv-ml/bin/lerobot-train --policy.type=groot --policy.push_to_hub=false \
    --policy.base_model_path="$BASE" --policy.model_params_fp32=false \
    --dataset.repo_id=tony/x --dataset.root="$WS/outputs/lerobot_ds_red_left_100" \
    --steps="$STEPS" --policy.max_steps="$STEPS" --batch_size=32 --num_workers=2 \
    --log_freq=10 --save_checkpoint=false --wandb.enable=false \
    --output_dir="$WS/outputs/groot_threads_smoke" > "$log" 2>&1
  echo "== OMP_NUM_THREADS=$T rc=$? wall=$(( $(date +%s) - t0 ))s  $(grep -oE '[0-9]+/[0-9]+ \[[^]]*\]' "$log" | tail -1)"
  grep -oE "step:[0-9]+.*" "$log" | grep -oE "step:[0-9]+|updt_s:[0-9.]+|data_s:[0-9.]+|smp/s:[0-9]+" | paste - - - - | tail -3 | sed 's/^/   /'
done
rm -rf "$WS/outputs/groot_threads_smoke"
