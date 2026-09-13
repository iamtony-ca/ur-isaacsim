#!/usr/bin/env bash
# Does h264 actually cut GR00T's data_s? Same model, same batch/workers, same
# 100 episodes, only the container codec differs. bench_decode.py measures
# decode_video_frames in isolation; this measures the number the user waits on.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(cd "$(dirname "$0")" && pwd)"
STEPS=${1:-60}; BATCH=${2:-32}; WORKERS=${3:-2}
cd "$WS"; source "$WS/src/setup/ml_env.sh"
BASE=$(deps/.venv-ml/bin/python -c "from huggingface_hub import snapshot_download; print(snapshot_download('nvidia/GR00T-N1.7-3B'))" 2>/dev/null)
[ -d "$BASE" ] || { echo "FAIL: no GR00T snapshot"; exit 1; }
for tag in av1:lerobot_ds_red_left_100 h264_crf23:lerobot_ds_red_left_100_h264_crf23 h264_crf30:lerobot_ds_red_left_100_h264_crf30; do
  name=${tag%%:*}; ds=${tag#*:}
  log="$LOG/groot_codec_$name.log"; rm -rf "$WS/outputs/groot_codec_smoke"
  t0=$(date +%s)
  deps/.venv-ml/bin/lerobot-train --policy.type=groot --policy.push_to_hub=false \
    --policy.base_model_path="$BASE" --policy.model_params_fp32=false \
    --dataset.repo_id=tony/x --dataset.root="$WS/outputs/$ds" \
    --steps="$STEPS" --policy.max_steps="$STEPS" --batch_size="$BATCH" --num_workers="$WORKERS" \
    --log_freq=10 --save_checkpoint=false --wandb.enable=false \
    --output_dir="$WS/outputs/groot_codec_smoke" > "$log" 2>&1
  rc=$?; t1=$(date +%s)
  echo "== $name  rc=$rc  wall=$((t1-t0))s  codec=$(python3 -c "import json;print(json.load(open('$WS/outputs/$ds/meta/info.json'))['features']['observation.images.wrist']['info'].get('video.codec'))")"
  grep -oE "step:[0-9]+.*" "$log" | grep -oE "step:[0-9]+|data_s:[0-9.]+|updt_s:[0-9.]+" | paste - - - | tail -4 | sed 's/^/   /'
done
rm -rf "$WS/outputs/groot_codec_smoke"
