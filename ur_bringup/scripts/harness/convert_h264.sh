#!/usr/bin/env bash
# Re-convert il_raw_red_left_100 twice with h264 (crf 30 = lerobot default value,
# and crf 23 = ffmpeg's h264 default) so decode speed, PSNR and size can be
# compared against the existing AV1 conversion of the SAME raw frames.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"
cd "$WS"
RAW=outputs/il_raw_red_left_100
for spec in "h264_crf30:" "h264_crf23:--crf 23"; do
  tag=${spec%%:*}; extra=${spec#*:}
  OUT="outputs/lerobot_ds_red_left_100_$tag"
  rm -rf "$OUT"
  t0=$(date +%s)
  deps/.venv-ml/bin/python src/ur_bringup/scripts/raw_to_lerobot.py \
    --raw "$RAW" --repo-id "tony/ur16e_pick_place_red_left_100_$tag" --root "$OUT" \
    --vcodec h264 $extra 2>&1 | grep -E "video encoder|converted episode_0000(0|5)0|done ->|Error|error" 
  echo "== $tag: $(( $(date +%s) - t0 )) s, videos $(du -sh "$OUT/videos" 2>/dev/null | cut -f1)"
done
echo "== ALL DONE"
