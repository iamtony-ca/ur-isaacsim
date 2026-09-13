#!/usr/bin/env bash
# Convert the v3 single-task set. NO --max-idle-run: the optimised state machine
# leaves only ~9.8% stationary frames at the source (v2 was 65.8%), so the
# post-processing that rescued v2 is no longer load-bearing. That matters beyond
# tidiness -- on real hardware there is no post-hoc thinning of a teleop session,
# so the pipeline has to work on raw data (HISTORY.md 31/32).
# One task only -- ACT cannot read the instruction (HISTORY.md 30).
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(dirname "$0")"
RAW="$WS/outputs/il_raw_wrist_only"
OUT="$WS/outputs/lerobot_ds_wrist_only"

# Do NOT wait for the collector's "ALL TASKS DONE". The caller stops it as soon
# as it has TARGET episodes, so that line never appears -- this loop sat 34 min
# waiting for a message that could not come. Check the ARTEFACT instead: what the
# conversion needs is episodes on disk, and the caller has already counted them.
n=$(ls -1d "$RAW"/episode_* 2>/dev/null | wc -l)
[ "$n" -ge 1 ] || { echo "no episodes under $RAW"; exit 1; }
echo "== collection: $n episodes, $(du -sh "$RAW" | cut -f1)"

# Dry run first: it prints the episode/task breakdown, which is the cheapest way
# to catch a task string that did not land before spending minutes on encoding.
"$WS/deps/.venv-ml/bin/python" "$WS/src/ur_bringup/scripts/raw_to_lerobot.py" \
  --raw "$RAW" --dry-run 2>&1 | grep -vE "torchcodec|Traceback|^\s|^OSError|^The above|FFmpeg|^$"

rm -rf "$OUT"
"$WS/deps/.venv-ml/bin/python" "$WS/src/ur_bringup/scripts/raw_to_lerobot.py" \
  --raw "$RAW" --repo-id tony/ur16e_pick_place_wrist_only --root "$OUT" \
  > "$LOG/convert_w1_run.log" 2>&1
# Judge by the artefact, not by log text: the torchcodec traceback is the
# documented harmless one and matching on "Error" has cried wolf before.
if [ -f "$OUT/meta/info.json" ]; then
  echo "== CONVERSION DONE"
  "$WS/deps/.venv-ml/bin/python" - "$OUT" <<'PY'
import json, sys
i = json.load(open(sys.argv[1] + "/meta/info.json"))
print(f"   episodes {i['total_episodes']}  frames {i['total_frames']}  fps {i['fps']}")
for k, v in i["features"].items():
    if k.startswith(("observation", "action")):
        print(f"   {k:32} {v['dtype']:6} {tuple(v['shape'])}")
PY
  du -sh "$OUT"
else
  echo "== CONVERSION FAILED"; tail -5 "$LOG/convert_w1_run.log"
fi
