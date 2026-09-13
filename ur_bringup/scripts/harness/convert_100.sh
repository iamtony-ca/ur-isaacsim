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
RAW="$WS/outputs/il_raw_red_left_100"
OUT="$WS/outputs/lerobot_ds_red_left_100"

for _ in $(seq 1 240); do
  grep -qE "ALL TASKS DONE|FAIL" "$LOG/collect_100.log" 2>/dev/null && break
  sleep 15
done
if ! grep -q "ALL TASKS DONE" "$LOG/collect_100.log" 2>/dev/null; then
  echo "collection did not finish cleanly:"; tail -3 "$LOG/collect_100.log"; exit 1
fi
echo "== collection: $(ls -1d "$RAW"/episode_* 2>/dev/null | wc -l) episodes, $(du -sh "$RAW" | cut -f1)"

# Dry run first: it prints the episode/task breakdown, which is the cheapest way
# to catch a task string that did not land before spending minutes on encoding.
"$WS/deps/.venv-ml/bin/python" "$WS/src/ur_bringup/scripts/raw_to_lerobot.py" \
  --raw "$RAW" --dry-run 2>&1 | grep -vE "torchcodec|Traceback|^\s|^OSError|^The above|FFmpeg|^$"

rm -rf "$OUT"
"$WS/deps/.venv-ml/bin/python" "$WS/src/ur_bringup/scripts/raw_to_lerobot.py" \
  --raw "$RAW" --repo-id tony/ur16e_pick_place_red_left_100 --root "$OUT" \
  > "$LOG/convert_100_run.log" 2>&1
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
  echo "== CONVERSION FAILED"; tail -5 "$LOG/convert_100_run.log"
fi
