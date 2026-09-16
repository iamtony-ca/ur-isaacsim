#!/usr/bin/env bash
# Wait for the 320x240 collection, then convert to LeRobot v3.0.
# Overrides (v3 recollection, HISTORY.md 46): RAW= OUT= REPO= COLLECT_LOG= TAG=.
# Codec is raw_to_lerobot.py's default (h264 crf23 since 45.1); pass VCODEC= to change it.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(dirname "$0")"
RAW="${RAW:-$WS/outputs/il_raw_3task_240}"
OUT="${OUT:-$WS/outputs/lerobot_ds_240}"
REPO="${REPO:-tony/ur16e_pick_place_3task_240}"
COLLECT_LOG="${COLLECT_LOG:-$LOG/collect240.log}"
TAG="${TAG:-convert240}"
VCODEC="${VCODEC:-}"

for _ in $(seq 1 240); do
  grep -qE "ALL TASKS DONE|FAIL" "$COLLECT_LOG" 2>/dev/null && break
  sleep 15
done
if ! grep -q "ALL TASKS DONE" "$COLLECT_LOG" 2>/dev/null; then
  echo "collection did not finish cleanly:"; tail -3 "$COLLECT_LOG"; exit 1
fi
echo "== collection: $(ls -1d "$RAW"/episode_* 2>/dev/null | wc -l) episodes, $(du -sh "$RAW" | cut -f1)"

# Dry run first: it prints the episode/task breakdown, which is the cheapest way
# to catch a task string that did not land before spending minutes on encoding.
"$WS/deps/.venv-ml/bin/python" "$WS/src/ur_bringup/scripts/raw_to_lerobot.py" \
  --raw "$RAW" --dry-run 2>&1 | grep -vE "torchcodec|Traceback|^\s|^OSError|^The above|FFmpeg|^$"

rm -rf "$OUT"
"$WS/deps/.venv-ml/bin/python" "$WS/src/ur_bringup/scripts/raw_to_lerobot.py" \
  --raw "$RAW" --repo-id "$REPO" --root "$OUT" ${VCODEC:+--vcodec "$VCODEC"} \
  > "$LOG/${TAG}_run.log" 2>&1
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
    if k.startswith("observation.images"):
        print(f"   {k:32} codec {v['info']['video.codec']}  crf {v['info'].get('video.crf')}")
print("   tasks:", json.load(open(sys.argv[1] + "/meta/info.json")).get("total_tasks"))
PY
  du -sh "$OUT"
else
  echo "== CONVERSION FAILED"; tail -5 "$LOG/${TAG}_run.log"
fi
