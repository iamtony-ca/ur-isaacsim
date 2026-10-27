#!/usr/bin/env bash
# Convert the single-task red->left set (default il_raw_red_left_100$GRASP_SFX; the
# final 190-ep set: RAW=.../il_raw_red_left_190_phys DS=.../lerobot_ds_red_left_190_phys
# REPO=tony/ur16e_pick_place_red_left_190_phys, HISTORY.md 58.5).
# Written for the v3 set. NO --max-idle-run: the optimised state machine
# leaves only ~9.8% stationary frames at the source (v2 was 65.8%), so the
# post-processing that rescued v2 is no longer load-bearing. That matters beyond
# tidiness -- on real hardware there is no post-hoc thinning of a teleop session,
# so the pipeline has to work on raw data (HISTORY.md 31/32).
# One task only -- ACT cannot read the instruction (HISTORY.md 30).
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(dirname "$0")"
source "$S/grasp_mode.sh"   # GRASP_MODE=physical (default) | attach -- HISTORY.md 53
# Overridable for other sets (HISTORY.md 58); the defaults are the original set's.
# The destination is DS= (not OUT= as in convert240.sh); without it the ORIGINAL
# dataset is the one wiped below.
[ -n "${OUT:-}" ] && [ -z "${DS:-}" ] && { echo "FAIL: convert_100.sh takes DS=, not OUT="; exit 2; }
[ -n "${RAW:-}${REPO:-}" ] && [ -z "${DS:-}" ] && { echo "FAIL: RAW/REPO given but DS unset"; exit 2; }
RAW="${RAW:-$WS/outputs/il_raw_red_left_100$GRASP_SFX}"
OUT="${DS:-$WS/outputs/lerobot_ds_red_left_100$GRASP_SFX}"
REPO="${REPO:-tony/ur16e_pick_place_red_left_100$GRASP_SFX}"

# Wait for the collector PROCESS, then judge by what is on disk. This used to wait
# on $LOG/collect_100.log, which collect_100.sh never writes itself -- unless the
# caller happened to redirect into exactly that name it timed out and exited 1.
for _ in $(seq 1 960); do
  pgrep -f "[c]ollect_100.sh" > /dev/null || break
  sleep 15
done
pgrep -f "[c]ollect_100.sh" > /dev/null && { echo "FAIL: collect_100.sh still running after 4 h"; exit 1; }
n=$(ls -1d "$RAW"/episode_* 2>/dev/null | wc -l)
[ "$n" -ge 1 ] || { echo "FAIL: no episodes under $RAW"; exit 1; }
echo "== collection: $(ls -1d "$RAW"/episode_* 2>/dev/null | wc -l) episodes, $(du -sh "$RAW" | cut -f1)"

# Dry run first: it prints the episode/task breakdown, which is the cheapest way
# to catch a task string that did not land before spending minutes on encoding.
"$WS/deps/.venv-ml/bin/python" "$WS/src/ur_bringup/scripts/raw_to_lerobot.py" \
  --raw "$RAW" --dry-run 2>&1 | grep -vE "torchcodec|Traceback|^\s|^OSError|^The above|FFmpeg|^$"

# A wipe target must be one of our artefact dirs (HISTORY.md 58 review).
_wipe_ok() { case "$1" in */outputs/il_raw_*|*/outputs/lerobot_ds_*|*/outputs/act_*) return 0;; esac
             echo "FAIL: refusing to wipe '$1' (not outputs/{il_raw,lerobot_ds,act}_*)"; return 1; }
_wipe_ok "$OUT" || exit 2
rm -rf "$OUT"
"$WS/deps/.venv-ml/bin/python" "$WS/src/ur_bringup/scripts/raw_to_lerobot.py" \
  --raw "$RAW" --repo-id "$REPO" --root "$OUT" \
  > "$LOG/${TAG:-convert_100}_run.log" 2>&1
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
  echo "== CONVERSION FAILED"; tail -5 "$LOG/${TAG:-convert_100}_run.log"
fi
