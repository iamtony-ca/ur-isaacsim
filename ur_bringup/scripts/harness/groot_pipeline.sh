#!/usr/bin/env bash
# GR00T N1.7 v3: recollect (30 ep/task, h264) -> convert -> train (abs, OMP_NUM_THREADS=8)
# -> rollout, unattended. HISTORY.md 46.
#
# Why a v3 at all: 240_v2 had 7 ep/task, and 44.3's 3/8 could not separate "reads the
# instruction" from "memorised three trajectories". This run gives GR00T the same footing
# ACT had (red_left_100: 100 ep, radius 0.06) -- 30 ep per colour/marker pair, 90 in all.
#
# Every stage checks the ARTEFACT the previous one had to produce, not its exit code
# (pipeline_100.sh). Nothing here duplicates the stage scripts: each is the existing
# harness script with its environment overrides, so a fix in collect240.sh or
# groot_train_abs.sh is a fix here too.
#
# Two rollouts, on purpose:
#   A  radius 0.06, 10 trials/task (30)  -- ACT's scene: the number to put next to 9/10.
#   B  radius 0.025, 3 trials/task (9)   -- the 44.3 scene: the number to put next to 3/8.
# Training at 0.06 covers both. Seeds differ from the collection's (161) so the poses
# are new, and from each other.
#
# usage: groot_pipeline.sh [episodes_per_task]      (nohup ... > outputs/harness_logs/pipe_v3.log &)
set -uo pipefail
S="$(cd "$(dirname "$0")" && pwd)"
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
N=${1:-30}
V=v3
RAW="$WS/outputs/il_raw_3task_$V"
DS="$WS/outputs/lerobot_ds_3task_$V"
REPO="tony/ur16e_pick_place_3task_$V"
OUT="$WS/outputs/groot_3task_${V}_abs"
CK="$OUT/checkpoints/last/pretrained_model"
STEPS=10000
BUDGET_H=6                  # user gate (plan_groot_n17.md 4.1): report instead of running past this
t0=$(date +%s)
stamp() { echo "== [$(date '+%H:%M:%S') +$(( ($(date +%s) - t0) / 60 ))m] $*"; }
cd "$WS"

stamp "[0/6] preflight"
for pat in "[u]r16e_isaac_ros2.py" "[l]erobot-train" "[p]olicy_server" "[p]ick_place_demo" "[i]l_recorder.py"; do
  if ps -eo args --no-headers | grep -q -- "$pat"; then echo "FAIL: '$pat' already running -- not starting on top of it"; exit 1; fi
done
if [ -e "$RAW" ] || [ -e "$DS" ] || [ -e "$OUT" ]; then
  echo "FAIL: $V outputs already exist ($RAW / $DS / $OUT). Pick a new V or move them; this script never deletes a dataset."; exit 1
fi
source "$WS/src/setup/ml_env.sh"
BASE=$(deps/.venv-ml/bin/python -c "from huggingface_hub import snapshot_download; print(snapshot_download('nvidia/GR00T-N1.7-3B'))" 2>/dev/null)
[ -d "${BASE:-/nonexistent}" ] || { echo "FAIL: GR00T snapshot not resolvable offline (setup/check_hf_cache.sh)"; exit 1; }
echo "   base $BASE"
echo "   GPU $(nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader)   disk $(df -h "$WS/outputs" | awk 'NR==2{print $4" free"}')"

stamp "[1/6] collect $N ep/task (radius 0.06, seed 161, h264 later)"
N="$N" RAW="$RAW" RADIUS=0.06 SEED=161 TAG=c3v3 bash "$S/collect240.sh" "$N" > "$LOG/collect_$V.log" 2>&1
n=$(ls -1d "$RAW"/episode_* 2>/dev/null | wc -l)
echo "   episodes on disk: $n   (grasp frames: $(ls -1 "$LOG/grasp_c3v3" 2>/dev/null | wc -l))"
grep -aE "SHORT|no progress|FAIL" "$LOG/collect_$V.log" | head -5
[ "$n" -ge $(( N * 3 * 9 / 10 )) ] || { echo "FAIL: only $n episodes, expected ~$(( N * 3 ))"; tail -5 "$LOG/collect_$V.log"; exit 1; }

stamp "[2/6] convert -> $DS (h264 crf23)"
RAW="$RAW" OUT="$DS" REPO="$REPO" COLLECT_LOG="$LOG/collect_$V.log" TAG="convert_$V" bash "$S/convert240.sh" > "$LOG/pipe_convert_$V.log" 2>&1
[ -f "$DS/meta/info.json" ] || { echo "FAIL: conversion produced no dataset"; tail -5 "$LOG/pipe_convert_$V.log"; exit 1; }
grep -aE "episodes .* frames|codec|tasks:" "$LOG/pipe_convert_$V.log"
deps/.venv-ml/bin/python - "$DS" "$n" <<'PY'
import json, sys
i = json.load(open(sys.argv[1] + "/meta/info.json"))
codecs = {v["info"]["video.codec"] for k, v in i["features"].items() if k.startswith("observation.images")}
ok = i["total_episodes"] == int(sys.argv[2]) and i.get("total_tasks") == 3 and codecs == {"h264"}
print(f"   check episodes={i['total_episodes']} tasks={i.get('total_tasks')} codecs={codecs} -> {'ok' if ok else 'MISMATCH'}")
sys.exit(0 if ok else 1)
PY
[ $? -eq 0 ] || { echo "FAIL: dataset does not match the collection"; exit 1; }

stamp "[3/6] stop the sim stack, then train ($STEPS steps, abs, OMP_NUM_THREADS=8)"
bash "$S/shutdown_all.sh" > "$LOG/pipe_shutdown1_$V.log" 2>&1
rm -f /dev/shm/torch_* 2>/dev/null
echo "   GPU $(nvidia-smi --query-gpu=memory.used --format=csv,noheader)  shm $(df -h /dev/shm | awk 'NR==2{print $3"/"$2}')"
DS="$DS" REPO="$REPO" OUT="$OUT" TAG="groot_$V" nohup bash "$S/groot_train_abs.sh" > "$LOG/train_$V.log" 2>&1 &
TPID=$!
# 6 h gate: read the tqdm ETA once the run has settled, and stop rather than overrun.
sleep 900
eta=$(tr '\r' '\n' < "$LOG/train_$V.log" | grep -oE "$STEPS \[[0-9:]+<[0-9:]+" | tail -1 | sed 's/.*<//')
h=$(echo "${eta:-0:00}" | awk -F: '{print (NF==3)? $1 : 0}')
echo "   after 15 min: eta ${eta:-?}"
if [ "${h:-0}" -ge "$BUDGET_H" ]; then
  echo "STOP: predicted ${eta} exceeds the ${BUDGET_H} h budget -- killing training, report to the user (plan 4.1)"
  kill "$TPID" 2>/dev/null; pkill -P "$TPID" 2>/dev/null; exit 2
fi
bash "$S/train_watchdog.sh" "$LOG/train_$V.log" "$STEPS" 600 900 > "$LOG/train_${V}_watch.log" 2>&1
tail -3 "$LOG/train_${V}_watch.log"
wait "$TPID"; trc=$?
grep -aE "^== (rc|peak)" "$LOG/train_$V.log"
[ -f "$CK/config.json" ] || { echo "FAIL: no final checkpoint (rc=$trc)"; tr '\r' '\n' < "$LOG/train_$V.log" | grep -aE "Error|Traceback" | tail -5; exit 1; }
echo "   checkpoints: $(ls "$OUT/checkpoints")"

export START_POSE=ready   # this pipeline collects with the current READY (HISTORY.md 47)
stamp "[4/6] rollout A: radius 0.06, seed 171, 10/task"
RADIUS=0.06 SEED=171 TAG=gr8v3a ROLL_TAG=grv3a bash "$S/groot_v8.sh" 10 90 "$CK" > "$LOG/rollout_${V}_a.log" 2>&1
grep -aE "^     |^   |^== (per-task|total)" "$LOG/rollout_${V}_a.log" | grep -vE "stack ready|forward_position|attempt"

stamp "[5/6] rollout B: radius 0.025, seed 0, 3/task (the 44.3 scene)"
RADIUS=0.025 SEED=0 TAG=gr8v3b ROLL_TAG=grv3b bash "$S/groot_v8.sh" 3 90 "$CK" > "$LOG/rollout_${V}_b.log" 2>&1
grep -aE "^     |^   |^== (per-task|total)" "$LOG/rollout_${V}_b.log" | grep -vE "stack ready|forward_position|attempt"

stamp "[6/6] shutdown"
bash "$S/shutdown_rollout.sh" > "$LOG/pipe_shutdown2_$V.log" 2>&1
tail -4 "$LOG/pipe_shutdown2_$V.log"
stamp "PIPELINE DONE"
