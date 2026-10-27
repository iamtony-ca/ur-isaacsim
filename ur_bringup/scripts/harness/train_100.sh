#!/usr/bin/env bash
# ACT on the single-task red->left set (default lerobot_ds_red_left_100$GRASP_SFX,
# 100 ep; written for the 50-ep v3 set), converted with NO idle thinning (raw idle
# ~10%). Final checkpoint: CK=outputs/act_red_left_190_phys STEPS=200000 on the
# 190-ep set -> checkpoints/200000 (HISTORY.md 58.5-58.7).
#
# What this checks: act_red_left_thin got 8/8 from post-processed data. If v3
# matches that WITHOUT post-processing, the fix lives in the state machine and
# transfers to real teleop data, where post-hoc thinning is not an option.
#
# Loss is not comparable across v2/thin/v3 -- different frame mixes, different
# floors. Judge by the rollout (HISTORY.md 32.5).
#
# ONE task on purpose: ACT has no language input (HISTORY.md 30), so a mixed set
# is contradictory supervision. This is the first run where "did it learn the
# task?" is a question the setup can actually answer -- the wide randomisation
# means a memorised trajectory will not score well.
#
# 60000 steps at the measured ~15 step/s = ~1 h 06 m (100-ep physical set, HISTORY.md
# 54.5; the 190-ep set ran 200k at ~12.7 step/s = ~4 h 23 m, 58.6). ALOHA trained ACT for 100k+ on
# 50 episodes/task; 60k is the point where the loss curve has usually flattened,
# and the checkpoint is what gets rolled out, not the loss.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(dirname "$0")"
source "$S/grasp_mode.sh"   # GRASP_MODE=physical (default) | attach -- HISTORY.md 53
# Overridable for other sets (HISTORY.md 58); the defaults are the original run's.
if [ -n "${DS:-}${REPO:-}${STEPS:-}${SAVE_FREQ:-}${TAG:-}" ] && [ -z "${CK:-}" ]; then
  echo "FAIL: overrides given but CK unset -- would wipe the original checkpoint"; exit 2; fi
# CK is WIPED before training -- point it at a new directory.
# To CONTINUE a run (more steps, or after a stop/reboot) never rerun this script:
# use lerobot-train --resume=true --config_path=<CK>/checkpoints/last/pretrained_model/
# train_config.json (HISTORY.md 54.6, 58.6; e.g. outputs/harness_logs/resume_train_190.sh).
DS="${DS:-$WS/outputs/lerobot_ds_red_left_100$GRASP_SFX}"
REPO="${REPO:-tony/ur16e_pick_place_red_left_100$GRASP_SFX}"
CK="${CK:-$WS/outputs/act_red_left_100$GRASP_SFX}"
STEPS="${STEPS:-60000}"
SAVE_FREQ="${SAVE_FREQ:-20000}"
TAG="${TAG:-train_100}"
cd "$WS"
for _ in $(seq 1 160); do
  [ -f "$DS/meta/info.json" ] && ! ps -eo args --no-headers | grep -q "[r]aw_to_lerobot" && break
  sleep 15
done
ps -eo args --no-headers | grep -q "[r]aw_to_lerobot" && { echo "FAIL: conversion still running"; exit 1; }
[ -f "$DS/meta/info.json" ] || { echo "FAIL: no dataset"; exit 1; }

deps/.venv-ml/bin/python - "$DS" <<'PY'
import json, sys
i = json.load(open(sys.argv[1] + "/meta/info.json"))
print(f"dataset: {i['total_episodes']} ep, {i['total_frames']} frames, fps {i['fps']}")
for k, v in i["features"].items():
    if k.startswith(("observation", "action")):
        print(f"   {k:32} {v['dtype']:6} {tuple(v['shape'])}")
PY

# num_workers=4, measured (bench_workers.sh, Isaac stopped, 400 steps each):
#     8 -> died on /dev/shm      6 -> died on /dev/shm
#     4 -> 12.52 step/s          2 -> 7.88 step/s        0 -> 1.36 step/s
# The ladder still falls back, because the /dev/shm failure is intermittent and
# depends on what else holds shared memory: Isaac's ROS stack alone parks ~14 MiB
# of Fast-DDS segments in the 64 MiB /dev/shm, which is what pushed an earlier
# num_workers=2 run over the edge. Stop Isaac before training.
# A wipe target must be one of our artefact dirs (HISTORY.md 58 review).
_wipe_ok() { case "$1" in */outputs/il_raw_*|*/outputs/lerobot_ds_*|*/outputs/act_*) return 0;; esac
             echo "FAIL: refusing to wipe '$1' (not outputs/{il_raw,lerobot_ds,act}_*)"; return 1; }
_wipe_ok "$CK" || exit 2
echo "== train_100: DS=$DS CK=$CK STEPS=$STEPS SAVE_FREQ=$SAVE_FREQ"
rm -rf "$CK"
for W in 4 2 0; do
  echo "== num_workers=$W"
  UR_WS_TORCH_SHM_FIX=1 deps/.venv-ml/bin/lerobot-train \
    --policy.type=act --policy.push_to_hub=false \
    --dataset.repo_id="$REPO" --dataset.root="$DS" \
    --steps="$STEPS" --save_freq="$SAVE_FREQ" --batch_size=32 --num_workers="$W" \
    --output_dir="$CK" > "$LOG/${TAG}_w$W.log" 2>&1
  if find "$CK" -path "*pretrained_model/config.json" 2>/dev/null | grep -q .; then
    echo "   DONE (workers=$W)"
    grep -oE "[0-9]+/$STEPS \[[0-9:]+<[0-9:]+, *[0-9.]+s/step\]" "$LOG/${TAG}_w$W.log" | tail -1
    grep -oE "loss:[0-9.]+" "$LOG/${TAG}_w$W.log" | tail -1
    break
  fi
  echo "   failed: $(grep -oE '(RuntimeError|OSError|ValueError).*' "$LOG/${TAG}_w$W.log" | tail -1 | cut -c1-70)"
  rm -rf "$CK"
done
