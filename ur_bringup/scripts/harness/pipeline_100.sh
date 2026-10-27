#!/usr/bin/env bash
# collection (already running) -> convert -> train -> rollout, unattended.
#
# Each stage checks the ARTEFACT the previous one was supposed to produce, not
# just its exit code: a conversion that logs happily but writes no meta/info.json
# has failed, and training on the previous dataset would look like success.
set -uo pipefail
S="$(cd "$(dirname "$0")" && pwd)"
source "$S/grasp_mode.sh"   # GRASP_MODE=physical (default) | attach -- HISTORY.md 53
# This pipeline rolls out what pick_place_demo.py JUST collected, and its READY is
# reset_pose `ready` in either grasp mode -- not grasp_mode.sh's ready_v1 for attach.
export START_POSE=ready
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
RAW="$WS/outputs/il_raw_red_left_100$GRASP_SFX"
DS="$WS/outputs/lerobot_ds_red_left_100$GRASP_SFX"
CKPT="$WS/outputs/act_red_left_100$GRASP_SFX/checkpoints/last/pretrained_model"

echo "== [1/4] waiting for the collection"
for _ in $(seq 1 400); do
  grep -aq "ALL TASKS DONE" "$LOG/collect_100.log" 2>/dev/null && break
  pgrep -f "[c]ollect_100.sh" >/dev/null || break
  sleep 30
done
n=$(ls -1d "$RAW"/episode_* 2>/dev/null | wc -l)
echo "   episodes on disk: $n"
[ "$n" -ge 90 ] || { echo "FAIL: only $n episodes, expected ~100"; exit 1; }

echo "== [2/4] convert (no idle thinning)"
rm -rf "$DS"
env -u OUT -u DS -u RAW -u REPO -u TAG "$S/convert_100.sh" > "$LOG/pipe_convert.log" 2>&1
[ -f "$DS/meta/info.json" ] || { echo "FAIL: conversion produced no dataset"; tail -5 "$LOG/pipe_convert.log"; exit 1; }
grep -a "episodes .* frames" "$LOG/pipe_convert.log" | tail -1

echo "== [3/4] stop Isaac, then train"
# Isaac parks ~14 MiB of Fast-DDS in the 64 MiB /dev/shm; that is what kills the
# dataloader workers (HISTORY.md 32.4).
ps -eo pid,args --no-headers \
  | grep -E "[u]r16e_isaac_ros2.py|[r]os2 launch ur_bringup|[r]os2_control_node|[m]ove_group|[l]ib/rviz2" \
  | awk '{print $1}' | while read -r p; do kill "$p" 2>/dev/null; done
sleep 12; rm -f /dev/shm/torch_* 2>/dev/null
echo "   shm free: $(df -h /dev/shm | awk 'NR==2{print $4}')"
env -u CK -u DS -u REPO -u STEPS -u SAVE_FREQ -u TAG "$S/train_100.sh" > "$LOG/pipe_train.log" 2>&1
[ -f "$CKPT/config.json" ] || { echo "FAIL: no checkpoint"; tail -5 "$LOG/pipe_train.log"; exit 1; }
echo "   trained: $(ls $WS/outputs/act_red_left_100$GRASP_SFX/checkpoints/)"

echo "== [4/4] rollout x10"
env -u SEED -u RAND "$S/rollout_gui_then_n.sh" > "$LOG/pipe_rollout.log" 2>&1
grep -aE "^     |rollouts succeeded" "$LOG/pipe_rollout.log" | tail -12
