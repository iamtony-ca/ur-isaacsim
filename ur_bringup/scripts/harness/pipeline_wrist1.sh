#!/usr/bin/env bash
# collection (already running) -> convert -> train -> rollout, unattended.
#
# Each stage checks the ARTEFACT the previous one was supposed to produce, not
# just its exit code: a conversion that logs happily but writes no meta/info.json
# has failed, and training on the previous dataset would look like success.
set -uo pipefail
S="$(cd "$(dirname "$0")" && pwd)"
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
RAW="$WS/outputs/il_raw_wrist_only"
DS="$WS/outputs/lerobot_ds_wrist_only"
CK="$WS/outputs/act_wrist_only/checkpoints/last/pretrained_model"

# TARGET is 50, not 100: this run only has to prove the wrist-only path works
# end to end. The 2-camera comparison at 100 episodes already exists, and 50 is
# where v5 sat, so a wrist-only 50 is also the closer like-for-like.
TARGET=${TARGET:-50}
echo "== [1/4] waiting for $TARGET episodes"
for _ in $(seq 1 400); do
  n=$(ls -1d "$RAW"/episode_* 2>/dev/null | wc -l)
  [ "$n" -ge "$TARGET" ] && break
  grep -aq "ALL TASKS DONE" "$LOG/collect_wrist1.log" 2>/dev/null && break
  pgrep -f "[c]ollect_wrist1.sh" >/dev/null || break
  sleep 20
done
# Stop the collector once the target is reached -- it was told to gather 100.
# Kill by PID, never a broad pattern (CLAUDE.md), and give the recorder a moment
# to finish writing the episode it is on.
ps -eo pid,args --no-headers | grep "[c]ollect_wrist1.sh" | awk '{print $1}' \
  | while read -r pp; do kill "$pp" 2>/dev/null; done
sleep 3
# Let the demo finish the cycle it is on BEFORE killing the recorder: stopping
# mid-episode leaves a directory with frames but no meta.json/data.json, and the
# converter (correctly) drops it -- measured: asked for 50, got 49.
for _ in $(seq 1 30); do
  pgrep -f "[r]os2 launch ur_bringup pick_place" >/dev/null || break
  sleep 2
done
ps -eo pid,args --no-headers | grep -E "[i]l_recorder.py|[r]os2 launch ur_bringup pick_place" \
  | awk '{print $1}' | while read -r pp; do kill "$pp" 2>/dev/null; done
sleep 6
n=$(ls -1d "$RAW"/episode_* 2>/dev/null | wc -l)
echo "   episodes on disk: $n"
[ "$n" -ge "$TARGET" ] || { echo "FAIL: only $n episodes, expected $TARGET"; exit 1; }

echo "== [2/4] convert (no idle thinning)"
rm -rf "$DS"
"$S/convert_wrist1.sh" > "$LOG/w1_convert.log" 2>&1
[ -f "$DS/meta/info.json" ] || { echo "FAIL: conversion produced no dataset"; tail -5 "$LOG/w1_convert.log"; exit 1; }
grep -a "episodes .* frames" "$LOG/w1_convert.log" | tail -1

echo "== [3/4] stop Isaac, then train"
# Isaac parks ~14 MiB of Fast-DDS in the 64 MiB /dev/shm; that is what kills the
# dataloader workers (HISTORY.md 32.4).
ps -eo pid,args --no-headers \
  | grep -E "[u]r16e_isaac_ros2.py|[r]os2 launch ur_bringup|[r]os2_control_node|[m]ove_group|[l]ib/rviz2" \
  | awk '{print $1}' | while read -r p; do kill "$p" 2>/dev/null; done
sleep 12; rm -f /dev/shm/torch_* 2>/dev/null
echo "   shm free: $(df -h /dev/shm | awk 'NR==2{print $4}')"
"$S/train_wrist1.sh" > "$LOG/w1_train.log" 2>&1
[ -f "$CK/config.json" ] || { echo "FAIL: no checkpoint"; tail -5 "$LOG/w1_train.log"; exit 1; }
echo "   trained: $(ls $WS/outputs/act_wrist_only/checkpoints/)"

echo "== [4/4] rollout x10"
"$S/rollout_gui_then_n_w1.sh" > "$LOG/w1_rollout.log" 2>&1
grep -aE "^     |rollouts succeeded" "$LOG/w1_rollout.log" | tail -12
