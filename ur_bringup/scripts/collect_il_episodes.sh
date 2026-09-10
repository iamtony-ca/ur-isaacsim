#!/usr/bin/env bash
# Collect pick&place episodes into ONE raw directory.
#
# DEFAULT (3 tasks, for the VLA rung):
#   1. red -> left    2. blue -> left    3. red -> right
# 1 vs 2 differ in WHICH object, 1 vs 3 in WHERE. Both objects and both markers
# are present in every episode, so the instruction is the only thing that
# disambiguates (plan_il_vla.md 2.8).
#
# ONE task (for the ACT rung) -- set TASKS to a single "obj|place|text" spec:
#   TASKS="red|left|put the red block on the left marker" \
#     collect_il_episodes.sh 50 outputs/il_raw_red_left
#
# ★ WHICH ONE YOU WANT DEPENDS ON THE POLICY, and it is not a preference:
#   ACT does NOT read the task string. Verified in lerobot 0.6.1 -- its forward()
#   consumes OBS_IMAGES + OBS_STATE (+OBS_ENV_STATE) and there is no tokenizer
#   anywhere in policies/act/. pi0/pi05/smolvla/groot all have one.
#   So a 3-task set fed to ACT is contradictory supervision: identical pixels,
#   three different targets, and it converges to the average. Train ACT on ONE
#   task per checkpoint; keep the 3-task set for the VLA rung, where the
#   instruction is actually an input. HISTORY.md 30.
#
# One recorder for the whole run: it keeps a single episode counter and one
# output directory, so raw_to_lerobot sees one dataset.
#
# Requires the Isaac pick_place scene + control stack + MoveIt to be up already
# (README.md 5). Run the demo through pick_place_demo.launch.py, NOT `ros2 run`:
# the launch layers config/common/pick_place*.yaml, which is where every task
# parameter lives. `ros2 run` gets the code defaults instead, silently collecting
# episodes with untuned grasp values.
#
# usage: collect_il_episodes.sh [episodes_per_task] [out_dir]
set +u
source /opt/ros/jazzy/setup.bash
source /isaac-sim/volume/ur_ws/install/setup.bash
set -u
export ROS_DOMAIN_ID=0
WS=/isaac-sim/volume/ur_ws
LOGS=${LOGS:-/tmp/il_collect}
mkdir -p "$LOGS"
N=${1:-20}
OUT=${2:-$WS/outputs/il_raw_3task}
ME=$$

# Camera set. "none" = that camera is not recorded, and then it is absent
# from meta.json's video_keys, so the converter builds one fewer image feature and
# ACT sizes its backbone stack accordingly -- the rest of the pipeline needs no
# change. Wrist-only:
#   CAM_EXTERIOR=none collect_il_episodes.sh 50 outputs/il_raw_wrist_only
# ("none", not "": rcl refuses to parse an empty -p override at all.)
# The camera set is part of the dataset's identity: a policy trained on two
# cameras cannot be rolled out with one. Keep separate out_dirs per set, and pass
# the matching --robot.cameras_ros at inference.
CAM_EXTERIOR=${CAM_EXTERIOR-/static_cam/color/image_raw}
CAM_WRIST=${CAM_WRIST-/camera/color/image_raw}

# $TASKS overrides the default set: newline- or semicolon-separated
# "object|place|instruction" specs. One spec = the ACT rung, three = the VLA rung.
if [ -n "${TASKS:-}" ]; then
  IFS=$'\n;' read -r -d '' -a TASKS < <(printf '%s\0' "$TASKS")
else
  TASKS=(
    "red|left|put the red block on the left marker"
    "blue|left|put the blue block on the left marker"
    "red|right|put the red block on the right marker"
  )
fi

echo "== collecting $N episodes/task into $OUT"
# APPEND=1 keeps what is already there and tops up to `have + N`. Growing a set
# is otherwise destructive: the wipe below is what makes a re-run reproducible,
# but it also means "collect 50 more" silently becomes "throw 50 away and collect
# 50". il_recorder continues the episode numbering from disk, so appending is
# safe as long as the PARAMETERS are unchanged -- if they are not, start a new
# out_dir instead, because a dataset mixing two grasp geometries is worse than
# either half (HISTORY.md 33/35).
if [ "${APPEND:-0}" = 1 ]; then
  echo "   APPEND=1: keeping $(ls -1d "$OUT"/episode_* 2>/dev/null | wc -l) existing episode(s)"
else
  rm -rf "$OUT"
fi
for spec in "${TASKS[@]}"; do
  IFS='|' read -r obj plc text <<< "$spec"
  echo "=============================================================="
  echo "== task: $text"

  # Wait for the PREVIOUS recorder's services to disappear first. `ros2 service
  # list` keeps showing them for a while after the process dies, so the "is the
  # recorder up?" check below passes on the OLD one and the state machine then
  # calls a service nobody is serving -> cycles fail with `recorder_unavailable`
  # (seen on the first cycle of a task, right after a task switch).
  for _ in $(seq 1 20); do
    ros2 service list 2>/dev/null | grep -q "/il/start_episode" || break; sleep 2
  done

  # The recorder carries the task string, so it is restarted per task. out_dir is
  # shared and il_recorder continues the episode numbering from what is on disk.
  nohup ros2 run ur_bringup il_recorder.py --ros-args \
    -p use_sim_time:=true -p out_dir:="$OUT" -p task:="$text" -p auto_reset:=false \
    -p cameras.exterior:="$CAM_EXTERIOR" -p cameras.wrist:="$CAM_WRIST" \
    > "$LOGS/recorder_${obj}_${plc}.log" 2>&1 &
  for _ in $(seq 1 30); do
    ros2 service list 2>/dev/null | grep -q "/il/start_episode" && break; sleep 2
  done
  ros2 service list 2>/dev/null | grep -q "/il/start_episode" \
    || { echo "FAIL: recorder for '$text' never came up"; exit 1; }

  # Ask for exactly what is still missing, then top up if cycles failed. Asking
  # for N+margin in one go over-collects now that the success rate is high: at
  # 6/6 a request for 20 produced 22, and "20 per task" in the dataset card would
  # have been wrong. Failed cycles record nothing, so counting episodes on disk
  # is the honest measure.
  have=$(ls -1d "$OUT"/episode_* 2>/dev/null | wc -l)
  target=$(( have + N ))
  attempt=0
  while [ "$have" -lt "$target" ] && [ "$attempt" -lt 4 ]; do
    attempt=$(( attempt + 1 ))
    want=$(( target - have ))
    echo "   attempt $attempt: $want more episode(s) (have $have, want $target)"
    ros2 launch ur_bringup pick_place_demo.launch.py \
      use_sim:=true cycles:="$want" record:=true \
      object_topic:="/scene/objects/${obj}/pose" \
      place_topic:="/scene/places/${plc}/pose" \
      > "$LOGS/collect_${obj}_${plc}_$attempt.log" 2>&1
    grep -E "cycles succeeded" "$LOGS/collect_${obj}_${plc}_$attempt.log" \
      | sed 's/.*pick_place_demo\]: /   /'
    before=$have
    have=$(ls -1d "$OUT"/episode_* 2>/dev/null | wc -l)
    if [ "$have" -eq "$before" ]; then
      echo "   no progress in this attempt; stopping this task rather than looping"
      break
    fi
  done
  [ "$have" -lt "$target" ] && echo "   SHORT: $((target - have)) episode(s) missing for '$text'"

  # Kill by PID excluding this shell: matching these patterns has repeatedly
  # killed the shell issuing the kill.
  ps -eo pid,args --no-headers | grep "[i]l_recorder.py" \
    | awk -v me="$ME" '$1 != me {print $1}' \
    | while read -r p; do kill "$p" 2>/dev/null; done
  sleep 4
  echo "   episodes on disk: $(ls -1 "$OUT" 2>/dev/null | wc -l)"
done

echo "== ALL TASKS DONE: $(ls -1 "$OUT" | wc -l) episodes, $(du -sh "$OUT" | cut -f1)"
echo "   convert with:  deps/.venv-ml/bin/python src/ur_bringup/scripts/raw_to_lerobot.py $OUT <dest>"
