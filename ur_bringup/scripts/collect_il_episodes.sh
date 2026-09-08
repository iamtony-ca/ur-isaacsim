#!/usr/bin/env bash
# Collect language-conditioned pick&place episodes into ONE raw directory.
#
#   1. red  -> left      2. blue -> left      3. red  -> right
#
# 1 vs 2 differ in WHICH object, 1 vs 3 in WHERE. Both objects and both markers
# are present in every episode, so the instruction is the only thing that
# disambiguates -- which is the point (plan_il_vla.md 2.8). A single task string
# would let the policy ignore language and still be right.
#
# One recorder for all three: it keeps a single episode counter and one output
# directory, so raw_to_lerobot sees one dataset with three task strings.
#
# Requires the Isaac pick_place scene + control stack + MoveIt to be up already
# (README.md 5). Run the demo through pick_place_demo.launch.py, NOT `ros2 run`:
# the sim-only gripper compensations live in config/common/pick_place_sim.yaml
# and only the launch layers them. Bypassing it silently collects episodes with
# the pre-2026-09-08 behaviour, where the gripper never actually grips
# (HISTORY.md 26).
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

TASKS=(
  "red|left|put the red block on the left marker"
  "blue|left|put the blue block on the left marker"
  "red|right|put the red block on the right marker"
)

echo "== collecting $N episodes/task into $OUT"
rm -rf "$OUT"
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
    > "$LOGS/recorder_${obj}_${plc}.log" 2>&1 &
  for _ in $(seq 1 30); do
    ros2 service list 2>/dev/null | grep -q "/il/start_episode" && break; sleep 2
  done
  ros2 service list 2>/dev/null | grep -q "/il/start_episode" \
    || { echo "FAIL: recorder for '$text' never came up"; exit 1; }

  # A little headroom: failed cycles are discarded, so a 1:1 request can come up
  # short. Verified at 6/6 on 2026-09-08, hence the small margin -- it used to
  # need +67% when most cycles failed.
  CYCLES=$(( N + 2 ))
  echo "   running up to $CYCLES cycles for $N episodes"
  ros2 launch ur_bringup pick_place_demo.launch.py \
    use_sim:=true cycles:="$CYCLES" record:=true \
    object_topic:="/scene/objects/${obj}/pose" \
    place_topic:="/scene/places/${plc}/pose" \
    > "$LOGS/collect_${obj}_${plc}.log" 2>&1
  grep -E "cycles succeeded" "$LOGS/collect_${obj}_${plc}.log" \
    | sed 's/.*pick_place_demo\]: //'

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
