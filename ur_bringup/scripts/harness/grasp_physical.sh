#!/usr/bin/env bash
# Physical-contact grasp trial: Isaac (no weld) + control/MoveIt + N pick&place
# cycles of the red block, with a per-step physics log. HISTORY.md 53.
#
#   ROS_DOMAIN_ID=42 ./grasp_physical.sh <tag> <cycles> [isaac args...]
#
#   <tag>          names the logs: $LOG/gp_<tag>_{isaac.log,demo.log,steps.csv}
#   isaac args     appended to the Isaac command, e.g. --gripper-profile asset
#   DEMO_PARAMS    extra pick_place_demo parameters as YAML lines, e.g.
#                    DEMO_PARAMS=$'grasp_bias_y: 0.010'
#                  added to the mode default grip_closed (0.8 physical / 0.52
#                  attach) unless they set grip_closed themselves
#   MODE=attach    regression run of the weld path instead
#   SEED           object randomisation seed (default 91)
#
# Isaac is always restarted: one blow-out contaminates every later cycle, and
# the per-step log belongs to the Isaac process that writes it.
#
# Cameras are off: nothing here reads images, and they cost GPU and CPU on a
# shared host. Results: grasp_log_stats.py summarises the CSV + demo log.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"
S="$(dirname "$0")"
TAG=${1:?tag}; N=${2:?cycles}; shift 2
ISAAC_EXTRA=("$@")
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
P="$LOG/gp_$TAG"
MODE="${MODE:-physical}"
if [ "$MODE" = physical ]; then DEF_PARAMS="grip_closed: 0.8"; else DEF_PARAMS="grip_closed: 0.52"; fi

ps -eo pid,args --no-headers | grep "[u]r16e_isaac_ros2.py" | awk '{print $1}' \
  | while read -r p; do kill "$p" 2>/dev/null; done
sleep 8
nohup nice -n 5 /isaac-sim/python.sh "$WS/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py" \
  --asset-path "$WS/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd" \
  --scene pick_place --table --table-height 0.20 --table-pose 0.72,0.0 --table-size 0.70,0.90 \
  --object-pose 0.60,0.0,0.225 --object-size 0.035,0.035,0.035 \
  --place-pose 0.60,0.315,0.0 --object-names red,blue --place-names left,right \
  --object-spacing 0.20 --randomize-object --randomize-radius 0.06 --seed "${SEED:-91}" \
  --grasp-mode "$MODE" --grasp-log "$P.steps.csv" --headless "${ISAAC_EXTRA[@]}" \
  > "$P.isaac.log" 2>&1 &
for _ in $(seq 1 90); do grep -q "scene topics" "$P.isaac.log" && break; sleep 5; done
grep -q "scene topics" "$P.isaac.log" || { echo "FAIL: isaac"; tail -20 "$P.isaac.log"; exit 1; }
for _ in $(seq 1 60); do timeout 5 ros2 topic echo --once /clock > /dev/null 2>&1 && break; sleep 5; done
grep -E "grasp mode|gripper physics|finger_joint \(PhysX\)" "$P.isaac.log"
"$S/bringup.sh" > "$P.bringup.log" 2>&1
tail -1 "$P.bringup.log" | grep -q "== ready" || { echo "FAIL: bringup"; tail -3 "$P.bringup.log"; exit 1; }
echo "== stack ready; $N cycles"
Y="$P.demo.yaml"
{ echo "/**:"; echo "  ros__parameters:"; echo "    grasp_mode: $MODE"
  grep -qE '^[[:space:]]*grip_closed:' <<< "${DEMO_PARAMS:-}" || echo "    $DEF_PARAMS"
  [ -n "${DEMO_PARAMS:-}" ] && printf '%s\n' "$DEMO_PARAMS" | sed 's/^/    /'; } > "$Y"
ros2 launch ur_bringup pick_place_demo.launch.py use_sim:=true cycles:="$N" \
  object_topic:=/scene/objects/red/pose place_topic:=/scene/places/left/pose \
  extra_params:="$Y" > "$P.demo.log" 2>&1
grep -E "cycles succeeded" "$P.demo.log" | sed 's/.*pick_place_demo\]: /   /'
"$WS/deps/.venv-ml/bin/python" "$S/grasp_log_stats.py" "$P"
