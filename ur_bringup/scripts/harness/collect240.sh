#!/usr/bin/env bash
# Fresh Isaac at 320x240 for BOTH cameras, then collect 7 episodes per task.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(dirname "$0")"
ME=$$
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID=0
ps -eo pid,args --no-headers | grep "[u]r16e_isaac_ros2.py" \
  | awk -v me="$ME" '$1!=me{print $1}' | while read -r p; do kill "$p" 2>/dev/null; done
sleep 10
nohup /isaac-sim/python.sh "$WS/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py" \
  --asset-path "$WS/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd" \
  --scene pick_place --table --table-height 0.20 --table-pose 0.72,0.0 --table-size 0.70,0.90 \
  --object-pose 0.60,0.0,0.225 --object-size 0.035,0.035,0.035 \
  --place-pose 0.60,0.315,0.0 --object-names red,blue --place-names left,right \
  --object-spacing 0.20 --randomize-object --randomize-radius 0.025 \
  --camera-res 320x240 --grasp-attach --with-camera --with-static-cam --headless \
  > "$LOG/c240_isaac.log" 2>&1 &
for _ in $(seq 1 90); do grep -q "scene topics" "$LOG/c240_isaac.log" && break; sleep 5; done
grep -q "scene topics" "$LOG/c240_isaac.log" || { echo "FAIL: isaac"; exit 1; }
grep -E "eye-in-hand camera|static cam  " "$LOG/c240_isaac.log"
"$S/bringup.sh" > "$LOG/c240_bringup.log" 2>&1
tail -1 "$LOG/c240_bringup.log" | grep -q "== ready" || { echo "FAIL: bringup"; exit 1; }
echo "   stack ready"
LOGS="$LOG/c240_logs" "$WS/src/ur_bringup/scripts/collect_il_episodes.sh" 7 \
  "$WS/outputs/il_raw_3task_240"
