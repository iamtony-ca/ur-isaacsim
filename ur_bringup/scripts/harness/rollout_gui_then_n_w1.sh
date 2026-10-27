#!/usr/bin/env bash
# Bring Isaac back up (training killed it), then score 10 rollouts.
set -uo pipefail
S="$(cd "$(dirname "$0")" && pwd)"
source "$S/grasp_mode.sh"   # GRASP_MODE=physical (default) | attach -- HISTORY.md 53
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
ME=$$
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"   # shared machine: another project may occupy domain 0 (HISTORY.md 47)
cd "$WS"
ps -eo pid,args --no-headers | grep "[u]r16e_isaac_ros2.py" \
  | awk -v me="$ME" '$1!=me{print $1}' | while read -r p; do kill "$p" 2>/dev/null; done
sleep 10
nohup /isaac-sim/python.sh "$WS/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py" \
  --asset-path "$WS/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd" \
  --scene pick_place --table --table-height 0.20 --table-pose 0.72,0.0 --table-size 0.70,0.90 \
  --object-pose 0.60,0.0,0.225 --object-size 0.035,0.035,0.035 \
  --place-pose 0.60,0.315,0.0 --object-names red,blue --place-names left,right \
  --object-spacing 0.20 --randomize-object --randomize-radius 0.06 --seed "${SEED:-151}" \
  --camera-res 320x240 "${ISAAC_GRASP[@]}" ${GRASP_LOG:+--grasp-log "$GRASP_LOG"} --with-camera --with-static-cam --headless \
  > "$LOG/rw1_isaac.log" 2>&1 &
for _ in $(seq 1 90); do grep -q "scene topics" "$LOG/rw1_isaac.log" && break; sleep 5; done
grep -q "scene topics" "$LOG/rw1_isaac.log" || { echo "FAIL: isaac"; exit 1; }
# "scene topics" is printed before the first physics step. On a cold shader cache Isaac
# then compiles for ~3.5 min before /clock ticks, and controllers spawned meanwhile time out
# ("Switch controller timed out", 2026-10-19). Wait for a real /clock message.
for _ in $(seq 1 60); do timeout 5 ros2 topic echo --once /clock > /dev/null 2>&1 && break; sleep 5; done
timeout 5 ros2 topic echo --once /clock > /dev/null 2>&1 || { echo "FAIL: isaac never published /clock"; exit 1; }
"$S/bringup.sh" > "$LOG/rw1_bringup.log" 2>&1
tail -1 "$LOG/rw1_bringup.log" | grep -q "== ready" || { echo "FAIL: bringup"; exit 1; }
timeout 90 ros2 launch ur_bringup policy_inference.launch.py use_sim:=true > "$LOG/rw1_pi.log" 2>&1
"$S/rollout_wrist1.sh" "${1:-10}" "${2:-70}" "${3:-$WS/outputs/act_wrist_only$GRASP_SFX/checkpoints/last/pretrained_model}"
