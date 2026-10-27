#!/usr/bin/env bash
# Bring Isaac back up (training killed it), then score 10 rollouts.
# usage: rollout_gui_then_n.sh [N=10] [seconds=70] [checkpoint]   env SEED (131), RAND
#   default checkpoint = train_100.sh's act_red_left_100$GRASP_SFX/checkpoints/last;
#   final ACT: outputs/act_red_left_190_phys/checkpoints/200000/pretrained_model (HISTORY.md 58.7).
# GRASP_MODE (grasp_mode.sh): physical (default) = contact-friction scene and the
# *_phys checkpoint; attach = the weld scene and the weld-era checkpoint
# (HISTORY.md 53). GRASP_LOG=<csv> adds the per-step gripper/arm log.
set -uo pipefail
S="$(cd "$(dirname "$0")" && pwd)"
source "$S/grasp_mode.sh"   # GRASP_MODE=physical (default) | attach -- HISTORY.md 53
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
ME=$$
# RAND="--randomize-radius-x 0.10 ..." = extra Isaac randomisation args, e.g. the
# wide-range evaluation of HISTORY.md 58 (default: the original +/-0.06 square).
read -r -a RAND_EXTRA <<< "${RAND:-}"
echo "== rollout_gui_then_n: SEED=${SEED:-131} RAND=${RAND:-<none>}"
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
  --object-spacing 0.20 --randomize-object --randomize-radius 0.06 --seed "${SEED:-131}" "${RAND_EXTRA[@]}" \
  --camera-res 320x240 "${ISAAC_GRASP[@]}" ${GRASP_LOG:+--grasp-log "$GRASP_LOG"} --with-camera --with-static-cam --headless \
  > "$LOG/r100_isaac.log" 2>&1 &
for _ in $(seq 1 90); do grep -q "scene topics" "$LOG/r100_isaac.log" && break; sleep 5; done
grep -q "scene topics" "$LOG/r100_isaac.log" || { echo "FAIL: isaac"; exit 1; }
# "scene topics" is printed before the first physics step. On a cold shader cache Isaac
# then compiles for ~3.5 min before /clock ticks, and controllers spawned meanwhile time out
# ("Switch controller timed out", 2026-10-19). Wait for a real /clock message.
for _ in $(seq 1 60); do timeout 5 ros2 topic echo --once /clock > /dev/null 2>&1 && break; sleep 5; done
timeout 5 ros2 topic echo --once /clock > /dev/null 2>&1 || { echo "FAIL: isaac never published /clock"; exit 1; }
"$S/bringup.sh" > "$LOG/r100_bringup.log" 2>&1
tail -1 "$LOG/r100_bringup.log" | grep -q "== ready" || { echo "FAIL: bringup"; exit 1; }
timeout 90 ros2 launch ur_bringup policy_inference.launch.py use_sim:=true > "$LOG/r100_pi.log" 2>&1
"$S/rollout_n.sh" "${1:-10}" "${2:-70}" "${3:-$WS/outputs/act_red_left_100$GRASP_SFX/checkpoints/last/pretrained_model}"
