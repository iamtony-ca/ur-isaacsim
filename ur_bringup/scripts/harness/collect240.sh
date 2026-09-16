#!/usr/bin/env bash
# Fresh Isaac at 320x240 for BOTH cameras, then collect N episodes per task (3-task scene, VLA rung).
#
# Defaults reproduce lerobot_ds_240_v2 (7 ep/task, radius 0.025, seed 0). The v3 recollection
# (HISTORY.md 46) overrides them through the environment instead of copying this file:
#   N=30 RAW=$WS/outputs/il_raw_3task_v3 RADIUS=0.06 SEED=161 TAG=c3v3 collect240.sh
# RADIUS 0.06 = the ACT baseline's scene (collect_100.sh: largest radius that keeps the two cubes
# apart), so the GR00T number becomes comparable to ACT's 9/10. The rollout driver (groot_v8.sh)
# takes the SAME two variables -- the scene must match what the policy saw.
#
# usage: collect240.sh [episodes_per_task]
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(dirname "$0")"
ME=$$
N=${1:-7}
RAW="${RAW:-$WS/outputs/il_raw_3task_240}"
RADIUS="${RADIUS:-0.025}"
SEED="${SEED:-0}"
TAG="${TAG:-c240}"          # log-file prefix in $LOG
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
  --object-spacing 0.20 --randomize-object --randomize-radius "$RADIUS" --seed "$SEED" \
  --camera-res 320x240 --grasp-attach --with-camera --with-static-cam --headless \
  > "$LOG/${TAG}_isaac.log" 2>&1 &
for _ in $(seq 1 90); do grep -q "scene topics" "$LOG/${TAG}_isaac.log" && break; sleep 5; done
grep -q "scene topics" "$LOG/${TAG}_isaac.log" || { echo "FAIL: isaac"; exit 1; }
grep -E "eye-in-hand camera|static cam  " "$LOG/${TAG}_isaac.log"
"$S/bringup.sh" > "$LOG/${TAG}_bringup.log" 2>&1
tail -1 "$LOG/${TAG}_bringup.log" | grep -q "== ready" || { echo "FAIL: bringup"; exit 1; }
echo "   stack ready"
# Grasp evidence per cycle (collect_100.sh does the same): the state machine's SUCCESS
# comes from a joint threshold, not contact (HISTORY.md 28). 0.45 < grip_closed 0.52.
rm -rf "$LOG/grasp_$TAG"; mkdir -p "$LOG/grasp_$TAG"
nohup python3 "$S/grasp_watch.py" "$LOG/grasp_$TAG" 14400 0.45 > "$LOG/grasp_$TAG.log" 2>&1 &
W=$!
echo "== $N episodes/task, radius $RADIUS, seed $SEED -> $RAW"
LOGS="$LOG/${TAG}_logs" "$WS/src/ur_bringup/scripts/collect_il_episodes.sh" "$N" "$RAW"
kill "$W" 2>/dev/null
echo "== grasp frames: $(ls -1 "$LOG/grasp_$TAG" | wc -l)"
