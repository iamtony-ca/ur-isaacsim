#!/usr/bin/env bash
# v3: same 50 episodes of "red -> left", collected with the OPTIMISED state
# machine (HISTORY.md 32): cycle time 41.3 s -> 13.3 s, and the idle mass that
# made v2 unusable for ACT should now be absent at the SOURCE rather than removed
# by post-processing.
#
# The point of this run is to check exactly that: if the raw idle fraction is low,
# --max-idle-run stops being load-bearing and the same pipeline works on real
# hardware, where there is no post-hoc thinning of a teleop session.
#
# Different seed (21, was 11) so this is not a re-run of the same object poses.
#
# Single-task collection for the ACT rung: 50 episodes of "red -> left".
#
# WHY ONE TASK: ACT does not read the instruction (verified in lerobot 0.6.1 --
# no tokenizer in policies/act/, forward() takes images + state only). Feeding it
# three tasks over an identical scene is contradictory supervision. The 3-task
# set stays as-is for the VLA rung. HISTORY.md 30.
#
# WHY A WIDER RANDOM RANGE: with the object nearly fixed, ACT can memorise one
# trajectory and ignore the camera -- and then a good training loss tells you
# nothing. 0.06 is the largest radius that keeps the two 35 mm cubes apart: their
# homes are 0.20 m apart in y, so worst case leaves 0.08 m of clearance.
# The 3-task run used 0.025, which was too generous to the policy.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(dirname "$0")"
ME=$$
N=${1:-50}
OUT="$WS/outputs/il_raw_red_left_100"
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
  --object-spacing 0.20 --randomize-object --randomize-radius 0.06 --seed 91 \
  --camera-res 320x240 --grasp-attach --with-camera --with-static-cam --headless \
  > "$LOG/c100_isaac.log" 2>&1 &
for _ in $(seq 1 90); do grep -q "scene topics" "$LOG/c100_isaac.log" && break; sleep 5; done
grep -q "scene topics" "$LOG/c100_isaac.log" || { echo "FAIL: isaac"; exit 1; }
"$S/bringup.sh" > "$LOG/c100_bringup.log" 2>&1
tail -1 "$LOG/c100_bringup.log" | grep -q "== ready" || { echo "FAIL: bringup"; tail -3 "$LOG/c100_bringup.log"; exit 1; }
echo "   stack ready"

rm -rf "$LOG/grasp_c100"; mkdir -p "$LOG/grasp_c100"
nohup python3 "$S/grasp_watch.py" "$LOG/grasp_c100" 14400 0.45 > "$LOG/grasp_c100.log" 2>&1 &
W=$!
TASKS="red|left|put the red block on the left marker" LOGS="$LOG/c100_logs" \
  "$WS/src/ur_bringup/scripts/collect_il_episodes.sh" "$N" "$OUT"
kill "$W" 2>/dev/null
echo "== grasp frames: $(ls -1 "$LOG/grasp_c100" | wc -l)"
