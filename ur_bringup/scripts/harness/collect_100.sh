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
# Seed: the default is 91 -- the 100-episode sets (il_raw_red_left_100 and _phys;
# HISTORY.md 54.5, 58.2); 21 was the 50-ep v3 run, 11 before it. Run it as
# `collect_100.sh 100` (N still defaults to 50). The final 190-ep set added an outer
# band through the overrides below (HISTORY.md 58.3): OUT=.../il_raw_red_left_band_phys
# SEED=301 TAG=cband RAND="--randomize-radius-x 0.10 --randomize-radius-y 0.06
# --randomize-inner-x 0.06 --randomize-yaw-edge-frac 0.3 --randomize-yaw-edge-deg 10".
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
source "$S/grasp_mode.sh"   # GRASP_MODE=physical (default) | attach -- HISTORY.md 53
ME=$$
N=${1:-50}
# Overrides without OUT would wipe the ORIGINAL set (collect_il_episodes.sh rm -rf's OUT).
if [ -n "${SEED:-}${RAND:-}${TAG:-}" ] && [ -z "${OUT:-}" ]; then
  echo "FAIL: SEED/RAND/TAG given but OUT unset -- would wipe the original raw set"; exit 2; fi
OUT="${OUT:-$WS/outputs/il_raw_red_left_100$GRASP_SFX}"
# Top-up runs (HISTORY.md 58) override these; the defaults are the original set's.
#   OUT=...  a NEW raw dir -- collect_il_episodes.sh wipes OUT unless APPEND=1
#   SEED=... never 91 again (same layouts)   TAG=...  log prefix (default c100)
#   RAND="--randomize-radius-x 0.10 --randomize-inner-x 0.06 ..."  extra Isaac args
SEED="${SEED:-91}"
TAG="${TAG:-c100}"
read -r -a RAND_EXTRA <<< "${RAND:-}"
# A wipe target must be one of our artefact dirs (HISTORY.md 58 review).
_wipe_ok() { case "$1" in */outputs/il_raw_*|*/outputs/lerobot_ds_*|*/outputs/act_*) return 0;; esac
             echo "FAIL: refusing to wipe '$1' (not outputs/{il_raw,lerobot_ds,act}_*)"; return 1; }
_wipe_ok "$OUT" || exit 2
echo "== collect_100: OUT=$OUT SEED=$SEED RAND=${RAND:-<none>}"
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"   # shared machine: another project may occupy domain 0 (HISTORY.md 47)

ps -eo pid,args --no-headers | grep "[u]r16e_isaac_ros2.py" \
  | awk -v me="$ME" '$1!=me{print $1}' | while read -r p; do kill "$p" 2>/dev/null; done
sleep 10
nohup /isaac-sim/python.sh "$WS/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py" \
  --asset-path "$WS/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd" \
  --scene pick_place --table --table-height 0.20 --table-pose 0.72,0.0 --table-size 0.70,0.90 \
  --object-pose 0.60,0.0,0.225 --object-size 0.035,0.035,0.035 \
  --place-pose 0.60,0.315,0.0 --object-names red,blue --place-names left,right \
  --object-spacing 0.20 --randomize-object --randomize-radius 0.06 --seed "$SEED" "${RAND_EXTRA[@]}" \
  --camera-res 320x240 "${ISAAC_GRASP[@]}" --with-camera --with-static-cam --headless \
  > "$LOG/${TAG}_isaac.log" 2>&1 &
for _ in $(seq 1 90); do grep -q "scene topics" "$LOG/${TAG}_isaac.log" && break; sleep 5; done
grep -q "scene topics" "$LOG/${TAG}_isaac.log" || { echo "FAIL: isaac"; exit 1; }
# "scene topics" is printed before the first physics step; on a cold shader cache
# /clock only starts minutes later and controllers spawned meanwhile time out.
for _ in $(seq 1 60); do timeout 5 ros2 topic echo --once /clock > /dev/null 2>&1 && break; sleep 5; done
timeout 5 ros2 topic echo --once /clock > /dev/null 2>&1 || { echo "FAIL: isaac never published /clock"; exit 1; }
"$S/bringup.sh" > "$LOG/${TAG}_bringup.log" 2>&1
tail -1 "$LOG/${TAG}_bringup.log" | grep -q "== ready" || { echo "FAIL: bringup"; tail -3 "$LOG/${TAG}_bringup.log"; exit 1; }
echo "   stack ready"

rm -rf "$LOG/grasp_${TAG}"; mkdir -p "$LOG/grasp_${TAG}"
nohup python3 "$S/grasp_watch.py" "$LOG/grasp_${TAG}" 14400 0.45 > "$LOG/grasp_${TAG}.log" 2>&1 &
W=$!
TASKS="red|left|put the red block on the left marker" LOGS="$LOG/${TAG}_logs" \
  "$WS/src/ur_bringup/scripts/collect_il_episodes.sh" "$N" "$OUT"
kill "$W" 2>/dev/null
echo "== grasp frames: $(ls -1 "$LOG/grasp_${TAG}" | wc -l)"
