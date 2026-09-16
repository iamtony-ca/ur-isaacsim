#!/usr/bin/env bash
# V8 driver: bring up the SAME scene the data was collected in, then roll out.
#
# The Isaac arguments below are copied verbatim from collect240.sh (which produced
# lerobot_ds_240_v2). They are not a reasonable-looking reconstruction -- table
# height, object size, marker distance, spacing, camera resolution and the
# randomisation radius all have to match what the policy saw, or a failure cannot
# be attributed to the policy. In particular:
#
#   --camera-res 320x240   ACT does not resize, and GR00T resizes to 256x256 from
#                          whatever it gets; feeding 640x480 here changes the crop
#                          geometry relative to training.
#   --with-static-cam      the exterior camera. This policy has TWO camera inputs;
#                          without it the client waits forever on a dead topic.
#   --randomize-object     leave it ON. Rolling out on the fixed nominal pose would
#                          measure memorisation, which is exactly the thing 7
#                          episodes per task is at risk of.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(cd "$(dirname "$0")" && pwd)"
PER=${1:-3}
RUN_S=${2:-90}
CKPT="${3:-$WS/outputs/groot_240_v2_rel/checkpoints/last/pretrained_model}"
# Scene randomisation MUST match the collection (collect240.sh takes the same two).
# Defaults = 240_v2 / the 44.3 rollout. v3 (HISTORY.md 46): RADIUS=0.06, a seed
# different from the collection's so the poses are new, not replayed.
RADIUS="${RADIUS:-0.025}"
SEED="${SEED:-0}"
TAG="${TAG:-gr8}"           # log-file prefix; groot_rollout.sh gets ROLL_TAG
ME=$$
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID=0
cd "$WS"

# Workload-specific only -- this machine is shared and a broad pkill is forbidden.
ps -eo pid,args --no-headers | grep "[u]r16e_isaac_ros2.py" \
  | awk -v me="$ME" '$1!=me{print $1}' | while read -r p; do kill "$p" 2>/dev/null; done
ps -eo pid,args --no-headers | grep "[p]olicy_server" \
  | awk -v me="$ME" '$1!=me{print $1}' | while read -r p; do kill "$p" 2>/dev/null; done
sleep 10

echo "== isaac (3-task scene, 320x240, static cam, radius $RADIUS, seed $SEED)"
nohup /isaac-sim/python.sh "$WS/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py" \
  --asset-path "$WS/src/ur_bringup/isaac/assets/ur16e_2f85_d405.usd" \
  --scene pick_place --table --table-height 0.20 --table-pose 0.72,0.0 --table-size 0.70,0.90 \
  --object-pose 0.60,0.0,0.225 --object-size 0.035,0.035,0.035 \
  --place-pose 0.60,0.315,0.0 --object-names red,blue --place-names left,right \
  --object-spacing 0.20 --randomize-object --randomize-radius "$RADIUS" --seed "$SEED" \
  --camera-res 320x240 --grasp-attach --with-camera --with-static-cam --headless \
  > "$LOG/${TAG}_isaac.log" 2>&1 &
for _ in $(seq 1 90); do grep -q "scene topics" "$LOG/${TAG}_isaac.log" && break; sleep 5; done
grep -q "scene topics" "$LOG/${TAG}_isaac.log" || { echo "FAIL: isaac"; tail -5 "$LOG/${TAG}_isaac.log"; exit 1; }
grep -E "eye-in-hand camera|static cam  " "$LOG/${TAG}_isaac.log"

echo "== control stack"
"$S/bringup.sh" > "$LOG/${TAG}_bringup.log" 2>&1
tail -1 "$LOG/${TAG}_bringup.log" | grep -q "== ready" || {
  echo "FAIL: bringup"; tail -8 "$LOG/${TAG}_bringup.log"; exit 1; }
echo "   stack ready"

# Spawn forward_position_controller (INACTIVE). bringup.sh does not: only the
# teleop launches and policy_inference.launch.py do. Without it,
# switch_control_mode.py streaming fails, the policy publishes into a topic
# nobody serves, and the arm never moves -- which looks exactly like a broken
# policy (the launch's own docstring says so; I still left it out the first time
# and burned two trials on a stationary arm). The spawner exits on its own;
# timeout is a guard against a hung controller_manager, same as the ACT driver.
echo "== streaming controller"
timeout 90 ros2 launch ur_bringup policy_inference.launch.py use_sim:=true > "$LOG/${TAG}_pi.log" 2>&1
if ! timeout 10 ros2 control list_controllers 2>/dev/null | grep -q "^forward_position_controller"; then
  echo "FAIL: forward_position_controller not loaded after policy_inference.launch.py"
  tail -5 "$LOG/${TAG}_pi.log"; exit 1
fi
echo "   forward_position_controller loaded"

# `bash <script>`, not exec: nothing in the scratchpad is chmod +x, and exec'ing
# a non-executable file fails AFTER Isaac and the control stack are already up --
# so the cost of that mistake is a full 2-minute scene reload, not an error.
exec bash "$S/groot_rollout.sh" "$PER" "$RUN_S" "$CKPT"
