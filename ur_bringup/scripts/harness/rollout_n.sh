#!/usr/bin/env bash
# N independent rollouts of a trained policy against the running Isaac scene.
# One success is not a success rate -- the first rollout of act_red_left_thin
# worked, and that says nothing about how often it works.
#
# Each trial: reset (new object pose) -> READY -> verify start state -> run the
# policy -> judge from GT (block within place_tol of the marker AND released).
# The policy server is started ONCE per run, before the first trial, after killing
# any leftover one -- a stale server cannot serve an old checkpoint (it holds :8080
# and the client never says which model answered). Each trial starts a new client.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(cd "$(dirname "$0")" && pwd)"
source "$S/grasp_mode.sh"   # GRASP_MODE=physical (default) | attach -- HISTORY.md 53
N=${1:-6}
RUN_S=${2:-90}
# Default = the same checkpoint rollout_gui_then_n.sh passes: the train_100.sh output
# for this GRASP_MODE. On another PC pass the one you carried over, e.g.
#   .../act_red_left_190_phys/checkpoints/200000/pretrained_model      (final, HISTORY.md 58.7)
#   .../act_red_left_100_phys_long/checkpoints/200000/pretrained_model (100-ep, 54.6)
CKPT="${3:-$WS/outputs/act_red_left_100$GRASP_SFX/checkpoints/last/pretrained_model}"
ME=$$
# Every trial starts where the demonstrations started: START_POSE comes from
# grasp_mode.sh -- ready for physical-mode data, ready_v1 for the weld-era
# checkpoints collected before 2026-09-16 (HISTORY.md 47; red_left_100,
# wrist_only and 3task_v3 alike -- see reset_pose.py).
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"   # shared machine: another project may occupy domain 0 (HISTORY.md 47)
cd "$WS"
[ -d "$CKPT" ] || { echo "FAIL: no checkpoint at $CKPT (pass it as the 3rd argument)"; exit 1; }
echo "== $N rollouts x ${RUN_S}s  ckpt=$CKPT"

ps -eo pid,args --no-headers | grep "[p]olicy_server" \
  | awk -v me="$ME" '$1!=me{print $1}' | while read -r p; do kill "$p" 2>/dev/null; done
sleep 3
nohup "$WS/deps/.venv-ml/bin/python" -m lerobot.async_inference.policy_server \
  --host=127.0.0.1 --port=8080 > "$LOG/rn_server.log" 2>&1 &
for _ in $(seq 1 45); do grep -qi "started on" "$LOG/rn_server.log" && break; sleep 2; done
grep -qi "started on" "$LOG/rn_server.log" || { echo "FAIL: policy server"; exit 1; }

ok=0
for i in $(seq 1 "$N"); do
  echo "---- trial $i/$N"
  python3 "$WS/src/ur_bringup/isaac/common/switch_control_mode.py" trajectory > /dev/null 2>&1
  sleep 2
  ros2 service call /scene/reset_episode std_srvs/srv/Trigger > /dev/null 2>&1
  # reset_pose was failing with error_code -1 and the failure was being swallowed
  # by >/dev/null, so trials started from wherever the last one stopped -- every
  # demonstration begins at READY, so that alone can sink a trial. Retry, and if
  # it still will not go, say so instead of scoring a rollout that never had the
  # right initial state.
  parked=0
  for a in 1 2 3; do
    r=$(python3 "$WS/src/ur_bringup/isaac/common/reset_pose.py" "$START_POSE" 2>&1 | tail -1)
    case "$r" in *"error_code: 0"*) parked=1; break;; esac
    echo "     reset_pose attempt $a: $r"
    sleep 2
  done
  [ "$parked" = 1 ] || { echo "     SKIP: could not reach READY"; continue; }
  sleep 2
  python3 "$WS/src/ur_bringup/isaac/common/switch_control_mode.py" streaming > /dev/null 2>&1
  # Optional video of the trial (RECORD_DIR set): listens to the camera topics only, so it
  # cannot change the rollout. Named after the judge result below.
  REC=""
  if [ -n "${RECORD_DIR:-}" ]; then
    mkdir -p "$RECORD_DIR"
    "$WS/deps/.venv-ml/bin/python" "$S/record_rollout.py" --out "$RECORD_DIR/trial_$(printf %02d "$i").mp4" \
      --label "$(basename "$(dirname "$(dirname "$(dirname "$CKPT")")")")  trial $i/$N" > "$LOG/rec_$i.log" 2>&1 &
    REC=$!
  fi
  timeout "$RUN_S" "$WS/deps/.venv-ml/bin/python" -m lerobot.async_inference.robot_client \
    --robot.type=ur16e_ros --robot.id=sim \
    --policy_type=act --pretrained_name_or_path="$CKPT" \
    --actions_per_chunk=50 --task="put the red block on the left marker" \
    --server_address=127.0.0.1:8080 > "$LOG/rn_client_$i.log" 2>&1
  # Where did this trial start? Read it back from Isaac's own reset line.
  IL=$(ls -t "$LOG"/*isaac.log 2>/dev/null | head -1)
  pos=$(grep -a "reset_episode: red@" "$IL" 2>/dev/null | tail -1 | sed 's/.*red@//; s/ yaw.*//')
  if [ -n "$REC" ]; then
    kill -INT "$REC" 2>/dev/null; wait "$REC" 2>/dev/null
  fi
  res=$(python3 "$S/judge_rollout.py" 2>/dev/null)
  if [ -n "$REC" ] && [ -f "$RECORD_DIR/trial_$(printf %02d "$i").mp4" ]; then
    mv "$RECORD_DIR/trial_$(printf %02d "$i").mp4" "$RECORD_DIR/trial_$(printf %02d "$i")_$(echo "$res" | awk '{print $1}').mp4"
  fi
  echo "     start=$pos  $res"
  case "$res" in SUCCESS*) ok=$((ok+1));; esac
done
echo "== $ok/$N rollouts succeeded"
ps -eo pid,args --no-headers | grep "[p]olicy_server" \
  | awk -v me="$ME" '$1!=me{print $1}' | while read -r p; do kill "$p" 2>/dev/null; done
