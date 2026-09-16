#!/usr/bin/env bash
# V8: GR00T N1.7 rollouts across all THREE language-conditioned tasks.
#
# Derived from rollout_wrist1.sh (the ACT harness). What changes, and why:
#
#   --policy_type=groot        (was act)
#   --actions_per_chunk=40     GrootConfig.n_action_steps is 40, ACT's was 50.
#                              Mismatch here does not error, it just desynchronises
#                              the client's queue from what the model produces.
#   no --robot.cameras_ros     UR16eROSConfig's DEFAULT map is already
#                              {exterior: /static_cam/..., wrist: /camera/...},
#                              which is exactly the 2 cameras this dataset has. The
#                              ACT scripts overrode it only because those policies
#                              were wrist-only.
#   --policy_device=cuda       *** AsyncClientConfig.policy_device defaults to
#                              "cpu". The ACT scripts never set it and were fine
#                              -- 80 M fp32 on CPU works. This checkpoint is bf16
#                              (model_params_fp32=false), and bf16 params on CPU
#                              die with "mixed dtype (CPU): expect parameter to
#                              have scalar type of Float". The server logs that as
#                              "Error in StreamActions" and keeps accepting
#                              observations, so the client never crashes -- it just
#                              receives nothing, which reads as a policy that does
#                              nothing. Measured: 0 chunks served across a whole
#                              trial. Latency needs the GPU anyway (V7: 80 ms on
#                              cuda vs a 1333 ms budget; a 3B forward on CPU is not
#                              close).
#   ml_env.sh for the SERVER     the processor pulls the Cosmos-Reason2-2B
#                              tokenizer, so the server needs HF_HOME and the
#                              token. Without it the server dies at load with a
#                              401 and the client just times out -- a failure that
#                              looks like "the policy produced nothing".
#
# Rotates through the 3 tasks instead of repeating one, because both blocks and
# both markers are in every episode: a policy that ignores the instruction and
# always does red->left scores 1/3 here and 10/10 on a single-task harness. That
# difference is the whole point of running GR00T instead of ACT.
#
# Scored with judge_task.py, which separates WRONG_OBJECT / WRONG_PLACE / FAIL --
# a language failure and a grasp failure need opposite fixes.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(cd "$(dirname "$0")" && pwd)"
PER=${1:-3}                    # trials per task
RUN_S=${2:-90}
CKPT="${3:-$WS/outputs/groot_240_v2_rel/checkpoints/last/pretrained_model}"
T="${ROLL_TAG:-gr}"           # log-file prefix (two rollouts in one pipeline must not overwrite each other)
ME=$$
# Every trial starts where the demonstrations started. Datasets/checkpoints from
# before 2026-09-16 start at the old pose (HISTORY.md 47): default ready_v1;
# START_POSE=ready for anything collected with the new READY.
START_POSE="${START_POSE:-ready_v1}"
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"   # shared machine: another project may occupy domain 0 (HISTORY.md 47)
cd "$WS"
source "$WS/src/setup/ml_env.sh"

[ -d "$CKPT" ] || { echo "FAIL: no checkpoint at $CKPT"; exit 1; }
echo "== $PER trials x 3 tasks, ${RUN_S}s each   ckpt=$CKPT"

# The exterior camera is required here (2-camera policy). Say so up front instead
# of letting the client block on a topic that will never publish.
if ! timeout 10 ros2 topic info /static_cam/color/image_raw > /dev/null 2>&1; then
  echo "FAIL: /static_cam/color/image_raw is not advertised."
  echo "      Isaac must run with --with-static-cam (this policy uses 2 cameras)."
  exit 1
fi

TASKS=(
  "red|left|put the red block on the left marker"
  "blue|left|put the blue block on the left marker"
  "red|right|put the red block on the right marker"
)

kill_servers() {
  ps -eo pid,args --no-headers | grep "[p]olicy_server" \
    | awk -v me="$ME" '$1!=me{print $1}' | while read -r p; do kill "$p" 2>/dev/null; done
}

kill_servers
sleep 3
# One server for the whole sweep: a 3B load takes ~40 s, and restarting it per
# trial would cost more than the rollouts. The ACT script restarted per trial to
# guard against a stale checkpoint; here the path is fixed for the whole run, so
# the risk it was guarding against does not exist.
nohup "$WS/deps/.venv-ml/bin/python" -m lerobot.async_inference.policy_server \
  --host=127.0.0.1 --port=8080 > "$LOG/${T}_server.log" 2>&1 &
for _ in $(seq 1 90); do grep -qi "started on" "$LOG/${T}_server.log" && break; sleep 2; done
if ! grep -qi "started on" "$LOG/${T}_server.log"; then
  echo "FAIL: policy server did not start"
  grep -iE "401|gated|Traceback|Error" "$LOG/${T}_server.log" | head -5
  exit 1
fi
trap 'kill_servers' EXIT

declare -A OK CNT
t=0
for i in $(seq 1 "$PER"); do
  for spec in "${TASKS[@]}"; do
    IFS='|' read -r obj plc text <<< "$spec"
    t=$((t+1))
    echo "---- trial $t  ($obj -> $plc)"
    # Same retry as the streaming switch below. When this one silently failed
    # (transient list_controllers timeout), reset_pose got "goal REJECTED by
    # controller" three times and the trial was SKIPped -- correctly, but wasted.
    for a in 1 2 3; do
      python3 "$WS/src/ur_bringup/isaac/common/switch_control_mode.py" trajectory > "$LOG/${T}_switch_traj_$t.log" 2>&1 && break
      echo "     trajectory switch attempt $a: $(tail -1 "$LOG/${T}_switch_traj_$t.log")"; sleep 3
    done
    sleep 2
    ros2 service call /scene/reset_episode std_srvs/srv/Trigger > /dev/null 2>&1

    # Every demonstration starts at READY. A trial that starts anywhere else is
    # not a fair test, so retry and SKIP rather than score it (the ACT harness
    # learned this the hard way -- the failure was being swallowed by >/dev/null).
    parked=0
    for a in 1 2 3; do
      r=$(python3 "$WS/src/ur_bringup/isaac/common/reset_pose.py" "$START_POSE" 2>&1 | tail -1)
      case "$r" in *"error_code: 0"*) parked=1; break;; esac
      echo "     reset_pose attempt $a: $r"
      sleep 2
    done
    [ "$parked" = 1 ] || { echo "     SKIP: could not reach READY"; continue; }
    sleep 2
    # Do NOT send this to /dev/null. It fails with "controller ... is not loaded"
    # when the streaming controller was never spawned, and with the output
    # discarded that failure became "the policy does nothing" for two trials.
    # A wrong control mode is a whole-run fault, not a per-trial one: abort.
    # Retry: controller_manager can miss the 10 s service deadline right after
    # reset_pose (measured: "list_controllers timed out" once, stack healthy).
    # A transient timeout is not the "controller not loaded" fault; only give up
    # after three tries, and print the last output so the two stay distinguishable.
    switched=0
    for a in 1 2 3; do
      if python3 "$WS/src/ur_bringup/isaac/common/switch_control_mode.py" streaming > "$LOG/${T}_switch_$t.log" 2>&1; then
        switched=1; break
      fi
      echo "     switch attempt $a: $(tail -1 "$LOG/${T}_switch_$t.log")"; sleep 3
    done
    if [ "$switched" != 1 ]; then
      echo "FAIL: could not switch to streaming control:"; sed 's/^/       /' "$LOG/${T}_switch_$t.log"; exit 1
    fi
    # awk on the state column: a plain grep for "active" also matches "inactive".
    if ! timeout 10 ros2 control list_controllers 2>/dev/null | awk '$1=="forward_position_controller"{print $NF}' | grep -qx "active"; then
      echo "FAIL: forward_position_controller is not ACTIVE after the switch"; exit 1
    fi

    # Are the observations actually flowing? The first V8 run scored nothing and
    # logged "joint states are stale" for the whole trial -- the policy was never
    # asked anything, yet the trial would have counted as a policy failure. Check
    # the precondition and SKIP instead of scoring a trial that never ran.
    if ! python3 "$S/obs_ready.py"; then
      echo "     SKIP: observations not flowing (see above)"
      continue
    fi

    timeout "$RUN_S" "$WS/deps/.venv-ml/bin/python" -m lerobot.async_inference.robot_client \
      --robot.type=ur16e_ros --robot.id=sim \
      --policy_type=groot --pretrained_name_or_path="$CKPT" \
      --policy_device=cuda \
      --actions_per_chunk=40 --task="$text" \
      --server_address=127.0.0.1:8080 > "$LOG/${T}_client_$t.log" 2>&1

    # Did a chunk actually reach the robot? Count "Observation N | Total time",
    # which the server logs only AFTER postprocessing succeeds.
    #
    # My first version of this check counted "Preprocessing and inference took",
    # and that was wrong: it is logged BEFORE postprocessing. With relative
    # actions the inference succeeded and the postprocessor then raised
    # NotImplementedError, so the count was non-zero while ZERO actions were
    # delivered -- the trial would have been scored as a policy failure again,
    # by the very check meant to prevent that. Count the last step, not the first.
    chunks=$(grep -c "| Total time:" "$LOG/${T}_server.log" 2>/dev/null); chunks=${chunks:-0}   # not `|| echo 0` (grep -c prints 0 and exits 1 -> "0\n0")
    got=$(( chunks - ${SEEN:-0} ))
    SEEN=$chunks
    if [ "$got" -le 0 ]; then
      echo "     SKIP: 0 action chunks delivered this trial -- harness fault, not a policy result"
      grep -oE "Error in StreamActions: .*" "$LOG/${T}_server.log" 2>/dev/null | sort -u | head -1 | cut -c1-160 | sed 's/^/            /'
      grep -oE "Error in observation sender: .*" "$LOG/${T}_client_$t.log" 2>/dev/null | sort -u | head -1 | sed 's/^/            /'
      continue
    fi

    res=$(python3 "$S/judge_task.py" --object="$obj" --place="$plc" 2>/dev/null)
    echo "     $res   (${got} chunks served)"
    k="$obj->$plc"
    CNT[$k]=$(( ${CNT[$k]:-0} + 1 ))
    case "$res" in SUCCESS*) OK[$k]=$(( ${OK[$k]:-0} + 1 ));; esac
  done
done

echo
echo "== per-task results"
tot=0; totok=0
for spec in "${TASKS[@]}"; do
  IFS='|' read -r obj plc text <<< "$spec"
  k="$obj->$plc"
  printf '   %-14s %s/%s\n' "$k" "${OK[$k]:-0}" "${CNT[$k]:-0}"
  tot=$(( tot + ${CNT[$k]:-0} )); totok=$(( totok + ${OK[$k]:-0} ))
done
echo "== total $totok/$tot"
echo "   ACT baseline for reference: 9/10 on ONE task (single-task policy, no language)"
echo "   grep WRONG_OBJECT above: that is the language-reading failure, not a grasp failure"
