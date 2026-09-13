#!/usr/bin/env bash
# Does the async_inference serving path work for an ABSOLUTE-action GR00T
# checkpoint? 40 s, no scoring -- the only question is whether chunks come out
# the far end of the postprocessor.
#
# Worth its own run before committing 3.7 h to a full absolute training: the
# relative run trained perfectly and then could not be served at all, because the
# smoke checked that A and B both TRAIN and never checked that they SERVE.
# This closes that hole for A before paying for it.
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(cd "$(dirname "$0")" && pwd)"
CKPT="${1:-$WS/outputs/groot_abs_probe/checkpoints/last/pretrained_model}"
RUN_S="${2:-40}"
ME=$$
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID=0
cd "$WS"
source "$WS/src/setup/ml_env.sh"

[ -d "$CKPT" ] || { echo "FAIL: no checkpoint at $CKPT"; exit 1; }

ps -eo pid,args --no-headers | grep "[p]olicy_server" \
  | awk -v me="$ME" '$1!=me{print $1}' | while read -r p; do kill "$p" 2>/dev/null; done
sleep 3

rm -f "$LOG/probe_server.log" "$LOG/probe_client.log"
nohup "$WS/deps/.venv-ml/bin/python" -m lerobot.async_inference.policy_server \
  --host=127.0.0.1 --port=8080 > "$LOG/probe_server.log" 2>&1 &
for _ in $(seq 1 60); do grep -qi "started on" "$LOG/probe_server.log" && break; sleep 2; done
if ! grep -qi "started on" "$LOG/probe_server.log"; then
  echo "FAIL: server did not start"; tail -5 "$LOG/probe_server.log"; exit 1
fi

python3 "$WS/src/ur_bringup/isaac/common/switch_control_mode.py" streaming > /dev/null 2>&1
python3 "$S/obs_ready.py" || { echo "FAIL: observations not flowing"; exit 1; }

timeout "$RUN_S" "$WS/deps/.venv-ml/bin/python" -m lerobot.async_inference.robot_client \
  --robot.type=ur16e_ros --robot.id=sim \
  --policy_type=groot --pretrained_name_or_path="$CKPT" \
  --policy_device=cuda --actions_per_chunk=40 \
  --task="put the red block on the left marker" \
  --server_address=127.0.0.1:8080 > "$LOG/probe_client.log" 2>&1

echo "== device:    $(grep -oE 'Device: [a-z]+' "$LOG/probe_server.log" | tail -1)"
echo "== delivered: $(grep -c '| Total time:' "$LOG/probe_server.log") chunks (post-postprocess)"
grep -oE "Observation [0-9]+ \| Total time: [0-9.]+ms" "$LOG/probe_server.log" | tail -3 | sed 's/^/     /'
echo "== server errors:"
grep -oE "Error in StreamActions: .*" "$LOG/probe_server.log" | sort -u | head -2 | cut -c1-150 | sed 's/^/     /' \
  || echo "     (none)"
echo "== client errors:"
grep -oE "Error in observation sender: .*" "$LOG/probe_client.log" | sort -u | head -2 | sed 's/^/     /' \
  || echo "     (none)"

ps -eo pid,args --no-headers | grep "[p]olicy_server" \
  | awk -v me="$ME" '$1!=me{print $1}' | while read -r p; do kill "$p" 2>/dev/null; done
exit 0
