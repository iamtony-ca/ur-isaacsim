#!/usr/bin/env bash
# Clear leftover inference clients and spawn the streaming controller on a LIVE
# stack. Lives in a file so the kill pattern is not in the caller's command line
# (an inline `ps | grep pattern | kill` killed the calling shell once).
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(cd "$(dirname "$0")" && pwd)"; ME=$$
set +u; source /opt/ros/jazzy/setup.bash; source "$WS/install/setup.bash"; set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"   # shared machine: another project may occupy domain 0 (HISTORY.md 47)
for p in $(pgrep -f "lerobot.async_inference"); do [ "$p" != "$ME" ] && kill -9 "$p" 2>/dev/null; done
sleep 2
pgrep -f "lerobot.async_inference" >/dev/null && echo "clients: STILL PRESENT" || echo "clients: cleared"
pgrep -f ur16e_isaac_ros2.py >/dev/null && echo "isaac: alive" || { echo "isaac: DEAD"; exit 1; }
pgrep -f ros2_control_node >/dev/null && echo "control: alive" || { echo "control: DEAD"; exit 1; }
if ! timeout 10 ros2 control list_controllers 2>/dev/null | grep -q "^forward_position_controller"; then
  timeout 90 ros2 launch ur_bringup policy_inference.launch.py use_sim:=true > "$LOG/gr8_pi.log" 2>&1
fi
echo "=== controllers ==="; timeout 10 ros2 control list_controllers 2>/dev/null | awk '{print "  "$1, $NF}'
