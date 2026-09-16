#!/usr/bin/env bash
# Restart the sim control stack + MoveIt cleanly, leaving Isaac alone.
#
# Kills by PID, never by broad pattern:
#   * CLAUDE.md forbids broad pkill on this shared machine, and
#   * `pkill -f <pattern>` has twice matched THIS script's own command line.
# Also verifies there is exactly ONE robot_state_publisher afterwards: a stale RSP
# survived a launch-parent kill once and re-published an OLD /robot_description,
# which move_group latched -- every Cartesian goal then silently "succeeded"
# without moving (see HISTORY.md).
set -uo pipefail
WS="${UR_WS:-$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
LOG="${HARNESS_LOG:-$WS/outputs/harness_logs}"; mkdir -p "$LOG"   # logs never go into the source tree
S="$(dirname "$0")"

# ROS's setup.bash reads AMENT_TRACE_SETUP_FILES et al. without defaulting them,
# so sourcing it under `set -u` aborts with "unbound variable" (HISTORY.md 23 --
# the identical trap, repeated here).
set +u
source /opt/ros/jazzy/setup.bash
source "$WS/install/setup.bash"
set -u
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"   # shared machine: another project may occupy domain 0 (HISTORY.md 47)

# controller_manager/spawner is in the list on purpose: a spawner orphaned by a dead
# controller_manager keeps ~/.ros/locks/ros2-control-controller-spawner.lock and every
# later spawner then dies with "Failed to acquire lock" (HISTORY.md 47).
echo "== stopping control stack + MoveIt (Isaac untouched)"
# `ros2 launch ur_bringup` parents are in the list too: killing only the children
# leaves the launch process to tear down at its own pace. And the wait is a LOOP,
# not a fixed sleep: a ros2_control_node with active controllers can outlive
# `kill` by more than 6 s, and the next stack's spawners then attach to the OLD
# controller_manager ("A controller named ... was already loaded", "can not be
# configured from 'active' state") -- seen 2026-09-16 (HISTORY.md 48.8).
STOP_PAT="ros2 launch ur_bringup|ros2_control_node|robot_state_publisher|moveit_ros_move_group/move_group|lib/rviz2/rviz2|rclcpp_components/component_container|controller_manager/spawner"
ME=$$
_stack_pids() { ps -eo pid,args --no-headers | grep -E "$STOP_PAT" | grep -v grep | awk -v me="$ME" -v pp="$PPID" '$1!=me && $1!=pp {print $1}'; }
_stack_pids | while read -r p; do kill "$p" 2>/dev/null && echo "   TERM $p"; done
for _ in $(seq 1 15); do [ -z "$(_stack_pids)" ] && break; sleep 2; done
if [ -n "$(_stack_pids)" ]; then
  echo "   still alive after 30 s -> KILL"; _stack_pids | while read -r p; do kill -9 "$p" 2>/dev/null; done; sleep 3
fi
rm -f ~/.ros/locks/ros2-control-controller-spawner.lock   # orphaned-spawner lock (HISTORY.md 47.4)
sleep 2

echo "== starting control stack"
nohup ros2 launch ur_bringup ur16e_2f85_d405.launch.py use_sim:=true > "$LOG/control.log" 2>&1 &
for _ in $(seq 1 60); do
  grep -q "Configured and activated scaled_joint_trajectory_controller" "$LOG/control.log" && break
  sleep 2
done
grep -q "Configured and activated scaled_joint_trajectory_controller" "$LOG/control.log" \
  || { echo "   FAIL: controllers never activated"; tail -5 "$LOG/control.log"; exit 1; }
echo "   controllers active"

n=$(ps -eo args --no-headers | grep -c "[r]obot_state_publisher")
[ "$n" = 1 ] || { echo "   FAIL: $n robot_state_publishers (must be 1)"; exit 1; }
echo "   exactly 1 robot_state_publisher"

echo "== starting MoveIt (cuMotion)"
# ur_only:=false is REQUIRED. It defaults to TRUE, which gives move_group the
# UR-ARM-ONLY model (urdf/ur16e/ur16e_sim.urdf.xacro) for interactive RViz. With
# that model there is no gripper_frame, and MoveIt reports an unknown constraint
# link as an ALREADY-SATISFIED goal: every Cartesian goal returns SUCCESS and the
# arm never moves. The launch's own help says false = "programmatic cuMotion",
# which is exactly what pick_place_demo.py is.
nohup ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py \
  use_sim:=true ur_only:=false > "$LOG/moveit.log" 2>&1 &
# "Ready to take commands for planning group" is printed by RViz's MoveGroupInterface,
# NOT by move_group -- so on a headless container (no X socket, RViz dies) it never
# appears. move_group's own line is "You can start planning now!". Accept either; the
# action-list check below is the real readiness gate (2026-09-16, HISTORY.md 48).
MG_READY="Ready to take commands for planning group|You can start planning now"
for _ in $(seq 1 60); do
  grep -qE "$MG_READY" "$LOG/moveit.log" && break
  sleep 2
done
grep -qE "$MG_READY" "$LOG/moveit.log" \
  || { echo "   FAIL: move_group not ready"; tail -5 "$LOG/moveit.log"; exit 1; }
echo "   move_group ready"

# The model check lives in pick_place_demo.py (check_ee_link_known), which asks
# /compute_fk -- the only source that reflects move_group's actual kinematic model.
# Two checks that DO NOT work were tried here first:
#   * grepping the log for "gripper_frame not found" -- only emitted when a
#     constraint is evaluated, so it is always absent at bring-up time;
#   * reading the `robot_description` PARAMETER -- with ur_only:=false the model
#     arrives on the topic and the parameter still holds an arm-only default, so
#     this reported a false failure.

# Retry: move_group logs "ready" before its action shows up in discovery, so a
# single immediate check fails intermittently (it did, after a clean bring-up).
for a in /move_action /gripper_controller/gripper_cmd; do
  for _ in $(seq 1 15); do
    ros2 action list 2>/dev/null | grep -qx "$a" && break
    sleep 2
  done
  ros2 action list 2>/dev/null | grep -qx "$a" \
    && echo "   action ok: $a" || { echo "   FAIL: missing action $a"; exit 1; }
done
echo "== ready"
