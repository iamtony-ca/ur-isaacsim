#!/usr/bin/env bash
# Stop everything this workspace's sim/real stack runs, then give a real verdict.
#
# usage: shutdown_rollout.sh [--all] [--dry-run]
#   (default) Isaac, `ros2 launch|run ur_bringup` parents and every node they start --
#             control stack, MoveIt, cuMotion/nvblox containers, teleop, OMY bridge/leader,
#             il_recorder, ur_robot_driver helpers, RViz -- and the policy server/client.
#             Training/conversion (ML venv) and harness loop scripts are left alone.
#   --all     also the harness/collection loop scripts (*.sh under scripts/, killed first so
#             they cannot restart anything) and every process of the ML venv (lerobot-train,
#             raw_to_lerobot.py, DataLoader workers). shutdown_all.sh = this with --all.
#   --dry-run list what would be stopped; kill nothing (last line CLEAN or RUNNING: ...).
# Last line: CLEAN (exit 0) or `LEFT: <pid(name)...> nodes(domain N): ...` (exit 1). The node
# check is `ros2 node list` on $ROS_DOMAIN_ID (unset = 0): nodes there with no process of ours
# left belong to another container on the same domain -- still LEFT, the domain is not free.
# TERM_WAIT (default 30 s) = how long SIGTERM gets before SIGKILL; a ros2_control_node with
# active controllers can outlive SIGTERM by more than 6 s (bringup.sh, HISTORY.md 48.8).
#
# What counts as "ours" -- by executable PATH (argv[0], or the script after an interpreter:
# `python3 X`, `bash X`, `/isaac-sim/python.sh X`), never by a substring of the command line,
# so a caller whose command line merely mentions a pattern is never hit (an inline
# ps|grep|kill killed the calling shell twice), and neither are this script's ancestors
# (groot_pipeline.sh calls it):
#   - $WS/install/** and $WS/src/ur_bringup/{scripts,isaac}/** (offline tools excluded:
#     build_*, convert_*, raw_to_lerobot.py, il_tail_stats.py), ur16e_isaac_ros2.py (Isaac)
#   - $WS/deps/.venv-ml python: `-m lerobot.async_inference.*` and record_rollout.py
#     (with --all: every venv process)
#   - /opt/ros/jazzy/lib/<pkg>/<exe> from the list below, `ros2 launch|run ur_bringup`, and
#     ur_robot_driver's socat PTY bridge -- ONLY when the process environment has this
#     workspace's install sourced (AMENT_PREFIX_PATH contains $WS/install/). The same
#     binaries started by another workspace in this container are reported as "not ours"
#     and left running.
# The ros2 daemon of OUR domain is stopped (the old version killed every domain's daemon).
set -uo pipefail
S="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${UR_WS:-$(cd "$S/../../../.." && pwd)}"   # workspace root (override: UR_WS=...)
ALL=0; DRY=0
for a in "$@"; do
  case $a in
    --all) ALL=1 ;;
    --dry-run) DRY=1 ;;
    -h|--help) sed -n '2,/^set -uo/p' "$0" | sed '$d'; exit 0 ;;
    *) echo "unknown argument: $a (--all | --dry-run)" >&2; exit 2 ;;
  esac
done
TERM_WAIT="${TERM_WAIT:-30}"
DOMAIN="${ROS_DOMAIN_ID:-0}"

# /opt/ros/jazzy/lib/<pkg>/<exe> started by this workspace's launch files (and the ones
# they include: ur_control.launch.py, omy_l100_leader_ai.launch.py).
ROS_EXES=" controller_manager/ros2_control_node controller_manager/spawner
 controller_manager/unspawner controller_manager/hardware_spawner
 robot_state_publisher/robot_state_publisher moveit_ros_move_group/move_group
 rclcpp_components/component_container rclcpp_components/component_container_mt
 rclcpp_components/component_container_isolated rviz2/rviz2
 tf2_ros/static_transform_publisher moveit_servo/servo_node joy/joy_node
 nvblox_ros/nvblox_node realsense2_camera/realsense2_camera_node
 ur_robot_driver/dashboard_client ur_robot_driver/robot_state_helper
 ur_robot_driver/urscript_interface ur_robot_driver/controller_stopper_node
 ur_robot_driver/trajectory_until_node ur_robot_driver/wait_for_robot_description
 ur_robot_driver/tool_communication.py ur_client_library/tool_communication.py "
ROS_EXES=$(echo $ROS_EXES)" "; ROS_EXES=" $ROS_EXES"   # one space between entries

# This script and every ancestor (the caller, a pipeline above it, the terminal) are off limits.
SKIP=" "; p=$$
while [ -n "$p" ] && [ "$p" -gt 1 ] 2>/dev/null; do
  SKIP+="$p "; p=$(awk '/^PPid:/{print $2}' "/proc/$p/status" 2>/dev/null)
done

absp() {   # $1 word, $2 pid -> absolute path (relative words resolved against the process cwd)
  local c
  case $1 in
    /*) printf '%s' "$1" ;;
    *) c=$(readlink "/proc/$2/cwd" 2>/dev/null) || c=/nonexistent
       realpath -ms "$c/$1" 2>/dev/null || printf '%s/%s' "$c" "$1" ;;
  esac
}
ours_env() {   # $1 pid: was it started with this workspace's install sourced?
  local e
  e=$(tr '\0' '\n' 2>/dev/null < "/proc/$1/environ" | sed -n 's/^AMENT_PREFIX_PATH=//p' | tail -1)
  case ":$e:" in *":$WS/install/"*) return 0 ;; esac
  return 1
}

classify() {   # $1 pid -> CLS (stack | loop | ml | foreign | "") and WHAT
  CLS=""; WHAT=""
  local pid=$1 interp="" prog="" mod="" w b i=0 n rel=0
  local -a a
  mapfile -d '' a 2>/dev/null < "/proc/$pid/cmdline" || return 0
  n=${#a[@]}; [ "$n" -gt 0 ] || return 0             # kernel threads, zombies
  while [ "$i" -lt "$n" ]; do                         # skip interpreters and their options
    w=${a[$i]}
    case ${w##*/} in
      python|python[0-9]*|bash|sh|dash|python.sh)
        [ -n "$interp" ] || interp=$(absp "$w" "$pid"); i=$((i+1))
        while [ "$i" -lt "$n" ]; do
          case ${a[$i]} in
            -m) mod=${a[$((i+1))]:-}; i=$n ;;
            -c|-) i=$n ;;                               # inline code: never ours by path
            -*) i=$((i+1)); continue ;;
          esac
          break
        done ;;
      *) case $w in
           /*) prog=$w ;;
           */*) rel=1; prog=$(absp "$w" "$pid") ;;
           *) if [ "$i" -gt 0 ]; then rel=1; prog=$(absp "$w" "$pid")   # `python3 x.py`: cwd-relative
              else prog=$w; fi ;;                    # bare argv[0] = found on PATH, not a path
         esac
         break ;;
    esac
  done
  b=${prog##*/}
  case $b in shutdown_rollout.sh|shutdown_all.sh) return 0 ;; esac
  case $interp in
    "$WS"/deps/.venv-ml/bin/python*)
      if [[ $mod == lerobot.async_inference.* ]]; then CLS=stack; WHAT=${mod##*.}
      elif [ "$prog" = "$WS/src/ur_bringup/scripts/harness/record_rollout.py" ]; then CLS=stack; WHAT=$b
      else CLS=ml; WHAT=${b:-${mod:-venv-python}}; fi
      return 0 ;;
  esac
  [ -n "$prog" ] || return 0
  if [ "$b" = ur16e_isaac_ros2.py ] && [ "$prog" != "$b" ] && { [[ $prog == "$WS"/src/* ]] || { [ "$rel" = 1 ] && [ ! -e "$prog" ]; }; }; then
    CLS=stack; WHAT=$b; return 0                      # Isaac (relative path + Kit changed its cwd)
  fi
  case $prog in
    "$WS"/install/*|"$WS"/src/ur_bringup/scripts/*|"$WS"/src/ur_bringup/isaac/*)
      case $b in
        *.sh) CLS=loop ;;
        build_*|convert_*|raw_to_lerobot.py|il_tail_stats.py|bench_*) ;;   # offline tools
        *) CLS=stack ;;
      esac
      WHAT=$b; return 0 ;;
    /opt/ros/jazzy/bin/ros2)
      [[ ${a[$((i+1))]:-} =~ ^(launch|run)$ && ${a[$((i+2))]:-} == ur_bringup ]] || return 0
      WHAT="ros2_${a[$((i+1))]}_${a[$((i+3))]:-}" ;;
    /opt/ros/jazzy/lib/*)
      [[ $ROS_EXES == *" ${prog#/opt/ros/jazzy/lib/} "* ]] || return 0
      WHAT=$b ;;
    *)
      [ "$b" = socat ] && [[ " ${a[*]} " == *" pty,"*"link="* ]] || return 0
      WHAT=socat ;;
  esac
  if ours_env "$pid"; then CLS=stack; else CLS=foreign; fi
}

scan() {   # -> TARGETS ("pid:class:name", per mode) and FOREIGN
  TARGETS=(); FOREIGN=()
  local d pid
  for d in /proc/[0-9]*; do
    pid=${d#/proc/}
    case $SKIP in *" $pid "*) continue ;; esac
    classify "$pid"
    case $CLS in
      stack) TARGETS+=("$pid:$CLS:$WHAT") ;;
      loop|ml) [ "$ALL" = 1 ] && TARGETS+=("$pid:$CLS:$WHAT") ;;
      foreign) FOREIGN+=("$pid:$WHAT") ;;
    esac
  done
}
names() { local t; for t in "$@"; do printf '%s(%s) ' "${t%%:*}" "${t##*:}"; done; }
declare -A TERMED=()
signal() {   # $1 signal, $2 class filter (regex) -- over the current TARGETS; TERM once per pid
  local t pid
  for t in "${TARGETS[@]}"; do
    [[ ${t#*:} =~ ^($2): ]] || continue
    pid=${t%%:*}
    if [ "$1" = TERM ]; then [ -n "${TERMED[$pid]:-}" ] && continue; TERMED[$pid]=1; fi
    kill "-$1" "$pid" 2>/dev/null && echo "   $1 $pid ${t##*:}"
  done
}

echo "=== shutdown ($([ "$ALL" = 1 ] && echo all || echo rollout)$([ "$DRY" = 1 ] && echo ', dry-run'); WS=$WS, ROS_DOMAIN_ID=$DOMAIN)"
scan
if [ "$DRY" = 1 ]; then
  echo "would stop: ${#TARGETS[@]}"
  for t in "${TARGETS[@]}"; do echo "   ${t%%:*} ${t#*:}"; done
else
  if [ "$ALL" = 1 ] && [[ " ${TARGETS[*]} " == *":loop:"* ]]; then
    signal TERM loop; sleep 1; scan                  # loops first: they would restart things
  fi
  signal TERM 'stack|loop|ml'
  for ((s = 0; s < TERM_WAIT; s++)); do      # newcomers (a launch still tearing down) get TERM too
    sleep 1; scan; [ ${#TARGETS[@]} -eq 0 ] && break; signal TERM 'stack|loop|ml'
  done
  if [ ${#TARGETS[@]} -gt 0 ]; then
    echo "   still alive after ${TERM_WAIT} s -> KILL"; signal KILL 'stack|loop|ml'
  fi
  sleep 2; scan   # verdict scan after a settle: something that keeps restarting a node shows up here
fi
[ ${#FOREIGN[@]} -gt 0 ] && echo "not ours (left running): $(names "${FOREIGN[@]}")"

NODES=""
if [ -f /opt/ros/jazzy/setup.bash ]; then
  set +u; source /opt/ros/jazzy/setup.bash 2>/dev/null; set -u; export ROS_DOMAIN_ID="$DOMAIN"
  if [ "$DRY" = 0 ]; then
    timeout 15 ros2 daemon stop >/dev/null 2>&1
    for p in $(pgrep -f "ros2cli\.daemon\.daemonize.* --ros-domain-id $DOMAIN( |$)"); do kill "$p" 2>/dev/null; done
    [[ " ${FOREIGN[*]} " == *":spawner "* ]] ||
      rm -f ~/.ros/locks/ros2-control-controller-spawner.lock   # orphaned-spawner lock (HISTORY.md 47.4)
  fi
  NODES=$(timeout 30 ros2 node list --no-daemon --spin-time 3 2>/dev/null | tr '\n' ' ' | sed 's/ *$//')
else
  echo "(no /opt/ros/jazzy -- node check skipped)"
fi
if command -v nvidia-smi >/dev/null; then
  echo "GPU $(nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader | head -1)  shm $(df -h /dev/shm | awk 'NR==2{print $3"/"$2}')"
fi
if [ ${#TARGETS[@]} -eq 0 ] && [ -z "$NODES" ]; then echo "CLEAN"; exit 0; fi
[ ${#TARGETS[@]} -eq 0 ] &&
  echo "(no process of ours left: these nodes belong to another workspace/container on domain $DOMAIN)"
echo "$([ "$DRY" = 1 ] && echo RUNNING || echo LEFT): $(names "${TARGETS[@]}")${NODES:+nodes(domain $DOMAIN): $NODES}"
exit 1
