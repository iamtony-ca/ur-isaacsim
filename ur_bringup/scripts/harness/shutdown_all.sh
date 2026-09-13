#!/usr/bin/env bash
# Stop everything this workspace started. Workload-specific patterns ONLY --
# CLAUDE.md forbids broad pkill here because the machine is shared with other
# projects. Never kills this shell.
set -uo pipefail
ME=$$
for pat in "[u]r16e_isaac_ros2.py" "[p]olicy_server" "[r]obot_client" \
           "[r]os2 launch ur_bringup" "[r]os2_control_node" "[r]obot_state_publisher" \
           "[m]oveit_ros_move_group/move_group" "[l]ib/rviz2/rviz2" \
           "[p]ick_place_demo" "[i]l_recorder.py" "[l]erobot-train" "[r]aw_to_lerobot.py" \
           "[f]ull_res""tart.sh" "[c]ollect240.sh" "[i]nfer_gui.sh"; do
  ps -eo pid,args --no-headers | grep -- "$pat" \
    | awk -v me="$ME" '$1 != me {print $1}' \
    | while read -r p; do kill "$p" 2>/dev/null; done
done
sleep 8
# Anything that ignored SIGTERM
for pat in "[u]r16e_isaac_ros2.py" "[p]olicy_server" "[r]os2_control_node"; do
  ps -eo pid,args --no-headers | grep -- "$pat" \
    | awk -v me="$ME" '$1 != me {print $1}' \
    | while read -r p; do kill -9 "$p" 2>/dev/null; done
done
sleep 3
echo "남은 프로세스:"
for pat in "[u]r16e_isaac_ros2.py" "[p]olicy_server" "[r]os2_control_node" \
           "[r]obot_state_publisher" "[m]ove_group" "[l]erobot"; do
  printf "  %-34s %s\n" "${pat//[\[\]]/}" "$(ps -eo args --no-headers | grep -c -- "$pat")"
done
echo "GPU:"; nvidia-smi --query-gpu=memory.used,utilization.gpu --format=csv,noheader | head -1
