#!/usr/bin/env bash
# Stop everything the GR00T rollout harness started -- and nothing else. This is
# a shared machine (CLAUDE.md: no broad pkill), so: workload-specific patterns
# only, and it lives in a file so the patterns are not in the caller's command
# line (an inline ps|grep|kill matched and killed the calling shell twice).
ME=$$
kill_pat() { for p in $(pgrep -f "$1"); do [ "$p" != "$ME" ] && [ "$p" != "$PPID" ] && kill "$p" 2>/dev/null; done; }
kill_pat "lerobot.async_inference"          # policy_server + robot_client
kill_pat "ur16e_isaac_ros2.py"              # Isaac
kill_pat "ros2 launch ur_bringup"           # launch parents
kill_pat "lib/rviz2/rviz2"
sleep 5
# Launch children that outlive their parent (seen yesterday): kill by exact
# executable path, which nothing outside this workspace's stack runs here.
for exe in /opt/ros/jazzy/lib/controller_manager/ros2_control_node \
           /opt/ros/jazzy/lib/moveit_ros_move_group/move_group \
           /opt/ros/jazzy/lib/robot_state_publisher/robot_state_publisher \
           /opt/ros/jazzy/lib/rclcpp_components/component_container_mt \
           /opt/ros/jazzy/lib/controller_manager/spawner; do
  for p in $(pgrep -f "^$exe"); do kill "$p" 2>/dev/null; done
done
sleep 4
for p in $(pgrep -f "lerobot.async_inference"); do [ "$p" != "$ME" ] && kill -9 "$p" 2>/dev/null; done
echo "=== remaining (should be empty) ==="
ps -eo pid,args --no-headers | grep -E "[a]sync_inference|[u]r16e_isaac_ros2|[r]os2 launch ur_bringup|[r]os2_control_node|[m]ove_group|[r]obot_state_publisher|[c]omponent_container|[r]viz2" | cut -c1-90
echo "=== GPU / shm ==="; nvidia-smi --query-gpu=memory.used,memory.total --format=csv,noheader; df -h /dev/shm | tail -1 | awk '{print "shm", $3"/"$2}'
set +u; source /opt/ros/jazzy/setup.bash 2>/dev/null; set -u; export ROS_DOMAIN_ID=0
timeout 15 ros2 daemon stop >/dev/null 2>&1; for p in $(pgrep -f "ros2cli.daemon"); do kill "$p" 2>/dev/null; done
echo "=== ros2 nodes (after daemon stop) ==="; timeout 15 ros2 node list 2>/dev/null | head -5; echo "(empty = clean)"
