cmake_minimum_required(VERSION 3.8)
project(ur_bringup)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY launch config urdf srdf isaac meshes cumotion lerobot_robot_ur16e_ros
  DESTINATION share/${PROJECT_NAME}
)

# ROS nodes we launch with Node(...) rather than `python3 <path>`.
# (The isaac/ scripts stay data-only: they are run via /isaac-sim/python.sh or
#  plain python3 with an explicit path, so they do not belong here.)
install(
  PROGRAMS
    scripts/teleop_joy.py
    scripts/il_recorder.py
    scripts/raw_to_lerobot.py
    scripts/omy_to_ur16e.py
    scripts/virtual_omy_leader.py
    scripts/omy_leader_calib.py
    scripts/pick_place_demo.py
    scripts/collect_il_episodes.sh
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
