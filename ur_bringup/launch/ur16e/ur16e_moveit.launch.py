"""MoveIt2 for UR16e, shared by sim and real.

Thin wrapper over the official ur_moveit_config/ur_moveit.launch.py. The robot
control side (ur16e.launch.py) exposes the same
scaled_joint_trajectory_controller/follow_joint_trajectory action in both
modes, so MoveIt is launched identically; only use_sim_time differs.

  ros2 launch ur_bringup ur16e_moveit.launch.py use_sim:=true   # Isaac
  ros2 launch ur_bringup ur16e_moveit.launch.py use_sim:=false  # real
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    use_sim = LaunchConfiguration("use_sim")
    launch_rviz = LaunchConfiguration("launch_rviz")

    return LaunchDescription([
        DeclareLaunchArgument("use_sim", default_value="true",
                              description="Use sim time (Isaac /clock) for MoveIt"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(get_package_share_directory("ur_moveit_config"),
                             "launch", "ur_moveit.launch.py")
            ),
            launch_arguments={
                "ur_type": "ur16e",
                "use_sim_time": use_sim,
                "launch_rviz": launch_rviz,
            }.items(),
        ),
    ])
