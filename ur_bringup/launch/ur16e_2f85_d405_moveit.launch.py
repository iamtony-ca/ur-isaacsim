"""MoveIt2 for UR16e + 2F-85 + D405 (Set 3): collision-aware gripper + depth->OctoMap.

Same as ur16e_2f85_moveit.launch.py (combined gripper SRDF over ur_moveit_config's
kinematics/pipelines), PLUS a 3D perception sensor: the D405 depth point cloud is
integrated into an OctoMap that move_group treats as collision geometry, so the
planner avoids whatever the eye-in-hand camera sees. The camera body itself is
visual-only (no SRDF camera disables needed); obstacle avoidance is the OctoMap.

Pair with the camera control stack:
    ros2 launch ur_bringup ur16e_2f85_d405.launch.py
    ros2 launch ur_bringup ur16e_2f85_d405_moveit.launch.py     # use_octomap:=true (default)

OctoMap needs ros-jazzy-moveit-ros-perception (PointCloudOctomapUpdater plugin) +
the camera publishing /camera/depth/color/points. use_octomap:=false falls back to
the plain collision-aware gripper MoveIt (no perception plugin required).
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder

# OctoMap built in this fixed frame (the combined URDF root) so it persists as
# the arm moves; the D405 cloud is transformed into it via TF.
OCTOMAP_FRAME = "world"
OCTOMAP_RESOLUTION = 0.02  # m


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_octomap = LaunchConfiguration("use_octomap")

    pkg = FindPackageShare("ur_bringup")

    # combined SRDF (arm + gripper disables); the camera is collision-free so it
    # needs no extra disable rules — Set 3 reuses the gripper SRDF as-is.
    robot_description_semantic = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([pkg, "srdf", "ur16e_2f85.srdf.xacro"]), " ",
            "name:=ur16e",
        ]),
        value_type=str,
    )
    semantic_param = {"robot_description_semantic": robot_description_semantic}

    sensors_3d_path = os.path.join(
        get_package_share_directory("ur_bringup"), "config", "sensors_3d.yaml"
    )

    # Two configs: octomap one loads sensors_3d, plain one doesn't (so
    # use_octomap:=false needs no perception plugin). Both reuse ur_moveit_config
    # kinematics/pipelines/joint-limits; combined SRDF applied via override below.
    def build_config(with_octomap):
        b = (
            MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
            .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur16e"})
        )
        if with_octomap:
            b = b.sensors_3d(sensors_3d_path)
        return b.to_moveit_configs()

    cfg_plain = build_config(False)
    cfg_octo = build_config(True)

    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    common_params = {"use_sim_time": use_sim_time, "publish_robot_description_semantic": True}

    move_group_octo = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        condition=IfCondition(use_octomap),
        parameters=[
            cfg_octo.to_dict(),  # includes sensors_3d (D405 octomap updater)
            semantic_param,
            common_params,
            {"octomap_frame": OCTOMAP_FRAME, "octomap_resolution": OCTOMAP_RESOLUTION},
        ],
    )
    move_group_plain = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        condition=UnlessCondition(use_octomap),
        parameters=[cfg_plain.to_dict(), semantic_param, common_params],
    )

    rviz_config = PathJoinSubstitution(
        [FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"]
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config],
        parameters=[
            cfg_plain.robot_description_kinematics,
            cfg_plain.planning_pipelines,
            cfg_plain.joint_limits,
            semantic_param,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument(
            "use_octomap", default_value="true",
            description="Build an OctoMap from the D405 depth cloud for collision "
                        "avoidance. Needs ros-jazzy-moveit-ros-perception + the camera. "
                        "Set false for the plain collision-aware gripper MoveIt (no perception)."),
        wait_robot_description,
        RegisterEventHandler(
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[move_group_octo, move_group_plain, rviz_node],
            )
        ),
    ])
