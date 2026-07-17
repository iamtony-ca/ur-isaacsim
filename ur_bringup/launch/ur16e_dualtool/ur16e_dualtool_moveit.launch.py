"""MoveIt2 for UR16e + dual-tool EOAT (collision-aware). Mirrors
ur16e_2f85_moveit.launch.py: feeds move_group/RViz our combined SRDF
(srdf/common/ur16e_dualtool.srdf.xacro = arm group + EOAT disable_collisions +
gripper group) instead of ur_moveit_config's arm-only SRDF, so the EOAT collision
boxes in /robot_description are planned around correctly and — per the collision-
discovery contract — EOAT-vs-arm-body pairs stay ENABLED so MoveIt rejects poses
that drive the tool into the arm.

Reuses ur_moveit_config's kinematics / planning pipelines / joint limits; only the
semantic description is overridden.

    ros2 launch ur_bringup ur16e_dualtool.launch.py
    ros2 launch ur_bringup ur16e_dualtool_moveit.launch.py
"""
from pathlib import Path

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")

    pkg = FindPackageShare("ur_bringup")

    # our combined SRDF (arm + EOAT disables + gripper group), built from xacro
    robot_description_semantic = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([pkg, "srdf", "common", "ur16e_dualtool.srdf.xacro"]), " ",
            "name:=ur16e",
        ]),
        value_type=str,
    )
    semantic_param = {"robot_description_semantic": robot_description_semantic}

    # kinematics / planning pipelines / joint limits reused from ur_moveit_config.
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur16e"})
        .to_moveit_configs()
    )

    # move_group reads /robot_description from the control stack's RSP (carries the
    # EOAT collision boxes); we only override the semantic.
    wait_robot_description = Node(
        package="ur_robot_driver",
        executable="wait_for_robot_description",
        output="screen",
    )

    move_group_node = Node(
        package="moveit_ros_move_group",
        executable="move_group",
        output="screen",
        parameters=[
            moveit_config.to_dict(),
            semantic_param,
            {"use_sim_time": use_sim_time, "publish_robot_description_semantic": True},
        ],
    )

    rviz_config = PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_moveit",
        output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config],
        parameters=[
            moveit_config.robot_description_kinematics,
            moveit_config.planning_pipelines,
            moveit_config.joint_limits,
            semantic_param,
            {"use_sim_time": use_sim_time},
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        wait_robot_description,
        RegisterEventHandler(
            OnProcessExit(target_action=wait_robot_description, on_exit=[move_group_node, rviz_node]),
        ),
    ])
