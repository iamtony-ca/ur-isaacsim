"""MoveIt2 for UR16e + Robotiq 2F-85 (collision-aware gripper).

Like ur16e_moveit.launch.py, but feeds move_group/RViz our combined SRDF
(srdf/ur16e_2f85.srdf.xacro = arm group + gripper disable_collisions) instead of
ur_moveit_config's arm-only SRDF, so the gripper collision meshes in
/robot_description are planned around correctly (no false START_STATE_IN_COLLISION).

Reuses ur_moveit_config's kinematics / planning pipelines / joint limits via
MoveItConfigsBuilder; only the semantic description is overridden.

Use this (NOT the shared ur16e_moveit.launch.py) with the gripper control stack:
    ros2 launch ur_bringup ur16e_2f85.launch.py
    ros2 launch ur_bringup ur16e_2f85_moveit.launch.py

(For the camera rig with depth->OctoMap collision avoidance, use the separate
 set: ur16e_2f85_d405.launch.py + ur16e_2f85_d405_moveit.launch.py.)
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

    # our combined SRDF (arm + gripper disables), built from xacro
    robot_description_semantic = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([pkg, "srdf", "ur16e_2f85.srdf.xacro"]), " ",
            "name:=ur16e",
        ]),
        value_type=str,
    )
    semantic_param = {"robot_description_semantic": robot_description_semantic}

    # kinematics / planning pipelines / joint limits reused from ur_moveit_config.
    # The builder needs a valid SRDF to initialise (use the stock arm one); the
    # combined gripper SRDF is applied via semantic_param override below.
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur16e"})
        .to_moveit_configs()
    )

    # move_group reads /robot_description from the topic (our control stack's RSP,
    # which carries the gripper collision meshes); we only override the semantic.
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
            semantic_param,  # overrides the arm-only semantic from the builder
            {"use_sim_time": use_sim_time, "publish_robot_description_semantic": True},
        ],
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
            OnProcessExit(
                target_action=wait_robot_description,
                on_exit=[move_group_node, rviz_node],
            )
        ),
    ])
