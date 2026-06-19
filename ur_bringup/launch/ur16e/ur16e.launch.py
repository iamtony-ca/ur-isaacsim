"""UR16e bringup with a single use_sim switch.

  use_sim:=true   -> Isaac Sim backend. Starts robot_state_publisher +
                     ros2_control_node (topic_based hardware) + controller
                     spawners. Isaac Sim must be running its ROS2-bridge
                     OmniGraph, publishing joint states and subscribing to
                     joint commands on the matching topics.

  use_sim:=false  -> Real UR16e. Delegates to the official
                     ur_robot_driver/ur_control.launch.py (RTDE), so the real
                     path stays fully maintained upstream.

In both cases the robot exposes the same joint_trajectory_controller
follow_joint_trajectory action, so MoveIt2 (ur16e_moveit.launch.py) and any
higher-level app are identical across sim and real.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("ur_bringup")

    use_sim = LaunchConfiguration("use_sim")
    use_sim_time = LaunchConfiguration("use_sim_time")
    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_rviz = LaunchConfiguration("launch_rviz")
    joint_commands_topic = LaunchConfiguration("joint_commands_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    declared_args = [
        DeclareLaunchArgument("use_sim", default_value="true",
                              description="true: Isaac Sim backend; false: real UR16e via ur_robot_driver"),
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Sim nodes use /clock from Isaac. Set false to test the "
                                          "controller stack without Isaac running."),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.102",
                              description="IP of the real UR16e (only used when use_sim:=false)"),
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="Real path only: run ur_robot_driver with mock hardware "
                                          "(no physical robot) to dry-run the pipeline."),
        DeclareLaunchArgument("headless_mode", default_value="false",
                              description="Real path only: use the UR headless External Control "
                                          "(no PolyScope program needed; robot must allow remote control)."),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("joint_commands_topic", default_value="/isaac_joint_commands",
                              description="Topic Isaac Sim subscribes to for position commands"),
        DeclareLaunchArgument("joint_states_topic", default_value="/isaac_joint_states",
                              description="Topic Isaac Sim publishes joint states on"),
    ]

    # ---------------- SIM (Isaac) backend ----------------
    sim_description = Command([
        FindExecutable(name="xacro"), " ",
        PathJoinSubstitution([pkg, "urdf", "ur16e", "ur16e_sim.urdf.xacro"]), " ",
        "joint_commands_topic:=", joint_commands_topic, " ",
        "joint_states_topic:=", joint_states_topic,
    ])
    sim_robot_description = ParameterValue(sim_description, value_type=str)
    sim_time = ParameterValue(use_sim_time, value_type=bool)

    controllers_yaml = PathJoinSubstitution([pkg, "config", "ur16e", "ur16e_controllers.yaml"])

    sim_group = GroupAction(
        condition=IfCondition(use_sim),
        actions=[
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                output="screen",
                parameters=[{"robot_description": sim_robot_description, "use_sim_time": sim_time}],
            ),
            Node(
                package="controller_manager",
                executable="ros2_control_node",
                output="screen",
                parameters=[{"robot_description": sim_robot_description, "use_sim_time": sim_time}, controllers_yaml],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["joint_state_broadcaster", "-c", "/controller_manager"],
            ),
            Node(
                package="controller_manager",
                executable="spawner",
                arguments=["scaled_joint_trajectory_controller", "-c", "/controller_manager"],
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                condition=IfCondition(launch_rviz),
                arguments=[],
            ),
        ],
    )

    # ---------------- REAL backend (official UR driver) ----------------
    real_group = GroupAction(
        condition=UnlessCondition(use_sim),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(get_package_share_directory("ur_robot_driver"),
                                 "launch", "ur_control.launch.py")
                ),
                launch_arguments={
                    "ur_type": "ur16e",
                    "robot_ip": robot_ip,
                    "use_mock_hardware": use_mock_hardware,
                    "headless_mode": headless_mode,
                    "initial_joint_controller": "scaled_joint_trajectory_controller",
                    "launch_rviz": launch_rviz,
                }.items(),
            ),
        ],
    )

    return LaunchDescription(declared_args + [sim_group, real_group])
