"""UR16e + Robotiq 2F-85 bringup for the Isaac Sim backend.

Standalone launch for the gripper variant; the arm-only ur16e.launch.py is left
untouched. Starts robot_state_publisher + ros2_control_node (topic_based
hardware) + spawners for joint_state_broadcaster,
scaled_joint_trajectory_controller (6 arm joints) and gripper_controller
(finger_joint GripperCommand action).

Isaac Sim must be running the composed UR16e+2F-85 scene, e.g.:
    /isaac-sim/python.sh \
        /isaac-sim/ur_ws/src/ur_bringup/isaac/ur16e_isaac_ros2.py \
        --asset-path /isaac-sim/ur_ws/src/ur_bringup/isaac/assets/ur16e_with_2f85.usd

Then (this launch):
    ros2 launch ur_bringup ur16e_2f85.launch.py

The arm still exposes the same follow_joint_trajectory action, so
ur16e_moveit.launch.py works unchanged; the gripper adds a GripperCommand
action at /gripper_controller/gripper_cmd.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("ur_bringup")

    use_sim_time = LaunchConfiguration("use_sim_time")
    launch_rviz = LaunchConfiguration("launch_rviz")
    joint_commands_topic = LaunchConfiguration("joint_commands_topic")
    joint_states_topic = LaunchConfiguration("joint_states_topic")

    declared_args = [
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="Sim nodes use /clock from Isaac. Set false to test the "
                                          "controller stack without Isaac running."),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        DeclareLaunchArgument("joint_commands_topic", default_value="/isaac_joint_commands",
                              description="Topic Isaac Sim subscribes to for position commands"),
        DeclareLaunchArgument("joint_states_topic", default_value="/isaac_joint_states",
                              description="Topic Isaac Sim publishes joint states on"),
    ]

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([pkg, "urdf", "ur16e_2f85_sim.urdf.xacro"]), " ",
            "joint_commands_topic:=", joint_commands_topic, " ",
            "joint_states_topic:=", joint_states_topic,
        ]),
        value_type=str,
    )
    sim_time = ParameterValue(use_sim_time, value_type=bool)
    controllers_yaml = PathJoinSubstitution([pkg, "config", "ur16e_2f85_controllers.yaml"])

    return LaunchDescription(declared_args + [
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": sim_time}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[{"robot_description": robot_description, "use_sim_time": sim_time}, controllers_yaml],
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
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_controller", "-c", "/controller_manager"],
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            condition=IfCondition(launch_rviz),
            arguments=[],
        ),
    ])
