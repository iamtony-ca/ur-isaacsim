"""Standalone bringup for the REAL Robotiq 2F-85 gripper (gripper stack only).

The arm-only / sim files are untouched; this is the real-hardware counterpart of
ur16e_2f85.launch.py (which is Isaac/topic_based). It brings up ONLY the gripper
(namespace `gripper`, own /gripper/controller_manager).

There are two real wiring topologies; pick com_port accordingly:

  A) Gripper MOUNTED ON THE UR WRIST (the normal case): the 2F-85 is wired to the
     UR tool connector (24V + RS-485), NOT to this PC. The PC reaches it only via
     ur_robot_driver's tool-communication bridge (UR tool serial -> /tmp/ttyUR).
     => use ur16e_2f85_real.launch.py instead — it starts the arm WITH that bridge
        and this gripper stack (com_port:=/tmp/ttyUR) in one shot.

  B) Gripper BENCH-WIRED to this PC via a USB-RS485 adapter => run this launch
     standalone with com_port:=/dev/ttyUSB0, alongside the arm:
        ros2 launch ur_bringup ur16e.launch.py use_sim:=false robot_ip:=<UR16e_IP>
        ros2 launch ur_bringup robotiq_2f85_real.launch.py com_port:=/dev/ttyUSB0

Everything lives in the `gripper` namespace so its controller_manager
(/gripper/controller_manager), joint_state_broadcaster and robot_state_publisher
do not collide with the arm's (which own /controller_manager and the world->tool0
TF). The gripper's TF subtree hangs off tool0, joining the arm tree cleanly.

Validate with no hardware attached (mirrors the arm's use_mock_hardware):
    ros2 launch ur_bringup robotiq_2f85_real.launch.py use_fake_hardware:=true

Control it with the same demo, pointed at the namespaced action:
    python3 .../isaac/gripper_demo.py \
        --action /gripper/gripper_controller/gripper_cmd \
        --joint-states-topic /gripper/joint_states
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                   PathJoinSubstitution)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

GRIPPER_NS = "gripper"
CM = "/gripper/controller_manager"


def generate_launch_description():
    pkg = FindPackageShare("ur_bringup")

    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    global_joint_states = LaunchConfiguration("global_joint_states")
    mock_sensor_commands = LaunchConfiguration("mock_sensor_commands")
    com_port = LaunchConfiguration("com_port")
    prefix = LaunchConfiguration("prefix")
    launch_rviz = LaunchConfiguration("launch_rviz")

    declared_args = [
        DeclareLaunchArgument(
            "global_joint_states", default_value="true",
            description="Publish the gripper joint state into the GLOBAL /joint_states so real "
                        "matches sim (7 joints on one topic). false = /gripper/joint_states only."),
        DeclareLaunchArgument("use_fake_hardware", default_value="false",
                              description="true = mock_components/GenericSystem (no serial/hardware), "
                                          "to validate the stack with nothing plugged in."),
        DeclareLaunchArgument("mock_sensor_commands", default_value="false"),
        DeclareLaunchArgument("com_port", default_value="/dev/ttyUSB0",
                              description="Serial port robotiq_driver opens. /dev/ttyUSB0 = direct "
                                          "USB-RS485 (bench). For a wrist-mounted gripper use "
                                          "ur16e_2f85_real.launch.py (com_port:=/tmp/ttyUR bridge)."),
        DeclareLaunchArgument("prefix", default_value="",
                              description="TF/joint prefix. Keep empty: robotiq_activation_controller "
                                          "expects unprefixed reactivate_gripper interfaces."),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
    ]

    robot_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([pkg, "urdf", "ur16e_2f85", "robotiq_2f85_real.urdf.xacro"]), " ",
            "use_fake_hardware:=", use_fake_hardware, " ",
            "mock_sensor_commands:=", mock_sensor_commands, " ",
            "com_port:=", com_port, " ",
            "prefix:=", prefix,
        ]),
        value_type=str,
    )
    controllers_yaml = PathJoinSubstitution([pkg, "config", "ur16e_2f85", "robotiq_2f85_real_controllers.yaml"])

    gripper_group = GroupAction([
        PushRosNamespace(GRIPPER_NS),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            output="screen",
            parameters=[{"robot_description": robot_description}],
        ),
        Node(
            package="controller_manager",
            executable="ros2_control_node",
            output="screen",
            parameters=[{"robot_description": robot_description}, controllers_yaml],
        ),
        # ** sim/real parity for /joint_states **
        # In sim ONE controller_manager owns arm + finger_joint, so /joint_states
        # carries all 7 joints. On real the gripper has its own CM in the
        # `gripper` namespace, so by default it would only publish
        # /gripper/joint_states and the global /joint_states would be arm-only.
        # Anything that consumes the WHOLE robot state then stalls on real while
        # working in sim -- MoveIt Servo sits at "Waiting to receive robot state
        # update" forever, and an IL recorder would silently lose the gripper.
        # Publishing into the global topic (multiple publishers on /joint_states
        # is the standard ROS pattern; robot_state_publisher and MoveIt's
        # CurrentStateMonitor merge them) makes real look like sim.
        # Set global_joint_states:=false to keep the old namespaced-only behaviour.
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", CM,
                       "--controller-ros-args", "-r joint_states:=/joint_states"],
            condition=IfCondition(global_joint_states),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["joint_state_broadcaster", "-c", CM],
            condition=UnlessCondition(global_joint_states),
        ),
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["gripper_controller", "-c", CM],
        ),
        # Real-only: (re)activates the physical gripper. The mock backend has no
        # gripper to activate, so skip it under use_fake_hardware.
        Node(
            package="controller_manager",
            executable="spawner",
            arguments=["robotiq_activation_controller", "-c", CM],
            condition=UnlessCondition(use_fake_hardware),
        ),
    ])

    rviz = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2_gripper",
        condition=IfCondition(launch_rviz),
        arguments=[],
    )

    return LaunchDescription(declared_args + [gripper_group, rviz])
