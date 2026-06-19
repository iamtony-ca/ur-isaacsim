"""REAL UR16e + Robotiq 2F-85 — one command for the wrist-mounted gripper.

This is the real-hardware counterpart of the sim set's ur16e_2f85.launch.py.
The arm-only ur16e.launch.py is left UNTOUCHED; this new file composes:

  1. the official ur_robot_driver (ur_control.launch.py) for the arm (RTDE), with
     use_tool_communication:=true so the driver bridges the UR wrist tool RS-485
     to a virtual serial device on this PC (tool_device_name, default /tmp/ttyUR);
  2. the gripper stack (robotiq_2f85_real.launch.py, namespace `gripper`) with
     com_port pointed at that same virtual device.

Why this exists: on a real UR the 2F-85 is mounted on the wrist and wired to the
UR *tool connector* (24V + RS-485 Modbus RTU) — it is NOT plugged into this PC.
The only way the PC reaches it is the ur_robot_driver tool-communication bridge:
the UR exposes the tool serial over TCP (tool_tcp_port, 54321) and the driver's
ur_tool_comm node mirrors it to /tmp/ttyUR, which robotiq_driver then opens.
(The direct USB-RS485 topology, com_port:=/dev/ttyUSB0, is only for a gripper
bench-wired straight to the PC — use robotiq_2f85_real.launch.py standalone.)

Usage (real):
    ros2 launch ur_bringup ur16e_2f85_real.launch.py robot_ip:=<UR16e_IP>
    ros2 launch ur_bringup ur16e_2f85_moveit.launch.py use_sim:=false   # optional MoveIt

Dry-run with no hardware at all (arm mock + gripper mock, no tool bridge):
    ros2 launch ur_bringup ur16e_2f85_real.launch.py \
        use_mock_hardware:=true use_fake_hardware:=true use_tool_communication:=false

Prereqs on the real robot: e-Series tool I/O set to RS-485 / Robotiq, the PC user
allowed to write tool_device_name (e.g. dialout group), and tool_voltage:=24.
"""
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription,
                            TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    ur_driver = FindPackageShare("ur_robot_driver")
    pkg = FindPackageShare("ur_bringup")

    robot_ip = LaunchConfiguration("robot_ip")
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    headless_mode = LaunchConfiguration("headless_mode")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    tool_voltage = LaunchConfiguration("tool_voltage")
    tool_device_name = LaunchConfiguration("tool_device_name")
    tool_tcp_port = LaunchConfiguration("tool_tcp_port")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    com_port = LaunchConfiguration("com_port")
    gripper_startup_delay = LaunchConfiguration("gripper_startup_delay")

    declared_args = [
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.102",
                              description="IP of the real UR16e."),
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="Arm: run ur_robot_driver with mock hardware (no robot)."),
        DeclareLaunchArgument("headless_mode", default_value="false",
                              description="Arm: UR headless External Control (no PolyScope program)."),
        DeclareLaunchArgument("launch_rviz", default_value="false"),
        # --- tool communication bridge (UR wrist RS-485 -> /tmp/ttyUR) ---
        DeclareLaunchArgument("use_tool_communication", default_value="true",
                              description="Bridge the UR tool RS-485 to a virtual serial device so "
                                          "this PC can drive the wrist-mounted gripper. Set false "
                                          "for a gripper bench-wired to the PC (com_port:=/dev/ttyUSB0)."),
        DeclareLaunchArgument("tool_voltage", default_value="24",
                              description="Tool I/O voltage. 2F-85 needs 24 V."),
        DeclareLaunchArgument("tool_device_name", default_value="/tmp/ttyUR",
                              description="Virtual serial device the driver creates for tool comm; "
                                          "the gripper's com_port should match it."),
        DeclareLaunchArgument("tool_tcp_port", default_value="54321"),
        # 2F-85 serial is 115200 8N1 (parity 0, stop bits 1) — ur_control defaults already match.
        # --- gripper ---
        DeclareLaunchArgument("use_fake_hardware", default_value="false",
                              description="Gripper: mock_components instead of robotiq_driver (no serial)."),
        DeclareLaunchArgument("com_port", default_value="/tmp/ttyUR",
                              description="Serial port robotiq_driver opens. Defaults to the tool-comm "
                                          "bridge device; use /dev/ttyUSB0 for a direct USB-RS485 link."),
        DeclareLaunchArgument("gripper_startup_delay", default_value="8.0",
                              description="Seconds to wait before starting the gripper, so the tool-comm "
                                          "bridge (/tmp/ttyUR) exists before robotiq_driver opens it."),
    ]

    # Arm via the official driver, with the tool-communication bridge enabled.
    arm = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([ur_driver, "launch", "ur_control.launch.py"])
        ),
        launch_arguments={
            "ur_type": "ur16e",
            "robot_ip": robot_ip,
            "use_mock_hardware": use_mock_hardware,
            "headless_mode": headless_mode,
            "initial_joint_controller": "scaled_joint_trajectory_controller",
            "launch_rviz": launch_rviz,
            "use_tool_communication": use_tool_communication,
            "tool_voltage": tool_voltage,
            "tool_device_name": tool_device_name,
            "tool_tcp_port": tool_tcp_port,
        }.items(),
    )

    # Gripper stack (namespace `gripper`, own controller_manager). Delayed so the
    # tool-comm bridge device is present before robotiq_driver tries to open it.
    gripper = TimerAction(
        period=gripper_startup_delay,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([pkg, "launch", "robotiq_2f85_real.launch.py"])
                ),
                launch_arguments={
                    "use_fake_hardware": use_fake_hardware,
                    "com_port": com_port,
                }.items(),
            ),
        ],
    )

    return LaunchDescription(declared_args + [arm, gripper])
