"""REAL UR16e + 2F-85 + D405 — the full eye-in-hand rig in one command.

Real-hardware counterpart of the sim set's ur16e_2f85_d405.launch.py. Composes the
existing real launches, each left UNTOUCHED (Set 3 reuses Set 2, as in sim):
  1. ur16e_2f85_real.launch.py  (Set 2) — arm (ur_robot_driver / RTDE) + wrist
     2F-85 gripper (robotiq_driver via the UR tool-comm bridge /tmp/ttyUR).
  2. d405_real.launch.py        (Set 3) — D405 on USB3 (sim-parity topics) + camera
     TF (tool0 -> camera_link -> *_optical_frame).

The camera is a USB3 device on this PC, independent of the arm/gripper buses, so it
simply runs alongside. After bringup, add MoveIt + depth->OctoMap exactly like sim
(same topics), only with real time:
    ros2 launch ur_bringup ur16e_2f85_d405_moveit.launch.py use_sim:=false

Usage (real):
    ros2 launch ur_bringup ur16e_2f85_d405_real.launch.py robot_ip:=<UR16e_IP>

Dry-run, NO hardware (arm+gripper mock, camera TF only, no device):
    ros2 launch ur_bringup ur16e_2f85_d405_real.launch.py \
        use_mock_hardware:=true use_fake_hardware:=true \
        use_tool_communication:=false enable_camera:=false

Hand-eye: pass the calibrated tool0->camera via cam_xyz / cam_rpy (see README §9).
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    pkg = FindPackageShare("ur_bringup")

    robot_ip = LaunchConfiguration("robot_ip")
    # arm + gripper passthrough (-> ur16e_2f85_real.launch.py)
    use_mock_hardware = LaunchConfiguration("use_mock_hardware")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")
    headless_mode = LaunchConfiguration("headless_mode")
    use_tool_communication = LaunchConfiguration("use_tool_communication")
    com_port = LaunchConfiguration("com_port")
    # camera passthrough (-> d405_real.launch.py)
    serial_no = LaunchConfiguration("serial_no")
    cam_xyz = LaunchConfiguration("cam_xyz")
    cam_rpy = LaunchConfiguration("cam_rpy")
    enable_camera = LaunchConfiguration("enable_camera")

    declared_args = [
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.102",
                              description="IP of the real UR16e."),
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="Arm: ur_robot_driver mock hardware (no robot)."),
        DeclareLaunchArgument("use_fake_hardware", default_value="false",
                              description="Gripper: mock_components instead of robotiq_driver."),
        DeclareLaunchArgument("headless_mode", default_value="false"),
        DeclareLaunchArgument("use_tool_communication", default_value="true",
                              description="Bridge the UR tool RS-485 to /tmp/ttyUR for the wrist gripper."),
        DeclareLaunchArgument("com_port", default_value="/tmp/ttyUR",
                              description="Serial port robotiq_driver opens (tool-comm bridge by default)."),
        DeclareLaunchArgument("serial_no", default_value="",
                              description="D405 serial; empty = first RealSense found."),
        DeclareLaunchArgument("cam_xyz", default_value="0 -0.067 0.01847",
                              description="tool0 -> camera_link translation (HAND-EYE result)."),
        DeclareLaunchArgument("cam_rpy", default_value="0 -1.4311700 1.5707963",
                              description="tool0 -> camera_link rotation rpy (HAND-EYE result)."),
        DeclareLaunchArgument("enable_camera", default_value="true",
                              description="false = camera TF only (no realsense2_camera node)."),
    ]

    # Arm + wrist gripper (Set 2 real launch, reused unchanged).
    arm_gripper = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "ur16e_2f85", "ur16e_2f85_real.launch.py"])
        ),
        launch_arguments={
            "robot_ip": robot_ip,
            "use_mock_hardware": use_mock_hardware,
            "use_fake_hardware": use_fake_hardware,
            "headless_mode": headless_mode,
            "use_tool_communication": use_tool_communication,
            "com_port": com_port,
        }.items(),
    )

    # Eye-in-hand D405 (USB3) + camera TF.
    camera = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([pkg, "launch", "ur16e_2f85_d405", "d405_real.launch.py"])
        ),
        launch_arguments={
            "serial_no": serial_no,
            "cam_xyz": cam_xyz,
            "cam_rpy": cam_rpy,
            "enable_camera": enable_camera,
        }.items(),
    )

    return LaunchDescription(declared_args + [arm_gripper, camera])
