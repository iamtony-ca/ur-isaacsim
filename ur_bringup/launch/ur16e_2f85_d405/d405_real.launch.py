"""Standalone bringup for the REAL Intel RealSense D405 (eye-in-hand camera).

Real-hardware counterpart of the sim camera (ur16e_isaac_ros2.py --with-camera).
Unlike the gripper (which is on the UR tool RS-485 bus), the D405 is a USB3 device
wired straight to THIS PC, so it is independent of the arm/gripper stacks and just
runs alongside them.

Brings up two things:
  1. realsense2_camera node, publishing the SAME topics as the sim camera
     (/camera/color/image_raw, /camera/depth/image_rect_raw,
      /camera/depth/color/points, /camera/{color,depth}/camera_info) so the
     perception stack (OctoMap, cuMotion, FoundationPose) needs NO changes.
  2. a camera-only robot_state_publisher (d405_real.urdf.xacro, root tool0) that
     puts tool0 -> camera_link -> *_optical_frame on the TF tree. The driver's own
     TF is disabled (publish_tf:=false in the yaml) so this URDF — sim frames +
     the hand-eye extrinsic — is the single source of camera TF.

Dependencies (apt): ros-jazzy-realsense2-camera ros-jazzy-librealsense2

Usage (real, camera alone):
    ros2 launch ur_bringup d405_real.launch.py
TF only, NO device attached (validate the URDF/TF wiring):
    ros2 launch ur_bringup d405_real.launch.py enable_camera:=false
Hand-eye: pass the calibrated tool0->camera as cam_xyz / cam_rpy (see README §9).

NOTE on topic names: this puts the node in namespace `camera` to yield
/camera/<topic>. Some realsense2_camera versions instead nest as
/camera/camera/<topic> (camera_name under camera_namespace). If you see that, set
camera_name:='' on the node or add remappings so the topics match the sim names.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                   PathJoinSubstitution)
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare

CAMERA_NS = "camera"


def generate_launch_description():
    pkg = FindPackageShare("ur_bringup")

    serial_no = LaunchConfiguration("serial_no")
    cam_xyz = LaunchConfiguration("cam_xyz")
    cam_rpy = LaunchConfiguration("cam_rpy")
    prefix = LaunchConfiguration("prefix")
    enable_camera = LaunchConfiguration("enable_camera")
    use_sim_time = LaunchConfiguration("use_sim_time")

    declared_args = [
        DeclareLaunchArgument("serial_no", default_value="",
                              description="D405 serial (e.g. _128422270893). Empty = first device "
                                          "found; set it when more than one RealSense is connected."),
        DeclareLaunchArgument("cam_xyz", default_value="0 -0.067 0.01847",
                              description="tool0 -> camera_link translation = HAND-EYE result. "
                                          "Default = sim nominal bracket mount."),
        DeclareLaunchArgument("cam_rpy", default_value="0 -1.4311700 1.5707963",
                              description="tool0 -> camera_link rotation rpy = HAND-EYE result. "
                                          "Default = 0, -pi/2+8deg, pi/2 (sim nominal)."),
        DeclareLaunchArgument("prefix", default_value=""),
        DeclareLaunchArgument("enable_camera", default_value="true",
                              description="false = publish camera TF only (skip the realsense2_camera "
                                          "node), to validate TF/URDF with no device attached."),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
    ]

    # Camera TF (tool0 -> camera_link -> optical frames): sim parity + hand-eye.
    # cam_xyz/cam_rpy hold spaces, so wrap each in single quotes -> one xacro arg.
    camera_description = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([pkg, "urdf", "ur16e_2f85_d405", "d405_real.urdf.xacro"]), " ",
            "prefix:=", prefix, " ",
            "cam_xyz:='", cam_xyz, "' ",
            "cam_rpy:='", cam_rpy, "'",
        ]),
        value_type=str,
    )
    realsense_yaml = PathJoinSubstitution([pkg, "config", "ur16e_2f85_d405", "d405_real.yaml"])

    camera_group = GroupAction([
        PushRosNamespace(CAMERA_NS),
        # Camera-only robot_state_publisher: owns tool0 -> camera_* TF.
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="camera_state_publisher",
            output="screen",
            parameters=[{"robot_description": camera_description,
                         "use_sim_time": use_sim_time}],
        ),
        # realsense2_camera driver (USB3 D405). publish_tf:=false comes from the yaml.
        Node(
            package="realsense2_camera",
            executable="realsense2_camera_node",
            name="camera",
            output="screen",
            parameters=[realsense_yaml,
                        {"serial_no": serial_no, "use_sim_time": use_sim_time}],
            condition=IfCondition(enable_camera),
        ),
    ])

    return LaunchDescription(declared_args + [camera_group])
