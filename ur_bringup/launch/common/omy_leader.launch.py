"""OMY-L100 leader stack (ROBOTIS open_manipulator) for use NEXT TO a UR16e.

    ros2 launch ur_bringup omy_leader.launch.py port_name:=/dev/ttyUSB0
    ros2 launch ur_bringup omy_leader.launch.py use_mock_hardware:=true     # controllers only

Why a wrapper instead of `open_manipulator_bringup omy_l100_leader_ai.launch.py` directly
(found 2026-09-17 on the first real UR16e + L100 machine, HISTORY.md 49.6):

  ROBOTIS' gravity_compensation_controller subscribes to the ABSOLUTE topic
  `/joint_states` -- the OMY-F3M follower's -- to pull the leader back toward the
  follower when /collision_flag is set. Our follower is a UR16e, whose /joint_states
  carries shoulder_pan_joint... so with both stacks on one domain the controller logs

      Joint name 'joint1' not found in the first joint state message

  for EVERY UR joint-state message (500 Hz of ERROR lines). It is harmless -- that
  data is only read when the self-collision feature is on, and we launch it off --
  but it buries every real message in T1. This wrapper remaps that one absolute
  name to /leader/joint_states (the leader's own broadcaster), so the index mapping
  initialises and the log goes quiet. Nothing else changes: the ROBOTIS launch is
  included as-is, in the /leader namespace it sets up itself; relative names
  (`joint_states` -> /leader/joint_states) are not touched by an absolute rule.

Arguments are forwarded verbatim to the ROBOTIS launch.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    args = [
        DeclareLaunchArgument("port_name", default_value="/dev/ttyUSB0",
                              description="U2D2 serial port (a /dev/serial/by-id path is safer with two FTDIs)."),
        DeclareLaunchArgument("use_mock_hardware", default_value="false",
                              description="ros2_control mock: controllers load, but a mocked L100 reports 0.0 forever "
                                          "(effort-commanded, position is state-only). Use virtual_leader for motion."),
        DeclareLaunchArgument("use_self_collision_avoidance", default_value="false",
                              description="ROBOTIS self-collision feature. Off: it assumes an OMY-F3M follower."),
    ]
    robotis = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(PathJoinSubstitution([
            FindPackageShare("open_manipulator_bringup"), "launch", "omy_l100_leader_ai.launch.py"])),
        launch_arguments={
            "port_name": LaunchConfiguration("port_name"),
            "use_mock_hardware": LaunchConfiguration("use_mock_hardware"),
            "use_self_collision_avoidance": LaunchConfiguration("use_self_collision_avoidance"),
        }.items(),
    )
    return LaunchDescription(args + [
        GroupAction([
            SetRemap(src="/joint_states", dst="/leader/joint_states"),
            robotis,
        ]),
    ])
