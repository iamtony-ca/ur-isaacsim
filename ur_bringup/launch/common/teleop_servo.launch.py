"""MoveIt Servo teleoperation for UR16e (+ 2F-85) — sim or real.

Drives the arm from EE twist / joint-jog commands (gamepad, keyboard) instead of
planned trajectories. This is step 1 of the teleop -> IL pick&place pipeline:
first make the arm hand-drivable, then record demos, then train.

    ur_bringup/docs/plan_il_vla.md   IL/VLA plan (2.3 = why streaming control)

Pipeline
--------
    gamepad/keyboard -> TwistStamped -> servo_node -> /forward_position_controller/commands
                                                   -> ros2_control -> Isaac / real UR16e

Bring-up (on top of the Set 2/3 control stack, which must already be running):

    # 1) Isaac + control stack (example: Set 3)
    /isaac-sim/python.sh .../isaac/common/ur16e_isaac_ros2.py \
        --asset-path .../assets/ur16e_2f85_d405.usd --with-camera
    ros2 launch ur_bringup ur16e_2f85_d405.launch.py use_sim:=true

    # 2) this launch — spawns forward_position_controller (INACTIVE) + servo_node
    ros2 launch ur_bringup teleop_servo.launch.py use_sim_time:=true

    # 3) hand the arm over to streaming control
    python3 .../isaac/common/switch_control_mode.py streaming

    # 4a) keyboard teleop (no gamepad needed — good for first verification)
    ros2 run moveit_servo servo_keyboard_input
    # 4b) gamepad teleop
    ros2 launch ur_bringup teleop_dualsense.launch.py

    # back to MoveIt/cuMotion planning:
    python3 .../isaac/common/switch_control_mode.py trajectory

sim vs real
-----------
This launch is IDENTICAL for both; only `use_sim_time` changes.

    sim :  ros2 launch ur_bringup teleop_servo.launch.py use_sim_time:=true
    real:  ros2 launch ur_bringup teleop_servo.launch.py use_sim_time:=false

Servo does not touch hardware -- it consumes /joint_states and publishes joint
targets to forward_position_controller. Which backend executes those targets
(topic_based -> Isaac, or ur_robot_driver -> RTDE) is decided by the control
launch underneath, exactly like MoveIt/cuMotion. So the teleop layer is backend
agnostic by construction.

One asymmetry worth knowing: on REAL, `ur_control.launch.py` publishes an
ARM-ONLY /robot_description (the 2F-85 lives in its own `gripper` namespace with
a separate description). Servo here does NOT read /robot_description -- it builds
its own model from `description_file`, so it keeps the arm+gripper collision
model on real too, which is what we want (the gripper is a real self-collision
hazard while hand-driving). The <ros2_control> block in that xacro is ignored by
the robot model, so the sim-named file is correct for real as well.

The streaming controller must exist on whichever stack is running:
`forward_position_controller` is defined in config/common/ur16e_2f85_controllers.yaml
(sim). For real, add the same block to the controller yaml the real launch feeds
ur_control.launch.py, or spawn it with `-p` against that controller_manager.

Notes
-----
* forward_position_controller is spawned INACTIVE on purpose: it shares command
  interfaces with scaled_joint_trajectory_controller, so activating it here would
  fight the trajectory controller the base launch just started.
* Servo needs the robot model itself (URDF + SRDF + kinematics) because it solves
  IK and checks collisions. We reuse the SAME combined gripper SRDF as the MoveIt
  launches so self-collision behaviour matches.
* `primary_scene_monitor` must be FALSE if move_group is running at the same time
  (only one node may own the planning scene).
"""
import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                  PathJoinSubstitution)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    primary_scene_monitor = LaunchConfiguration("primary_scene_monitor")
    servo_params_file = LaunchConfiguration("servo_params_file")
    description_file = LaunchConfiguration("description_file")

    pkg = FindPackageShare("ur_bringup")
    share = get_package_share_directory("ur_bringup")

    declared = [
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="true for Isaac (/clock), false for the real robot"),
        DeclareLaunchArgument(
            "description_file",
            default_value=PathJoinSubstitution(
                [pkg, "urdf", "ur16e_2f85_d405", "ur16e_2f85_d405_sim.urdf.xacro"]),
            description="Robot xacro Servo builds its kinematic/collision model from. "
                        "Set 2: urdf/ur16e_2f85/ur16e_2f85_sim.urdf.xacro. "
                        "SIM AND REAL BOTH USE THESE FILES -- see the sim/real note in the "
                        "module docstring: Servo only needs kinematics + collision geometry, "
                        "and the <ros2_control> block inside is ignored by the robot model."),
        DeclareLaunchArgument(
            "servo_params_file",
            default_value=os.path.join(share, "config", "common", "ur16e_servo.yaml")),
        DeclareLaunchArgument(
            "primary_scene_monitor", default_value="true",
            description="false when move_group is also running (it owns the planning scene)"),
    ]

    # --- robot model for Servo ------------------------------------------------
    robot_description = ParameterValue(
        Command([FindExecutable(name="xacro"), " ", description_file]), value_type=str)

    # Same combined arm+gripper SRDF the MoveIt launches use, so Servo's
    # self-collision checks match what the planner would allow.
    robot_description_semantic = ParameterValue(
        Command([
            FindExecutable(name="xacro"), " ",
            PathJoinSubstitution([pkg, "srdf", "common", "ur16e_2f85.srdf.xacro"]), " ",
            "name:=ur16e",
        ]),
        value_type=str,
    )

    # kinematics + joint limits come from the official ur_moveit_config.
    moveit_config = (
        MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
        .robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur16e"})
        .to_moveit_configs()
    )

    servo_node = Node(
        package="moveit_servo",
        executable="servo_node",
        name="servo_node",
        output="screen",
        parameters=[
            servo_params_file,
            {"robot_description": robot_description},
            {"robot_description_semantic": robot_description_semantic},
            moveit_config.robot_description_kinematics,
            moveit_config.joint_limits,
            {"use_sim_time": use_sim_time,
             "is_primary_planning_scene_monitor": primary_scene_monitor},
        ],
    )

    # Loaded but NOT started — see module docstring.
    spawn_streaming_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager", "--inactive"],
        output="screen",
    )

    return LaunchDescription(declared + [spawn_streaming_controller, servo_node])
