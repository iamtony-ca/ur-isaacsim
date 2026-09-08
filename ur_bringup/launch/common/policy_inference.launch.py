"""Run a trained LeRobot policy on the arm, using upstream's own async inference.

    # 1. control stack + Isaac (or the real robot) already running -- README.md 5
    # 2. this launch: loads the streaming controller
    ros2 launch ur_bringup policy_inference.launch.py use_sim:=true
    # 3. hand the arm over to the streaming controller
    python3 src/ur_bringup/isaac/common/switch_control_mode.py streaming
    # 4. upstream policy server (ML env, its own terminal)
    deps/.venv-ml/bin/python -m lerobot.async_inference.policy_server \
        --host=127.0.0.1 --port=8080
    # 5. upstream robot client (ML env), with OUR robot adapter
    deps/.venv-ml/bin/python -m lerobot.async_inference.robot_client \
        --robot.type=ur16e_ros --robot.id=sim \
        --policy_type=act --pretrained_name_or_path=outputs/act_.../pretrained_model \
        --actions_per_chunk=50 --task="put the red block on the left marker"

This launch deliberately does almost nothing: it spawns the streaming controller
and nothing else. The policy, the server and the client are all upstream LeRobot
code, and the only piece of ours in the loop is the robot plugin
ur_bringup/lerobot_robot_ur16e_ros (--robot.type=ur16e_ros), which lerobot
discovers by itself: register_third_party_plugins() imports installed
distributions named lerobot_robot_* -- no PYTHONPATH, no wrapper. Switching from ACT to pi0,
pi05, smolvla or groot is then a change of --policy_type, not new code.

Why the controller has to be spawned at all: forward_position_controller is
defined in config/common/ur16e_2f85_controllers.yaml but the control launches do
not spawn it -- only the teleop launches did, because until now nothing else
streamed per-step targets. Without it, switch_control_mode.py fails with
"controller 'forward_position_controller' is not loaded" and, if that is missed,
the policy publishes into a topic nobody serves and the arm simply never moves,
which looks exactly like a broken policy.

Spawned INACTIVE on purpose: it shares command interfaces with
scaled_joint_trajectory_controller (MoveIt / cuMotion), and controller_manager
refuses to activate both. switch_control_mode.py does the swap.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node


def generate_launch_description():
    args = [
        DeclareLaunchArgument(
            "use_sim", default_value="true",
            description="documentation only here; the backend is decided by the "
                        "control launch that is already running"),
    ]
    spawn_streaming_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager", "--inactive"],
        output="screen",
    )
    return LaunchDescription(args + [spawn_streaming_controller])
