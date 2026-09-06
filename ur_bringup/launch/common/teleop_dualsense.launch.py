"""Gamepad teleop: joy_node + teleop_joy -> MoveIt Servo.

Run AFTER teleop_servo.launch.py and after switching to streaming control:

    ros2 launch ur_bringup teleop_servo.launch.py use_sim_time:=true
    python3 .../isaac/common/switch_control_mode.py streaming
    ros2 launch ur_bringup teleop_dualsense.launch.py

*** Move the arm to a non-singular pose first, or nothing will happen: ***
    python3 .../isaac/common/switch_control_mode.py trajectory
    python3 .../isaac/common/reset_pose.py ready
    python3 .../isaac/common/switch_control_mode.py streaming
home/up/zero have elbow_joint = 0 (fully extended = elbow singularity) and Servo
refuses to move there ("Very close to a singularity, emergency stop").

No gamepad? Verify the whole chain with the keyboard instead:
    ros2 run moveit_servo servo_keyboard_input

Pad mapping defaults to a PS5 DualSense over USB. To adapt another pad, watch
`ros2 topic echo /joy` and override e.g.:
    ros2 launch ur_bringup teleop_dualsense.launch.py axis_lx:=0 button_deadman:=4
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# name -> default, forwarded straight to teleop_joy as ROS parameters
PARAMS = {
    "axis_lx": "0", "axis_ly": "1", "axis_rx": "3", "axis_ry": "4",
    "axis_l2": "2", "axis_r2": "5",
    "button_deadman": "4", "button_rotate_mode": "5", "button_reassert": "1",
    "button_ep_start": "3", "button_ep_stop": "2", "button_ep_discard": "0",
    "deadzone": "0.08", "linear_scale": "1.0", "angular_scale": "1.0",
    "publish_rate": "50.0", "twist_frame": "base_link",
    "trigger_idle_is_positive": "True",
    "gripper_open": "0.0", "gripper_closed": "0.8",
}


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    joy_device = LaunchConfiguration("joy_device")

    declared = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("joy_device", default_value="/dev/input/js0",
                              description="gamepad device (USB-C wired recommended: "
                                          "bluetooth jitter shows up directly in demo quality)"),
    ] + [DeclareLaunchArgument(k, default_value=v) for k, v in PARAMS.items()]

    joy_node = Node(
        package="joy", executable="joy_node", name="joy_node", output="screen",
        parameters=[{
            "device_name": joy_device,
            # Deadzone/repeat are handled in teleop_joy so behaviour is identical
            # whether joy_node or a replayed /joy bag drives it.
            "deadzone": 0.0,
            "autorepeat_rate": 20.0,
            "use_sim_time": use_sim_time,
        }],
    )

    teleop = Node(
        package="ur_bringup", executable="teleop_joy.py", name="teleop_joy", output="screen",
        parameters=[{k: LaunchConfiguration(k) for k in PARAMS},
                    {"use_sim_time": use_sim_time}],
    )

    return LaunchDescription(declared + [joy_node, teleop])
