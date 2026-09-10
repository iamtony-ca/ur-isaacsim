"""OMY-L100 leader teleop for the UR16e -- joint-direct, no MoveIt Servo.

    /leader/joint_states -> omy_to_ur16e -> /forward_position_controller/commands
                                         -> ros2_control -> Isaac / real UR16e

This is the sibling of teleop_servo.launch.py, not a replacement:

    teleop_servo   gamepad/keyboard -> Cartesian twist -> Servo (IK) -> streaming ctrl
    teleop_omy     leader arm       -> joint-direct map ------------> streaming ctrl

Servo is deliberately NOT started here. The leader is a 6R arm with the same axis
sequence as the UR16e (HISTORY.md 21), so there is nothing to solve -- and skipping
IK removes the elbow-singularity failure mode that forces the Servo path to start
from `ready` (HISTORY.md 15). Joint-direct has no singularities.

Bring-up (Set 2/3 control stack must already be running)
--------------------------------------------------------
    # 1) Isaac + control stack
    /isaac-sim/python.sh .../isaac/common/ur16e_isaac_ros2.py \
        --asset-path .../assets/ur16e_2f85_d405.usd --with-camera
    ros2 launch ur_bringup ur16e_2f85_d405.launch.py use_sim:=true

    # 2) this launch
    #    SIM (no hardware):  virtual_leader:=true  -- fakes /leader/joint_states
    #    REAL:               virtual_leader:=false + run the ROBOTIS leader stack:
    #      ros2 launch open_manipulator_bringup omy_l100_leader_ai.launch.py \
    #          port_name:=/dev/ttyUSB0 use_self_collision_avoidance:=false
    ros2 launch ur_bringup teleop_omy.launch.py use_sim_time:=true virtual_leader:=true

    # 3) meet at the rendezvous pose, then engage.
    #    /omy_bridge/sync needs move_group (ur16e_moveit.launch.py): it plans a
    #    COLLISION-CHECKED path to `rendezvous` and leaves the arm in streaming mode.
    #    Put the leader down in its rest pose first -- see HISTORY.md 41.
    ros2 service call /omy_bridge/sync   std_srvs/srv/Trigger
    ros2 service call /omy_bridge/enable std_srvs/srv/Trigger

    #    Without move_group, do the same by hand (check the arm's surroundings first):
    #      switch_control_mode.py trajectory && reset_pose.py ready
    #      switch_control_mode.py streaming

    # how far off is the leader?  (rad, per joint, mapped leader minus follower)
    ros2 topic echo /omy_bridge/engage_error

    # back to MoveIt/cuMotion:
    ros2 service call /omy_bridge/disable std_srvs/srv/Trigger
    python3 .../isaac/common/switch_control_mode.py trajectory

With `pad:=true` the same three actions are on the gamepad -- hold the deadman (L1)
and press Options=enable / R3=sync; Create=disable needs no deadman.

forward_position_controller is spawned INACTIVE here for the same reason as in
teleop_servo.launch.py: it shares command interfaces with
scaled_joint_trajectory_controller, so activating it at launch would fight the
trajectory controller the base launch just started. switch_control_mode.py does
the atomic swap.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    declared = [
        DeclareLaunchArgument(
            "use_sim_time", default_value="true",
            description="true with Isaac (/clock), false on real hardware."),
        DeclareLaunchArgument(
            "virtual_leader", default_value="false",
            description="Fake /leader/joint_states instead of reading an L100. "
                        "Set true for sim testing; the mocked ROBOTIS leader "
                        "stack cannot move (effort-commanded, position is "
                        "state-only), so this is the only hardware-free path."),
        DeclareLaunchArgument(
            "max_joint_speed", default_value="1.0",
            description="[rad/s] per-joint slew limit on the follower command."),
        DeclareLaunchArgument(
            "engage_tol", default_value="0.15",
            description="[rad] per-joint match required before /omy_bridge/enable "
                        "is accepted."),
        DeclareLaunchArgument(
            "leader_amplitude", default_value="0.25",
            description="virtual_leader only: [rad] peak deviation of the fake "
                        "leader motion. amplitude*2pi/period is its peak speed -- "
                        "keep it under max_joint_speed or you only exercise the "
                        "slew limiter."),
        DeclareLaunchArgument(
            "leader_period", default_value="8.0",
            description="virtual_leader only: [s] sine period."),
        DeclareLaunchArgument(
            "leader_joints", default_value="[0, 2]",
            description="virtual_leader only: leader joint indices to animate."),
        DeclareLaunchArgument(
            "limit_margin", default_value="0.95",
            description="Fraction of the UR16e joint limits the bridge clamps to. "
                        "The elbow needs this: UR16e and L100 both allow +-pi, so "
                        "the leader can drive it to the limit with zero margin."),
        DeclareLaunchArgument(
            "rendezvous", default_value="[0.0, -1.5707, 1.5707, -1.5707, -1.5707, 0.0]",
            description="UR16e joint target for /omy_bridge/sync. Default = "
                        "reset_pose.py `ready`, which is what the L100's own rest "
                        "pose maps to (HISTORY.md 41). Change it only together with "
                        "the episode reset pose -- teleop must start where the "
                        "recorded demonstrations start."),
        # Exposed as launch args because the bridge reads its parameters ONCE, in the
        # constructor: `ros2 param set /omy_to_ur16e offset ...` returns success and
        # does nothing (HISTORY.md 42.3-C). Calibration therefore re-launches with the
        # measured values -- omy_leader_calib.py --mode match prints this exact line.
        DeclareLaunchArgument(
            "offset", default_value="[0.0, -1.5707963267948966, 0.0, 0.0, 0.0, 0.0]",
            description="[rad] per-joint offset in q_ur[i] = sign[i]*q_leader[i] + offset[i]. "
                        "J2 = -pi/2 reconciles the two zero poses; J4/J6 have no derivable "
                        "value and come from omy_leader_calib.py on real hardware."),
        DeclareLaunchArgument(
            "sign", default_value="[1.0, 1.0, 1.0, 1.0, -1.0, 1.0]",
            description="Per-joint sign. J5 = -1 because the L100 joint5 spins about "
                        "+Z and the UR wrist_2_joint about -Z (measured, HISTORY.md 21). "
                        "Do not flip it again."),
        DeclareLaunchArgument(
            "pad", default_value="false",
            description="Start joy_node so the pad can call enable/disable/sync "
                        "without the operator letting go of the leader."),
        DeclareLaunchArgument(
            "joy_device", default_value="/dev/input/js0",
            description="pad only: gamepad device."),
    ]
    use_sim_time = LaunchConfiguration("use_sim_time")

    # Buttons are read by the bridge itself, not by teleop_joy: teleop_joy is the
    # Servo/Cartesian path and would publish twists nothing is listening to here.
    joy_node = Node(
        package="joy", executable="joy_node", name="joy_node", output="screen",
        condition=IfCondition(LaunchConfiguration("pad")),
        parameters=[{
            "device_name": LaunchConfiguration("joy_device"),
            "use_sim_time": use_sim_time,
        }],
    )

    spawn_streaming_controller = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["forward_position_controller", "-c", "/controller_manager", "--inactive"],
        output="screen",
    )

    bridge = Node(
        package="ur_bringup",
        executable="omy_to_ur16e.py",
        name="omy_to_ur16e",
        output="screen",
        parameters=[{
            "use_sim_time": use_sim_time,
            "max_joint_speed": LaunchConfiguration("max_joint_speed"),
            "engage_tol": LaunchConfiguration("engage_tol"),
            "limit_margin": LaunchConfiguration("limit_margin"),
            "rendezvous": ParameterValue(LaunchConfiguration("rendezvous"),
                                         value_type=None),
            "offset": ParameterValue(LaunchConfiguration("offset"), value_type=None),
            "sign": ParameterValue(LaunchConfiguration("sign"), value_type=None),
        }],
    )

    virtual_leader = Node(
        package="ur_bringup",
        executable="virtual_omy_leader.py",
        name="virtual_omy_leader",
        output="screen",
        condition=IfCondition(LaunchConfiguration("virtual_leader")),
        parameters=[{
            "use_sim_time": use_sim_time,
            "amplitude": LaunchConfiguration("leader_amplitude"),
            "period": LaunchConfiguration("leader_period"),
            "move_joints": ParameterValue(LaunchConfiguration("leader_joints"),
                                          value_type=None),
        }],
    )

    return LaunchDescription(
        declared + [spawn_streaming_controller, bridge, virtual_leader, joy_node])
