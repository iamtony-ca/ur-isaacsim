"""MoveIt2 for UR16e (+2F-85/D405) using NVIDIA cuMotion as the planner.

Adds the GPU motion planner (Isaac ROS cuMotion) as a MoveIt planning pipeline
and makes it the default. Execution stays on our existing
scaled_joint_trajectory_controller (sim or real).

Two robot-model modes (ur_only):

  ur_only:=true  (DEFAULT, needed for INTERACTIVE RViz cuMotion):
    move_group/RViz use a UR-arm-only robot model (UR16e, 6 joints) + the plain
    ur_moveit_config SRDF. Why: the cuMotion MoveIt plugin forwards the request's
    full start_state to the GPU solver without filtering to the arm c-space, so a
    12-joint start (RViz always sends the full robot state incl. the 2F-85 finger
    + mimics) is rejected ("c-space [12] must equal [6]"). A UR-arm-only move_group
    makes the start state 6 joints, so dragging the marker -> Plan -> Execute works.
    The gripper still EXECUTES via its own controller and is visible in Isaac; it
    just isn't shown in this RViz, and cuMotion's gripper collision still comes from
    the XRDF spheres (cuMotion plans WITH the gripper either way). (Mirrors NVIDIA's
    official UR cuMotion example.)

  ur_only:=false (gripper-aware move_group, programmatic use):
    move_group uses the full robot_description published by the control stack + our
    combined gripper SRDF. cuMotion plans fine when the request's start_state is
    empty (planner falls back to /joint_states filtered to the arm), e.g.
    moveit_plan_execute_demo.py — but interactive RViz <random valid> will fail.

Bring up on top of the control stack:
    ros2 launch ur_bringup ur16e_2f85_d405.launch.py            # (or ur16e_2f85 / *_real)
    ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py   # RViz: drag marker -> Plan
    #   real: add use_sim_time:=false

Requires the Isaac ROS cuMotion apt packages + CUDA 13 + VPI (see HARDWARE.md §4).
UR16e cuMotion robot config (URDF/XRDF): cumotion/ (gen_xrdf.py).
"""
import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction, RegisterEventHandler
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.substitutions import (Command, FindExecutable, LaunchConfiguration,
                                   PathJoinSubstitution)
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


def launch_setup(context, *args, **kwargs):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context) in ("true", "True", "1")
    launch_rviz = LaunchConfiguration("launch_rviz")
    read_esdf_world = LaunchConfiguration("read_esdf_world").perform(context) in ("true", "True", "1")
    ur_only = LaunchConfiguration("ur_only").perform(context) in ("true", "True", "1")

    ur_share = get_package_share_directory("ur_bringup")

    from moveit_configs_utils import MoveItConfigsBuilder

    builder = MoveItConfigsBuilder(robot_name="ur", package_name="ur_moveit_config")
    extra_move_group_params = []
    if ur_only:
        # UR-arm-only robot model (6 joints) + plain ur SRDF -> RViz start state is 6 joints.
        builder = builder.robot_description(
            file_path=os.path.join(ur_share, "urdf", "ur16e", "ur16e_sim.urdf.xacro"),
        ).robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur16e"})
    else:
        # full robot_description from the control stack's RSP; our combined gripper SRDF.
        builder = builder.robot_description_semantic(Path("srdf") / "ur.srdf.xacro", {"name": "ur16e"})
        semantic = ParameterValue(
            Command([
                FindExecutable(name="xacro"), " ",
                PathJoinSubstitution([FindPackageShare("ur_bringup"), "srdf", "common", "ur16e_2f85.srdf.xacro"]),
                " ", "name:=ur16e",
            ]),
            value_type=str,
        )
        extra_move_group_params.append({"robot_description_semantic": semantic})

    cfg = builder.to_moveit_configs()

    # Add cuMotion as a planning pipeline and make it the default (ompl stays available).
    cumotion_planning_yaml = os.path.join(
        get_package_share_directory("isaac_ros_cumotion_moveit"),
        "config", "isaac_ros_cumotion_planning.yaml",
    )
    with open(cumotion_planning_yaml) as f:
        cumotion_planning = yaml.safe_load(f)
    cfg.planning_pipelines["planning_pipelines"].insert(0, "isaac_ros_cumotion")
    cfg.planning_pipelines["isaac_ros_cumotion"] = cumotion_planning
    cfg.planning_pipelines["default_planning_pipeline"] = "isaac_ros_cumotion"

    # cuMotion planner node (always the full UR16e+2F-85 model for correct collision).
    cumotion_urdf = os.path.join(ur_share, "cumotion", "ur16e_2f85.urdf")
    cumotion_xrdf = os.path.join(ur_share, "cumotion", "ur16e_2f85.xrdf")
    cumotion_container = ComposableNodeContainer(
        name="cumotion_container", namespace="", package="rclcpp_components",
        executable="component_container_mt", output="screen",
        composable_node_descriptions=[
            ComposableNode(
                name="static_planning_scene_server",
                package="isaac_ros_cumotion",
                plugin="nvidia::isaac_ros::cumotion::StaticPlanningSceneServer",
                parameters=[{"moveit_collision_objects_scene_file": ""}],
            ),
            ComposableNode(
                name="cumotion_planner",
                package="isaac_ros_cumotion",
                plugin="nvidia::isaac_ros::cumotion::CumotionPlanner",
                parameters=[{
                    "urdf_file_path": cumotion_urdf,
                    "xrdf_file_path": cumotion_xrdf,
                    "read_esdf_world": read_esdf_world,
                    "update_esdf_on_request": True,
                    "esdf_service_name": "/nvblox_node/get_esdf_and_gradient",
                    "joint_states_topic": "/joint_states",
                    "time_dilation_factor": 0.5,
                    "interpolation_dt": 0.05,
                    "use_sim_time": use_sim_time,
                }],
            ),
        ],
    )

    common = {"use_sim_time": use_sim_time, "publish_robot_description_semantic": True,
              "trajectory_execution.allowed_start_tolerance": 0.1}
    move_group = Node(
        package="moveit_ros_move_group", executable="move_group", output="screen",
        parameters=[cfg.to_dict(), common, *extra_move_group_params],
    )

    rviz_config = PathJoinSubstitution([FindPackageShare("ur_moveit_config"), "config", "moveit.rviz"])
    rviz_params = [cfg.robot_description_kinematics, cfg.planning_pipelines, cfg.joint_limits,
                   {"use_sim_time": use_sim_time}, *extra_move_group_params]
    if ur_only:
        rviz_params.insert(0, cfg.robot_description)            # UR-arm-only model for display
        rviz_params.insert(1, cfg.robot_description_semantic)
    rviz_node = Node(
        package="rviz2", executable="rviz2", name="rviz2_cumotion", output="log",
        condition=IfCondition(launch_rviz),
        arguments=["-d", rviz_config], parameters=rviz_params,
    )

    # gate move_group/rviz on the control stack's /robot_description (TF/joint source).
    wait_rd = Node(package="ur_robot_driver", executable="wait_for_robot_description", output="screen")
    return [
        cumotion_container,
        wait_rd,
        RegisterEventHandler(OnProcessExit(target_action=wait_rd, on_exit=[move_group, rviz_node])),
    ]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true",
                              description="true for Isaac sim (/clock); false for the real robot."),
        DeclareLaunchArgument("launch_rviz", default_value="true"),
        DeclareLaunchArgument(
            "ur_only", default_value="true",
            description="true = UR-arm-only move_group/RViz so INTERACTIVE RViz cuMotion works "
                        "(start state = 6 joints). false = full gripper-aware model (programmatic "
                        "cuMotion via empty start_state; RViz interactive <random valid> will fail)."),
        DeclareLaunchArgument(
            "read_esdf_world", default_value="false",
            description="Let cuMotion pull a world ESDF from nvblox (D405 depth). Needs nvblox."),
        OpaqueFunction(function=launch_setup),
    ])
