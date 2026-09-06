"""nvblox + robot self-segmentation for cuMotion world-collision (UR16e+2F-85+D405).

Builds a 3D ESDF from a depth camera and serves it to cuMotion
(/nvblox_node/get_esdf_and_gradient) so the GPU planner avoids real obstacles in
the workspace, in real time.

*** Uses a STATIC external camera by default (depth_image/depth_info default to
/static_cam/depth/*). *** The eye-in-hand D405 moves with the arm and mostly sees
the robot itself, so its TSDF is dominated by the robot and the arm's own start
pose ends up "in collision". A camera fixed over the workspace gives a stable map
of the actual obstacles. The eye-in-hand D405 stays dedicated to grasp perception.
Point depth_image/depth_info back at /camera/depth/* to use the eye-in-hand camera.

Pipeline (use_robot_segmenter:=true, DEFAULT):
    static cam depth (/static_cam/depth/image_rect_raw + .../camera_info)
      -> robot_segmenter  (masks the arm/gripper out of the depth using our
                           cumotion URDF/XRDF + TF)  -> /cumotion/camera_0/world_depth
      -> nvblox_node  -> 3D ESDF (base_link frame)  -> get_esdf_and_gradient
      -> cuMotion (read_esdf_world:=true)

The segmenter still masks the robot out of the static camera's view so the arm
is never reconstructed as an obstacle. use_robot_segmenter:=false feeds raw depth
to nvblox (only for debugging the camera->nvblox path).

This launch also publishes the static base_link->static_cam_depth_optical_frame TF
(static_cam_tf:=true). Its pose MUST match the Isaac static camera
(ur16e_isaac_ros2.py --with-static-cam, --static-cam-xyz/--static-cam-target).

Run on top of the Set 3 stack:
    # sim: Isaac with BOTH cameras
    /isaac-sim/python.sh .../isaac/common/ur16e_isaac_ros2.py \
        --asset-path .../ur16e_2f85_d405.usd --with-camera --with-static-cam
    ros2 launch ur_bringup ur16e_2f85_d405.launch.py
    ros2 launch ur_bringup ur16e_2f85_d405_nvblox.launch.py
    ros2 launch ur_bringup ur16e_2f85_d405_cumotion_moveit.launch.py read_esdf_world:=true
        # real: depth_image/depth_info:=<your static cam>, use_sim_time:=false, static_cam_tf to your rig

nvblox config: config/ur16e_2f85_d405/nvblox_cumotion.yaml (3D ESDF, voxel 0.02,
global_frame=base_link -- must match cuMotion's robot base frame). See HARDWARE.md §4.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode
from launch_ros.parameter_descriptions import ParameterValue

WORLD_DEPTH = "/cumotion/camera_0/world_depth"   # robot-masked depth -> nvblox
# The static camera pose now lives in launch/common/static_cam_tf.launch.py --
# ONE definition shared by nvblox and by teleop/IL recording. It must stay in sync
# with ur16e_isaac_ros2.py --static-cam-xyz/--static-cam-target.


def generate_launch_description():
    use_sim_time = LaunchConfiguration("use_sim_time")
    depth_image = LaunchConfiguration("depth_image")
    depth_info = LaunchConfiguration("depth_info")
    use_seg = LaunchConfiguration("use_robot_segmenter")
    static_cam_tf = LaunchConfiguration("static_cam_tf")
    seg_buffer = ParameterValue(LaunchConfiguration("segmenter_buffer"), value_type=float)

    ur_share = get_package_share_directory("ur_bringup")
    # nvblox's stock base config. It ships ONLY inside nvblox_examples_bringup, whose
    # dependency tree (triton / tensor_rt / visual_slam / detectnet / unet + model
    # installers) is ~100 extra debs and forces container-wide python3.12 upgrades --
    # unacceptable on a machine shared with other workspaces. So we vendor the single
    # yaml under config/ur16e_2f85_d405/vendor/ and use the apt copy only if it happens
    # to be installed. See that file's header for how to refresh it.
    nvblox_base = os.path.join(ur_share, "config", "ur16e_2f85_d405", "vendor", "nvblox_base.yaml")
    try:
        _apt_base = os.path.join(get_package_share_directory("nvblox_examples_bringup"),
                                 "config", "nvblox", "nvblox_base.yaml")
        if os.path.isfile(_apt_base):
            nvblox_base = _apt_base
    except Exception:
        pass
    nvblox_cumotion = os.path.join(ur_share, "config", "ur16e_2f85_d405", "nvblox_cumotion.yaml")
    cumotion_urdf = os.path.join(ur_share, "cumotion", "ur16e_2f85.urdf")
    cumotion_xrdf = os.path.join(ur_share, "cumotion", "ur16e_2f85.xrdf")

    # Robot self-segmentation: depth -> masked world_depth (arm/gripper removed).
    segmenter = ComposableNodeContainer(
        name="robot_segmenter_container", namespace="", package="rclcpp_components",
        executable="component_container_mt", output="screen",
        condition=IfCondition(use_seg),
        composable_node_descriptions=[
            ComposableNode(
                name="robot_segmenter",
                package="isaac_ros_cumotion_robot_segmenter",
                plugin="nvidia::isaac_ros::manipulator::RobotSegmenter",
                parameters=[{
                    "urdf_path": cumotion_urdf,
                    "xrdf_path": cumotion_xrdf,
                    "robot_base_frame": "base_link",
                    # How far BEYOND the XRDF collision spheres the robot is masked out of
                    # the depth image. Too small -> the arm's silhouette leaks into the TSDF
                    # and cuMotion rejects the start pose with "world collision detected";
                    # too large -> real obstacles close to the arm get masked away too.
                    # Only read at node init, so change it here / via the launch arg, not
                    # with `ros2 param set`.
                    "additional_buffer_distance": seg_buffer,
                    "input_qos": "SYSTEM_DEFAULT",
                    "output_qos": "SYSTEM_DEFAULT",
                    "use_sim_time": use_sim_time,
                }],
                remappings=[
                    ("depth_image", depth_image),
                    ("camera_info_depth", depth_info),
                    ("joint_states", "/joint_states"),
                    ("robot_mask", "/cumotion/camera_0/robot_mask"),
                    ("robot_depth", WORLD_DEPTH),
                ],
            ),
        ],
    )

    # nvblox on the MASKED depth (segmenter on) ...
    nvblox_masked = Node(
        package="nvblox_ros", executable="nvblox_node", name="nvblox_node", output="screen",
        condition=IfCondition(use_seg),
        parameters=[nvblox_base, nvblox_cumotion, {"use_sim_time": use_sim_time}],
        remappings=[("camera_0/depth/image", WORLD_DEPTH),
                    ("camera_0/depth/camera_info", depth_info)],
    )
    # ... or on RAW depth (segmenter off, debug only)
    nvblox_raw = Node(
        package="nvblox_ros", executable="nvblox_node", name="nvblox_node", output="screen",
        condition=UnlessCondition(use_seg),
        parameters=[nvblox_base, nvblox_cumotion, {"use_sim_time": use_sim_time}],
        remappings=[("camera_0/depth/image", depth_image),
                    ("camera_0/depth/camera_info", depth_info)],
    )

    # base_link -> static camera optical frames. Lives in its own launch so teleop
    # / IL recording can publish it WITHOUT running nvblox (they need the exterior
    # camera, not an ESDF mapper). Single source of truth for the pose.
    static_tf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(ur_share, "launch", "common", "static_cam_tf.launch.py")),
        condition=IfCondition(static_cam_tf),
        launch_arguments={"use_sim_time": use_sim_time}.items(),
    )

    return LaunchDescription([
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("depth_image", default_value="/static_cam/depth/image_rect_raw",
                              description="depth image for nvblox (default: static workspace camera)"),
        DeclareLaunchArgument("depth_info", default_value="/static_cam/depth/camera_info"),
        DeclareLaunchArgument("use_robot_segmenter", default_value="true",
                              description="Mask the robot out of the depth before nvblox. "
                                          "false = raw depth (debug)."),
        DeclareLaunchArgument("segmenter_buffer", default_value="0.12",
                              description="robot_segmenter additional_buffer_distance [m] — how far past the "
                                          "XRDF spheres the arm is cut out of the depth. Raise it (0.2~0.3) if "
                                          "cuMotion rejects the start pose with 'world collision detected' "
                                          "(= arm residue leaked into the TSDF)."),
        DeclareLaunchArgument("static_cam_tf", default_value="true",
                              description="Publish base_link->static_cam_depth_optical_frame TF "
                                          "(set false if your camera TF comes from elsewhere)."),
        static_tf,
        segmenter,
        nvblox_masked,
        nvblox_raw,
    ])
