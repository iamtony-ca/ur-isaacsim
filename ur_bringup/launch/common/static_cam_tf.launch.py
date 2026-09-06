"""TF for the STATIC workspace camera (base_link -> its optical frames).

Split out of ur16e_2f85_d405_nvblox.launch.py so it can be used WITHOUT nvblox.
Teleop / IL demo recording needs the exterior camera (it is the policy's
`video.exterior` view) but has no reason to run an ESDF mapper, and previously
the only publisher of this TF was the nvblox launch.

    ros2 launch ur_bringup static_cam_tf.launch.py           # standalone (teleop/IL)
    # nvblox launch includes this automatically (static_cam_tf:=true, default)

*** The pose here MUST match the Isaac static camera. ***
Defaults correspond to ur16e_isaac_ros2.py --with-static-cam with its default
--static-cam-xyz 1.10,0.0,1.10 and --static-cam-target 0.30,0.0,0.15. Change one
and you must change the other, or depth back-projects to the wrong place and
nvblox maps obstacles into thin air (HISTORY.md 12).

Frames
------
    static_cam_depth_optical_frame   nvblox / perception depth
    static_cam_color_optical_frame   RGB (policy input)

In sim BOTH images come from one render product, so the colour and depth optical
frames coincide exactly -- the colour frame is published as an identity child of
the depth frame. On a real D435/D455 they do NOT coincide (a few cm of baseline);
there the RealSense driver publishes the true offset, so set publish_color:=false
and let the driver own the frames.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# base_link -> static_cam_depth_optical_frame, for xyz=(1.10,0,1.10) looking at
# (0.30,0,0.15) in ROS optical convention (z = view direction, x = image right).
SCAM_TF = ["1.10", "0.0", "1.10",
           "0.66424980", "0.66424980", "-0.24242978", "-0.24242978"]  # x y z qx qy qz qw

DEPTH_FRAME = "static_cam_depth_optical_frame"
COLOR_FRAME = "static_cam_color_optical_frame"


def generate_launch_description():
    args = [
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("parent_frame", default_value="base_link"),
        DeclareLaunchArgument("publish_color", default_value="true",
                              description="Also publish the colour optical frame (identity to depth, "
                                          "because sim renders both from one sensor). Set false on "
                                          "real hardware, where the camera driver owns the frames."),
    ]
    for n, d in (("x", SCAM_TF[0]), ("y", SCAM_TF[1]), ("z", SCAM_TF[2]),
                 ("qx", SCAM_TF[3]), ("qy", SCAM_TF[4]),
                 ("qz", SCAM_TF[5]), ("qw", SCAM_TF[6])):
        args.append(DeclareLaunchArgument(n, default_value=d))

    use_sim_time = {"use_sim_time": LaunchConfiguration("use_sim_time")}

    depth_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="static_cam_tf",
        parameters=[use_sim_time],
        arguments=["--x", LaunchConfiguration("x"),
                   "--y", LaunchConfiguration("y"),
                   "--z", LaunchConfiguration("z"),
                   "--qx", LaunchConfiguration("qx"),
                   "--qy", LaunchConfiguration("qy"),
                   "--qz", LaunchConfiguration("qz"),
                   "--qw", LaunchConfiguration("qw"),
                   "--frame-id", LaunchConfiguration("parent_frame"),
                   "--child-frame-id", DEPTH_FRAME],
    )

    color_tf = Node(
        package="tf2_ros", executable="static_transform_publisher", name="static_cam_color_tf",
        condition=IfCondition(LaunchConfiguration("publish_color")),
        parameters=[use_sim_time],
        arguments=["--x", "0", "--y", "0", "--z", "0",
                   "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                   "--frame-id", DEPTH_FRAME, "--child-frame-id", COLOR_FRAME],
    )

    return LaunchDescription(args + [depth_tf, color_tf])
