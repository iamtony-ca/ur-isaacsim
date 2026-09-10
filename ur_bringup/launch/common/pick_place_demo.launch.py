"""Run the pick&place state machine against Isaac or the real UR16e.

    ros2 launch ur_bringup pick_place_demo.launch.py use_sim:=true cycles:=6
    ros2 launch ur_bringup pick_place_demo.launch.py use_sim:=false cycles:=1

`use_sim` picks which parameter files are layered, and that is the only
difference between the two paths -- the same node, the same task parameters, the
same MoveIt actions. This is what makes "prove it in sim, then run it on the real
arm" a configuration change rather than a rewrite.

    pick_place.yaml       task parameters, both backends
    pick_place_sim.yaml   ON TOP, sim only

The sim file is layered LAST so it wins, and it is absent on real hardware. It is
currently EMPTY: both overrides it once carried turned out to be bugs, not real
sim/real differences (see the file's own header, and HISTORY.md 28). The seam is
kept because a genuine difference will show up the day the real gripper runs, and
because getting the direction right matters -- a sim compensation that leaks onto
the real robot aims the arm into the table.

Expects the control stack and MoveIt to be up already -- see README.md.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    share = get_package_share_directory("ur_bringup")
    cfg = os.path.join(share, "config", "common")

    use_sim = LaunchConfiguration("use_sim")
    args = [
        DeclareLaunchArgument(
            "use_sim", default_value="true",
            description="true = Isaac (adds the sim-only gripper compensations), "
                        "false = real UR16e + real 2F-85"),
        DeclareLaunchArgument("cycles", default_value="1",
                              description="how many pick&place cycles to run"),
        # Substitution as the default, so it tracks use_sim without a
        # PythonExpression -- "not true" is a NameError in launch's evaluator, not
        # a boolean, which is how the first version of this file broke.
        DeclareLaunchArgument(
            "reset_each", default_value=use_sim,
            description="call /scene/reset_episode before each cycle. Defaults to "
                        "use_sim: the service only exists in the Isaac scene"),
        DeclareLaunchArgument("record", default_value="false",
                              description="drive il_recorder.py over /il/* services"),
        # Which part, and which marker. Both objects and both markers exist in
        # every episode, so these are what make one language-conditioned task
        # differ from another (plan_il_vla.md 2.8).
        DeclareLaunchArgument("object_topic", default_value="/scene/object_pose",
                              description="e.g. /scene/objects/red/pose"),
        DeclareLaunchArgument("place_topic", default_value="/scene/place_pose",
                              description="e.g. /scene/places/left/pose"),
        # Layered LAST, so it wins over both files above. Defaults to an empty
        # overrides file rather than "": the argument was previously declared and
        # never used, so anything passed to it was silently dropped.
        DeclareLaunchArgument(
            "extra_params", default_value=os.path.join(cfg, "empty_overrides.yaml"),
            description="extra YAML, layered last (wins over pick_place*.yaml)"),
    ]

    common = [
        os.path.join(cfg, "pick_place.yaml"),
        {"use_sim_time": use_sim,
         "cycles": LaunchConfiguration("cycles"),
         "reset_each": LaunchConfiguration("reset_each"),
         "record": LaunchConfiguration("record"),
         "object_topic": LaunchConfiguration("object_topic"),
         "place_topic": LaunchConfiguration("place_topic")},
    ]

    # Two Node actions rather than one, because a parameter file has to be present
    # or absent as a whole -- there is no "include this file only if" for a single
    # entry in the list.
    sim_node = Node(
        package="ur_bringup", executable="pick_place_demo.py", name="pick_place_demo",
        output="screen", condition=IfCondition(use_sim),
        parameters=[common[0], os.path.join(cfg, "pick_place_sim.yaml"), common[1],
                    LaunchConfiguration("extra_params")],
    )
    real_node = Node(
        package="ur_bringup", executable="pick_place_demo.py", name="pick_place_demo",
        output="screen",
        condition=UnlessCondition(use_sim),
        parameters=common + [LaunchConfiguration("extra_params")],
    )
    return LaunchDescription(args + [sim_node, real_node])
