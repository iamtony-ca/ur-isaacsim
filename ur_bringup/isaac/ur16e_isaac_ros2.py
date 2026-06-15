# SPDX-License-Identifier: Apache-2.0
#
# Isaac Sim 5.1.0 standalone scene + ROS2-bridge OmniGraph for a UR16e.
#
# Spins up Isaac Sim, loads the UR16e, and builds the ROS2 action graph so the
# robot exchanges joints with ros2_control's topic_based hardware:
#
#     ur_bringup (controller_manager / JointStateTopicSystem)
#        --(position cmd)-->  /isaac_joint_commands  --> ArticulationController
#        <--(joint state )--  /isaac_joint_states    <-- PublishJointState
#                             /clock                 <-- PublishClock
#
# Topic names match ur16e_sim.urdf.xacro defaults, so on the ROS side just run:
#     ros2 launch ur_bringup ur16e.launch.py use_sim:=true
#     ros2 launch ur_bringup ur16e_moveit.launch.py use_sim:=true   # optional
#
# Run (uses Isaac Sim's bundled python):
#     /isaac-sim/python.sh \
#         /isaac-sim/ur_ws/src/ur_bringup/isaac/ur16e_isaac_ros2.py [--headless]
#
# Make sure ROS_DOMAIN_ID matches the ROS side (default 0) before launching.
import argparse
import sys

import numpy as np
from isaacsim import SimulationApp

# ---- CLI -------------------------------------------------------------------
parser = argparse.ArgumentParser(description="UR16e Isaac Sim + ROS2 bridge scene")
parser.add_argument("--headless", action="store_true", help="run without the GUI")
parser.add_argument("--no-env", action="store_true", help="skip loading the Simple_Room background")
parser.add_argument("--robot-prim", default="/UR16e", help="stage path the robot USD is referenced onto")
parser.add_argument(
    "--articulation-root",
    default=None,
    help="prim with ArticulationRootAPI (default: <robot-prim>/root_joint, as in the Isaac UR16e USD)",
)
parser.add_argument(
    "--asset-path",
    default="/Isaac/Robots/UniversalRobots/ur16e/ur16e.usd",
    help="UR16e USD path relative to the Isaac assets root (or an absolute/omniverse path)",
)
parser.add_argument("--joint-states-topic", default="isaac_joint_states")
parser.add_argument("--joint-commands-topic", default="isaac_joint_commands")
args, _ = parser.parse_known_args()

CONFIG = {"renderer": "RaytracedLighting", "headless": args.headless}
simulation_app = SimulationApp(CONFIG)

# ---- imports that require the app to be live -------------------------------
import carb
import omni.graph.core as og
import usdrt.Sdf
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import extensions, prims, stage, viewports
from isaacsim.storage.native import get_assets_root_path

# enable ROS2 bridge extension
extensions.enable_extension("isaacsim.ros2.bridge")
simulation_app.update()

simulation_context = SimulationContext(stage_units_in_meters=1.0)

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

ROBOT_PRIM = args.robot_prim
# The Isaac UR16e USD has no default prim and applies ArticulationRootAPI to the
# fixed base joint, so the articulation root is a child prim, not ROBOT_PRIM.
ARTICULATION_ROOT = args.articulation_root or (ROBOT_PRIM + "/root_joint")

# resolve robot USD path: allow absolute / omniverse URLs, else relative to assets root
robot_usd = args.asset_path
if robot_usd.startswith("/Isaac") or not (robot_usd.startswith("/") or "://" in robot_usd):
    robot_usd = assets_root_path + (robot_usd if robot_usd.startswith("/") else "/" + robot_usd)

viewports.set_camera_view(eye=np.array([1.6, 1.6, 1.2]), target=np.array([0.0, 0.0, 0.3]))

# background environment (optional)
if not args.no_env:
    stage.add_reference_to_stage(
        assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd", "/background"
    )

# load the UR16e at the world origin (fixed base comes from the USD)
prims.create_prim(
    ROBOT_PRIM,
    "Xform",
    position=np.array([0.0, 0.0, 0.0]),
    usd_path=robot_usd,
)
simulation_app.update()

# ---- ROS2 action graph -----------------------------------------------------
try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishJointState", "isaacsim.ros2.bridge.ROS2PublishJointState"),
                ("SubscribeJointState", "isaacsim.ros2.bridge.ROS2SubscribeJointState"),
                ("ArticulationController", "isaacsim.core.nodes.IsaacArticulationController"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
            ],
            og.Controller.Keys.CONNECT: [
                ("OnPlaybackTick.outputs:tick", "PublishJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "SubscribeJointState.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnPlaybackTick.outputs:tick", "ArticulationController.inputs:execIn"),
                ("Context.outputs:context", "PublishJointState.inputs:context"),
                ("Context.outputs:context", "SubscribeJointState.inputs:context"),
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("ReadSimTime.outputs:simulationTime", "PublishJointState.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("SubscribeJointState.outputs:jointNames", "ArticulationController.inputs:jointNames"),
                ("SubscribeJointState.outputs:positionCommand", "ArticulationController.inputs:positionCommand"),
                ("SubscribeJointState.outputs:velocityCommand", "ArticulationController.inputs:velocityCommand"),
                ("SubscribeJointState.outputs:effortCommand", "ArticulationController.inputs:effortCommand"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("ArticulationController.inputs:robotPath", ARTICULATION_ROOT),
                ("PublishJointState.inputs:topicName", args.joint_states_topic),
                ("SubscribeJointState.inputs:topicName", args.joint_commands_topic),
                ("PublishJointState.inputs:targetPrim", [usdrt.Sdf.Path(ARTICULATION_ROOT)]),
            ],
        },
    )
except Exception as e:
    carb.log_error(f"Failed to build ROS2 action graph: {e}")
    simulation_app.close()
    sys.exit()

simulation_app.update()

# physics must be initialized before the articulation can be driven
simulation_context.initialize_physics()
simulation_context.play()

print("=" * 70)
print("UR16e Isaac Sim ROS2 bridge running.")
print(f"  robot prim         : {ROBOT_PRIM}")
print(f"  articulation root  : {ARTICULATION_ROOT}")
print(f"  publishes states   : /{args.joint_states_topic}")
print(f"  subscribes commands: /{args.joint_commands_topic}")
print("  publishes clock    : /clock")
print("Verify the exact joint names with:  ros2 topic echo /%s --once" % args.joint_states_topic)
print("=" * 70, flush=True)

# OnPlaybackTick drives the graph every rendered step; just keep stepping.
while simulation_app.is_running():
    simulation_context.step(render=True)

simulation_context.stop()
simulation_app.close()
