# SPDX-License-Identifier: Apache-2.0
#
# PLAYGROUND: a minimal differential-drive AMR (Jetbot) on the Isaac Sim ROS2
# bridge -- the mobile-robot analogue of the UR16e arm bridge, wired the way a
# nav2 stack expects:
#
#   nav2 / teleop --(/cmd_vel Twist)--> [ROS2SubscribeTwist]
#                                          -> [DifferentialController]  (v,w -> wheel rad/s)
#                                          -> [IsaacArticulationController]  (spins the wheel joints)
#   wheels move -> [IsaacComputeOdometry] -> [ROS2PublishOdometry]      (/odom)
#                                         -> [ROS2PublishRawTransformTree] (odom->base_link on /tf)
#   [ROS2PublishClock] -> /clock
#
# Drive it from another terminal (source ROS 2 first):
#   ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
#       "{linear: {x: 0.2}, angular: {z: 0.5}}"
#   ros2 topic echo /odom --once
#   ros2 run tf2_tools view_frames        # odom -> base_link
#
# Run (Isaac bundled python):
#   /isaac-sim/python.sh /isaac-sim/ur_ws/src/playground/amr_diffdrive_ros2.py [--headless]
import sys

import numpy as np
from isaacsim import SimulationApp

# ---- CLI -------------------------------------------------------------------
import argparse
_ap = argparse.ArgumentParser(description="Minimal diff-drive AMR (Jetbot) ROS2 bridge")
_ap.add_argument("--headless", action="store_true")
args, _ = _ap.parse_known_args()

simulation_app = SimulationApp({"renderer": "RaytracedLighting", "headless": args.headless})

# ---- imports that require the app to be live -------------------------------
import carb
import omni.graph.core as og
import usdrt.Sdf
from isaacsim.core.api import SimulationContext
from isaacsim.core.utils import extensions, prims, stage, viewports
from isaacsim.storage.native import get_assets_root_path

# Two extensions are needed: the ROS2 bridge nodes, AND the wheeled-robots
# extension that provides the DifferentialController node type.
extensions.enable_extension("isaacsim.ros2.bridge")
extensions.enable_extension("isaacsim.robot.wheeled_robots")
simulation_app.update()

simulation_context = SimulationContext(stage_units_in_meters=1.0)

assets_root_path = get_assets_root_path()
if assets_root_path is None:
    carb.log_error("Could not find Isaac Sim assets folder")
    simulation_app.close()
    sys.exit()

# ---- scene: floor + Jetbot -------------------------------------------------
ROBOT_PRIM = "/World/Jetbot"
# Jetbot: a 2-wheel differential-drive robot. Params from the official
# isaacsim.robot.wheeled_robots usage docs:
#   wheel joints = left_wheel_joint / right_wheel_joint
#   wheel_radius = 0.035 m,  wheel_base (distance) = 0.10 m
WHEEL_JOINTS = ["left_wheel_joint", "right_wheel_joint"]
WHEEL_RADIUS = 0.035
WHEEL_DISTANCE = 0.10

viewports.set_camera_view(eye=np.array([1.5, 1.5, 1.0]), target=np.array([0.0, 0.0, 0.0]))
stage.add_reference_to_stage(
    assets_root_path + "/Isaac/Environments/Simple_Room/simple_room.usd", "/background")
prims.create_prim(
    ROBOT_PRIM, "Xform",
    position=np.array([0.0, 0.0, 0.05]),   # a touch above the floor so it drops onto its wheels
    usd_path=assets_root_path + "/Isaac/Robots/NVIDIA/Jetbot/jetbot.usd",
)
simulation_app.update()

# ---- ROS2 action graph -----------------------------------------------------
try:
    og.Controller.edit(
        {"graph_path": "/ActionGraph", "evaluator_name": "execution"},
        {
            og.Controller.Keys.CREATE_NODES: [
                ("OnTick", "omni.graph.action.OnPlaybackTick"),
                ("ReadSimTime", "isaacsim.core.nodes.IsaacReadSimulationTime"),
                ("Context", "isaacsim.ros2.bridge.ROS2Context"),
                ("PublishClock", "isaacsim.ros2.bridge.ROS2PublishClock"),
                # --- command path: /cmd_vel -> wheels ---
                ("SubTwist", "isaacsim.ros2.bridge.ROS2SubscribeTwist"),
                ("BreakLin", "omni.graph.nodes.BreakVector3"),   # Twist.linear (vec3) -> x
                ("BreakAng", "omni.graph.nodes.BreakVector3"),   # Twist.angular (vec3) -> z
                ("DiffController", "isaacsim.robot.wheeled_robots.DifferentialController"),
                ("ArtController", "isaacsim.core.nodes.IsaacArticulationController"),
                # --- feedback path: wheels -> /odom + TF ---
                ("ComputeOdom", "isaacsim.core.nodes.IsaacComputeOdometry"),
                ("PublishOdom", "isaacsim.ros2.bridge.ROS2PublishOdometry"),
                ("PublishOdomTF", "isaacsim.ros2.bridge.ROS2PublishRawTransformTree"),
            ],
            og.Controller.Keys.CONNECT: [
                # execution (tick) fan-out -- every active node runs each step
                ("OnTick.outputs:tick", "PublishClock.inputs:execIn"),
                ("OnTick.outputs:tick", "SubTwist.inputs:execIn"),
                ("OnTick.outputs:tick", "DiffController.inputs:execIn"),
                ("OnTick.outputs:tick", "ArtController.inputs:execIn"),
                ("OnTick.outputs:tick", "ComputeOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishOdom.inputs:execIn"),
                ("OnTick.outputs:tick", "PublishOdomTF.inputs:execIn"),
                # shared ROS2 context (one DDS participant)
                ("Context.outputs:context", "PublishClock.inputs:context"),
                ("Context.outputs:context", "SubTwist.inputs:context"),
                ("Context.outputs:context", "PublishOdom.inputs:context"),
                ("Context.outputs:context", "PublishOdomTF.inputs:context"),
                # sim time -> header stamps
                ("ReadSimTime.outputs:simulationTime", "PublishClock.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdom.inputs:timeStamp"),
                ("ReadSimTime.outputs:simulationTime", "PublishOdomTF.inputs:timeStamp"),
                # command path data flow
                ("SubTwist.outputs:linearVelocity", "BreakLin.inputs:tuple"),
                ("SubTwist.outputs:angularVelocity", "BreakAng.inputs:tuple"),
                ("BreakLin.outputs:x", "DiffController.inputs:linearVelocity"),   # forward v
                ("BreakAng.outputs:z", "DiffController.inputs:angularVelocity"),  # yaw rate w
                ("OnTick.outputs:deltaSeconds", "DiffController.inputs:dt"),
                ("DiffController.outputs:velocityCommand", "ArtController.inputs:velocityCommand"),
                # feedback path data flow
                ("ComputeOdom.outputs:linearVelocity", "PublishOdom.inputs:linearVelocity"),
                ("ComputeOdom.outputs:angularVelocity", "PublishOdom.inputs:angularVelocity"),
                ("ComputeOdom.outputs:position", "PublishOdom.inputs:position"),
                ("ComputeOdom.outputs:orientation", "PublishOdom.inputs:orientation"),
                ("ComputeOdom.outputs:position", "PublishOdomTF.inputs:translation"),
                ("ComputeOdom.outputs:orientation", "PublishOdomTF.inputs:rotation"),
            ],
            og.Controller.Keys.SET_VALUES: [
                ("SubTwist.inputs:topicName", "/cmd_vel"),
                # DifferentialController: the unicycle->wheels conversion needs the geometry
                ("DiffController.inputs:wheelRadius", WHEEL_RADIUS),
                ("DiffController.inputs:wheelDistance", WHEEL_DISTANCE),
                # which articulation, which joints (order matches velocityCommand = [left, right])
                ("ArtController.inputs:targetPrim", [usdrt.Sdf.Path(ROBOT_PRIM)]),
                ("ArtController.inputs:jointNames", WHEEL_JOINTS),
                # odometry source + ROS frames
                ("ComputeOdom.inputs:chassisPrim", [usdrt.Sdf.Path(ROBOT_PRIM)]),
                ("PublishOdom.inputs:topicName", "/odom"),
                ("PublishOdom.inputs:odomFrameId", "odom"),
                ("PublishOdom.inputs:chassisFrameId", "base_link"),
                ("PublishOdomTF.inputs:topicName", "/tf"),
                ("PublishOdomTF.inputs:parentFrameId", "odom"),
                ("PublishOdomTF.inputs:childFrameId", "base_link"),
            ],
        },
    )
except Exception as e:
    carb.log_error(f"Failed to build AMR action graph: {e}")
    simulation_app.close()
    sys.exit()

simulation_app.update()

simulation_context.initialize_physics()
simulation_context.play()

print("=" * 70)
print("Jetbot diff-drive ROS2 bridge running.")
print(f"  robot prim         : {ROBOT_PRIM}")
print(f"  wheel joints       : {WHEEL_JOINTS}  (r={WHEEL_RADIUS} m, b={WHEEL_DISTANCE} m)")
print("  subscribes         : /cmd_vel (geometry_msgs/Twist)")
print("  publishes          : /odom, /tf (odom->base_link), /clock")
print("  drive it:  ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \\")
print('             \"{linear: {x: 0.2}, angular: {z: 0.5}}\"')
print("=" * 70, flush=True)

while simulation_app.is_running():
    simulation_context.step(render=True)

simulation_context.stop()
simulation_app.close()
