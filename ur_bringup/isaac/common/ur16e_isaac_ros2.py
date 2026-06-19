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
# --- eye-in-hand RealSense D405 (sim) ---
parser.add_argument("--with-camera", action="store_true",
                    help="attach an eye-in-hand D405 camera and publish RGB/depth/points/camera_info "
                         "(Set 3 rig; pair with the ur16e_2f85_d405_* launches whose URDF carries the "
                         "camera frames. Only meaningful with the 2F-85 asset.)")
parser.add_argument("--camera-parent", default="wrist_3_link",
                    help="articulation link the camera is parented to (moves with the arm)")
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

import os as _os0  # optional viewport override for inspection: VIEW_EYE/VIEW_TARGET="x,y,z"
_veye = _os0.environ.get("VIEW_EYE"); _vtgt = _os0.environ.get("VIEW_TARGET")
_eye = np.array([float(v) for v in _veye.split(",")]) if _veye else np.array([1.6, 1.6, 1.2])
_tgt = np.array([float(v) for v in _vtgt.split(",")]) if _vtgt else np.array([0.0, 0.0, 0.3])
viewports.set_camera_view(eye=_eye, target=_tgt)
if _os0.environ.get("NO_DOF") == "1":  # disable depth-of-field blur (sharp close-ups for inspection)
    import carb as _carb0
    _carb0.settings.get_settings().set("/rtx/post/dof/enabled", False)

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

# ---- Robotiq coupling / camera-mount VISUALS ------------------------------
# NOTE: the gripper coupling (GRP-ES-CPL-077) + camera-mount visuals and the
# +18 mm gripper standoff are represented in the URDF/RViz model. They are NOT
# injected into the Isaac stage at runtime: shifting the baked 2F-85 via the
# fixed joint / prim Xform desynchronises the articulation (broken robot or a
# dead finger_joint drive). The correct way to add them to Isaac is to BAKE the
# standoff + coupling mesh into the asset in build_ur16e_2f85.py and re-export,
# so the articulation is consistent from the start. Until then Isaac keeps the
# gripper at its baked flange pose so it actuates normally.

# ---- eye-in-hand D405 camera graph (optional) ------------------------------
# Single render product (one camera) -> RGB + depth + pointcloud + camera_info,
# on the same topic names the real realsense2_camera driver uses, so the sim and
# real perception stacks (and downstream cuMotion / DepthAnything / FoundationPose)
# subscribe identically. depth is rendered from the same sensor as color, so it
# is inherently aligned to the color frame.
if args.with_camera:
    import omni.usd
    from pxr import Gf, UsdGeom, Vt

    CAM_PRIM = f"{ROBOT_PRIM}/{args.camera_parent}/d405_camera"
    CAM_W, CAM_H = 640, 480
    # Eye-in-hand D405 mount — kept IDENTICAL to the URDF (realsense_d405_macro
    # <origin>). Pose taken from PickNik's open-source UR RealSense camera adapter
    # (picknik_accessories ur_realsense_camera_adapter, d415_mount_joint):
    # camera_link at xyz=(0,-0.067,0.0171) from tool0, pitched (-pi/2 + 6deg)
    # about Y with +pi/2 yaw so the optical axis (camera_link +x) looks down the
    # tool axis toward the grasp region (~6deg off-axis). That adapter targets the
    # D415/L515 (no D405 variant) and adds ~7 mm flange->gripper that we omit, so
    # this is a faithful *representative* mount pending a D405 bracket / hand-eye.
    # We compose the UR fixed chain wrist_3->flange->tool0 with this mount and the
    # ROS-optical->USD-camera convention so the Isaac sensor pose EXACTLY matches
    # RViz/TF. Change these numbers HERE and in realsense_d405_macro's <origin>
    # together.
    def _rpy(r, p, y):   # URDF fixed-axis (XYZ) Euler -> 3x3 rotation
        Rx = np.array([[1, 0, 0], [0, np.cos(r), -np.sin(r)], [0, np.sin(r), np.cos(r)]])
        Ry = np.array([[np.cos(p), 0, np.sin(p)], [0, 1, 0], [-np.sin(p), 0, np.cos(p)]])
        Rz = np.array([[np.cos(y), -np.sin(y), 0], [np.sin(y), np.cos(y), 0], [0, 0, 1]])
        return Rz @ Ry @ Rx

    _w2tool0 = _rpy(0, -np.pi / 2, -np.pi / 2) @ _rpy(np.pi / 2, 0, np.pi / 2)
    _R = (_w2tool0
          @ _rpy(0, -np.pi / 2 + np.deg2rad(8.0), np.pi / 2)  # tool0->camera_link: 8deg pitch
          #   matches the bracket's actual mount-surface normal so the body sits flush
          #   (PickNik nominal is 6deg; the visual mesh surface measures 8deg)
          @ _rpy(-np.pi / 2, 0, -np.pi / 2)      # camera_link -> ROS optical (z-fwd)
          @ _rpy(np.pi, 0, 0))                   # ROS optical -> USD camera (-z fwd)
    # camera_link is held by the camera mount, which is flush on the flange, so the
    # d415_mount is taken directly off tool0 (no coupling offset — the coupling is
    # ABOVE the mount, on the gripper side).
    _t = _w2tool0 @ np.array([0.0, -0.067, 0.01847])  # camera_link in wrist_3: solved (gap=0)
    #     for the 8deg pitch so the now-parallel D405 back face seats flush on the bracket
    # Put the sensor at the D405 FRONT FACE: +half-depth (11.5 mm) forward along
    # the optical axis. The 23 mm body box (child, behind the lens) then sits
    # centred on camera_link, nesting its back into the bracket cradle instead of
    # poking through it, while the lens stays the front-most point (no occlusion).
    # Mirrors realsense_d405_macro's optical-frame forward offset.
    _t = _t + 0.0115 * (_R @ np.array([0.0, 0.0, -1.0]))
    _M3 = Gf.Matrix3d(*[float(v) for v in _R.T.flatten()])  # USD rows = world axes

    stage_obj = omni.usd.get_context().get_stage()
    cam = UsdGeom.Camera.Define(stage_obj, CAM_PRIM)
    cam.GetFocalLengthAttr().Set(1.88)        # ~87 deg HFOV with the apertures below
    cam.GetHorizontalApertureAttr().Set(3.6)
    cam.GetVerticalApertureAttr().Set(2.7)
    cam.GetClippingRangeAttr().Set(Gf.Vec2f(0.01, 10.0))
    xf = UsdGeom.Xformable(cam.GetPrim())
    xf.ClearXformOpOrder()
    xf.AddTransformOp().Set(Gf.Matrix4d(_M3, Gf.Vec3d(float(_t[0]), float(_t[1]), float(_t[2]))))

    # Visible D405 housing so the camera shows up in the GUI (a UsdGeom.Camera
    # prim itself draws no geometry). Mirrors the URDF visual exactly: a
    # 42 x 42 x 23 mm dark-aluminium box. Parented UNDER the camera prim so it
    # tracks the exact sensor pose, and offset to +Z (cameras look down -Z) so
    # the front face sits at the lens and the body never occludes the view.
    import os as _os
    if _os.environ.get("CAM_BODY", "1") != "0":
        body = UsdGeom.Cube.Define(stage_obj, CAM_PRIM + "/body")
        body.GetSizeAttr().Set(1.0)
        bxf = UsdGeom.Xformable(body.GetPrim())
        bxf.ClearXformOpOrder()
        bxf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.0115))   # front face flush at lens
        bxf.AddScaleOp().Set(Gf.Vec3f(0.042, 0.042, 0.023))    # x=right, y=up, z=optical(thin)
        body.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.25, 0.25, 0.27)]))
    # NOTE: the PickNik camera bracket is set aside for now (focusing on the real
    # GRP-ES-CPL-077 gripper coupling first). The coupling standoff is applied to
    # the gripper below (outside this block) for both Set 2 and Set 3.
    simulation_app.update()

    try:
        og.Controller.edit(
            {"graph_path": "/CameraGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("CamContext", "isaacsim.ros2.bridge.ROS2Context"),
                    ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("RGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("Depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("DepthPCL", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("ColorInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    ("DepthInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "RenderProduct.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "RGB.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "Depth.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "DepthPCL.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "ColorInfo.inputs:execIn"),
                    ("RenderProduct.outputs:execOut", "DepthInfo.inputs:execIn"),
                    ("RenderProduct.outputs:renderProductPath", "RGB.inputs:renderProductPath"),
                    ("RenderProduct.outputs:renderProductPath", "Depth.inputs:renderProductPath"),
                    ("RenderProduct.outputs:renderProductPath", "DepthPCL.inputs:renderProductPath"),
                    ("RenderProduct.outputs:renderProductPath", "ColorInfo.inputs:renderProductPath"),
                    ("RenderProduct.outputs:renderProductPath", "DepthInfo.inputs:renderProductPath"),
                    ("CamContext.outputs:context", "RGB.inputs:context"),
                    ("CamContext.outputs:context", "Depth.inputs:context"),
                    ("CamContext.outputs:context", "DepthPCL.inputs:context"),
                    ("CamContext.outputs:context", "ColorInfo.inputs:context"),
                    ("CamContext.outputs:context", "DepthInfo.inputs:context"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("RenderProduct.inputs:cameraPrim", [usdrt.Sdf.Path(CAM_PRIM)]),
                    ("RenderProduct.inputs:width", CAM_W),
                    ("RenderProduct.inputs:height", CAM_H),
                    ("RGB.inputs:type", "rgb"),
                    ("RGB.inputs:topicName", "/camera/color/image_raw"),
                    ("RGB.inputs:frameId", "camera_color_optical_frame"),
                    ("Depth.inputs:type", "depth"),
                    ("Depth.inputs:topicName", "/camera/depth/image_rect_raw"),
                    ("Depth.inputs:frameId", "camera_depth_optical_frame"),
                    ("DepthPCL.inputs:type", "depth_pcl"),
                    ("DepthPCL.inputs:topicName", "/camera/depth/color/points"),
                    ("DepthPCL.inputs:frameId", "camera_depth_optical_frame"),
                    ("ColorInfo.inputs:topicName", "/camera/color/camera_info"),
                    ("ColorInfo.inputs:frameId", "camera_color_optical_frame"),
                    ("DepthInfo.inputs:topicName", "/camera/depth/camera_info"),
                    ("DepthInfo.inputs:frameId", "camera_depth_optical_frame"),
                ],
            },
        )
        print(f"  eye-in-hand camera  : {CAM_PRIM} ({CAM_W}x{CAM_H})")
        print("  camera topics       : /camera/color/image_raw, /camera/depth/image_rect_raw,")
        print("                        /camera/depth/color/points, /camera/{color,depth}/camera_info")
    except Exception as e:
        carb.log_error(f"Failed to build camera graph: {e}")

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
