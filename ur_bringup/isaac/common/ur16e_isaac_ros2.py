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
# --- static external depth camera (sim) : overlooks the workspace for nvblox ---
parser.add_argument("--with-static-cam", action="store_true",
                    help="add a STATIC camera overlooking the workspace (not attached to the arm) and "
                         "publish RGB + depth + camera_info on /static_cam/{color,depth}/*. "
                         "RGB is the IL/VLA policy's exterior view; depth is what nvblox "
                         "uses to build the obstacle ESDF for cuMotion; the eye-in-hand D405 is for grasp "
                         "perception. Pose is fixed in the base frame -- keep it in sync with the static TF "
                         "in ur16e_2f85_d405_nvblox.launch.py.")
parser.add_argument("--static-cam-xyz", default="1.10,0.0,1.10",
                    help="static camera position in the base frame (m), comma-separated")
parser.add_argument("--static-cam-target", default="0.30,0.0,0.15",
                    help="point in the base frame the static camera looks at (m), comma-separated")
# --- demo obstacle (sim) : a box the static camera sees -> nvblox -> cuMotion avoids ---
parser.add_argument("--obstacle", action="store_true",
                    help="spawn a visible box obstacle in the workspace (for the nvblox/cuMotion "
                         "real-time avoidance demo). The static camera sees it, nvblox maps it, "
                         "cuMotion routes around it.")
# default obstacle: a pillar in the +x/+y workspace, raised so its base clears BOTH
# the home pose (arm up, links near x~0) and the all-zeros startup pose (arm
# horizontal at z~0.18). cuMotion must route around it to reach goals beyond.
parser.add_argument("--obstacle-pose", default="0.5,0.1,0.6",
                    help="obstacle box center in the base frame (m), comma-separated")
parser.add_argument("--obstacle-size", default="0.12,0.5,0.1",
                    help="obstacle box size x,y,z (m), comma-separated")
# --- pick&place scene (teleop -> IL demo collection) ---
# Everything here is OPT-IN so the nvblox / cuMotion demos above keep their scene.
parser.add_argument("--scene", default="none", choices=["none", "pick_place"],
                    help="'pick_place' spawns a work surface + a PHYSICS-enabled object + a "
                         "place-target marker. This is the scene teleop demos are recorded in. "
                         "Unlike --obstacle (visual only) the object has rigid body + collider "
                         "+ mass, because the gripper must actually hold it.")
parser.add_argument("--table", action="store_true",
                    help="spawn an explicit work surface (FixedCuboid) so the object rests on a "
                         "known plane regardless of what the background environment provides")
parser.add_argument("--table-height", default="0.0",
                    help="top surface height of the work table [m] in the base frame")
parser.add_argument("--object-pose", default="0.55,0.0,0.03",
                    help="object spawn position x,y,z (m) in the base frame")
parser.add_argument("--object-size", default="0.05,0.05,0.05",
                    help="object size x,y,z (m). 2F-85 stroke is 85 mm, so keep x/y below ~0.07")
parser.add_argument("--object-mass", default="0.2", help="object mass [kg]")
parser.add_argument("--object-friction", default="1.2",
                    help="static=dynamic friction of the object's physics material. Low friction "
                         "is the usual reason a parallel gripper drops the part.")
parser.add_argument("--place-pose", default="0.55,0.35,0.0",
                    help="place target center x,y,z (m); z is overridden to sit on the table")
parser.add_argument("--place-size", default="0.12", help="place target marker edge [m]")
parser.add_argument("--randomize-object", action="store_true",
                    help="on /scene/reset_episode, re-sample the object position and yaw. "
                         "Demo diversity comes from this — a policy trained on one pose "
                         "only learns that pose.")
parser.add_argument("--randomize-radius", default="0.10",
                    help="+/- range [m] applied to object x and y when randomizing")
parser.add_argument("--seed", default="0", help="RNG seed for object randomization (reproducibility)")
# --- D5 fallback: hold the part with a FIXED JOINT instead of contact friction ---
parser.add_argument("--grasp-attach", action="store_true",
                    help="When the gripper closes on the object, weld it to the gripper with a USD "
                         "fixed joint (and release on open) instead of relying on contact physics. "
                         "This is the to_do.md D5 fallback. The 2F-85 is driven through PhysX mimic "
                         "joints; under contact load the fingers get forced PAST their limits "
                         "(finger_joint went to -1.17 rad) and the part is batted away instead of "
                         "held. Welding makes 'close the gripper -> the part comes along' true, "
                         "which is all the IL data pipeline needs.\n"
                         "*** Sim-only mechanism. It is INVISIBLE to a policy: the policy sees "
                         "images + joint states, which look the same as on the real robot where "
                         "real friction does the holding. Never put grasp state in the dataset. ***")
parser.add_argument("--grasp-close", default="0.25",
                    help="finger_joint [rad] above which the gripper counts as closing (attach arms)")
parser.add_argument("--grasp-release", default="0.15",
                    help="finger_joint [rad] below which the part is released")
parser.add_argument("--grasp-distance", default="0.09",
                    help="max distance [m] from the finger midpoint to the object centre for attach")
parser.add_argument("--grasp-link", default="wrist_3_link/gripper/Robotiq_2F_85/base_link",
                    help="USD prim PATH (relative to --robot-prim) of the link the object is welded "
                         "to. Must be a PATH, not a name: the USD link names differ from the URDF "
                         "ones AND the gripper's own base link is literally called 'base_link', "
                         "colliding with the robot's. Name lookup silently grabs the wrong prim.")
parser.add_argument("--grasp-tips", default="wrist_3_link/gripper/Robotiq_2F_85/left_inner_finger,"
                                            "wrist_3_link/gripper/Robotiq_2F_85/right_inner_finger",
                    help="comma-separated USD prim paths (relative to --robot-prim) of the two "
                         "finger pads; their midpoint is the grasp centre")
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

# ---- static external depth camera (optional) ------------------------------
# A camera FIXED in the world (base frame), overlooking the workspace, used by
# nvblox to build the obstacle ESDF for cuMotion. Unlike the eye-in-hand D405,
# it does NOT move with the arm, so the TSDF is stable and the arm is not the
# dominant thing in view -> a clean world map. Publishes depth + camera_info on
# /static_cam/depth/* with frame static_cam_depth_optical_frame; the matching
# base_link->static_cam_depth_optical_frame TF is published by the nvblox launch
# (ur16e_2f85_d405_nvblox.launch.py) -- keep the pose here and there in sync.
if args.with_static_cam:
    import omni.usd
    from pxr import Gf, UsdGeom, Vt

    SCAM_PRIM = "/World/static_cam"          # NOT under the robot -> world-fixed
    SCAM_W, SCAM_H = 640, 480
    _p = np.array([float(v) for v in args.static_cam_xyz.split(",")])
    _tg = np.array([float(v) for v in args.static_cam_target.split(",")])

    # ROS optical frame: z = view direction, x = image-right, y = image-down.
    _z = _tg - _p; _z = _z / np.linalg.norm(_z)
    _x = np.cross(_z, np.array([0.0, 0.0, 1.0])); _x = _x / np.linalg.norm(_x)
    _y = np.cross(_z, _x)
    _Ropt = np.column_stack([_x, _y, _z])            # optical axes in base frame
    # USD camera looks down -Z with +Y up: optical -> USD = rotate pi about X
    # (identical convention to the eye-in-hand block above).
    _Rx = np.array([[1, 0, 0], [0, -1, 0], [0, 0, -1]])
    _Rusd = _Ropt @ _Rx
    _M3s = Gf.Matrix3d(*[float(v) for v in _Rusd.T.flatten()])

    stage_obj = omni.usd.get_context().get_stage()
    scam = UsdGeom.Camera.Define(stage_obj, SCAM_PRIM)
    scam.GetFocalLengthAttr().Set(1.88)              # ~87 deg HFOV (D405-like)
    scam.GetHorizontalApertureAttr().Set(3.6)
    scam.GetVerticalApertureAttr().Set(2.7)
    scam.GetClippingRangeAttr().Set(Gf.Vec2f(0.05, 10.0))
    sxf = UsdGeom.Xformable(scam.GetPrim())
    sxf.ClearXformOpOrder()
    sxf.AddTransformOp().Set(Gf.Matrix4d(_M3s, Gf.Vec3d(float(_p[0]), float(_p[1]), float(_p[2]))))

    # small visible housing so the camera shows up in the GUI
    sbody = UsdGeom.Cube.Define(stage_obj, SCAM_PRIM + "/body")
    sbody.GetSizeAttr().Set(1.0)
    sbxf = UsdGeom.Xformable(sbody.GetPrim())
    sbxf.ClearXformOpOrder()
    sbxf.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, 0.04))
    sbxf.AddScaleOp().Set(Gf.Vec3f(0.06, 0.06, 0.05))
    sbody.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.05, 0.35, 0.55)]))
    simulation_app.update()

    try:
        og.Controller.edit(
            {"graph_path": "/StaticCamGraph", "evaluator_name": "execution"},
            {
                og.Controller.Keys.CREATE_NODES: [
                    ("OnTick", "omni.graph.action.OnPlaybackTick"),
                    ("Ctx", "isaacsim.ros2.bridge.ROS2Context"),
                    ("RP", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
                    ("Depth", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("DepthInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                    # RGB is what the IL/VLA policy actually consumes (video.exterior).
                    # Same render product as depth, so colour and depth are pixel
                    # aligned by construction here (a real D435/D455 has a small
                    # colour<->depth baseline; its driver publishes the offset).
                    ("RGB", "isaacsim.ros2.bridge.ROS2CameraHelper"),
                    ("ColorInfo", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
                ],
                og.Controller.Keys.CONNECT: [
                    ("OnTick.outputs:tick", "RP.inputs:execIn"),
                    ("RP.outputs:execOut", "Depth.inputs:execIn"),
                    ("RP.outputs:execOut", "DepthInfo.inputs:execIn"),
                    ("RP.outputs:execOut", "RGB.inputs:execIn"),
                    ("RP.outputs:execOut", "ColorInfo.inputs:execIn"),
                    ("RP.outputs:renderProductPath", "Depth.inputs:renderProductPath"),
                    ("RP.outputs:renderProductPath", "DepthInfo.inputs:renderProductPath"),
                    ("RP.outputs:renderProductPath", "RGB.inputs:renderProductPath"),
                    ("RP.outputs:renderProductPath", "ColorInfo.inputs:renderProductPath"),
                    ("Ctx.outputs:context", "Depth.inputs:context"),
                    ("Ctx.outputs:context", "DepthInfo.inputs:context"),
                    ("Ctx.outputs:context", "RGB.inputs:context"),
                    ("Ctx.outputs:context", "ColorInfo.inputs:context"),
                ],
                og.Controller.Keys.SET_VALUES: [
                    ("RP.inputs:cameraPrim", [usdrt.Sdf.Path(SCAM_PRIM)]),
                    ("RP.inputs:width", SCAM_W),
                    ("RP.inputs:height", SCAM_H),
                    ("Depth.inputs:type", "depth"),
                    ("Depth.inputs:topicName", "/static_cam/depth/image_rect_raw"),
                    ("Depth.inputs:frameId", "static_cam_depth_optical_frame"),
                    ("DepthInfo.inputs:topicName", "/static_cam/depth/camera_info"),
                    ("DepthInfo.inputs:frameId", "static_cam_depth_optical_frame"),
                    ("RGB.inputs:type", "rgb"),
                    ("RGB.inputs:topicName", "/static_cam/color/image_raw"),
                    ("RGB.inputs:frameId", "static_cam_color_optical_frame"),
                    ("ColorInfo.inputs:topicName", "/static_cam/color/camera_info"),
                    ("ColorInfo.inputs:frameId", "static_cam_color_optical_frame"),
                ],
            },
        )
        print(f"  static cam          : {SCAM_PRIM} @ {list(_p)} -> {list(_tg)} ({SCAM_W}x{SCAM_H})")
        print("  static cam topics   : /static_cam/color/image_raw, /static_cam/color/camera_info,")
        print("                        /static_cam/depth/image_rect_raw, /static_cam/depth/camera_info")
        print("  static cam TF       : ros2 launch ur_bringup static_cam_tf.launch.py "
              "(nvblox launch includes it; run it standalone for teleop/IL recording)")
    except Exception as e:
        carb.log_error(f"Failed to build static camera graph: {e}")

# ---- demo obstacle (optional) ---------------------------------------------
# A plain visible box in the workspace. The static camera renders it into depth,
# nvblox turns it into ESDF voxels, and cuMotion (read_esdf_world:=true) plans
# around it. No physics needed -- it's an obstacle the camera *sees*.
if args.obstacle:
    import omni.usd
    from pxr import Gf, UsdGeom, Vt

    OBS_PRIM = "/World/demo_obstacle"
    _op = np.array([float(v) for v in args.obstacle_pose.split(",")])
    _osz = np.array([float(v) for v in args.obstacle_size.split(",")])
    stage_obj = omni.usd.get_context().get_stage()
    obs = UsdGeom.Cube.Define(stage_obj, OBS_PRIM)
    obs.GetSizeAttr().Set(1.0)
    oxf = UsdGeom.Xformable(obs.GetPrim())
    oxf.ClearXformOpOrder()
    oxf.AddTranslateOp().Set(Gf.Vec3d(float(_op[0]), float(_op[1]), float(_op[2])))
    oxf.AddScaleOp().Set(Gf.Vec3f(float(_osz[0]), float(_osz[1]), float(_osz[2])))
    obs.GetDisplayColorAttr().Set(Vt.Vec3fArray([Gf.Vec3f(0.85, 0.2, 0.15)]))
    simulation_app.update()
    print(f"  demo obstacle       : {OBS_PRIM} @ {list(_op)} size {list(_osz)}")

# ---- pick&place scene (optional) ------------------------------------------
# Work surface + a graspable object + a place-target marker. Unlike --obstacle
# above (visual only, for nvblox), the object here has REAL PHYSICS: rigid body,
# collider, mass and a friction material, because the 2F-85 has to actually hold
# it. This is the scene the teleop -> IL demo collection runs in.
#
# We use isaacsim.core.api.objects (DynamicCuboid/FixedCuboid/VisualCuboid): one
# call gives rigid body + collision + mass + visual material, instead of hand
# applying the USD physics APIs.
#   NOTE: in Isaac Sim 6.0.1 `isaacsim.core.api` lives under extsDeprecated (it
#   still works and the rest of this script already depends on it). The successor
#   is isaacsim.core.experimental.objects -- migrate both together, not piecemeal.
scene_objects = {}
if args.scene == "pick_place":
    from isaacsim.core.api.objects import DynamicCuboid, FixedCuboid, VisualCuboid
    from isaacsim.core.api.materials import PhysicsMaterial

    _obj_p = np.array([float(v) for v in args.object_pose.split(",")])
    _obj_s = np.array([float(v) for v in args.object_size.split(",")])
    _plc_p = np.array([float(v) for v in args.place_pose.split(",")])
    _tbl_z = float(args.table_height)

    # Work surface. The robot base sits at z=0, so the table top is at z=table_height
    # (0.0 = the robot stands on the same plane the object rests on).
    # FixedCuboid = collider, no rigid body -> immovable, objects land on it.
    if args.table:
        scene_objects["table"] = FixedCuboid(
            prim_path="/World/work_table", name="work_table",
            position=np.array([0.55, 0.0, _tbl_z - 0.01]),
            scale=np.array([1.0, 1.2, 0.02]),
            color=np.array([0.35, 0.32, 0.30]),
        )

    # Friction matters more than anything else for whether the gripper holds.
    # These are deliberately generous; T2-1 tunes them against a real grasp test.
    _grip_mat = PhysicsMaterial(
        prim_path="/World/physics_materials/grasp_material",
        static_friction=float(args.object_friction),
        dynamic_friction=float(args.object_friction),
        restitution=0.0,
    )
    scene_objects["object"] = DynamicCuboid(
        prim_path="/World/pick_object", name="pick_object",
        position=_obj_p, scale=_obj_s,
        color=np.array([0.10, 0.45, 0.85]),
        mass=float(args.object_mass),
        physics_material=_grip_mat,
    )
    # Place target: visual only (no collider) so the arm can put the object down
    # onto it without fighting a phantom obstacle.
    scene_objects["place"] = VisualCuboid(
        prim_path="/World/place_target", name="place_target",
        position=np.array([_plc_p[0], _plc_p[1], _tbl_z + 0.001]),
        scale=np.array([float(args.place_size), float(args.place_size), 0.002]),
        color=np.array([0.15, 0.75, 0.25]),
    )
    print(f"  pick object         : /World/pick_object @ {list(_obj_p)} "
          f"size {list(_obj_s)} mass {args.object_mass}kg mu {args.object_friction}")
    print(f"  place target        : /World/place_target @ {list(_plc_p[:2])} (visual only)")

# physics must be initialized before the articulation can be driven
simulation_context.initialize_physics()
simulation_context.play()

# Start the arm at the HOME pose (arm up) instead of the all-zeros USD default
# (arm stretched horizontally) so the initial view is sane and clear of obstacles.
# ros2_control's reset_pose later commands the same home; this just fixes the
# pre-control startup pose. Defensive: never let this abort the sim.
try:
    from isaacsim.core.prims import SingleArticulation
    for _ in range(5):
        simulation_context.step(render=False)          # let the articulation register
    _art = SingleArticulation(ARTICULATION_ROOT)
    _art.initialize()
    _home = {"shoulder_pan_joint": 0.0, "shoulder_lift_joint": -1.5708, "elbow_joint": 0.0,
             "wrist_1_joint": 0.0, "wrist_2_joint": 0.0, "wrist_3_joint": 0.0}
    _names = list(_art.dof_names)
    _pos = _art.get_joint_positions()
    for _n, _v in _home.items():
        if _n in _names:
            _pos[_names.index(_n)] = _v
    from isaacsim.core.utils.types import ArticulationAction
    _art.set_joint_positions(_pos)                      # teleport to home
    _art.apply_action(ArticulationAction(joint_positions=_pos))   # hold under the drive
    print("  initial pose       : home (arm up)")
except Exception as _e:
    carb.log_warn(f"could not set initial home pose (continuing): {_e}")

print("=" * 70)
print("UR16e Isaac Sim ROS2 bridge running.")
print(f"  robot prim         : {ROBOT_PRIM}")
print(f"  articulation root  : {ARTICULATION_ROOT}")
print(f"  publishes states   : /{args.joint_states_topic}")
print(f"  subscribes commands: /{args.joint_commands_topic}")
print("  publishes clock    : /clock")
print("Verify the exact joint names with:  ros2 topic echo /%s --once" % args.joint_states_topic)

# OnPlaybackTick drives the graph every rendered step; just keep stepping.
# ---- ground-truth object pose + episode reset (pick_place scene only) -------
# GT pose is published ONLY as supervision/verification for us -- it tells the
# grasp test whether the object actually came up with the gripper, and lets a
# demo recorder auto-label episode success. It is NOT a policy input: the IL/VLA
# policy sees pixels + joint states, exactly the same set in sim and on the real
# robot (ur_bringup/docs/plan_il_vla.md 2.6). Keep it out of the dataset.
#
# The reset service is the sim half of the sim/real-common episode reset
# contract: the recorder calls the SAME service name on real hardware, where it
# instead prompts a human to re-place the part.
if args.scene == "pick_place":
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from std_srvs.srv import Trigger

        if not rclpy.ok():
            rclpy.init(args=[])
        _node = rclpy.create_node("isaac_scene")
        _pose_pub = _node.create_publisher(PoseStamped, "/scene/object_pose", 10)
        _obj = scene_objects["object"]
        _obj_home = np.array([float(v) for v in args.object_pose.split(",")])
        _rng = np.random.default_rng(int(args.seed))

        def _reset_episode(request, response):
            """Re-place the object for a new demo episode."""
            # Always let go first, else the part would be re-placed while still
            # welded to the gripper and get dragged around.
            try:
                _detach()
            except NameError:
                pass
            p = _obj_home.copy()
            if args.randomize_object:
                r = float(args.randomize_radius)
                p[0] += _rng.uniform(-r, r)
                p[1] += _rng.uniform(-r, r)
            yaw = _rng.uniform(-np.pi, np.pi) if args.randomize_object else 0.0
            quat = np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])  # w,x,y,z
            _obj.set_world_pose(position=p, orientation=quat)
            # Kill momentum, else the part keeps the velocity it had when grabbed.
            try:
                _obj.set_linear_velocity(np.zeros(3))
                _obj.set_angular_velocity(np.zeros(3))
            except Exception:
                pass
            response.success = True
            response.message = f"object at [{p[0]:.3f}, {p[1]:.3f}, {p[2]:.3f}] yaw {yaw:+.2f}"
            _node.get_logger().info(f"reset_episode: {response.message}")
            return response

        # ---- D5 fallback: weld the part to the gripper on close ------------
        # Reference pattern: isaacsim.robot_setup.assembler.robot_assembler
        # (_create_fixed_joint + set_opposite_body_transform). We inline the
        # local-frame maths instead of importing it, because that module is part
        # of a GUI extension and pulling it into a standalone script drags in the
        # editor stack.
        from pxr import Gf, Sdf, UsdGeom, UsdPhysics
        import omni.usd as _ou
        from std_msgs.msg import Bool

        _stage = _ou.get_context().get_stage()
        _JOINT_PATH = "/World/grasp_weld"
        # `manual` = attached through the service rather than by the gripper closing.
        # Without this the automatic release (finger_joint below the open threshold)
        # instantly undoes a service attach, because the fingers are still open.
        _grasp = {"active": False, "manual": False}
        _grasp_pub = _node.create_publisher(Bool, "/scene/grasp_active", 10)

        def _prim_at(rel):
            """Resolve a prim path given relative to the robot prim. PATHS, not names:
            USD link names differ from URDF ones and 'base_link' exists twice."""
            path = rel if rel.startswith("/") else f"{ROBOT_PRIM}/{rel.strip()}"
            pr = _stage.GetPrimAtPath(path)
            if not (pr and pr.IsValid()):
                _node.get_logger().error(
                    f"grasp: prim not found: {path}. Grasp welding will NOT work. "
                    "List the real names with: UsdPhysics.RigidBodyAPI prims under the robot.")
                return None
            return pr

        _grasp_link_prim = _prim_at(args.grasp_link)
        _tip_prims = [_prim_at(r) for r in args.grasp_tips.split(",")]
        _obj_prim = _stage.GetPrimAtPath("/World/pick_object")

        def _world_xf(prim):
            return UsdGeom.XformCache().GetLocalToWorldTransform(prim)

        def _tcp():
            """Point between the finger pads -- where the part must be to grip it."""
            tips = [p for p in _tip_prims if p and p.IsValid()]
            if len(tips) == 2:
                a = _world_xf(tips[0]).ExtractTranslation()
                b = _world_xf(tips[1]).ExtractTranslation()
                return np.array([(a[i] + b[i]) * 0.5 for i in range(3)])
            if _grasp_link_prim:
                t = _world_xf(_grasp_link_prim).ExtractTranslation()
                return np.array([t[0], t[1], t[2]])
            return None

        def _set_collision(enabled):
            """Objects welded to the gripper must stop colliding with it, else the
            fingers keep driving into the part and blow past their joint limits."""
            if not (_obj_prim and _obj_prim.IsValid()):
                return
            api = UsdPhysics.CollisionAPI.Get(_stage, _obj_prim.GetPath())
            if api:
                api.GetCollisionEnabledAttr().Set(bool(enabled))

        def _attach():
            if _grasp["active"] or not (_grasp_link_prim and _obj_prim and _obj_prim.IsValid()):
                return False
            j = UsdPhysics.FixedJoint.Define(_stage, _JOINT_PATH)
            jp = j.GetPrim()
            jp.GetRelationship("physics:body0").SetTargets([_grasp_link_prim.GetPath()])
            jp.GetRelationship("physics:body1").SetTargets([_obj_prim.GetPath()])
            # Preserve the CURRENT relative pose: express the object frame in the
            # gripper-link frame and put that on body0; body1 keeps identity.
            rel = _world_xf(_grasp_link_prim).GetInverse() * _world_xf(_obj_prim)
            t = rel.ExtractTranslation()
            q = rel.ExtractRotationQuat()
            j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in t]))
            j.CreateLocalRot0Attr().Set(Gf.Quatf(float(q.GetReal()), *[float(v) for v in q.GetImaginary()]))
            j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            _set_collision(False)
            _grasp["active"] = True
            _grasp["manual"] = False
            _node.get_logger().info("grasp: object WELDED to gripper (D5 fallback)")
            return True

        def _detach():
            if not _grasp["active"]:
                return False
            if _stage.GetPrimAtPath(_JOINT_PATH):
                _stage.RemovePrim(_JOINT_PATH)
            _set_collision(True)
            _grasp["active"] = False
            _grasp["manual"] = False
            _node.get_logger().info("grasp: object released")
            return True

        def _srv_attach(req, res):
            res.success = _attach()
            if res.success:
                # Held until /scene/detach_object; the automatic open-release rule
                # must not fire on an attach the caller asked for explicitly.
                _grasp["manual"] = True
            res.message = "welded (manual hold)" if res.success else "already grasped or prims missing"
            return res

        def _srv_detach(req, res):
            res.success = _detach()
            res.message = "released" if res.success else "nothing grasped"
            return res

        if args.grasp_attach:
            _node.create_service(Trigger, "/scene/attach_object", _srv_attach)
            _node.create_service(Trigger, "/scene/detach_object", _srv_detach)

        _g_close = float(args.grasp_close)
        _g_open = float(args.grasp_release)
        _g_dist = float(args.grasp_distance)

        def _grasp_step():
            """Attach when the gripper closes with the part between the pads.

            Automatic on purpose: during teleop the operator just squeezes the
            trigger, exactly as on the real robot. Making them call a service
            would distort demo timing and break sim/real parity.
            """
            if not args.grasp_attach:
                return
            try:
                fj = float(_art.get_joint_positions()[_names.index("finger_joint")])
            except Exception:
                return
            if not _grasp["active"] and fj >= _g_close:
                tcp, opos = _tcp(), _obj.get_world_pose()[0]
                if tcp is not None and float(np.linalg.norm(np.asarray(opos) - tcp)) <= _g_dist:
                    _attach()
            elif _grasp["active"] and not _grasp["manual"] and fj <= _g_open:
                _detach()
            _grasp_pub.publish(Bool(data=_grasp["active"]))

        _node.create_service(Trigger, "/scene/reset_episode", _reset_episode)
        print("  scene services      : /scene/reset_episode" +
              (", /scene/attach_object, /scene/detach_object" if args.grasp_attach else ""))
        if args.grasp_attach:
            print(f"  grasp weld (D5)     : ON  close>={_g_close} open<={_g_open} "
                  f"dist<={_g_dist}m link={args.grasp_link}")
        print("  scene topics        : /scene/object_pose (GT, verification only)")

        def _scene_spin():
            rclpy.spin_once(_node, timeout_sec=0.0)
            _grasp_step()
            pos, quat = _obj.get_world_pose()
            m = PoseStamped()
            m.header.stamp = _node.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in pos)
            m.pose.orientation.w = float(quat[0]); m.pose.orientation.x = float(quat[1])
            m.pose.orientation.y = float(quat[2]); m.pose.orientation.z = float(quat[3])
            _pose_pub.publish(m)
    except Exception as _e:
        carb.log_warn(f"scene ROS interface unavailable (continuing): {_e}")
        _scene_spin = None
else:
    _scene_spin = None

print("=" * 70, flush=True)

while simulation_app.is_running():
    simulation_context.step(render=True)
    if _scene_spin is not None:
        _scene_spin()

simulation_context.stop()
simulation_app.close()
