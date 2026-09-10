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
import math
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
# Position/size are arguments because RAISING the table makes them load-bearing.
# At the default height 0.0 the slab sits below the robot and its footprint never
# matters. Raise it to a realistic working height and a 1.0 x 1.2 m slab centred at
# x=0.55 intersects the arm at spawn: PhysX resolves the overlap by flinging the
# arm, wrist_2 wound out to -26 rad, and every plan then died with
# START_STATE_INVALID ("outside bounds"). Keep the slab clear of the robot's own
# footprint whenever table_height is non-zero.
parser.add_argument("--table-pose", default="0.55,0.0",
                    help="work table centre x,y (m) in the base frame")
parser.add_argument("--table-size", default="1.0,1.2",
                    help="work table size x,y (m). Shrink/push it out when raising the table, "
                         "so it does not overlap the robot")
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
# --- multi-object / multi-destination (language-conditioned tasks) ---
# A single object and a single destination make the language instruction REDUNDANT:
# ignoring it still gives the right answer, so a VLA learns to ignore it and you
# have paid 3B parameters for an ACT (plan_il_vla.md 2.8). Two objects x two
# destinations means the same observation maps to different actions depending on
# what was asked -- which is the whole point.
parser.add_argument("--object-names", default="block",
                    help="comma-separated object names. Each gets a colour from a fixed "
                         "palette and a home position spread along y from --object-pose. "
                         "Poses are published per object on /scene/objects/<name>/pose.")
parser.add_argument("--place-names", default="target",
                    help="comma-separated place-target names, spread along y from "
                         "--place-pose. Published on /scene/places/<name>/pose (latched).")
parser.add_argument("--object-spacing", default="0.14",
                    help="y spacing [m] between object homes (and between place targets)")
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
# The arm approaches, descends and releases FULLY OPEN (0), so these sit low:
# close above 0 by enough that the attach only arms once the demo really closes,
# release above 0 so opening back up always crosses it. 0.35/0.32 were needed
# only while grip_approach was 0.30, which was itself a workaround for the
# inverted-convention mistake (HISTORY.md 28).
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
parser.add_argument("--gripper-collision", default="convexDecomposition",
                    choices=["convexDecomposition", "convexHull", "none"],
                    help="collision approximation for the 2F-85 meshes. The stock asset ships\n                         convexHull, which FILLS the concave inner finger/knuckle and closes the\n                         jaw: measured, the gripper then cannot descend past the top of the part\n                         it is meant to grasp. 'none' leaves the asset alone")
parser.add_argument("--camera-res", default="640x480",
                    help="WxH for BOTH the eye-in-hand and the static camera. Lower is often\n                         better for IL: ACT feeds the dataset resolution straight into its\n                         ResNet, and GR00T N1.7 resizes everything to 256x256 anyway\n                         (N1_7_DEFAULT_IMAGE_TARGET_SIZE), so 640x480 is discarded there. It\n                         also decides whether DataLoader workers fit in a 64 MiB /dev/shm:\n                         2 cameras x batch 8 is 56 MiB at 640x480 but 14 MiB at 320x240.\n                         NOTE the static camera also feeds nvblox obstacle mapping -- keep\n                         640x480 for that, this is for IL collection")
parser.add_argument("--grasp-tcp-offset", type=float, default=0.1294,
                    help="[m] from the inner-finger PIVOT midpoint to the finger PADS, along "
                         "the tool axis. The readable links are the pivots, not the pads; this "
                         "is the URDF gripper_frame->finger_tip distance, measured from TF")
parser.add_argument("--grasp-debug", action="store_true",
                    help="log the finger-pad prim world poses and the resulting TCP every ~2 s. "
                         "Use this to compare Isaac's grasp TCP against the URDF finger-tip TF "
                         "at the SAME instant: the two are different links, and any offset "
                         "between them aims the whole descent wrong (CLAUDE.md pitfall 7)")
args, _ = parser.parse_known_args()

# The asset's finger_joint uses the SAME convention as the URDF and the real
# gripper: 0 = open, 0.8 = closed. Verified by photographing the wrist camera at
# each opening (HISTORY.md 28) after a previous session concluded the opposite
# and added a boundary inversion for it.
#
# That conclusion came from a derived number -- the separation between the two
# inner_finger LINK ORIGINS, which is 0 at one end of travel and 84.9 mm at the
# other. Link origins are not the visible geometry: the origins coincide while
# the fingers are wide apart. The same mistake, on finger_tip_link, had already
# cost 31 mm of aim earlier the same day. When the simulator renders images,
# check a geometric claim by looking at one.
GRIPPER_JOINT = "finger_joint"
GRAPH_STATES_TOPIC = args.joint_states_topic
GRAPH_COMMANDS_TOPIC = args.joint_commands_topic

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

# ---- gripper collision approximation ---------------------------------------
# The stock Robotiq_2F_85 colliders are all convexHull. The inner finger and the
# inner knuckle are concave (an L in profile), so their hulls fill the space
# BETWEEN the jaws. Measured consequence: the gripper stops dead ~150 mm above
# whatever it is descending onto -- i.e. at the part's top face -- so the pads
# never get around the part. A 35 mm cube gave zero pad overlap, a 20 mm cube
# only 7.6 mm, and the "successful" grasps were the proximity weld firing on a
# part the gripper was merely resting on. Real 2F-85 hardware picks a 50 mm block
# without trouble; this is an artefact of the approximation, not the mechanism.
if args.gripper_collision != "none":
    from pxr import UsdPhysics as _UP

    _st0 = stage.get_current_stage()
    _n_fixed = 0
    for _p in _st0.Traverse():
        if "Robotiq_2F_85" not in str(_p.GetPath()):
            continue
        if "PhysicsCollisionAPI" not in _p.GetAppliedSchemas():
            continue
        _a = _p.GetAttribute("physics:approximation")
        if not _a:
            _a = _UP.MeshCollisionAPI.Apply(_p).CreateApproximationAttr()
        _a.Set(args.gripper_collision)
        _n_fixed += 1
    print(f"  gripper collision   : {args.gripper_collision} on {_n_fixed} meshes")

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
                ("PublishJointState.inputs:topicName", GRAPH_STATES_TOPIC),
                ("SubscribeJointState.inputs:topicName", GRAPH_COMMANDS_TOPIC),
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
    CAM_W, CAM_H = [int(v) for v in args.camera_res.lower().split('x')]
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
    SCAM_W, SCAM_H = [int(v) for v in args.camera_res.lower().split('x')]
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
        _tbl_c = [float(v) for v in args.table_pose.split(",")]
        _tbl_s = [float(v) for v in args.table_size.split(",")]
        scene_objects["table"] = FixedCuboid(
            prim_path="/World/work_table", name="work_table",
            position=np.array([_tbl_c[0], _tbl_c[1], _tbl_z - 0.01]),
            scale=np.array([_tbl_s[0], _tbl_s[1], 0.02]),
            color=np.array([0.35, 0.32, 0.30]),
        )
        print(f"  work table          : top z={_tbl_z} centre {_tbl_c} size {_tbl_s}")

    # Friction matters more than anything else for whether the gripper holds.
    # These are deliberately generous; T2-1 tunes them against a real grasp test.
    _grip_mat = PhysicsMaterial(
        prim_path="/World/physics_materials/grasp_material",
        static_friction=float(args.object_friction),
        dynamic_friction=float(args.object_friction),
        restitution=0.0,
    )
    # Colours are how the language instruction identifies an object, so they must be
    # far apart in RGB -- a policy that has to tell "red" from "orange" is being asked
    # a perception question we did not intend to pose.
    _PALETTE = {
        "red":    (0.85, 0.12, 0.10),
        "blue":   (0.10, 0.35, 0.90),
        "yellow": (0.92, 0.85, 0.10),
        "green":  (0.15, 0.70, 0.25),
        "purple": (0.55, 0.15, 0.75),
    }
    _obj_names = [n.strip() for n in args.object_names.split(",") if n.strip()]
    _plc_names = [n.strip() for n in args.place_names.split(",") if n.strip()]
    _spacing = float(args.object_spacing)

    def _spread(base, i, n):
        """Lay n items out along y, centred on base."""
        return base + (i - (n - 1) / 2.0) * _spacing

    scene_objects["objects"] = {}
    _obj_homes = {}
    for i, nm in enumerate(_obj_names):
        home = np.array([_obj_p[0], _spread(_obj_p[1], i, len(_obj_names)), _obj_p[2]])
        _obj_homes[nm] = home
        scene_objects["objects"][nm] = DynamicCuboid(
            prim_path=f"/World/pick_object_{nm}", name=f"pick_object_{nm}",
            position=home, scale=_obj_s,
            color=np.array(_PALETTE.get(nm, (0.10, 0.45, 0.85))),
            mass=float(args.object_mass),
            physics_material=_grip_mat,
        )
        print(f"  object '{nm}'{'':<12.12} : /World/pick_object_{nm} @ {[round(float(v), 3) for v in home]} "
              f"colour {_PALETTE.get(nm, 'default')}")
    # Legacy single-object handle: everything written before multi-object support
    # (and the /scene/object_pose topic) keeps working on the FIRST object.
    scene_objects["object"] = scene_objects["objects"][_obj_names[0]]

    # Place targets: visual only (no collider) so the arm can put the object down
    # onto one without fighting a phantom obstacle.
    scene_objects["places"] = {}
    for j, nm in enumerate(_plc_names):
        pos = np.array([_plc_p[0], _spread(_plc_p[1], j, len(_plc_names)), _tbl_z + 0.001])
        scene_objects["places"][nm] = VisualCuboid(
            prim_path=f"/World/place_target_{nm}", name=f"place_target_{nm}",
            position=pos,
            scale=np.array([float(args.place_size), float(args.place_size), 0.002]),
            color=np.array([0.15, 0.75, 0.25]),
        )
        print(f"  place  '{nm}'{'':<12.12} : /World/place_target_{nm} @ {[round(float(v), 3) for v in pos[:2]]}")
    scene_objects["place"] = scene_objects["places"][_plc_names[0]]

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
# GT pose has two consumers, and the distinction matters:
#
#   1. SUPERVISION -- tells the grasp test whether the part actually came up with
#      the gripper, and lets the demo recorder auto-label episode success.
#   2. The pick&place STATE MACHINE's target (to_do.md D10, 2026-09-07). The
#      state machine only produces the trajectories the policy imitates, and the
#      policy never sees a pose, so using GT here cannot change the dataset --
#      it only avoids blocking data collection on the perception stack (M1/M2).
#
# It is NOT a policy input either way: the IL/VLA policy sees pixels + joint
# states, exactly the same set in sim and on the real robot
# (ur_bringup/docs/plan_il_vla.md 2.6). Keep it out of the dataset.
#
# Consumers must read the pose from the TOPIC only, never reach into the sim, so
# that a real perception node publishing the same type can be swapped in with a
# remap (/scene/object_pose -> /target/pose). That is the whole exit strategy.
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
        # Where the part is supposed to end up. It is a launch argument, so the
        # state machine would otherwise have to be told the same numbers twice --
        # and the two would drift apart the first time someone moves the marker.
        # Latched (transient-local) because it never changes during a run: a
        # consumer that starts late still gets it without waiting for a tick.
        from rclpy.qos import QoSProfile, DurabilityPolicy
        _place_pub = _node.create_publisher(
            PoseStamped, "/scene/place_pose",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        _place_size = float(args.place_size)
        # One topic per named object / destination. The language instruction picks
        # which pair the state machine is pointed at, so the topics must be
        # addressable by name -- and each stays a plain PoseStamped, so a real
        # perception node can still be remapped onto any of them.
        _obj_pubs = {nm: _node.create_publisher(
                         PoseStamped, f"/scene/objects/{nm}/pose", 10)
                     for nm in scene_objects["objects"]}
        _plc_pubs = {nm: _node.create_publisher(
                         PoseStamped, f"/scene/places/{nm}/pose",
                         QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
                     for nm in scene_objects["places"]}
        # The work surface, so the PLANNER can know about it. Without this the
        # table exists only in physics: cuMotion plans straight through it and the
        # arm is stopped by contact instead of routing around. Measured symptom --
        # a descent reported as "fraction 1.00 / SUCCEEDED" ending 50 mm off in y
        # with the part untouched, because a link was resting on the table.
        # Published as [cx, cy, cz, sx, sy, sz] (centre + full size, base frame) so
        # the geometry has ONE source of truth: the same numbers that spawned it.
        from std_msgs.msg import Float64MultiArray
        _table_pub = _node.create_publisher(
            Float64MultiArray, "/scene/table_box",
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
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
            # EVERY object is re-placed, not just the one this episode targets:
            # a distractor left where the last episode dropped it is a different
            # scene, and the policy would see the same instruction with an
            # inconsistent layout.
            msgs = []
            for nm, prim in scene_objects["objects"].items():
                p = _obj_homes[nm].copy()
                if args.randomize_object:
                    r = float(args.randomize_radius)
                    p[0] += _rng.uniform(-r, r)
                    p[1] += _rng.uniform(-r, r)
                yaw = _rng.uniform(-np.pi, np.pi) if args.randomize_object else 0.0
                quat = np.array([np.cos(yaw / 2), 0.0, 0.0, np.sin(yaw / 2)])  # w,x,y,z
                prim.set_world_pose(position=p, orientation=quat)
                # Kill momentum, else the part keeps the velocity it had when grabbed.
                try:
                    prim.set_linear_velocity(np.zeros(3))
                    prim.set_angular_velocity(np.zeros(3))
                except Exception:
                    pass
                msgs.append(f"{nm}@[{p[0]:.3f}, {p[1]:.3f}] yaw {yaw:+.2f}")
            response.success = True
            response.message = "; ".join(msgs)
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

        # Physics-backed reader for the two pads.
        #
        # UsdGeom.XformCache (what _tcp() used to use) reads the USD STAGE, but Isaac
        # writes simulated link poses to Fabric, not back to USD. The pads' authored
        # stage transforms are identity, so BOTH pads resolved to their common
        # ancestor and the "midpoint" collapsed onto the gripper BASE -- 98.3 mm above
        # the real pads. With grasp_distance 0.09 m that leaves a perfectly grasped
        # part 0.0983 m from the TCP: the weld could not fire, by 8 mm.
        #
        # SingleRigidPrim does NOT fix it either: a PhysX articulation link is not a
        # standalone rigid body, so its physics view never binds and it falls back to
        # the same broken USD path (measured: 0.1 mm pad separation). The articulation
        # view's get_link_transforms() is the API that actually reports link poses.
        _art_view = getattr(_art, "_articulation_view", None)
        _tip_link_idx = []
        try:
            _body_names = list(_art_view.body_names)
            for _r in args.grasp_tips.split(","):
                _tip_link_idx.append(_body_names.index(_r.strip().split("/")[-1]))
        except Exception as _e:
            _node.get_logger().error(
                f"grasp: cannot map the finger pads to articulation links "
                f"({type(_e).__name__}: {_e}). links={getattr(_art_view, 'body_names', None)}")
            _tip_link_idx = []
        # The articulation's own DOF names, which are NOT necessarily the ros2_control
        # ones -- the weld check indexes this list, so a mismatch disables grasping.
        print(f"  articulation DOFs   : {_names}")

        # Which object is welded is decided AT GRASP TIME by whichever one is
        # actually between the pads -- the gripper cannot know what the instruction
        # asked for, and welding the wrong one would silently fake a success.
        _obj_prims = {nm: _stage.GetPrimAtPath(f"/World/pick_object_{nm}")
                      for nm in scene_objects["objects"]}
        _held = {"name": None}

        def _obj_prim_now():
            nm = _held["name"]
            return _obj_prims.get(nm) if nm else None

        def _world_xf(prim):
            return UsdGeom.XformCache().GetLocalToWorldTransform(prim)

        def _pad_positions():
            """World positions of the two finger pads, straight from PhysX."""
            if len(_tip_link_idx) != 2:
                return None
            try:
                pv = _art_view._physics_view
                xf = pv.get_link_transforms()
                # The backend may be warp / torch / numpy depending on how the
                # simulation view was created, so normalise before indexing.
                if hasattr(xf, "numpy"):
                    xf = xf.numpy()
                xf = np.asarray(xf).reshape(pv.count, pv.max_links, 7)
                return [np.asarray(xf[0, i, 0:3], dtype=float) for i in _tip_link_idx]
            except Exception as e:
                if not _grasp.get("pad_read_warned"):
                    _grasp["pad_read_warned"] = True
                    _node.get_logger().error(f"grasp: cannot read pad poses ({type(e).__name__}: {e})")
                return None

        def _resolve_link(path, near=None):
            """Articulation link index for a USD prim path.

            Matching on the last path component ALONE is a trap here: the USD has
            two prims called base_link (the robot's and the gripper's), Isaac
            renames the second to base_link_0, and body_names.index("base_link")
            silently returns the ROBOT base a metre away. That made the tool axis
            point from the wrist to the robot base and the grasp attach use the
            wrong body. When several links match, pick the one physically nearest
            `near` -- the candidates are a metre apart, so this cannot be ambiguous.
            """
            last = path.strip().split("/")[-1]
            bn = list(_art_view.body_names)
            cands = [i for i, n in enumerate(bn)
                     if n == last or (n.startswith(last + "_") and n[len(last) + 1:].isdigit())]
            if not cands:
                raise KeyError(f"no articulation link matches {last!r}; links={bn}")
            if len(cands) > 1 and near is not None:
                xf = _link_xf_all()
                cands.sort(key=lambda i: float(np.linalg.norm(
                    np.asarray(xf[0, i, 0:3], dtype=float) - np.asarray(near, dtype=float))))
            return cands[0]

        def _link_xf_all():
            pv = _art_view._physics_view
            xf = pv.get_link_transforms()
            if hasattr(xf, "numpy"):
                xf = xf.numpy()
            return np.asarray(xf).reshape(pv.count, pv.max_links, 7)

        def _link_pose(idx):
            """(position, quaternion xyzw) of an articulation link, from PhysX."""
            xf = _link_xf_all()
            return (np.asarray(xf[0, idx, 0:3], dtype=float),
                    np.asarray(xf[0, idx, 3:7], dtype=float))

        def _quat_mat(q):
            """Rotation matrix from a quaternion given as (x, y, z, w)."""
            x, y, z, w = [float(v) for v in q]
            return np.array([
                [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])

        def _mat_quat(R):
            """(w, x, y, z) from a rotation matrix, via the numerically safe branch."""
            tr = float(R[0, 0] + R[1, 1] + R[2, 2])
            if tr > 0.0:
                s = math.sqrt(tr + 1.0) * 2.0
                return ((0.25 * s), (R[2, 1] - R[1, 2]) / s,
                        (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s)
            i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
            j_, k = (i + 1) % 3, (i + 2) % 3
            s = math.sqrt(max(1e-12, 1.0 + R[i, i] - R[j_, j_] - R[k, k])) * 2.0
            q = [0.0, 0.0, 0.0]
            q[i], q[j_], q[k] = 0.25 * s, (R[j_, i] + R[i, j_]) / s, (R[k, i] + R[i, k]) / s
            return ((R[k, j_] - R[j_, k]) / s, q[0], q[1], q[2])

        # ---- which gripper part is deepest over the part? ------------------
        # The URDF answers this by transforming each link's collision mesh by its
        # TF and taking the lowest point INSIDE the part's footprint. Isaac has to
        # be asked the same way, or the two are not comparable.
        #
        # Local geometry comes from the authored USD: every gripper link's Xform is
        # authored at identity (verified), so a link's authored mesh coordinates
        # ARE its coordinates in that link's own frame. Multiplying by the runtime
        # link transform gives the world geometry for whatever pose the joints are
        # in.
        #
        # Built lazily, because the link<->prim mapping needs _grasp_idx, which is
        # resolved further down. And it is built by INDEX: matching prims by name
        # alone maps the robot's base_link (body 0) onto the gripper's base_link
        # prim and leaves the real gripper base (body_names "base_link_0")
        # unmapped -- the same name collision that earlier made the tool axis point
        # at the robot base.
        _geom = {"local": None}

        def _build_link_geometry():
            from pxr import Usd as _U, UsdGeom as _UG
            bbc = _UG.BBoxCache(_U.TimeCode.Default(), [_UG.Tokens.default_])
            names = list(_art_view.body_names)
            prims = {}
            for pr in _stage.Traverse():
                if "Robotiq_2F_85" not in str(pr.GetPath()):
                    continue
                prims.setdefault(pr.GetName(), pr)
            out = {}
            for i, nm in enumerate(names):
                pr = prims.get(nm)
                if pr is None and i == _grasp_idx:
                    pr = prims.get("base_link")      # renamed to base_link_0 in the articulation
                if pr is None or (nm == "base_link" and i != _grasp_idx):
                    continue                          # body 0 is the ROBOT base
                r = bbc.ComputeWorldBound(pr).ComputeAlignedRange()
                lo, hi = r.GetMin(), r.GetMax()
                if lo[0] > hi[0]:
                    continue
                out[i] = np.array([[x, y, z] for x in (lo[0], hi[0])
                                   for y in (lo[1], hi[1]) for z in (lo[2], hi[2])], dtype=float)
            return out

        def _deepest_in_footprint(half=0.0175):
            """(depth below the gripper base, link name) for geometry over the part.

            Only geometry inside the part's footprint can touch a part centred
            under the tool -- the jaws at full open straddle it. Reporting the
            whole gripper's lowest point answers a different question.
            """
            if _grasp_idx is None:
                return None, None
            try:
                if _geom["local"] is None:
                    _geom["local"] = _build_link_geometry()
                    _node.get_logger().info(
                        f"link geometry: {len(_geom['local'])} gripper links mapped "
                        f"{sorted(_art_view.body_names[i] for i in _geom['local'])}")
                xf = _link_xf_all()
                bp = np.asarray(xf[0, _grasp_idx, 0:3], dtype=float)
                # Build the frame from the MEASURED tool axis, not from the gripper
                # base's local +z: that local axis is not the tool axis on this
                # asset (deriving the axis from it is what put the TCP 90 degrees
                # off earlier). Filtering the footprint in the wrong plane finds
                # nothing and reports "no interference" -- a silent false negative.
                zax = _tool_axis()
                if zax is None:
                    return None, None
                tmp = np.array([1.0, 0.0, 0.0])
                if abs(float(np.dot(tmp, zax))) > 0.9:
                    tmp = np.array([0.0, 1.0, 0.0])
                xax = np.cross(tmp, zax); xax /= np.linalg.norm(xax)
                yax = np.cross(zax, xax)
                bR = np.stack([xax, yax, zax], axis=1)      # columns = basis vectors
                bn = list(_art_view.body_names)
                best, who = None, None
                for i, local in _geom["local"].items():
                    p_i = np.asarray(xf[0, i, 0:3], dtype=float)
                    R_i = _quat_mat(np.asarray(xf[0, i, 3:7], dtype=float))
                    world = local @ R_i.T + p_i                 # link frame -> world
                    rel = (world - bp) @ bR                     # world -> gripper base
                    inside = rel[(np.abs(rel[:, 0]) <= half) & (np.abs(rel[:, 1]) <= half)]
                    if inside.size == 0:
                        continue
                    d = float(inside[:, 2].max())
                    if best is None or d > best:
                        best, who = d, bn[i]
                return best, who
            except Exception as e:
                if not _grasp.get("deep_warned"):
                    _grasp["deep_warned"] = True
                    _node.get_logger().error(f"footprint probe failed: {type(e).__name__}: {e}")
                return None, None

        def _tool_axis():
            """Unit vector along the tool axis, pointing AWAY from the wrist.

            Measured as wrist -> finger-pivot midpoint. Both come straight from the
            physics view, so this needs no assumption about which local axis of
            which link is "the tool axis" -- an assumption that was wrong twice
            (the gripper base's local +z is not the tool axis, and the link-name
            lookup grabbed the robot base).
            """
            try:
                pads = _pad_positions()
                if pads is None:
                    return None
                if _wrist_idx is None:
                    return None
                mid = (pads[0] + pads[1]) * 0.5
                wrist, _ = _link_pose(_wrist_idx)
                d = mid - wrist
                n = float(np.linalg.norm(d))
                return None if n < 5e-3 else d / n
            except Exception as e:
                if not _grasp.get("axis_warned"):
                    _grasp["axis_warned"] = True
                    _node.get_logger().error(f"grasp: cannot derive the tool axis "
                                             f"({type(e).__name__}: {e})")
                return None

        def _tcp():
            """Point between the finger PADS -- where the part must be to grip it.

            The two prims we can read are the inner-finger links, and a link's
            origin is its PIVOT, not its pad: measured, the pivots swing apart
            exactly like the fingers (0 -> 84.8 mm, matching the URDF) but their
            midpoint stays at pivot height, which is the gripper base. That put the
            TCP 98.3 mm above the part -- past the 0.09 m weld radius, so a
            correctly executed grasp still logged "nearest object beyond 0.09 m".
            grasp_tcp_offset walks that midpoint down the tool axis to the pads.
            """
            pads = _pad_positions()
            if pads is not None:
                mid = (pads[0] + pads[1]) * 0.5
                off = float(args.grasp_tcp_offset)
                if off:
                    axis = _tool_axis()
                    if axis is not None:
                        return mid + axis * off
                return mid
            if _grasp_link_prim:
                t = _world_xf(_grasp_link_prim).ExtractTranslation()
                return np.array([t[0], t[1], t[2]])
            return None

        # PROVE the TCP is at the pads, do not just assert it. The old check tested
        # prim.IsValid() only: both prims were valid, so it printed "finger pads"
        # while actually reporting the gripper base for both, and that lie is what
        # kept the real bug invisible. Two distinct pads must be physically APART --
        # ~135 mm with the gripper open. If they read as one point, the pose source
        # is broken and no weld can ever land, so say so at start-up.
        if args.grasp_debug:
            # Dump every articulation link with its physics-view pose. Guessing which
            # API is "the right one" has now failed twice; this shows what the links
            # actually are, in what order, and where they really sit.
            try:
                _pv = _art_view._physics_view
                _x = _pv.get_link_transforms()
                if hasattr(_x, "numpy"):
                    _x = _x.numpy()
                _x = np.asarray(_x)
                print(f"  link transforms     : raw shape {_x.shape}, "
                      f"count={_pv.count} max_links={_pv.max_links}")
                _x = _x.reshape(_pv.count, _pv.max_links, 7)
                print(f"  articulation links  : {len(_art_view.body_names)}")
                for _i, _bn in enumerate(_art_view.body_names):
                    print(f"      [{_i:2d}] {_bn:<34} {[round(float(v), 4) for v in _x[0, _i, 0:3]]}")
                print(f"  pad link indices    : {_tip_link_idx}")
            except Exception as _e:
                print(f"  link transform dump failed: {type(_e).__name__}: {_e}")

        # Resolve the two links the grasp needs, and SAY which ones were picked.
        # The gripper base is disambiguated from the robot base by proximity to the
        # finger pivots; if that ever picks wrong, this line shows it immediately
        # instead of the failure surfacing as an unexplained missed grasp.
        _grasp_idx = _wrist_idx = None
        try:
            _p0 = _pad_positions()
            _near = None if _p0 is None else (_p0[0] + _p0[1]) * 0.5
            _grasp_idx = _resolve_link(args.grasp_link, near=_near)
            _wrist_idx = _resolve_link(args.grasp_link.strip().split("/")[0], near=_near)
            _bn = list(_art_view.body_names)
            print(f"  grasp links         : gripper base [{_grasp_idx}] {_bn[_grasp_idx]}, "
                  f"wrist [{_wrist_idx}] {_bn[_wrist_idx]}")
        except Exception as _e:
            _node.get_logger().error(
                f"grasp: cannot resolve the grasp links ({type(_e).__name__}: {_e}); "
                "the TCP offset and the attach transform will both be wrong")

        _pads0 = _pad_positions()
        if _pads0 is None:
            print(f"  grasp TCP source    : *** FALLBACK to {args.grasp_link} *** "
                  "-- pad poses unreadable, welds will never fire")
        else:
            _sep = float(np.linalg.norm(_pads0[0] - _pads0[1]))
            # Coincident pads only prove the TCP is broken when the gripper is
            # OPEN. At boot the articulation sits at its raw zero, which under the
            # inversion is CLOSED -- pads legitimately touch. Checking blind here
            # printed "*** BROKEN ***" on a healthy gripper, which is exactly the
            # kind of lying diagnostic this check was added to replace.
            try:
                _fj0 = float(_art.get_joint_positions()[_names.index(GRIPPER_JOINT)])
            except Exception:
                _fj0 = None
            _shut = _fj0 is not None and _fj0 > 0.4   # rad; 0 = open, ~0.8 = closed
            if _sep < 0.02 and not _shut:
                print("  grasp TCP source    : *** BROKEN *** both pads report the same point "
                      f"(separation {_sep * 1000:.1f} mm) with the gripper open; TCP collapsed "
                      "onto their common ancestor, welds cannot fire")
                _node.get_logger().error(
                    f"grasp TCP is not at the finger pads: separation {_sep * 1000:.1f} mm at "
                    f"finger_joint={_fj0}. pads={[list(np.round(p, 4)) for p in _pads0]}")
            else:
                print(f"  grasp TCP source    : finger pads, separation {_sep * 1000:.1f} mm "
                      f"at finger_joint={'?' if _fj0 is None else round(_fj0, 3)} "
                      f"({'closed' if _shut else 'open'}) ({args.grasp_tips})")

        def _set_collision(enabled):
            """Objects welded to the gripper must stop colliding with it, else the
            fingers keep driving into the part and blow past their joint limits."""
            op = _obj_prim_now()
            if not (op and op.IsValid()):
                return
            api = UsdPhysics.CollisionAPI.Get(_stage, op.GetPath())
            if api:
                api.GetCollisionEnabledAttr().Set(bool(enabled))

        def _attach(name=None):
            if name is not None:
                _held["name"] = name
            op = _obj_prim_now()
            if (_grasp["active"] or _grasp_idx is None
                    or not (_grasp_link_prim and op and op.IsValid())):
                if not _grasp.get("attach_warned"):
                    _grasp["attach_warned"] = True
                    _node.get_logger().error(
                        f"attach refused: name={_held['name']} active={_grasp['active']} "
                        f"grasp_link={'ok' if _grasp_link_prim else 'MISSING'} "
                        f"grasp_link_idx={_grasp_idx} "
                        f"obj_prim={'ok' if (op and op.IsValid()) else 'MISSING'} "
                        f"known={list(_obj_prims)}")
                return False
            j = UsdPhysics.FixedJoint.Define(_stage, _JOINT_PATH)
            jp = j.GetPrim()
            jp.GetRelationship("physics:body0").SetTargets([_grasp_link_prim.GetPath()])
            jp.GetRelationship("physics:body1").SetTargets([op.GetPath()])
            # Preserve the CURRENT relative pose: express the object frame in the
            # gripper-link frame and put that on body0; body1 keeps identity.
            #
            # Both poses come from PHYSICS. This used to read _world_xf() (the USD
            # stage), which for a simulated link returns its authored pose, not
            # where it actually is -- so the joint was authored with a nonsense
            # relative transform and PhysX snapped the part somewhere else the
            # instant it welded (measured: the part jumped 0.6 m mid-lift and the
            # cycle then failed as "grasp_failed").
            gp, gq = _link_pose(_grasp_idx)
            # scene_objects holds the DynamicCuboid WRAPPERS (physics-backed
            # get_world_pose); _obj_prims holds the raw USD Prims, which are what
            # the joint needs for its body target. Mixing them up crashed the whole
            # sim mid-grasp and left the demo waiting on a gripper action forever.
            opos, oq = scene_objects["objects"][_held["name"]].get_world_pose()
            Rg = _quat_mat(gq)
            Ro = _quat_mat(np.asarray([oq[1], oq[2], oq[3], oq[0]], dtype=float))
            rel_R = Rg.T @ Ro
            rel_t = Rg.T @ (np.asarray(opos, dtype=float) - gp)
            rw, rx, ry, rz = _mat_quat(rel_R)
            j.CreateLocalPos0Attr().Set(Gf.Vec3f(*[float(v) for v in rel_t]))
            j.CreateLocalRot0Attr().Set(Gf.Quatf(float(rw), float(rx), float(ry), float(rz)))
            j.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
            j.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, 0.0, 0.0, 0.0))
            _set_collision(False)
            _grasp["active"] = True
            _grasp["manual"] = False
            _node.get_logger().info(
                f"grasp: '{_held['name']}' WELDED to gripper (D5 fallback)")
            return True

        def _detach():
            if not _grasp["active"]:
                return False
            if _stage.GetPrimAtPath(_JOINT_PATH):
                _stage.RemovePrim(_JOINT_PATH)
            _set_collision(True)
            _grasp["active"] = False
            _grasp["manual"] = False
            _node.get_logger().info(f"grasp: '{_held['name']}' released")
            _held["name"] = None
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

        def _rate(key, period=120):
            """True at most once per `period` physics steps (~2 s at 60 Hz).

            The weld diagnostics used to be once-per-run flags. That is useless
            here: any probe that squeezes the gripper before the real grasp burns
            the single allowed line, and the actual failure then logs nothing.
            """
            n = _grasp.get("step", 0)
            if n - _grasp.get(f"_t_{key}", -10 ** 9) < period:
                return False
            _grasp[f"_t_{key}"] = n
            return True

        def _dists(tcp):
            return {nm: round(float(np.linalg.norm(
                        np.asarray(pr.get_world_pose()[0]) - tcp)), 4)
                    for nm, pr in scene_objects["objects"].items()}

        def _grasp_step():
            """Attach when the gripper closes with the part between the pads.

            Automatic on purpose: during teleop the operator just squeezes the
            trigger, exactly as on the real robot. Making them call a service
            would distort demo timing and break sim/real parity.
            """
            if not args.grasp_attach:
                return
            _grasp["step"] = _grasp.get("step", 0) + 1
            try:
                _grasp_step_inner()
            except Exception:
                # This runs inside the physics callback: an exception here takes
                # Isaac down, and the demo then blocks forever on a gripper action
                # that will never complete (observed: a 17-minute silent hang).
                # Report it in full, once, and keep the sim alive.
                if not _grasp.get("step_warned"):
                    _grasp["step_warned"] = True
                    import traceback as _tb
                    _node.get_logger().error(
                        "grasp step raised; grasping is now unreliable:\n" + _tb.format_exc())

        def _grasp_step_inner():
            try:
                fj = float(_art.get_joint_positions()[_names.index("finger_joint")])
            except Exception as e:
                # This except used to `return` silently, and it HID the cause of
                # every weld failure: if the articulation has no DOF called
                # finger_joint, this function bails on every physics step, the weld
                # never fires, and all you see downstream is "grasp_failed".
                # Say it once, loudly, with the names that DO exist.
                if not _grasp.get("dof_warned"):
                    _grasp["dof_warned"] = True
                    _node.get_logger().error(
                        f"grasp DISABLED: cannot read finger_joint ({type(e).__name__}: {e}). "
                        f"articulation DOFs = {_names}")
                return
            if args.grasp_debug and _rate("dbg"):
                # Report the SAME source the weld uses. This used to print _world_xf()
                # (the broken USD path) next to a _tcp() computed another way, which
                # made the two look inconsistent for reasons that had nothing to do
                # with the bug.
                pads = _pad_positions()
                ws = None if pads is None else [[round(float(v), 4) for v in p] for p in pads]
                sep = None if pads is None else round(float(np.linalg.norm(pads[0] - pads[1])), 4)
                tcp = _tcp()
                deep, who = _deepest_in_footprint()
                _node.get_logger().info(
                    f"grasp debug: fj={fj:+.4f} sep={sep} tcp="
                    f"{None if tcp is None else [round(float(v), 4) for v in tcp]} "
                    f"| deepest over the part: "
                    f"{'n/a' if deep is None else f'{deep*1000:.1f} mm ({who})'}")
            if fj < -0.02 and _rate("neg"):
                # finger_joint driven BELOW its own lower limit (0.0) while being
                # commanded shut: the pads are being forced open by contact. This
                # is the signature of the grasp failing mechanically rather than
                # the weld logic refusing, and nothing else in the log shows it.
                tcp = _tcp()
                _node.get_logger().error(
                    f"finger forced open: fj={fj:.3f} (limit 0.0), tcp="
                    f"{None if tcp is None else [round(float(v), 3) for v in tcp]}"
                    f"{'' if tcp is None else f' distances={_dists(tcp)}'}")
            if not _grasp["active"] and fj >= _g_close:
                tcp = _tcp()
                if _rate("check"):
                    # Shows every input the weld decision uses. Rate-limited, not
                    # once-per-run, so the line that lands during the real grasp
                    # survives whatever squeezed the gripper earlier.
                    _node.get_logger().info(
                        f"weld check: fj={fj:.3f} >= {_g_close}, tcp="
                        f"{None if tcp is None else [round(float(v), 3) for v in tcp]}")
                if tcp is not None:
                    # Nearest object within reach wins. Picking the nearest rather
                    # than a fixed one is what makes "grasp whatever is actually in
                    # the gripper" true when several objects are on the table.
                    best, best_d = None, None
                    for nm, prim in scene_objects["objects"].items():
                        d = float(np.linalg.norm(
                            np.asarray(prim.get_world_pose()[0]) - tcp))
                        if d <= _g_dist and (best_d is None or d < best_d):
                            best, best_d = nm, d
                    if best is not None:
                        _attach(best)
                    elif _rate("skip"):
                        # The gripper closed far enough to weld but nothing was
                        # close enough to the TCP. Almost always means the TCP is
                        # wrong (see the FALLBACK note above), not that the aim was.
                        _node.get_logger().warn(
                            f"weld skipped: fj={fj:.3f} >= {_g_close} but nearest object "
                            f"is beyond {_g_dist} m. TCP={[round(float(v), 3) for v in tcp]} "
                            f"distances={_dists(tcp)}")
            elif _grasp["active"] and not _grasp["manual"] and fj <= _g_open:
                _detach()
            _grasp_pub.publish(Bool(data=_grasp["active"]))

        def _reset_gripper(request, response):
            """Force the gripper linkage back inside its limits.

            *** Nothing on the ROS side can do this. ***
            When the fingers close on a part that is wedged or off-centre, the
            reaction load drives the 2F-85 mimic linkage OUTSIDE its joint limits
            (measured finger_joint = -0.558 and -0.974 against a range of [0, 0.8];
            the same blow-out HISTORY.md 16 records). From there the joint is in an
            invalid physics state: commanding the gripper open does nothing, and
            detach + reset_episode does not help either -- both were tried and both
            failed. Unattended collection then dies on its first bad grasp.

            Teleporting the joints sidesteps physics entirely, which is the only
            thing that works. Zero the velocities too, or the linkage springs
            straight back out.
            """
            try:
                _detach()
            except NameError:
                pass
            try:
                pos = np.array(_art.get_joint_positions(), dtype=float)
                vel = np.array(_art.get_joint_velocities(), dtype=float)
                # ALL gripper joints to the asset's authored zero (= fully OPEN).
                # That pose is the one configuration the linkage is guaranteed to be
                # self-consistent in. Teleporting only the driven joint and leaving
                # the passive ones produced a physically impossible linkage
                # (measured: finger_joint 0.539 with the pads 91 mm apart).
                fixed = []
                for i, nm in enumerate(_names):
                    if "finger" in nm or "knuckle" in nm:
                        if abs(pos[i]) > 1e-4 or abs(vel[i]) > 1e-4:
                            fixed.append(f"{nm}={pos[i]:+.3f}")
                        pos[i] = 0.0
                        vel[i] = 0.0
                _art.set_joint_positions(pos)
                _art.set_joint_velocities(vel)
                response.success = True
                response.message = ("gripper joints zeroed: " + ", ".join(fixed)) if fixed \
                    else "gripper already inside limits"
            except Exception as e:
                response.success = False
                response.message = f"reset_gripper failed: {e}"
            _node.get_logger().info(f"reset_gripper: {response.message}")
            return response

        _node.create_service(Trigger, "/scene/reset_gripper", _reset_gripper)
        _node.create_service(Trigger, "/scene/reset_episode", _reset_episode)
        print("  scene services      : /scene/reset_episode, /scene/reset_gripper" +
              (", /scene/attach_object, /scene/detach_object" if args.grasp_attach else ""))
        if args.grasp_attach:
            print(f"  grasp weld (D5)     : ON  close>={_g_close} open<={_g_open} "
                  f"dist<={_g_dist}m link={args.grasp_link}")
        print("  scene topics        : /scene/object_pose (GT target, plan_il_vla.md 3.2),")
        print(f"                        /scene/place_pose (latched, marker edge {_place_size} m)")

        def _stamped(pos, quat=(1.0, 0.0, 0.0, 0.0)):
            m = PoseStamped()
            m.header.stamp = _node.get_clock().now().to_msg()
            m.header.frame_id = "base_link"
            m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in pos)
            m.pose.orientation.w = float(quat[0]); m.pose.orientation.x = float(quat[1])
            m.pose.orientation.y = float(quat[2]); m.pose.orientation.z = float(quat[3])
            return m

        # Publish the place target once; TRANSIENT_LOCAL replays it to late joiners.
        _place_pub.publish(_stamped(scene_objects["place"].get_world_pose()[0]))
        for nm, prim in scene_objects["places"].items():
            _plc_pubs[nm].publish(_stamped(prim.get_world_pose()[0]))
        if "table" in scene_objects:
            _tp = scene_objects["table"].get_world_pose()[0]
            _table_pub.publish(Float64MultiArray(
                data=[float(_tp[0]), float(_tp[1]), float(_tp[2]),
                      float(_tbl_s[0]), float(_tbl_s[1]), 0.02]))
            print(f"                        /scene/table_box (latched) "
                  f"centre {[round(float(v), 3) for v in _tp]} size {_tbl_s + [0.02]}")

        def _scene_spin():
            rclpy.spin_once(_node, timeout_sec=0.0)
            _grasp_step()
            for nm, prim in scene_objects["objects"].items():
                pos, quat = prim.get_world_pose()
                m = _stamped(pos, quat)
                _obj_pubs[nm].publish(m)
                if prim is _obj:                    # legacy single-object topic
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
