# SPDX-License-Identifier: Apache-2.0
"""
Compose a single-articulation **UR16e + Robotiq 2F-85** USD for the Isaac Sim
backend.

The shipped Isaac UR16e USD has no gripper variant, and the Robotiq 2F-85 USD
(`Robotiq_2F_85_edit.usd`) ships as its own articulation. Referencing the
gripper under the arm wrist as-is fails (own ArticulationRootAPI, instanceable
visuals). This script bakes the fixes ONCE:

  Step 1 (fix-EE):  open the 2F-85 USD, flatten, drop instanceable, remove its
                    ArticulationRootAPI, set resetXformStack on rigid bodies.
                    -> assets/robotiq_2f85_fixed.usda
  Step 2 (compose): reference the bare UR16e onto the default prim, reference
                    the fixed gripper under wrist_3_link, and author a fixed
                    joint wrist_3_link -> gripper base_link so the whole thing
                    is ONE articulation rooted at the UR16e base.
                    -> assets/ur16e_with_2f85.usd

The 2F-85 has a single actuated DOF (`finger_joint`); the other 5 joints follow
via PhysxMimicJointAPI baked into the asset, so ROS only ever commands
`finger_joint`.

Run (Isaac Sim bundled python):
    /isaac-sim/python.sh \
        /isaac-sim/ur_ws/src/ur_bringup/isaac/build_ur16e_2f85.py
"""
from __future__ import annotations

import argparse
from pathlib import Path

# Parse before SimulationApp so --help is cheap. Defaults reproduce the original
# Set-2 asset (gripper flush on the flange, no coupling). Pass the Set-3 options
# to bake the GRP-ES-CPL-077 coupling + PickNik camera mount + gripper standoff
# into a separate USD so Isaac's EE matches the URDF/RViz exactly:
#   build_ur16e_2f85.py --out assets/ur16e_2f85_d405.usd --gripper-z 0.018 \
#       --camera-mount-usd assets/picknik_camera_adapter.usd \
#       --coupling-usd assets/ur_to_robotiq_coupling.usd --coupling-z 0.007
_ap = argparse.ArgumentParser()
_ap.add_argument("--out", default=None, help="output USD path (default assets/ur16e_with_2f85.usd)")
_ap.add_argument("--gripper-z", type=float, default=0.0,
                 help="gripper standoff along the tool axis (m); 0.011 coupling, 0.018 coupling+mount")
_ap.add_argument("--coupling-usd", default=None, help="coupling mesh USD to bake in (visual)")
_ap.add_argument("--coupling-z", type=float, default=0.0, help="coupling z offset along tool axis (m)")
_ap.add_argument("--camera-mount-usd", default=None, help="camera-mount mesh USD to bake in at the flange (visual)")
_args = _ap.parse_args()

from isaacsim import SimulationApp

sim_app = SimulationApp({"headless": True})

import sys  # noqa: E402

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402

THIS_DIR = Path(__file__).resolve().parent
ASSETS = THIS_DIR / "assets"
ASSETS.mkdir(exist_ok=True)

UR16E_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/5.1/Isaac/Robots/UniversalRobots/ur16e/ur16e.usd"
)
EE_URL = (
    "https://omniverse-content-production.s3-us-west-2.amazonaws.com"
    "/Assets/Isaac/5.1/Isaac/Robots/Robotiq/2F-85/Robotiq_2F_85_edit.usd"
)

EE_FIXED = ASSETS / "robotiq_2f85_fixed.usda"
COMPOSED = Path(_args.out).resolve() if _args.out else ASSETS / "ur16e_with_2f85.usd"

# Mount: attach gripper base to UR wrist_3_link. localPos0=(0,0,0) places the
# gripper base at the wrist_3_link origin (= flange face on the Isaac UR16e
# USD), localRot0 is the -90 deg about Z used by the known-correct Isaac UR +
# Robotiq coupling. Tune here if the gripper is visually rotated.
ATTACH_LINK = "wrist_3_link"
ATTACH_LOCAL_ROT_WXYZ = (0.7071068, 0.0, 0.0, -0.7071068)

# finger_joint drive: asset ships stiffness=3.0/damping=2e-4 which is sluggish
# for position commands. Bump for a crisp open/close demo.
DRIVE_JOINT = "finger_joint"
DRIVE_STIFFNESS = 20.0
DRIVE_DAMPING = 1.0


def info(m: str) -> None:
    print(f"[build-2f85] {m}", flush=True)


# ---------------------------------------------------------- Step 1: fix EE --
info(f"[1/2] opening EE: {EE_URL}")
src = Usd.Stage.Open(EE_URL)
if src is None:
    info("FAILED to open EE URL")
    sim_app.close()
    sys.exit(1)

work_layer = src.Flatten()
work = Usd.Stage.Open(work_layer)

n_inst = 0
for prim in work.Traverse():
    if prim.IsInstanceable():
        prim.SetInstanceable(False)
        n_inst += 1
info(f"  cleared instanceable on {n_inst} prim(s)")

removed_art = []
for prim in work.Traverse():
    if "PhysicsArticulationRootAPI" in prim.GetAppliedSchemas():
        prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        removed_art.append(str(prim.GetPath()))
info(f"  removed ArticulationRootAPI from: {removed_art or '(none)'}")

n_rb = 0
for prim in work.Traverse():
    if "PhysicsRigidBodyAPI" in prim.GetAppliedSchemas():
        xf = UsdGeom.Xformable(prim)
        if xf:
            xf.SetResetXformStack(True)
            n_rb += 1
info(f"  set resetXformStack on {n_rb} rigid body prim(s)")

work.GetRootLayer().Export(str(EE_FIXED))
info(f"  wrote {EE_FIXED}")


# ----------------------------------------------------- Step 2: composition --
info("[2/2] composing UR16e + 2F-85")
out = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(out, 1.0)
UsdGeom.SetStageUpAxis(out, UsdGeom.Tokens.z)

# Reference UR16e directly onto the default prim so links (root_joint,
# wrist_3_link, ...) are direct children — matching the bare UR16e layout the
# ur16e_isaac_ros2.py loader expects (robot-prim/root_joint articulation root).
root = out.DefinePrim("/UR16e", "Xform")
out.SetDefaultPrim(root)
root.GetReferences().AddReference(UR16E_URL)

wrist_path = Sdf.Path("/UR16e").AppendChild(ATTACH_LINK)
gripper_path = wrist_path.AppendChild("gripper")
gripper_prim = out.DefinePrim(gripper_path, "Xform")
gripper_prim.GetReferences().AddReference(str(EE_FIXED))

out.GetRootLayer().subLayerPaths  # noqa: B018 (force composition)
sim_app.update()

# Locate the gripper base_link + finger_joint in the composed stage.
base_link_path = None
finger_joint_path = None
for prim in out.Traverse():
    pth = prim.GetPath()
    if not str(pth).startswith(str(gripper_path)):
        continue
    if prim.GetName() == "base_link" and "PhysicsRigidBodyAPI" in prim.GetAppliedSchemas():
        base_link_path = pth
    if prim.GetName() == DRIVE_JOINT and prim.IsA(UsdPhysics.Joint):
        finger_joint_path = pth
info(f"  base_link    : {base_link_path}")
info(f"  finger_joint : {finger_joint_path}")
if base_link_path is None:
    info("FAILED to locate gripper base_link")
    sim_app.close()
    sys.exit(1)

# Fixed joint: wrist_3_link (arm articulation link) -> gripper base_link.
fj_path = wrist_path.AppendChild("gripper_fixed_joint")
fj = UsdPhysics.FixedJoint.Define(out, fj_path)
fj.CreateBody0Rel().SetTargets([str(wrist_path)])
fj.CreateBody1Rel().SetTargets([str(base_link_path)])
w, x, y, z = ATTACH_LOCAL_ROT_WXYZ
# localPos0 = standoff along the tool axis (wrist_3 +Z). The articulation solver
# places base_link there on play — same mechanism as localRot0 below, which the
# original (standoff 0) already relied on, so this stays consistent.
fj.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, float(_args.gripper_z)))
fj.CreateLocalRot0Attr().Set(Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z))))
fj.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
fj.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
info(f"  authored fixed joint {fj_path}: {wrist_path} -> {base_link_path}  (gripper_z={_args.gripper_z})")

# Optional VISUAL meshes baked under wrist_3_link (no physics, just decoration
# that moves with the wrist): the PickNik camera mount flush on the flange, and
# the GRP-ES-CPL-077 coupling on top of it. ROS tool0 frame == wrist_3 here, so
# they go in at identity. Order matches the URDF: flange -> mount(0) ->
# coupling(coupling_z) -> gripper(gripper_z).
for _vname, _vusd, _vz in [("camera_adapter", _args.camera_mount_usd, 0.0),
                           ("coupling", _args.coupling_usd, float(_args.coupling_z))]:
    if _vusd:
        _vp = out.DefinePrim(str(wrist_path.AppendChild(_vname)), "Xform")
        _vp.GetReferences().AddReference(str(Path(_vusd).resolve()))
        _vx = UsdGeom.Xformable(_vp)
        _vx.ClearXformOpOrder()
        _vx.AddTranslateOp().Set(Gf.Vec3d(0.0, 0.0, _vz))
        info(f"  baked visual {_vname} at z={_vz} from {_vusd}")

# Bump the finger_joint drive for responsive position control.
if finger_joint_path is not None:
    fjoint = out.GetPrimAtPath(finger_joint_path)
    if fjoint.HasAPI(UsdPhysics.DriveAPI, "angular"):
        drive = UsdPhysics.DriveAPI(fjoint, "angular")
    else:
        drive = UsdPhysics.DriveAPI.Apply(fjoint, "angular")
    drive.CreateStiffnessAttr().Set(DRIVE_STIFFNESS)
    drive.CreateDampingAttr().Set(DRIVE_DAMPING)
    drive.CreateTargetPositionAttr().Set(0.0)
    info(f"  set finger_joint drive stiffness={DRIVE_STIFFNESS} damping={DRIVE_DAMPING}")

info("  flattening")
flat_layer = out.Flatten()
flat = Usd.Stage.Open(flat_layer)
cleared = 0
for prim in flat.Traverse():
    if prim.IsInstanceable():
        prim.SetInstanceable(False)
        cleared += 1
info(f"  cleared instanceable on {cleared} prim(s) in flat output")
flat.GetRootLayer().Export(str(COMPOSED))
info(f"  wrote {COMPOSED} ({COMPOSED.stat().st_size / 1024:.0f} KB)")


# ---------------------------------------------------------- sanity report --
chk = Usd.Stage.Open(str(COMPOSED))
n_art = sum(1 for p in chk.Traverse() if "PhysicsArticulationRootAPI" in p.GetAppliedSchemas())
n_rbody = sum(1 for p in chk.Traverse() if "PhysicsRigidBodyAPI" in p.GetAppliedSchemas())
n_joint = sum(1 for p in chk.Traverse() if p.IsA(UsdPhysics.Joint))
n_mimic = sum(1 for p in chk.Traverse() for s in p.GetAppliedSchemas() if "Mimic" in s)
info(f"  ArticulationRoot={n_art} (expect 1)  RigidBody={n_rbody}  Joint={n_joint}  Mimic={n_mimic}")
art_roots = [str(p.GetPath()) for p in chk.Traverse()
             if "PhysicsArticulationRootAPI" in p.GetAppliedSchemas()]
info(f"  art root path(s): {art_roots}")

sim_app.close()
info("done.")
