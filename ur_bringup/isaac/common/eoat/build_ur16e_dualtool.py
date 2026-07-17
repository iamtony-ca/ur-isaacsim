# SPDX-License-Identifier: Apache-2.0
"""Compose a single-articulation **UR16e + dual-tool EOAT** USD for the Isaac backend.

Generalises build_ur16e_2f85.py to our config-driven EOAT: it references the bare
UR16e and the EOAT assembly (emitted by build_eoat_usd.py), joins them into ONE
articulation rooted at the UR16e base, and drops the EOAT's standalone bits.

Step 1 (embed): reference UR16e onto /UR16e (its articulation root stays
                /UR16e/root_joint), reference the EOAT USD under the attach link,
                then STRIP the EOAT's own ArticulationRootAPI + PhysxArticulationAPI
                and its standalone world_fixed base joint (so it joins the UR
                articulation instead of being a second one — same fix build_ur16e_2f85
                applied to the Robotiq asset).
Step 2 (mount): author a fixed joint  <attach_link> -> EOAT tool0  at the flange
                (localRot reproduces the ROS tool0 frame, as for the 2F-85).
                Disable articulation self-collision (planner owns collisions).

The EOAT USD stays independently play-testable (verify_articulation.py); the
standalone bits are removed only in this embedded copy.

Run (Isaac bundled python):
    /isaac-sim/python.sh build_ur16e_dualtool.py \
        [--eoat-usd assets/ur16e_dualtool.usd] [--out assets/ur16e_dualtool_full.usd]
"""
from __future__ import annotations

import argparse
from pathlib import Path

_ap = argparse.ArgumentParser()
_ap.add_argument("--eoat-usd", default=None, help="EOAT assembly USD (default assets/ur16e_dualtool.usd)")
_ap.add_argument("--out", default=None, help="output USD (default assets/ur16e_dualtool_full.usd)")
_ap.add_argument("--attach-link", default="wrist_3_link", help="UR link the EOAT mounts on")
_ap.add_argument("--standoff", type=float, default=0.0, help="EOAT tool0 offset along wrist +Z (m)")
_ap.add_argument("--solver-pos-iters", type=int, default=32,
                 help="DEFAULT position-solver iters on the combined articulation (raise for heavy EOAT)")
_ap.add_argument("--solver-vel-iters", type=int, default=4, help="DEFAULT velocity-solver iters")
_ap.add_argument("--no-home", action="store_true",
                 help="do NOT bake the arm-up HOME pose (leave all-zeros horizontal USD default)")
_args = _ap.parse_args()

# Arm-up HOME pose (deg) baked as the articulation default so ANY loader (GUI /
# verify_articulation / a raw stage-open) starts CLEAR OF THE FLOOR — the all-zeros
# USD default is the arm stretched horizontal and can spawn in floor collision.
# Mirrors the runtime teleport in ur16e_isaac_ros2.py (_home, shoulder_lift=-90deg).
HOME_DEG = {"shoulder_pan_joint": 0.0, "shoulder_lift_joint": -90.0, "elbow_joint": 0.0,
            "wrist_1_joint": 0.0, "wrist_2_joint": 0.0, "wrist_3_joint": 0.0}

from isaacsim import SimulationApp  # noqa: E402
sim_app = SimulationApp({"headless": True})

import sys  # noqa: E402
from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics  # noqa: E402
try:
    from pxr import PhysxSchema  # noqa: E402
except Exception:
    PhysxSchema = None
from isaacsim.storage.native import get_assets_root_path  # noqa: E402

THIS_DIR = Path(__file__).resolve().parent
ASSETS = THIS_DIR.parent.parent / "assets"           # isaac/assets
EOAT_USD = Path(_args.eoat_usd).resolve() if _args.eoat_usd else ASSETS / "ur16e_dualtool.usd"
OUT = Path(_args.out).resolve() if _args.out else ASSETS / "ur16e_dualtool_full.usd"

# EOAT tool0 vs wrist_3_link mount rotation. IDENTITY: in this UR16e URDF the ROS
# tool0 == wrist_3_link (TF wrist_3->tool0 is identity) and the EOAT (damper) mounts
# at tool0, so the Isaac fixed joint must ALSO be identity to match RViz/URDF (함정 #7).
# A -90°Z here (inherited from the 2F-85 build) rotated the whole EOAT 90° in the
# played sim vs RViz (the fixed joint snaps the EOAT by that rotation on play) — the
# 2026-07-17 "EE 90° between Isaac and RViz" bug. Keep identity unless TF wrist_3->tool0
# gains a real rotation (then set this to that rotation).
ATTACH_LOCAL_ROT_WXYZ = (1.0, 0.0, 0.0, 0.0)


def info(m: str) -> None:
    print(f"[build-full] {m}", flush=True)


assets_root = get_assets_root_path()
if assets_root is None:
    info("FAILED to resolve Isaac assets root")
    sim_app.close(); sys.exit(1)
UR16E_URL = assets_root + "/Isaac/Robots/UniversalRobots/ur16e/ur16e.usd"
info(f"UR16e: {UR16E_URL}")
info(f"EOAT : {EOAT_USD}")

out = Usd.Stage.CreateInMemory()
UsdGeom.SetStageMetersPerUnit(out, 1.0)
UsdGeom.SetStageUpAxis(out, UsdGeom.Tokens.z)

root = out.DefinePrim("/UR16e", "Xform")
out.SetDefaultPrim(root)
root.GetReferences().AddReference(UR16E_URL)

attach_path = Sdf.Path("/UR16e").AppendChild(_args.attach_link)
eoat_path = attach_path.AppendChild("eoat")
eoat_prim = out.DefinePrim(eoat_path, "Xform")
eoat_prim.GetReferences().AddReference(str(EOAT_USD))
sim_app.update()

# strip EOAT standalone articulation bits + find its tool0 link
tool0_path = None
removed = []
for prim in out.Traverse():
    p = prim.GetPath()
    if not str(p).startswith(str(eoat_path)):
        continue
    if "PhysicsArticulationRootAPI" in prim.GetAppliedSchemas():
        prim.RemoveAPI(UsdPhysics.ArticulationRootAPI)
        if PhysxSchema:
            prim.RemoveAPI(PhysxSchema.PhysxArticulationAPI)
        removed.append(str(p))
    if prim.GetName() == "world_fixed" and prim.IsA(UsdPhysics.Joint):
        # NOTE: RemovePrim CANNOT delete a prim that comes from a reference (its spec
        # lives in the referenced EOAT layer, not this one) — it silently no-ops. We
        # deactivate here (recorded locally) AND purge post-flatten below. If left in,
        # world_fixed pins EOAT tool0 to the WORLD and fights the arm articulation
        # (disjointed-transform snap → the arm gets yanked / blows up).
        prim.SetActive(False)
        removed.append(str(p) + "(world_fixed)")
    if prim.GetName() == "tool0" and "PhysicsRigidBodyAPI" in prim.GetAppliedSchemas():
        tool0_path = p
info(f"  stripped from EOAT: {removed or '(none)'}")
info(f"  EOAT tool0 link: {tool0_path}")
if tool0_path is None:
    info("FAILED to locate EOAT tool0 link"); sim_app.close(); sys.exit(1)

# mount: fixed joint attach_link -> EOAT tool0
fj = UsdPhysics.FixedJoint.Define(out, attach_path.AppendChild("eoat_mount_joint"))
fj.CreateBody0Rel().SetTargets([str(attach_path)])
fj.CreateBody1Rel().SetTargets([str(tool0_path)])
w, x, y, z = ATTACH_LOCAL_ROT_WXYZ
fj.CreateLocalPos0Attr().Set(Gf.Vec3f(0.0, 0.0, float(_args.standoff)))
fj.CreateLocalRot0Attr().Set(Gf.Quatf(w, Gf.Vec3f(x, y, z)))
fj.CreateLocalPos1Attr().Set(Gf.Vec3f(0.0, 0.0, 0.0))
fj.CreateLocalRot1Attr().Set(Gf.Quatf(1.0, Gf.Vec3f(0.0, 0.0, 0.0)))
info(f"  mounted {attach_path} -> {tool0_path} (standoff={_args.standoff})")

# disable self-collision on the UR articulation root (planner owns collisions)
if PhysxSchema:
    for prim in out.Traverse():
        if "PhysicsArticulationRootAPI" in prim.GetAppliedSchemas():
            api = PhysxSchema.PhysxArticulationAPI.Apply(prim)
            api.CreateEnabledSelfCollisionsAttr(False)
            api.CreateSolverPositionIterationCountAttr(int(_args.solver_pos_iters))
            api.CreateSolverVelocityIterationCountAttr(int(_args.solver_vel_iters))
            info(f"  self-collision OFF + solver iters={_args.solver_pos_iters}/{_args.solver_vel_iters}"
                 f" on {prim.GetPath()}")

# bake HOME pose as the articulation default: set each arm revolute joint's initial
# JointState position (start AT home, no drive-in transient) + the drive target (hold
# there). Angular values are degrees per USD Physics. ros2_control re-commands home later.
if not _args.no_home:
    n_home = 0
    for prim in out.Traverse():
        nm = prim.GetName()
        if nm not in HOME_DEG or not prim.IsA(UsdPhysics.Joint):
            continue
        deg = HOME_DEG[nm]
        drive = UsdPhysics.DriveAPI.Get(prim, "angular") or UsdPhysics.DriveAPI.Apply(prim, "angular")
        drive.CreateTargetPositionAttr().Set(deg)
        if PhysxSchema:
            js = PhysxSchema.JointStateAPI.Apply(prim, "angular")
            js.CreatePositionAttr().Set(deg)
            js.CreateVelocityAttr().Set(0.0)
        n_home += 1
    info(f"  baked HOME pose (arm up, shoulder_lift=-90deg) on {n_home} arm joint(s)"
         if n_home else "  WARNING: no arm joints matched HOME_DEG (pose not baked)")

# flatten + clear instanceable + save
flat = Usd.Stage.Open(out.Flatten())
# Purge world_fixed on the flattened (now-local) stage — deactivation alone leaves an
# inactive spec; RemovePrim here fully deletes it so no stray joint pins tool0 to world.
purged = []
for prim in list(flat.Traverse()):
    if prim.GetName() == "world_fixed" and prim.IsA(UsdPhysics.Joint):
        purged.append(str(prim.GetPath()))
        flat.RemovePrim(prim.GetPath())
if purged:
    info(f"  purged post-flatten: {purged}")
for prim in flat.Traverse():
    if prim.IsInstanceable():
        prim.SetInstanceable(False)
OUT.parent.mkdir(parents=True, exist_ok=True)
flat.GetRootLayer().Export(str(OUT))

chk = Usd.Stage.Open(str(OUT))
n_art = sum(1 for p in chk.Traverse() if "PhysicsArticulationRootAPI" in p.GetAppliedSchemas())
n_rb = sum(1 for p in chk.Traverse() if "PhysicsRigidBodyAPI" in p.GetAppliedSchemas())
n_j = sum(1 for p in chk.Traverse() if p.IsA(UsdPhysics.Joint))
roots = [str(p.GetPath()) for p in chk.Traverse() if "PhysicsArticulationRootAPI" in p.GetAppliedSchemas()]
info(f"  ArticulationRoot={n_art}(expect 1) roots={roots}  RigidBody={n_rb}  Joint={n_j}")
info(f"  saved: {OUT} ({OUT.stat().st_size/1024:.0f} KB)")
sim_app.close()
info("done.")
