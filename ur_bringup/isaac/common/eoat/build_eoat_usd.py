# SPDX-License-Identifier: Apache-2.0
"""Emit an Isaac-Sim PHYSICS ARTICULATION USD from the EOAT graph (eoat_model).

This is the USD emitter of the single-source pipeline: the same link/joint graph
that feeds the URDF emitter is authored here as a PhysX articulation so the sim
asset and the MoveIt2/cuMotion collision model stay identical (함정 #7).

Per graph Link  -> nested Xform with RigidBodyAPI + MassAPI(mass/com/inertia from
                   the model) + a `geo` child (referenced normalized USD, or a
                   placeholder Cube for geometry-less links) carrying collision.
Per graph Joint -> UsdPhysics Fixed/Revolute/Prismatic joint (body0=parent link,
                   body1=child link, localPos0/Rot0 = joint origin). Driven joints
                   get a DriveAPI (stiffness/damping/target) + joint limits.
                   `mimic` joints get PhysxMimicJointAPI best-effort (parallel
                   gripper); if that API is unavailable the joint still simulates
                   and the ROS/env layer can mirror the target (as oht_bolting does).

Collision approximation is per-link from the graph: convexHull / convexDecomposition
use MeshCollisionAPI on the referenced meshes; boundingCube authors ONE box sized
to the link's (possibly overridden) size — so parts like copick3d whose visual USD
includes an FOV cone still get a body-only collider.

The whole thing is ONE articulation rooted at the top prim; `--fix-base` (default)
pins tool0 to the world so it can be play-tested standalone. When mounted on the
UR16e the top-level fixed joint is dropped and tool0 is fixed-jointed to wrist_3.

Run (Isaac bundled python):
    /isaac-sim/python.sh build_eoat_usd.py eoat_dualtool.yaml [--no-fix-base] [--no-mimic]
"""
from __future__ import annotations

import argparse
import math
import os
import re
import sys
from pathlib import Path

import numpy as np

_ap = argparse.ArgumentParser()
_ap.add_argument("cfg")
_ap.add_argument("--no-fix-base", dest="fix_base", action="store_false",
                 help="do NOT pin tool0 to world (leave the articulation free-floating)")
_ap.add_argument("--physx-mimic", dest="physx_mimic", action="store_true", default=False,
                 help="ALSO couple mirror fingers with USD-native PhysxMimicJointAPI (finicky); "
                      "default drives both fingers so the ROS/MoveIt URDF <mimic> enforces the single DOF")
_args = _ap.parse_args()

sys.path.insert(0, str(Path(__file__).resolve().parent))
from eoat_model import load_graph, _rot  # noqa: E402

from isaacsim import SimulationApp  # noqa: E402
_app = SimulationApp({"headless": True})

from pxr import Gf, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade  # noqa: E402
try:
    from pxr import PhysxSchema  # noqa: E402
except Exception:
    PhysxSchema = None


def _pn(name: str) -> str:
    """USD-safe prim name (identifiers can't start with a digit, e.g. '2fg14')."""
    s = re.sub(r"[^A-Za-z0-9_]", "_", name)
    return s if (s[:1].isalpha() or s[:1] == "_") else "_" + s


def _quat_from_rpy(rpy_deg) -> Gf.Quatf:
    """rpy(deg) -> quat, ROS convention R = Rz·Ry·Rx (shared with URDF + eoat_model)."""
    return _mat_to_quatf(_rot(rpy_deg))


_AX = {0: "X", 1: "Y", 2: "Z"}
# 180° flip that reverses a token axis WITHOUT touching it (rotate about a
# perpendicular axis): applied equally to both joint frames so the rest config is
# unchanged but the DOF points the -ve way. Lets a -X finger keep positive limits
# and gearing=1, so both parallel jaws move symmetrically on the same +target.
_FLIP = {
    "X": Gf.Quatf(0.0, Gf.Vec3f(0.0, 1.0, 0.0)),   # 180° about Y: +X -> -X
    "Y": Gf.Quatf(0.0, Gf.Vec3f(1.0, 0.0, 0.0)),   # 180° about X: +Y -> -Y
    "Z": Gf.Quatf(0.0, Gf.Vec3f(1.0, 0.0, 0.0)),   # 180° about X: +Z -> -Z
}
_IDENT = Gf.Quatf(1.0, 0.0, 0.0, 0.0)


def _axis_token_sign(axis):
    """[vx,vy,vz] -> (token, sign). USD joint axis is a positive token; a negative
    component becomes a joint-frame flip (see _FLIP), not a limit sign flip."""
    i = max(range(3), key=lambda k: abs(axis[k]))
    return _AX[i], (-1.0 if axis[i] < 0 else 1.0)


def _mat_to_quatf(R) -> Gf.Quatf:
    """3x3 rotation (columns = axes) -> Gf.Quatf (Shepperd, convention-independent)."""
    t = R[0, 0] + R[1, 1] + R[2, 2]
    if t > 0:
        s = math.sqrt(t + 1.0) * 2.0
        w, x, y, z = 0.25 * s, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2.0
        w, x, y, z = (R[2, 1] - R[1, 2]) / s, 0.25 * s, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = math.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2.0
        w, x, y, z = (R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, 0.25 * s, (R[1, 2] + R[2, 1]) / s
    else:
        s = math.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2.0
        w, x, y, z = (R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, 0.25 * s
    return Gf.Quatf(float(w), Gf.Vec3f(float(x), float(y), float(z)))


def _diagonalize_inertia(I6):
    """[ixx,iyy,izz,ixy,ixz,iyz] -> (diagonalInertia Vec3f, principalAxes Quatf).
    USD MassAPI stores inertia as a diagonal + a principal-axes rotation, so a real
    CAD tensor with off-diagonal terms (rotated principal axes) is preserved here
    instead of being dropped. Box placeholders are already diagonal (identity axes)."""
    ixx, iyy, izz, ixy, ixz, iyz = (float(v) for v in I6)
    if abs(ixy) < 1e-12 and abs(ixz) < 1e-12 and abs(iyz) < 1e-12:
        return Gf.Vec3f(ixx, iyy, izz), Gf.Quatf(1.0, 0.0, 0.0, 0.0)
    M = np.array([[ixx, ixy, ixz], [ixy, iyy, iyz], [ixz, iyz, izz]], dtype=float)
    w, V = np.linalg.eigh(M)                 # eigenvalues ascending, V columns orthonormal
    if np.linalg.det(V) < 0:
        V[:, 0] = -V[:, 0]                   # ensure right-handed principal frame
    return Gf.Vec3f(float(w[0]), float(w[1]), float(w[2])), _mat_to_quatf(V)


def info(m: str) -> None:
    print(f"[build-eoat] {m}", flush=True)


g = load_graph(Path(_args.cfg).resolve())
info(f"graph {g.name}: {len(g.links)} links, {len(g.joints)} joints  -> {g.out_usd.name}")

stage = Usd.Stage.CreateNew(str(g.out_usd))
UsdGeom.SetStageMetersPerUnit(stage, 1.0)
UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)

top = UsdGeom.Xform.Define(stage, f"/{_pn(g.name)}")
stage.SetDefaultPrim(top.GetPrim())
UsdPhysics.ArticulationRootAPI.Apply(top.GetPrim())          # ONE articulation
if PhysxSchema:
    _artapi = PhysxSchema.PhysxArticulationAPI.Apply(top.GetPrim())
    # Robots/grippers disable self-collision and leave link-vs-link checking to the
    # motion planner (MoveIt/cuMotion). Without this, the placeholder finger colliders
    # sit inside the parent body's collision mesh and penetration-recovery overpowers
    # the finger drive. (Real broken-out CAD wouldn't overlap, but keep it off anyway.)
    _artapi.CreateEnabledSelfCollisionsAttr(False)
    # solver iteration counts (DEFAULT from config; raise pos iters for heavy chains)
    _artapi.CreateSolverPositionIterationCountAttr(int(g.solver_pos_iters))
    _artapi.CreateSolverVelocityIterationCountAttr(int(g.solver_vel_iters))

# physics-material cache: one UsdPhysics material per (friction,restitution) pair,
# bound to each link (grasping-critical friction lives here). DEFAULT values unless
# overridden per part in the config.
_MAT_SCOPE = f"/{_pn(g.name)}/PhysicsMaterials"
_mat_cache = {}


def _link_material(friction: float, restitution: float):
    key = (round(friction, 4), round(restitution, 4))
    if key not in _mat_cache:
        mp = f"{_MAT_SCOPE}/mat_{len(_mat_cache)}"
        mat = UsdShade.Material.Define(stage, mp)
        pm = UsdPhysics.MaterialAPI.Apply(mat.GetPrim())
        pm.CreateStaticFrictionAttr(float(friction))
        pm.CreateDynamicFrictionAttr(float(friction))
        pm.CreateRestitutionAttr(float(restitution))
        _mat_cache[key] = mat
    return _mat_cache[key]

# ---- links: nested Xform tree (parent path lookup), rigid body + mass + geo ----
path_of = {g.root_link: f"/{_pn(g.name)}/{_pn(g.root_link)}"}
# root (tool0) link
root_link = UsdGeom.Xform.Define(stage, path_of[g.root_link])
UsdPhysics.RigidBodyAPI.Apply(root_link.GetPrim())
rm = UsdPhysics.MassAPI.Apply(root_link.GetPrim())
rm.CreateMassAttr(0.05)                                       # nominal; base is fixed to world


def _add_collision(link_prim, geo_prim, phys, size):
    approx = phys.collision
    if approx == "none":
        return
    if approx == "boundingCube" or geo_prim is None:
        # explicit box collider sized to the (possibly overridden) body size,
        # centred at com — body-only, ignores visual extras like the FOV cone.
        sz = size or [0.05, 0.05, 0.05]
        cube = UsdGeom.Cube.Define(stage, link_prim.GetPath().AppendChild("collision"))
        cube.CreateSizeAttr(1.0)
        xf = UsdGeom.Xformable(cube)
        xf.AddTranslateOp().Set(Gf.Vec3d(*phys.com))
        xf.AddScaleOp().Set(Gf.Vec3f(sz[0], sz[1], sz[2]))
        UsdGeom.Imageable(cube).CreateVisibilityAttr("invisible")
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        return
    # convexHull / convexDecomposition: approximate each referenced mesh
    n = 0
    for prim in Usd.PrimRange(geo_prim):
        if prim.IsA(UsdGeom.Mesh):
            UsdPhysics.CollisionAPI.Apply(prim)
            mc = UsdPhysics.MeshCollisionAPI.Apply(prim)
            mc.CreateApproximationAttr(approx)
            n += 1
    if n == 0:  # nothing to approximate -> fall back to a box
        _add_collision(link_prim, None, phys._replace(collision="boundingCube") if hasattr(phys, "_replace") else phys, size)


for l in g.links:
    if l.id == g.root_link:
        continue
    parent_path = path_of[l.parent]
    lp = f"{parent_path}/{_pn(l.id)}"
    path_of[l.id] = lp
    link = UsdGeom.Xform.Define(stage, lp)
    # local pose in parent frame (initial state; joint localPos0 mirrors this)
    xf = UsdGeom.XformCommonAPI(link)
    xf.SetTranslate(Gf.Vec3d(*[float(v) for v in l.xyz]))
    xf.SetRotate(Gf.Vec3f(*[float(v) for v in l.rpy]))
    UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
    # geometry: referenced normalized USD, or placeholder cube for geometry-less links
    geo_prim = None
    if l.geom and "#" not in str(l.geom):
        npart = g.norm_dir / f"{l.geom}.usd"
        if npart.exists():
            geo_prim = stage.DefinePrim(f"{lp}/geo", "Xform")
            rel = os.path.relpath(str(npart), str(g.out_usd.parent))
            geo_prim.GetReferences().AddReference(assetPath=rel, primPath="/Part")
    if geo_prim is None:  # placeholder visual box (e.g. fingers until real CAD)
        sz = l.size or (l.physics.com and [0.03, 0.03, 0.05]) or [0.03, 0.03, 0.05]
        vis = UsdGeom.Cube.Define(stage, f"{lp}/geo")
        vis.CreateSizeAttr(1.0)
        vxf = UsdGeom.Xformable(vis)
        vxf.AddTranslateOp().Set(Gf.Vec3d(*l.physics.com))
        vxf.AddScaleOp().Set(Gf.Vec3f(*sz))
    # mass properties (pinned from the model so USD == URDF; full tensor preserved)
    p = l.physics
    mapi = UsdPhysics.MassAPI.Apply(link.GetPrim())
    mapi.CreateMassAttr(float(p.mass))
    mapi.CreateCenterOfMassAttr(Gf.Vec3f(*[float(c) for c in p.com]))
    diag, axes = _diagonalize_inertia(p.inertia)
    mapi.CreateDiagonalInertiaAttr(diag)
    mapi.CreatePrincipalAxesAttr(axes)
    _add_collision(link.GetPrim(), geo_prim, p, l.size)
    # bind friction/restitution material to this link's colliders
    UsdShade.MaterialBindingAPI.Apply(link.GetPrim())
    UsdShade.MaterialBindingAPI(link.GetPrim()).Bind(
        _link_material(p.friction, p.restitution),
        bindingStrength=UsdShade.Tokens.weakerThanDescendants, materialPurpose="physics")
    info(f"  link {l.id:26s} m={p.mass*1000:7.1f}g coll={p.collision} μ={p.friction}")

# ---- joints -----------------------------------------------------------------
drive_joint_paths = {}   # joint.name -> prim path (for mimic reference)
for j in g.joints:
    jp = Sdf.Path(path_of[j.child]).AppendChild(_pn(j.name))
    pos0 = Gf.Vec3f(*[float(v) for v in j.xyz])
    rot0 = _quat_from_rpy(j.rpy)
    rot1 = _IDENT
    if j.jtype == "fixed":
        joint = UsdPhysics.FixedJoint.Define(stage, jp)
    elif j.jtype in ("revolute", "prismatic"):
        joint = (UsdPhysics.RevoluteJoint if j.jtype == "revolute"
                 else UsdPhysics.PrismaticJoint).Define(stage, jp)
        token, sign = _axis_token_sign(j.axis or [0, 0, 1])
        joint.CreateAxisAttr(token)
        if sign < 0:                                         # -ve axis -> flip both joint frames
            rot0 = rot0 * _FLIP[token]
            rot1 = rot1 * _FLIP[token]
        if j.limit:                                          # limits stay positive
            joint.CreateLowerLimitAttr(float(j.limit.get("lower", 0.0)))
            joint.CreateUpperLimitAttr(float(j.limit.get("upper", 0.0)))
    else:
        info(f"  WARN unknown joint type '{j.jtype}' for {j.name} -> fixed")
        joint = UsdPhysics.FixedJoint.Define(stage, jp)
    joint.CreateBody0Rel().SetTargets([path_of[j.parent]])
    joint.CreateBody1Rel().SetTargets([path_of[j.child]])
    joint.CreateLocalPos0Attr().Set(pos0)
    joint.CreateLocalRot0Attr().Set(rot0)
    joint.CreateLocalPos1Attr().Set(Gf.Vec3f(0, 0, 0))
    joint.CreateLocalRot1Attr().Set(rot1)
    drive_joint_paths[j.name] = jp
    # fidelity on articulated joints: armature / joint friction / max velocity
    # (DEFAULT values from the model unless overridden per joint in config)
    if PhysxSchema and j.jtype in ("revolute", "prismatic"):
        try:
            pj = PhysxSchema.PhysxJointAPI.Apply(joint.GetPrim())
            pj.CreateArmatureAttr(float(j.armature))
            pj.CreateJointFrictionAttr(float(j.joint_friction))
            if j.limit and "velocity" in j.limit:
                pj.CreateMaxJointVelocityAttr(float(j.limit["velocity"]))
        except Exception as e:
            info(f"  PhysxJointAPI partial for {j.name}: {e}")
    # drive on the commanded joint (fingers): position PD
    if j.drive and j.jtype in ("revolute", "prismatic"):
        dtype = "angular" if j.jtype == "revolute" else "linear"
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), dtype)
        drive.CreateStiffnessAttr(float(j.drive.get("stiffness", 1000.0)))
        drive.CreateDampingAttr(float(j.drive.get("damping", 50.0)))
        drive.CreateTargetPositionAttr(float(j.drive.get("target", 0.0)))
        if j.limit and "effort" in j.limit:
            drive.CreateMaxForceAttr(float(j.limit["effort"]))
    info(f"  joint {j.name:30s} {j.jtype:9s} {j.parent} -> {j.child}")

# ---- mimic (parallel gripper): finger_right follows finger_left -------------
# Default: give the mirror joint its OWN drive (same PD as the reference) so it is
# actuated too — the single-DOF behaviour is enforced upstream (URDF <mimic> in
# MoveIt/ros2_control, or the env mirroring the target, as oht_bolting does).
# --physx-mimic ALSO adds USD-native PhysxMimicJointAPI coupling (version-finicky).
for j in g.joints:
    if not j.mimic:
        continue
    src = next((k for k in g.joints if k.name == j.mimic), None)
    mim_prim = stage.GetPrimAtPath(drive_joint_paths[j.name])
    dtype = "linear" if j.jtype == "prismatic" else "angular"
    drv = UsdPhysics.DriveAPI.Apply(mim_prim, dtype)
    if src and src.drive:
        drv.CreateStiffnessAttr(float(src.drive.get("stiffness", 1000.0)))
        drv.CreateDampingAttr(float(src.drive.get("damping", 50.0)))
        drv.CreateTargetPositionAttr(float(src.drive.get("target", 0.0)))
        if src.limit and "effort" in src.limit:
            drv.CreateMaxForceAttr(float(src.limit["effort"]))
    coupled = False
    if _args.physx_mimic and PhysxSchema and hasattr(PhysxSchema, "PhysxMimicJointAPI"):
        try:
            axis_token = "transX" if j.jtype == "prismatic" else "rotX"
            mj = PhysxSchema.PhysxMimicJointAPI.Apply(mim_prim, axis_token)
            mj.CreateReferenceJointRel().SetTargets([drive_joint_paths[j.mimic]])
            mj.CreateGearingAttr(1.0)
            mj.CreateOffsetAttr(0.0)
            coupled = True
        except Exception as e:
            info(f"  PhysxMimicJointAPI failed for {j.name}: {e}")
    info(f"  mimic {j.name} <- {j.mimic}  (drive-mirror{' + PhysxMimicJointAPI' if coupled else ''})")

# ---- optional fixed base (standalone play test) -----------------------------
if _args.fix_base:
    fb = UsdPhysics.FixedJoint.Define(stage, Sdf.Path(path_of[g.root_link]).AppendChild("world_fixed"))
    fb.CreateBody1Rel().SetTargets([path_of[g.root_link]])   # body0 empty = world
    info("  fixed base joint world -> tool0 (standalone). Drop this when mounting on UR16e.")

stage.GetRootLayer().Save()

# ---- sanity report ----------------------------------------------------------
chk = Usd.Stage.Open(str(g.out_usd))
n_art = sum(1 for p in chk.Traverse() if "PhysicsArticulationRootAPI" in p.GetAppliedSchemas())
n_rb = sum(1 for p in chk.Traverse() if "PhysicsRigidBodyAPI" in p.GetAppliedSchemas())
n_j = sum(1 for p in chk.Traverse() if p.IsA(UsdPhysics.Joint))
n_col = sum(1 for p in chk.Traverse() if "PhysicsCollisionAPI" in p.GetAppliedSchemas())
info(f"  ArticulationRoot={n_art}(expect 1)  RigidBody={n_rb}  Joint={n_j}  Collision={n_col}")
info(f"  saved: {g.out_usd} ({g.out_usd.stat().st_size/1024:.0f} KB)")

_app.close()
info("done.")
