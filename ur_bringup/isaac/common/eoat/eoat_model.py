# SPDX-License-Identifier: Apache-2.0
"""Single source of truth: parse the EOAT yaml config into a link/joint graph
that BOTH the Isaac-USD emitter and the URDF/xacro emitter consume, so the sim
asset and the MoveIt2/cuMotion collision model can never drift (함정 #7).

Pure python — NO Isaac / USD dependency, so it loads instantly and is unit
testable. The one thing it needs from geometry (part thickness/size, to resolve
+Z stacking and approximate box inertia) it reads from the sidecar JSON that
normalize_part.py writes next to each normalized USD.

Graph produced:
    Link : one rigid body. id, geometry source, parent link, local pose in the
           parent frame (== the connecting joint's origin), resolved physics
           (mass + inertia + collision approx).
    Joint: name, type (fixed|revolute|prismatic), parent/child link, origin
           (xyz+rpy), axis, limit, drive, optional mimic.

Config schema (see eoat_dualtool.yaml for a worked example):
    meta.defaults: {density, collision}          # fallback physics for every part
    parts.<name>:
        cad: <file in cad_dir>
        normalize: {rpy_deg, seat, center_xy}    # used by normalize_part.py
        physics: {density | mass, com, inertia, collision, size}   # optional override
        sublinks: {<sub>: {cad?|geom_prim?, physics?}}   # internal DOF bodies (e.g. fingers)
        joints: [ {name, child, type, axis, origin_xyz, origin_rpy?,
                   limit:{lower,upper,effort?,velocity?}, drive:{stiffness,damping,target?},
                   mimic?} ]                      # parent defaults to the part's own link
    chain: [ {id?, part, parent, joint, gap?, mount:{xyz,rpy}?,
              axis?, limit?, drive?} ]            # inter-part tree (mostly fixed)

Run standalone to dump the resolved graph (pure python, no Isaac):
    python3 eoat_model.py eoat_dualtool.yaml
"""
from __future__ import annotations

import json
import math
import sys
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
import yaml

DEG = 1  # rpy stored in degrees throughout; emitters convert as needed


# --- transforms (ROS rpy convention R = Rz·Ry·Rx, shared by both emitters) -----
def _rot(rpy_deg):
    rx, ry, rz = (math.radians(float(v)) for v in rpy_deg)
    cx, sx, cy, sy, cz, sz = (math.cos(rx), math.sin(rx), math.cos(ry),
                              math.sin(ry), math.cos(rz), math.sin(rz))
    Rx = np.array([[1, 0, 0], [0, cx, -sx], [0, sx, cx]])
    Ry = np.array([[cy, 0, sy], [0, 1, 0], [-sy, 0, cy]])
    Rz = np.array([[cz, -sz, 0], [sz, cz, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def mat(xyz, rpy_deg):
    """xyz(m)+rpy(deg) -> 4x4 homogeneous (ROS convention)."""
    M = np.eye(4)
    M[:3, :3] = _rot(rpy_deg)
    M[:3, 3] = [float(v) for v in xyz]
    return M


def decompose(M):
    """4x4 -> (xyz m, rpy deg) in ROS convention — inverse of mat()."""
    R = M[:3, :3]
    sy = -max(-1.0, min(1.0, float(R[2, 0])))
    pitch = math.asin(sy)
    if abs(R[2, 0]) < 0.999999:                       # not gimbal-locked
        roll = math.atan2(float(R[2, 1]), float(R[2, 2]))
        yaw = math.atan2(float(R[1, 0]), float(R[0, 0]))
    else:
        roll = math.atan2(-float(R[1, 2]), float(R[1, 1]))
        yaw = 0.0
    xyz = [round(float(M[0, 3]), 6), round(float(M[1, 3]), 6), round(float(M[2, 3]), 6)]
    rpy = [round(math.degrees(roll), 4), round(math.degrees(pitch), 4), round(math.degrees(yaw), 4)]
    return xyz, rpy


@dataclass
class Physics:
    """Resolved mass properties for one rigid body. Numbers are FINAL here (both
    emitters use them verbatim), so density approximations happen once, in-model."""
    mass: float                       # kg
    com: list                         # [x,y,z] m, in the link frame
    inertia: list                     # [ixx,iyy,izz,ixy,ixz,iyz] kg·m² about com
    collision: str                    # convexHull | convexDecomposition | boundingCube | none
    density: float | None = None      # kg/m³ if mass was derived from density (informational)
    approx: bool = True               # True = box-from-bbox placeholder, False = supplied
    # fidelity (physics material) — DEFAULT values until measured/spec supplied
    friction: float = 0.8             # static=dynamic friction coeff (grasping-critical on fingers)
    restitution: float = 0.0          # bounciness (0 = inelastic)


@dataclass
class Link:
    id: str
    part: str | None                  # parts.<part> geometry key (None = massless frame)
    geom: str | None                  # normalized usd stem, or "part#prim" selection, or None
    parent: str
    xyz: list                         # local translate in parent frame (m)
    rpy: list                         # local rotate in parent frame (deg, XYZ)
    physics: Physics | None
    size: list | None = None          # [x,y,z] m (from sidecar) for reference/box inertia


@dataclass
class Joint:
    name: str
    jtype: str                        # fixed | revolute | prismatic
    parent: str
    child: str
    xyz: list
    rpy: list
    axis: list | None = None          # unit axis in child frame (revolute/prismatic)
    limit: dict | None = None         # {lower,upper,effort,velocity}
    drive: dict | None = None         # {stiffness,damping,target}
    mimic: str | None = None          # name of joint this one follows (parallel gripper)
    # fidelity — DEFAULT values until measured/spec supplied
    armature: float = 0.0             # reflected rotor inertia (motor_inertia × gear²)
    joint_friction: float = 0.0       # joint Coulomb friction


@dataclass
class Graph:
    name: str
    root_link: str
    cad_dir: Path
    norm_dir: Path
    out_usd: Path
    out_urdf: Path | None
    # articulation solver — DEFAULT values (raise position iters for heavy chains)
    solver_pos_iters: int = 32
    solver_vel_iters: int = 1
    links: list = field(default_factory=list)
    joints: list = field(default_factory=list)

    def link(self, lid: str) -> Link | None:
        return next((l for l in self.links if l.id == lid), None)


# --------------------------------------------------------------- physics ----
def _box_inertia(mass: float, size: list) -> list:
    """Solid-box inertia about the box centre — the density placeholder. Replaced
    verbatim when the mechanical team supplies real inertia."""
    x, y, z = (max(s, 1e-4) for s in size)
    ixx = mass * (y * y + z * z) / 12.0
    iyy = mass * (x * x + z * z) / 12.0
    izz = mass * (x * x + y * y) / 12.0
    return [ixx, iyy, izz, 0.0, 0.0, 0.0]


def _resolve_physics(spec: dict, defaults: dict, size: list | None) -> Physics:
    """Turn a (possibly empty) physics spec + bbox size into final mass props.
    Priority: explicit mass/inertia > density×bbox-volume box approx > defaults."""
    coll = spec.get("collision", defaults.get("collision", "convexHull"))
    size = spec.get("size", size) or [0.05, 0.05, 0.05]
    fric = float(spec.get("friction", defaults.get("friction", 0.8)))
    rest = float(spec.get("restitution", defaults.get("restitution", 0.0)))
    if "mass" in spec:                                    # supplied mass (real data path)
        mass = float(spec["mass"])
        com = spec.get("com", [0.0, 0.0, size[2] / 2.0])
        inertia = spec.get("inertia") or _box_inertia(mass, size)
        approx = "inertia" not in spec
        return Physics(mass, com, inertia, coll, None, approx, fric, rest)
    density = float(spec.get("density", defaults.get("density", 1000.0)))
    vol = size[0] * size[1] * size[2]                     # bbox volume (over-estimate)
    mass = max(density * vol, 1e-4)
    com = spec.get("com", [0.0, 0.0, size[2] / 2.0])      # box centre (parts are base@0, centred)
    return Physics(mass, com, _box_inertia(mass, size), coll, density, True, fric, rest)


# ----------------------------------------------------------------- build ----
def load_graph(cfg_path: Path) -> Graph:
    cfg = yaml.safe_load(Path(cfg_path).read_text())
    base = Path(cfg_path).parent
    meta = cfg["meta"]
    defaults = meta.get("defaults", {})
    cad_dir = (base / meta["cad_dir"]).resolve()
    norm_dir = (base / meta["out_normalized"]).resolve()
    g = Graph(
        name=meta["name"],
        root_link=meta["root_link"],
        cad_dir=cad_dir,
        norm_dir=norm_dir,
        out_usd=(base / meta["out_assembly"]).resolve(),
        out_urdf=(base / meta["out_urdf"]).resolve() if meta.get("out_urdf") else None,
    )
    g.solver_pos_iters = int(defaults.get("solver_position_iterations", 32))
    g.solver_vel_iters = int(defaults.get("solver_velocity_iterations", 1))
    jarm = float(defaults.get("joint_armature", 0.0))
    jfri = float(defaults.get("joint_friction", 0.0))
    parts = cfg["parts"]

    def sidecar(part: str) -> dict:
        p = norm_dir / f"{part}.json"
        return json.loads(p.read_text()) if p.exists() else {}

    # root frame (massless): UR tool0
    g.links.append(Link(g.root_link, None, None, g.root_link, [0, 0, 0], [0, 0, 0], None))
    thick = {g.root_link: 0.0}
    world = {g.root_link: np.eye(4)}   # each link's pose in the root_link (tool0) frame
    # optional UR TCP offset (TCP pose relative to tool0/flange). tcp_pose values are
    # given relative to THIS frame; omit -> TCP == tool0 (flange).
    _tcpo = meta.get("tcp_offset")
    tcp_frame = mat(_tcpo.get("xyz", [0, 0, 0]), _tcpo.get("rpy", [0, 0, 0])) if _tcpo else np.eye(4)

    for e in cfg["chain"]:
        part = e["part"]
        pid = e.get("id", part)
        parent = e["parent"]
        if not g.link(parent):
            raise ValueError(f"chain '{pid}': parent '{parent}' not defined yet (order)")
        sc = sidecar(part)
        size = sc.get("size")
        mount = e.get("mount")
        tcp = e.get("tcp_pose")                             # pose in root_link (tool0/TCP) frame
        if tcp:  # mechanical-team data path: absolute TCP pose -> back-compute parent-local mount
            child_w = tcp_frame @ mat(tcp.get("xyz", [0, 0, 0]), tcp.get("rpy", [0, 0, 0]))
            xyz, rpy = decompose(np.linalg.inv(world[parent]) @ child_w)
            world[pid] = child_w
        elif mount:                                        # explicit pose in parent frame (Y-branch / off-axis)
            xyz = [float(v) for v in mount.get("xyz", [0, 0, 0])]
            rpy = [float(v) for v in mount.get("rpy", [0, 0, 0])]
            world[pid] = world[parent] @ mat(xyz, rpy)
        else:                                              # +Z face-to-face stack on parent top
            xyz = [0.0, 0.0, thick[parent] + float(e.get("gap", 0.0))]
            rpy = [0.0, 0.0, 0.0]
            world[pid] = world[parent] @ mat(xyz, rpy)
        phys = _resolve_physics(parts[part].get("physics", {}), defaults, size)
        # collision/visual box: an explicit physics.size override wins over the
        # measured sidecar bbox (e.g. copick3d whose bbox includes the FOV cone).
        psize = parts[part].get("physics", {}).get("size")
        g.links.append(Link(pid, part, part, parent, xyz, rpy, phys, psize or size))
        thick[pid] = sc.get("thickness_z", 0.0)
        # connecting joint parent -> this link (structural, usually fixed)
        g.joints.append(Joint(
            name=e.get("joint_name", f"{pid}_joint"),
            jtype=e.get("joint", "fixed"), parent=parent, child=pid,
            xyz=xyz, rpy=rpy, axis=e.get("axis"), limit=e.get("limit"),
            drive=e.get("drive"), mimic=e.get("mimic"),
            armature=float(e.get("armature", jarm)),
            joint_friction=float(e.get("joint_friction", jfri))))

        # internal DOF of this part (e.g. 2FG14 fingers): sublinks + joints
        pspec = parts[part]
        for sub, subspec in (pspec.get("sublinks") or {}).items():
            sub_id = f"{pid}__{sub}" if pid != part else f"{part}__{sub}"
            sub_phys = _resolve_physics(subspec.get("physics", {}), defaults,
                                        subspec.get("physics", {}).get("size"))
            geom = subspec.get("cad") or subspec.get("geom_prim")
            # placed by its joint origin below; link local pose starts at identity
            g.links.append(Link(sub_id, subspec.get("part"), geom, pid,
                                [0, 0, 0], [0, 0, 0], sub_phys,
                                subspec.get("physics", {}).get("size")))
        for j in (pspec.get("joints") or []):
            child_id = f"{pid}__{j['child']}" if pid != part else f"{part}__{j['child']}"
            jparent = j.get("parent")
            jparent = (f"{pid}__{jparent}" if jparent and jparent != part else pid)
            jxyz = [float(v) for v in j.get("origin_xyz", [0, 0, 0])]
            jrpy = [float(v) for v in j.get("origin_rpy", [0, 0, 0])]
            # reflect joint origin into the child link's local pose (keeps USD nesting == URDF origin)
            cl = g.link(child_id)
            if cl:
                cl.xyz, cl.rpy = jxyz, jrpy
            mimic = j.get("mimic")
            if mimic:
                mimic = f"{pid}__{mimic}" if pid != part and "__" not in mimic else mimic
            g.joints.append(Joint(
                name=(f"{pid}__{j['name']}" if pid != part else j["name"]),
                jtype=j.get("type", "fixed"), parent=jparent, child=child_id,
                xyz=jxyz, rpy=jrpy, axis=j.get("axis"), limit=j.get("limit"),
                drive=j.get("drive"), mimic=mimic,
                armature=float(j.get("armature", jarm)),
                joint_friction=float(j.get("joint_friction", jfri))))
    return g


# ------------------------------------------------------------------ dump ----
def _dump(g: Graph) -> None:
    print(f"# graph: {g.name}  root={g.root_link}")
    print(f"#   {len(g.links)} links, {len(g.joints)} joints")
    print(f"#   out_usd  = {g.out_usd}")
    print(f"#   out_urdf = {g.out_urdf}")
    print("\n## LINKS")
    for l in g.links:
        if l.physics:
            p = l.physics
            ph = (f"m={p.mass*1000:7.1f}g coll={p.collision:16s}"
                  f"{'~box' if p.approx else ' real'} "
                  f"{'ρ='+str(int(p.density)) if p.density else ''}")
        else:
            ph = "(massless frame)"
        print(f"  {l.id:26s} <- {l.parent:22s} xyz={[round(v,4) for v in l.xyz]} "
              f"rpy={l.rpy}  {ph}")
    print("\n## JOINTS")
    for j in g.joints:
        extra = ""
        if j.jtype != "fixed":
            extra = f" axis={j.axis} limit={j.limit}" + (f" mimic<-{j.mimic}" if j.mimic else "")
        print(f"  {j.name:30s} {j.jtype:10s} {j.parent:22s} -> {j.child:26s}"
              f" xyz={[round(v,4) for v in j.xyz]} rpy={j.rpy}{extra}")


if __name__ == "__main__":
    _dump(load_graph(Path(sys.argv[1]).resolve()))
