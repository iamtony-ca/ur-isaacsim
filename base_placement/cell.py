"""Cell description (YAML) -> targets, obstacles, mount candidates.

Everything is expressed in ONE cell frame (any right-handed frame you like, z up is
assumed only for `floor`/`ceiling` and for pedestals). See examples/confined_cell.yaml
for the full schema with comments.
"""
import numpy as np
import yaml

import ur16e_kin as kin

MOUNT_KINDS = ("floor", "ceiling", "wall_xmin", "wall_xmax", "wall_ymin", "wall_ymax")

# Rotation that takes the base z axis onto the mount surface normal (pointing INTO the cell).
MOUNT_R = {
    "floor": np.eye(3),
    "ceiling": np.diag([1.0, -1.0, -1.0]),                             # Rx(pi):    base z = -z
    "wall_xmin": np.array([[0, 0, 1], [0, 1, 0], [-1, 0, 0]], float),  # Ry(+pi/2): base z = +x
    "wall_xmax": np.array([[0, 0, -1], [0, 1, 0], [1, 0, 0]], float),  # Ry(-pi/2): base z = -x
    "wall_ymin": np.array([[1, 0, 0], [0, 0, 1], [0, -1, 0]], float),  # Rx(-pi/2): base z = +y
    "wall_ymax": np.array([[1, 0, 0], [0, 0, -1], [0, 1, 0]], float),  # Rx(+pi/2): base z = -y
}
# which cell axes the mount-plane coordinates (u, v) run along
MOUNT_UV = {"floor": "xy", "ceiling": "xy", "wall_xmin": "yz", "wall_xmax": "yz",
            "wall_ymin": "xz", "wall_ymax": "xz"}
# the cell face each mount kind is bolted to; it is always solid for that mount, even if
# `walls` marks the face open (else the arm could swing behind its own mounting plate)
MOUNT_FACE = {"floor": "zmin", "ceiling": "zmax", "wall_xmin": "xmin", "wall_xmax": "xmax",
              "wall_ymin": "ymin", "wall_ymax": "ymax"}
WALL_KEYS = ("xmin", "xmax", "ymin", "ymax", "zmin", "zmax")

# allowed keys per YAML mapping: a typo must fail loudly, not silently fall back to a default
_KEYS = {
    "": {"cell", "targets", "order", "cyclic", "robot", "tool", "mounts", "search", "scoring"},
    "cell": {"bounds", "walls", "obstacles"},
    "cell.bounds": {"lo", "hi"},
    "cell.walls": set(WALL_KEYS),
    "cell.obstacles[]": {"name", "lo", "hi"},
    "targets[]": {"name", "xyz", "approach", "xaxis", "rpy_deg", "quat_xyzw"},
    "mounts[]": {"name", "kind", "height", "u", "v"},
    "robot": {"base_radius", "base_height", "pedestal_radius", "mount_plate", "joint_limits_deg", "joint_velocity_deg"},
    "tool": {"set", "tcp", "pad_offset", "custom", "extra_spheres"},
    "tool.custom": {"xyz", "rpy_deg"},
    "tool.extra_spheres[]": {"name", "xyz", "r"},
    "search": {"grid_step", "yaw_step_deg", "refine_seeds", "top_k", "robustness"},
    "search.robustness": {"dxy", "dyaw_deg", "n_random"},
}


def _check_keys(d, where):
    if d is None:
        return
    if not isinstance(d, dict):
        raise ValueError(f"{where or 'top level'}: expected a mapping, got {type(d).__name__}")
    bad = sorted(set(d) - _KEYS[where])
    if bad:
        import difflib
        hints = {b: difflib.get_close_matches(b, _KEYS[where], 1) for b in bad}
        hint = ", ".join(f"{b!r}" + (f" (did you mean {h[0]!r}?)" if h else "") for b, h in hints.items())
        raise ValueError(f"{where or 'top level'}: unknown key(s) {hint}; allowed: {sorted(_KEYS[where])}")


def _validate_keys(C):
    _check_keys(C, "")
    for sec in ("cell", "robot", "tool", "search"):
        _check_keys(C.get(sec), sec)
    c = C.get("cell") or {}
    _check_keys(c.get("bounds"), "cell.bounds")
    _check_keys(c.get("walls"), "cell.walls")
    for o in c.get("obstacles") or []:
        _check_keys(o, "cell.obstacles[]")
    for t in C.get("targets") or []:
        _check_keys(t, "targets[]")
    for m in C.get("mounts") or []:
        _check_keys(m, "mounts[]")
    tool = C.get("tool") or {}
    if tool.get("custom") is not None:
        _check_keys(tool["custom"], "tool.custom")
    for s in tool.get("extra_spheres") or []:
        _check_keys(s, "tool.extra_spheres[]")
    _check_keys((C.get("search") or {}).get("robustness"), "search.robustness")
    for k, v in ((c.get("walls") or {}).items()):
        if not isinstance(v, bool):
            raise ValueError(f"cell.walls.{k} must be true/false (unquoted), got {v!r}")
    if "cyclic" in C and not isinstance(C["cyclic"], bool):
        raise ValueError(f"cyclic must be true/false (unquoted), got {C['cyclic']!r}")


def _Rz(a):
    c, s = np.cos(a), np.sin(a)
    R = np.zeros(np.shape(a) + (3, 3))
    R[..., 0, 0] = c
    R[..., 0, 1] = -s
    R[..., 1, 0] = s
    R[..., 1, 1] = c
    R[..., 2, 2] = 1
    return R


def target_pose(t):
    """One target dict -> 4x4 TCP pose in the cell frame.

    Orientation, one of:
      approach: [..] + xaxis: [..]   tool z (direction the fingers point) and tool x
      rpy_deg: [r, p, y]             URDF-style fixed-axis roll/pitch/yaw
      quat_xyzw: [x, y, z, w]
    With tool.tcp tool0 / gripper_frame / pad (no rotation vs tool0) and the 2F-85,
    tool x is the jaw open/close direction (the pads face +-x); tool y runs along the
    finger width. With tcp custom + rpy_deg that no longer holds.
    """
    p = np.array(t["xyz"], float)
    for key in ("approach", "xaxis", "quat_xyzw"):
        if key in t and np.linalg.norm(np.array(t[key], float)) < 1e-9:
            raise ValueError(f"target {t['name']}: {key} must not be a zero vector")
    if "approach" in t:
        z = np.array(t["approach"], float)
        z /= np.linalg.norm(z)
        x = np.array(t.get("xaxis", [1, 0, 0]), float)
        x -= z * (x @ z)
        if np.linalg.norm(x) < 1e-6:
            raise ValueError(f"target {t['name']}: xaxis is parallel to approach")
        x /= np.linalg.norm(x)
        R = np.stack([x, np.cross(z, x), z], 1)
    elif "rpy_deg" in t:
        R = kin.rpy_to_R(*np.radians(t["rpy_deg"]))
    elif "quat_xyzw" in t:
        R = kin.quat_xyzw_to_R(t["quat_xyzw"])
    else:
        raise ValueError(f"target {t['name']}: give approach+xaxis, rpy_deg or quat_xyzw")
    return kin.make_T(R, p)


class Cell:
    def __init__(self, path):
        with open(path, "rb") as f:
            raw = f.read()
        import hashlib
        self.path = path
        self.sha256 = hashlib.sha256(raw).hexdigest()
        C = yaml.safe_load(raw)
        if not isinstance(C, dict):
            raise ValueError("the cell YAML must be a mapping")
        _validate_keys(C)
        self.raw = C
        c = C["cell"]
        self.lo = np.array(c["bounds"]["lo"], float)
        self.hi = np.array(c["bounds"]["hi"], float)
        if self.lo.shape != (3,) or self.hi.shape != (3,):
            raise ValueError("cell.bounds lo/hi must be [x, y, z]")
        self.walls = {k: True for k in WALL_KEYS}
        self.walls.update(c.get("walls") or {})
        self.boxes = c.get("obstacles", []) or []
        self.box_names = [b["name"] for b in self.boxes]
        self.box_lo = np.array([b["lo"] for b in self.boxes], float).reshape(-1, 3)
        self.box_hi = np.array([b["hi"] for b in self.boxes], float).reshape(-1, 3)

        tl = C.get("targets") or []
        if not tl:
            raise ValueError("targets: at least one target is required")
        self.names = [t["name"] for t in tl]
        if len(set(self.names)) != len(self.names):
            raise ValueError("target names must be unique")
        self.T_tcp = np.array([target_pose(t) for t in tl])
        order = C.get("order")
        if order:
            unknown = [n for n in order if n not in self.names]
            if unknown:
                raise ValueError(f"order: unknown target(s) {unknown}; targets are {self.names}")
        self.order = list(range(len(tl))) if not order else [self.names.index(n) for n in order]
        if sorted(self.order) != list(range(len(tl))):
            raise ValueError("`order` must list every target exactly once")
        self.cyclic = bool(C.get("cyclic", True))

        r = C.get("robot", {}) or {}
        self.base_r = float(r.get("base_radius", 0.095))      # UR16e footprint dia 190 mm
        self.base_h = float(r.get("base_height", 0.0993))     # ur10e/collision/base.stl
        self.pedestal_r = float(r.get("pedestal_radius", 0.15))
        self.plate = float(r.get("mount_plate", 0.02))        # adapter plate for wall/ceiling
        self.limit_override_deg = r.get("joint_limits_deg", {}) or {}
        self.velocity_override_deg = r.get("joint_velocity_deg", {}) or {}
        for k, v in self.limit_override_deg.items():
            if not 0 < float(v) <= 360:
                raise ValueError(f"robot.joint_limits_deg.{k}: symmetric limit must be in (0, 360], got {v}")
        for k, v in self.velocity_override_deg.items():
            if not float(v) > 0:
                raise ValueError(f"robot.joint_velocity_deg.{k} must be > 0, got {v}")
        if min(self.base_r, self.base_h, self.pedestal_r) <= 0 or self.plate < 0:
            raise ValueError("robot base/pedestal sizes must be positive (mount_plate >= 0)")

        self.tool = C.get("tool", {}) or {}
        if int(self.tool.get("set", 2)) not in (1, 2, 3):
            raise ValueError(f"tool.set must be 1, 2 or 3, got {self.tool.get('set')}")
        self.mounts = C.get("mounts") or []
        if not self.mounts:
            raise ValueError("mounts: at least one mount candidate is required")
        if np.any(self.hi <= self.lo):
            raise ValueError(f"cell.bounds: hi {self.hi.tolist()} must exceed lo {self.lo.tolist()}")
        for kind, names in (("mount", [m["name"] for m in self.mounts]), ("obstacle", self.box_names)):
            dup = sorted({n for n in names if names.count(n) > 1})
            if dup:
                # results are keyed by name: a repeat would silently overwrite a mount's grid
                raise ValueError(f"{kind} names must be unique, repeated: {dup}")
        for b in self.boxes:
            if np.any(np.array(b["hi"], float) <= np.array(b["lo"], float)):
                raise ValueError(f"obstacle {b['name']}: hi must exceed lo")
        for m in self.mounts:
            if m["kind"] not in MOUNT_KINDS:
                raise ValueError(f"mount {m['name']}: kind must be one of {MOUNT_KINDS}")
            for axn in ("u", "v"):
                r = m.get(axn)
                if not isinstance(r, (list, tuple)) or len(r) != 2 or not float(r[0]) <= float(r[1]):
                    raise ValueError(f"mount {m['name']}: {axn} must be [lo, hi] with lo <= hi, got {r}")
            if m["kind"] != "floor" and float(m.get("height", 0.0)) != 0.0:
                raise ValueError(f"mount {m['name']}: height (pedestal) applies to floor mounts only")
            h = float(m.get("height", 0.0))
            if m["kind"] == "floor" and not 0.0 <= h < (self.hi[2] - self.lo[2]) - self.base_h:
                raise ValueError(f"mount {m['name']}: pedestal height {h} must be >= 0 and leave room for the base")
        self.search = C.get("search", {}) or {}
        self.scoring = C.get("scoring", {}) or {}

    # ------------------------------------------------------------- mounts ---
    def walls_for(self, mount):
        """Wall flags as seen by a robot on `mount`: its own mounting face is always solid."""
        w = dict(self.walls)
        w[MOUNT_FACE[mount["kind"]]] = True
        return w

    def mount_pose(self, mount, u, v, yaw):
        """Cell pose of base_link for mount candidates (u, v, yaw arrays).

        floor   : (u, v) = (x, y); base on a pedestal of height `height` (0 = on the floor)
        ceiling : (u, v) = (x, y); base hangs from an adapter plate under zmax
        wall_*  : (u, v) along the wall's in-plane axes (MOUNT_UV), plate on that wall
        yaw     : rotation about the base's own z (= J1) axis
        """
        kind = mount["kind"]
        u, v, yaw = np.broadcast_arrays(np.asarray(u, float), np.asarray(v, float), np.asarray(yaw, float))
        M = u.shape
        T = np.zeros(M + (4, 4))
        T[..., 3, 3] = 1
        T[..., :3, :3] = MOUNT_R[kind] @ _Rz(yaw)
        lo, hi, pl = self.lo, self.hi, self.plate
        full = lambda val: np.full(M, val)          # noqa: E731
        if kind == "floor":
            p = np.stack([u, v, full(lo[2] + float(mount.get("height", 0.0)))], -1)
        elif kind == "ceiling":
            p = np.stack([u, v, full(hi[2] - pl)], -1)
        elif kind == "wall_xmin":
            p = np.stack([full(lo[0] + pl), u, v], -1)
        elif kind == "wall_xmax":
            p = np.stack([full(hi[0] - pl), u, v], -1)
        elif kind == "wall_ymin":
            p = np.stack([u, full(lo[1] + pl), v], -1)
        else:  # wall_ymax
            p = np.stack([u, full(hi[1] - pl), v], -1)
        T[..., :3, 3] = p
        return T

    def pedestal(self, mount, T_wb):
        """(K,3) [cx, cy, h] of a floor pedestal column, or None."""
        h = float(mount.get("height", 0.0)) if mount["kind"] == "floor" else 0.0
        if h <= 0:
            return None
        return np.stack([T_wb[:, 0, 3], T_wb[:, 1, 3], np.full(len(T_wb), h)], 1)

    def footprint_points(self, mount, T_wb):
        """Points of the base housing (+ pedestal) that must be inside the cell and
        outside every obstacle box."""
        ang = np.linspace(0, 2 * np.pi, 24, endpoint=False)
        circ = np.stack([np.cos(ang), np.sin(ang), np.zeros_like(ang)], 1)
        pts_b = np.concatenate([circ * self.base_r, circ * self.base_r + [0, 0, self.base_h],
                                [[0, 0, 0], [0, 0, self.base_h]]])
        P = np.einsum("mij,pj->mpi", T_wb[:, :3, :3], pts_b) + T_wb[:, None, :3, 3]
        h = float(mount.get("height", 0.0)) if mount["kind"] == "floor" else 0.0
        if h > 0:
            zs = np.linspace(0, h, max(2, int(np.ceil(h / 0.05)) + 1))
            ped = (circ[None] * self.pedestal_r + np.stack([0 * zs, 0 * zs, zs + self.lo[2]], 1)[:, None]).reshape(-1, 3)
            Pp = ped[None] + np.stack([T_wb[:, 0, 3], T_wb[:, 1, 3], 0 * T_wb[:, 0, 3]], 1)[:, None]
            P = np.concatenate([P, Pp], 1)
        return P
