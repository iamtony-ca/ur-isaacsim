"""UR16e kinematics for base-placement search (numpy only, vectorised).

Two models, on purpose:

* URDF model -- joint origins/axes parsed from the URDF that xacro generates from
  ur_description (ur_type:=ur16e) or from our set-2/3 sim xacros. This is the ground
  truth for every collision sphere and every reported pose.
* DH model   -- the classic UR DH table, used only for the closed-form IK
  (Hawkins 2013, "Analytic Inverse Kinematics for the Universal Robots UR-5/UR-10
  Arms", up to 8 branches; ROS-I ur_kinematics/ur_kin.cpp has the same UR16e block).
  Its frames are tied to the URDF by two constant transforms (`reconcile`):
      T(base_link -> tool0)(q) = Rz(pi) @ DH(q) @ I
  i.e. the DH base frame is the ROS-I `base` frame (= base_link rotated 180 deg about
  z, "X+ points backwards" in ur_macro.xacro) and DH frame 6 == tool0.

Measured (2026-10-16) against the xacro-generated UR16e URDF: FK max error 3e-10 m /
4e-8 rad, IK round-trip 3e-10 m, the generating q found among the returned branches in
99.8 % of random samples (the misses are ill-conditioned samples close to one of the
three singularities, where the closest branch is 1e-6..4e-6 rad off). `self_test.py`
repeats that check.
"""
import hashlib
import os
import subprocess
import tempfile
import xml.etree.ElementTree as ET

import numpy as np

# ---------------------------------------------------------------- DH (UR16e) ---
# ur_description/config/ur16e/default_kinematics.yaml (same values as UR's DH page)
D1, A2, A3, D4, D5, D6 = 0.1807, -0.4784, -0.36, 0.17415, 0.11985, 0.11655
DH_D = np.array([D1, 0, 0, D4, D5, D6])
DH_A = np.array([0, A2, A3, 0, 0, 0])
DH_ALPHA = np.array([np.pi / 2, 0, 0, np.pi / 2, -np.pi / 2, 0])

ARM_JOINTS = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
              "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
ARM_LINKS = ["shoulder_link", "upper_arm_link", "forearm_link",
             "wrist_1_link", "wrist_2_link", "wrist_3_link"]

UR_XACRO = "/opt/ros/jazzy/share/ur_description/urdf/ur.urdf.xacro"
_SRC = os.path.normpath(os.path.join(os.path.dirname(os.path.abspath(__file__)), ".."))
_WS = os.path.dirname(_SRC)                          # workspace root (src/..)
SET_XACRO = {
    1: UR_XACRO,
    2: os.path.join(_SRC, "ur_bringup/urdf/ur16e_2f85/ur16e_2f85_sim.urdf.xacro"),
    3: os.path.join(_SRC, "ur_bringup/urdf/ur16e_2f85_d405/ur16e_2f85_d405_sim.urdf.xacro"),
}
# Tuned UR16e + 2F-85 sphere set already used by cuMotion in this workspace.
XRDF = os.path.join(_SRC, "ur_bringup/cumotion/ur16e_2f85.xrdf")


# ------------------------------------------------------------ small helpers ---
def rpy_to_R(r, p, y):
    """URDF convention: fixed-axis roll-pitch-yaw, R = Rz(y) Ry(p) Rx(r)."""
    cr, sr, cp, sp, cy, sy = np.cos(r), np.sin(r), np.cos(p), np.sin(p), np.cos(y), np.sin(y)
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    return Rz @ Ry @ Rx


def R_to_rpy(R):
    """Inverse of rpy_to_R (URDF rpy), gimbal-safe."""
    sp = -R[2, 0]
    if abs(sp) > 1 - 1e-9:
        y = np.arctan2(-R[0, 1], R[1, 1])
        return 0.0, np.sign(sp) * np.pi / 2, y
    return np.arctan2(R[2, 1], R[2, 2]), np.arcsin(sp), np.arctan2(R[1, 0], R[0, 0])


def R_to_quat_wxyz(R):
    """Rotation matrix -> unit quaternion [w, x, y, z] with w >= 0 (Shepperd's method).

    The earlier "|component| from the diagonal + copysign from the off-diagonal
    differences" shortcut is wrong for every 180-degree rotation (all differences are
    0, so all signs come out +). Ceiling mounts are Rx(pi) Rz(yaw), i.e. 180-degree
    rotations, so that bug turned the Isaac orientation and the MoveIt scene by up to
    180 deg (found 2026-10-16). Branch on the largest of (trace, R00, R11, R22) instead.
    """
    R = np.asarray(R, float)
    t = np.trace(R)
    k = int(np.argmax([t, R[0, 0], R[1, 1], R[2, 2]]))
    if k == 0:
        s = 2 * np.sqrt(max(0.0, 1 + t))
        q = [s / 4, (R[2, 1] - R[1, 2]) / s, (R[0, 2] - R[2, 0]) / s, (R[1, 0] - R[0, 1]) / s]
    elif k == 1:
        s = 2 * np.sqrt(max(0.0, 1 + R[0, 0] - R[1, 1] - R[2, 2]))
        q = [(R[2, 1] - R[1, 2]) / s, s / 4, (R[0, 1] + R[1, 0]) / s, (R[0, 2] + R[2, 0]) / s]
    elif k == 2:
        s = 2 * np.sqrt(max(0.0, 1 - R[0, 0] + R[1, 1] - R[2, 2]))
        q = [(R[0, 2] - R[2, 0]) / s, (R[0, 1] + R[1, 0]) / s, s / 4, (R[1, 2] + R[2, 1]) / s]
    else:
        s = 2 * np.sqrt(max(0.0, 1 - R[0, 0] - R[1, 1] + R[2, 2]))
        q = [(R[1, 0] - R[0, 1]) / s, (R[0, 2] + R[2, 0]) / s, (R[1, 2] + R[2, 1]) / s, s / 4]
    q = np.array(q)
    q /= np.linalg.norm(q)
    return -q if q[0] < 0 else q


def quat_xyzw_to_R(q):
    x, y, z, w = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([[1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
                     [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
                     [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


def make_T(R=None, p=None):
    T = np.eye(4)
    if R is not None:
        T[:3, :3] = R
    if p is not None:
        T[:3, 3] = p
    return T


def rotz_b(q):
    """(N,) -> (N,4,4) rotations about z."""
    q = np.asarray(q, float)
    T = np.zeros(q.shape + (4, 4))
    c, s = np.cos(q), np.sin(q)
    T[..., 0, 0] = c
    T[..., 0, 1] = -s
    T[..., 1, 0] = s
    T[..., 1, 1] = c
    T[..., 2, 2] = 1
    T[..., 3, 3] = 1
    return T


def inv_T(T):
    R = T[..., :3, :3]
    p = T[..., :3, 3]
    Ti = np.zeros_like(T)
    Rt = np.swapaxes(R, -1, -2)
    Ti[..., :3, :3] = Rt
    Ti[..., :3, 3] = -np.einsum("...ij,...j->...i", Rt, p)
    Ti[..., 3, 3] = 1
    return Ti


def rot_angle(Ra, Rb):
    """Geodesic angle between rotation batches."""
    tr = np.einsum("...ij,...ij->...", Ra, Rb)
    return np.arccos(np.clip((tr - 1) / 2, -1, 1))


def wrap(q):
    return (q + np.pi) % (2 * np.pi) - np.pi


def axis_angle(axis, q):
    a = np.asarray(axis, float) / np.linalg.norm(axis)
    K = np.array([[0, -a[2], a[1]], [a[2], 0, -a[0]], [-a[1], a[0], 0]])
    return np.eye(3) + np.sin(q) * K + (1 - np.cos(q)) * K @ K


# ------------------------------------------------------------- URDF access ---
URDF_PROVENANCE = {}      # set_id -> dict(path, sha256, xacro) of the URDF actually used


def _src_prefix(cache_dir):
    """A private, throw-away ament prefix whose `ur_bringup` share dir is a symlink to
    src/ur_bringup. The set-2/3 xacros include $(find ur_bringup)/urdf/common/*.xacro;
    resolving that to the SAME src tree the top-level xacro comes from means no colcon
    build is needed and a stale (non-symlink) install is never mixed in. One prefix per
    call (removed afterwards): a shared, re-pointed link let concurrent runs from two
    checkouts read each other's xacros (found 2026-10-16)."""
    pre = tempfile.mkdtemp(prefix="_src_prefix.", dir=cache_dir)
    marker = os.path.join(pre, "share", "ament_index", "resource_index", "packages", "ur_bringup")
    os.makedirs(os.path.dirname(marker))
    open(marker, "w").close()
    os.symlink(os.path.join(_SRC, "ur_bringup"), os.path.join(pre, "share", "ur_bringup"))
    return pre


def generate_urdf(set_id, cache_dir=None):
    """Run xacro for set 1/2/3 and return the path of the generated URDF.

    Needs `source /opt/ros/jazzy/setup.bash` (xacro, ur_description; set 2/3 also the apt
    robotiq_description). ur_bringup is taken from this source tree, no build needed.
    xacro runs EVERY time (about 0.1 s): an mtime cache cannot see changes in included
    files or apt upgrades that keep old mtimes. The output is written atomically under a
    content-hash name and the ament prefix is private to the call, so concurrent workers
    and different checkouts never clash.
    """
    xacro_file = SET_XACRO[set_id]
    cache_dir = cache_dir or os.path.join(tempfile.gettempdir(), f"ur16e_base_placement-{os.getuid()}")
    os.makedirs(cache_dir, exist_ok=True)
    args = ["xacro", xacro_file]
    if set_id == 1:
        args += ["ur_type:=ur16e", "name:=ur16e"]
    env = os.environ.copy()
    pre = None
    if set_id >= 2:
        pre = _src_prefix(cache_dir)
        env["AMENT_PREFIX_PATH"] = pre + (":" + env["AMENT_PREFIX_PATH"] if env.get("AMENT_PREFIX_PATH") else "")
    try:
        urdf = subprocess.run(args, check=True, capture_output=True, text=True, env=env).stdout
    except FileNotFoundError as e:
        raise SystemExit("xacro not found -- run `source /opt/ros/jazzy/setup.bash` first") from e
    except subprocess.CalledProcessError as e:
        raise SystemExit(f"xacro failed for {xacro_file}:\n{e.stderr}") from e
    finally:
        if pre is not None:
            import shutil
            shutil.rmtree(pre, ignore_errors=True)
    sha = hashlib.sha256(urdf.encode()).hexdigest()
    out = os.path.join(cache_dir, f"set{set_id}-{sha[:16]}.urdf")
    if not os.path.exists(out):
        fd, tmp = tempfile.mkstemp(dir=cache_dir, suffix=".urdf.tmp")
        with os.fdopen(fd, "w") as f:
            f.write(urdf)
        os.replace(tmp, out)
    URDF_PROVENANCE[set_id] = dict(path=out, sha256=sha, xacro=xacro_file, args=args[2:])
    return out


class UrdfModel:
    """Parsed URDF joint tree (origins as 4x4, axes, types, limits)."""

    def __init__(self, path):
        root = ET.parse(path).getroot()
        self.path = path
        self.joints = {}
        self.child_of = {}
        self.links = {lk.get("name") for lk in root.findall("link")}
        for j in root.findall("joint"):
            o = j.find("origin")
            xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
            rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
            ax = j.find("axis")
            axis = np.array([float(v) for v in ax.get("xyz").split()]) if ax is not None else np.array([1., 0, 0])
            lim = j.find("limit")

            def _f(key):
                return float(lim.get(key)) if lim is not None and lim.get(key) is not None else None
            jd = dict(name=j.get("name"), type=j.get("type"),
                      parent=j.find("parent").get("link"), child=j.find("child").get("link"),
                      T=make_T(rpy_to_R(*rpy), xyz), axis=axis,
                      lower=_f("lower"), upper=_f("upper"), velocity=_f("velocity"))
            self.joints[jd["name"]] = jd
            self.child_of[jd["child"]] = jd
            self.links.update((jd["parent"], jd["child"]))

    def has_link(self, name):
        return name in self.links

    def _check(self, *names):
        for n in names:
            if n not in self.links:
                # Without this, an unknown link silently resolved to the tree root and
                # gave a transform ~0.9 m off (e.g. tcp 'pad' on the bare-arm set 1).
                raise KeyError(f"link '{n}' is not in URDF {self.path}")

    def chain(self, root_link, tip_link):
        out = []
        link = tip_link
        while link != root_link:
            j = self.child_of[link]
            out.append(j)
            link = j["parent"]
        return out[::-1]

    def _to_root(self, link, qmap=None):
        T = np.eye(4)
        while link in self.child_of:
            j = self.child_of[link]
            Tj = j["T"]
            if j["type"] in ("revolute", "continuous"):
                Tj = Tj @ make_T(axis_angle(j["axis"], (qmap or {}).get(j["name"], 0.0)))
            T = Tj @ T
            link = j["parent"]
        return T

    def fixed_T(self, root_link, tip_link, qmap=None):
        """root -> tip with movable joints from qmap (default 0)."""
        self._check(root_link, tip_link)
        try:
            ch = self.chain(root_link, tip_link)
        except KeyError:
            return inv_T(self._to_root(root_link, qmap)) @ self._to_root(tip_link, qmap)
        T = np.eye(4)
        for j in ch:
            T = T @ j["T"]
            if j["type"] in ("revolute", "continuous"):
                T = T @ make_T(axis_angle(j["axis"], (qmap or {}).get(j["name"], 0.0)))
        return T


class URDFArm:
    """Vectorised URDF FK base_link -> each arm link (+ tool0)."""

    def __init__(self, urdf_path):
        self.m = UrdfModel(urdf_path)
        ch = self.m.chain("base_link", "wrist_3_link")
        self.arm = [j for j in ch if j["type"] == "revolute"]
        names = [j["name"] for j in self.arm]
        assert names == ARM_JOINTS, f"unexpected arm joints {names} (tf_prefix set?)"
        for j in self.arm:
            assert np.allclose(j["axis"], [0, 0, 1]), j
        self.pre = []
        link = "base_link"
        for j in self.arm:
            self.pre.append(self.m.fixed_T(link, j["parent"]) @ j["T"])
            link = j["child"]
        self.T_w3_tool0 = self.m.fixed_T("wrist_3_link", "tool0")
        self.lower = np.array([j["lower"] for j in self.arm])
        self.upper = np.array([j["upper"] for j in self.arm])
        self.vmax = np.array([j["velocity"] for j in self.arm])

    def link_frames(self, q):
        """q (N,6) -> (N,6,4,4) base_link -> {shoulder..wrist_3}_link."""
        q = np.atleast_2d(q)
        N = q.shape[0]
        out = np.empty((N, 6, 4, 4))
        T = np.broadcast_to(np.eye(4), (N, 4, 4))
        for k in range(6):
            T = T @ self.pre[k] @ rotz_b(q[:, k])
            out[:, k] = T
        return out

    def fk_tool0(self, q):
        return self.link_frames(q)[:, 5] @ self.T_w3_tool0


# ------------------------------------------------------------------ DH FK ---
def dh_A(theta, d, a, alpha):
    theta = np.asarray(theta, float)
    ct, st = np.cos(theta), np.sin(theta)
    ca, sa = np.cos(alpha), np.sin(alpha)
    T = np.zeros(theta.shape + (4, 4))
    T[..., 0, 0] = ct
    T[..., 0, 1] = -st * ca
    T[..., 0, 2] = st * sa
    T[..., 0, 3] = a * ct
    T[..., 1, 0] = st
    T[..., 1, 1] = ct * ca
    T[..., 1, 2] = -ct * sa
    T[..., 1, 3] = a * st
    T[..., 2, 1] = sa
    T[..., 2, 2] = ca
    T[..., 2, 3] = d
    T[..., 3, 3] = 1
    return T


def dh_fk(q):
    q = np.atleast_2d(q)
    T = np.broadcast_to(np.eye(4), q.shape[:1] + (4, 4))
    for i in range(6):
        T = T @ dh_A(q[:, i], DH_D[i], DH_A[i], DH_ALPHA[i])
    return T


# ------------------------------------------------------------------ DH IK ---
def branch_flags(b):
    """branch index b = 4*s + 2*w + e -> (shoulder, wrist, elbow) bits."""
    return (b >> 2) & 1, (b >> 1) & 1, b & 1


BRANCH_NAMES = []
for _b in range(8):
    _s, _w, _e = branch_flags(_b)
    BRANCH_NAMES.append(f"{'S1' if _s == 0 else 'S2'}/{'W+' if _w == 0 else 'W-'}/{'E+' if _e == 0 else 'E-'}")


def dh_ik(T, tol=1e-9):
    """Closed-form IK for DH flange poses.

    T: (N,4,4) pose of DH frame 6 in DH frame 0.
    Returns Q (N,8,6) wrapped to (-pi, pi], valid (N,8) bool.
    Each branch flag is the sign of a quantity that vanishes exactly at one of the
    three UR singularities (shoulder / wrist / elbow), so two configurations with
    different flags cannot be joined without passing a singularity (or going around
    through a joint limit). Keeping ONE branch for all targets is what removes
    "reconfiguration" motions between targets.
    """
    T = np.asarray(T, float)
    N = T.shape[0]
    R = T[:, :3, :3]
    p = T[:, :3, 3]
    p05 = p - D6 * R[:, :, 2]
    r = np.hypot(p05[:, 0], p05[:, 1])
    phi = np.arctan2(p05[:, 1], p05[:, 0])
    ok_s = r >= abs(D4) - tol
    asn = np.arcsin(np.clip(D4 / np.maximum(r, 1e-12), -1, 1))
    th1 = np.stack([phi + asn, phi + np.pi - asn], 1)

    Q = np.zeros((N, 2, 2, 2, 6))
    V = np.zeros((N, 2, 2, 2), bool)
    for s in range(2):
        t1 = th1[:, s]
        s1, c1 = np.sin(t1), np.cos(t1)
        c5 = R[:, 0, 2] * s1 - R[:, 1, 2] * c1
        ok5 = np.abs(c5) <= 1 + 1e-9
        a5 = np.arccos(np.clip(c5, -1, 1))
        vx = R[:, 0, 0] * s1 - R[:, 1, 0] * c1
        vy = R[:, 0, 1] * s1 - R[:, 1, 1] * c1
        A1i = inv_T(dh_A(t1, DH_D[0], DH_A[0], DH_ALPHA[0]))
        for w in range(2):
            t5 = a5 if w == 0 else -a5
            s5 = np.sin(t5)
            sg = np.where(np.abs(s5) < 1e-10, 0.0, np.sign(s5))
            t6 = np.where(sg == 0, 0.0, np.arctan2(-vy * sg, vx * sg))
            A5i = inv_T(dh_A(t5, DH_D[4], DH_A[4], DH_ALPHA[4]))
            A6i = inv_T(dh_A(t6, DH_D[5], DH_A[5], DH_ALPHA[5]))
            T14 = A1i @ T @ A6i @ A5i
            px, py = T14[:, 0, 3], T14[:, 1, 3]
            c3 = (px * px + py * py - A2 * A2 - A3 * A3) / (2 * A2 * A3)
            ok3 = np.abs(c3) <= 1 + 1e-9
            a3 = np.arccos(np.clip(c3, -1, 1))
            t234 = np.arctan2(T14[:, 1, 0], T14[:, 0, 0])
            for e in range(2):
                t3 = a3 if e == 0 else -a3
                t2 = np.arctan2(py, px) - np.arctan2(A3 * np.sin(t3), A2 + A3 * np.cos(t3))
                t4 = t234 - t2 - t3
                Q[:, s, w, e] = np.stack([t1, t2, t3, t4, t5, t6], 1)
                V[:, s, w, e] = ok_s & ok5 & ok3
    Q = wrap(Q.reshape(N, 8, 6))
    return Q, V.reshape(N, 8)


# ---------------------------------------------------- singularity measures ---
def ur_measures(q):
    """UR-specific singularity distances and |det J| (6x6 geometric Jacobian).

    det J = a2*a3*sin(q3)*sin(q5)*(a2*c2 + a3*c23 + d5*s234)   (checked numerically)
      |sin q5| -> wrist singularity (J4 and J6 axes aligned)
      |sin q3| -> elbow singularity (arm stretched; folded is blocked mechanically)
      |sh| [m] -> shoulder singularity (wrist point on the J1-axis cylinder of radius d4)
    """
    q = np.asarray(q)
    s3, s5 = np.sin(q[..., 2]), np.sin(q[..., 4])
    sh = A2 * np.cos(q[..., 1]) + A3 * np.cos(q[..., 1] + q[..., 2]) + D5 * np.sin(q[..., 1] + q[..., 2] + q[..., 3])
    detJ = A2 * A3 * s3 * s5 * sh
    return np.abs(s5), np.abs(s3), np.abs(sh), np.abs(detJ)


def max_abs_detJ():
    """max over q of |det J| (normaliser for the manipulability score)."""
    from scipy.optimize import minimize
    g = np.linspace(-np.pi, np.pi, 121)
    q2, q3, q4 = np.meshgrid(g, g, g, indexing="ij")
    q = np.zeros(q2.shape + (6,))
    q[..., 1], q[..., 2], q[..., 3], q[..., 4] = q2, q3, q4, np.pi / 2
    w = ur_measures(q)[3]
    i = np.unravel_index(np.argmax(w), w.shape)
    x0 = np.array([g[i[0]], g[i[1]], g[i[2]]])

    def f(x):
        return -ur_measures(np.array([0, x[0], x[1], x[2], np.pi / 2, 0]))[3]
    res = minimize(f, x0, method="Nelder-Mead", options=dict(xatol=1e-10, fatol=1e-14))
    return float(-res.fun)


# --------------------------------------------- DH <-> URDF frame reconciliation ---
def reconcile(arm):
    """Constant transforms with T_base_link_tool0(q) = Bl_dh0 @ DH(q) @ Dh6_tool0."""
    T_bl_dh0 = arm.m.fixed_T("base_link", "base_link_inertia")      # Rz(pi) in ur_macro.xacro
    q0 = np.zeros((1, 6))
    T_dh6_tool0 = inv_T(dh_fk(q0)[0]) @ inv_T(T_bl_dh0) @ arm.fk_tool0(q0)[0]
    T_dh6_tool0[np.abs(T_dh6_tool0) < 1e-12] = 0.0
    return T_bl_dh0, T_dh6_tool0


# ------------------------------------------------------------------- TCP ---
def tcp_transform(set_urdf, tcp="gripper_frame", pad_offset=0.032, custom=None):
    """tool0 -> TCP.

    tcp:
      tool0          -- flange
      gripper_frame  -- the frame MoveIt / cuMotion plan for in this workspace
                        (set 2: tool0 + 11 mm coupling, set 3: + 18 mm, no rotation)
      pad            -- 2F-85 finger-pad centre, gripper OPEN: the two
                        robotiq_85_*_finger_tip_link origins (at finger_joint = 0)
                        averaged, + pad_offset along the tool axis (0.032 = value
                        pick_place_demo.py uses)
      custom         -- `custom` = dict(xyz=[..], rpy_deg=[..]) relative to tool0
    """
    if tcp == "custom":
        if not isinstance(custom, dict) or not ({"xyz", "rpy_deg"} & set(custom)):
            raise ValueError("tool.tcp: custom needs tool.custom: {xyz: [..], rpy_deg: [..]} (relative to tool0)")
        return make_T(rpy_to_R(*np.radians(custom.get("rpy_deg", [0, 0, 0]))), custom.get("xyz", [0, 0, 0]))
    if tcp == "tool0":
        return np.eye(4)
    if tcp not in ("gripper_frame", "pad"):
        raise ValueError(f"unknown tool.tcp '{tcp}' (tool0 | gripper_frame | pad | custom)")
    m = UrdfModel(set_urdf)
    if not m.has_link("gripper_frame"):
        raise ValueError(f"tool.tcp '{tcp}' needs the gripper (tool.set 2 or 3); set 1 is the bare arm -- "
                         "use tcp: tool0 or custom")
    T_t_gf = m.fixed_T("tool0", "gripper_frame")
    if tcp == "gripper_frame":
        return T_t_gf
    if tcp == "pad":
        tips = [m.fixed_T("gripper_frame", lk) for lk in
                ("robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link")]
        tip_p = np.mean([t[:3, 3] for t in tips], 0)
        return T_t_gf @ make_T(np.eye(3), tip_p + np.array([0, 0, pad_offset]))
    raise ValueError(f"unknown tcp '{tcp}'")


# --------------------------------------------------------- collision spheres ---
def load_spheres(arm, set_urdf, with_gripper=True, extra_tool0_spheres=(), xrdf=XRDF, which="collision"):
    """Collision spheres of the cuMotion XRDF expressed in the 6 arm-link frames.

    which = "collision" (the world set: spheres that COVER the meshes, for the cell) or
    "self_collision" (the self set: covering spheres only on the links that touch each
    other, medial ones elsewhere -- see ur_bringup/cumotion/gen_xrdf.py). Older XRDFs
    with one geometry give the same spheres for both.
    Gripper spheres are re-expressed in wrist_3_link through `set_urdf` (so the set-3
    camera bracket's +7 mm is applied), gripper open. The D405 bracket itself is NOT in
    the XRDF -- add it with `extra_tool0_spheres` = [{xyz:[..], r:..}, ...] (tool0 frame).
    The XRDF's base-housing spheres (`base_link_inertia`, self set) are skipped: the
    evaluator models the housing as the cell YAML's cylinder.
    """
    import yaml
    with open(xrdf) as f:
        X = yaml.safe_load(f)
    geom = X["geometry"][X[which]["geometry"]]["spheres"]
    env_buf = X["collision"].get("buffer_distance", {})
    m2 = UrdfModel(set_urdf)
    idx, cen, rad, ebuf, nm = [], [], [], [], []
    for link, sph in geom.items():
        if link == "base_link_inertia":
            continue
        if link in ARM_LINKS:
            k, T = ARM_LINKS.index(link), np.eye(4)
        else:
            if not with_gripper:
                continue
            k, T = 5, m2.fixed_T("wrist_3_link", link)
        for s in sph:
            c = T[:3, :3] @ np.array(s["center"], float) + T[:3, 3]
            idx.append(k)
            cen.append(c)
            rad.append(float(s["radius"]))
            ebuf.append(float(env_buf.get(link, 0.0)))
            nm.append(link)
    if extra_tool0_spheres:
        T_w3_t0 = arm.T_w3_tool0
        for s in extra_tool0_spheres:
            c = T_w3_t0[:3, :3] @ np.array(s["xyz"], float) + T_w3_t0[:3, 3]
            idx.append(5)
            cen.append(c)
            rad.append(float(s["r"]))
            ebuf.append(0.0)
            nm.append(s.get("name", "extra_tool"))
    sc = X["self_collision"]
    # radius = raw sphere radius; env_buffer = XRDF per-link environment buffer (applied
    # only against the cell); self_buffer = XRDF per-link self buffer (applied per pair).
    return dict(link_idx=np.array(idx), center=np.array(cen), radius=np.array(rad),
                env_buffer=np.array(ebuf), name=np.array(nm), ignore=sc["ignore"],
                self_buffer=sc.get("buffer_distance", {}))


# Arm link pairs that MoveIt does NOT check (ur_moveit_config ur_macro.srdf.xacro:
# adjacent pairs + wrist_1/wrist_3 "Never"). Everything else between arm links IS checked.
SRDF_ARM_IGNORE = {frozenset(p) for p in [
    ("shoulder_link", "upper_arm_link"), ("upper_arm_link", "forearm_link"),
    ("forearm_link", "wrist_1_link"), ("wrist_1_link", "wrist_2_link"),
    ("wrist_1_link", "wrist_3_link"), ("wrist_2_link", "wrist_3_link")]}


def self_pairs(sph):
    """Sphere pairs for self collision, following MoveIt's SRDF rather than the XRDF.

    The pre-2026-10-16 cuMotion XRDF's ignore list also skipped forearm<->wrist_2 and
    forearm<->wrist_3, which MoveIt does check (fixed in the regenerated XRDF, HISTORY.md 51).
    With it, the sweep accepted a candidate whose fixture pose MoveIt rejects as
    forearm<->wrist_2/3 contact (HISTORY.md 50.1). Measured
    on 1000 random configs vs MoveIt (this numpy model; cuMotion's own checker gives 49 -> 44
    for the same list change): missed self collisions 49 (XRDF list) -> 39 (SRDF
    list); with the evaluator's default self_margin 0.010 m -> 16 (32 false alarms).
    Those numbers are for the pre-regeneration XRDF (git history before 2026-10-16's
    regeneration). The evaluator now uses the regenerated WORLD set for self checks too,
    with a negative self_margin -- see HISTORY.md 51.6 (defaults still being re-tuned).
    Skipped: same link, gripper-internal, gripper vs wrist_1/2/3 (mounted on the wrist).
    """
    names = sph["name"]
    tool = {n for n in names if n not in ARM_LINKS}
    ii, jj, buf = [], [], []
    S = len(names)
    for i in range(S):
        for j in range(i + 1, S):
            a, b = names[i], names[j]
            if a == b:
                continue
            if a in tool and b in tool:
                continue
            if (a in tool and b in ("wrist_3_link", "wrist_2_link", "wrist_1_link")) or \
                    (b in tool and a in ("wrist_3_link", "wrist_2_link", "wrist_1_link")):
                continue
            if frozenset((a, b)) in SRDF_ARM_IGNORE:
                continue
            ii.append(i)
            jj.append(j)
            buf.append(sph["self_buffer"].get(a, 0.0) + sph["self_buffer"].get(b, 0.0))
    return np.array(ii, int), np.array(jj, int), np.array(buf)


def sphere_centers(arm, q, sph, T_world_base=None):
    """World sphere centres (N,S,3) for configs q (N,6); base pose (N,4,4) or None."""
    F = arm.link_frames(q)
    if T_world_base is not None:
        F = T_world_base[:, None] @ F
    out = np.empty((len(F), len(sph["link_idx"]), 3))
    for k in np.unique(sph["link_idx"]):          # one matmul per link, not a 4x4 copy per sphere
        m = sph["link_idx"] == k
        out[:, m] = (F[:, k, :3, :3] @ sph["center"][m].T).transpose(0, 2, 1) + F[:, k, None, :3, 3]
    return out
