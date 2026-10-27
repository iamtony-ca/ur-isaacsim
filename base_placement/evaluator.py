"""Vectorised evaluation of UR16e base candidates for a fixed set of TCP targets.

Problem.evaluate(mount, T_cell_base) -> per candidate:
  hard  : footprint fits, every target has IK inside the joint limits, ONE IK branch
          reaches every target, sphere model collision-free against the cell at every
          target (and against itself / its own base housing)
  soft  : score in [0, 1] = weighted sum of normalised terms (see DEFAULT_WEIGHTS)
"""
import numpy as np

import cell as cellmod
import ur16e_kin as kin


def cellmod_face(mount):
    return cellmod.MOUNT_FACE[mount["kind"]]

TWO_PI = 2 * np.pi

# Every term is normalised to [0, 1], higher = better. Weights sum to 1.
DEFAULT_WEIGHTS = dict(
    manip=0.25,   # min over targets of |det J| / max|det J|  (Yoshikawa w for a square J)
    ur=0.10,      # min over targets of min(|sin q5|, |sin q3|, shoulder_dist / sh_sat)
    jlim=0.10,    # min over targets & joints of (distance to the joint limit) / half range
    time=0.20,    # 1 - T_cycle / t_cycle_max  (velocity-limited joint moves, no accel)
    path=0.25,    # fraction of straight joint-space moves between targets that are collision free
    clear=0.10,   # min clearance of shoulder..wrist_1 spheres at the targets / clear_sat
)
DEFAULT_CONST = dict(
    t_cycle_max=15.0,      # s   -> time score 0
    clear_sat=0.10,        # m   -> clearance score 1
    sh_sat=0.25,           # m   -> shoulder-singularity distance considered "far"
    # Re-tuned 2026-10-20 for the regenerated XRDF, whose world spheres COVER the meshes
    # (HISTORY.md 51.6, 54.2). Example cell, MoveIt --pick all x3: all 60 target poses of the
    # top-10 mesh-collision-free every run; every FAIL was a plan near the shelf.
    env_margin=0.000,      # m   hard: sphere surface (+ XRDF env buffer) to any wall/box/pedestal.
                           #     The covering spheres already contain each link mesh + the XRDF's
                           #     5 mm buffer (check_xrdf.py), so no extra margin is needed for
                           #     mesh clearance. 5 mm drops both 15 cm-pedestal bases, 10 mm also
                           #     the 30 cm ones, all MoveIt-valid at their targets.
                           #     (old XRDF, 2026-10-16: 20 mm) The meshes stuck out of the spheres (+ buffer)
                           #     by up to 28/34/39 mm on shoulder/upper arm/forearm (HISTORY.md
                           #     50.2). Example top-10 vs MoveIt: 10 mm -> 2 targets in contact
                           #     (0.2 mm and -9 mm mesh clearance), 20 mm -> 0 with the same
                           #     recommended; 30 mm moves recommended, 40 mm leaves no base.
    self_margin=-0.025,    # m   hard: sphere-sphere self clearance. NEGATIVE: covering spheres of
                           #     neighbouring links overlap while their meshes do not; -25 mm = 11 missed
                           #     / 17 false alarms vs MoveIt on random configs (HISTORY.md 51.6).
                           #     (old XRDF, +10 mm: the spheres under-filled
                           #     the meshes). Chosen against MoveIt on the example cell's top-10
                           #     (HISTORY.md 50): 0 mm -> 7/10 MoveIt-valid, 5 -> 8, 10 -> 9,
                           #     15 -> 9 but drops every (MoveIt-valid) 15 cm-pedestal base.
                           #     Random configs: 0.010 = 16 missed / 32 false alarms per 1000 (old XRDF;
                           #     with the regenerated world set +0.010 refuses 347 of 790 valid configs).
    base_self_margin=0.005,  # m arm spheres vs the robot's own base housing
    path_step_deg=2.3,     # sum over joints of |dq| between collision samples on a joint move
    foot_margin=0.01,      # m   base / pedestal footprint clearance to obstacle boxes
)


class Problem:
    def __init__(self, cell, cache_dir=None):
        self.cell = cell
        tool = cell.tool
        self.set_id = int(tool.get("set", 2))
        set_urdf = kin.generate_urdf(self.set_id, cache_dir)
        ur_urdf = kin.generate_urdf(1, cache_dir)
        self.arm = kin.URDFArm(ur_urdf)
        # The XRDF's WORLD spheres (they cover the meshes, <= 20 mm overshoot) serve both the
        # cell and the self checks. Its separate self set (cuMotion's) has ~43k SRDF pairs and
        # made the grid ~12x and refinement ~30x slower before the speed-ups (run aborted); the world set has ~9k and, with a negative
        # self_margin for the overshoot, agrees better with MoveIt (HISTORY.md 51).
        self.sph = kin.load_spheres(self.arm, set_urdf, with_gripper=self.set_id >= 2,
                                    extra_tool0_spheres=tool.get("extra_spheres", []) or [],
                                    which="collision")
        self.sph_self = self.sph
        self.T_bl_dh0, self.T_dh6_tool0 = kin.reconcile(self.arm)
        tcp = tool.get("tcp", "gripper_frame" if self.set_id >= 2 else "tool0")
        self.T_tool0_tcp = kin.tcp_transform(set_urdf, tcp, float(tool.get("pad_offset", 0.032)),
                                             tool.get("custom"))
        self.names = cell.names
        self.order = cell.order
        self.cyclic = cell.cyclic
        self.T_w_tcp = cell.T_tcp
        self.T_w_tool0 = self.T_w_tcp @ kin.inv_T(self.T_tool0_tcp)
        self.T_right = self.T_w_tool0 @ kin.inv_T(self.T_dh6_tool0)
        self.pi, self.pj, self.pbuf = kin.self_pairs(self.sph_self)
        self.rad = self.sph["radius"]
        self.ebuf = self.sph["env_buffer"]
        self.rad_s = self.sph_self["radius"]
        self.base_chk = ~np.isin(self.sph_self["name"], ["shoulder_link", "upper_arm_link"])
        self.wmax = kin.max_abs_detJ()

        # joint limits / velocities: URDF, then symmetric overrides from the cell YAML
        self.lower, self.upper, self.vmax = self.arm.lower.copy(), self.arm.upper.copy(), self.arm.vmax.copy()
        for key, deg in cell.limit_override_deg.items():
            j = _joint_index(key)
            self.lower[j], self.upper[j] = -np.radians(deg), np.radians(deg)
        for key, deg in cell.velocity_override_deg.items():
            self.vmax[_joint_index(key)] = np.radians(deg)

        w = dict(DEFAULT_WEIGHTS)
        uw = cell.scoring.get("weights", {}) or {}
        bad = sorted(set(uw) - set(DEFAULT_WEIGHTS))
        if bad:
            raise ValueError(f"scoring.weights: unknown terms {bad} (known: {sorted(DEFAULT_WEIGHTS)})")
        w.update(uw)
        if any(v < 0 for v in w.values()) or sum(w.values()) <= 0:
            raise ValueError("scoring.weights must be >= 0 with a positive sum")
        tot = sum(w.values())
        self.W = {k: v / tot for k, v in w.items()}
        self.K = dict(DEFAULT_CONST)
        uk = {k: v for k, v in cell.scoring.items() if k != "weights"}
        bad = sorted(set(uk) - set(DEFAULT_CONST))
        if bad:
            raise ValueError(f"scoring: unknown keys {bad} (known: weights, {sorted(DEFAULT_CONST)})")
        self.K.update({k: float(v) for k, v in uk.items()})

        self.box_c = (cell.box_lo + cell.box_hi) / 2
        self.box_h = (cell.box_hi - cell.box_lo) / 2
        self._walls = dict(cell.walls)      # set per mount in evaluate()/detail()
        if self.set_id == 1 and tcp != "tool0" and not tool.get("extra_spheres"):
            print("  WARNING: tool.set 1 has no tool collision geometry; a custom TCP offset is not "
                  "collision-checked -- add tool.extra_spheres for the real tool")

    # ------------------------------------------------------------ geometry ---
    def footprint_ok(self, mount, T_wb):
        """Base housing (+ adapter plate for wall/ceiling, + floor pedestal) fits.

        Inside the cell: rim sample points (error <= r(1-cos 7.5 deg) ~ 1 mm).
        Against obstacle boxes: exact cylinder-vs-box test. Sampling only the rim missed
        boxes that sit INSIDE the base disk or under the pedestal (found 2026-10-16).
        Mount rotations are signed axis permutations, so the cylinder axis is a cell
        axis; a small tilt (robustness perturbation) inflates the radius by h*sin(tilt).
        """
        P = self.cell.footprint_points(mount, T_wb)
        M = len(T_wb)
        ar = np.arange(M)
        z = T_wb[:, :3, 2]
        ax = np.abs(z).argmax(1)
        zc = z[ar, ax]
        # A tilted base (robustness random_6dof) dips its rim into its own mounting face by
        # base_r*sin(tilt) -- that is the surface it sits on, not a collision.
        lo_eff = np.broadcast_to(self.cell.lo - 1e-9, (M, 3)).copy()
        hi_eff = np.broadcast_to(self.cell.hi + 1e-9, (M, 3)).copy()
        face = cellmod_face(mount)
        fax, fside = "xyz".index(face[0]), face[1:]
        tol = self.cell.base_r * np.sqrt(np.maximum(0.0, 1 - zc ** 2))
        if fside == "min":
            lo_eff[:, fax] -= tol
        else:
            hi_eff[:, fax] += tol
        ok = ((P >= lo_eff[:, None]) & (P <= hi_eff[:, None])).all(-1).all(1)
        if not len(self.box_c):
            return ok
        fm = self.K["foot_margin"]
        blo, bhi = self.cell.box_lo, self.cell.box_hi                  # (B,3)
        s = np.sign(zc)
        p = T_wb[:, :3, 3]
        plate = 0.0 if mount["kind"] == "floor" else self.cell.plate
        a0 = p[ar, ax] - s * plate
        a1 = p[ar, ax] + s * self.cell.base_h
        amin, amax = np.minimum(a0, a1), np.maximum(a0, a1)
        r_eff = self.cell.base_r + fm + (self.cell.base_h + plate) * np.sqrt(np.maximum(0.0, 1 - zc ** 2))
        hit = self._cyl_box_hit(p, ax, amin, amax, r_eff, fm, blo, bhi)
        h = float(mount.get("height", 0.0)) if mount["kind"] == "floor" else 0.0
        if h > 0:
            axz = np.full(M, 2)
            pz0 = np.full(M, self.cell.lo[2])
            hit |= self._cyl_box_hit(p, axz, pz0, pz0 + h, np.full(M, self.cell.pedestal_r + fm), fm, blo, bhi)
        return ok & ~hit

    @staticmethod
    def _cyl_box_hit(p, ax, amin, amax, r, fm, blo, bhi):
        """(M,) any box intersects the axis-aligned cylinder (axis index ax, through p,
        axial extent [amin, amax], radius r)."""
        M = len(p)
        onehot = np.zeros((M, 1, 3))
        onehot[np.arange(M), 0, ax] = 1.0
        delta = np.maximum(np.maximum(blo[None] - p[:, None], 0.0), p[:, None] - bhi[None])   # (M,B,3)
        radial = np.linalg.norm(delta * (1 - onehot), axis=-1)                                 # (M,B)
        lo_ax = np.take_along_axis(np.broadcast_to(blo[None], (M,) + blo.shape), ax[:, None, None], 2)[..., 0]
        hi_ax = np.take_along_axis(np.broadcast_to(bhi[None], (M,) + bhi.shape), ax[:, None, None], 2)[..., 0]
        axial = (amax[:, None] + fm > lo_ax) & (amin[:, None] - fm < hi_ax)
        return ((radial < r[:, None]) & axial).any(1)

    def env_sdf(self, Cw, ped):
        """Signed distance of points Cw (K,S,3) to the enabled cell walls, boxes, pedestal."""
        lo, hi, wl = self.cell.lo, self.cell.hi, self._walls
        d = np.full(Cw.shape[:2], np.inf)
        for ax, name in enumerate("xyz"):
            if wl.get(f"{name}min", True):
                d = np.minimum(d, Cw[..., ax] - lo[ax])
            if wl.get(f"{name}max", True):
                d = np.minimum(d, hi[ax] - Cw[..., ax])
        # boxes one at a time and x/y/z as separate arrays: reductions over the length-3
        # and box axes were a third of the run time with the covering (195-sphere) set
        X, Y, Z = Cw[..., 0], Cw[..., 1], Cw[..., 2]
        for (cx, cy, cz), (hx, hy, hz) in zip(self.box_c, self.box_h):
            qx, qy, qz = np.abs(X - cx) - hx, np.abs(Y - cy) - hy, np.abs(Z - cz) - hz
            out = np.sqrt(np.maximum(qx, 0) ** 2 + np.maximum(qy, 0) ** 2 + np.maximum(qz, 0) ** 2)
            d = np.minimum(d, out + np.minimum(np.maximum(np.maximum(qx, qy), qz), 0))
        if ped is not None:
            cx, cy, h = ped[:, 0:1], ped[:, 1:2], ped[:, 2:3]
            dr = np.hypot(Cw[..., 0] - cx, Cw[..., 1] - cy) - self.cell.pedestal_r
            dz = np.abs(Cw[..., 2] - (lo[2] + h / 2)) - h / 2
            dp = np.hypot(np.maximum(dr, 0), np.maximum(dz, 0)) + np.minimum(np.maximum(dr, dz), 0)
            d = np.minimum(d, dp)
        return d

    def collide(self, q, T_wb, ped, chunk=20000, self_chunk=200):
        """q (K,6), T_wb (K,4,4), ped (K,3)|None -> env clearance (K,), self_ok (K,),
        arm clearance (K,) of shoulder..wrist_1 spheres only (the wrist/gripper spheres
        are pinned by the TCP pose and do not depend on the base)."""
        K = len(q)
        clear = np.empty(K)
        clear_arm = np.empty(K)
        arm_sel = self.sph["link_idx"] <= 3
        selfok = np.zeros(K, bool)
        for a in range(0, K, chunk):
            b = min(K, a + chunk)
            Cb = kin.sphere_centers(self.arm, q[a:b], self.sph)
            Cw = Cb @ T_wb[a:b, :3, :3].transpose(0, 2, 1) + T_wb[a:b, None, :3, 3]
            dsph = self.env_sdf(Cw, None if ped is None else ped[a:b]) - self.rad - self.ebuf
            c = dsph.min(1)
            clear[a:b] = c
            clear_arm[a:b] = dsph[:, arm_sel].min(1)
            ok = c >= self.K["env_margin"]
            if ok.any():
                Cs = kin.sphere_centers(self.arm, q[a:b][ok], self.sph_self)
                X = Cs[:, self.base_chk]
                dr = np.hypot(X[..., 0], X[..., 1]) - self.cell.base_r
                dz = np.abs(X[..., 2] - self.cell.base_h / 2) - self.cell.base_h / 2
                dbase = np.hypot(np.maximum(dr, 0), np.maximum(dz, 0)) + np.minimum(np.maximum(dr, dz), 0)
                bok = (dbase - self.rad_s[self.base_chk] >= self.K["base_self_margin"]).all(1)
                sok = np.ones(len(Cs), bool)
                if len(self.pi):
                    # ~9k pairs: small chunks bound the memory, and only the configurations
                    # still in question are checked
                    # float32: ~1e-4 mm at these distances, half the memory traffic of the gathers
                    rr2 = (np.maximum(self.rad_s[self.pi] + self.rad_s[self.pj] + self.pbuf
                                      + self.K["self_margin"], 0.0) ** 2).astype(np.float32)
                    todo = np.where(bok)[0]
                    for k in range(0, len(todo), self_chunk):
                        sl = todo[k:k + self_chunk]
                        P = Cs[sl].astype(np.float32)
                        d2 = sum((P[..., ax][:, self.pi] - P[..., ax][:, self.pj]) ** 2 for ax in range(3))
                        sok[sl] = (d2 - rr2).min(1) >= 0
                tmp = np.zeros(b - a, bool)
                tmp[ok] = bok & sok
                selfok[a:b] = tmp
        return clear, selfok, clear_arm

    # ----------------------------------------------------------------- IK ---
    def _alias_ok(self, Q):
        """(…,6) principal values -> (…) bool: every joint has v or its 2pi alias in limits."""
        alt = Q - TWO_PI * np.where(Q >= 0, 1.0, -1.0)
        ok = ((Q >= self.lower - 1e-9) & (Q <= self.upper + 1e-9)) | \
             ((alt >= self.lower - 1e-9) & (alt <= self.upper + 1e-9))
        return ok.all(-1)

    def ik_all(self, T_wb):
        """(M,4,4) -> Q (M,nt,8,6), V (M,nt,8) (IK exists AND inside joint limits)."""
        M, nt = len(T_wb), len(self.names)
        L = kin.inv_T(self.T_bl_dh0) @ kin.inv_T(T_wb)
        T = L[:, None] @ self.T_right[None]
        Q, V = kin.dh_ik(T.reshape(-1, 4, 4))
        Q = Q.reshape(M, nt, 8, 6)
        V = V.reshape(M, nt, 8) & self._alias_ok(Q)
        return Q, V

    # -------------------------------------------------------- joint aliases ---
    def unwrap_cycle(self, Q):
        """Pick +-2pi aliases (inside the limits) that minimise per-joint travel along
        the visiting order (closed if cyclic). Q (F,nt,6) principal -> (F,nt,6)."""
        Qo = Q[:, self.order]
        F, nt, _ = Qo.shape
        out = Qo.copy()
        for j in range(6):
            v = Qo[:, :, j]
            alt = v - TWO_PI * np.where(v >= 0, 1.0, -1.0)
            opts = np.stack([v, alt], -1)
            pen = np.where((opts >= self.lower[j] - 1e-9) & (opts <= self.upper[j] + 1e-9), 0.0, np.inf)
            best_cost = np.full(F, np.inf)
            best_path = np.zeros((F, nt), int)
            for s0 in range(2):
                cost = np.full((F, 2), np.inf)
                cost[:, s0] = pen[:, 0, s0]
                back = np.zeros((F, nt, 2), int)
                for t in range(1, nt):
                    d = np.abs(opts[:, t, None, :] - opts[:, t - 1, :, None])
                    tot = cost[:, :, None] + d + pen[:, t, None, :]
                    back[:, t] = tot.argmin(1)
                    cost = tot.min(1)
                close = cost + (np.abs(opts[:, nt - 1, :] - opts[:, 0, s0][:, None]) if self.cyclic else 0.0)
                last = close.argmin(1)
                c = close.min(1)
                path = np.zeros((F, nt), int)
                path[:, nt - 1] = last
                for t in range(nt - 1, 0, -1):
                    path[:, t - 1] = back[np.arange(F), t, path[:, t]]
                better = c < best_cost
                best_cost[better] = c[better]
                best_path[better] = path[better]
            out[:, :, j] = np.take_along_axis(opts, best_path[..., None], -1)[..., 0]
        # A uniform 2pi shift of one joint's whole trajectory keeps the travel, FK,
        # collisions and every other term exactly; only the joint-limit margin changes.
        # The DP above picks between such ties by float noise (q5 reported at -270 deg
        # where +90 is equivalent), so centre each joint explicitly (found 2026-10-16).
        half = (self.upper - self.lower) / 2
        mid = (self.upper + self.lower) / 2
        for j in range(6):
            cur = out[:, :, j]
            best_m = ((half[j] - np.abs(cur - mid[j])) / half[j]).min(1)
            best = cur
            for k in (-1, 1):
                c = cur + k * TWO_PI
                inl = ((c >= self.lower[j] - 1e-9) & (c <= self.upper[j] + 1e-9)).all(1)
                m = np.where(inl, ((half[j] - np.abs(c - mid[j])) / half[j]).min(1), -np.inf)
                better = m > best_m + 1e-12
                best = np.where(better[:, None], c, best)
                best_m = np.where(better, m, best_m)
            out[:, :, j] = best
        return out[:, np.argsort(self.order)]

    def _moves(self, Qo):
        """Visiting-order configs (F,nt,6) -> move start/end (F,nm,6)."""
        if self.cyclic:
            return Qo, np.roll(Qo, -1, axis=1)
        return Qo[:, :-1], Qo[:, 1:]

    # ------------------------------------------------------------- metrics ---
    def soft_metrics(self, Qa, clear_t):
        s5, s3, sh, dj = kin.ur_measures(Qa)
        manip = (dj / self.wmax).min(1)
        ur = np.minimum(np.minimum(s5, s3), np.minimum(sh / self.K["sh_sat"], 1)).min(1)
        half = (self.upper - self.lower) / 2
        mid = (self.upper + self.lower) / 2
        jl = ((half - np.abs(Qa - mid)) / half).min((1, 2))
        A, B = self._moves(Qa[:, self.order])
        tmove = (np.abs(B - A) / self.vmax).max(-1) if A.shape[1] else np.zeros((len(Qa), 0))
        return dict(manip=manip, ur=ur, jlim=jl, tcycle=tmove.sum(1), tmove=tmove,
                    clear=clear_t.min(1), s5=s5, s3=s3, sh=sh, wnorm=dj / self.wmax)

    def path_check(self, Qa, T_wb, ped):
        """Straight joint-space moves between consecutive targets: per-move free flag,
        min clearance and min normalised |det J| (F,nm). NOT a planner: a blocked
        straight move only means the planner will have to detour."""
        A, B = self._moves(Qa[:, self.order])
        F, nm, _ = A.shape
        if nm == 0:
            z = np.ones((F, 0))
            return z.astype(bool), z, z, 0
        A, B = A.reshape(-1, 6), B.reshape(-1, 6)
        step = np.radians(self.K["path_step_deg"])
        # samples by the SUM of joint changes: with several joints moving, a max-joint step
        # lets the arm sweep further between samples and skip a thin obstacle
        n = np.maximum(2, np.ceil(np.abs(B - A).sum(1) / step).astype(int))
        cnt = n - 1
        mid = np.repeat(np.arange(F * nm), cnt)
        k = np.arange(cnt.sum()) - np.repeat(np.cumsum(cnt) - cnt, cnt) + 1
        s = (k / np.repeat(n, cnt))[:, None]
        Qs = A[mid] + s * (B[mid] - A[mid])
        cand = mid // nm
        clear, selfok, _ = self.collide(Qs, T_wb[cand], None if ped is None else ped[cand])
        ok = (clear >= self.K["env_margin"]) & selfok
        free = np.ones(F * nm, bool)
        np.logical_and.at(free, mid, ok)
        cmin = np.full(F * nm, np.inf)
        np.minimum.at(cmin, mid, clear)
        wn = kin.ur_measures(Qs)[3] / self.wmax
        wmin = np.full(F * nm, np.inf)
        np.minimum.at(wmin, mid, wn)
        return free.reshape(F, nm), cmin.reshape(F, nm), wmin.reshape(F, nm), int(len(Qs))

    # -------------------------------------------------------------- driver ---
    def evaluate(self, mount, T_wb, with_path=True, chunk=1500):
        """Evaluate M base poses on one mount. Returns dict of arrays (M,) unless noted."""
        M = len(T_wb)
        nt = len(self.names)
        self._walls = self.cell.walls_for(mount)
        ped_all = self.cell.pedestal(mount, T_wb)
        res = dict(foot=np.zeros(M, bool), reach_any=np.zeros(M, bool), common_ik=np.zeros(M, bool),
                   collfree_any=np.zeros(M, bool), feasible=np.zeros(M, bool),
                   branch=np.full(M, -1), score=np.full(M, -np.inf), score_nopath=np.full(M, -np.inf),
                   n_feasible_branches=np.zeros(M, int), branch_scores=np.full((M, 8), -np.inf),
                   Q=np.full((M, nt, 6), np.nan), n_path_samples=0,
                   # per-target diagnostics: which target blocks this base?
                   tgt_reach=np.zeros((M, nt), bool),    # some IK branch inside the limits
                   tgt_cfree=np.zeros((M, nt), bool),    # some branch also collision free
                   best_cov=np.zeros(M, int))            # most targets ONE branch serves collision free
        for k in ["manip", "ur", "jlim", "tcycle", "clear", "clear_all", "path_frac", "path_clear", "path_wmin"]:
            res[k] = np.full(M, np.nan)
        foot = self.footprint_ok(mount, T_wb)
        res["foot"] = foot
        idx_all = np.where(foot)[0]
        W = self.W
        for a in range(0, len(idx_all), chunk):
            idx = idx_all[a:a + chunk]
            m = len(idx)
            Tc = T_wb[idx]
            Q, V = self.ik_all(Tc)
            res["reach_any"][idx] = V.any(2).all(1)
            res["common_ik"][idx] = V.all(1).any(1)
            mi, ti, bi = np.where(V)
            clear = np.full((m, nt, 8), -np.inf)
            clear_arm = np.full((m, nt, 8), -np.inf)
            okc = np.zeros((m, nt, 8), bool)
            if len(mi):
                pc = None if ped_all is None else ped_all[idx][mi]
                c, so, ca = self.collide(Q[mi, ti, bi], Tc[mi], pc)
                clear[mi, ti, bi] = c
                clear_arm[mi, ti, bi] = ca
                okc[mi, ti, bi] = (c >= self.K["env_margin"]) & so
            res["collfree_any"][idx] = okc.any(2).all(1)
            res["tgt_reach"][idx] = V.any(2)
            res["tgt_cfree"][idx] = okc.any(2)
            res["best_cov"][idx] = okc.sum(1).max(1)
            bok = okc.all(1)
            fm, fb = np.where(bok)
            if len(fm) == 0:
                continue
            Qa = self.unwrap_cycle(Q[fm, :, fb])
            met = self.soft_metrics(Qa, clear_arm[fm, :, fb])
            met["clear_all"] = clear[fm, :, fb].min(1)
            s_time = np.clip(1 - met["tcycle"] / self.K["t_cycle_max"], 0, 1)
            s_clear = np.clip(met["clear"] / self.K["clear_sat"], 0, 1)
            pre = (W["manip"] * met["manip"] + W["ur"] * met["ur"] + W["jlim"] * met["jlim"]
                   + W["time"] * s_time + W["clear"] * s_clear)
            if with_path:
                free, pclr, pw, ns = self.path_check(Qa, Tc[fm], None if ped_all is None else ped_all[idx][fm])
                res["n_path_samples"] += ns
                pfrac = free.mean(1) if free.shape[1] else np.ones(len(fm))
                score = pre + W["path"] * pfrac
            else:
                pclr = pw = None
                pfrac = np.full(len(fm), np.nan)
                score = pre + W["path"]            # optimistic when paths are not checked
            bs = np.full((m, 8), -np.inf)
            bs[fm, fb] = score
            res["branch_scores"][idx] = bs
            best_b = bs.argmax(1)
            has = bok.any(1)
            gi = idx[has]
            res["feasible"][gi] = True
            res["n_feasible_branches"][idx] = bok.sum(1)
            res["branch"][gi] = best_b[has]
            row = {(int(x), int(y)): r for r, (x, y) in enumerate(zip(fm, fb))}
            rows = np.array([row[(int(mm), int(best_b[mm]))] for mm in np.where(has)[0]])
            res["score"][gi] = score[rows]
            res["score_nopath"][gi] = pre[rows]
            for k in ["manip", "ur", "jlim", "tcycle", "clear", "clear_all"]:
                res[k][gi] = met[k][rows]
            if with_path and pclr is not None and pclr.shape[1]:
                res["path_frac"][gi] = pfrac[rows]
                res["path_clear"][gi] = pclr[rows].min(1)
                res["path_wmin"][gi] = pw[rows].min(1)
            res["Q"][gi] = Qa[rows]
        return res

    def detail(self, mount, T_wb, branch):
        """Per-target report for ONE base pose and branch (+ URDF FK check)."""
        T_wb = T_wb[None]
        self._walls = self.cell.walls_for(mount)
        Q, V = self.ik_all(T_wb)
        Qa = self.unwrap_cycle(Q[0, :, branch][None])[0]
        nt = len(self.names)
        ped = self.cell.pedestal(mount, T_wb)
        Tr = np.repeat(T_wb, nt, 0)
        c, so, ca = self.collide(Qa, Tr, None if ped is None else np.repeat(ped, nt, 0))
        met = self.soft_metrics(Qa[None], ca[None])
        free, pclr, pw, _ = self.path_check(Qa[None], T_wb, ped)
        T_reach = T_wb[0] @ self.arm.fk_tool0(Qa) @ self.T_tool0_tcp
        perr = np.linalg.norm(T_reach[:, :3, 3] - self.T_w_tcp[:, :3, 3], axis=1)
        rerr = kin.rot_angle(T_reach[:, :3, :3], self.T_w_tcp[:, :3, :3])
        half = (self.upper - self.lower) / 2
        jlm = ((half - np.abs(Qa - (self.upper + self.lower) / 2)) / half).min(1)
        nm = free.shape[1]
        rows = []
        for t in range(nt):
            k = self.order.index(t)
            mv = None
            if k < nm:
                mv = dict(to=self.names[self.order[(k + 1) % nt]], t_s=float(met["tmove"][0, k]),
                          straight_free=bool(free[0, k]), min_clear_m=float(pclr[0, k]),
                          min_w_norm=float(pw[0, k]))
            rows.append(dict(target=self.names[t], ik_valid=bool(V[0, t, branch]),
                             q_rad=[float(x) for x in Qa[t]], q_deg=np.degrees(Qa[t]).round(2).tolist(),
                             clearance_m=float(c[t]), clearance_arm_m=float(ca[t]), self_ok=bool(so[t]),
                             w_norm=float(met["wnorm"][0, t]), abs_sin_q5=float(met["s5"][0, t]),
                             abs_sin_q3=float(met["s3"][0, t]), shoulder_dist_m=float(met["sh"][0, t]),
                             jl_margin=float(jlm[t]), fk_pos_err_m=float(perr[t]),
                             fk_rot_err_rad=float(rerr[t]), move_to_next=mv))
        return rows, Qa


def _joint_index(key):
    names = kin.ARM_JOINTS
    short = {"shoulder_pan": 0, "shoulder_lift": 1, "elbow": 2, "wrist_1": 3, "wrist_2": 4, "wrist_3": 5,
             "j1": 0, "j2": 1, "j3": 2, "j4": 3, "j5": 4, "j6": 5}
    if key in names:
        return names.index(key)
    k = key.lower().removesuffix("_joint")
    if k in short:
        return short[k]
    raise ValueError(f"unknown joint '{key}'")
