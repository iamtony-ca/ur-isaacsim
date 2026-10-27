#!/usr/bin/env python3
"""Re-check candidate base poses with MoveIt (real meshes, real planner) -- no execution.

The sweep in place_base.py uses a sphere model (a fast filter: with the pre-regeneration
XRDF ~95 % agreement with MoveIt's meshes on SELF-collision of 1000 random configurations;
for the regenerated XRDF see HISTORY.md 51.6; the cell itself is boxes on both sides) and
straight joint moves. Before anything is bolted down, ask the
running move_group about the same candidate(s):
  1. put the cell (enabled walls, obstacle boxes, pedestal) into the planning scene,
     expressed in base_link -- so the robot URDF does NOT have to be moved first; the
     objects are read back to make sure move_group really holds them
  2. /compute_fk: tool0 of each target configuration vs. the pose place_base.py expects
     (catches arm-frame mismatches between the sweep and the MoveIt model; the TCP
     offset itself is not seen by MoveIt and is not checked here)
  3. /check_state_validity for every target configuration (mesh collision; joint
     limits only as MoveIt knows them -- the YAML overrides were applied by the sweep)
  4. MoveGroup plan_only between consecutive targets (start = previous target) with
     the configured planner. Moves into or out of an INVALID target are skipped:
     MoveIt's start-state check reports misleading contacts for a self-colliding
     start state (seen 2026-10-16: "bench" contacts for a forearm<->wrist pose).

Needs move_group for the SAME set as the cell YAML (`tool.set`), which needs the built
workspace (unlike place_base.py). In every terminal:
    source /opt/ros/jazzy/setup.bash && source <ws>/install/setup.bash && export ROS_DOMAIN_ID=<id>
Set 2 without Isaac (the sim control stack runs on its own when use_sim_time is false):
    ros2 launch ur_bringup ur16e_2f85.launch.py use_sim_time:=false
    ros2 launch ur_bringup ur16e_2f85_moveit.launch.py use_sim_time:=false launch_rviz:=false
then:
    python3 moveit_validate.py out/confined/results.json --pick all
OMPL is randomised: one run is one sample. For a decision, run --pick all two or three
times (or raise --retries) and prefer candidates that pass every time.
--pick recommended | best | N (rank in `top`, 1 = best) | all (every top candidate,
reports which pass). Exit 0 if any candidate passes, 1 if all checked ones fail,
2 if the check was cut short (move_group gone) and nothing passed before that.
--keep leaves the last cell in the scene.
"""
import argparse
import json
import sys

import numpy as np
import rclpy
from geometry_msgs.msg import Pose
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (CollisionObject, Constraints, JointConstraint, MotionPlanRequest, MoveItErrorCodes,
                             PlanningOptions, PlanningScene, PlanningSceneComponents, RobotState)
from moveit_msgs.srv import ApplyPlanningScene, GetPlanningScene, GetPositionFK, GetStateValidity
from rclpy.action import ActionClient
from rclpy.node import Node
from sensor_msgs.msg import JointState
from shape_msgs.msg import SolidPrimitive

import cell as cellmod
import ur16e_kin as kin

GROUP = "ur_manipulator"
FRAME = "base_link"
WALL_T = 0.20          # wall slab thickness [m], outside the bounds; thick so OMPL's
                       # discrete edge checks cannot step across it
CELL_PREFIX = "bp_cell_"   # generated objects (walls, pedestal)
OBS_PREFIX = "bp_obs_"     # the cell's obstacle boxes -- separate namespace, so an obstacle
                           # named 'pedestal' or 'wall_xmin' cannot replace a generated object
# rosidl puts message constants on the metaclass, not in vars(cls)
ERR = {getattr(MoveItErrorCodes, k): k for k in dir(type(MoveItErrorCodes))
       if k.isupper() and isinstance(getattr(MoveItErrorCodes, k, None), int)}


def _pose(T):
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    w, x, y, z = kin.R_to_quat_wxyz(T[:3, :3])
    p.orientation.w, p.orientation.x, p.orientation.y, p.orientation.z = float(w), float(x), float(y), float(z)
    return p


def _obj(oid, prim, T_base_center):
    co = CollisionObject()
    co.header.frame_id = FRAME
    co.id = oid
    co.primitives = [prim]
    co.primitive_poses = [_pose(T_base_center)]
    co.operation = CollisionObject.ADD
    return co


def cell_objects(C, mount, T_cb):
    """Cell geometry as CollisionObjects in base_link (= inv(T_cell_base) * cell)."""
    T_bc = kin.inv_T(T_cb)
    objs = []
    lo, hi = C.lo, C.hi
    ext = hi - lo
    mid = (lo + hi) / 2
    walls = C.walls_for(mount)              # the mounting face is always solid
    for ax, n in enumerate("xyz"):
        for side in ("min", "max"):
            flag = f"{n}{side}"
            if not walls.get(flag, True):
                continue
            c = mid.copy()
            c[ax] = (lo[ax] - WALL_T / 2) if side == "min" else (hi[ax] + WALL_T / 2)
            size = ext + 2 * WALL_T
            size[ax] = WALL_T
            objs.append(_obj(f"{CELL_PREFIX}wall_{flag}", SolidPrimitive(type=SolidPrimitive.BOX, dimensions=size.tolist()),
                             T_bc @ kin.make_T(np.eye(3), c)))
    for name, bl, bh in zip(C.box_names, C.box_lo, C.box_hi):
        objs.append(_obj(f"{OBS_PREFIX}{name}", SolidPrimitive(type=SolidPrimitive.BOX, dimensions=(bh - bl).tolist()),
                         T_bc @ kin.make_T(np.eye(3), (bl + bh) / 2)))
    h = float(mount.get("height", 0.0)) if mount["kind"] == "floor" else 0.0
    if h > 0:
        c = np.array([T_cb[0, 3], T_cb[1, 3], lo[2] + h / 2])
        objs.append(_obj(f"{CELL_PREFIX}pedestal", SolidPrimitive(type=SolidPrimitive.CYLINDER,
                                                    dimensions=[float(h - 0.002), float(C.pedestal_r)]),
                         T_bc @ kin.make_T(np.eye(3), c)))
    ids = [o.id for o in objs]
    if len(set(ids)) != len(ids):
        raise ValueError(f"duplicate planning-scene ids {ids}")
    return objs


class Validator(Node):
    def __init__(self):
        super().__init__("base_placement_validate")
        self.apply = self.create_client(ApplyPlanningScene, "/apply_planning_scene")
        self.getps = self.create_client(GetPlanningScene, "/get_planning_scene")
        self.valid = self.create_client(GetStateValidity, "/check_state_validity")
        self.fk = self.create_client(GetPositionFK, "/compute_fk")
        self.move = ActionClient(self, MoveGroup, "/move_action")
        self.limits = None          # (lower, upper) arm joint limits used by the sweep

    def call(self, cli, req, timeout=10.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            raise RuntimeError(f"service {cli.srv_name} not available -- is move_group running "
                               f"(same ROS_DOMAIN_ID, same tool set)?")
        fut = cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=timeout)
        if fut.result() is None:
            raise RuntimeError(f"{cli.srv_name}: no response within {timeout} s")
        return fut.result()

    def scene(self, objs, remove=False):
        ps = PlanningScene(is_diff=True)
        for o in objs:
            if remove:
                o = CollisionObject(id=o.id, operation=CollisionObject.REMOVE)
                o.header.frame_id = FRAME
            ps.world.collision_objects.append(o)
        return self.call(self.apply, ApplyPlanningScene.Request(scene=ps)).success

    def world_ids(self):
        req = GetPlanningScene.Request()
        req.components.components = PlanningSceneComponents.WORLD_OBJECT_NAMES
        return {o.id for o in self.call(self.getps, req).scene.world.collision_objects}

    @staticmethod
    def state(q):
        rs = RobotState(is_diff=True)
        rs.joint_state = JointState(name=list(kin.ARM_JOINTS), position=[float(v) for v in q])
        return rs

    def check(self, q):
        r = self.call(self.valid, GetStateValidity.Request(robot_state=self.state(q), group_name=GROUP))
        pairs = sorted({f"{c.contact_body_1}<->{c.contact_body_2}" for c in r.contacts})
        return r.valid, pairs[:4]

    def fk_tool0(self, q):
        req = GetPositionFK.Request(fk_link_names=["tool0"], robot_state=self.state(q))
        req.header.frame_id = FRAME
        r = self.call(self.fk, req)
        if r.error_code.val != MoveItErrorCodes.SUCCESS or not r.pose_stamped:
            return None
        p = r.pose_stamped[0].pose
        R = kin.quat_xyzw_to_R([p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w])
        return kin.make_T(R, [p.position.x, p.position.y, p.position.z])

    def plan(self, q_from, q_to, attempts=5, time_s=10.0, timeout=60.0):
        if not self.move.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("/move_action not available")
        req = MotionPlanRequest(group_name=GROUP, num_planning_attempts=attempts, allowed_planning_time=time_s,
                                max_velocity_scaling_factor=1.0, max_acceleration_scaling_factor=1.0)
        req.start_state = self.state(q_from)
        c = Constraints()
        for n, v in zip(kin.ARM_JOINTS, q_to):
            c.joint_constraints.append(JointConstraint(joint_name=n, position=float(v), tolerance_above=1e-3,
                                                       tolerance_below=1e-3, weight=1.0))
        req.goal_constraints = [c]
        opts = PlanningOptions(plan_only=True)
        opts.planning_scene_diff.is_diff = True          # same as omy_to_ur16e.py's sync goal
        opts.planning_scene_diff.robot_state.is_diff = True
        goal = MoveGroup.Goal(request=req, planning_options=opts)
        gf = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, gf, timeout_sec=timeout)
        h = gf.result()
        if h is None or not h.accepted:
            return False, None, "goal rejected"
        rf = h.get_result_async()
        rclpy.spin_until_future_complete(self, rf, timeout_sec=timeout)
        if rf.result() is None:
            return False, None, "no result (timeout)"
        res = rf.result().result
        code = res.error_code.val
        jt = res.planned_trajectory.joint_trajectory
        pts = jt.points
        dur = (pts[-1].time_from_start.sec + pts[-1].time_from_start.nanosec * 1e-9) if pts else None
        ok = code == MoveItErrorCodes.SUCCESS
        if ok and self.limits is not None and pts:
            # move_group only knows the URDF limits; the sweep may have used tighter ones
            # (robot.joint_limits_deg, e.g. elbow +-160 per the UR16e manual)
            idx = [list(jt.joint_names).index(n) for n in kin.ARM_JOINTS]
            Q = np.array([[p.positions[i] for i in idx] for p in pts])
            lo, hi = self.limits
            bad = np.where((Q < lo - 1e-6) | (Q > hi + 1e-6))
            if len(bad[0]):
                j = kin.ARM_JOINTS[bad[1][0]]
                return False, dur, f"leaves the YAML joint limit on {j} ({np.degrees(Q[bad][0]):.1f} deg)"
        return ok, dur, ERR.get(code, str(code))


def validate(node, C, R, c, keep, do_plan, attempts, retries):
    """One candidate. Returns (passed, summary str)."""
    # by NAME (names are unique): mount_index of older results pointed into a --mounts subset
    mount = next((m for m in C.mounts if m["name"] == c["mount"]), None)
    if mount is None:
        return False, f"mount {c['mount']!r} is not in {R['cell']} any more (YAML edited since place_base?)"
    spec = c.get("mount_spec")
    if spec and (spec["kind"] != mount["kind"] or abs(spec["height"] - float(mount.get("height", 0.0))) > 1e-9):
        return False, f"mount {c['mount']!r} changed in the YAML since place_base ({spec} vs {mount})"
    rows = c.get("targets") or []
    if not rows:
        return False, "results.json has no per-target joints for this candidate (made by an older place_base.py; rerun it)"
    T_cb = np.array(c["T_cell_base_link"])
    T_re = C.mount_pose(mount, np.array([c["u"]]), np.array([c["v"]]), np.array([c["yaw"]]))[0]
    if np.abs(T_re - T_cb).max() > 1e-5:
        return False, "the mount pose rebuilt from the YAML differs from results.json (YAML edited since place_base?)"
    T_t0_tcp = np.array(R["tool0_to_tcp"])
    exp = kin.inv_T(T_cb) @ C.T_tcp @ kin.inv_T(T_t0_tcp)           # base_link -> tool0 per target (cell order)
    names = [d["target"] for d in rows]
    if names != C.names:
        return False, f"target names in results.json {names} differ from the cell YAML {C.names}"
    Q = {d["target"]: np.array(d["q_rad"]) for d in rows}
    print(f"\n=== {c['mount']} u={c['u']:.3f} v={c['v']:.3f} yaw={np.degrees(c['yaw']):.1f} deg  "
          f"branch {c['branch_name']}  sweep score {c['score']:.3f}")
    print(f"    {c['placement']['xacro_origin']}")
    objs = cell_objects(C, mount, T_cb)
    ok_all = True
    try:
        added = node.scene(objs)
        missing = sorted({o.id for o in objs} - node.world_ids())
        print(f"    planning scene: {len(objs)} objects in {FRAME} -> {'OK' if added and not missing else 'FAILED'}"
              + (f", missing {missing}" if missing else ""))
        if not added or missing:
            return False, "move_group did not take the cell objects (does its model have base_link?)"
        print("    target            FK tool0 err [mm / deg]   state")
        valid = {}
        for i, n in enumerate(names):
            T = node.fk_tool0(Q[n])
            if T is None:
                fk = "FK FAILED"
                ok_all = False
            else:
                e_p = np.linalg.norm(T[:3, 3] - exp[i][:3, 3]) * 1e3
                e_r = np.degrees(kin.rot_angle(T[:3, :3], exp[i][:3, :3]))
                fk = f"{e_p:8.3f} / {e_r:6.3f}"
                ok_all &= bool(e_p < 1.0 and e_r < 0.1)
            v, contacts = node.check(Q[n])
            valid[n] = v
            ok_all &= bool(v)
            print(f"    {n:16s}  {fk:24s}  {'OK' if v else 'IN COLLISION ' + ', '.join(contacts)}")
        if do_plan:
            order = R["order"]
            moves = list(zip(order, order[1:] + ([order[0]] if R["cyclic"] else [])))
            print("    move                               planned   duration [s]")
            for f, t in moves:
                if not (valid[f] and valid[t]):
                    print(f"    {f:16s} -> {t:16s}  SKIP   -   ({'start' if not valid[f] else 'goal'} state invalid)")
                    ok_all = False
                    continue
                # OMPL is randomised and its edge collision checks are coarse: in a tight
                # cell a found path often fails MoveIt's ValidateSolution (7 of 9 failures
                # on 2026-10-16) and a fresh request succeeds. Retry before calling it FAIL.
                msgs = []
                for k in range(retries):
                    ok, dur, msg = node.plan(Q[f], Q[t], attempts=attempts)
                    msgs.append(msg)
                    if ok:
                        break
                ok_all &= ok
                tries = "" if len(msgs) == 1 else f" (try {len(msgs)}/{retries})"
                print(f"    {f:16s} -> {t:16s}  {'OK  ' if ok else 'FAIL'}   "
                      f"{'-' if dur is None or not ok else f'{dur:.2f}'}   {'' if ok else '/'.join(msgs)}{tries if ok else ''}")
    finally:
        if not keep:
            try:
                node.scene(objs, remove=True)     # always: a failed ADD can still be partial
            except RuntimeError as e:
                print(f"    (cleanup: {e})")
    return ok_all, "PASS" if ok_all else "FAIL"


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("results", help="results.json from place_base.py")
    ap.add_argument("--pick", default="recommended", help="recommended | best | N (1 = best) | all")
    ap.add_argument("--keep", action="store_true", help="leave the (last) cell in the planning scene")
    ap.add_argument("--no-plan", action="store_true", help="only FK + state validity")
    ap.add_argument("--attempts", type=int, default=5, help="num_planning_attempts per request")
    ap.add_argument("--retries", type=int, default=3, help="planning requests per move before FAIL")
    a = ap.parse_args()
    with open(a.results) as f:
        R = json.load(f)
    top = R.get("top") or []
    if not top:
        sys.exit("results.json has no feasible candidate -- nothing to validate (see the blocker table in report.md)")
    if a.pick in ("recommended", "best"):
        cands = [R[a.pick]]
    elif a.pick == "all":
        cands = top
    else:
        try:
            k = int(a.pick)
        except ValueError:
            sys.exit(f"--pick must be recommended | best | N | all, got {a.pick!r}")
        if not 1 <= k <= len(top):
            sys.exit(f"--pick {k}: results.json has {len(top)} candidates")
        cands = [top[k - 1]]
    C = cellmod.Cell(R["cell"])
    if R.get("cell_sha256") and R["cell_sha256"] != C.sha256:
        print(f"WARNING: {R['cell']} changed since place_base.py wrote {a.results} -- each candidate is "
              "re-checked against the YAML below, but rerun place_base.py if targets or obstacles changed")

    rclpy.init()
    node = Validator()
    jl = R.get("joint_limits_rad") or {}
    if "lower" in jl and "upper" in jl:
        node.limits = (np.array(jl["lower"]), np.array(jl["upper"]))
    results = []                          # (candidate, True/False/None = not checked, message)
    try:
        for i, c in enumerate(cands):
            keep = a.keep and i == len(cands) - 1
            try:
                ok, msg = validate(node, C, R, c, keep, not a.no_plan, a.attempts, a.retries)
            except RuntimeError as e:     # a service vanished: the rest cannot be checked either
                print(f"\nERROR: {e}")
                results.append((c, None, f"ERROR: {e}"))
                results += [(d, None, "not checked") for d in cands[i + 1:]]
                break
            results.append((c, ok, msg))
            print(f"    -> {msg}")
    finally:
        node.destroy_node()
        rclpy.shutdown()

    def rank(c):
        return next((i + 1 for i, t in enumerate(top) if t is c or (
            t["mount"] == c["mount"] and abs(t["u"] - c["u"]) < 1e-9 and abs(t["v"] - c["v"]) < 1e-9
            and abs(t["yaw"] - c["yaw"]) < 1e-9)), "-")

    passed = [c for c, ok, _ in results if ok]
    print("\nSUMMARY  (rank = place in `top`; robust = robust_score from place_base.py)")
    for c, ok, msg in results:
        print(f"  #{rank(c):<3} {c['mount']:14s} u={c['u']:.3f} v={c['v']:.3f} "
              f"robust={'-' if c.get('robust_score') is None else format(c['robust_score'], '.3f')}  {msg}")
    if passed:
        first = passed[0]
        sturdy = max(passed, key=lambda c: -1e9 if c.get("robust_score") is None else c["robust_score"])
        print(f"\nRESULT: PASS ({len(passed)}/{len(results)})\n"
              f"  best-scored passing candidate:  #{rank(first)} {first['mount']} u={first['u']:.3f} v={first['v']:.3f}\n"
              f"    {first['placement']['xacro_origin']}")
        if sturdy is not first:
            print(f"  most robust passing candidate:  #{rank(sturdy)} {sturdy['mount']} u={sturdy['u']:.3f} "
                  f"v={sturdy['v']:.3f}\n    {sturdy['placement']['xacro_origin']}")
        sys.exit(0)
    if any(ok is None for _, ok, _ in results):
        print("\nRESULT: INCOMPLETE (check cut short, nothing passed before that)")
        sys.exit(2)
    print("\nRESULT: FAIL (no candidate passed)")
    sys.exit(1)


if __name__ == "__main__":
    main()
