#!/usr/bin/env python3
"""Does the simulated 2F-85 actually HOLD an object?  (T2-1 gate test)

Everything downstream -- teleop demo collection, IL training, the pick&place
state machine -- assumes the gripper can pick a part up. That had never been
tested: gripper_demo.py only checks that finger_joint moves, with no object in
the scene. The 2F-85 is driven through PhysX mimic joints, which makes contact
force transmission the thing most likely to quietly not work.

This script closes that gap. It drives a scripted approach -> close -> lift and
measures the OBJECT's height from /scene/object_pose (ground truth published by
the Isaac scene, verification only -- never a policy input).

    PASS  object rises with the gripper            -> physical grasping works
    FAIL  gripper closes but the object stays put  -> see the tuning ladder below

Prereqs (4 terminals):
  1. Isaac:   /isaac-sim/python.sh .../isaac/common/ur16e_isaac_ros2.py \
                  --asset-path .../assets/ur16e_2f85_d405.usd --scene pick_place --table
  2. Control: ros2 launch ur_bringup ur16e_2f85_d405.launch.py use_sim:=true
  3. MoveIt:  ros2 launch ur_bringup ur16e_2f85_d405_moveit.launch.py use_sim:=true launch_rviz:=false
  4. This:    python3 .../isaac/ur16e_2f85/grasp_test.py

Why MoveIt is needed even though this is a physics test: we only use its
/compute_ik service to turn "tool0 above the object, pointing down" into joint
angles. EXECUTION still goes straight to the trajectory controller, so no
planner sits in the loop and a physics failure cannot be confused with a
planning failure. Hand-written joint waypoints were tried first and are a trap:
the UR16e wrist has a ~0.174 m lateral offset, so shoulder_pan=0 puts tool0 at
y=+0.174, not y=0 -- the gripper closes 17 cm away from the part and the test
"fails" for the wrong reason.

If it FAILS, tune in this order (cheapest first) -- re-run Isaac with:
  1. friction   --object-friction 2.0      (most common cause)
  2. mass       --object-mass 0.05         (lighter part is easier to hold)
  3. size       --object-size 0.04,0.04,0.04
  4. gripper drive gain: finger_joint stiffness in the USD (build_ur16e_2f85.py)
  5. last resort: to_do.md D5 fallback -- attach the object with a fixed joint on
     grasp instead of relying on contact physics.
"""
import argparse
import math
import sys

import rclpy
from tf2_ros import Buffer, TransformListener
from control_msgs.action import FollowJointTrajectory, GripperCommand
from geometry_msgs.msg import PoseStamped
from rclpy.action import ActionClient
from sensor_msgs.msg import JointState
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (BoundingVolume, CollisionObject, Constraints, MotionPlanRequest,
                             OrientationConstraint, PlanningOptions, PositionConstraint)
from shape_msgs.msg import SolidPrimitive
from std_srvs.srv import Trigger
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
TRAJ_ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"
GRIP_ACTION = "/gripper_controller/gripper_cmd"

READY = [0.0, -1.5707, 1.5707, -1.5707, -1.5707, 0.0]
GROUP = "ur_manipulator"
# The tool-down orientation is MEASURED from TF at the READY pose, not assumed.
# Guessing a quaternion here is a trap: (1,0,0,0) "180 deg about X" leaves the
# 2F-85 tilted, the pads land 160 mm apart at different heights, and the cube
# wedges the fingers open (finger_joint goes NEGATIVE) instead of being gripped.
# READY is a known tool-pointing-down arm configuration, so whatever tool0's
# orientation is there is by definition the orientation we want at the grasp.


class GraspTest:
    def __init__(self, node):
        self.n = node
        self.js = {}
        self.obj = None
        node.create_subscription(JointState, "/joint_states",
                                 lambda m: self.js.update(dict(zip(m.name, m.position))), 10)
        node.create_subscription(PoseStamped, "/scene/object_pose", self._on_obj, 10)
        self.traj = ActionClient(node, FollowJointTrajectory, TRAJ_ACTION)
        self.grip = ActionClient(node, GripperCommand, GRIP_ACTION)
        self.reset = node.create_client(Trigger, "/scene/reset_episode")
        self.move = ActionClient(node, MoveGroup, "/move_action")
        # MoveIt does not know about the Isaac object, so without this the planner
        # sweeps the open gripper straight through the part on the way in and bats
        # it across the table -- after which nothing downstream can work.
        self.co_pub = node.create_publisher(CollisionObject, "/collision_object", 10)
        self.tf = Buffer()
        TransformListener(self.tf, node)
        self.tool_down_quat = None

    def capture_tool_orientation(self, timeout=8.0):
        """Read tool0's orientation in base_link (call at the READY pose)."""
        end = self.n.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and self.n.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)
            try:
                r = self.tf.lookup_transform("base_link", "tool0", rclpy.time.Time()).transform.rotation
            except Exception:
                continue
            self.tool_down_quat = (r.x, r.y, r.z, r.w)
            return self.tool_down_quat
        return None

    def finger_drop(self, timeout=8.0):
        """How far BELOW tool0 the fingertips sit, measured live from TF.

        Hardcoding this is a trap: it depends on the coupling standoff, the
        gripper USD and how far the fingers are open. Measuring it keeps the
        grasp height right when any of those change.
        """
        end = self.n.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and self.n.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)
            try:
                t0 = self.tf.lookup_transform("base_link", "tool0", rclpy.time.Time())
                lf = self.tf.lookup_transform("base_link", "robotiq_85_left_finger_tip_link",
                                              rclpy.time.Time())
                rf = self.tf.lookup_transform("base_link", "robotiq_85_right_finger_tip_link",
                                              rclpy.time.Time())
            except Exception:
                continue
            tip_z = 0.5 * (lf.transform.translation.z + rf.transform.translation.z)
            return t0.transform.translation.z - tip_z
        return None

    def _on_obj(self, m):
        self.obj = (m.pose.position.x, m.pose.position.y, m.pose.position.z)

    def spin(self, sec):
        end = self.n.get_clock().now().nanoseconds + int(sec * 1e9)
        while rclpy.ok() and self.n.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)

    def wait_obj(self, timeout=10.0):
        end = self.n.get_clock().now().nanoseconds + int(timeout * 1e9)
        while rclpy.ok() and self.obj is None and self.n.get_clock().now().nanoseconds < end:
            rclpy.spin_once(self.n, timeout_sec=0.05)
        return self.obj

    def scene_object(self, add, xyz=None, size=(0.05, 0.05, 0.05), oid="pick_object"):
        """Add/remove the part in MoveIt's planning scene.

        ADD before the approach so the planner routes around it; REMOVE just
        before the final vertical descent, because a grasp must touch the part
        and the planner would otherwise refuse every pose that does.
        """
        co = CollisionObject()
        co.header.frame_id = "base_link"
        co.id = oid
        if add:
            sp = SolidPrimitive(); sp.type = SolidPrimitive.BOX
            sp.dimensions = [float(v) for v in size]
            co.primitives.append(sp)
            pose = PoseStamped().pose
            pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
            pose.orientation.w = 1.0
            co.primitive_poses.append(pose)
            co.operation = CollisionObject.ADD
        else:
            co.operation = CollisionObject.REMOVE
        for _ in range(5):            # latch-ish: monitor may miss a single message
            self.co_pub.publish(co)
            self.spin(0.1)
        self.spin(0.5)

    def goto_pose(self, xyz, tol=0.005, timeout=60.0):
        """Plan+execute so tool0 reaches xyz with the captured tool-down orientation.

        Raw /compute_ik + direct trajectory was tried first and is WRONG here:
        the returned solution's FK did not match the requested pose (tool0 landed
        ~10 cm off) even though the controller reported "Goal reached". Letting
        move_group plan and execute makes reaching the pose its responsibility,
        and it also keeps the arm from sweeping through the table/part on the way.
        """
        req = MotionPlanRequest()
        req.group_name = GROUP
        req.num_planning_attempts = 10
        req.allowed_planning_time = 5.0
        req.max_velocity_scaling_factor = 0.3
        req.max_acceleration_scaling_factor = 0.3

        c = Constraints()
        pc = PositionConstraint()
        pc.header.frame_id = "base_link"
        pc.link_name = "tool0"
        pc.weight = 1.0
        vol = BoundingVolume()
        sp = SolidPrimitive(); sp.type = SolidPrimitive.SPHERE; sp.dimensions = [tol]
        vol.primitives.append(sp)
        pose = PoseStamped().pose
        pose.position.x, pose.position.y, pose.position.z = (float(v) for v in xyz)
        pose.orientation.w = 1.0
        vol.primitive_poses.append(pose)
        pc.constraint_region = vol
        c.position_constraints.append(pc)

        oc = OrientationConstraint()
        oc.header.frame_id = "base_link"
        oc.link_name = "tool0"
        q = self.tool_down_quat or (0.0, 0.0, 0.0, 1.0)
        (oc.orientation.x, oc.orientation.y, oc.orientation.z, oc.orientation.w) = q
        oc.absolute_x_axis_tolerance = 0.05
        oc.absolute_y_axis_tolerance = 0.05
        oc.absolute_z_axis_tolerance = 0.05
        oc.weight = 1.0
        c.orientation_constraints.append(oc)
        req.goal_constraints.append(c)

        goal = MoveGroup.Goal()
        goal.request = req
        goal.planning_options = PlanningOptions()
        goal.planning_options.plan_only = False

        fut = self.move.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=15.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("  ! move_action goal rejected"); return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.n, rf, timeout_sec=timeout)
        res = rf.result()
        code = res.result.error_code.val if res else None
        if code != 1:
            print(f"  ! move_action failed (error_code {code})"); return False
        return True

    def goto_joints(self, q, secs=4.0):
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = JointTrajectory(joint_names=ARM)
        goal.trajectory.points = [JointTrajectoryPoint(
            positions=list(q),
            time_from_start=Duration(sec=int(secs), nanosec=int((secs % 1) * 1e9)))]
        fut = self.traj.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("  ! trajectory goal rejected")
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.n, rf, timeout_sec=secs + 10.0)
        return True

    def gripper(self, position, effort=100.0, settle=2.0):
        goal = GripperCommand.Goal()
        goal.command.position = float(position)
        goal.command.max_effort = float(effort)
        fut = self.grip.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.n, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            print("  ! gripper goal rejected")
            return False
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(self.n, rf, timeout_sec=settle + 10.0)
        self.spin(settle)
        return True


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--close", type=float, default=0.55,
                    help="finger_joint target when gripping [rad] (0=open, ~0.8=fully closed). "
                         "For a 50 mm cube the fingers meet well before 0.8.")
    ap.add_argument("--effort", type=float, default=100.0)
    ap.add_argument("--lift-threshold", type=float, default=0.03,
                    help="object must rise at least this much [m] to count as grasped")
    ap.add_argument("--approach-height", type=float, default=0.30,
                    help="tool0 height above the object for the pre-grasp pose [m]. Must clear the "
                         "OPEN fingers (~0.12 m below tool0) plus the part, because the object is "
                         "NOT in MoveIt's planning scene -- the planner will happily sweep the "
                         "gripper straight through it on the way in and bat it across the table.")
    ap.add_argument("--grasp-offset", type=float, default=None,
                    help="tool0 height above the object CENTRE at grasp [m]. Default: MEASURED "
                         "from TF (tool0 -> fingertip drop) so the pads straddle the object centre.")
    ap.add_argument("--object-size", type=float, nargs=3, default=(0.05, 0.05, 0.05),
                    help="object x y z size [m] used for the planning-scene collision box; "
                         "must match the Isaac --object-size")
    ap.add_argument("--grasp-bias", type=float, default=0.0,
                    help="extra [m] added to the measured grasp height; negative grips lower")
    args = ap.parse_args()

    rclpy.init()
    node = rclpy.create_node("grasp_test")
    t = GraspTest(node)
    try:
        for cli, name in ((t.traj, TRAJ_ACTION), (t.grip, GRIP_ACTION)):
            if not cli.wait_for_server(timeout_sec=15.0):
                print(f"FAIL: action server not available: {name}  (control stack up?)")
                return 1
        if not t.move.wait_for_server(timeout_sec=20.0):
            print("FAIL: /move_action unavailable — start MoveIt "
                  "(ur16e_2f85_d405_moveit.launch.py use_sim:=true)")
            return 1
        if t.wait_obj() is None:
            print("FAIL: no /scene/object_pose. Start Isaac with --scene pick_place")
            return 1

        if t.reset.wait_for_service(timeout_sec=5.0):
            f = t.reset.call_async(Trigger.Request())
            rclpy.spin_until_future_complete(node, f, timeout_sec=10.0)
            t.spin(1.5)

        ox, oy, oz = t.obj
        print(f"object start : ({ox:.4f}, {oy:.4f}, {oz:.4f})")
        z0 = oz

        print("open gripper ..."); t.gripper(0.0)
        print("-> ready ...");     t.goto_joints(READY)
        t.spin(1.0)
        q = t.capture_tool_orientation()
        if q is None:
            print("FAIL: could not read tool0 orientation from TF"); return 1
        print(f"tool orientation (measured at READY): "
              f"({q[0]:+.4f}, {q[1]:+.4f}, {q[2]:+.4f}, {q[3]:+.4f})")

        drop = t.finger_drop()
        if args.grasp_offset is not None:
            g_off = args.grasp_offset
            print(f"grasp offset : {g_off:.4f} m (given)")
        elif drop is None:
            g_off = 0.13
            print(f"grasp offset : {g_off:.4f} m (TF unavailable, fallback)")
        else:
            g_off = drop + args.grasp_bias
            print(f"grasp offset : {g_off:.4f} m (measured tool0->fingertip drop {drop:.4f})")
        pre = (ox, oy, oz + args.approach_height)
        gra = (ox, oy, oz + g_off)
        print("add object to planning scene (approach must avoid it) ...")
        t.scene_object(True, (ox, oy, oz), size=args.object_size)

        print(f"-> pregrasp {tuple(round(v,3) for v in pre)} ...")
        if not t.goto_pose(pre):
            print("FAIL: could not reach pre-grasp pose"); return 1
        t.spin(1.0)
        moved = math.dist(t.obj[:2], (ox, oy))
        if moved > 0.02:
            print(f"  ! object shifted {moved:.3f} m during approach — the arm swept it. "
                  "Raise --approach-height.")
        # From here the gripper must TOUCH the part, so take it back out of the
        # planning scene; we are already centred directly above it, so the
        # remaining motion is a short vertical descent.
        print("remove object from planning scene (grasp must contact it) ...")
        t.scene_object(False)
        # Descend in small steps so the fingers come straight down onto the part
        # instead of the planner choosing an arbitrary (object-unaware) path.
        steps = 3
        for i in range(1, steps + 1):
            z = pre[2] + (gra[2] - pre[2]) * i / steps
            if not t.goto_pose((ox, oy, z)):
                print(f"FAIL: could not descend to z={z:.3f}"); return 1
        print(f"-> grasp    {tuple(round(v,3) for v in gra)} (descended in {steps} steps)")
        t.spin(1.0)
        print(f"object before close: {tuple(round(v, 4) for v in t.obj)}")
        # Diagnostics: where the pads actually are relative to the part. If the
        # tips are not straddling the object centre, the grasp geometry is wrong
        # and no amount of friction tuning will help.
        for fr in ("tool0", "robotiq_85_left_finger_tip_link", "robotiq_85_right_finger_tip_link"):
            try:
                tr = t.tf.lookup_transform("base_link", fr, rclpy.time.Time()).transform.translation
                print(f"  {fr:34s} ({tr.x:+.4f}, {tr.y:+.4f}, {tr.z:+.4f})")
            except Exception as e:
                print(f"  {fr:34s} n/a ({type(e).__name__})")

        print(f"close gripper to {args.close} rad ...")
        t.gripper(args.close, effort=args.effort, settle=2.5)
        print(f"  finger_joint = {t.js.get('finger_joint', float('nan')):.4f} rad")

        print("-> lift ...")
        t.goto_pose(pre)
        t.spin(2.0)

        z1 = t.obj[2]
        rise = z1 - z0
        print(f"\nobject z: {z0:.4f} -> {z1:.4f}   rise = {rise:+.4f} m "
              f"(threshold {args.lift_threshold})")
        if rise >= args.lift_threshold:
            print("RESULT: PASS — the gripper physically holds and lifts the object")
            return 0
        print("RESULT: FAIL — object did not come up with the gripper")
        print("        tuning ladder: friction -> mass -> size -> finger drive gain")
        print("        -> last resort: fixed-joint attach on grasp (to_do.md D5)")
        return 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
