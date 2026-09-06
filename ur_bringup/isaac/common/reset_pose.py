#!/usr/bin/env python3
"""Reset the UR16e arm to a named pose by sending a trajectory DIRECTLY to the
scaled_joint_trajectory_controller (bypassing MoveIt).

Why direct-to-controller: MoveIt refuses to plan FROM a start state that is out of
joint bounds. Repeated interactive planning can let a wrist joint accumulate past
+/-2*pi ("Joint 'wrist_2_joint' ... outside bounds"), which then blocks every new
plan (incl. RViz named states). This script commands the joints back to a valid
pose regardless, recovering the robot.

Usage (sim or real, after the control stack is up):
    python3 .../isaac/common/reset_pose.py [ready|home|up|zero]   # default: home

Named poses home/up/zero match ur_moveit_config's SRDF group_states; `ready` is
ours (see below).

*** For TELEOP / policy rollouts use `ready`, not `home`. ***
home/up/zero all have elbow_joint = 0, i.e. the arm fully extended, which is an
ELBOW SINGULARITY. MoveIt Servo refuses to move there:
    [servo] Very close to a singularity, emergency stop     (status code 2)
so gamepad/keyboard teleop looks "dead" even though everything is wired up.
`ready` bends the elbow ~90 deg, well clear of the singular set.

NOTE: this talks to scaled_joint_trajectory_controller, so the arm must be in
TRAJECTORY mode. If you are in streaming/teleop mode, switch first:
    python3 .../isaac/common/switch_control_mode.py trajectory
    python3 .../isaac/common/reset_pose.py ready
    python3 .../isaac/common/switch_control_mode.py streaming
"""
import sys

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration

ARM = ["shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
       "wrist_1_joint", "wrist_2_joint", "wrist_3_joint"]
POSES = {
    # Teleop/policy start pose: elbow bent ~90 deg, tool pointing down at the
    # table in front of the base. Away from the elbow singularity (elbow~0) AND
    # from the wrist singularity (wrist_2 ~ 0). Use this for Servo teleop and as
    # the per-episode reset pose when recording demos.
    "ready": [0.0, -1.5707, 1.5707, -1.5707, -1.5707, 0.0],
    "home": [0.0, -1.5707, 0.0, 0.0, 0.0, 0.0],
    "up":   [0.0, -1.5707, 0.0, -1.5707, 0.0, 0.0],
    "zero": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
}
ACTION = "/scaled_joint_trajectory_controller/follow_joint_trajectory"


def main():
    name = sys.argv[1] if len(sys.argv) > 1 else "home"
    move_secs = int(sys.argv[2]) if len(sys.argv) > 2 else 6
    if name not in POSES:
        print(f"unknown pose '{name}'. choose from: {list(POSES)}")
        return
    goal = POSES[name]

    rclpy.init()
    n = Node("reset_pose")
    ac = ActionClient(n, FollowJointTrajectory, ACTION)
    if not ac.wait_for_server(timeout_sec=10.0):
        print(f"controller action not available: {ACTION} (is the control stack up?)")
        rclpy.shutdown()
        return

    jt = JointTrajectory()
    jt.joint_names = ARM
    p = JointTrajectoryPoint()
    p.positions = [float(v) for v in goal]
    p.time_from_start = Duration(sec=move_secs)
    jt.points = [p]
    g = FollowJointTrajectory.Goal()
    g.trajectory = jt

    print(f"resetting to '{name}' {goal} over {move_secs}s (direct to controller)...")
    fut = ac.send_goal_async(g)
    rclpy.spin_until_future_complete(n, fut)
    gh = fut.result()
    if not gh.accepted:
        print("goal REJECTED by controller")
        rclpy.shutdown()
        return
    rf = gh.get_result_async()
    rclpy.spin_until_future_complete(n, rf)
    print("done, error_code:", rf.result().result.error_code, "(0 = SUCCESSFUL)")
    rclpy.shutdown()


if __name__ == "__main__":
    main()
