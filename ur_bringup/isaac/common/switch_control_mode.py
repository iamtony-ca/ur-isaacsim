#!/usr/bin/env python3
"""Switch the UR16e arm between TRAJECTORY and STREAMING control.

Two arm controllers claim the same position command interfaces, so exactly one
may be active at a time:

    trajectory : scaled_joint_trajectory_controller   (MoveIt / cuMotion plan+execute)
    streaming  : forward_position_controller          (MoveIt Servo teleop, learned policies)

Usage (control stack already up):
    python3 .../isaac/common/switch_control_mode.py streaming
    python3 .../isaac/common/switch_control_mode.py trajectory
    python3 .../isaac/common/switch_control_mode.py --status

Notes
-----
* Switching to `streaming` while the arm is mid-trajectory is refused by
  controller_manager unless the trajectory controller is deactivated first --
  which is exactly what this script does (deactivate + activate in one atomic
  switch request).
* The streaming controller holds its LAST commanded value. On activation
  ros2_control seeds it from the current joint state, so the arm does not jump;
  but do not leave it active with nothing publishing for long -- start Servo (or
  the policy runner) right after switching.

See ur_bringup/docs/plan_il_vla.md 2.3 for why the two modes exist.
"""
import argparse
import sys

import rclpy
from controller_manager_msgs.srv import ListControllers, SwitchController

TRAJ = "scaled_joint_trajectory_controller"
STREAM = "forward_position_controller"
MODES = {"trajectory": (TRAJ, STREAM), "streaming": (STREAM, TRAJ)}


def _call(node, cli, req, what, timeout=10.0):
    if not cli.wait_for_service(timeout_sec=timeout):
        print(f"FAIL: service {cli.srv_name} unavailable "
              f"(is the control stack running?)")
        return None
    fut = cli.call_async(req)
    rclpy.spin_until_future_complete(node, fut, timeout_sec=timeout)
    if fut.result() is None:
        print(f"FAIL: {what} timed out")
        return None
    return fut.result()


def show_status(node, cm):
    res = _call(node, node.create_client(ListControllers, f"{cm}/list_controllers"),
                ListControllers.Request(), "list_controllers")
    if res is None:
        return 1
    print(f"{'controller':<40s} {'state':<12s} type")
    for c in res.controller:
        mark = " <" if c.name in (TRAJ, STREAM) else ""
        print(f"{c.name:<40s} {c.state:<12s} {c.type}{mark}")
    active = {c.name for c in res.controller if c.state == "active"}
    if STREAM in active and TRAJ in active:
        print("\n!! BOTH arm controllers active -- this should not happen "
              "(conflicting command interfaces).")
    elif STREAM in active:
        print("\nmode: STREAMING (Servo teleop / policy)")
    elif TRAJ in active:
        print("\nmode: TRAJECTORY (MoveIt / cuMotion)")
    else:
        print("\nmode: none -- no arm controller active")
    return 0


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("mode", nargs="?", choices=sorted(MODES), help="target control mode")
    ap.add_argument("--status", action="store_true", help="print controller states and exit")
    ap.add_argument("--controller-manager", default="/controller_manager")
    args = ap.parse_args()
    if not args.mode and not args.status:
        ap.error("give a mode (trajectory|streaming) or --status")

    rclpy.init()
    node = rclpy.create_node("switch_control_mode")
    cm = args.controller_manager.rstrip("/")
    try:
        if args.status:
            return show_status(node, cm)

        activate, deactivate = MODES[args.mode]

        # Only deactivate what is actually running, else the switch is rejected.
        listed = _call(node, node.create_client(ListControllers, f"{cm}/list_controllers"),
                       ListControllers.Request(), "list_controllers")
        if listed is None:
            return 1
        states = {c.name: c.state for c in listed.controller}
        if activate not in states:
            print(f"FAIL: controller '{activate}' is not loaded.\n"
                  f"      Spawn it first, e.g.:\n"
                  f"      ros2 run controller_manager spawner {activate} "
                  f"-c {cm} --inactive")
            return 1
        if states.get(activate) == "active":
            print(f"already in {args.mode} mode ({activate} active)")
            return 0

        req = SwitchController.Request()
        req.activate_controllers = [activate]
        req.deactivate_controllers = [deactivate] if states.get(deactivate) == "active" else []
        # STRICT: fail loudly rather than half-switch and leave the arm uncommanded.
        req.strictness = SwitchController.Request.STRICT
        req.activate_asap = True
        req.timeout.sec = 5

        res = _call(node, node.create_client(SwitchController, f"{cm}/switch_controller"),
                    req, "switch_controller")
        if res is None:
            return 1
        if not res.ok:
            print(f"FAIL: switch to {args.mode} rejected by controller_manager")
            return 1
        print(f"OK: mode = {args.mode}  (activated {activate}"
              + (f", deactivated {deactivate}" if req.deactivate_controllers else "") + ")")
        return 0
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
