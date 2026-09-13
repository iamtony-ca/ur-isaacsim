#!/usr/bin/env python3
"""Save wrist-camera frames at the grasp moment of every pick&place cycle.

The point is to make the grasp checkable by LOOKING, per cycle, instead of
trusting "SUCCESS" from the state machine. The previous dataset was collected
with the jaws open the whole way through and the demo still reported 6/6, because
the attach fires on a joint threshold, not on contact (HISTORY.md 28).

Triggers off finger_joint alone, so it does not have to be wired into the state
machine or kept in sync with it:
    rising through CLOSE_TH -> closed_<n>.jpg  (jaws shut on the part)
    +3 s later            -> lift_<n>.jpg     (part should be off the table)
    falling through 0.20  -> open_<n>.jpg     (release)
"""
import sys, time
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSPresetProfiles
from sensor_msgs.msg import Image, JointState
from PIL import Image as PImage

OUT = sys.argv[1] if len(sys.argv) > 1 else "/tmp"
SECS = float(sys.argv[2]) if len(sys.argv) > 2 else 1800.0
# Threshold for "the jaws have shut on the part". MUST stay below grip_closed.
# It was hardcoded at 0.70, which stopped firing the moment grip_closed dropped
# from 0.8 to the measured stall 0.599 (HISTORY.md 33) -- the watcher went blind
# and the collection ran with no grasp evidence at all. Pass it explicitly so it
# tracks the demo's value instead of drifting out of sync again.
CLOSE_TH = float(sys.argv[3]) if len(sys.argv) > 3 else 0.50


def decode(m):
    a = np.frombuffer(m.data, np.uint8).reshape(m.height, m.step)[:, :m.width * 3]
    a = a.reshape(m.height, m.width, 3)
    return a[:, :, ::-1] if m.encoding.startswith("bgr") else a


def main():
    rclpy.init()
    n = Node("grasp_watch")
    n.set_parameters([rclpy.parameter.Parameter("use_sim_time", rclpy.Parameter.Type.BOOL, True)])
    st = {"fj": None, "img": None, "prev": None, "cyc": 0, "pending": None}

    n.create_subscription(Image, "/camera/color/image_raw",
                          lambda m: st.__setitem__("img", decode(m)),
                          QoSPresetProfiles.SENSOR_DATA.value)

    def save(tag):
        if st["img"] is None:
            print(f"   {tag}: no image yet", flush=True); return
        f = f"{OUT}/{tag}.jpg"
        PImage.fromarray(st["img"]).save(f, quality=95)
        print(f"   saved {f}  fj={st['fj']:.3f}", flush=True)

    def on_js(m):
        if "finger_joint" not in m.name:
            return
        v = float(m.position[m.name.index("finger_joint")])
        st["fj"], p = v, st["prev"]
        st["prev"] = v
        if p is None:
            return
        if p <= CLOSE_TH < v:                   # jaws just shut
            st["cyc"] += 1
            save(f"closed_{st['cyc']}")
            st["pending"] = time.time() + 3.0   # wall clock: only a delay, not a timestamp
        elif p >= 0.20 > v:                     # jaws just released
            save(f"open_{st['cyc']}")

    n.create_subscription(JointState, "/joint_states", on_js, 10)
    print(f"watching for grasp events -> {OUT}", flush=True)
    end = time.time() + SECS
    while rclpy.ok() and time.time() < end:
        rclpy.spin_once(n, timeout_sec=0.05)
        if st["pending"] and time.time() >= st["pending"]:
            st["pending"] = None
            save(f"lift_{st['cyc']}")
    rclpy.shutdown()


main()
