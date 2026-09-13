#!/usr/bin/env python3
"""Judge one language-conditioned rollout: right block, right marker, let go?

judge_rollout.py hardcodes red->left, so it cannot score the 3-task dataset --
it would call "blue to the left" a success whenever the RED block happened to sit
on the left marker. This one takes the pair the instruction actually asked for.

The reason it reports more than SUCCESS/FAIL: with both blocks and both markers
present in every episode (collect_3tasks.sh), a failure has three very different
causes and they demand different fixes.

  WRONG_OBJECT   the distractor moved and the target did not -> the policy is not
                 reading the instruction. More data will not fix a language bug.
  WRONG_PLACE    target block moved, landed near the OTHER marker -> reading
                 "which object" but not "where".
  FAIL           right block, right marker, but not placed or not released ->
                 an execution problem (grasp, descent, timing), language is fine.

Collapsing those into one FAIL is how you end up "improving" the wrong thing.

Same tolerance (35 mm) and same released-jaws condition as judge_rollout.py, so
GR00T numbers stay comparable to the ACT baseline.
"""
import argparse
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState

TOL = 0.035
# Anything below this and the jaws are open; matches judge_rollout.py so the two
# agree on "released".
HELD = 0.4


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--object", required=True, help="target object name, e.g. red")
    ap.add_argument("--place", required=True, help="target place name, e.g. left")
    ap.add_argument("--objects", default="red,blue", help="all objects in the scene")
    ap.add_argument("--places", default="left,right", help="all markers in the scene")
    ap.add_argument("--timeout", type=float, default=15.0)
    a = ap.parse_args()

    objs = [o for o in a.objects.split(",") if o]
    plcs = [p for p in a.places.split(",") if p]
    if a.object not in objs or a.place not in plcs:
        print(f"UNKNOWN (target {a.object}/{a.place} not in scene {objs}/{plcs})")
        return 2

    rclpy.init()
    n = Node("judge_task")
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                         reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
    op, pp, grip = {}, {}, {}

    def obj_cb(k):
        return lambda m: op.__setitem__(k, m.pose.position)

    def plc_cb(k):
        return lambda m: pp.__setitem__(k, m.pose.position)

    for o in objs:
        n.create_subscription(PoseStamped, f"/scene/objects/{o}/pose", obj_cb(o), 10)
    for p in plcs:
        # Markers are latched, but subscribe both ways: a latched-only subscription
        # misses a republish and a volatile-only one misses the latch.
        for q in (latched, 10):
            n.create_subscription(PoseStamped, f"/scene/places/{p}/pose", plc_cb(p), q)
    n.create_subscription(
        JointState, "/joint_states",
        lambda m: grip.__setitem__("g", m.position[m.name.index("finger_joint")])
        if "finger_joint" in m.name else None, 10)

    want = len(objs) + len(plcs) + 1
    end = time.time() + a.timeout
    while time.time() < end and (len(op) + len(pp) + len(grip)) < want:
        rclpy.spin_once(n, timeout_sec=0.1)

    missing = ([f"objects/{o}" for o in objs if o not in op]
               + [f"places/{p}" for p in plcs if p not in pp]
               + ([] if grip else ["joint_states/finger_joint"]))
    if missing:
        print(f"UNKNOWN (missing {missing})")
        rclpy.shutdown()
        return 2

    def dist(a_, b_):
        return ((a_.x - b_.x) ** 2 + (a_.y - b_.y) ** 2) ** 0.5

    tgt, mark = op[a.object], pp[a.place]
    d = dist(tgt, mark)
    held = grip["g"] > HELD

    # Nearest marker for every object, so "what actually happened" is reported
    # rather than inferred from the single number we hoped for.
    layout = []
    for o in objs:
        near = min(plcs, key=lambda p: dist(op[o], pp[p]))
        layout.append(f"{o}->{near}@{dist(op[o], pp[near])*1000:.0f}mm")

    if d <= TOL and not held:
        tag, why = "SUCCESS", ""
    else:
        others = [o for o in objs if o != a.object]
        placed_wrong_obj = [o for o in others if dist(op[o], mark) <= TOL]
        other_marks = [p for p in plcs if p != a.place]
        tgt_on_other = [p for p in other_marks if dist(tgt, pp[p]) <= TOL]
        if placed_wrong_obj and d > TOL:
            tag = "WRONG_OBJECT"
            why = f" ({placed_wrong_obj[0]} is on {a.place} instead)"
        elif tgt_on_other:
            tag = "WRONG_PLACE"
            why = f" ({a.object} landed on {tgt_on_other[0]})"
        else:
            tag = "FAIL"
            why = " (still holding)" if held else " (too far)"

    print(f"{tag} target={a.object}->{a.place} d={d*1000:.0f}mm grip={grip['g']:.2f}"
          f"{why}  [{' '.join(layout)}]")
    rclpy.shutdown()
    return 0 if tag == "SUCCESS" else 1


if __name__ == "__main__":
    raise SystemExit(main())
