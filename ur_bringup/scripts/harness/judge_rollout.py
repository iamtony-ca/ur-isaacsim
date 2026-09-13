#!/usr/bin/env python3
"""Judge one rollout from ground truth: is the block on the marker, and let go?

Same criterion the state machine uses for its own cycles (place_tol 35 mm), so a
policy rollout and a scripted demo are scored the same way.
"""
import rclpy, time
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy, HistoryPolicy
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState

TOL = 0.035
rclpy.init(); n = Node("judge"); st = {}
latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                     reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
n.create_subscription(PoseStamped, "/scene/objects/red/pose",
                      lambda m: st.__setitem__("p", m.pose.position), 10)
for q in (latched, 10):
    n.create_subscription(PoseStamped, "/scene/places/left/pose",
                          lambda m: st.__setitem__("L", m.pose.position), q)
n.create_subscription(JointState, "/joint_states",
                      lambda m: st.__setitem__("g", m.position[m.name.index("finger_joint")])
                      if "finger_joint" in m.name else None, 10)
e = time.time() + 15
while time.time() < e and len(st) < 3:
    rclpy.spin_once(n, timeout_sec=0.1)
if len(st) < 3:
    print("UNKNOWN (missing", {"p", "L", "g"} - set(st), ")")
else:
    p, L, g = st["p"], st["L"], st["g"]
    d = ((p.x - L.x) ** 2 + (p.y - L.y) ** 2) ** 0.5
    held = g > 0.4          # jaws still shut = never released
    tag = "SUCCESS" if (d <= TOL and not held) else "FAIL"
    why = "" if tag == "SUCCESS" else (" (still holding)" if held else " (too far)")
    print(f"{tag} d={d*1000:.0f}mm grip={g:.2f}{why}")
rclpy.shutdown()
