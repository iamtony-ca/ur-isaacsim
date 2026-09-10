#!/usr/bin/env python3
"""OMY-L100 leader bring-up + calibration helper. Run this on the REAL machine.

The sim work is done (HISTORY.md 22): the joint-direct map is proven and the
bridge's safety envelope is verified. What sim CANNOT give you is the real
leader's ENCODER ZERO and the wrist offsets, because:

  * the Dynamixel zero depends on how the arm was assembled/homed, and
  * the L100 is the leader for the OMY-F3M, not a scaled UR16e -- its lateral
    offset accumulates to -46 mm where the UR16e's is +290.7 mm, opposite sign
    (HISTORY.md 21). So J4/J6 have no derivable offset, only a comfortable one.

This tool measures both instead of making you guess.

    check    is the leader alive and sane? (run FIRST, needs no UR16e)
    match    *** the one that matters *** hold the leader so it LOOKS like the
             UR16e's current pose, run this, and it solves for the offset vector
             that makes the mapping exact. Covers encoder zero AND wrist offsets
             in one shot.
    verify   residual with the parameters currently in effect.

Usage
-----
    # 1) leader only -- no UR16e needed
    ros2 run ur_bringup omy_leader_calib.py --mode check

    # 2) with BOTH running and the arm in a pose you can copy by hand:
    #    physically pose the leader to mirror the robot, hold it still, then
    ros2 run ur_bringup omy_leader_calib.py --mode match

    # 3) apply what it prints, then confirm
    ros2 run ur_bringup omy_leader_calib.py --mode verify

`match` prints a ready-to-paste `ros2 param set` line AND the launch-argument
form. *** Put the final numbers in teleop_omy.launch.py defaults ***, otherwise
they vanish with the node -- and mirror them into virtual_omy_leader.py, or the
sim regression's engage gate will start refusing.

SAFETY: this tool only READS. It never commands the arm. Keep the bridge
disabled while calibrating.
"""
import argparse
import math
import statistics
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from rclpy.utilities import remove_ros_args
from sensor_msgs.msg import JointState

UR_JOINTS = [
    "shoulder_pan_joint", "shoulder_lift_joint", "elbow_joint",
    "wrist_1_joint", "wrist_2_joint", "wrist_3_joint",
]
LEADER_JOINTS = ["joint1", "joint2", "joint3", "joint4", "joint5", "joint6"]
LEADER_GRIPPER = "rh_r1_joint"
# Keep in step with omy_to_ur16e.py.
DEF_SIGN = [1.0, 1.0, 1.0, 1.0, -1.0, 1.0]
DEF_OFFSET = [0.0, -math.pi / 2, 0.0, 0.0, 0.0, 0.0]
UR_LIMITS = [2 * math.pi, 2 * math.pi, math.pi, 2 * math.pi, 2 * math.pi, 2 * math.pi]


class Calib(Node):
    def __init__(self):
        super().__init__("omy_leader_calib")
        qos = QoSProfile(depth=50, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.lead, self.foll, self.grip = [], [], []
        self.lead_names = None
        self.create_subscription(JointState, "/leader/joint_states", self._l, qos)
        self.create_subscription(JointState, "/joint_states", self._f, qos)

    def _l(self, m):
        i = {n: k for k, n in enumerate(m.name)}
        self.lead_names = list(m.name)
        if all(n in i for n in LEADER_JOINTS):
            self.lead.append([m.position[i[n]] for n in LEADER_JOINTS])
        if LEADER_GRIPPER in i:
            self.grip.append(m.position[i[LEADER_GRIPPER]])

    def _f(self, m):
        i = {n: k for k, n in enumerate(m.name)}
        if all(n in i for n in UR_JOINTS):
            self.foll.append([m.position[i[n]] for n in UR_JOINTS])

    def collect(self, sec):
        self.lead.clear(); self.foll.clear(); self.grip.clear()
        t0 = time.time()
        while time.time() - t0 < sec:
            rclpy.spin_once(self, timeout_sec=0.02)
        return time.time() - t0


def col(rows, k):
    return [r[k] for r in rows]


def mode_check(n, sec):
    el = n.collect(sec)
    ok = True
    print("\n== leader ==")
    if not n.lead:
        print("  FAIL  /leader/joint_states 가 없다.")
        print("        ros2 launch open_manipulator_bringup omy_l100_leader_ai.launch.py \\")
        print("            port_name:=/dev/ttyUSB0 use_self_collision_avoidance:=false")
        return 1
    print(f"  OK    발행 {len(n.lead)/el:.0f} Hz  (실물 기대치 ~300 Hz)")
    missing = [j for j in LEADER_JOINTS + [LEADER_GRIPPER] if j not in (n.lead_names or [])]
    if missing:
        print(f"  FAIL  관절 누락: {missing}  → 다이나믹셀 ID 1..7 배선/설정 확인"); ok = False
    else:
        print(f"  OK    관절 7개 전부 존재 (6 + {LEADER_GRIPPER})")
    print("\n  관절별 현재값 / 잡음(표준편차):")
    for k, j in enumerate(LEADER_JOINTS):
        v = col(n.lead, k)
        sd = statistics.pstdev(v) if len(v) > 1 else 0.0
        flag = "   ★ 잡음 큼" if sd > 0.01 else ""
        print(f"    {j:<8}{math.degrees(statistics.mean(v)):+8.2f}°   sd {math.degrees(sd):5.3f}°{flag}")
    if n.grip:
        print(f"    {LEADER_GRIPPER:<8}{math.degrees(statistics.mean(n.grip)):+8.2f}°  "
              f"(트리거를 쥐었다 놓으며 값이 변하는지 확인)")
    print("\n== follower (UR16e) ==")
    if not n.foll:
        print("  --    /joint_states 없음. 리더만 확인하는 단계라면 정상.")
    else:
        print(f"  OK    발행 {len(n.foll)/el:.0f} Hz")
    print("\n  다음: 리더를 UR16e 와 같은 모양으로 잡고  --mode match")
    return 0 if ok else 1


def mode_match(n, sec, sign):
    el = n.collect(sec)
    if not n.lead or not n.foll:
        print("\n  FAIL  리더와 UR16e 가 둘 다 발행 중이어야 한다 "
              f"(leader={len(n.lead)}, follower={len(n.foll)}). --mode check 먼저.")
        return 1
    print(f"\n  {sec:.0f}초 평균 (leader {len(n.lead)} / follower {len(n.foll)} 샘플)\n")
    moved = False
    for k, j in enumerate(LEADER_JOINTS):
        if statistics.pstdev(col(n.lead, k)) > 0.02:
            print(f"  ★ 경고: {j} 가 측정 중 움직였다(sd {math.degrees(statistics.pstdev(col(n.lead,k))):.2f}°). "
                  "리더를 고정하고 다시 측정할 것."); moved = True
    off = []
    print(f"  {'#':<4}{'UR16e':>10}{'leader':>10}{'sign':>7}{'→ offset':>12}{'기본값':>10}{'차이':>9}")
    print("  " + "-" * 64)
    for k in range(6):
        u = statistics.mean(col(n.foll, k))
        g = statistics.mean(col(n.lead, k))
        o = u - sign[k] * g
        off.append(o)
        d = math.degrees(o - DEF_OFFSET[k])
        flag = "  ★" if abs(d) > 10 else ""
        print(f"  J{k+1:<3}{math.degrees(u):>9.2f}°{math.degrees(g):>9.2f}°{sign[k]:>7.0f}"
              f"{math.degrees(o):>11.2f}°{math.degrees(DEF_OFFSET[k]):>9.2f}°{d:>8.2f}°{flag}")
    print("\n  ★ = 기본값과 10° 이상 차이. J4/J6 은 원래 유도값이 없으니 커도 정상이고,"
          "\n    J1~J3·J5 가 크면 엔코더 영점이 URDF 영점과 다르다는 뜻이다.")
    lst = ", ".join(f"{v:.5f}" for v in off)
    print("\n  ── 적용 (임시) — 브리지를 이 인자로 다시 띄운다 ─────────")
    print(f'  ros2 launch ur_bringup teleop_omy.launch.py offset:="[{lst}]"')
    print("  ★ `ros2 param set /omy_to_ur16e offset ...` 은 쓰지 말 것 — 브리지는")
    print("    파라미터를 생성자에서 한 번만 읽으므로 param set 은 성공을 반환하고")
    print("    아무 일도 하지 않는다 (HISTORY.md 42.3-C).")
    print("\n  ── 영구화 (이게 진짜 할 일) ─────────────────────────────")
    print("  teleop_omy.launch.py 의 bridge 파라미터에 offset 기본값으로 넣고,")
    print("  virtual_omy_leader.py 의 offset 도 같은 값으로 맞출 것")
    print("  (안 그러면 sim 회귀테스트의 engage 게이트가 거부한다).")
    return 1 if moved else 0


def mode_verify(n, sec, sign, offset):
    n.collect(sec)
    if not n.lead or not n.foll:
        print("\n  FAIL  리더와 UR16e 가 둘 다 필요하다."); return 1
    print(f"\n  현재 파라미터 기준 잔차 (offset={[round(math.degrees(v),1) for v in offset]}°)\n")
    worst = 0.0
    for k, j in enumerate(UR_JOINTS):
        u = statistics.mean(col(n.foll, k))
        g = statistics.mean(col(n.lead, k))
        e = abs(u - (sign[k] * g + offset[k]))
        worst = max(worst, math.degrees(e))
        print(f"  {j:<22}{math.degrees(e):7.2f}°   "
              f"{'OK' if math.degrees(e) < 8.6 else '★ engage_tol(0.15rad=8.6°) 초과'}")
    print(f"\n  최대 잔차 {worst:.2f}°  →  "
          f"{'engage 가 수락될 것' if worst < 8.6 else 'engage 는 거부된다. --mode match 로 다시 구할 것'}")
    return 0 if worst < 8.6 else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--mode", choices=["check", "match", "verify"], default="check")
    ap.add_argument("--seconds", type=float, default=3.0, help="평균 낼 시간")
    ap.add_argument("--sign", default=None, help="쉼표 구분 6개 (기본: 1,1,1,1,-1,1)")
    ap.add_argument("--offset", default=None, help="verify 용, 쉼표 구분 6개 [rad]")
    a = ap.parse_args(remove_ros_args(sys.argv)[1:])

    sign = [float(x) for x in a.sign.split(",")] if a.sign else list(DEF_SIGN)
    offset = [float(x) for x in a.offset.split(",")] if a.offset else list(DEF_OFFSET)
    if len(sign) != 6 or len(offset) != 6:
        print("sign/offset 은 각각 6개여야 한다"); return 2

    rclpy.init()
    n = Calib()
    print(f"\nOMY-L100 캘리브레이션 도우미 — mode={a.mode}, {a.seconds:.0f}초 수집")
    print("(읽기 전용: 팔에 명령을 보내지 않는다)")
    try:
        if a.mode == "check":
            rc = mode_check(n, a.seconds)
        elif a.mode == "match":
            rc = mode_match(n, a.seconds, sign)
        else:
            rc = mode_verify(n, a.seconds, sign, offset)
    finally:
        n.destroy_node()
        rclpy.shutdown()
    print()
    return rc


if __name__ == "__main__":
    sys.exit(main())
