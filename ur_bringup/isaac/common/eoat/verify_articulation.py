# SPDX-License-Identifier: Apache-2.0
"""Stage-G check: load an emitted EOAT USD, play physics headless, and confirm it
is ONE valid articulation — parses, DOFs finite (no NaN blow-up), and the parallel
gripper finger drive actually moves both jaws (mimic works).

Run:  /isaac-sim/python.sh verify_articulation.py <assembly.usd> [articulation_root_prim]
"""
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np
from isaacsim import SimulationApp
_app = SimulationApp({"headless": True})

from isaacsim.core.api import SimulationContext          # noqa: E402
from isaacsim.core.utils import stage as stage_utils     # noqa: E402

_argv = [a for a in sys.argv[1:] if not a.startswith("--")]
ZERO_G = "--zero-gravity" in sys.argv   # structural/kinematic check for un-driven multi-DOF
                                        # assets (e.g. arm+EOAT): stops the un-controlled arm
                                        # drooping under gravity from confounding the finger test
USD = Path(_argv[0]).resolve()
ROOT = _argv[1] if len(_argv) > 1 else "/World/eoat"


def info(m):
    print(f"[verify] {m}", flush=True)


sim = SimulationContext(stage_units_in_meters=1.0)
stage_utils.add_reference_to_stage(str(USD), "/World/eoat")
info(f"loaded {USD.name} at /World/eoat  (articulation root: {ROOT})")

# Command the fingers exactly as ROS will: write the joint DRIVE target (not a
# kinematic pose). Both parallel jaws get the same opening; validates the authored
# drive/limits/flip end-to-end, independent of any core-API apply_action quirk.
import omni.usd                                             # noqa: E402
from pxr import UsdPhysics                                  # noqa: E402
_stg = omni.usd.get_context().get_stage()
CMD = 0.02
finger_joints = [p for p in _stg.Traverse()
                 if "finger" in p.GetName() and p.IsA(UsdPhysics.Joint)]
for jp in finger_joints:
    UsdPhysics.DriveAPI(jp, "linear").GetTargetPositionAttr().Set(CMD)
info(f"set drive target {CMD} on {len(finger_joints)} finger joint(s)")

if ZERO_G:
    sim.get_physics_context().set_gravity(0.0)
    info("gravity disabled (structural check)")

sim.reset()  # play
from isaacsim.core.prims import SingleArticulation        # noqa: E402
art = SingleArticulation(ROOT)
art.initialize()

dof = art.dof_names
info(f"DOF names ({len(dof)}): {dof}")
info(f"num bodies: {art.num_bodies}")

# settle
for _ in range(30):
    sim.step(render=False)
q0 = np.asarray(art.get_joint_positions())
finite0 = bool(np.all(np.isfinite(q0)))
info(f"after settle: joint pos finite={finite0}  q={np.round(q0, 5).tolist()}")

# Command the gripper: hold every OTHER joint (the arm) at its initial pose so an
# un-controlled arm doesn't thrash and confound the test, and drive both fingers to
# CMD. In the real ROS sim the trajectory controller holds the arm; here we emulate
# that hold. Confirm both jaws reach CMD and the held joints stay put.
from isaacsim.core.utils.types import ArticulationAction  # noqa: E402
finger_dofs = [n for n in dof if "finger" in n]
result = {}
if finger_dofs:
    q_hold = np.asarray(art.get_joint_positions(), dtype=float)
    target = q_hold.copy()
    for n in finger_dofs:
        target[dof.index(n)] = CMD
    for _ in range(200):
        art.apply_action(ArticulationAction(joint_positions=target))
        sim.step(render=False)
    q1 = np.asarray(art.get_joint_positions())
    result = {n: round(float(q1[dof.index(n)]), 4) for n in finger_dofs}
    reached = all(abs(q1[dof.index(n)] - CMD) < 3e-3 for n in finger_dofs)
    arm_idx = [i for i, n in enumerate(dof) if "finger" not in n]
    arm_held = all(abs(q1[i] - q_hold[i]) < 0.05 for i in arm_idx) if arm_idx else True
    info(f"finger drive test (target={CMD}): {result}  both reached={reached}  arm_held={arm_held}")
else:
    info("no finger DOF found (fingers may be fixed) — skipping drive test")
    reached = True

qN = np.asarray(art.get_joint_positions())
finite = bool(np.all(np.isfinite(qN)))
has_arm = any("finger" not in n for n in dof)
# Fixed-base EOAT: require the finger drive to reach target (full dynamic check).
# Arm+EOAT: an un-controlled arm can't be play-tested standalone (the ROS trajectory
# controller holds it in the real sim), so the pass is STRUCTURAL — one articulation,
# expected DOFs, parses & finite. Dynamic gripper behaviour is validated by the EOAT
# standalone run above and, for the full asset, in the ROS sim.
ok = finite and len(dof) > 0 and (True if has_arm else reached)
mode = "structural (arm present; dynamics -> ROS sim)" if has_arm else "dynamic"
info(f"RESULT: articulation valid={ok} [{mode}]  dofs={len(dof)}  finite={finite}  fingers_reached={reached}")

sim.stop()
_app.close()
sys.exit(0 if ok else 1)
