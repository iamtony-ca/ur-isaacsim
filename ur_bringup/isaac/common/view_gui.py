# SPDX-License-Identifier: Apache-2.0
"""Open a USD in the Isaac Sim GUI for inspection / pose tuning, and LIVE-PRINT the
transform of whatever prim you select — so you can gizmo a part (e.g. a gripper
finger) into place and read back the numbers to bake into the config.

By default it opens the EOAT assembly and drops the wheel at the gripper grasp, so you
see the gripper body + custom fingers + wheel together (the render you saw, but live).

    /isaac-sim/python.sh isaac/common/view_gui.py                 # EOAT assembly + wheel
    /isaac-sim/python.sh isaac/common/view_gui.py <some.usd>      # any USD
    /isaac-sim/python.sh isaac/common/view_gui.py --no-wheel

In the GUI: click a prim (e.g. a finger's `geo`), move/rotate it with the gizmo; the
terminal prints its LOCAL transform (xyz m / rpy deg, ROS Rz·Ry·Rx). CLOSE the window
to exit. (Needs a display — run it yourself with the ! prefix so it uses your session.)
"""
from __future__ import annotations

import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
ASSETS = HERE.parent / "assets"
args = [a for a in sys.argv[1:] if not a.startswith("--")]
USD = Path(args[0]).resolve() if args else (ASSETS / "ur16e_dualtool.usd")
ADD_WHEEL = "--no-wheel" not in sys.argv

from isaacsim import SimulationApp
kit = SimulationApp({"headless": False, "renderer": "RayTracedLighting"})

import omni.usd  # noqa: E402
from pxr import Usd, UsdGeom, Gf  # noqa: E402

ctx = omni.usd.get_context()
ctx.open_stage(str(USD))
stage = ctx.get_stage()

# drop the wheel at the gripper grasp. Parent it UNDER the gripper prim with a LOCAL
# transform (0,0,0.145) so it inherits the gripper's orientation — the wheel bore axis
# then aligns with the gripper +Z (tool axis) and seats in the finger cradle. (Placing it
# at a world position with identity rotation mis-orients it once the gripper is tilted.)
if ADD_WHEEL:
    wheel = ASSETS / "manip" / "wheel.usd"
    grip = next((p for p in stage.Traverse() if p.GetName() == "gripper_2fg14"), None)
    if grip and wheel.exists():
        w = UsdGeom.Xform.Define(stage, grip.GetPath().AppendChild("graspwheel"))
        w.GetPrim().GetReferences().AddReference(str(wheel))
        w.AddTranslateOp().Set(Gf.Vec3d(0, 0, 0.179))          # gripper LINK frame (finger cradle centre)
        print("[view] wheel parented under gripper_2fg14 at local (0,0,0.145) — inherits tool-axis orientation")

sel = ctx.get_selection()
print(f"[view] opened {USD.name}. Select a prim and move it — its LOCAL transform prints below. "
      "CLOSE the window to exit.")
_last = None
_n = 0
while kit.is_running():
    kit.update()
    _n += 1
    if _n % 60 == 0:
        paths = sel.get_selected_prim_paths()
        if paths:
            p = stage.GetPrimAtPath(paths[0])
            if p and p.IsA(UsdGeom.Xformable):
                t = Gf.Transform(UsdGeom.Xformable(p).GetLocalTransformation())
                tr = t.GetTranslation()
                az, ay, ax = t.GetRotation().Decompose(Gf.Vec3d.ZAxis(), Gf.Vec3d.YAxis(), Gf.Vec3d.XAxis())
                cur = (paths[0], tuple(round(v, 4) for v in tr), tuple(round(v, 1) for v in (ax, ay, az)))
                if cur != _last:
                    _last = cur
                    print(f"[view] {paths[0]}  xyz(m)={cur[1]}  rpy(deg)={cur[2]}", flush=True)
                    # also mirror to a sidecar file so the value is always readable
                    try:
                        (HERE / "_view_last.txt").write_text(
                            f"{paths[0]}\nxyz={cur[1]}\nrpy={cur[2]}\n")
                    except Exception:
                        pass
kit.close()
