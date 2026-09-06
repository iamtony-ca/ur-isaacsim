#!/usr/bin/env bash
# One command to stand this workspace up on a FRESH machine.
#
#   git clone <repo> /isaac-sim/volume/ur_ws/src
#   /isaac-sim/volume/ur_ws/src/setup/bootstrap.sh --dry-run   # look first
#   /isaac-sim/volume/ur_ws/src/setup/bootstrap.sh             # then do it
#
# It is setup.sh's stages in the right order plus the things a fresh box needs
# that individual stages do not cover: the vcs bootstrap chicken-and-egg, and a
# final go/no-go. WS is derived from where THIS file lives, so cloning the repo
# somewhere other than /isaac-sim/volume/ur_ws just works.
#
#   --dry-run     print the plan, change nothing (recommended first run)
#   --with-udev   also install the U2D2 udev rule -- REAL OMY-L100 hardware only,
#                 writes /etc/udev. Omitted by default because it is the one
#                 system-wide change here and it is useless without the device.
#   --no-ml       skip the ~8 GB torch/lerobot venv (sim + teleop do not need it)
#
# ASSUMPTIONS (checked by the preflight stage, which runs first and is read-only):
#   * NVIDIA Isaac Sim container, Isaac 6.0.1, /isaac-sim/python.sh present
#   * ROS 2 Jazzy already in the image at /opt/ros/jazzy -- NOT installed here.
#     Installing a ROS distro is a base-image decision, and quietly doing it to
#     someone's machine is the exact class of side effect this workspace forbids.
#   * NVIDIA GPU + driver. sm_89/sm_120 (RTX 40/50) triggers an nvblox SOURCE
#     build automatically -- the apt binary is sm_75-only and aborts otherwise.
#   * >= 30 GiB free, and sudo for apt.
#
# Everything installed lands either in apt (guarded: refuses any install that
# would upgrade or remove an existing package) or in the workspace overlay.
# `rm -rf $WS/build $WS/install $WS/deps` undoes the source-built half entirely.
set -euo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
export WS="${WS:-$(cd "$HERE/../.." && pwd)}"

DRY=""; WITH_UDEV=0; NO_ML=0
for a in "$@"; do
  case "$a" in
    --dry-run)   DRY="--dry-run" ;;
    --with-udev) WITH_UDEV=1 ;;
    --no-ml)     NO_ML=1 ;;
    -h|--help)   sed -n '2,32p' "$0" | sed 's/^# \?//'; exit 0 ;;
    *) echo "unknown option: $a (see --help)"; exit 2 ;;
  esac
done

echo "=============================================================="
echo " UR16e / Isaac Sim workspace bootstrap"
echo "   WS       = $WS"
echo "   dry-run  = ${DRY:-no}"
echo "   ML venv  = $([ "$NO_ML" = 1 ] && echo skip || echo yes)"
echo "   udev     = $([ "$WITH_UDEV" = 1 ] && echo yes || echo "no (real HW only)")"
echo "=============================================================="

# Stage order matters. `pin` MUST precede `repos`: the NVIDIA repos ship
# higher-versioned copies of ROS packages (robotiq_description 0.0.1 -> 9.0.1),
# so if they are added before the pin exists, the very next apt upgrade
# silently replaces workspace-critical packages.
STAGES="preflight pin repos base cumotion sources build leader"
[ "$NO_ML" = 1 ] || STAGES="$STAGES ml"
[ "$WITH_UDEV" = 1 ] && STAGES="$STAGES udev"
STAGES="$STAGES verify"

echo
echo ">> stages: $STAGES"
echo
# shellcheck disable=SC2086
"$HERE/setup.sh" $DRY $STAGES

if [ -n "$DRY" ]; then
  echo
  echo "That was a dry run. Re-run without --dry-run to apply."
  exit 0
fi

cat <<EOF

==============================================================
 Bootstrap finished. Every terminal from here needs:

   source /opt/ros/jazzy/setup.bash
   source $WS/install/setup.bash
   export ROS_DOMAIN_ID=0

 SMOKE TEST (simulation, no hardware at all)
 ------------------------------------------------------------
 1) Isaac        /isaac-sim/python.sh \\
                   $WS/src/ur_bringup/isaac/common/ur16e_isaac_ros2.py \\
                   --asset-path $WS/src/ur_bringup/isaac/assets/ur16e_with_2f85.usd
                 (add --headless if you have no display)
 2) control      ros2 launch ur_bringup ur16e_2f85.launch.py use_sim:=true
 3) pose         python3 $WS/src/ur_bringup/isaac/common/reset_pose.py ready
 4) teleop       ros2 launch ur_bringup teleop_omy.launch.py \\
                   use_sim_time:=true virtual_leader:=true
 5) engage       python3 $WS/src/ur_bringup/isaac/common/switch_control_mode.py streaming
                 ros2 service call /omy_bridge/enable std_srvs/srv/Trigger

 The arm should now track the fake leader. Expected end-to-end error ~0.24 deg
 (HISTORY.md 22). Shut down with:
   ros2 service call /omy_bridge/disable std_srvs/srv/Trigger
   python3 $WS/src/ur_bringup/isaac/common/switch_control_mode.py trajectory

 REAL HARDWARE
 ------------------------------------------------------------
   UR16e / 2F-85 / D405   -> HARDWARE.md 1, 2, 3
   OMY-L100 leader        -> HARDWARE.md 4-B  (run bootstrap --with-udev first)
   calibration            -> ros2 run ur_bringup omy_leader_calib.py --mode check

 Re-check the machine any time (read-only):  $HERE/check_env.sh
==============================================================
EOF
