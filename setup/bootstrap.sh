#!/usr/bin/env bash
# One command to stand this workspace up on a FRESH machine.
#
#   (src/ in place: git clone <repo> /isaac-sim/volume/ur_ws/src, or a copy of src/)
#   # STOCK Isaac Sim image only (no sudo/git/gpg in it) -- on the HOST, once, as root:
#   docker exec -u 0 -it <container> bash /isaac-sim/volume/ur_ws/src/setup/container_prereqs.sh --sudo-user isaac-sim
#   /isaac-sim/volume/ur_ws/src/setup/bootstrap.sh --dry-run   # look first
#   /isaac-sim/volume/ur_ws/src/setup/bootstrap.sh --fresh     # then do it (--fresh: container made for this)
#
# It is setup.sh's stages in the right order plus the things a fresh box needs
# that individual stages do not cover: the vcs bootstrap chicken-and-egg, and a
# final go/no-go. WS is derived from where THIS file lives, so cloning the repo
# somewhere other than /isaac-sim/volume/ur_ws just works.
#
#   --dry-run     print the plan, change nothing (recommended first run)
#   --fresh       this container was created FOR this workspace (nobody else's
#                 project lives in it). Sets ALLOW_UPGRADES=ubuntu: the apt guard
#                 then accepts upgrades of already-installed packages when -- and
#                 only when -- they come from Ubuntu's own repos. A fresh Isaac Sim
#                 image has stale base libs and ros-jazzy-desktop / moveit need the
#                 noble-updates/-security point releases (14 + 4 packages on 2026-09-16,
#                 23 + 4 on 2026-10-21 -- the count grows with Ubuntu updates; the guard
#                 checks that every one is Ubuntu-origin). Without --fresh the `ros`
#                 stage stops at that guard by design. NVIDIA-origin upgrades and
#                 removals are still refused even with --fresh.
#   --with-udev   also install the U2D2 udev rule -- REAL OMY-L100 hardware only,
#                 writes /etc/udev. Omitted by default because it is the one
#                 system-wide change here and it is useless without the device.
#   --no-ml       skip the ~8 GB torch/lerobot venv (sim + teleop do not need it)
#   --with-groot  also run the `groot` stage (lerobot[groot] extra, ~1 GB). The
#                 model files themselves are gated on HF and must be downloaded
#                 with a token or copied in by hand afterwards (SETUP.md 2-C-2).
#
# ASSUMPTIONS (checked by the preflight stage, which runs first and is read-only):
#   * NVIDIA Isaac Sim container, Isaac 6.0.1 or 6.1.0-rc.26 (both verified; HISTORY.md
#     14 and 48; another 6.1.0 build only warns), /isaac-sim/python.sh present
#   * sudo WITHOUT a password, git, curl, gpg, locales. A stock Isaac Sim image has none
#     of sudo/git/gpg: container_prereqs.sh (above) adds them; preflight prints the
#     exact command when anything is missing. Verified from the STOCK 6.1.0 image
#     (headless, workspace copied in with rsync) on 2026-10-21.
#   * ROS 2 Jazzy: if /opt/ros/jazzy is missing (the Isaac Sim base image has no
#     ROS -- and no python3), the `ros` stage installs ros-jazzy-desktop +
#     ros-dev-tools from packages.ros.org by the official deb procedure. It never
#     touches an existing ROS. Verified end-to-end on a fresh 6.1.0 container
#     2026-09-16 (HISTORY.md 48).
#   * NVIDIA GPU + driver (>= R580 for the CUDA 13.2 toolkit cuMotion pulls in). Any GPU
#     other than sm_75 (e.g. sm_86/89/120) triggers an nvblox SOURCE build automatically
#     -- the apt binary is sm_75-only with no PTX and aborts otherwise.
#   * BUILD_JOBS (default 2) caps colcon/make parallelism -- the host CPU is shared.
#   * Disk in two places (preflight checks both): the container ROOT fs, where apt
#     installs -- measured 2026-10-21 from the stock image: ~11 GB of packages (CUDA 4.8 +
#     Nsight 2.4 + ROS 2.8 + rest), container layer +17 GB; preflight wants >= 20 GiB --
#     and the workspace volume: ML venv 7.9 GB + pip cache + build, GR00T models +6.5 GB;
#     preflight wants >= 30 GiB (both from one pool when they share a disk).
#   * Container started with --shm-size=8g or more if you will train (a 64 MiB
#     default /dev/shm needs the DataLoader workaround, SETUP.md 2-C).
#
# Everything installed lands either in apt (guarded: refuses any install that
# would upgrade or remove an existing package) or in the workspace overlay.
# `rm -rf $WS/build $WS/install $WS/deps` undoes the source-built half entirely.
set -euo pipefail

HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
export WS="${WS:-$(cd "$HERE/../.." && pwd)}"

DRY=""; WITH_UDEV=0; NO_ML=0; WITH_GROOT=0; FRESH=0
for a in "$@"; do
  case "$a" in
    --dry-run)    DRY="--dry-run" ;;
    --fresh)      FRESH=1 ;;
    --with-udev)  WITH_UDEV=1 ;;
    --no-ml)      NO_ML=1 ;;
    --with-groot) WITH_GROOT=1 ;;
    -h|--help)    awk 'NR>1 && !/^#/{exit} NR>1{sub(/^# ?/,""); print}' "$0"; exit 0 ;;
    *) echo "unknown option: $a (see --help)"; exit 2 ;;
  esac
done
[[ "${BUILD_JOBS:-2}" =~ ^[1-9][0-9]*$ ]] || { echo "BUILD_JOBS must be a positive integer (got '$BUILD_JOBS')"; exit 2; }
if [ "$FRESH" = 1 ]; then
  export ALLOW_UPGRADES="${ALLOW_UPGRADES:-ubuntu}"
elif [ -z "${ALLOW_UPGRADES:-}" ] && [ ! -d "/opt/ros/${ROS_DISTRO:-jazzy}" ]; then
  # A machine without ROS is almost certainly a fresh container, and the ros stage
  # WILL hit the base-lib upgrade guard. Say so up front instead of 10 minutes in.
  echo "NOTE: /opt/ros/${ROS_DISTRO:-jazzy} is missing, so the ros stage will install ROS 2. On a fresh"
  echo "      image that needs Ubuntu base-lib point releases, which the apt guard refuses unless"
  echo "      you pass --fresh (container made for this workspace) -- see --help."
  echo
fi
[ "$WITH_GROOT" = 1 ] && [ "$NO_ML" = 1 ] && { echo "--with-groot needs the ML venv; drop --no-ml"; exit 2; }

echo "=============================================================="
echo " UR16e / Isaac Sim workspace bootstrap"
echo "   WS       = $WS"
echo "   dry-run  = ${DRY:-no}"
echo "   fresh    = $([ "$FRESH" = 1 ] && echo "yes (ALLOW_UPGRADES=ubuntu)" || echo "no (shared-container guard: refuse every upgrade)")"
echo "   ML venv  = $([ "$NO_ML" = 1 ] && echo skip || echo yes)"
echo "   GR00T    = $([ "$WITH_GROOT" = 1 ] && echo yes || echo "no (setup.sh groot later)")"
echo "   udev     = $([ "$WITH_UDEV" = 1 ] && echo yes || echo "no (real HW only)")"
echo "   jobs     = BUILD_JOBS=${BUILD_JOBS:-2} (colcon --parallel-workers + make -j)"
echo "=============================================================="

# Stage order matters. `pin` MUST precede `repos`: the NVIDIA repos ship
# higher-versioned copies of ROS packages (robotiq_description 0.0.1 -> 9.0.1),
# so if they are added before the pin exists, the very next apt upgrade
# silently replaces workspace-critical packages.
STAGES="preflight ros pin repos base cumotion sources build leader"
[ "$NO_ML" = 1 ] || STAGES="$STAGES ml"
[ "$WITH_GROOT" = 1 ] && STAGES="$STAGES groot"
[ "$WITH_UDEV" = 1 ] && STAGES="$STAGES udev"
STAGES="$STAGES verify"

echo
echo ">> stages: $STAGES"
echo
rc=0
# shellcheck disable=SC2086
"$HERE/setup.sh" $DRY $STAGES || rc=$?

if [ -n "$DRY" ]; then
  echo
  if [ "$rc" = 3 ]; then
    echo "That was a dry run, and PREFLIGHT FAILED (see FAIL above) -- fix that first."
    exit "$rc"
  elif [ "$rc" != 0 ]; then
    echo "That was a dry run, and setup.sh itself stopped (rc=$rc, see above) -- fix that first."
    exit "$rc"
  fi
  echo "That was a dry run. Re-run without --dry-run to apply."
  exit 0
fi
[ "$rc" = 0 ] || { echo; echo "bootstrap STOPPED: a stage failed (rc=$rc, see FAIL above). Fix it and re-run: every stage is idempotent, so re-running is safe."; exit "$rc"; }

cat <<EOF

==============================================================
 Bootstrap finished. Every terminal from here needs:

   source /opt/ros/jazzy/setup.bash
   source $WS/install/setup.bash
   export ROS_DOMAIN_ID=42     # 42 is what the FIRST container on this host uses: pick a
                               # non-zero id that is UNIQUE per container on the host (a
                               # second container on 42 joins the first one's graph), the
                               # SAME in every terminal of this container (SETUP.md 9)

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

 The arm should now track the fake leader. Expected end-to-end error: a few tenths
 of a degree at most (0.116 deg measured 2026-10-21; 0.14-0.24 earlier, HISTORY.md 22).
 Shut down with:
   ros2 service call /omy_bridge/disable std_srvs/srv/Trigger
   python3 $WS/src/ur_bringup/isaac/common/switch_control_mode.py trajectory
 Stop everything (Isaac, the launches and every node they started; last line CLEAN):
   bash $WS/src/ur_bringup/scripts/harness/shutdown_rollout.sh

 REAL HARDWARE
 ------------------------------------------------------------
   UR16e / 2F-85 / D405   -> HARDWARE.md 1, 2, 3
   OMY-L100 leader        -> HARDWARE.md 4-B  (run bootstrap --with-udev first)
   calibration            -> ros2 run ur_bringup omy_leader_calib.py --mode check

 Re-check the machine any time (read-only):  $HERE/check_env.sh
==============================================================
EOF
