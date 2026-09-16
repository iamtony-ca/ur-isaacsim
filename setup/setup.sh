#!/usr/bin/env bash
# Reproduce the UR16e ROS 2 + Isaac Sim workspace on a fresh machine.
#
#   ./setup.sh                 # everything (asks nothing, refuses unsafe apt actions)
#   ./setup.sh --dry-run       # print what WOULD happen, change nothing
#   ./setup.sh base sources build       # only these stages
#   ./setup.sh --list          # show stages
#
# Env overrides:
#   WS=/path/to/ur_ws          workspace root       (default /isaac-sim/volume/ur_ws)
#   ALLOW_UPGRADES=ubuntu      accept upgrades of already-installed packages ONLY when
#                              they come from Ubuntu's own repos (noble-updates/-security).
#                              A FRESH Isaac Sim container needs this: its base libs are
#                              stale and ros-jazzy-desktop / moveit require the point
#                              releases (measured 2026-09-16: 14 + 4 packages). This is
#                              what `bootstrap.sh --fresh` sets.
#   ALLOW_UPGRADES=1           accept EVERY upgrade/removal apt proposes (last resort)
#
# ---------------------------------------------------------------------------
# DESIGN RULE: this machine is shared with other workspaces.
#   * Every apt install is SIMULATED first; if it would upgrade or remove an
#     already-installed package the script REFUSES (ALLOW_UPGRADES overrides, above).
#   * NVIDIA apt repos are PINNED BEFORE they are added, so they can only supply
#     packages nobody else provides -- they never shadow ROS/Ubuntu packages.
#   * Everything we build lands in the workspace overlay ($WS/install), never in
#     /opt/ros. Removing the workspace fully undoes the source-built parts.
# Manual/background for every stage: ../SETUP.md
# ---------------------------------------------------------------------------
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=lib.sh
source "$HERE/lib.sh"
ROS_STAGED=0

# Every valid stage name...
STAGES=(preflight ros pin repos base cumotion sources build leader ml groot verify udev)
# ...and what a bare `./setup.sh` runs. `udev` is EXCLUDED on purpose: it is the
# only stage that writes outside the workspace (/etc/udev) and it is meaningless
# without a U2D2 attached. Ask for it explicitly on the real machine.
DEFAULT_STAGES=(preflight ros pin repos base cumotion sources build leader ml verify)
declare -A STAGE_DESC=(
  [preflight]="check the machine can host this workspace at all (read-only)"
  [ros]="ROS 2 $ROS_DISTRO desktop + ros-dev-tools from packages.ros.org, ONLY if /opt/ros/$ROS_DISTRO is missing"
  [pin]="apt pin so NVIDIA repos cannot shadow ROS/Ubuntu packages (do this FIRST)"
  [repos]="add Isaac ROS / CUDA / VPI apt repos + keys"
  [base]="core ROS packages (UR, MoveIt, ros2_control, robotiq_description, teleop)"
  [cumotion]="cuMotion + nvblox apt packages (individual, never the metapackages)"
  [sources]="vcs import + nvblox_core submodule"
  [build]="colcon build (ur_bringup, gripper driver, nvblox_ros for this GPU)"
  [leader]="OMY-L100 teleop leader stack (gravity-compensated ros2_control, 7 pkgs)"
  [udev]="U2D2 udev rule for the OMY-L100 leader (REAL HW only, writes /etc/udev)"
  [ml]="ML venv for the IL/VLA track (torch with sm_120 + lerobot), workspace-local"
  [groot]="GR00T N1.7: lerobot[groot] extra into the ML venv + workspace-local HF cache (not default)"
  [verify]="run check_env.sh"
)

usage() { echo "usage: $0 [--dry-run] [--list] [stage ...]"; echo "stages: ${STAGES[*]}"; }

# ---------------------------------------------------------------------------
# Read-only. Fails fast on a fresh machine BEFORE anything is installed, because
# every failure below costs an hour to discover halfway through a build instead.
stage_preflight() {
  step "preflight — can this machine host the workspace? (read-only)"
  local fail=0

  # Isaac Sim. Verified on 6.0.1 (5.1.0 -> 6.0.1 needed no code change) and on
  # 6.1.0-rc.26 (fresh container, 2026-09-16, HISTORY.md 48: headless Isaac ->
  # /isaac_joint_states + /clock -> controllers -> reset_pose ready -> MoveIt plan+execute
  # SUCCESS, no code change). In 6.1.0 isaacsim.core.api / .prims / .utils moved to
  # /isaac-sim/extsDeprecated and still import (via isaacsim.core.deprecation_manager),
  # so a LATER release may drop them -- that is why any other version warns loudly.
  if [ -x /isaac-sim/python.sh ]; then
    local iv; iv="$(cat /isaac-sim/VERSION 2>/dev/null | cut -d+ -f1)"
    ok "Isaac Sim present (${iv:-version unknown})"
    case "$iv" in
      6.0.1*|6.1.0*) ok "Isaac $iv = a verified version (6.0.1, 6.1.0-rc.26)" ;;
      "") warn "cannot read /isaac-sim/VERSION" ;;
      *) warn "Isaac $iv is NOT a verified version (6.0.1 / 6.1.0). Expect possible API drift in ur_bringup/isaac/common/ur16e_isaac_ros2.py"
         warn "(isaacsim.core.api/.prims/.utils are already in extsDeprecated on 6.1.0). Run SETUP.md 5 'smoke' first and record the result in HISTORY.md." ;;
    esac
  else
    err "/isaac-sim/python.sh missing. Run inside the Isaac Sim container."; fail=1
  fi

  # ROS. The `ros` stage installs it from packages.ros.org when it is missing
  # (2026-09-16, HISTORY.md 47.6 -- the Isaac Sim base image ships no ROS). It is a
  # system-wide install, so it happens only on a machine where /opt/ros/$ROS_DISTRO
  # does not exist yet; it never touches an existing ROS.
  if [ -d "/opt/ros/$ROS_DISTRO" ]; then ok "ROS 2 $ROS_DISTRO present"
  elif [ "$ROS_STAGED" = 1 ]; then
    warn "/opt/ros/$ROS_DISTRO missing -- the 'ros' stage will install ros-$ROS_DISTRO-desktop + ros-dev-tools"
  else
    err "/opt/ros/$ROS_DISTRO missing. Run the 'ros' stage (setup.sh ros) or use an image with ROS 2 $ROS_DISTRO."
    fail=1
  fi

  # GPU. sm_120 (Blackwell) is why nvblox gets source-built; see stage_build.
  local sm; sm="$(gpu_sm)"
  if [ -n "$sm" ]; then
    ok "GPU detected: $(nvidia-smi --query-gpu=name --format=csv,noheader 2>/dev/null | head -1) (sm_$sm)"
    [ "$sm" -ge 89 ] 2>/dev/null && echo "     -> apt nvblox is sm_75-only; the build stage will compile nvblox_ros from source."
  else
    warn "no GPU visible (nvidia-smi failed). Isaac and cuMotion will not run."
  fi

  for c in git curl sudo; do need_cmd "$c" || fail=1; done
  command -v vcs >/dev/null 2>&1 && ok "vcstool present" \
    || warn "vcstool missing — the base stage installs python3-vcstool"

  # Disk: Isaac assets + the ML venv (~8 GB) + build trees.
  local free; free="$(df -BG --output=avail "$WS" 2>/dev/null | tail -1 | tr -dc '0-9')"
  if [ -n "$free" ]; then
    [ "$free" -ge 30 ] && ok "disk free: ${free} GiB" || warn "only ${free} GiB free; want >= 30"
  fi

  [ -f "$WS/src/ur16e.repos" ] && ok "workspace source tree looks right ($WS/src)" \
    || { err "$WS/src/ur16e.repos missing — clone the repo into $WS/src first"; fail=1; }

  [ "$fail" = 0 ] && ok "preflight passed" || { err "preflight FAILED — fix the above first"; return 1; }
}

# Set by stage_pin so stage_repos can tell "the pin is missing" from "the pin is being
# applied in this same invocation". Without it, --dry-run on a FRESH machine always dies
# at stage 2: stage_pin only prints "would write", so the file stage_repos looks for does
# not exist, and set -e turns that into "the whole plan stops here" -- which defeats the
# point of a dry run.
PIN_STAGED=0

# ---------------------------------------------------------------------------
# ros — ROS 2 <distro> itself. The official "Ubuntu (deb packages)" procedure from
# docs.ros.org, in code, gated on /opt/ros/<distro> being ABSENT:
#   locale -> universe -> ros2-apt-source .deb (adds ros2.sources + keyring)
#   -> apt update -> ros-<distro>-desktop + ros-dev-tools (colcon, vcstool, rosdep)
# Verified on a FRESH Isaac Sim 6.1.0 container 2026-09-16 (HISTORY.md 48): Ubuntu 24.04.3,
# ros2-apt-source 1.3.0~noble, ros-jazzy-desktop 0.11.0, ros-dev-tools 1.0.3 (1446 new
# packages; the image ships no python3 at all -- this stage is what brings it).
# Runs BEFORE `pin`/`repos`: the NVIDIA pin only stops NVIDIA repos from shadowing ROS
# packages; it does not create the ROS repo.
# A fresh image has stale base libs, and ros-jazzy-desktop needs their noble-updates
# point releases (measured: 14 packages -- util-linux, dpkg, zlib1g, libsystemd0, ...).
# The guard refuses that by default; ALLOW_UPGRADES=ubuntu (bootstrap.sh --fresh) accepts
# Ubuntu-origin upgrades only, which is the right call on a container made for this
# workspace and the wrong one on a shared container.
# ---------------------------------------------------------------------------
stage_ros() {
  step "ros — ROS 2 $ROS_DISTRO from packages.ros.org (only when missing)"
  if [ -d "/opt/ros/$ROS_DISTRO" ]; then ok "ROS 2 $ROS_DISTRO already at /opt/ros/$ROS_DISTRO — nothing to do"; return 0; fi
  local codename; codename="$(. /etc/os-release && echo "${VERSION_CODENAME:-}")"
  case "$ROS_DISTRO:$codename" in
    jazzy:noble) ok "Ubuntu $codename matches ROS 2 $ROS_DISTRO" ;;
    *) err "this stage knows ROS 2 jazzy on Ubuntu noble (24.04) only; got ROS_DISTRO=$ROS_DISTRO on '$codename'"; return 1 ;;
  esac
  need_cmd curl || return 1
  # A fresh container has NEVER run `apt-get update` (/var/lib/apt/lists is empty), so
  # even `apt-get install -s` fails with "Unable to locate package" on stock Ubuntu
  # packages. Refresh the lists once before the first guarded install. Measured on a
  # fresh Isaac Sim 6.1.0 container 2026-09-16 (HISTORY.md 48).
  apt_lists_refresh
  apt_guarded_install locales curl || return 1
  if locale -a 2>/dev/null | grep -qi "en_US.utf8"; then ok "locale en_US.UTF-8 present"
  else run "sudo locale-gen en_US en_US.UTF-8 && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8"; fi
  # `software-properties-common` exists ONLY to provide `add-apt-repository universe`.
  # It depends on packagekit, which on a fresh noble image drags 11 base-lib upgrades
  # (util-linux, libsystemd0, ...) and trips the guard. Ubuntu's official docker/Isaac
  # images already ship `universe` in ubuntu.sources, so install it only when needed.
  if apt-cache policy 2>/dev/null | grep -q "${codename}/universe"; then ok "universe component enabled"
  else
    apt_guarded_install software-properties-common || return 1
    run "sudo add-apt-repository -y universe"
  fi
  if [ -f /etc/apt/sources.list.d/ros2.sources ] || apt_installed ros2-apt-source; then
    ok "ros2-apt-source already installed"
  elif [ "$DRY_RUN" = 1 ]; then
    echo "  [dry-run] would download ros2-apt-source_<latest>.${codename}_all.deb from"
    echo "            github.com/ros-infrastructure/ros-apt-source and dpkg -i it (adds ros2.sources + keyring)"
  else
    local ver deb
    ver="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | grep -F '"tag_name"' | awk -F'"' '{print $4}')"
    [ -n "$ver" ] || { err "could not resolve the latest ros-apt-source release (network? GitHub API?)"; return 1; }
    deb="/tmp/ros2-apt-source_${ver}.${codename}_all.deb"
    curl -fsSL -o "$deb" "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${codename}_all.deb" || { err "download failed: $deb"; return 1; }
    sudo dpkg -i "$deb" >/dev/null && ok "ros2-apt-source $ver installed (ros2.sources + keyring)"
  fi
  if [ "$DRY_RUN" = 1 ]; then
    echo "  [dry-run] would: sudo apt-get update && install ros-$ROS_DISTRO-desktop ros-dev-tools (guarded)"
    return 0
  fi
  run "sudo apt-get update -qq"
  apt_guarded_install "ros-$ROS_DISTRO-desktop" ros-dev-tools || return 1
  [ -f "/opt/ros/$ROS_DISTRO/setup.bash" ] && ok "ROS 2 $ROS_DISTRO installed: /opt/ros/$ROS_DISTRO/setup.bash" \
    || { err "/opt/ros/$ROS_DISTRO/setup.bash still missing after install"; return 1; }
}

stage_pin() {
  step "pin — isolate NVIDIA repos (Pin-Priority 100)"
  local f=/etc/apt/preferences.d/99-nvidia-isolate.pref
  if [ -f "$f" ]; then ok "$f already present"; PIN_STAGED=1; return 0; fi
  echo "  NVIDIA repos ship higher-versioned copies of ROS packages"
  echo "  (measured: robotiq_description 0.0.1 -> 9.0.1, moveit_task_constructor_core -> 99.99.0)."
  echo "  Priority 100 = never upgrade an already-installed package."
  PIN_STAGED=1
  if [ "$DRY_RUN" = 1 ]; then echo "  [dry-run] would write $f"; return 0; fi
  sudo tee "$f" >/dev/null <<'EOF'
# UR16e workspace: keep NVIDIA repos from shadowing ROS/Ubuntu packages.
# Priority 100 = "install only if no version of this package is installed".
# New packages (cuMotion/nvblox/CUDA/VPI) still install normally.
Package: *
Pin: origin isaac.download.nvidia.com
Pin-Priority: 100

Package: *
Pin: origin developer.download.nvidia.com
Pin-Priority: 100

Package: *
Pin: origin repo.download.nvidia.com
Pin-Priority: 100
EOF
  ok "wrote $f"
}

stage_repos() {
  step "repos — Isaac ROS / CUDA / VPI"
  [ -f /etc/apt/preferences.d/99-nvidia-isolate.pref ] || [ "$PIN_STAGED" = 1 ] || {
    err "pin missing. Run the 'pin' stage first (order matters)."; return 1; }
  need_cmd curl
  _repo() { # name key_url list_line
    local name="$1" key="$2" line="$3"
    if [ -f "/etc/apt/sources.list.d/$name.list" ]; then ok "$name already added"; return 0; fi
    if [ "$DRY_RUN" = 1 ]; then echo "  [dry-run] would add repo $name"; return 0; fi
    curl -fsSL "$key" | sudo gpg --dearmor --yes -o "/usr/share/keyrings/$name.gpg"
    echo "$line" | sudo tee "/etc/apt/sources.list.d/$name.list" >/dev/null
    ok "added $name"
  }
  _repo nvidia-isaac-ros "https://isaac.download.nvidia.com/isaac-ros/repos.key" \
    "deb [signed-by=/usr/share/keyrings/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4 noble main"
  _repo nvidia-cuda "https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/3bf863cc.pub" \
    "deb [signed-by=/usr/share/keyrings/nvidia-cuda.gpg] https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/ /"
  _repo nvidia-vpi "https://repo.download.nvidia.com/jetson/jetson-ota-public.asc" \
    "deb [signed-by=/usr/share/keyrings/nvidia-vpi.gpg] https://repo.download.nvidia.com/jetson/x86_64/noble r38.2 main"
  run "sudo apt-get update -qq"
  # Prove the pin works before anything gets installed. Only meaningful once the NVIDIA
  # repos are actually in sources.list -- in a dry run they are not, so the candidate
  # would be the ROS one and "pin verified" would be verifying nothing.
  if [ "$DRY_RUN" = 1 ]; then
    echo "  [dry-run] pin effectiveness is checked here on a real run"
    echo "            (apt-cache policy ros-$ROS_DISTRO-robotiq-description must NOT be 9.x)"
    return 0
  fi
  local cand
  cand="$(apt-cache policy "ros-$ROS_DISTRO-robotiq-description" 2>/dev/null | awk '/Candidate:/{print $2}')"
  case "$cand" in
    9.*) err "PIN NOT EFFECTIVE: robotiq_description candidate is $cand (NVIDIA). Fix the pin before continuing."; return 1 ;;
    "")  warn "could not read robotiq_description candidate (repo not refreshed yet?)" ;;
    *)   ok "pin verified: robotiq_description candidate = $cand (ROS repo)" ;;
  esac
}

stage_base() {
  step "base — core ROS stack"
  apt_guarded_install \
    "ros-$ROS_DISTRO-ur" "ros-$ROS_DISTRO-moveit" \
    "ros-$ROS_DISTRO-ros2-control" "ros-$ROS_DISTRO-ros2-controllers" \
    "ros-$ROS_DISTRO-ros2-control-cmake" \
    "ros-$ROS_DISTRO-robotiq-description" \
    "ros-$ROS_DISTRO-moveit-ros-perception" \
    "ros-$ROS_DISTRO-moveit-servo" \
    "ros-$ROS_DISTRO-joy" \
    python3-vcstool python3-colcon-common-extensions || return 1
  # (Without `|| return 1` a refused/failed install here was silently masked by the
  # optional realsense line below and the stage reported success -- found on the
  # 2026-09-16 fresh-container run.)
  echo "  (optional, real D405 camera only)"
  apt_guarded_install "ros-$ROS_DISTRO-realsense2-camera" "ros-$ROS_DISTRO-librealsense2" || \
    warn "realsense install skipped/failed — only needed for the real camera"
}

stage_cumotion() {
  step "cumotion — GPU planning + nvblox (individual packages only)"
  echo "  NOT installing ros-$ROS_DISTRO-isaac-ros-cumotion-examples or -isaac-ros-nvblox:"
  echo "  those metapackages pull triton/tensor_rt/visual_slam (+400 debs) and force"
  echo "  container-wide python3.12 upgrades. We vendor the one yaml we need instead."
  apt_guarded_install \
    "ros-$ROS_DISTRO-isaac-ros-cumotion" \
    "ros-$ROS_DISTRO-isaac-ros-cumotion-moveit" \
    "ros-$ROS_DISTRO-isaac-ros-cumotion-robot-description" \
    "ros-$ROS_DISTRO-isaac-ros-cumotion-robot-segmenter" \
    "ros-$ROS_DISTRO-nvblox-ros" "ros-$ROS_DISTRO-nvblox-msgs" "ros-$ROS_DISTRO-nvblox-rviz-plugin"
}

stage_sources() {
  step "sources — vcs import + submodules"
  need_cmd vcs
  run "cd '$WS' && vcs import src < src/ur16e.repos"
  local nv="$WS/src/isaac_ros_nvblox"
  if [ -d "$nv" ]; then
    # nvblox_core is a git submodule; vcs import does NOT fetch it.
    run "cd '$nv' && git submodule update --init --recursive --depth 1"
    ok "nvblox_core submodule initialised"
  else
    warn "$nv missing — did vcs import run?"
  fi

  # Mask the open_manipulator packages we do not use, RIGHT AFTER import rather
  # than in the leader stage. Otherwise a plain `colcon build` run between the two
  # stages tries to build open_manipulator_gui and drags in Qt. Only the leader's
  # 5 packages plus its 2 sibling repos are ever wanted here.
  local om="$WS/src/open_manipulator" p
  if [ -d "$om" ]; then
    for p in open_manipulator open_manipulator_collision open_manipulator_gui \
             open_manipulator_moveit_config open_manipulator_playground \
             open_manipulator_teleop; do
      [ -d "$om/$p" ] && run "touch '$om/$p/COLCON_IGNORE'"
    done
    ok "COLCON_IGNORE set on 6 unused open_manipulator packages"
  fi
}

stage_build() {
  step "build — colcon"
  # ROS's setup.bash reads AMENT_TRACE_SETUP_FILES et al. without defaulting them,
  # so sourcing it under `set -u` aborts with "unbound variable". Drop -u across the
  # source only. (Invisible in --dry-run, which skips the source entirely -- this
  # cost a full fresh-machine run to find. See HISTORY.md 23.)
  if [ "$DRY_RUN" != 1 ]; then
    set +u; # shellcheck disable=SC1090
    source "/opt/ros/$ROS_DISTRO/setup.bash"; set -u
  fi
  run "cd '$WS' && colcon build --symlink-install --packages-up-to ur_bringup --cmake-args -DBUILD_TESTING=OFF"
  ok "ur_bringup + topic_based built"

  if [ -d "$WS/src/ros2_robotiq_gripper" ]; then
    run "cd '$WS' && colcon build --symlink-install --packages-select serial robotiq_driver robotiq_controllers --cmake-args -DBUILD_TESTING=OFF"
    ok "real 2F-85 gripper driver built"
  fi

  # nvblox: the apt binary is compiled for a fixed GPU arch list with no PTX.
  # Build from source only when this GPU is not covered.
  local sm archs nvbin="/opt/ros/$ROS_DISTRO/lib/nvblox_ros/nvblox_node"
  sm="$(gpu_sm)"; archs="$(cuda_archs_of "$nvbin")"
  echo "  GPU sm_${sm:-?} ; apt nvblox_node archs: ${archs:-<unknown>}"
  if [ -n "$sm" ] && [ -n "$archs" ] && grep -qw "sm_$sm" <<<"$archs"; then
    ok "apt nvblox supports this GPU — no source build needed"
  elif [ -d "$WS/src/isaac_ros_nvblox" ]; then
    warn "apt nvblox does NOT cover sm_${sm:-?} -> building nvblox_ros from source"
    local nvcc; nvcc="$(ls /usr/local/cuda*/bin/nvcc 2>/dev/null | head -1 || true)"
    [ -n "$nvcc" ] || { err "nvcc not found — run the cumotion stage first"; return 1; }
    run "cd '$WS' && CUDACXX='$nvcc' PATH=\"\$(dirname '$nvcc'):\$PATH\" colcon build --symlink-install --packages-select nvblox_ros --cmake-args -DUSE_NATIVE_CUDA_ARCHITECTURE=1 -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF"
    ok "nvblox_ros built for sm_$sm (workspace overlay only)"
  else
    warn "nvblox source not present and apt build may not run on this GPU — see SETUP.md 2-B-4"
  fi
}

# ---------------------------------------------------------------------------
# OMY-L100 teleop leader (IL data collection). See plan_il_vla.md 3.5, SETUP.md 2-D.
#
# Only SEVEN of open_manipulator's 13 packages are built. The rest get COLCON_IGNORE:
# the gui pulls Qt, and moveit_config/collision/playground/teleop are OMY-specific.
# open_manipulator_bringup IS built even though its package.xml declares
# gz_ros2_control / ros_gz_* -- it is ament_python, so colcon never resolves those
# and Gazebo is neither installed nor needed. Its share dir has to exist because
# omy_l100.urdf.xacro does $(find open_manipulator_bringup).
stage_leader() {
  step "leader — OMY-L100 teleop leader stack"
  local om="$WS/src/open_manipulator"
  if [ ! -d "$om" ]; then
    warn "$om missing — run the 'sources' stage first"; return 0
  fi

  # Verified on 2026-09-06: `0 upgraded, 2 newly installed, 0 to remove`.
  # Both come from the OFFICIAL ROS repo, not the NVIDIA one (which the pin stage
  # holds at Priority 100 precisely so it cannot shadow packages like these).
  apt_guarded_install ros-jazzy-dynamixel-sdk ros-jazzy-dynamixel-interfaces || return 1

  # ROS's setup.bash reads AMENT_TRACE_SETUP_FILES et al. without defaulting them,
  # so sourcing it under `set -u` aborts with "unbound variable". Drop -u across the
  # source only. (Invisible in --dry-run, which skips the source entirely -- this
  # cost a full fresh-machine run to find. See HISTORY.md 23.)
  if [ "$DRY_RUN" != 1 ]; then
    set +u; # shellcheck disable=SC1090
    source "/opt/ros/$ROS_DISTRO/setup.bash"; set -u
  fi
  run "cd '$WS' && colcon build --symlink-install --cmake-args -DBUILD_TESTING=OFF --packages-select robotis_interfaces dynamixel_hardware_interface open_manipulator_description open_manipulator_bringup om_gravity_compensation_controller om_spring_actuator_controller om_joint_trajectory_command_broadcaster"
  ok "leader stack built (7 packages)"
}

# ---------------------------------------------------------------------------
# udev for the U2D2 (OMY-L100 leader). SEPARATE stage because it is the only
# thing here that writes outside the workspace -- /etc/udev/rules.d is system
# wide and the rule matches EVERY ftdi_sio tty, so on a shared machine it also
# touches other projects' FTDI devices. The effects (mode 0666, latency_timer 1)
# are benign-to-beneficial, but it is opt-in on purpose. Real hardware only.
stage_udev() {
  step "udev — U2D2 (OMY-L100 leader), real hardware only"
  local src="$WS/src/open_manipulator/open_manipulator_bringup/open-manipulator-cdc.rules"
  local dst="/etc/udev/rules.d/99-open-manipulator-cdc.rules"
  [ -f "$src" ] || { warn "$src missing — run the 'sources' stage first"; return 0; }
  echo "  rule contents:"; sed 's/^/    /' "$src"
  echo "  -> mode 0666 (no sudo to open the port) and latency_timer=1."
  echo "     The FTDI default is 16 ms, which throttles a 4 Mbps DYNAMIXEL sync-read"
  echo "     to roughly 60 Hz. This single attribute is the difference between a"
  echo "     300 Hz leader and an unusable one."
  run "sudo cp '$src' '$dst'"
  run "sudo udevadm control --reload-rules"
  run "sudo udevadm trigger"
  if [ "$DRY_RUN" != 1 ]; then
    local lt=/sys/bus/usb-serial/devices/ttyUSB0/latency_timer
    if [ -e "$lt" ]; then
      [ "$(cat "$lt")" = "1" ] && ok "ttyUSB0 latency_timer = 1" \
        || warn "ttyUSB0 latency_timer = $(cat "$lt") (expected 1) — replug the U2D2"
    else
      warn "no /dev/ttyUSB0 yet — plug the U2D2 in, then re-run this stage"
    fi
  fi
  echo "  smoke test WITHOUT the L100 attached:"
  echo "    ros2 launch open_manipulator_bringup omy_l100_leader_ai.launch.py \\"
  echo "        use_mock_hardware:=true use_self_collision_avoidance:=false"
  echo "    ros2 control list_controllers -c /leader/controller_manager   # 4 active"
}

# ---------------------------------------------------------------------------
# ML environment. Deliberately a workspace-local venv:
#   * system python must not gain torch -- other projects share this container
#   * Isaac's bundled python must not gain torch either
#   * `rm -rf deps/.venv-ml` undoes the whole thing
stage_ml() {
  step "ml — IL/VLA training + LeRobot conversion venv"
  # NOTE: two statements, not `local venv=... py="$venv/..."`. Bash expands all
  # arguments to `local` BEFORE assigning any of them, so the second would see
  # an unset venv and die under `set -u` (it did: bootstrap --dry-run crashed here).
  local venv="$WS/deps/.venv-ml"
  local py="$venv/bin/python"
  local torch_index="${TORCH_INDEX:-https://download.pytorch.org/whl/cu128}"

  if [ ! -x "$py" ]; then
    if [ "$DRY_RUN" = 1 ]; then echo "  [dry-run] would create venv $venv"; return 0; fi
    # --without-pip is REQUIRED, not an optimisation. Plain `python3 -m venv` runs
    # ensurepip, which this image does not ship (python3-venv is not installed),
    # so it fails outright -- and installing python3-venv would drag a SYSTEM-WIDE
    # python3.12 upgrade (measured: 7 packages), which our own isolation rule
    # forbids. Create the venv empty, then bootstrap pip inside it below.
    python3 -m venv --without-pip "$venv" || { err "venv creation failed"; return 1; }
    ok "created $venv (no pip yet -- bootstrapped below)"
  else ok "venv already present: $venv"; fi
  [ "$DRY_RUN" = 1 ] && { echo "  [dry-run] would install torch + $HERE/requirements-ml.txt"; return 0; }

  # `python3 -m venv` here has no ensurepip, and apt-installing python3-venv drags
  # a SYSTEM-WIDE python3.12 upgrade (measured: 7 packages) -- refused by our own
  # isolation rule. Bootstrap pip inside the venv instead.
  if ! "$py" -m pip --version >/dev/null 2>&1; then
    warn "venv has no pip (ensurepip missing); bootstrapping with get-pip.py"
    local gp; gp="$(mktemp /tmp/get-pip-XXXX.py)"
    curl -fsSL https://bootstrap.pypa.io/get-pip.py -o "$gp" || { err "get-pip download failed"; return 1; }
    "$py" "$gp" -q || { err "pip bootstrap failed"; return 1; }
    rm -f "$gp"
  fi
  ok "pip: $("$py" -m pip --version | cut -d" " -f1-2)"

  # torch FIRST and from the CUDA index -- the default PyPI wheel may lack sm_120.
  echo "  installing torch from $torch_index (large: several GB) ..."
  "$py" -m pip install -q --index-url "$torch_index" torch torchvision || {
    err "torch install failed"; return 1; }
  _verify_torch_arch "$py" || return 1

  echo "  installing $HERE/requirements-ml.txt ..."
  "$py" -m pip install -q -r "$HERE/requirements-ml.txt" || {
    err "requirements-ml install failed"; return 1; }
  # Re-check: a later dependency can silently pull a different torch build.
  _verify_torch_arch "$py" || {
    err "torch was replaced by something without sm_120 -- reinstall it from $torch_index"; return 1; }

  _install_shm_workaround "$venv" || return 1
  _install_robot_plugin "$py" || return 1

  ok "ML venv ready: $py"
  echo "  use it with:  $py -m ...   (never 'source' it into a ROS shell)"
}

# ---------------------------------------------------------------------------
# GR00T N1.7 (VLA) on top of the ML venv. Not in DEFAULT_STAGES: ACT-only users do
# not need transformers/diffusers/peft (19 packages). Two halves:
#   1. `lerobot[groot]` extra of the SAME pinned lerobot -- refused outright if the
#      resolver wants to change ANY installed package (a replaced torch = sm_120 gone).
#   2. deps/hf_cache: the model files. Either `hf download` (needs the gated-repo
#      token, SETUP.md 2-C-2) or COPIED IN BY HAND from another PC -- check_hf_cache.sh
#      verifies the hand-placed layout offline, which is how training runs anyway
#      (ml_env.sh sets HF_HUB_OFFLINE=1).
stage_groot() {
  step "groot — lerobot[groot] extra + HF cache (deps/hf_cache)"
  local venv="$WS/deps/.venv-ml"
  local py="$venv/bin/python"
  local hf="$WS/deps/hf_cache"
  [ -x "$py" ] || { err "ML venv missing -- run: $0 ml"; return 1; }
  if [ "$DRY_RUN" = 1 ]; then
    echo "  [dry-run] would pip install -r $HERE/requirements-groot.txt into $venv"
    echo "            (refusing if any already-installed package would change)"
    echo "  [dry-run] would mkdir $hf and run check_hf_cache.sh"
    return 0
  fi
  local rep; rep="$(mktemp /tmp/groot-plan-XXXX.json)"
  "$py" -m pip install -q --dry-run --report "$rep" -r "$HERE/requirements-groot.txt" || {
    rm -f "$rep"; err "pip could not resolve requirements-groot.txt"; return 1; }
  local changed
  changed="$("$py" - "$rep" <<'PLAN'
import json, sys, importlib.metadata as md
norm = lambda n: n.lower().replace("_", "-")
have = {norm(d.metadata["Name"]): d.version for d in md.distributions()}
for p in json.load(open(sys.argv[1])).get("install", []):
    n, v = norm(p["metadata"]["name"]), p["metadata"]["version"]
    if n in have and have[n] != v:
        print(f"{n} {have[n]} -> {v}")
PLAN
)"
  rm -f "$rep"
  if [ -n "$changed" ]; then
    err "lerobot[groot] would CHANGE already-installed packages -- refusing (isolation rule):"
    echo "$changed" | sed 's/^/    /'
    err "  a replaced torch loses sm_120 (SETUP.md 2-C). Resolve by hand, then re-run."
    return 1
  fi
  echo "  installing $HERE/requirements-groot.txt (transformers, diffusers, peft, ... ~1 GB) ..."
  "$py" -m pip install -q -r "$HERE/requirements-groot.txt" || { err "install failed"; return 1; }
  _verify_torch_arch "$py" || return 1
  "$py" -c "import transformers, lerobot.policies.groot.modeling_groot" 2>/dev/null || {
    err "lerobot groot policy not importable after install"; return 1; }
  ok "lerobot[groot] ready (transformers $("$py" -c 'import transformers;print(transformers.__version__)'))"
  mkdir -p "$hf" && chmod 700 "$hf"
  ok "HF cache: $hf  — every GR00T command needs:  source src/setup/ml_env.sh"
  if "$HERE/check_hf_cache.sh"; then ok "model files in place (offline OK)"
  else
    warn "model files NOT in place yet. Either:"
    echo "    a) copy deps/hf_cache/hub/models--nvidia--{GR00T-N1.7-3B,Cosmos-Reason2-2B} from a PC that has them"
    echo "       (layout + required files: SETUP.md 2-C-2, then re-run: src/setup/check_hf_cache.sh)"
    echo "    b) download (needs the gated-repo token, SETUP.md 2-C-2 gate steps):"
    echo "       source src/setup/ml_env.sh; HF_HUB_OFFLINE=0 deps/.venv-ml/bin/hf download nvidia/GR00T-N1.7-3B"
    echo "       HF_HUB_OFFLINE=0 deps/.venv-ml/bin/hf download nvidia/Cosmos-Reason2-2B --include 'config.json' 'tokenizer*' 'vocab.json' 'merges.txt' '*preprocessor_config.json'"
  fi
}

# A container started without --shm-size gets Docker's 64 MiB /dev/shm (the first
# machine's Isaac container did). PyTorch's default 'file_descriptor' sharing
# strategy hands batches between DataLoader workers through /dev/shm, so ANY
# num_workers>0 dies partway into training:
#     RuntimeError: unable to allocate shared memory (shm) for file <...> (11)
#     RuntimeError: DataLoader worker (pid ...) exited unexpectedly
# Raising /dev/shm needs the container recreated, which a shared container does
# not allow (HISTORY.md 24). A container made for this workspace should simply be
# started with `--shm-size=8g` or more (the 2026-09-16 container has 32 GiB and
# needs none of this; check_env.sh reports which case you are in). The workaround
# is installed regardless -- it is inert unless UR_WS_TORCH_SHM_FIX=1 is set.
#
# One ACT batch (8 x 2 cameras x 3x480x640 float32) is ~59 MiB, so /dev/shm holds
# barely one in-flight batch. Measured: 1 of 3 identical unfixed runs died, peak
# shm 43 MiB -- i.e. the failure is REAL but intermittent, which is worse than
# deterministic (a run can survive smoke-testing and then die hours into training).
#
# 'file_system' passes the same tensors as regular files in the temp dir instead.
# It must take effect in the WORKERS, not just the parent. Two traps:
#   - lerobot pins `dataloader_multiprocessing_context = "spawn"` (configs/train.py),
#     deliberately NOT inheriting parent state -- so a parent-side
#     set_sharing_strategy() never reaches the workers (measured: they still died).
#   - a sitecustomize.py in the venv is USELESS here: /usr/lib/python3.12/
#     sitecustomize.py already exists and the stdlib dir precedes site-packages on
#     sys.path, so ours is shadowed and silently never imported (measured).
# A .pth file has neither problem: `site` executes its `import` line in every
# interpreter that uses this venv, spawned workers included, and .pth files do not
# collide by name. It is gated on an env var so ordinary venv python start-up does
# not pay for importing torch; env vars propagate to spawned children for free.
#
# Measured on this box, ACT / batch 8 / 2 cameras / 200 steps:
#     num_workers=0            data_s 0.13   47 smp/s   40 s
#     file_system + 4 workers  data_s 0.001 161 smp/s   16 s   (identical loss)
_install_shm_workaround() {
  local venv="$1" sp
  sp="$("$venv/bin/python" -c 'import site; print(site.getsitepackages()[0])' 2>/dev/null)" \
    || { err "could not locate site-packages in $venv"; return 1; }

  # Remove the earlier, broken attempt if a previous setup run left one behind.
  [ -f "$sp/sitecustomize.py" ] && grep -q "ur_ws setup" "$sp/sitecustomize.py" 2>/dev/null \
    && rm -f "$sp/sitecustomize.py"

  cat > "$sp/ur_ws_shm_fix.py" <<'EOF'
"""Installed by ur_ws setup/setup.sh (stage `ml`) -- see SETUP.md 2-C.

A container started without --shm-size has a 64 MiB /dev/shm (Docker default) and
it cannot be enlarged without recreating the container (impossible when shared).
PyTorch's default 'file_descriptor' sharing strategy moves DataLoader batches
through /dev/shm; one ACT batch is ~59 MiB, so workers intermittently die with
"unable to allocate shared memory". 'file_system' uses the temp dir instead.

Loaded via ur_ws_shm_fix.pth when UR_WS_TORCH_SHM_FIX=1, so it reaches spawned
DataLoader workers too. Unset the variable for stock behaviour.
"""
import torch.multiprocessing as _mp

_mp.set_sharing_strategy("file_system")
EOF

  # site executes any .pth line beginning with "import". Keep it on ONE line --
  # that is a hard requirement of the .pth format. Failures must stay silent:
  # a raising .pth breaks every interpreter in the venv.
  cat > "$sp/ur_ws_shm_fix.pth" <<'EOF'
import os; os.environ.get("UR_WS_TORCH_SHM_FIX") == "1" and __import__("importlib").import_module("ur_ws_shm_fix")
EOF

  if "$venv/bin/python" -c 'import sys' 2>&1 | grep -q .; then
    err "the .pth broke interpreter start-up -- removing it"
    rm -f "$sp/ur_ws_shm_fix.pth" "$sp/ur_ws_shm_fix.py"; return 1
  fi
  local got
  got="$(UR_WS_TORCH_SHM_FIX=1 "$venv/bin/python" -c \
        'import torch.multiprocessing as m; print(m.get_sharing_strategy())' 2>/dev/null)"
  if [ "$got" != "file_system" ]; then
    err "shm workaround did not take effect (strategy=$got) -- train with --num_workers=0"
    return 1
  fi
  local shm_mb; shm_mb="$(df -m /dev/shm 2>/dev/null | awk 'NR==2{print $2}')"
  if [ -n "$shm_mb" ] && [ "$shm_mb" -lt 1024 ]; then
    ok "shm workaround active (/dev/shm is only ${shm_mb} MiB):"
    echo "       run training with  UR_WS_TORCH_SHM_FIX=1  (ml_env.sh sets it; see SETUP.md 2-C)"
  else
    ok "shm workaround installed but not needed here (/dev/shm = ${shm_mb:-?} MiB; the 64 MiB trap is a --shm-size issue)"
  fi
}

# Our LeRobot robot adapter (ur_bringup/lerobot_robot_ur16e_ros, --robot.type=ur16e_ros).
# lerobot discovers third-party robots by scanning installed distributions named
# `lerobot_robot_*`, so the package must be pip-installed into THIS venv; nothing
# else (PYTHONPATH, sourcing ROS) registers it. Editable, so edits to robot.py are
# live. Found missing on the 2026-09-16 fresh container: every rollout died with
# "argument --robot.type: invalid choice: 'ur16e_ros'" (HISTORY.md 48.8) -- the
# first machine had it installed by hand and no document said so.
_install_robot_plugin() {
  local py="$1" src="$WS/src/ur_bringup/lerobot_robot_ur16e_ros"
  [ -f "$src/pyproject.toml" ] || { err "robot plugin source missing: $src"; return 1; }
  "$py" -m pip install -q -e "$src" || { err "robot plugin install failed"; return 1; }
  "$py" - <<'EOF' || { err "lerobot does not see robot type ur16e_ros after install"; return 1; }
import importlib.metadata as md
assert any(d.metadata["Name"].startswith("lerobot_robot_ur16e_ros") for d in md.distributions()), "dist not installed"
EOF
  ok "LeRobot robot plugin installed (editable): lerobot_robot_ur16e_ros -> --robot.type=ur16e_ros"
}

_verify_torch_arch() {
  local py="$1" sm; sm="$(gpu_sm)"
  local out; out="$("$py" - <<'EOF' 2>&1
import torch
print(torch.__version__, torch.version.cuda, torch.cuda.is_available())
print(" ".join(torch.cuda.get_arch_list()))
EOF
)" || { err "torch import failed:"; echo "$out" | tail -5; return 1; }
  echo "  torch: $(head -1 <<<"$out")"
  echo "  archs: $(tail -1 <<<"$out")"
  if [ -z "$sm" ]; then warn "no GPU detected; skipping arch check"; return 0; fi
  if grep -qw "sm_$sm" <<<"$out"; then ok "torch supports this GPU (sm_$sm)"; return 0; fi
  err "torch has NO kernels for sm_$sm. Pick a CUDA index that does, e.g."
  err "  TORCH_INDEX=https://download.pytorch.org/whl/cu130 $0 ml"
  return 1
}

stage_verify() { step "verify"; "$HERE/check_env.sh"; }

# ---------------------------------------------------------------------------
main() {
  local want=()
  while [ $# -gt 0 ]; do
    case "$1" in
      --dry-run) DRY_RUN=1 ;;
      --list) for s in "${STAGES[@]}"; do printf "  %-10s %s\n" "$s" "${STAGE_DESC[$s]}"; done; return 0 ;;
      -h|--help) usage; return 0 ;;
      -*) usage; return 2 ;;
      *) want+=("$1") ;;
    esac
    shift
  done
  [ ${#want[@]} -eq 0 ] && want=("${DEFAULT_STAGES[@]}")
  # preflight downgrades "ROS missing" to a warning when the ros stage is in the plan.
  ROS_STAGED=0; [[ " ${want[*]} " == *" ros "* ]] && ROS_STAGED=1

  echo "${C_H}UR16e workspace setup${C_0}"
  echo "  WS=$WS  ROS_DISTRO=$ROS_DISTRO  DRY_RUN=$DRY_RUN  ALLOW_UPGRADES=$ALLOW_UPGRADES"
  [ -d "$WS/src" ] || { err "workspace src not found at $WS/src (set WS=...)"; return 1; }

  local unsimulated=() rc=0
  for s in "${want[@]}"; do
    [[ " ${STAGES[*]} " == *" $s "* ]] || { err "unknown stage '$s' (see --list)"; return 2; }
    # `set -e` would abort the whole run on a stage that returns non-zero. That is right
    # for a real install and WRONG for a dry run: later stages legitimately cannot be
    # simulated because they depend on side effects earlier stages did not perform (no
    # NVIDIA repos in sources.list -> `apt-get install -s` for cuMotion cannot resolve,
    # no cumotion -> no nvcc for build, ...). Aborting there hides the rest of the plan,
    # which is the only thing a dry run is for.
    "stage_$s" && rc=0 || rc=$?
    [ "$rc" = 0 ] && continue
    if [ "$DRY_RUN" = 1 ]; then
      warn "stage '$s' could not be simulated (rc=$rc) — see above; continuing"
      unsimulated+=("$s")
    else
      err "stage '$s' FAILED (rc=$rc) — stopping."
      return "$rc"
    fi
  done

  if [ ${#unsimulated[@]} -gt 0 ]; then
    echo
    warn "not simulated: ${unsimulated[*]}"
    echo "  These depend on what an earlier stage would have done (NVIDIA repos added,"
    echo "  CUDA installed, sources imported...). A dry run does none of it, so apt/cmake"
    echo "  cannot resolve them yet. This is EXPECTED on a fresh machine and does not"
    echo "  mean the real run will fail. Re-run --dry-run after the real 'pin repos'"
    echo "  stages if you want to preview the rest."
  fi
  echo; ok "done: ${want[*]}"
  echo "  next: source /opt/ros/$ROS_DISTRO/setup.bash && source $WS/install/setup.bash"
}
main "$@"
