#!/usr/bin/env bash
# Reproduce the UR16e ROS 2 + Isaac Sim workspace on a fresh machine.
#
#   ./setup.sh                 # everything (asks nothing, refuses unsafe apt actions)
#   ./setup.sh --dry-run       # print what WOULD happen, change nothing
#   ./setup.sh base sources build       # only these stages
#   ./setup.sh --list          # show stages
#
# Env overrides:
#   WS=/path/to/ur_ws          workspace root       (default: derived from this file,
#                              <ws>/src/setup/setup.sh -> <ws>)
#   BUILD_JOBS=2               colcon --parallel-workers AND make -j for every colcon
#                              build here (default 2: the host CPU is shared with other
#                              containers; the nvblox CUDA build otherwise takes every core)
#   TORCH_INDEX=<url>          PyTorch wheel index for the ml stage (default .../whl/cu128)
#   ML_UNLOCKED=1              ml/groot: do NOT use requirements-ml.lock as pip constraints
#                              (resolve today's latest transitive deps instead)
#   ALLOW_UPGRADES=ubuntu      accept upgrades of already-installed packages ONLY when
#                              they come from Ubuntu's own repos (noble-updates/-security).
#                              A FRESH Isaac Sim container needs this: its base libs are
#                              stale and ros-jazzy-desktop / moveit require the point
#                              releases (14 + 4 packages on 2026-09-16, 23 + 4 on
#                              2026-10-21: the count drifts, the ORIGIN is what is checked).
#                              This is what `bootstrap.sh --fresh` sets.
#   ALLOW_UPGRADES=1           accept EVERY upgrade/removal apt proposes (last resort)
#   ROS_APT_SOURCE_VERSION=x.y.z  ros stage: another ros2-apt-source release than the
#   ROS_APT_SOURCE_SHA256=<hex>   pinned 1.3.0 -- needs its sha256 too (or =skip)
#
# A STOCK Isaac Sim image has no sudo/git/gpg: run container_prereqs.sh as root first
# (preflight prints the exact command).
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
HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"
# Same rule as bootstrap.sh: the workspace is wherever this file lives (WS=... overrides).
# Without it a moved workspace silently checked/built /isaac-sim/volume/ur_ws instead.
export WS="${WS:-$(cd "$HERE/../.." && pwd)}"
# shellcheck source=lib.sh
source "$HERE/lib.sh"
ROS_STAGED=0
: "${BUILD_JOBS:=2}"
[[ "$BUILD_JOBS" =~ ^[1-9][0-9]*$ ]] || { echo "BUILD_JOBS must be a positive integer (got '$BUILD_JOBS')" >&2; exit 2; }
# Every colcon build goes through this: --parallel-workers caps packages built at once,
# MAKEFLAGS caps the compiler jobs inside one package (colcon-cmake honours an explicit -j).
COLCON_BUILD="MAKEFLAGS=-j$BUILD_JOBS colcon build --parallel-workers $BUILD_JOBS"

# ros stage: the ros2-apt-source release (repo file + ROS apt key) is PINNED and checked.
# 1.3.0 = what the 2026-09-16 fresh container installed (ros2-apt-source 1.3.0~noble);
# the sha256 is of .../releases/download/1.3.0/ros2-apt-source_1.3.0.noble_all.deb,
# re-downloaded 2026-10-21 and identical to the 2026-09-16 copy. For a newer release set
# BOTH: ROS_APT_SOURCE_VERSION=x.y.z ROS_APT_SOURCE_SHA256=<sha256 of its .noble deb>
# (or ROS_APT_SOURCE_SHA256=skip to install unverified).
ROS_APT_SOURCE_VERSION_DEFAULT=1.3.0
ROS_APT_SOURCE_SHA256_DEFAULT=f31d84adf5054c7d60ded0e82c0f776a77ea33af53d08f40b9ad7c94cca55296
: "${ROS_APT_SOURCE_VERSION:=$ROS_APT_SOURCE_VERSION_DEFAULT}"
if [ -z "${ROS_APT_SOURCE_SHA256:-}" ] && [ "$ROS_APT_SOURCE_VERSION" = "$ROS_APT_SOURCE_VERSION_DEFAULT" ]; then
  ROS_APT_SOURCE_SHA256="$ROS_APT_SOURCE_SHA256_DEFAULT"
fi
: "${ROS_APT_SOURCE_SHA256:=}"

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
  [base]="core ROS packages (UR, MoveIt, ros2_control, robotiq_description, teleop) + scipy/matplotlib"
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
      6.0.1*|6.1.0-rc.26*) ok "Isaac $iv = a verified version (6.0.1, 6.1.0-rc.26)" ;;
      6.1.0*) warn "Isaac $iv: the 6.1.0 line, but only 6.1.0-rc.26 was verified (GA / other builds untested)."
              warn "Expect no API change; still run SETUP.md 5 'smoke' first and record the result in HISTORY.md." ;;
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
    [ "$sm" != 75 ] && echo "     -> apt nvblox is built for sm_75 only (no PTX); the build stage confirms that with cuobjdump and compiles nvblox_ros from source for sm_$sm."
    # Driver: the cumotion stage pulls the CUDA 13 toolkit; a driver below R580 (= CUDA 12.x)
    # cannot run CUDA 13 code, so cuMotion / the source-built nvblox die at start-up. The
    # driver is the HOST's (the container only sees it), so this can only warn.
    local drv; drv="$(nvidia-smi --query-gpu=driver_version --format=csv,noheader 2>/dev/null | head -1 | tr -d ' ')"
    if [[ "${drv%%.*}" =~ ^[0-9]+$ ]] && [ "${drv%%.*}" -lt 580 ]; then
      warn "NVIDIA driver $drv < R580: cuMotion / nvblox (CUDA 13 toolkit) will not start -- upgrade the HOST driver"
    elif [ -n "$drv" ]; then ok "NVIDIA driver $drv (>= R580 for the CUDA 13 toolkit)"; fi
  else
    warn "no GPU visible (nvidia-smi failed). Isaac and cuMotion will not run."
  fi
  # /dev/shm: Docker's default 64 MiB (no --shm-size) breaks DataLoader workers in the ML
  # track (SETUP.md 2-C). Fixable only by recreating the container, so say it BEFORE setup.
  local shm_mb; shm_mb="$(df -m /dev/shm 2>/dev/null | awk 'NR==2{print $2}')"
  if [ -n "$shm_mb" ] && [ "$shm_mb" -lt 1024 ]; then
    warn "/dev/shm is ${shm_mb} MiB (container made without --shm-size): sim/teleop are fine, ML training then needs"
    warn "  the file_system workaround or --num_workers=0 (SETUP.md 2-C). A new container: docker run --shm-size=8g"
  elif [ -n "$shm_mb" ]; then ok "/dev/shm: ${shm_mb} MiB"; fi

  # Container prerequisites. A STOCK Isaac Sim 6.1.0 image has none of sudo / git / gpg
  # (only gpgv) / locales: on this machine they came from an extra root layer
  # (2026-09-16 13:23, /var/log/apt/history.log), now container_prereqs.sh.
  #   sudo      every apt stage, non-interactively -> needs a NOPASSWD rule
  #   git, gpg  vcs import + nvblox submodule; repos stage `curl | gpg --dearmor`
  #   locales   ros stage `locale-gen en_US.UTF-8`
  local pre=0 c
  for c in git curl gpg; do need_cmd "$c" || pre=1; done
  if ! command -v sudo >/dev/null 2>&1; then err "missing command: sudo"; pre=1
  elif [ "$(id -u)" != 0 ] && ! sudo -n true 2>/dev/null; then
    err "sudo asks for a password (or $(id -un 2>/dev/null) is not in sudoers) -- the apt stages run unattended"; pre=1
  fi
  if locale -a 2>/dev/null | grep -qiE '^en_US\.utf-?8$' || command -v locale-gen >/dev/null 2>&1; then
    ok "locale: en_US.UTF-8 available or locale-gen present"
  else err "no en_US.UTF-8 locale and no locale-gen (package 'locales')"; pre=1; fi
  if [ "$pre" = 0 ]; then ok "container prerequisites: sudo (non-interactive), git, curl, gpg, locales"
  else
    err "container prerequisites missing (a stock Isaac Sim image lacks them). On the HOST, once:"
    err "    $(prereqs_hint)"
    fail=1
  fi
  echo "     colcon parallelism: BUILD_JOBS=$BUILD_JOBS (shared host CPU; BUILD_JOBS=N to change)"
  command -v vcs >/dev/null 2>&1 && ok "vcstool present" \
    || warn "vcstool missing — the base stage installs python3-vcstool"

  # Disk, in TWO places. apt writes the container's root filesystem (/opt/ros, /usr/local/cuda,
  # /usr), not the workspace volume. Measured 2026-10-21 on a stock 6.1.0 image: apt
  # installed ~11 GB (CUDA 4.8 + Nsight 2.4 + ROS 2.8 + rest) and the container layer grew
  # ~17 GB in all. The workspace volume gets the ML venv (7.9 GB) + its pip cache (~1.4 GB)
  # + build/install (~0.6 GB) + GR00T models (6.5 GB, groot only) + datasets/checkpoints.
  # WARN only, like before: the numbers are measurements, not hard limits.
  local want_root=2 want_ws=30 free_root free_ws
  if [ ! -d "/opt/ros/$ROS_DISTRO" ]; then want_root=20      # ROS + CUDA + Nsight + base libs
  elif ! ls -d /usr/local/cuda-* >/dev/null 2>&1; then want_root=10; fi   # CUDA + Nsight (cumotion)
  free_root="$(df -BG --output=avail /usr 2>/dev/null | tail -1 | tr -dc '0-9')"
  free_ws="$(df -BG --output=avail "$WS" 2>/dev/null | tail -1 | tr -dc '0-9')"
  # A bind-mounted volume on the same disk as the container's overlay reports the same
  # totals (here both are one NVMe partition): then both needs come out of one pool.
  if [ -n "$free_root" ] && [ -n "$free_ws" ] \
     && [ "$(stat -f -c '%b %S' /usr 2>/dev/null)" = "$(stat -f -c '%b %S' "$WS" 2>/dev/null)" ]; then
    [ "$free_ws" -ge $((want_root + want_ws)) ] \
      && ok "disk free: ${free_ws} GiB (container root fs and workspace share one disk; want >= $((want_root + want_ws)))" \
      || warn "only ${free_ws} GiB free on the disk shared by the container root fs and $WS; want >= $((want_root + want_ws)) ($want_root apt + $want_ws workspace)"
  else
    if [ -n "$free_root" ]; then
      [ "$free_root" -ge "$want_root" ] && ok "disk free (container root fs, where apt installs): ${free_root} GiB (want >= $want_root)" \
        || warn "only ${free_root} GiB free on the container root fs (apt installs there); want >= $want_root"
    fi
    if [ -n "$free_ws" ]; then
      [ "$free_ws" -ge "$want_ws" ] && ok "disk free (workspace $WS): ${free_ws} GiB (want >= $want_ws)" \
        || warn "only ${free_ws} GiB free for $WS; want >= $want_ws"
    fi
  fi

  # Network: every later stage downloads. One HEAD per host, with the scheme the stage
  # uses (the Ubuntu/ROS apt repos are plain http; packages.ros.org has no valid https
  # certificate). Any HTTP answer counts -- a 403/404 still proves the route; 6 s cap
  # each. WARN only: an apt/pip proxy can make a host unreachable to curl yet usable to
  # apt. Skipped without curl (reported above).
  if command -v curl >/dev/null 2>&1; then
    local h bad=()
    for h in http://archive.ubuntu.com http://packages.ros.org https://github.com \
             https://isaac.download.nvidia.com https://developer.download.nvidia.com https://repo.download.nvidia.com \
             https://gitlab.com https://download.pytorch.org https://pypi.org https://files.pythonhosted.org https://bootstrap.pypa.io; do
      curl -s -o /dev/null --head --connect-timeout 4 --max-time 6 "$h/" || bad+=("${h#*://}")
    done
    if [ ${#bad[@]} -eq 0 ]; then ok "network: download hosts reachable (Ubuntu, ROS, GitHub, NVIDIA x3, GitLab/Eigen, PyTorch, PyPI)"
    else warn "network: NOT reachable: ${bad[*]} -- the stages that download from them will fail (proxy? DNS?)"; fi
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
# point releases (14 packages on 2026-09-16, 23 on 2026-10-21 -- util-linux, dpkg, zlib1g,
# libsystemd0, ...).
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
  apt_lists_refresh || return 1
  apt_guarded_install locales curl || return 1
  if locale -a 2>/dev/null | grep -qi "en_US.utf8"; then ok "locale en_US.UTF-8 present"
  else run "sudo locale-gen en_US en_US.UTF-8 && sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8" \
         || { err "locale-gen failed"; return 1; }; fi
  # `software-properties-common` exists ONLY to provide `add-apt-repository universe`.
  # It depends on packagekit, which on a fresh noble image drags 11 base-lib upgrades
  # (util-linux, libsystemd0, ...) and trips the guard. Ubuntu's official docker/Isaac
  # images already ship `universe` in ubuntu.sources, so install it only when needed.
  if apt-cache policy 2>/dev/null | grep -q "${codename}/universe"; then ok "universe component enabled"
  else
    apt_guarded_install software-properties-common || return 1
    run "sudo add-apt-repository -y universe" || { err "add-apt-repository universe failed"; return 1; }
  fi
  if [ -f /etc/apt/sources.list.d/ros2.sources ] || apt_installed ros2-apt-source; then
    ok "ros2-apt-source already installed"
  else
    # PINNED release + sha256 (it installs the apt key every later ROS package is trusted
    # by, so "whatever GitHub calls latest today" is not good enough). See the variables
    # at the top of this file; a newer release needs BOTH overrides.
    local ver="$ROS_APT_SOURCE_VERSION" sum="$ROS_APT_SOURCE_SHA256"
    local url="https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ver}/ros2-apt-source_${ver}.${codename}_all.deb"
    if [ -z "$sum" ]; then
      err "ROS_APT_SOURCE_VERSION=$ver has no sha256: set ROS_APT_SOURCE_SHA256=<sha256 of $url>"
      err "  (curl -fsSL '$url' | sha256sum), or ROS_APT_SOURCE_SHA256=skip to install it unverified"
      return 1
    fi
    if [ "$DRY_RUN" = 1 ]; then
      echo "  [dry-run] would download ros2-apt-source $ver ($codename) from"
      echo "            $url"
      echo "            check sha256 = $sum, dpkg -i it (adds ros2.sources + keyring), delete the .deb"
    else
      local tmpd deb got
      tmpd="$(mktemp -d)"; deb="$tmpd/ros2-apt-source_${ver}.${codename}_all.deb"
      curl -fsSL -o "$deb" "$url" || { rm -rf "$tmpd"; err "download failed: $url"; return 1; }
      if [ "$sum" = skip ]; then warn "ROS_APT_SOURCE_SHA256=skip: installing $url UNVERIFIED"
      else
        got="$(sha256sum "$deb" | cut -d' ' -f1)"
        [ "$got" = "$sum" ] || { rm -rf "$tmpd"; err "sha256 mismatch for $url: got $got, want $sum -- not installing"; return 1; }
        ok "sha256 verified: ros2-apt-source $ver ($codename)"
      fi
      sudo dpkg -i "$deb" >/dev/null || { rm -rf "$tmpd"; err "dpkg -i $deb failed"; return 1; }
      rm -rf "$tmpd"   # (it used to stay behind in /tmp)
      ok "ros2-apt-source $ver installed (ros2.sources + keyring)"
    fi
  fi
  if [ "$DRY_RUN" = 1 ]; then
    echo "  [dry-run] would: sudo apt-get update && install ros-$ROS_DISTRO-desktop ros-dev-tools (guarded)"
    return 0
  fi
  run "sudo apt-get update -qq" || { err "apt-get update failed"; return 1; }
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
  [ $? = 0 ] && [ -s "$f" ] || { err "could not write $f"; return 1; }
  ok "wrote $f"
}

# Overridable only so the stage can be exercised against a scratch dir in tests.
APT_SOURCES_D="${APT_SOURCES_D:-/etc/apt/sources.list.d}"
APT_KEYRINGS_D="${APT_KEYRINGS_D:-/usr/share/keyrings}"
stage_repos() {
  step "repos — Isaac ROS / CUDA / VPI"
  [ -f /etc/apt/preferences.d/99-nvidia-isolate.pref ] || [ "$PIN_STAGED" = 1 ] || {
    err "pin missing. Run the 'pin' stage first (order matters)."; return 1; }
  need_cmd curl || return 1
  need_cmd gpg || return 1
  # A repo counts as added only when BOTH its .list and a non-empty keyring exist. The
  # key goes in first and the .list last: a failed key fetch (network, proxy, no gpg)
  # used to leave a .list pointing at a missing keyring, and every re-run then said
  # "already added" and never fetched the key again (manual rm was the only way out).
  _repo() { # name key_url list_line
    local name="$1" key="$2" line="$3"
    local list="$APT_SOURCES_D/$name.list" kr="$APT_KEYRINGS_D/$name.gpg"
    if [ -f "$list" ] && [ -s "$kr" ]; then ok "$name already added"; return 0; fi
    [ -f "$list" ] && warn "$name: $list exists but $kr is missing/empty (an earlier run failed halfway) -- re-adding"
    if [ "$DRY_RUN" = 1 ]; then echo "  [dry-run] would add repo $name (key -> $kr, then $list)"; return 0; fi
    local tmp; tmp="$(mktemp)"
    if ! curl -fsSL "$key" | gpg --batch --yes --dearmor -o "$tmp" || [ ! -s "$tmp" ]; then
      rm -f "$tmp"; err "$name: could not fetch/dearmor the key $key (network? proxy?)"; return 1
    fi
    sudo install -m 0644 "$tmp" "$kr" || { rm -f "$tmp"; err "$name: could not install $kr"; return 1; }
    rm -f "$tmp"
    echo "$line" | sudo tee "$list" >/dev/null || { err "$name: could not write $list"; return 1; }
    ok "added $name"
  }
  _repo nvidia-isaac-ros "https://isaac.download.nvidia.com/isaac-ros/repos.key" \
    "deb [signed-by=$APT_KEYRINGS_D/nvidia-isaac-ros.gpg] https://isaac.download.nvidia.com/isaac-ros/release-4 noble main" || return 1
  _repo nvidia-cuda "https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/3bf863cc.pub" \
    "deb [signed-by=$APT_KEYRINGS_D/nvidia-cuda.gpg] https://developer.download.nvidia.com/compute/cuda/repos/ubuntu2404/x86_64/ /" || return 1
  _repo nvidia-vpi "https://repo.download.nvidia.com/jetson/jetson-ota-public.asc" \
    "deb [signed-by=$APT_KEYRINGS_D/nvidia-vpi.gpg] https://repo.download.nvidia.com/jetson/x86_64/noble r38.2 main" || return 1
  run "sudo apt-get update -qq" || { err "apt-get update failed after adding the NVIDIA repos -- see the apt errors above"; return 1; }
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
    python3-vcstool python3-colcon-common-extensions \
    python3-scipy python3-matplotlib || return 1
  # scipy / matplotlib: base_placement/ (system python3; scipy in ur16e_kin.py, matplotlib
  # for place_base.py --plot). ros-jazzy-desktop happens to pull both in via an
  # alternative-dependency chain (rqt-plot -> matplotlib -> fonttools "scipy | munkres"),
  # so a ros-base machine lacked them -- name them explicitly (a no-op where present).
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
  # (In a dry run on a fresh box vcs is not there yet -- the base stage installs it.)
  if ! command -v vcs >/dev/null 2>&1; then
    if [ "$DRY_RUN" = 1 ]; then warn "vcs not installed yet (the base stage installs python3-vcstool)"
    else err "missing command: vcs (run the base stage)"; return 1; fi
  fi
  need_cmd git || [ "$DRY_RUN" = 1 ] || return 1
  run "cd '$WS' && vcs import src < src/ur16e.repos" || { err "vcs import failed (network? see above)"; return 1; }
  did "vcs import src < src/ur16e.repos"
  local nv="$WS/src/isaac_ros_nvblox"
  if [ -d "$nv" ] || [ "$DRY_RUN" = 1 ]; then
    # nvblox_core is a git submodule; vcs import does NOT fetch it.
    run "cd '$nv' && git submodule update --init --recursive --depth 1" \
      || { err "nvblox_core submodule update failed"; return 1; }
    did "nvblox_core submodule initialised"
  else
    err "$nv missing after vcs import"; return 1
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
      if [ -d "$om/$p" ]; then run "touch '$om/$p/COLCON_IGNORE'" || return 1; fi
    done
    did "COLCON_IGNORE set on 6 unused open_manipulator packages"
  fi
}

stage_build() {
  step "build — colcon (BUILD_JOBS=$BUILD_JOBS)"
  # ROS's setup.bash reads AMENT_TRACE_SETUP_FILES et al. without defaulting them,
  # so sourcing it under `set -u` aborts with "unbound variable". Drop -u across the
  # source only. (Invisible in --dry-run, which skips the source entirely -- this
  # cost a full fresh-machine run to find. See HISTORY.md 23.)
  if [ "$DRY_RUN" != 1 ]; then
    set +u; # shellcheck disable=SC1090
    source "/opt/ros/$ROS_DISTRO/setup.bash"; set -u
  fi
  run "cd '$WS' && $COLCON_BUILD --symlink-install --packages-up-to ur_bringup --cmake-args -DBUILD_TESTING=OFF" \
    || { err "colcon build ur_bringup failed"; return 1; }
  did "ur_bringup + topic_based built"

  if [ -d "$WS/src/ros2_robotiq_gripper" ]; then
    run "cd '$WS' && $COLCON_BUILD --symlink-install --packages-select serial robotiq_driver robotiq_controllers --cmake-args -DBUILD_TESTING=OFF" \
      || { err "colcon build of the real 2F-85 gripper driver failed"; return 1; }
    did "real 2F-85 gripper driver built"
  fi

  # nvblox: the apt binary is compiled for a fixed GPU arch list with no PTX.
  # Build from source only when this GPU is not covered.
  local sm archs nvbin="/opt/ros/$ROS_DISTRO/lib/nvblox_ros/nvblox_node"
  sm="$(gpu_sm)"; archs="$(cuda_archs_of "$nvbin")"
  echo "  GPU sm_${sm:-?} ; apt nvblox_node archs: ${archs:-<unknown>}"
  if [ -n "$sm" ] && [ -n "$archs" ] && grep -qw "sm_$sm" <<<"$archs"; then
    ok "apt nvblox supports this GPU — no source build needed"
  elif [ "$DRY_RUN" = 1 ] && [ ! -d "$WS/src/isaac_ros_nvblox" ]; then
    echo "  [dry-run] would build nvblox_ros from source for sm_${sm:-?} (isaac_ros_nvblox is imported by the sources stage)"
  elif [ -d "$WS/src/isaac_ros_nvblox" ]; then
    warn "apt nvblox does NOT cover sm_${sm:-?} -> building nvblox_ros from source"
    local nvcc; nvcc="$(ls /usr/local/cuda*/bin/nvcc 2>/dev/null | head -1 || true)"
    [ -n "$nvcc" ] || { err "nvcc not found — run the cumotion stage first"; return 1; }
    run "cd '$WS' && CUDACXX='$nvcc' PATH=\"\$(dirname '$nvcc'):\$PATH\" $COLCON_BUILD --symlink-install --packages-select nvblox_ros --cmake-args -DUSE_NATIVE_CUDA_ARCHITECTURE=1 -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF" \
      || { err "colcon build nvblox_ros failed"; return 1; }
    did "nvblox_ros built for sm_$sm (workspace overlay only)"
  else
    warn "nvblox source not present and apt build may not run on this GPU — see SETUP.md 2-B-4"
  fi
  _cmake_registry_cleanup || return 1
}

# nvblox_core downloads Eigen 3.4.0 as an ExternalProject, and Eigen's CMakeLists ends in
# `export(PACKAGE Eigen3)` under cmake_minimum_required(3.5) -- CMP0090 OLD, so CMake
# writes a USER package registry entry ~/.cmake/packages/Eigen3/<md5 of the dir> that
# points into $WS/build. It is outside the workspace, dangles once build/ is deleted,
# and until then find_package(Eigen3) of ANY other project of this user can pick up our
# build tree. -DCMAKE_EXPORT_NO_PACKAGE_REGISTRY=ON on the colcon command line cannot
# stop it: ExternalProject configures Eigen in a separate cmake run that only gets
# -DCMAKE_INSTALL_PREFIX (nvblox's eigen.cmake), and nvblox links Eigen by include path,
# not through the registry. So remove exactly the entries that point into $WS/build.
_cmake_registry_cleanup() {
  local reg="$HOME/.cmake/packages" f n=0
  [ -d "$reg" ] || return 0
  while IFS= read -r -d '' f; do
    case "$(head -1 "$f" 2>/dev/null)" in
      "$WS/build/"*) run "rm -f '$f'" || return 1; n=$((n + 1)) ;;
    esac
  done < <(find "$reg" -mindepth 2 -maxdepth 2 -type f -print0 2>/dev/null)
  [ "$n" -gt 0 ] && did "removed $n CMake user-registry entr$([ "$n" = 1 ] && echo y || echo ies) pointing into $WS/build ($reg)"
  # Empty dirs only (rmdir refuses anything else); a dry run leaves them.
  [ "$DRY_RUN" = 1 ] || rmdir "$reg"/* "$reg" "$HOME/.cmake" 2>/dev/null || true
  return 0
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
    if [ "$DRY_RUN" = 1 ]; then
      echo "  [dry-run] would apt-guard ros-$ROS_DISTRO-dynamixel-{sdk,interfaces} and build the 7 leader packages (after the sources stage imports them)"
      return 0
    fi
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
  run "cd '$WS' && $COLCON_BUILD --symlink-install --cmake-args -DBUILD_TESTING=OFF --packages-select robotis_interfaces dynamixel_hardware_interface open_manipulator_description open_manipulator_bringup om_gravity_compensation_controller om_spring_actuator_controller om_joint_trajectory_command_broadcaster" \
    || { err "colcon build of the leader stack failed"; return 1; }
  did "leader stack built (7 packages)"
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
  # udev is a HOST daemon. Inside a container this stage writes a rules file the
  # host's udevd never reads, and /sys is read-only -- so it would report success
  # and change nothing (seen 2026-09-17: latency_timer stayed 16). Say so instead.
  if [ -f /.dockerenv ] || grep -qE 'docker|containerd|kubepods' /proc/1/cgroup 2>/dev/null; then
    warn "this is a container: udev rules must be installed on the HOST, not here."
    echo "  On the host, run:"
    echo "    sudo tee /etc/udev/rules.d/99-open-manipulator-cdc.rules >/dev/null <<'EOF'"
    sed 's/^/    /' "$src"
    echo "    EOF"
    echo "    sudo udevadm control --reload-rules && sudo udevadm trigger   # then replug the U2D2"
    echo "  Quick check without a rule (until replug): echo 1 | sudo tee /sys/bus/usb-serial/devices/ttyUSB0/latency_timer"
    echo "  Then verify from inside the container: cat /sys/bus/usb-serial/devices/ttyUSB0/latency_timer  -> 1"
    local lt=/sys/bus/usb-serial/devices/ttyUSB0/latency_timer
    if [ -e "$lt" ]; then
      [ "$(cat "$lt")" = "1" ] && ok "ttyUSB0 latency_timer = 1 (host already configured)" \
        || warn "ttyUSB0 latency_timer = $(cat "$lt") -> do the host steps above"
    fi
    return 0
  fi
  echo "  rule contents:"; sed 's/^/    /' "$src"
  echo "  -> mode 0666 (no sudo to open the port) and latency_timer=1."
  echo "     The FTDI default is 16 ms, which throttles a 4 Mbps DYNAMIXEL sync-read"
  echo "     to roughly 60 Hz. This single attribute is the difference between a"
  echo "     300 Hz leader and an unusable one."
  run "sudo cp '$src' '$dst'" || { err "could not write $dst"; return 1; }
  run "sudo udevadm control --reload-rules" || { err "udevadm reload failed"; return 1; }
  run "sudo udevadm trigger" || warn "udevadm trigger failed -- replug the U2D2"
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
  echo "    ros2 launch ur_bringup omy_leader.launch.py \\"
  echo "        use_mock_hardware:=true use_self_collision_avoidance:=false"
  echo "    ros2 control list_controllers -c /leader/controller_manager   # 4 active"
}

# ---------------------------------------------------------------------------
# ML environment. Deliberately a workspace-local venv:
#   * system python must not gain torch -- other projects share this container
#   * Isaac's bundled python must not gain torch either
#   * `rm -rf deps/.venv-ml` undoes the whole thing
#
# Reproducibility: torch/torchvision are PINNED (lerobot 0.6.1 needs torch<2.12; an
# unpinned install took whatever the index served newest), and requirements-ml.lock --
# the verified 2026-09-16 venv -- is passed to pip as CONSTRAINTS, so transitive deps
# (pyarrow, huggingface-hub, ...) resolve to the verified versions instead of today's
# latest (measured 2026-10-20: 32 packages differed without it). ML_UNLOCKED=1 opts out.
TORCH_VERSION="${TORCH_VERSION:-2.11.0}"
TORCHVISION_VERSION="${TORCHVISION_VERSION:-0.26.0}"
PIP_VERSION="${PIP_VERSION:-26.2.1}"   # what get-pip.py installed in the verified venv
# Local version tag of the index, e.g. ".../whl/cu128" -> "+cu128". The tag is what makes
# pip REPLACE a torch of another CUDA build (2.11.0+cu128 already satisfies "==2.11.0").
_torch_tag() { local t="${1%/}"; t="${t##*/}"; [[ "$t" =~ ^cu[0-9]+$ ]] && echo "+$t"; return 0; }

# Writes the pip constraints file to $1: the lock minus the lines that must come from the
# PyTorch index (torch / torchvision / triton / nvidia-* / cuda-*: torch pins those itself,
# and a cu130 build needs different ones), plus the torch pin of THIS install.
_ml_constraints() {
  local out="$1" tag="$2" lock="$HERE/requirements-ml.lock"
  [ -f "$lock" ] || { err "$lock missing (or set ML_UNLOCKED=1)"; return 1; }
  grep -vE '^[[:space:]]*(#|$)' "$lock" \
    | grep -viE '^(-e[[:space:]]|torch==|torchvision==|triton==|nvidia-[a-z0-9-]+==|cuda-[a-z0-9-]+==|pip==|setuptools==|wheel==|lerobot[-_]robot[-_]ur16e[-_]ros)' \
    > "$out" || { err "could not filter $lock"; return 1; }
  printf 'torch==%s%s\ntorchvision==%s%s\n' "$TORCH_VERSION" "$tag" "$TORCHVISION_VERSION" "$tag" >> "$out"
}

# Sets ML_PIP_C (array): ("-c" <file>) or () when ML_UNLOCKED=1.
_ml_pip_c() {
  ML_PIP_C=()
  if [ "${ML_UNLOCKED:-0}" = 1 ]; then
    warn "ML_UNLOCKED=1: transitive deps resolve to today's latest, not requirements-ml.lock"
    return 0
  fi
  local c="$WS/deps/.ml-constraints.txt"
  # A dry run (groot shows the real pip plan) must not write into the workspace.
  [ "$DRY_RUN" = 1 ] && c="$(mktemp /tmp/ml-constraints-XXXX.txt)"
  mkdir -p "$WS/deps" && _ml_constraints "$c" "$(_torch_tag "$1")" || return 1
  ML_PIP_C=(-c "$c")
  ok "pip constraints: requirements-ml.lock -> $c ($(wc -l < "$c") pins)"
}

stage_ml() {
  step "ml — IL/VLA training + LeRobot conversion venv"
  # NOTE: two statements, not `local venv=... py="$venv/..."`. Bash expands all
  # arguments to `local` BEFORE assigning any of them, so the second would see
  # an unset venv and die under `set -u` (it did: bootstrap --dry-run crashed here).
  local venv="$WS/deps/.venv-ml"
  local py="$venv/bin/python"
  local torch_index="${TORCH_INDEX:-https://download.pytorch.org/whl/cu128}"
  local tag; tag="$(_torch_tag "$torch_index")"
  _ml_cache_env

  if [ ! -x "$py" ]; then
    if [ "$DRY_RUN" = 1 ]; then echo "  [dry-run] would create venv $venv + pip==$PIP_VERSION (get-pip.py)"
    else
      # --without-pip is REQUIRED, not an optimisation. Plain `python3 -m venv` runs
      # ensurepip, which this image does not ship (python3-venv is not installed),
      # so it fails outright -- and installing python3-venv would drag a SYSTEM-WIDE
      # python3.12 upgrade (measured: 7 packages), which our own isolation rule
      # forbids. Create the venv empty, then bootstrap pip inside it below.
      python3 -m venv --without-pip "$venv" || { err "venv creation failed"; return 1; }
      ok "created $venv (no pip yet -- bootstrapped below)"
    fi
  else ok "venv already present: $venv"; fi
  if [ "$DRY_RUN" = 1 ]; then
    echo "  [dry-run] would install torch==$TORCH_VERSION$tag torchvision==$TORCHVISION_VERSION$tag from $torch_index"
    echo "            then $HERE/requirements-ml.txt $([ "${ML_UNLOCKED:-0}" = 1 ] && echo "(ML_UNLOCKED=1: no constraints)" || echo "with -c requirements-ml.lock")"
    echo "            pip cache: $PIP_CACHE_DIR ; venv defaults TORCH_HOME/HF_HOME/PIP_CACHE_DIR -> $WS/deps/..."
    echo "            and fetch the ACT backbone resnet18 IMAGENET1K_V1 (45 MB) into $WS/deps/torch_home"
    return 0
  fi

  # `python3 -m venv` here has no ensurepip, and apt-installing python3-venv drags
  # a SYSTEM-WIDE python3.12 upgrade (measured: 7 packages) -- refused by our own
  # isolation rule. Bootstrap pip inside the venv instead.
  if ! "$py" -m pip --version >/dev/null 2>&1; then
    warn "venv has no pip (ensurepip missing); bootstrapping with get-pip.py"
    local gp; gp="$(mktemp /tmp/get-pip-XXXX.py)"
    curl -fsSL https://bootstrap.pypa.io/get-pip.py -o "$gp" || { rm -f "$gp"; err "get-pip download failed"; return 1; }
    # get-pip.py takes requirement specifiers: pin pip itself (the script is always latest).
    "$py" "$gp" -q "pip==$PIP_VERSION" || { rm -f "$gp"; err "pip bootstrap failed (pip==$PIP_VERSION)"; return 1; }
    rm -f "$gp"
  fi
  ok "pip: $("$py" -m pip --version | cut -d" " -f1-2)"

  _ml_pip_c "$torch_index" || return 1

  # torch FIRST and from the CUDA index -- the default PyPI wheel may lack sm_120.
  echo "  installing torch==$TORCH_VERSION$tag torchvision==$TORCHVISION_VERSION$tag from $torch_index (large: several GB) ..."
  "$py" -m pip install -q --index-url "$torch_index" \
      "torch==$TORCH_VERSION$tag" "torchvision==$TORCHVISION_VERSION$tag" || {
    err "torch install failed"; return 1; }
  _verify_torch_arch "$py" || return 1

  echo "  installing $HERE/requirements-ml.txt ..."
  "$py" -m pip install -q "${ML_PIP_C[@]}" -r "$HERE/requirements-ml.txt" || {
    err "requirements-ml install failed (a constraint conflict? ML_UNLOCKED=1 to resolve without the lock)"; return 1; }
  # Re-check: a later dependency can silently pull a different torch build.
  _verify_torch_arch "$py" || {
    err "torch was replaced by something without sm_120 -- reinstall it from $torch_index"; return 1; }

  _install_shm_workaround "$venv" || return 1
  _install_cache_paths "$venv" || return 1
  _prefetch_resnet18 "$py" || return 1
  _install_robot_plugin "$py" "${ML_PIP_C[@]}" || return 1

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
  _ml_cache_env
  # Constraints with the tag of the torch that is ALREADY installed, so they agree with it.
  local tv; tv="$("$py" -c 'import torch;print(torch.__version__)' 2>/dev/null)"
  _ml_pip_c "${tv#*+}" || return 1
  local rep; rep="$(mktemp /tmp/groot-plan-XXXX.json)"
  local prc=0 nocache=()
  [ "$DRY_RUN" = 1 ] && nocache=(--no-cache-dir)   # a dry run leaves no pip cache behind either
  "$py" -m pip install -q --dry-run "${nocache[@]}" --report "$rep" "${ML_PIP_C[@]}" -r "$HERE/requirements-groot.txt" || prc=$?
  # A dry run's constraints file is a mktemp in /tmp (_ml_pip_c) and has served its purpose
  # (it used to stay behind after every --dry-run).
  [ "$DRY_RUN" = 1 ] && [ ${#ML_PIP_C[@]} -eq 2 ] && rm -f "${ML_PIP_C[1]}"
  [ "$prc" = 0 ] || { rm -f "$rep"; err "pip could not resolve requirements-groot.txt"; return 1; }
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
  local nnew; nnew="$(grep -c . <<<"$(python3 -c 'import json,sys;[print(p["metadata"]["name"]) for p in json.load(open(sys.argv[1])).get("install",[])]' "$rep" 2>/dev/null)")"
  rm -f "$rep"
  if [ "$DRY_RUN" = 1 ]; then
    # The plan above is pip --dry-run (read-only), so a dry run can show the real answer.
    if [ -n "$changed" ]; then
      err "[dry-run] pip plan would CHANGE installed packages -- the real run refuses:"; echo "$changed" | sed 's/^/    /'; return 1
    fi
    echo "  [dry-run] pip plan: $nnew package(s) to install, 0 already-installed changed -> would install, then mkdir $hf + check_hf_cache.sh"
    return 0
  fi
  if [ -n "$changed" ]; then
    err "lerobot[groot] would CHANGE already-installed packages -- refusing (isolation rule):"
    echo "$changed" | sed 's/^/    /'
    err "  a replaced torch loses sm_120 (SETUP.md 2-C). Resolve by hand, then re-run."
    return 1
  fi
  echo "  installing $HERE/requirements-groot.txt (transformers, diffusers, peft, ... ~1 GB) ..."
  "$py" -m pip install -q "${ML_PIP_C[@]}" -r "$HERE/requirements-groot.txt" || { err "install failed"; return 1; }
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
    echo "       HF_HUB_OFFLINE=0 deps/.venv-ml/bin/hf download nvidia/Cosmos-Reason2-2B --include config.json --include 'tokenizer*' --include vocab.json --include merges.txt --include '*preprocessor_config.json'"
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

# Caches of the ML track stay INSIDE the workspace (deps/), not in the shared ~/.cache:
# on the 2026-10-21 stock-image run ~/.cache/pip grew 1.4 GB during this stage, and ACT
# training/rollouts put resnet18 in ~/.cache/torch and wrote ~/.cache/huggingface -- so a
# rollout needed internet. A value the caller exported still wins.
# (pip itself is pinned -- PIP_VERSION -- so its "new release available" check is noise.)
_ml_cache_env() {
  export PIP_CACHE_DIR="${PIP_CACHE_DIR:-$WS/deps/pip_cache}"
  export PIP_DISABLE_PIP_VERSION_CHECK=1
}

# The same defaults for EVERY process of the venv (lerobot-train, policy_server,
# robot_client, raw_to_lerobot.py ...), whether or not the caller sourced ml_env.sh -- the
# ACT harness scripts do not. A .pth for the reasons given at the shm fix above (reaches
# spawned workers; a venv sitecustomize.py is shadowed). os.environ.setdefault, so an
# exported value still wins. Relative to the venv (deps/.venv-ml -> deps/), so a moved
# workspace keeps working:
#   TORCH_HOME    deps/torch_home  torchvision weights (ACT's resnet18 backbone)
#   HF_HOME       deps/hf_cache    = ml_env.sh (GR00T models; datasets cache; lerobot calibration dir)
#   PIP_CACHE_DIR deps/pip_cache
_install_cache_paths() {
  local venv="$1" sp
  sp="$("$venv/bin/python" -c 'import site; print(site.getsitepackages()[0])' 2>/dev/null)" \
    || { err "could not locate site-packages in $venv"; return 1; }
  cat > "$sp/ur_ws_cache_paths.pth" <<'PTH'
import os, sys; _d = os.path.dirname(os.path.abspath(sys.prefix)); [os.environ.setdefault(_k, os.path.join(_d, _v)) for _k, _v in (("TORCH_HOME", "torch_home"), ("HF_HOME", "hf_cache"), ("PIP_CACHE_DIR", "pip_cache"))]
PTH
  if "$venv/bin/python" -c 'import sys' 2>&1 | grep -q .; then
    err "ur_ws_cache_paths.pth broke interpreter start-up -- removing it"; rm -f "$sp/ur_ws_cache_paths.pth"; return 1
  fi
  local got
  got="$(env -u TORCH_HOME -u HF_HOME -u PIP_CACHE_DIR "$venv/bin/python" -c 'import os; print(os.environ["TORCH_HOME"])' 2>/dev/null)"
  # (realpaths: a symlinked workspace path must not fail the comparison)
  [ -n "$got" ] && [ "$(readlink -m "$got")" = "$(readlink -m "$WS/deps/torch_home")" ] \
    || { err "venv cache defaults not effective (TORCH_HOME=${got:-unset})"; return 1; }
  ok "venv cache defaults: TORCH_HOME / HF_HOME / PIP_CACHE_DIR -> $WS/deps/{torch_home,hf_cache,pip_cache}"
}

# ACT (lerobot default pretrained_backbone_weights = ResNet18_Weights.IMAGENET1K_V1) builds
# its torchvision backbone WITH those weights whenever the policy is constructed -- also
# when a trained checkpoint is loaded for a rollout -- so without the file in TORCH_HOME
# every first rollout downloads it. Fetch it once here (hash-checked). The caller's own
# TORCH_HOME is dropped so the file lands where the .pth default points.
_prefetch_resnet18() {
  local py="$1" f
  f="$(env -u TORCH_HOME "$py" - <<'PY'
import os, torch
from torchvision.models import ResNet18_Weights
w = ResNet18_Weights.IMAGENET1K_V1
f = os.path.join(torch.hub.get_dir(), "checkpoints", os.path.basename(w.url))
if not os.path.isfile(f):
    w.get_state_dict(progress=False, check_hash=True)
print(f)
PY
)" || { err "could not fetch resnet18 IMAGENET1K_V1 into deps/torch_home (network?) -- ACT needs it"; return 1; }
  f="${f##*$'\n'}"   # torch.hub's "Downloading: ..." note can precede the path
  ok "ACT backbone weights: $f (rollouts download nothing)"
}

# Our LeRobot robot adapter (ur_bringup/lerobot_robot_ur16e_ros, --robot.type=ur16e_ros).
# lerobot discovers third-party robots by scanning installed distributions named
# `lerobot_robot_*`, so the package must be pip-installed into THIS venv; nothing
# else (PYTHONPATH, sourcing ROS) registers it. Editable, so edits to robot.py are
# live. Found missing on the 2026-09-16 fresh container: every rollout died with
# "argument --robot.type: invalid choice: 'ur16e_ros'" (HISTORY.md 48.8) -- the
# first machine had it installed by hand and no document said so.
_install_robot_plugin() {
  local py="$1" src="$WS/src/ur_bringup/lerobot_robot_ur16e_ros"; shift
  [ -f "$src/pyproject.toml" ] || { err "robot plugin source missing: $src"; return 1; }
  "$py" -m pip install -q "$@" -e "$src" || { err "robot plugin install failed"; return 1; }
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
  err "  (installs torch==$TORCH_VERSION+cu130 -- keep the 2.11 pin: lerobot 0.6.1 needs torch<2.12,"
  err "   and a newer torch from that index would be swapped for a PyPI build by the next step)"
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
  echo "  WS=$WS  ROS_DISTRO=$ROS_DISTRO  DRY_RUN=$DRY_RUN  ALLOW_UPGRADES=$ALLOW_UPGRADES  BUILD_JOBS=$BUILD_JOBS"
  [ -d "$WS/src" ] || { err "workspace src not found at $WS/src (set WS=...)"; return 1; }

  local unsimulated=() rc=0 preflight_failed=0
  for s in "${want[@]}"; do
    [[ " ${STAGES[*]} " == *" $s "* ]] || { err "unknown stage '$s' (see --list)"; return 2; }
    # `set -e` would abort the whole run on a stage that returns non-zero. That is right
    # for a real install and WRONG for a dry run: later stages legitimately cannot be
    # simulated because they depend on side effects earlier stages did not perform (no
    # NVIDIA repos in sources.list -> `apt-get install -s` for cuMotion cannot resolve,
    # no cumotion -> no nvcc for build, ...). Aborting there hides the rest of the plan,
    # which is the only thing a dry run is for.
    # NOTE: calling the stage on the left of && / || makes bash ignore `set -e` INSIDE it,
    # so a stage fails only through an explicit `|| return 1` -- every command that can
    # fail in a stage carries one (lib.sh run()). Keep it that way when adding stages.
    if [ "$s" = ml ] || [ "$s" = groot ]; then
      # The venv's pip must not see ROS python dists: build sources /opt/ros, and the
      # caller's shell usually has. With PYTHONPATH set, pip counts launch_ros & co. as
      # installed and prints "ERROR: pip's dependency resolver ..." in a successful run
      # (2026-10-20 fresh-workspace bootstrap). A subshell, because `local`+`unset` of
      # an exported variable does not remove it from the children's environment.
      ( unset PYTHONPATH AMENT_PREFIX_PATH; "stage_$s" ) && rc=0 || rc=$?
    else
      "stage_$s" && rc=0 || rc=$?
    fi
    [ "$rc" = 0 ] && continue
    if [ "$DRY_RUN" = 1 ] && [ "$s" = preflight ]; then
      # preflight is read-only, i.e. it DID run for real: the real run stops right here.
      err "preflight FAILED — a real run stops here. Continuing the preview only."
      preflight_failed=1
    elif [ "$DRY_RUN" = 1 ]; then
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
  if [ "$preflight_failed" = 1 ]; then
    echo; err "dry run finished, but preflight FAILED (see above) — fix that before the real run."
    return 3
  fi
  echo; ok "done: ${want[*]}$([ "$DRY_RUN" = 1 ] && echo "  (dry run: nothing was changed)")"
  echo "  next: source /opt/ros/$ROS_DISTRO/setup.bash && source $WS/install/setup.bash"
}
# Sourceable (for tests): `source setup.sh` defines the stages without running anything.
if [[ "${BASH_SOURCE[0]}" == "$0" ]]; then main "$@"; fi
