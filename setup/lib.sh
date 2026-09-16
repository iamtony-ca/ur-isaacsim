#!/usr/bin/env bash
# Shared helpers for the UR16e workspace provisioning scripts.
#
# The one rule these encode: THIS MACHINE IS SHARED WITH OTHER WORKSPACES.
# Every apt action is simulated first and refused if it would upgrade or remove
# anything that is already installed. Adding brand-new packages is safe; changing
# existing ones is how you break somebody else's project.
#
# ALLOW_UPGRADES=ubuntu relaxes that to "upgrades whose origin is Ubuntu's own repos
# are fine" -- what a FRESH container needs (stale base libs vs ros-jazzy-desktop),
# while anything from NVIDIA/ROS-shadowing origins or any removal is still refused.
# ALLOW_UPGRADES=1 accepts everything. See apt_guarded_install.

set -euo pipefail

WS_DEFAULT="/isaac-sim/volume/ur_ws"
: "${WS:=$WS_DEFAULT}"
: "${ROS_DISTRO:=jazzy}"
: "${DRY_RUN:=0}"
: "${ALLOW_UPGRADES:=0}"

C_OK=$'\033[32m'; C_WARN=$'\033[33m'; C_ERR=$'\033[31m'; C_H=$'\033[1m'; C_0=$'\033[0m'
ok()   { echo "${C_OK}  ok${C_0}   $*"; }
warn() { echo "${C_WARN}  warn${C_0} $*"; }
err()  { echo "${C_ERR}  FAIL${C_0} $*" >&2; }
step() { echo; echo "${C_H}== $*${C_0}"; }
run()  { if [ "$DRY_RUN" = 1 ]; then echo "  [dry-run] $*"; else eval "$@"; fi; }

need_cmd() { command -v "$1" >/dev/null 2>&1 || { err "missing command: $1"; return 1; }; }

apt_installed() { dpkg -l "$1" 2>/dev/null | grep -q "^ii"; }

# apt_lists_refresh -- `apt-get update` once if the package lists are empty (a fresh
# container) so that `apt-get install -s` can resolve anything at all. Refreshing
# lists installs nothing, but a dry run still only announces it.
apt_lists_refresh() {
  if ls /var/lib/apt/lists/*Packages* >/dev/null 2>&1; then return 0; fi
  warn "apt package lists are empty (fresh container) -- refreshing with apt-get update"
  if [ "$DRY_RUN" = 1 ]; then
    echo "  [dry-run] sudo apt-get update   (until then apt simulations below cannot resolve packages)"
    return 0
  fi
  sudo apt-get update -qq
}

# ---------------------------------------------------------------------------
# apt_guarded_install <pkg...>
#
# Simulates the install, prints the impact, and REFUSES if existing packages
# would be upgraded/removed (override with ALLOW_UPGRADES=1). This is the
# habit SETUP.md 2-B-2 describes, enforced in code.
# ---------------------------------------------------------------------------
apt_guarded_install() {
  local pkgs=("$@") missing=() sim summary upgrades removes
  for p in "${pkgs[@]}"; do apt_installed "$p" || missing+=("$p"); done
  if [ ${#missing[@]} -eq 0 ]; then ok "already installed: ${pkgs[*]}"; return 0; fi

  sim="$(apt-get install -s "${missing[@]}" 2>&1)" || { err "apt simulation failed:"; echo "$sim" | tail -20; return 1; }
  summary="$(grep -m1 'upgraded,' <<<"$sim" || true)"
  # Upgrade lines look like:  Inst dpkg [1.22.6ubuntu6.5] (1.22.6ubuntu6.7 Ubuntu:24.04/noble-updates [amd64])
  # Keep "<pkg> [<old>] (<new> <origin...>)" so the ORIGIN of every upgrade is visible.
  # apt may append " []" (auto-installed marker) and an " [amd64]" arch tag; strip both.
  upgrades="$(grep '^Inst' <<<"$sim" | grep -E '\[[0-9]' \
    | sed -E 's/^Inst //; s/ \[\]$//; s/ \[[a-z0-9]+\]\)$/)/' || true)"
  removes="$(grep '^Remv' <<<"$sim" | sed 's/ (.*//;s/^Remv //' || true)"

  echo "  target : ${missing[*]}"
  echo "  impact : ${summary:-<none>}"
  if [ -n "$upgrades" ] || [ -n "$removes" ]; then
    [ -n "$upgrades" ] && { warn "would UPGRADE existing packages:"; sed 's/^/           /' <<<"$upgrades"; }
    [ -n "$removes" ]  && { warn "would REMOVE packages:";            sed 's/^/           /' <<<"$removes"; }
    case "$ALLOW_UPGRADES" in
      1) warn "ALLOW_UPGRADES=1 -- proceeding despite the above" ;;
      ubuntu)
        # A FRESH container ships stale Ubuntu base libs; ros-jazzy-desktop / moveit
        # then need their noble-updates/-security point releases (measured 2026-09-16:
        # 14 + 4 packages, util-linux/systemd/ncurses). Those come from Ubuntu's own
        # repos and are the same thing `apt upgrade` would do. Accept ONLY those:
        # anything from another origin (NVIDIA, ROS shadowing, ...) or any removal
        # still stops here.
        local foreign
        foreign="$(grep -vE '\((\S+ )?(Ubuntu:[^ ,)]+(, )?)+\)$' <<<"$upgrades" || true)"
        if [ -n "$removes" ] || [ -n "$foreign" ]; then
          err "refusing: ALLOW_UPGRADES=ubuntu accepts Ubuntu-origin upgrades only, but this has:"
          [ -n "$foreign" ] && sed 's/^/           /' <<<"$foreign"
          [ -n "$removes" ] && err "           removals: $removes"
          return 1
        fi
        warn "ALLOW_UPGRADES=ubuntu -- every upgrade above comes from Ubuntu's own repos; proceeding" ;;
      *)
        err "refusing: this would change packages other workspaces may depend on."
        err "          Inspect the list above. A container made just for this workspace:"
        err "            ALLOW_UPGRADES=ubuntu $0 ...   (accept Ubuntu security/updates only -- bootstrap.sh --fresh)"
        err "          Anything else: ALLOW_UPGRADES=1 $0 ...   (accept everything listed)"
        return 1 ;;
    esac
  else
    ok "pure addition (0 upgraded, 0 removed)"
  fi
  # `sudo` drops the caller's environment, so DEBIAN_FRONTEND must be passed through
  # `env`: without it a package's debconf prompt (tzdata, keyboard-configuration, ...)
  # can block a non-interactive run forever on a fresh image.
  run "sudo env DEBIAN_FRONTEND=noninteractive apt-get install -y -q ${missing[*]}"
}

# GPU compute capability, e.g. "120" for sm_120 (RTX 5090). Empty if no nvidia-smi.
gpu_sm() {
  command -v nvidia-smi >/dev/null 2>&1 || return 0
  nvidia-smi --query-gpu=compute_cap --format=csv,noheader 2>/dev/null \
    | head -1 | tr -d ' .' || true
}

# Architectures baked into a CUDA binary, e.g. "sm_75" or "sm_120 sm_75 sm_86 sm_89".
cuda_archs_of() {
  local f="$1" cuobj
  cuobj="$(ls /usr/local/cuda*/bin/cuobjdump 2>/dev/null | head -1 || true)"
  [ -n "$cuobj" ] && [ -f "$f" ] || return 0
  "$cuobj" --list-elf "$f" 2>/dev/null | grep -oE 'sm_[0-9]+' | sort -u | tr '\n' ' '
}
