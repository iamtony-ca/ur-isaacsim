#!/usr/bin/env bash
# Give a STOCK NVIDIA Isaac Sim 6.1.0 container what bootstrap.sh / setup.sh assume.
# Run it ONCE as ROOT, from the host:
#
#   docker exec -u 0 -it <container> bash /isaac-sim/volume/ur_ws/src/setup/container_prereqs.sh --sudo-user isaac-sim
#   docker exec -u 0 -it <container> bash .../container_prereqs.sh --dry-run     # look first
#
# WHY: the stock image runs as a non-root user (isaac-sim, uid 1234) and ships no sudo,
# no git, no gpg (gpgv only) and no locales -- so preflight stops at once and the repos
# stage cannot dearmor the NVIDIA keys. The container this workspace was verified in got
# them from an extra root layer that was never written down (2026-09-16 13:23,
# /var/log/apt/history.log) plus /etc/sudoers.d/isaac-sim. This script IS that layer:
# the same package list, the same NOPASSWD rule.
#
#   --sudo-user NAME   also write /etc/sudoers.d/NAME = "NAME ALL=(ALL) NOPASSWD:ALL"
#                      (0440, checked with visudo -cf before it is put in place). The
#                      setup stages run apt non-interactively, so a password prompt = stuck.
#   --dry-run          print what would happen, change nothing (works without root)
#
# Idempotent: packages already installed are skipped (apt-get update only runs when
# something is missing), an identical sudoers file is left alone.
#
# DEDICATED CONTAINER ONLY: installing the missing packages also UPGRADES some base
# packages already in the image (libc6, mesa, curl, ... ~10 on stock 6.1.0, 2026-10-21;
# all from Ubuntu's own repos). The apt simulation is printed before anything is
# installed (and by --dry-run once the package lists exist); removals are refused.
#
# The mesa/vulkan/xkb/xauth half is for the Isaac GUI / X forwarding; headless runs do not
# need it, but it was in the verified image, so it is here too.
set -euo pipefail

PKGS=(sudo curl gnupg2 lsb-release locales acl ca-certificates git vim nano less tree
      mesa-vulkan-drivers vulkan-tools libgl1-mesa-dri mesa-utils libxkbcommon-x11-0 xauth)

DRY=0; SUDO_USER_NAME=""
while [ $# -gt 0 ]; do
  case "$1" in
    --dry-run)   DRY=1 ;;
    --sudo-user) [ $# -ge 2 ] || { echo "--sudo-user needs a NAME" >&2; exit 2; }
                 SUDO_USER_NAME="$2"; shift ;;
    --sudo-user=*) SUDO_USER_NAME="${1#*=}" ;;
    -h|--help)   awk 'NR>1 && !/^#/{exit} NR>1{sub(/^# ?/,""); print}' "$0"; exit 0 ;;
    *) echo "unknown option: $1 (see --help)" >&2; exit 2 ;;
  esac
  shift
done

SELF="$(cd "$(dirname "$0")" && pwd)/$(basename "$0")"
say()  { echo "  $*"; }
fail() { echo "  FAIL $*" >&2; exit 1; }

# Validate --sudo-user BEFORE installing anything: a typo should not cost a full
# apt run that then ends without the sudoers rule.
if [ -n "$SUDO_USER_NAME" ]; then
  # sudoers.d silently ignores file names containing '.' or ending in '~'.
  [[ "$SUDO_USER_NAME" =~ ^[a-z_][a-z0-9_-]*$ ]] || fail "--sudo-user '$SUDO_USER_NAME': not a plain user name"
  id -u "$SUDO_USER_NAME" >/dev/null 2>&1 || fail "user '$SUDO_USER_NAME' does not exist in this container (id -un as that user shows the name)"
fi

if [ "$(id -u)" != 0 ]; then
  if [ "$DRY" = 1 ]; then
    say "(not root: dry run only -- the real run needs: docker exec -u 0 ...)"
  else
    fail "must run as root, e.g. from the host:  docker exec -u 0 -it <container> bash $SELF ${SUDO_USER_NAME:+--sudo-user $SUDO_USER_NAME}"
  fi
fi

echo "== container prerequisites (stock Isaac Sim image -> what setup/ assumes)"

# --- packages -------------------------------------------------------------
# This is the ONE apt step here that is not guarded like setup.sh's: it runs as root on a
# container made FOR this workspace, and resolving the missing packages against today's
# Ubuntu archive UPGRADES already-installed base packages of the image (measured
# 2026-10-21 on stock 6.1.0: ~10, e.g. libc6, mesa, curl -- all from Ubuntu's own repos).
# It does not stop for that (it must run unattended), but it SIMULATES first and prints
# exactly what will be installed and upgraded, and it refuses removals (ALLOW_UPGRADES=1
# overrides). Same parsers as setup.sh's apt guard (lib.sh).
# shellcheck source=lib.sh
source "$(dirname "$SELF")/lib.sh" || fail "lib.sh missing next to $SELF"
apt_impact() {   # prints the simulated impact of installing "$@"; non-zero = refuse
  local sim upgrades removes foreign nnew
  sim="$(apt-get install -s --no-install-recommends "$@" 2>&1)" || {
    echo "$sim" | tail -20 | sed 's/^/    /'; say "FAIL apt simulation failed (see above)"; return 1; }
  upgrades="$(apt_sim_upgrades <<<"$sim")"
  removes="$(apt_sim_removes <<<"$sim")"
  foreign="$(apt_foreign_upgrades <<<"$upgrades")"
  nnew="$(grep -E '^Inst [^ ]+ \(' <<<"$sim" | grep -c . || true)"
  say "apt simulation: $(grep -m1 'upgraded,' <<<"$sim" || echo '<no summary>')"
  say "  new packages: $nnew (requested + their dependencies)"
  if [ -n "$upgrades" ]; then
    say "  NOTE these ALREADY-INSTALLED base packages of the image get UPGRADED (dedicated container: expected):"
    sed 's/^/           /' <<<"$upgrades"
    [ -n "$foreign" ] && say "  WARN some of those upgrades do NOT come from Ubuntu's own repos (see origins above)"
  else
    say "  upgrades of installed packages: none"
  fi
  if [ -n "$removes" ]; then
    say "  apt would REMOVE: $(tr '\n' ' ' <<<"$removes")"
    [ "${ALLOW_UPGRADES:-0}" = 1 ] && { say "  ALLOW_UPGRADES=1 -- proceeding despite removals"; return 0; }
    say "FAIL refusing: a removal is never expected on a stock image (inspect; ALLOW_UPGRADES=1 to force)"
    return 1
  fi
}
missing=()
for p in "${PKGS[@]}"; do
  dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q "install ok installed" || missing+=("$p")
done
if [ ${#missing[@]} -eq 0 ]; then
  say "ok   all ${#PKGS[@]} packages already installed"
else
  say "missing: ${missing[*]}"
  if [ "$DRY" = 1 ]; then
    # A stock image has never run `apt-get update`: until then apt cannot resolve anything,
    # and a dry run does not refresh the lists (that is a change, and needs root).
    if ls /var/lib/apt/lists/*Packages* >/dev/null 2>&1; then
      apt_impact "${missing[@]}" || say "(dry run: the real run stops at the refusal above)"
      say "[dry-run] apt-get update; apt-get install -y --no-install-recommends ${missing[*]}"
    else
      say "[dry-run] apt-get update   (package lists are empty: the impact -- new packages AND upgrades"
      say "          of installed base packages -- is printed by the real run before it installs)"
      say "[dry-run] apt-get install -y --no-install-recommends ${missing[*]}"
    fi
  else
    export DEBIAN_FRONTEND=noninteractive
    apt-get update -qq || fail "apt-get update failed (network/proxy?)"
    apt_impact "${missing[@]}" || fail "apt impact check refused (see above)"
    apt-get install -y -q --no-install-recommends "${missing[@]}" || fail "apt-get install failed"
    say "ok   installed: ${missing[*]}"
  fi
fi

# --- passwordless sudo for the container user -------------------------------
if [ -n "$SUDO_USER_NAME" ]; then
  dst="/etc/sudoers.d/$SUDO_USER_NAME"
  rule="$SUDO_USER_NAME ALL=(ALL) NOPASSWD:ALL"
  if [ -f "$dst" ] && [ "$(cat "$dst" 2>/dev/null)" = "$rule" ]; then
    say "ok   $dst already grants NOPASSWD sudo"
  elif [ "$DRY" = 1 ]; then
    # 0440 root: a non-root dry run cannot read an existing file to compare it.
    if [ -e "$dst" ] && [ ! -r "$dst" ]; then
      say "[dry-run] $dst exists (unreadable as non-root): left as is if it already equals \"$rule\", else rewritten"
    else say "[dry-run] write $dst (0440): $rule   (visudo -cf checked)"; fi
  else
    command -v visudo >/dev/null 2>&1 || fail "visudo missing -- the sudo package did not install?"
    tmp="$(mktemp)"
    echo "$rule" > "$tmp"
    visudo -cf "$tmp" >/dev/null || { rm -f "$tmp"; fail "visudo rejected the rule: $rule"; }
    install -m 0440 -o root -g root "$tmp" "$dst" || { rm -f "$tmp"; fail "could not write $dst"; }
    rm -f "$tmp"
    say "ok   wrote $dst: $rule"
  fi
else
  say "note no --sudo-user given: sudoers left as is (setup.sh needs NOPASSWD sudo for the container user)"
fi

echo
if [ "$DRY" = 1 ]; then
  echo "dry run: nothing changed."
else
  echo "done. Next, as the container user:  $(dirname "$SELF")/bootstrap.sh --dry-run"
fi
