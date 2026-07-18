# SPDX-License-Identifier: Apache-2.0
"""Local-first Isaac asset resolver — for reproducibility on other machines.

Problem: our scene/build scripts pulled the UR16e (and Robotiq, room) USD from the
live Isaac assets server (get_assets_root_path()) or a hardcoded S3 URL. A fresh
checkout on another PC then depends on network / a matching Isaac asset mount.

Fix: keep a VENDORED copy inside ur_bringup and use it FIRST; only fall back to the
network/Isaac root if the local copy is absent. So the repo is self-contained where
we've vendored, and still works if a vendored copy is missing.

Layout — mirror the Isaac-relative path under the vendor dir:
    ur_bringup/isaac/assets/vendor/Isaac/Robots/UniversalRobots/ur16e/ur16e.usd
so `resolve_asset("Isaac/Robots/UniversalRobots/ur16e/ur16e.usd")` finds it. USD
sub-references inside a vendored asset must be RELATIVE (the UR16e asset is: its
ur16e.usd references configuration/*.usd), so copying the folder keeps it intact.

Usage:
    from asset_paths import resolve_asset
    url, src = resolve_asset("Isaac/Robots/UniversalRobots/ur16e/ur16e.usd", log=info)
    # src in {"local","url","isaac","none"}; url is None only when src=="none"
"""
from __future__ import annotations

from pathlib import Path

# this file = ur_bringup/isaac/common/asset_paths.py  ->  vendor = ur_bringup/isaac/assets/vendor
VENDOR_DIR = Path(__file__).resolve().parent.parent / "assets" / "vendor"


def vendor_path(rel_path):
    """Absolute path a vendored copy of `rel_path` WOULD live at (may not exist)."""
    return VENDOR_DIR / str(rel_path).lstrip("/")


def resolve_asset(rel_path, fallback_url=None, log=None):
    """Resolve an Isaac-relative asset path, LOCAL (vendored) copy first.

    rel_path     : Isaac-relative path, e.g. "Isaac/Robots/.../ur16e.usd" (leading "/" ok).
    fallback_url : explicit URL/path to use when there is no vendored copy, BEFORE
                   consulting the live Isaac assets root. Pass the exact URL a script
                   used before so its no-local behavior is unchanged.
    log          : optional callable(str) for a one-line note about the source chosen.

    Returns (resolved, source) with source in {"local","url","isaac","none"}.
    `resolved` is None only when source == "none" (nothing found and no Isaac root).
    """
    rel = str(rel_path).lstrip("/")
    local = VENDOR_DIR / rel
    if local.exists():
        if log:
            log(f"asset LOCAL (vendored): {local}")
        return str(local), "local"
    if fallback_url:
        if log:
            log(f"asset URL (no vendored copy): {fallback_url}")
        return fallback_url, "url"
    # last resort: the live Isaac assets root (network / mounted assets)
    from isaacsim.storage.native import get_assets_root_path  # lazy: only needs Isaac here
    root = get_assets_root_path()
    if root is None:
        return None, "none"
    url = root.rstrip("/") + "/" + rel
    if log:
        log(f"asset Isaac-root (no vendored copy): {url}")
    return url, "isaac"
