#!/usr/bin/env bash
# patch_arena_sdk.sh
#
# Patches a Lucid ArenaSDK installation at ARENA_ROOT so it links and runs
# correctly on Ubuntu 24.04 (and other distros that ship ICU 74, libtiff 6,
# OpenLDAP 2.x instead of the old ABIs the Metavision bundle expects).
#
# Usage:
#   sudo ./patch_arena_sdk.sh [ARENA_ROOT]
#
# ARENA_ROOT defaults to /opt/lucid/ArenaSDK if not supplied.
#
# What this script does:
#   1. Installs patchelf if missing.
#   2. Fixes file permissions on the root-only Metavision OpenCV stubs.
#   3. Removes the Metavision DT_NEEDED entries from libarena.so and libgentl.so
#      (event cameras are not supported on this OS; RGB/depth cameras are fine).
#   4. Rewrites /etc/ld.so.conf.d/Arena_SDK.conf to expose only the two safe
#      lib directories (lib64 + GenICam), then refreshes the linker cache.
#
# Idempotent: safe to run more than once.

set -euo pipefail

# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------
RED='\033[0;31m'; GREEN='\033[0;32m'; YELLOW='\033[1;33m'; NC='\033[0m'
info()  { echo -e "${GREEN}[INFO]${NC}  $*"; }
warn()  { echo -e "${YELLOW}[WARN]${NC}  $*"; }
error() { echo -e "${RED}[ERROR]${NC} $*" >&2; exit 1; }

require_root() {
    [ "$(id -u)" -eq 0 ] || error "This script must be run as root (sudo)."
}

# ---------------------------------------------------------------------------
# Arguments
# ---------------------------------------------------------------------------
ARENA_ROOT="${1:-/opt/lucid/ArenaSDK}"

require_root

[ -d "$ARENA_ROOT" ] || error "ArenaSDK directory not found: $ARENA_ROOT"
info "ArenaSDK root: $ARENA_ROOT"

LIB64="$ARENA_ROOT/lib64"
GENICAM_LIB="$ARENA_ROOT/GenICam/library/lib/Linux64_x64"
METAVISION_LIB="$ARENA_ROOT/Metavision/lib"

[ -d "$LIB64" ]       || error "Expected lib64 directory missing: $LIB64"
[ -d "$GENICAM_LIB" ] || error "Expected GenICam lib directory missing: $GENICAM_LIB"

# ---------------------------------------------------------------------------
# Step 1 – install patchelf
# ---------------------------------------------------------------------------
info "Step 1: checking for patchelf..."
if ! command -v patchelf &>/dev/null; then
    info "  patchelf not found, installing via apt..."
    apt-get install -y patchelf
else
    info "  patchelf already installed ($(patchelf --version 2>&1 | head -1))"
fi

# ---------------------------------------------------------------------------
# Step 2 – fix permissions on Metavision OpenCV stub files (rwx------ → 755)
# ---------------------------------------------------------------------------
info "Step 2: fixing Metavision OpenCV stub permissions..."
if [ -d "$METAVISION_LIB" ]; then
    OPENCV_STUBS=(
        libopencv_core.so
        libopencv_highgui.so
        libopencv_imgcodecs.so
        libopencv_imgproc.so
        libopencv_videoio.so
    )
    for stub in "${OPENCV_STUBS[@]}"; do
        path="$METAVISION_LIB/$stub"
        if [ -f "$path" ]; then
            current=$(stat -c '%a' "$path")
            if [ "$current" != "755" ]; then
                chmod 755 "$path"
                info "  chmod 755 $path  (was $current)"
            else
                info "  $stub already 755, skipping"
            fi
        fi
    done
else
    warn "  Metavision/lib not found, skipping permission fix."
fi

# ---------------------------------------------------------------------------
# Step 3 – strip Metavision DT_NEEDED from libarena.so and libgentl.so
# ---------------------------------------------------------------------------
info "Step 3: removing Metavision DT_NEEDED entries from Arena libraries..."

patch_library() {
    local lib="$1"
    local needs=("libmetavision_sdk_core.so.4" "libmetavision_sdk_base.so.4")

    if [ ! -f "$lib" ]; then
        warn "  $lib not found, skipping."
        return
    fi

    # Back up once (don't overwrite an existing backup)
    local bak="${lib}.bak"
    if [ ! -f "$bak" ]; then
        cp "$lib" "$bak"
        info "  Backed up $lib → $bak"
    else
        info "  Backup already exists: $bak"
    fi

    local patched=0
    for need in "${needs[@]}"; do
        if patchelf --print-needed "$lib" 2>/dev/null | grep -q "^${need}$"; then
            patchelf --remove-needed "$need" "$lib"
            info "  Removed NEEDED $need from $(basename "$lib")"
            patched=1
        fi
    done

    if [ "$patched" -eq 0 ]; then
        info "  $(basename "$lib") already clean, nothing to remove."
    fi
}

patch_library "$LIB64/libarena.so"
patch_library "$LIB64/libgentl.so"

# ---------------------------------------------------------------------------
# Step 4 – rewrite Arena_SDK.conf with only the two safe paths
# ---------------------------------------------------------------------------
info "Step 4: updating /etc/ld.so.conf.d/Arena_SDK.conf..."
CONF_FILE="/etc/ld.so.conf.d/Arena_SDK.conf"

SAFE_CONF="$LIB64
$GENICAM_LIB"

if [ -f "$CONF_FILE" ] && [ "$(cat "$CONF_FILE")" = "$SAFE_CONF" ]; then
    info "  Arena_SDK.conf already correct, skipping."
else
    # Back up existing conf if it differs
    if [ -f "$CONF_FILE" ]; then
        cp "$CONF_FILE" "${CONF_FILE}.bak"
        info "  Backed up $CONF_FILE → ${CONF_FILE}.bak"
    fi
    printf '%s\n' "$SAFE_CONF" > "$CONF_FILE"
    info "  Wrote $CONF_FILE:"
    sed 's/^/    /' "$CONF_FILE"
fi

info "  Running ldconfig..."
ldconfig
info "  ldconfig done."

# ---------------------------------------------------------------------------
# Done
# ---------------------------------------------------------------------------
echo ""
info "All patches applied successfully."
info "Verify with:"
info "  patchelf --print-needed $LIB64/libarena.so"
info "  patchelf --print-needed $LIB64/libgentl.so"
info "  ldd <your_binary> | grep 'not found'"
