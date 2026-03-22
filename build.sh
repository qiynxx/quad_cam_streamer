#!/bin/bash
#
# One-click build script for quad_cam_streamer
#
# Usage:
#   ./build.sh              # Rebuild after code changes
#   ./build.sh init         # First-time build (compile dependencies)
#   ./build.sh clean        # Clean build artifacts
#   ./build.sh rootfs       # Rebuild + regenerate rootfs image
#   ./build.sh all          # Rebuild rkaiq + quad_cam_streamer
#

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SDK_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
BR_OUTPUT="$SDK_DIR/buildroot/output/rockchip_rk3576"
PKG_NAME="quad-cam-streamer"
RKAIQ_PKG="camera-engine-rkaiq"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log()  { echo -e "${GREEN}[BUILD]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
err()  { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

# Check buildroot output exists
check_env() {
    if [ ! -f "$BR_OUTPUT/.config" ]; then
        err "Buildroot output not found at $BR_OUTPUT\n       Run './build.sh' from SDK root first to initialize the build environment."
    fi
}

# First-time: ensure dependencies are built and package is enabled
enable_pkgs() {
    local changed=0
    for pkg in BR2_PACKAGE_JSON_FOR_MODERN_CPP BR2_PACKAGE_ZEROMQ BR2_PACKAGE_CPPZMQ BR2_PACKAGE_QUAD_CAM_STREAMER BR2_PACKAGE_CAMERA_ENGINE_RKAIQ; do
        if ! grep -q "^${pkg}=y" .config 2>/dev/null; then
            if grep -q "# ${pkg} is not set" .config; then
                sed -i "s/# ${pkg} is not set/${pkg}=y/" .config
            else
                echo "${pkg}=y" >> .config
            fi
            log "Enabled $pkg"
            changed=1
        fi
    done
    if [ "$changed" -eq 1 ]; then
        make olddefconfig
    fi
}

do_init() {
    check_env
    log "Initializing: enabling packages and building dependencies..."

    cd "$BR_OUTPUT"

    enable_pkgs

    # Build dependencies
    log "Building json-for-modern-cpp..."
    make json-for-modern-cpp

    log "Building zeromq..."
    make zeromq

    log "Building cppzmq..."
    make cppzmq

    log "Building $RKAIQ_PKG..."
    make "$RKAIQ_PKG"

    # Build the package
    log "Building $PKG_NAME..."
    make "$PKG_NAME"

    log "Init complete!"
    show_result
}

# Rebuild after code changes
do_rebuild() {
    check_env
    cd "$BR_OUTPUT"

    # Check if dependencies are built
    if [ ! -d "$BR_OUTPUT/build/cppzmq-"* ] 2>/dev/null; then
        warn "Dependencies not built yet. Running init first..."
        do_init
        return
    fi

    log "Rebuilding $PKG_NAME..."
    make "${PKG_NAME}-rebuild"

    log "Rebuild complete!"
    show_result
}

do_all() {
    check_env
    cd "$BR_OUTPUT"

    enable_pkgs

    if [ ! -d "$BR_OUTPUT/build/cppzmq-"* ] 2>/dev/null; then
        warn "Dependencies not built yet. Running init first..."
        do_init
        return
    fi

    log "Rebuilding $RKAIQ_PKG..."
    make "${RKAIQ_PKG}-rebuild"

    log "Rebuilding $PKG_NAME..."
    make "${PKG_NAME}-rebuild"

    log "All rebuilds complete!"
    show_result
}

# Clean build artifacts
do_clean() {
    check_env
    cd "$BR_OUTPUT"

    log "Cleaning $PKG_NAME..."
    make "${PKG_NAME}-dirclean" 2>/dev/null || true

    log "Clean complete. Run './build.sh' to rebuild."
}

# Rebuild + regenerate rootfs
do_rootfs() {
    do_rebuild
    log "Regenerating rootfs image..."
    cd "$SDK_DIR"
    ./build.sh rootfs
    log "Rootfs image updated!"
}

show_result() {
    local bin="$BR_OUTPUT/target/usr/bin/quad_cam_streamer"
    if [ -f "$bin" ]; then
        log "Output: $bin ($(du -h "$bin" | cut -f1))"
        log "Config: $BR_OUTPUT/target/etc/quad_cam_streamer/config.json"
        log "Init:   $BR_OUTPUT/target/etc/init.d/S99quad_cam_streamer"
        if [ -f "$BR_OUTPUT/target/etc/init.d/S40rkaiq_3A" ]; then
            log "RKAIQ:  $BR_OUTPUT/target/etc/init.d/S40rkaiq_3A"
        fi
    fi
}

case "${1:-rebuild}" in
    init)
        do_init
        ;;
    rebuild|"")
        do_rebuild
        ;;
    all)
        do_all
        ;;
    clean)
        do_clean
        ;;
    rootfs)
        do_rootfs
        ;;
    *)
        echo "Usage: $0 {init|rebuild|all|clean|rootfs}"
        echo ""
        echo "  init    - First-time build: enable packages, build dependencies"
        echo "  rebuild - Rebuild after code changes (default)"
        echo "  all     - Rebuild rkaiq + quad_cam_streamer"
        echo "  clean   - Clean build artifacts"
        echo "  rootfs  - Rebuild + regenerate rootfs image"
        exit 1
        ;;
esac
