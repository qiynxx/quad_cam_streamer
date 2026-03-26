#!/bin/bash
#
# Local cross-compile script for quad_cam_streamer
#
# Builds directly in ./build/ using the Buildroot toolchain,
# and copies the executable to the current directory.
#
# Usage:
#   ./build_local.sh            # Incremental build
#   ./build_local.sh clean      # Clean and rebuild
#   ./build_local.sh debug      # Build with debug symbols
#   ./build_local.sh push       # Build + adb push to device
#

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
SDK_DIR="$(cd "$SCRIPT_DIR/../.." && pwd)"
BR_OUTPUT="$SDK_DIR/buildroot/output/rockchip_rk3576"
HOST_DIR="$BR_OUTPUT/host"
SYSROOT="$HOST_DIR/aarch64-buildroot-linux-gnu/sysroot"
TOOLCHAIN_FILE="$HOST_DIR/share/buildroot/toolchainfile.cmake"

BUILD_DIR="$SCRIPT_DIR/build"
OUTPUT_BIN="$SCRIPT_DIR/quad_cam_streamer"
BUILD_TYPE="Release"

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

log()  { echo -e "${GREEN}[BUILD]${NC} $*"; }
warn() { echo -e "${YELLOW}[WARN]${NC} $*"; }
err()  { echo -e "${RED}[ERROR]${NC} $*"; exit 1; }

check_env() {
    [ -f "$TOOLCHAIN_FILE" ] || err "Toolchain file not found: $TOOLCHAIN_FILE\nRun SDK build first to initialize the Buildroot environment."
    [ -d "$SYSROOT" ] || err "Sysroot not found: $SYSROOT"
}

do_configure() {
    log "Configuring (CMAKE_BUILD_TYPE=$BUILD_TYPE)..."
    cmake -S "$SCRIPT_DIR" -B "$BUILD_DIR" \
        -DCMAKE_TOOLCHAIN_FILE="$TOOLCHAIN_FILE" \
        -DCMAKE_BUILD_TYPE="$BUILD_TYPE" \
        -DCMAKE_EXPORT_COMPILE_COMMANDS=ON
}

do_build() {
    check_env

    if [ ! -f "$BUILD_DIR/CMakeCache.txt" ]; then
        do_configure
    fi

    local jobs
    jobs=$(nproc 2>/dev/null || echo 4)

    log "Building with $jobs jobs..."
    cmake --build "$BUILD_DIR" -j "$jobs"

    cp "$BUILD_DIR/quad_cam_streamer" "$OUTPUT_BIN"

    local size
    size=$(du -h "$OUTPUT_BIN" | cut -f1)
    log "Output: $OUTPUT_BIN ($size)"
    file "$OUTPUT_BIN" | sed "s|$SCRIPT_DIR/||"
}

do_clean() {
    log "Cleaning build directory..."
    rm -rf "$BUILD_DIR"
    rm -f "$OUTPUT_BIN"
    log "Clean complete."
}

do_push() {
    do_build
    log "Pushing to device via adb..."
    adb push "$OUTPUT_BIN" /usr/bin/quad_cam_streamer
    adb push "$SCRIPT_DIR/config.json" /etc/quad_cam_streamer/config.json
    log "Push complete. Restart the service on device:"
    echo "  /etc/init.d/S99quad_cam_streamer restart"
}

case "${1:-build}" in
    build|"")
        do_build
        ;;
    debug)
        BUILD_TYPE="Debug"
        rm -rf "$BUILD_DIR"
        do_build
        ;;
    clean)
        do_clean
        ;;
    rebuild)
        do_clean
        do_build
        ;;
    push)
        do_push
        ;;
    configure)
        check_env
        do_configure
        ;;
    *)
        echo "Usage: $0 {build|debug|clean|rebuild|push|configure}"
        echo ""
        echo "  build     - Incremental build (default)"
        echo "  debug     - Clean + build with debug symbols"
        echo "  clean     - Remove build artifacts"
        echo "  rebuild   - Clean + build"
        echo "  push      - Build + adb push to device"
        echo "  configure - Run CMake configure only"
        exit 1
        ;;
esac
