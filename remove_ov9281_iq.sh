#!/bin/bash
# Remove OV9281 IQ files from fs-overlay to prevent rkaiq_3A_server from loading them

FS_OVERLAY="/home/zengqy/workspace/rk3576_purple_pi_oh2_linux_sdk/buildroot/board/rockchip/rk3576/fs-overlay/etc/iqfiles"

echo "Removing OV9281 IQ files from fs-overlay..."

cd "$FS_OVERLAY" || exit 1

# Remove OV9281 IQ files
rm -f ov9281_NOIQ_NOIQ.json
rm -f ov9281_DEFAULT_DEFAULT.json

echo "Done. OV9281 IQ files removed."
echo ""
echo "Now rebuild the firmware:"
echo "  cd /home/zengqy/workspace/rk3576_purple_pi_oh2_linux_sdk"
echo "  ./build.sh buildroot"
echo ""
echo "After flashing, rkaiq_3A_server will only manage IMX334."
