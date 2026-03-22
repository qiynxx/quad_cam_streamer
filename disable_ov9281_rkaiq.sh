#!/bin/bash
# Disable OV9281 from rkaiq_3A_server by renaming its IQ files
# This allows rkaiq_3A_server to only manage IMX334 cameras

IQ_DIR="/etc/iqfiles"

echo "Disabling OV9281 from rkaiq_3A_server..."

# Rename OV9281 IQ files to prevent rkaiq from loading them
if [ -f "$IQ_DIR/ov9281_NOIQ_NOIQ.json" ]; then
    mv "$IQ_DIR/ov9281_NOIQ_NOIQ.json" "$IQ_DIR/ov9281_NOIQ_NOIQ.json.disabled"
    echo "Renamed ov9281_NOIQ_NOIQ.json -> ov9281_NOIQ_NOIQ.json.disabled"
fi

# Check for any other OV9281 IQ files
for f in "$IQ_DIR"/ov9281*.json; do
    if [ -f "$f" ]; then
        mv "$f" "$f.disabled"
        echo "Renamed $(basename $f) -> $(basename $f).disabled"
    fi
done

echo "Done. Now rkaiq_3A_server will only manage IMX334 cameras."
echo ""
echo "To start the system:"
echo "1. rkaiq_3A_server --silent &"
echo "2. quad_cam_streamer /etc/quad_cam_streamer/config.json"
