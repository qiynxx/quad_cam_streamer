#!/bin/bash
# Start rkaiq_3A_server for IMX334 cameras
# This server must run BEFORE starting quad_cam_streamer

IQ_DIR="/etc/iqfiles"

# Camera 0: IMX334 on rkisp_vir0
rkaiq_3A_server -a /dev/video53 -d /dev/video53 -i "$IQ_DIR" &

echo "rkaiq_3A_server started for cam0 (IMX334)"
echo "Wait 2 seconds for initialization..."
sleep 2
echo "Ready. Now you can start quad_cam_streamer"
