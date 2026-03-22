# Quad Camera MJPEG Streamer

Captures 4 V4L2 cameras simultaneously on the Purple Pi OH2 (RK3576), JPEG-encodes each frame using Rockchip MPP hardware encoder, and streams via ZMQ.

## Architecture

```
V4L2 capture → MPP JPEG encode → ZMQ PUB (per-camera port)
```

Each camera runs in its own thread. Each gets a dedicated ZMQ PUB port:
- cam0: `tcp://*:5550`
- cam1: `tcp://*:5551`
- cam2: `tcp://*:5552`
- cam3: `tcp://*:5553`

## Project Structure

```
app/quad_cam_streamer/
├── build.sh                       # 一键编译脚本
├── CMakeLists.txt                 # CMake 构建配置
├── config.json                    # 默认配置文件
├── S99quad_cam_streamer           # SysV 开机自启脚本
├── src/                           # C++ 源码
│   ├── main.cpp                   # 入口：信号处理、线程管理
│   ├── config.h / config.cpp      # JSON 配置加载
│   ├── v4l2_camera.h / .cpp       # V4L2 摄像头采集 (MMAP)
│   ├── mpp_encoder.h / .cpp       # Rockchip MPP JPEG 硬件编码
│   └── zmq_streamer.h / .cpp      # ZMQ PUB 推流
├── tools/                         # Python 查看工具
│   ├── zmq_viewer.py              # ZMQ MJPEG 接收显示
│   └── requirements.txt           # Python 依赖
└── README.md
```

## Prerequisites

- Board: Purple Pi OH2 with 4 cameras connected (2x IMX334 + 2x OV9281)
- Camera pipeline configured in device tree
- rkaiq_3A_server running (for auto-exposure)

## Build

### 一键编译脚本（推荐）

项目提供 `build.sh` 一键编译脚本，位于 `app/quad_cam_streamer/build.sh`。

**首次编译**（自动启用依赖包并编译）：

```bash
cd app/quad_cam_streamer
./build.sh init
```

`init` 会依次完成：
1. 在 buildroot `.config` 中启用 `zeromq`、`cppzmq`、`json-for-modern-cpp`、`quad-cam-streamer`
2. 运行 `make olddefconfig` 解决依赖
3. 编译所有依赖库
4. 编译 `quad_cam_streamer`

**修改代码后重新编译**：

```bash
cd app/quad_cam_streamer
./build.sh             # 或 ./build.sh rebuild
```

**编译并打包到 rootfs 镜像**：

```bash
cd app/quad_cam_streamer
./build.sh rootfs
```

**清理构建产物**：

```bash
cd app/quad_cam_streamer
./build.sh clean
```

### 编译产物

编译成功后，以下文件会安装到 buildroot target rootfs 中：

| 文件 | rootfs 路径 | 说明 |
|------|------------|------|
| `quad_cam_streamer` | `/usr/bin/` | 主程序 (aarch64 ELF) |
| `config.json` | `/etc/quad_cam_streamer/` | 默认配置文件 |
| `S99quad_cam_streamer` | `/etc/init.d/` | 开机自启脚本 |

### 手动编译（底层命令）

如果不使用一键脚本，也可以手动操作：

```bash
cd buildroot/output/rockchip_rk3576

# 首次：编译依赖
make json-for-modern-cpp
make zeromq
make cppzmq

# 首次：编译 quad-cam-streamer
make quad-cam-streamer

# 修改代码后重新编译
make quad-cam-streamer-rebuild

# 清理
make quad-cam-streamer-dirclean
```

## Configuration

Edit `/etc/quad_cam_streamer/config.json` on the board:

```json
{
  "stream": {
    "base_port": 5550,
    "jpeg_quality": 80
  },
  "cameras": [
    {
      "name": "imx334_0",
      "enabled": true,
      "device": "/dev/video0",
      "width": 1920,
      "height": 1080,
      "fps": 30,
      "auto_exposure": true,
      "exposure_us": 10000,
      "analogue_gain": 16
    }
  ],
  "recording": {
    "enabled": true,
    "sd_mount_path": "/mnt/sdcard/SD",
    "output_format": "images",
    "video_codec": "mjpeg",
    "video_bitrate_mbps": 10
  }
}
```

### Configuration Fields

| Field | Description |
|-------|-------------|
| `stream.base_port` | Starting ZMQ port. Camera N uses `base_port + N`. |
| `stream.jpeg_quality` | JPEG quality factor (1-99, higher = better). |
| `cameras[].name` | Descriptive name for logging. |
| `cameras[].enabled` | Set `false` to skip this camera. |
| `cameras[].device` | V4L2 device path (`/dev/videoN`). |
| `cameras[].width/height` | Capture resolution. |
| `cameras[].fps` | Target frame rate. |
| `cameras[].auto_exposure` | Enable auto exposure via V4L2 control. |
| `cameras[].exposure_us` | Manual exposure time (used when auto_exposure=false). |
| `cameras[].analogue_gain` | Sensor analogue gain. |
| `recording.enabled` | Enable recording feature. |
| `recording.output_format` | Output format: `"images"` (JPEG files) or `"video"` (AVI container). |
| `recording.video_codec` | Video codec: `"mjpeg"` (fast, no re-encode) or `"h264"` (smaller files). |
| `recording.video_bitrate_mbps` | H.264 bitrate in Mbps (only used when `video_codec` is `"h264"`). |

### Recording Output Formats

**Save as JPEG images** (default, EuRoC dataset format):
```json
"recording": {
  "output_format": "images"
}
```
Output: Individual JPEG files in `cam0/data/`, `cam1/data/`, etc.

**Save as MJPEG video** (recommended, no re-encoding):
```json
"recording": {
  "output_format": "video",
  "video_codec": "mjpeg"
}
```
Output: `cam0/video.avi`, `cam1/video.avi`, etc. (MJPEG in AVI container)

**Save as H.264 video** (smaller files, higher CPU usage):
```json
"recording": {
  "output_format": "video",
  "video_codec": "h264",
  "video_bitrate_mbps": 10
}
```
Output: `cam0/video.avi`, `cam1/video.avi`, etc. (H.264 in AVI container)

**Note**: Video recording requires FFmpeg installed on the board.

## Finding Camera Device Paths

After booting, identify the correct `/dev/videoN` for each sensor:

```bash
# List all video devices
v4l2-ctl --list-devices

# Or check media topology
media-ctl -p -d /dev/media0
media-ctl -p -d /dev/media1

# Test a specific device
v4l2-ctl -d /dev/video0 --all
```

The ISP virtual video nodes (e.g., `rkisp_mainpath`) are the ones to use.

## Usage

### Board Side

```bash
# Manual start
quad_cam_streamer /etc/quad_cam_streamer/config.json

# Manual start with custom config
quad_cam_streamer /path/to/custom_config.json

# Service control
/etc/init.d/S99quad_cam_streamer start
/etc/init.d/S99quad_cam_streamer stop
/etc/init.d/S99quad_cam_streamer restart
```

### 开机自启管理

默认安装后服务会开机自动启动。可以通过以下方式控制：

**临时禁用**（本次开机不启动，重刷固件后恢复）：

```bash
# 去掉执行权限，init 系统不会运行没有执行权限的脚本
chmod -x /etc/init.d/S99quad_cam_streamer
```

**临时恢复**：

```bash
chmod +x /etc/init.d/S99quad_cam_streamer
```

**永久禁用**（重刷固件后也不启动）：

从 buildroot 的 CMakeLists.txt 安装列表中移除 init 脚本，或者在 config.json 中将所有摄像头设为 `"enabled": false`。

**手动停止正在运行的服务**：

```bash
/etc/init.d/S99quad_cam_streamer stop
```

### Viewer Side (Host PC)

Viewer 工具位于项目的 `tools/` 目录下。

**安装 Python 依赖**：

```bash
cd app/quad_cam_streamer/tools
pip install -r requirements.txt
```

**运行 Viewer**：

```bash
# View all 4 cameras in separate windows
python3 tools/zmq_viewer.py --host <board_ip>

# View in a 2x2 grid
python3 tools/zmq_viewer.py --host <board_ip> --grid

# View specific cameras only
python3 tools/zmq_viewer.py --host <board_ip> --cameras 0,1

# Custom port
python3 tools/zmq_viewer.py --host <board_ip> --base-port 5550
```

Press `q` to quit the viewer.

## Troubleshooting

### Camera fails to open
- Check device path: `ls -la /dev/video*`
- Verify driver loaded: `dmesg | grep -i imx334` or `dmesg | grep -i ov9281`
- Check media pipeline: `media-ctl -p`

### Encoder initialization fails
- Verify MPP is working: `mpi_enc_test`
- Check kernel logs: `dmesg | grep -i mpp`

### No video in viewer
- Verify network connectivity: `ping <board_ip>`
- Check firewall: ZMQ ports (5550-5553) must be accessible
- Test with `zmq_sub` or similar ZMQ diagnostic tool

### Low FPS
- Reduce resolution or JPEG quality in config
- Check CPU/memory usage on board: `top`
- Verify cameras are producing frames: `v4l2-ctl -d /dev/videoN --stream-count=10`
