# Quad Camera Streamer

运行在 Purple Pi OH2 (RK3576) 上的多传感器采集与推流程序。同时采集 4 路摄像头、最多 3 路 IMU，使用 Rockchip MPP 硬件 JPEG 编码，通过 ZMQ 实时推流，并支持按 EuRoC 数据集格式录制到 SD 卡。

---

## 软件架构

### 整体数据流

```
┌─────────────────────────────────────────────────────────────────────┐
│                          传感器硬件层                                │
│  IMX334×2 + OV9281×2      ICM-42688-P        串口IMU×2              │
│  (via V4L2 ISP pipeline)  (via I2C /dev/i2c-3) (UART /dev/ttySx)   │
└─────┬───────────────────────────┬────────────────────┬──────────────┘
      │                           │                    │
      ▼                           ▼                    ▼
┌──────────────┐         ┌────────────────┐   ┌───────────────────┐
│ 相机采集线程×4 │         │  IMU 读取线程   │   │ 串口IMU线程×2      │
│              │         │  (ImuReader)   │   │ (SerialImuReader) │
│ V4L2Camera   │         └───────┬────────┘   └────────┬──────────┘
│ (MMAP 采集)  │                 │                     │
│     ↓        │                 ▼                     ▼
│ NV12 变换    │         ZMQ PUB :5560          ZMQ PUB :5561/:5562
│ (旋转/镜像)  │
│     ↓        │         ┌──────────────────────────────────────────┐
│ AutoExposure │         │            EuRoC 录制子系统               │
│ (软件AE)     │         │                                          │
│     ↓        │──push──▶│  相机环形缓冲区 (300帧/路)                 │
│ MppEncoder   │         │  IMU 数据缓冲                             │
│ (硬件JPEG)   │         │       ↓                                  │
│     ↓        │         │  帧同步对齐 (时间戳匹配, ±33ms)            │
│ ZmqStreamer  │         │       ↓                                  │
│ PUB :5550-53 │         │  异步磁盘写入线程                          │
└──────────────┘         │  JPEG文件 / AVI视频 / IMU CSV             │
                         └──────────────────────┬───────────────────┘
                                                │
                                                ▼
                                         /mnt/sdcard/SD/
                                         euroc_NNN/ (EuRoC格式)
```

### 线程模型

| 线程 | 数量 | 主要职责 |
|------|------|---------|
| 主线程 | 1 | 信号处理、线程生命周期管理、STREAMON 同步屏障 |
| 相机采集线程 | 4 | V4L2采集 → 图像变换 → MPP编码 → ZMQ推流 → 录制缓冲区 |
| I2C IMU 线程 | 1 | I2C轮询读取 → ZMQ推流，端口 5560 |
| 串口 IMU 线程 | 最多 2 | UART 帧解析 → ZMQ推流，端口 5561/5562；断线自动重连 |
| EuRoC 磁盘写入线程 | 1 | 从环形缓冲区消费同步帧，写 JPEG/视频/CSV 到 SD 卡 |
| 按键监听线程 | 1 | 监听 ADC 按键事件，触发录制开始/停止 |
| 参数控制线程 | 1 | ZMQ REP 服务端，接收运行时参数更新命令，端口 5570 |
| 音频播放线程 | 按需 | ALSA PCM 合成音效，在后台异步播放 |

### 模块说明

#### 相机流水线（`v4l2_camera` → `mpp_encoder` → `zmq_streamer`）

每路相机独立运行一个线程，执行如下流水线：

1. **V4L2 采集**（`v4l2_camera.h`）：以 MMAP 零拷贝方式从 `/dev/videoN` 捕获 NV12 帧；通过 V4L2 控制接口和 subdev 接口调整曝光和模拟增益。
2. **图像变换**（`main.cpp`）：根据配置对 NV12 帧执行 180° 旋转（`rotate_180`）或水平镜像（`mirror`），用于校正不同安装方向的传感器。
3. **自动曝光**（`auto_exposure.h`）：纯软件实现的亮度控制回路，目标亮度 128，±10% 容差带；先调曝光（100µs–30000µs），触达上下限后再调增益。
4. **MPP 硬件编码**（`mpp_encoder.h`）：调用 Rockchip MPP API 将 NV12 帧硬件编码为 JPEG，编码质量可运行时调整（1–99）。
5. **ZMQ 推流**（`zmq_streamer.h`）：以 ZMQ PUB 方式向网络发布 JPEG 帧，端口分配为 `base_port + cam_index`（默认 5550–5553）。

可选模块：
- **rkaiq 3A**（`rkaiq_controller.h`）：当 `use_rkaiq: true` 时，使用 Rockchip ISP 引擎做全链路 3A 控制，替代软件 AE。

#### STREAMON 同步屏障

主线程在所有相机线程完成初始化后，通过 `std::barrier` 统一触发 `VIDIOC_STREAMON`，保证所有相机在同一时刻开始采集，减少多路图像的初始帧时间戳偏差。

#### IMU 子系统

| 接口 | 模块 | 传感器 | ZMQ 端口 |
|------|------|--------|---------|
| I2C `/dev/i2c-3` | `imu_reader.h` | ICM-42688-P（或 LSM6DS3） | 5560 |
| UART `/dev/ttyS4` | `serial_imu_reader.h` | 串口 IMU（左） | 5561 |
| UART `/dev/ttyS10` | `serial_imu_reader.h` | 串口 IMU（右） | 5562 |

串口 IMU 使用 31 字节二进制帧协议（含校验和），波特率最高支持 921600。断线时自动重连，并通过音频提示（`IMU_DISCONNECT` 声效）通知用户。

数据单位统一转换为 SI 制（加速度 m/s²，角速度 rad/s）后发布。I2C IMU 还支持配置 3×3 旋转矩阵，将传感器坐标系转换到机体坐标系。

#### EuRoC 录制子系统（`euroc_recorder.h`）

录制子系统分两层：

**缓冲层**：每路相机维护深度 300 帧（约 10 秒@30fps）的环形缓冲区。相机线程录制期间将编码后的 JPEG 及时间戳异步推入缓冲区，不阻塞采集流水线。

**同步与写入层**：磁盘写入线程执行帧组同步算法：
- 等待所有已启用相机的缓冲区均有数据；
- 比较各相机队首帧时间戳，差异在 1 帧周期（33.3ms@30fps）内则打包为一个帧组；
- 超出阈值则丢弃最旧帧并重试对齐；
- 帧组统一使用主相机（IMX334 #0）的时间戳作为文件名和 CSV 索引。

视频模式下通过 FFmpeg 管道写入 AVI 容器（MJPEG 直接透传，无需二次编码；H.264 模式需 CPU 转码）。

录制开始/结束时自动从 `calib_dir` 复制标定文件（`sensor.yaml`、`body.yaml`）到数据集目录。

#### 按键与音频

- **KeyMonitor**（`key_monitor.h`）：监听 Linux input 事件设备，自动识别 ADC 按键（`adc-keys`），500ms 去抖动后触发录制切换回调。
- **AudioPlayer**（`audio_player.h`）：基于 ALSA PCM 合成音效，支持 BOOT、REC\_START/STOP、REC\_BEEP（录制中定时提示音）、REC\_ERROR、IMU\_DISCONNECT 等事件。

#### 运行时参数控制（`param_controller.h`）

ZMQ REP 服务端，监听端口 **5570**，接收 JSON 命令，支持运行时修改：
- 各相机的曝光时间（`exposure_us`）
- 模拟增益（`analogue_gain`）
- JPEG 编码质量（`jpeg_quality`）

参数更新通过原子版本号通知相机线程，同时持久化写回 `config.json`。

命令示例：
```json
{"cmd": "set_exposure", "cam": 0, "value": 5000}
{"cmd": "set_quality", "value": 90}
```

#### PWM 硬件同步（`pwm_sync.h`）

当 `hardware_sync.enabled: true` 时，程序在采集开始前通过 sysfs 接口（`/sys/class/pwm`）配置 4 路 PWM 输出，向所有相机发送 FSIN 同步信号，确保多路传感器帧对齐。脉冲宽度 100µs，频率与 `hardware_sync.fps` 一致。

---

## 项目结构

```
app/quad_cam_streamer/
├── build.sh                       # 一键编译脚本（通过 Buildroot make 系统）
├── build_local.sh                 # 本地交叉编译脚本（直接 CMake，速度更快）
├── CMakeLists.txt                 # CMake 构建配置
├── config.json                    # 默认配置文件
├── S99quad_cam_streamer           # SysV 开机自启脚本
├── calib/                         # 传感器标定文件
│   ├── cam0_sensor.yaml           # 各相机内参/外参（EuRoC格式）
│   ├── cam1_sensor.yaml
│   ├── cam2_sensor.yaml
│   ├── cam3_sensor.yaml
│   ├── imu_sensor.yaml            # I2C IMU 参数
│   ├── imu_left_sensor.yaml       # 串口 IMU（左）参数
│   ├── imu_right_sensor.yaml      # 串口 IMU（右）参数
│   └── body.yaml                  # 机体坐标系描述
├── src/                           # C++ 源码
│   ├── main.cpp                   # 入口：信号处理、线程管理、STREAMON 屏障
│   ├── config.h / config.cpp      # JSON 配置加载（nlohmann/json）
│   ├── v4l2_camera.h / .cpp       # V4L2 摄像头采集（MMAP、subdev 控制）
│   ├── mpp_encoder.h / .cpp       # Rockchip MPP JPEG 硬件编码
│   ├── zmq_streamer.h / .cpp      # ZMQ PUB 推流
│   ├── auto_exposure.h / .cpp     # 软件自动曝光控制回路
│   ├── rkaiq_controller.h / .cpp  # Rockchip ISP 3A（可选，use_rkaiq=true 时启用）
│   ├── imu_reader.h / .cpp        # I2C IMU 读取（ICM-42688-P）
│   ├── serial_imu_reader.h / .cpp # 串口 IMU 读取（二进制帧协议）
│   ├── euroc_recorder.h / .cpp    # EuRoC 格式录制（图像/视频/IMU）
│   ├── key_monitor.h / .cpp       # 硬件按键监听（ADC 按键 input 事件）
│   ├── audio_player.h / .cpp      # ALSA 音效合成与异步播放
│   ├── pwm_sync.h / .cpp          # PWM FSIN 多相机硬件同步
│   └── param_controller.h / .cpp  # 运行时参数更新（ZMQ REP，端口 5570）
└── tools/
    ├── zmq_viewer.py              # Python ZMQ MJPEG 接收与显示
    └── requirements.txt
```

---

## 编译

### 依赖

编译前需完成以下准备：

**1. 主机工具链（Ubuntu/Debian）**
```bash
sudo apt install -y build-essential cmake git python3 \
    device-tree-compiler bc flex bison libssl-dev file unzip rsync cpio
```

**2. SDK 初始化**（首次构建）
```bash
cd rk3576_purple_pi_oh2_linux_sdk
./build.sh rockchip_rk3576_defconfig
./build.sh all    # 生成交叉工具链 + sysroot
```

**3. Buildroot 目标库**

| Buildroot 包 | 用途 |
|-------------|------|
| `camera-engine-rkaiq` | Rockchip ISP 3A 引擎 |
| `rockchip_mpp` | 硬件编解码 |
| `zeromq` / `cppzmq` | ZMQ 消息传输 |
| `json-for-modern-cpp` | JSON 解析 |
| `alsa-lib` | 音频播放 |

使用一键脚本安装（推荐）：
```bash
cd app/quad_cam_streamer
./build.sh init
```

### 编译命令

| 命令 | 说明 |
|------|------|
| `./build.sh init` | 首次编译，自动启用依赖包并编译 |
| `./build.sh` | 增量编译 |
| `./build.sh rootfs` | 编译 + 打包进 rootfs 镜像 |
| `./build.sh clean` | 清理构建产物 |
| `./build_local.sh` | 本地增量编译（更快，直接 CMake） |
| `./build_local.sh rebuild` | 本地清理重编 |
| `./build_local.sh push` | 本地编译 + adb push 到板子 |
| `./build_local.sh debug` | 编译带调试符号的版本 |

### 编译产物

| 文件 | rootfs 路径 |
|------|------------|
| `quad_cam_streamer` | `/usr/bin/` |
| `config.json` | `/etc/quad_cam_streamer/` |
| `calib/*.yaml` | `/etc/quad_cam_streamer/calib/` |
| `S99quad_cam_streamer` | `/etc/init.d/` |

---

## 配置

编辑板上 `/etc/quad_cam_streamer/config.json`：

```json
{
  "stream": {
    "host": "192.168.100.6",
    "base_port": 5550,
    "jpeg_quality": 80
  },
  "cameras": [
    {
      "name": "m01_b_imx334 5-0036",
      "enabled": true,
      "device": "/dev/video53",
      "subdev": "/dev/v4l-subdev8",
      "width": 1920, "height": 1080, "fps": 30,
      "format": "NV12",
      "rotate_180": false,
      "mirror": false,
      "auto_exposure": true,
      "exposure_us": 2000,
      "analogue_gain": 16,
      "use_rkaiq": false,
      "iq_file_dir": "/etc/iqfiles"
    }
  ],
  "imu": {
    "enabled": true,
    "i2c_device": "/dev/i2c-3",
    "i2c_addr": 107,
    "sampling_frequency": 208,
    "accel_range": 4,
    "gyro_range": 1000,
    "zmq_port": 5560,
    "rotation_matrix": [[0,0,-1],[0,1,0],[1,0,0]]
  },
  "serial_imus": [
    {"enabled": true, "name": "imu_left",  "uart_device": "/dev/ttyS4",  "baudrate": 921600, "zmq_port": 5561},
    {"enabled": true, "name": "imu_right", "uart_device": "/dev/ttyS10", "baudrate": 921600, "zmq_port": 5562}
  ],
  "recording": {
    "enabled": true,
    "sd_mount_path": "/mnt/sdcard/SD",
    "calib_dir": "/etc/quad_cam_streamer/calib",
    "record_key_code": 115,
    "input_device": "",
    "output_format": "image",
    "video_codec": "mjpeg",
    "video_bitrate_mbps": 10
  },
  "hardware_sync": {
    "enabled": true,
    "fps": 30
  }
}
```

### 主要配置项说明

| 字段 | 说明 |
|------|------|
| `stream.base_port` | ZMQ 起始端口，相机 N 使用 `base_port + N` |
| `stream.jpeg_quality` | JPEG 编码质量（1–99） |
| `cameras[].device` | V4L2 设备路径 |
| `cameras[].subdev` | V4L2 subdev 路径（用于曝光/增益控制） |
| `cameras[].rotate_180` | 帧旋转 180°（安装方向补偿） |
| `cameras[].mirror` | 帧水平镜像 |
| `cameras[].auto_exposure` | 启用软件自动曝光 |
| `cameras[].use_rkaiq` | 启用 Rockchip ISP 3A（与 auto_exposure 二选一） |
| `imu.rotation_matrix` | 传感器→机体坐标系 3×3 旋转矩阵 |
| `recording.output_format` | `"image"`（JPEG 文件）或 `"video"`（AVI 容器） |
| `recording.video_codec` | `"mjpeg"`（直接透传）或 `"h264"`（转码，需 FFmpeg） |
| `recording.record_key_code` | 触发录制的 Linux input 按键码（115 = KEY_VOLUMEDOWN） |
| `hardware_sync.enabled` | 启用 PWM FSIN 多相机帧同步 |

---

## ZMQ 推流端口汇总

| 数据源 | 端口 | 说明 |
|--------|------|------|
| 相机 0 (IMX334 #0) | 5550 | ZMQ PUB，JPEG 帧 |
| 相机 1 (IMX334 #1) | 5551 | ZMQ PUB，JPEG 帧 |
| 相机 2 (OV9281 #0) | 5552 | ZMQ PUB，JPEG 帧 |
| 相机 3 (OV9281 #1) | 5553 | ZMQ PUB，JPEG 帧 |
| I2C IMU | 5560 | ZMQ PUB，IMU 数据 |
| 串口 IMU（左） | 5561 | ZMQ PUB，IMU 数据 |
| 串口 IMU（右） | 5562 | ZMQ PUB，IMU 数据 |
| 参数控制服务 | 5570 | ZMQ REP，JSON 命令接口 |

---

## 录制数据集

录制格式基于 [EuRoC MAV Dataset](https://projects.asl.ethz.ch/datasets/doku.php?id=kmavvisualinertialdatasets)，按顺序编号创建目录（`euroc_001`, `euroc_002`, ...）。

### Image 模式目录结构

```
/mnt/sdcard/SD/euroc_001/mav0/
├── body.yaml
├── cam0/
│   ├── sensor.yaml          # 内参/畸变/外参
│   ├── data.csv             # 时间戳-文件名索引
│   └── data/
│       ├── 1719312345678.jpg
│       └── ...
├── cam1/ ... cam2/ ... cam3/
├── imu0/                    # I2C IMU
│   ├── sensor.yaml
│   └── data.csv
├── imu1/                    # 串口 IMU（左）
│   ├── sensor.yaml
│   └── data.csv
└── imu2/                    # 串口 IMU（右）
    ├── sensor.yaml
    └── data.csv
```

### Video 模式

与 Image 模式相同，`data/` 替换为 `video.avi`（MJPEG 或 H.264），`data.csv` 保留时间戳索引。

### CSV 格式

**相机**：
```
#timestamp [ns],filename
1719312345678,1719312345678.jpg
```

**IMU**：
```
#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]
1719312345000,0.001234,-0.002345,0.000123,0.123456,-9.801234,0.012345
```

---

## 使用

### 板端

```bash
# 手动启动（使用默认配置）
quad_cam_streamer /etc/quad_cam_streamer/config.json

# SysV 服务管理
/etc/init.d/S99quad_cam_streamer start|stop|restart

# 临时禁用开机自启
chmod -x /etc/init.d/S99quad_cam_streamer
```

### 查看摄像头设备

```bash
v4l2-ctl --list-devices
media-ctl -p -d /dev/media0
v4l2-ctl -d /dev/video53 --all
```

### PC 端查看推流

```bash
cd app/quad_cam_streamer/tools
pip install -r requirements.txt

python3 zmq_viewer.py --host <board_ip>           # 4路独立窗口
python3 zmq_viewer.py --host <board_ip> --grid    # 2×2 网格
python3 zmq_viewer.py --host <board_ip> --cameras 0,1  # 仅查看指定路
```

### 运行时更新参数

```bash
# 设置相机0曝光时间为 5000µs
echo '{"cmd":"set_exposure","cam":0,"value":5000}' | zmq_send tcp://<board_ip>:5570

# 设置 JPEG 质量
echo '{"cmd":"set_quality","value":90}' | zmq_send tcp://<board_ip>:5570
```

---

## 故障排查

| 现象 | 排查步骤 |
|------|---------|
| 摄像头打开失败 | `ls /dev/video*`；`dmesg \| grep -i imx334`；`media-ctl -p` |
| MPP 编码器初始化失败 | `mpi_enc_test`；`dmesg \| grep -i mpp` |
| PC 端无图像 | `ping <board_ip>`；检查防火墙开放 5550–5553 |
| 帧率低 | 降低分辨率或 JPEG 质量；`top` 检查 CPU 占用 |
| 串口 IMU 断线 | 检查 `/dev/ttySx` 权限和波特率；监听音效提示 |
| 录制无法启动 | 确认 SD 卡已挂载到 `sd_mount_path`；检查磁盘空间 |
| 录制帧不同步 | 确认 `hardware_sync.enabled: true`；检查 PWM sysfs 节点 |
