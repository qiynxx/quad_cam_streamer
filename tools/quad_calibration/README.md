# 四相机标定工具 (Quad Camera Calibration)

针对 Purple Pi OH2 板上 4 路相机（2x IMX334 + 2x OV9281）的内参/外参标定工具。
通过 ZMQ 接收 `quad_cam_streamer` 的 MJPEG 视频流，使用棋盘格标定板完成标定，
输出 EuRoC 格式的 `sensor.yaml` 和分辨率无关的归一化内参。

## 目录结构

```
quad_calibration/
    quad_calibrate.py    # 主脚本
    config.json          # 配置文件（相机、标定板、网络等）
    requirements.txt     # Python 依赖
    README.md            # 本文件
```

## 环境准备

```bash
pip install -r requirements.txt
```

依赖：`pyzmq`, `opencv-python`, `numpy`, `pyyaml`

## 配置说明 (config.json)

| 字段 | 说明 | 默认值 |
|------|------|--------|
| `host` | 板子 IP 地址 | `192.168.100.1` |
| `base_port` | ZMQ 端口基址，cam0=5550, cam1=5551, ... | `5550` |
| `cameras` | 相机列表，可通过 `enabled` 单独启用/禁用 | 4路全开 |
| `board.type` | 标定板类型 | `chessboard` |
| `board.cols` | 棋盘格内角点列数 | `12` |
| `board.rows` | 棋盘格内角点行数 | `9` |
| `board.square_size_mm` | 方格边长 (mm) | `25.0` |
| `reference_camera` | 基准相机编号（body frame 原点） | `0` (imx334_0) |
| `capture_dir` | 采集图像保存目录 | `calib_images` |
| `output_dir` | 标定结果输出目录 | `calib_output` |

> **关于内角点数**：12x9 内角点对应一个 13 列 x 10 行方格的棋盘。
> 请确认你的标定板实际内角点数量与配置一致，否则检测会失败。

## 使用方法

### 前提条件

确保板子上 `quad_cam_streamer` 已运行且网络连通：

```bash
# 在板子上
./quad_cam_streamer

# 在 PC 上验证连通性
ping 192.168.100.1
```

### 方式一：交互模式（推荐新手）

```bash
python quad_calibrate.py
```

自动进入采集模式，采集完按 Q 退出后会提示是否继续标定。

### 方式二：单独采集

```bash
python quad_calibrate.py capture
python quad_calibrate.py capture --host 192.168.100.6
```

### 方式三：单独标定（已有图像）

```bash
python quad_calibrate.py calibrate
python quad_calibrate.py calibrate --capture-dir calib_images --output-dir calib_output
```

### 命令行参数

| 参数 | 说明 |
|------|------|
| `--host` | 覆盖 config 中的板子 IP |
| `--base-port` | 覆盖 config 中的 ZMQ 端口基址 |
| `--capture-dir` | 覆盖采集图像目录 |
| `--output-dir` | 覆盖标定输出目录 |
| `--cell-width` | 预览窗口每格宽度像素，默认 640 |

## 采集操作

运行 capture 模式后会弹出一个 2x2 的四相机实时预览窗口。

**窗口布局：**

```
+------------+------------+
|  imx334_0  |  imx334_1  |
|   (cam0)   |   (cam1)   |
+------------+------------+
|  ov9281_0  |  ov9281_1  |
|   (cam2)   |   (cam3)   |
+------------+------------+
```

**状态指示：**
- 右上角绿色圆点 = 该相机检测到棋盘格
- 右上角红色圆点 = 未检测到
- 检测到时会在画面上绘制角点连线

**快捷键：**

| 按键 | 功能 |
|------|------|
| **空格** | 同时保存 4 个相机的当前帧 |
| **Q** | 退出采集 |

**采集图像保存结构：**

```
calib_images/
    cam0_imx334_0/frame_0000.png, frame_0001.png, ...
    cam1_imx334_1/frame_0000.png, frame_0001.png, ...
    cam2_ov9281_0/frame_0000.png, frame_0001.png, ...
    cam3_ov9281_1/frame_0000.png, frame_0001.png, ...
    capture_meta.json
```

## 采集技巧

1. **数量**：建议采集 **20~30 组**图像
2. **覆盖画面各区域**：中心、四角、边缘都要拍到
3. **变化角度**：不要只拍正面，倾斜 15°~45° 效果更好
4. **保持平整**：标定板不能弯曲
5. **外参需要共视**：做 cam0↔camN 外参时，标定板需要同时被两个相机看到，
   所以尽量把标定板放在多个相机的**重叠视野**内
6. **光照均匀**：避免强反光或阴影覆盖棋盘格
7. **对焦清晰**：确保图像不模糊，尤其是 OV9281 是黑白相机，
   棋盘格对比度本身就好，但要注意距离不要太远

## 标定流程

运行 `calibrate` 模式后自动执行 4 个步骤：

### Step 1: 单目内参标定

对每个相机独立标定：
- 焦距 `fx, fy`
- 光心 `cx, cy`
- 畸变系数 `k1, k2, p1, p2`（4 参数 radial-tangential 模型，固定 k3=0）

### Step 2: 双目外参标定

以 cam0 (imx334_0) 为基准，分别标定：
- cam0 ↔ cam1 (imx334_1)
- cam0 ↔ cam2 (ov9281_0)
- cam0 ↔ cam3 (ov9281_1)

使用 `cv2.stereoCalibrate` + `CALIB_FIX_INTRINSIC`，
仅求解旋转矩阵 R 和平移向量 T。

> IMX334 和 OV9281 分辨率不同没有关系，外参标定时使用各自的内参。

### Step 3: 导出 EuRoC YAML

每个相机生成一个 `sensor.yaml`。

### Step 4: 去畸变可视化

生成原图 vs 去畸变对比图，用于人工检查标定质量。

## 输出文件

```
calib_output/
    cam0_imx334_0/sensor.yaml    # cam0: 内参 + 单位阵（基准相机）
    cam1_imx334_1/sensor.yaml    # cam1: 内参 + 相对 cam0 的 T_BS
    cam2_ov9281_0/sensor.yaml    # cam2: 内参 + 相对 cam0 的 T_BS
    cam3_ov9281_1/sensor.yaml    # cam3: 内参 + 相对 cam0 的 T_BS
    calibration_full.json        # 所有标定数据的 JSON 汇总
    visualizations/              # 去畸变对比图
```

## sensor.yaml 格式说明

```yaml
sensor_type: camera
comment: imx334_0 - reference camera (body frame)

# 4x4 齐次变换矩阵（cam → body frame），基准相机为单位阵
T_BS:
  cols: 4
  rows: 4
  data: [1.0, 0.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, ...]

rate_hz: 30
resolution: [1920, 1080]

# 针孔模型
camera_model: pinhole
intrinsics: [fu, fv, cu, cv]              # 像素单位

# 畸变模型 (k1, k2, p1, p2)
distortion_model: radial-tangential
distortion_coefficients: [k1, k2, p1, p2]

# 标定时的分辨率
calibration_resolution: [1920, 1080]

# 归一化内参（分辨率无关）
intrinsics_normalized: [fu/w, fv/h, cu/w, cv/h]

# 内参标定精度
intrinsic_rms_error: 0.35
```

## 归一化内参的用法

标定是在某个分辨率下完成的（例如 IMX334 的 1920x1080），
归一化内参除以了分辨率，所以**可以直接缩放到任意分辨率**：

```python
# 从 sensor.yaml 读取归一化内参
norm = sensor['intrinsics_normalized']  # [fu/w, fv/h, cu/w, cv/h]

# 缩放到新分辨率
new_w, new_h = 960, 540
fu = norm[0] * new_w
fv = norm[1] * new_h
cu = norm[2] * new_w
cv = norm[3] * new_h
```

畸变系数 `distortion_coefficients` 本身就与分辨率无关，不需要缩放。

## 标定质量判断

| 指标 | 优秀 | 良好 | 需注意 |
|------|------|------|--------|
| 单目内参 RMS | < 0.3 | < 0.5 | > 1.0 |
| 双目外参 RMS | < 0.5 | < 1.0 | > 2.0 |

**其他检查方法：**
- 查看 `visualizations/` 目录中的去畸变对比图，直线应该是直的
- 归一化内参验证：`intrinsics_normalized[0] * resolution[0]` 应等于 `intrinsics[0]`
- 光心 `cu/w ≈ 0.5`, `cv/h ≈ 0.5`（接近图像中心为正常）

## 常见问题

**Q: 检测不到棋盘格？**
- 确认 `config.json` 中的 `cols` 和 `rows` 与实际标定板的**内角点数**一致
- 12x9 内角点 = 13 列 x 10 行方格
- 图像是否模糊？光照是否充足？

**Q: RMS 太大（> 2.0）？**
- 增加采集数量，覆盖更多角度
- 删除明显模糊或标定板弯曲的图像，重新标定
- 检查标定板是否真的平整

**Q: 外参标定失败（Not enough common frames）？**
- 需要标定板**同时出现在两个相机画面中**
- 把标定板放在相机重叠视野区域
- 建议每组至少 2 个相机能同时看到标定板

**Q: 只想标定部分相机？**
- 在 `config.json` 中把不需要的相机 `enabled` 设为 `false`
