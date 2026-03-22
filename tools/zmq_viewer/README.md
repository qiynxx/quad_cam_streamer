# ZMQ Viewer - 四相机可视化与参数控制工具

## 功能特性

- ✅ 2x2网格实时显示四路相机
- ✅ 时间戳同步匹配
- ✅ IMU数据实时显示
- ✅ **实时参数控制** (曝光、增益、JPEG质量)
- ✅ **启动时自动读取设备当前配置**
- ✅ 交互式滑块调参 (快速调整 1-5000us)
- ✅ 手动输入精确值 (支持 1-33000us 全范围)
- ✅ 参数持久化保存
- ✅ 美观直观的控制界面

## 安装依赖

```bash
pip install -r requirements.txt
```

## 使用方法

### 基础使用

```bash
# 默认连接 192.168.100.6，自动读取设备当前配置
python3 zmq_viewer.py

# 指定主机
python3 zmq_viewer.py --host 192.168.1.100

# 调整显示尺寸
python3 zmq_viewer.py --cell-width 800

# 禁用参数控制面板
python3 zmq_viewer.py --no-control
```

启动时会自动查询设备当前参数并显示：
```
Querying current parameters from device...
  cam0: exp=8000us gain=32 q=85 auto=False
  cam1: exp=10000us gain=16 q=85 auto=True
  cam2: exp=5000us gain=24 q=85 auto=False
  cam3: exp=5000us gain=24 q=85 auto=False
```

### 参数控制

启动后会显示两个窗口：
1. **Quad Camera Viewer** - 相机视频网格
2. **Camera Controls** - 参数控制面板

#### 控制面板操作

控制面板采用美观的深色主题，分为三个区域：

**1. 相机选择区 (顶部)**
- 显示当前选中的相机名称和索引
- 显示自动曝光状态 (绿色=开启, 灰色=关闭)

**2. 参数显示区 (中部)**
- 三个彩色参数框实时显示当前值
  - 蓝色: 曝光时间 (us)
  - 橙色: 增益
  - 紫色: JPEG质量

**3. Trackbar 控制区**
- **Camera** - 选择要调整的相机 (0-3)
- **Auto Exp** - 自动曝光开关 (0=关闭, 1=开启)
- **Exposure** - 曝光时间滑块 (1-5000 微秒，快速调整)
- **Gain** - 模拟增益滑块 (1-255)
- **Quality** - JPEG质量滑块 (1-100)

**4. 说明区 (底部)**
- 显示所有键盘快捷键和功能说明

#### 键盘快捷键

| 按键 | 功能 |
|------|------|
| `q` | 退出程序 |
| `SPACE` | 暂停/恢复视频流 |
| `s` | 保存当前参数到 config.json |
| `c` | 显示/隐藏控制面板 |
| `e` | 手动输入曝光值 (1-33000us，突破滑块限制) |
| `g` | 手动输入增益值 (1-255) |
| `j` | 手动输入JPEG质量 (1-100) |

**注意**:
- Trackbar 曝光范围限制在 1-5000us，适合快速调整
- 按 `e` 键可手动输入 1-33000us 全范围曝光值

## 典型场景

### 相机标定

```bash
# 1. 启动viewer (自动加载设备当前配置)
python3 zmq_viewer.py

# 2. 在控制面板中：
#    - 选择相机 (Camera滑块)
#    - 关闭自动曝光 (Auto Exp = 0)
#    - 用滑块快速调整曝光 (1-5000us)
#    - 或按 'e' 键精确输入曝光值 (支持 1-33000us)
#    - 观察视频窗口，调整到棋盘格清晰
#    - 按 's' 保存参数

# 3. 对所有相机重复步骤2

# 4. 验证参数已保存
python3 test_query.py  # 查询设备当前参数
```

### 低光环境调试

```bash
# 增加曝光时间和增益
# 在控制面板中：
#   Auto Exp = 0
#   Exposure = 20000 (20ms)
#   Gain = 64
# 按 's' 保存
```

### 网络带宽优化

```bash
# 降低JPEG质量减少带宽
# 在控制面板中：
#   Quality = 60
# 按 's' 保存
```

## 参数说明

### 曝光时间 (Exposure)

- 范围: 1-33000 微秒
- 推荐值:
  - 室内: 8000-15000us
  - 室外: 2000-5000us
  - 低光: 20000-33000us
  - 高速运动: 1000-3000us

### 模拟增益 (Gain)

- 范围: 1-255
- 推荐值:
  - 正常光照: 16-32
  - 低光: 64-128
  - 注意: 增益过高会增加噪点

### JPEG质量 (Quality)

- 范围: 1-100
- 推荐值:
  - 高质量: 85-95
  - 平衡: 70-80
  - 低带宽: 50-65

## 配置文件

`config.json` 示例：

```json
{
  "host": "192.168.100.6",
  "base_port": 5550,
  "cell_width": 640,
  "imu_port": 5560,
  "cameras": [
    {"index": 0, "name": "imx334_0", "enabled": true},
    {"index": 1, "name": "imx334_1", "enabled": true},
    {"index": 2, "name": "ov9281_0", "enabled": true},
    {"index": 3, "name": "ov9281_1", "enabled": true}
  ]
}
```

## 端口说明

- **5550-5553**: 相机视频流 (PUB/SUB)
- **5560**: IMU数据流 (PUB/SUB)
- **5570**: 参数控制端口 (REQ/REP)

## 故障排查

### 控制面板无响应

```bash
# 检查控制端口是否可达
nc -zv 192.168.100.6 5570

# 测试查询功能
python3 test_query.py 192.168.100.6

# 查看设备日志
ssh root@192.168.100.6 "journalctl -u quad_cam_streamer -f"
```

### 参数未保存

**问题**: 按 's' 保存后，重启 viewer 参数还是旧值

**原因**:
1. 设备上 config.json 没有写权限
2. 设备磁盘空间不足
3. 保存命令未成功发送

**解决**:
```bash
# 1. 检查设备磁盘空间
ssh root@192.168.100.6 "df -h"

# 2. 检查 config.json 权限
ssh root@192.168.100.6 "ls -la /etc/quad_cam_streamer/config.json"

# 3. 手动验证保存
python3 test_query.py  # 保存前查询
# 在 viewer 中调整参数并按 's'
python3 test_query.py  # 保存后查询，对比是否变化
```

### 启动时参数显示错误

**问题**: 启动时显示 "Warning: Could not query device parameters"

**原因**: 设备未运行或网络不通

**解决**:
```bash
# 检查设备是否运行
ssh root@192.168.100.6 "systemctl status quad_cam_streamer"

# 检查网络连通性
ping 192.168.100.6

# 手动测试查询
python3 test_query.py 192.168.100.6
```

### 视频卡顿

- 降低 JPEG 质量减少网络负载
- 检查网络延迟: `ping 192.168.100.6`

## 技术细节

### 参数更新机制

1. 滑块调整 → 发送ZMQ命令到设备
2. 设备更新运行时参数 (下一帧生效)
3. 按 's' 键 → 发送持久化命令
4. 设备原子写入 config.json

### 时间戳同步

- 使用参考相机 (默认cam0) 的最新帧作为锚点
- 其他相机匹配最接近的时间戳
- 显示同步误差 (sync spread)

## 开发

### 添加新参数

编辑 `zmq_viewer.py`:

```python
# 1. 在 CameraParams 类中添加字段
class CameraParams:
    def __init__(self, cam_idx, name):
        self.new_param = 100  # 新参数

# 2. 添加trackbar
cv2.createTrackbar("NewParam", control_window, 100, 200, on_new_param)

# 3. 添加回调
def on_new_param(val):
    p = cam_params[selected_cam]
    p.new_param = val
    param_ctrl.send_params(selected_cam, new_param=val)
```

## 许可证

与 quad_cam_streamer 项目相同
