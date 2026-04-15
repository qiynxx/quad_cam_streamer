# BLE 时间同步与命令协议

本文档描述 RK3576 (Central) 与 BLE 外设 (ESP32S3-L、ESP32S3-R、RK3588-W) 之间的时间同步协议及数据格式。

---

## 1. GATT 服务与特征

| UUID | 名称 | 方向 | 说明 |
|------|------|------|------|
| `12345678-1234-5678-1234-56789abcdeff` | Primary Service | -- | 主服务 UUID（RK3588-W 用于过滤多服务冲突） |
| `12345678-1234-5678-1234-56789abcdef1` | IMU Data | Peripheral → Central | IMU 数据 Notify（仅 ESP32） |
| `12345678-1234-5678-1234-56789abcdef2` | Control | Central → Peripheral | 控制命令（启停 IMU 等） |
| `12345678-1234-5678-1234-56789abcdef3` | Config | Central ← Peripheral | 设备角色配置（读取） |
| `12345678-1234-5678-1234-56789abcdef4` | TimeSync | 双向 | 时间同步 + 录制命令（Write + Notify） |

---

## 2. 命令格式（RK3576 → 外设，写入 TimeSync 特征）

所有多字节字段均为 **小端序 (Little-Endian)**。

### 2.1 Ping (0x01)

用于测量 RK3576 与外设之间的时钟偏移。

```
Byte 0:     0x01 (opcode)
Bytes 1-4:  t1_ms (uint32_t LE) — RK3576 发送时的 CLOCK_MONOTONIC 毫秒数
```
**总长度：5 字节**

### 2.2 SetOffset (0x02)

将计算出的时钟偏移下发给外设，外设据此校准自身时间戳。

```
Byte 0:     0x02 (opcode)
Bytes 1-4:  offset (int32_t LE) — 有符号毫秒偏移量
```
**总长度：5 字节**

偏移含义：`RK3576_时间 = 外设_时间 + offset`

### 2.3 StopPattern (0x04)

停止外设上的振动/LED pattern。

```
Byte 0:     0x04 (opcode)
```
**总长度：1 字节**

### 2.4 RecordStart (0x10)

通知外设（RK3588-W）开始录制。

```
Byte 0:     0x10 (opcode)
Bytes 1-4:  timestamp (uint32_t LE) — RK3576 发送时的 CLOCK_MONOTONIC 毫秒数
```
**总长度：5 字节**

### 2.5 RecordStop (0x11)

通知外设（RK3588-W）停止录制。

```
Byte 0:     0x11 (opcode)
Bytes 1-4:  timestamp (uint32_t LE) — RK3576 发送时的 CLOCK_MONOTONIC 毫秒数
```
**总长度：5 字节**

---

## 3. 响应格式（外设 → RK3576，TimeSync 特征 Notify）

### 3.1 Pong (0x81)

外设收到 Ping 后回复。

```
Byte 0:     0x81 (opcode)
Bytes 1-4:  t2 (uint32_t LE) — 外设收到 Ping 时的本地毫秒时间戳
Bytes 5-8:  rk_echo (uint32_t LE) — 原样回传 Ping 中的 t1_ms，用于校验匹配
```
**总长度：9 字节**

---

## 4. 时间同步算法

### 4.1 单次 Ping-Pong 测量

```
RK3576 (Central)                    外设 (Peripheral)
     │                                     │
     │  t1 = CLOCK_MONOTONIC (ms)          │
     │──── Ping [0x01 | t1] ─────────────>│
     │                                     │  t2 = 外设本地时间 (ms)
     │                                     │
     │<──── Pong [0x81 | t2 | t1] ────────│
     │  t4 = CLOCK_MONOTONIC (ms)          │
     │                                     │
```

计算：
- **RTT** = t4 - t1（往返时延）
- **mid** = t1 + RTT / 2（估计外设收到 Ping 时的 RK3576 时刻）
- **offset** = mid - t2（RK3576 时钟 - 外设时钟的差值）

### 4.2 单轮同步 (run_sync_round)

| 参数 | 值 | 说明 |
|------|---|------|
| kPingCount | 7 | 每轮发送 Ping 次数 |
| kPingIntervalMs | 50 ms | 两次 Ping 之间的间隔 |
| kMaxRttMs | 100 ms | RTT 超过此值的样本丢弃 |
| kMinSamples | 5 | 至少需要的有效样本数 |
| kBestCount | 3 | 从有效样本中取 RTT 最小的 N 个 |
| Pong 超时 | 300 ms | 等待 Pong 响应的超时时间 |

算法流程：

1. 发送 7 次 Ping，每次间隔 50ms
2. 每次 Ping 后等待 Pong（超时 300ms）
3. 校验：Pong 中的 `rk_echo` 必须与 Ping 中的 `t1` 一致
4. 丢弃 RTT > 100ms 的样本
5. 至少需要 5 个有效样本，否则本轮失败
6. 将有效样本按 RTT 升序排序
7. 取 RTT 最小的 3 个样本
8. 对这 3 个 offset 取中位数作为最终结果

### 4.3 初始同步 (do_time_sync)

1. 执行一轮 `run_sync_round`
2. 若失败，等待 500ms 后重试一次
3. 若仍失败，以 offset=0 继续（不阻断连接）

### 4.4 周期重同步 (resync_loop)

| 参数 | 值 | 说明 |
|------|---|------|
| kRoundIntervalSec | 30 s | 两轮重同步之间的间隔 |
| kDeviceGapMs | 3000 ms | 同一轮内两个设备之间的间隔 |

- 每 30 秒对所有 STREAMING 状态的设备做一轮重同步
- **逐个串行**执行，设备之间间隔 3 秒，避免 BLE 控制器拥塞
- 同步失败只打警告，不断连

---

## 5. 设备初始化流程

### 5.1 ESP32 设备 (LEFT / RIGHT)

```
Connected
  │
  ├─ Enable TimeSync Notify
  ├─ Subscribe TimeSync notifications
  ├─ Read Config characteristic → 确定 LEFT/RIGHT 角色
  ├─ StopPattern (0x04)
  ├─ do_time_sync → Ping/Pong 同步 → SetOffset (0x02)
  ├─ Enable IMU Notify
  ├─ Subscribe IMU notifications
  ├─ Write 0x01 to Control → 启动 IMU 数据流
  │
  └─ State = STREAMING
```

### 5.2 RK3588-W 设备 (WAIST)

```
Connected
  │
  ├─ Enable TimeSync Notify
  ├─ Subscribe TimeSync notifications
  ├─ Read Config characteristic（角色由设备名 "RK3588-W" 决定，不被 Config 字节覆盖）
  ├─ StopPattern (0x04)
  ├─ do_time_sync → 尝试 Ping/Pong（当前无 Pong 回复，超时后 offset=0 继续）
  ├─ [跳过] IMU Notify（RK3588-W 不发送 IMU 数据）
  │
  └─ State = STREAMING
```

RK3588-W 与 ESP32 的关键差异：

| 特性 | ESP32 (LEFT/RIGHT) | RK3588-W (WAIST) |
|------|---------------------|-------------------|
| 时间同步 | Ping/Pong 正常工作 | Ping 超时，offset=0 |
| IMU 数据 | 通过 Notify 推送 32 字节 IMU 包 | 不发送 |
| Watchdog IMU 超时 | 检查，超时后断连重试 | 跳过 |
| 录制命令 | 不接收 | 接收 START(0x10) / STOP(0x11) |
| 周期重同步 | 每 30s 重同步 | 每 30s 尝试（超时后继续） |

---

## 6. IMU 数据包格式（ESP32 → RK3576，IMU 特征 Notify）

```
Bytes  0- 3:  accel_x  (float32 LE, m/s²)
Bytes  4- 7:  accel_y  (float32 LE, m/s²)
Bytes  8-11:  accel_z  (float32 LE, m/s²)
Bytes 12-15:  gyro_x   (float32 LE, deg/s → 接收后转 rad/s)
Bytes 16-19:  gyro_y   (float32 LE, deg/s → 接收后转 rad/s)
Bytes 20-23:  gyro_z   (float32 LE, deg/s → 接收后转 rad/s)
Bytes 24-27:  (保留)
Bytes 28-31:  timestamp_ms (uint32_t LE, 外设本地毫秒时间戳)
```
**总长度：32 字节**

时间戳处理：
- 32 位时间戳通过 `expand_timestamp_ms` 扩展为 64 位（检测溢出并累加 wrap 计数）
- 最终转换为纳秒：`timestamp_ns = expanded_ms * 1,000,000`
- 结合 `sync_offset` 对齐到 RK3576 的 `CLOCK_MONOTONIC` 时间线

---

## 7. 录制命令转发逻辑

用户按键后，RK3576 根据本地录制状态决定发送的 BLE 命令：

| 按键前状态 | 本地 toggle 结果 | BLE 命令 | 说明 |
|-----------|-----------------|---------|------|
| 正在录制 | 停止 | **STOP (0x11)** | 一定发 STOP |
| 未录制 | 录制成功 | **START (0x10)** | SD 卡正常 |
| 未录制 | 录制失败 | **STOP (0x11)** | SD 异常，防止远端误触 |

命令仅发送到 `role == WAIST` 且 `state == STREAMING` 的设备。

---

## 8. 连接参数优化

连接成功后，RK3576 通过 `hcitool lecup` 收紧 LE 连接参数：

```
hcitool lecup <handle> 6 6 0 100
```

| 参数 | 值 | 说明 |
|------|---|------|
| Min Interval | 6 (7.5ms) | 最小连接间隔 |
| Max Interval | 6 (7.5ms) | 最大连接间隔 |
| Latency | 0 | 从机延迟 |
| Timeout | 100 (1000ms) | 监督超时 |

此操作为 best-effort，失败不影响连接。
