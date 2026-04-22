#!/usr/bin/env python3
"""
按键事件 ZMQ 接收工具

订阅 quad_cam_streamer 的按键事件 ZMQ 端口（默认 5565），
实时打印录制 START/STOP 状态及对应的 CLOCK_MONOTONIC 时间戳。

消息格式（13 字节）：
  [uint64_t timestamp_ns (8B LE)][uint8_t is_recording (1B)][uint32_t seq_num (4B LE)]
  - is_recording: 1 = START（开始录制）, 0 = STOP（停止录制）
  - timestamp_ns: CLOCK_MONOTONIC 纳秒，与 BLE 录制命令使用同一时间戳
  - seq_num: 当前储存文件夹序号（euroc_NNN 中的 NNN）

用法：
    python3 key_event_receiver.py
    python3 key_event_receiver.py --host 192.168.100.1
    python3 key_event_receiver.py --host 192.168.100.1 --port 5565

依赖：
    pip install pyzmq
"""

import argparse
import struct
import sys
import time

import zmq

KEY_EVENT_MSG_SIZE = 13  # 8 (uint64 timestamp_ns) + 1 (uint8 is_recording) + 4 (uint32 seq_num)


def main():
    parser = argparse.ArgumentParser(description="按键事件 ZMQ 接收工具")
    parser.add_argument("--host", default="192.168.100.1",
                        help="板端 IP 地址 (default: 192.168.100.1)")
    parser.add_argument("--port", type=int, default=5565,
                        help="按键事件 ZMQ 端口 (default: 5565)")
    args = parser.parse_args()

    addr = f"tcp://{args.host}:{args.port}"

    ctx = zmq.Context()
    sock = ctx.socket(zmq.SUB)
    sock.setsockopt(zmq.RCVHWM, 10)
    sock.setsockopt(zmq.SUBSCRIBE, b"")
    sock.connect(addr)

    print(f"Listening for key events on {addr}")
    print(f"Message format: [uint64 timestamp_ns][uint8 is_recording][uint32 seq_num]")
    print(f"Waiting for events... (Ctrl+C to quit)\n")
    print(f"{'#':>4}  {'State':<8}  {'Seq':>5}  {'Timestamp (ns)':>20}  {'Timestamp (s)':>16}")
    print(f"{'':->4}  {'':->8}  {'':->5}  {'':->20}  {'':->16}")

    count = 0

    try:
        while True:
            data = sock.recv()

            if len(data) != KEY_EVENT_MSG_SIZE:
                print(f"[WARN] Unexpected message size: {len(data)} bytes "
                      f"(expected {KEY_EVENT_MSG_SIZE}), skipping",
                      file=sys.stderr)
                continue

            ts_ns = struct.unpack("<Q", data[:8])[0]
            is_recording = data[8] != 0
            seq_num = struct.unpack("<I", data[9:13])[0]
            count += 1

            state = "START" if is_recording else "STOP"
            ts_s = ts_ns / 1e9

            print(f"{count:4d}  {state:<8}  {seq_num:>5d}  {ts_ns:>20d}  {ts_s:>16.6f}")

    except KeyboardInterrupt:
        print(f"\nReceived {count} events total.")
    finally:
        sock.close()
        ctx.term()


if __name__ == "__main__":
    main()
