#!/usr/bin/env python3
"""
ZMQ MJPEG + IMU Viewer for quad_cam_streamer

Receives timestamped JPEG frames and IMU data from ZMQ PUB sockets.
Displays cameras in a 2x2 grid with an IMU info panel at the bottom.
Frames are synchronized across cameras using timestamp matching.

Camera message format: [uint64_t timestamp_ns (8B LE)][JPEG data]
IMU message format: [uint64_t timestamp_ns (8B)][double accel[3] (24B)][double gyro[3] (24B)] = 56B

Features:
- Real-time camera parameter control (exposure, gain, JPEG quality)
- Interactive trackbars for parameter adjustment
- Save parameters to config.json on device

Keyboard shortcuts:
    q: Quit
    SPACE: Pause/resume
    s: Save current parameters to config.json
    c: Toggle control panel
    f: Toggle focus detection overlay

Usage:
    python3 zmq_viewer.py
    python3 zmq_viewer.py --host 192.168.1.100
    python3 zmq_viewer.py --cell-width 640

Requirements:
    pip install -r requirements.txt
"""

import argparse
import json
import os
import struct
import time
import sys
import threading
from collections import deque

import cv2
import numpy as np
import zmq


IMU_MSG_SIZE = 56  # 8 + 24 + 24 bytes
IMU_STRUCT_FMT = "<Qdddddd"  # uint64 + 6 doubles
FRAME_BUF_SIZE = 15  # ~500ms at 30fps, enough for network jitter


def load_config():
    """Load config.json from the same directory as this script."""
    script_dir = os.path.dirname(os.path.abspath(__file__))
    config_path = os.path.join(script_dir, "config.json")

    if not os.path.exists(config_path):
        return {}

    try:
        with open(config_path, "r") as f:
            cfg = json.load(f)
        print(f"Loaded config: {config_path}")
        return cfg
    except Exception as e:
        print(f"Warning: failed to load {config_path}: {e}")
        return {}


class ImuReceiver:
    """Background thread that receives IMU data via ZMQ with ring buffer."""

    def __init__(self, host, port, buf_size=500):
        self.host = host
        self.port = port
        self.lock = threading.Lock()
        self.latest = None  # (timestamp_ns, accel[3], gyro[3])
        self.buffer = deque(maxlen=buf_size)
        self.sample_count = 0
        self.rate_hz = 0.0
        self._running = False
        self._thread = None

    def start(self):
        self._running = True
        self._thread = threading.Thread(target=self._run, daemon=True)
        self._thread.start()

    def stop(self):
        self._running = False

    def _run(self):
        ctx = zmq.Context()
        sock = ctx.socket(zmq.SUB)
        sock.setsockopt(zmq.RCVHWM, 200)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        addr = f"tcp://{self.host}:{self.port}"
        sock.connect(addr)
        print(f"  IMU -> {addr}")

        count = 0
        rate_start = time.time()

        poller = zmq.Poller()
        poller.register(sock, zmq.POLLIN)

        while self._running:
            events = dict(poller.poll(timeout=100))
            if sock not in events:
                continue

            try:
                data = sock.recv(zmq.NOBLOCK)
            except zmq.Again:
                continue

            if len(data) != IMU_MSG_SIZE:
                continue

            vals = struct.unpack(IMU_STRUCT_FMT, data)
            ts_ns = vals[0]
            accel = vals[1:4]
            gyro = vals[4:7]

            count += 1
            elapsed = time.time() - rate_start
            if elapsed >= 1.0:
                rate = count / elapsed
                with self.lock:
                    self.rate_hz = rate
                count = 0
                rate_start = time.time()

            with self.lock:
                self.latest = (ts_ns, accel, gyro)
                self.buffer.append((ts_ns, accel, gyro))
                self.sample_count += 1

        sock.close()
        ctx.term()

    def get_latest(self):
        with self.lock:
            return self.latest, self.rate_hz

    def find_closest(self, target_ts_ns):
        """Find the IMU sample with timestamp closest to target_ts_ns."""
        with self.lock:
            if not self.buffer:
                return None
            return min(self.buffer, key=lambda x: abs(x[0] - target_ts_ns))


class ParamController:
    """Camera parameter controller via ZMQ REQ/REP."""

    def __init__(self, host, port=5570):
        self.host = host
        self.port = port
        self.ctx = zmq.Context()
        self.sock = None
        self._connect()

    def _connect(self):
        if self.sock:
            self.sock.close()
        self.sock = self.ctx.socket(zmq.REQ)
        self.sock.setsockopt(zmq.RCVTIMEO, 2000)
        self.sock.connect(f"tcp://{self.host}:{self.port}")

    def query_params(self):
        """Query current parameters from all cameras."""
        try:
            cmd = {"query": True}
            self.sock.send_string(json.dumps(cmd))
            reply = self.sock.recv_string()
            resp = json.loads(reply)
            if resp.get("status") == "ok":
                return resp.get("cameras", [])
            return None
        except zmq.Again:
            self._connect()
            return None
        except Exception as e:
            print(f"Query error: {e}")
            return None

    def send_params(self, cam_idx, auto_exp=None, exposure_us=None, gain=None, quality=None, persist=False):
        """Send parameter update command."""
        cmd = {"cam": cam_idx}
        if auto_exp is not None:
            cmd["auto_exposure"] = auto_exp
        if exposure_us is not None:
            cmd["exposure_us"] = exposure_us
        if gain is not None:
            cmd["analogue_gain"] = gain
        if quality is not None:
            cmd["jpeg_quality"] = quality
        if persist:
            cmd["persist"] = True

        try:
            self.sock.send_string(json.dumps(cmd))
            reply = self.sock.recv_string()
            resp = json.loads(reply)
            return resp.get("status") == "ok", resp.get("message", "")
        except zmq.Again:
            # Timeout, reconnect
            self._connect()
            return False, "Timeout"
        except Exception as e:
            return False, str(e)

    def close(self):
        if self.sock:
            self.sock.close()
        self.ctx.term()


class CameraParams:
    """Per-camera parameter state."""

    def __init__(self, cam_idx, name, auto_exp=True, exposure=10000, gain=16, quality=80):
        self.cam_idx = cam_idx
        self.name = name
        self.auto_exposure = auto_exp
        self.exposure_us = exposure
        self.gain = gain
        self.jpeg_quality = quality


def match_frames(frame_bufs, ref_idx, other_indices):
    """Match frames across cameras by timestamp.

    Uses ref_idx's latest frame as anchor, then for each other camera
    finds the frame with the closest timestamp.

    Returns:
        matched: dict {cam_idx: (ts_ns, frame)} or None if ref has no frames
        max_diff_ms: maximum timestamp difference from reference (ms)
    """
    if not frame_bufs[ref_idx]:
        return None, 0.0

    ref_ts, ref_frame = frame_bufs[ref_idx][-1]  # latest from reference
    matched = {ref_idx: (ref_ts, ref_frame)}
    max_diff = 0

    for idx in other_indices:
        if not frame_bufs[idx]:
            continue
        # Binary-search-like: deque is sorted by time, find closest
        best_ts, best_frame = min(frame_bufs[idx],
                                  key=lambda x: abs(x[0] - ref_ts))
        matched[idx] = (best_ts, best_frame)
        max_diff = max(max_diff, abs(best_ts - ref_ts))

    return matched, max_diff / 1e6  # ms


def trim_bufs(frame_bufs, matched):
    """Remove frames older than the matched set from each camera's buffer."""
    for idx, (ts, _) in matched.items():
        buf = frame_bufs[idx]
        # Keep matched frame and anything newer
        while len(buf) > 1 and buf[0][0] < ts:
            buf.popleft()


def compute_focus_map(frame, grid_size=(8, 6), threshold=100.0):
    """Compute focus quality map using Laplacian variance.

    Args:
        frame: Input BGR image
        grid_size: (cols, rows) for focus detection grid
        threshold: Laplacian variance threshold for "in focus"

    Returns:
        focus_map: 2D array of focus scores (higher = sharper)
        in_focus_mask: Boolean mask where True = in focus
    """
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    h, w = gray.shape
    cols, rows = grid_size
    cell_h = h // rows
    cell_w = w // cols

    focus_map = np.zeros((rows, cols), dtype=np.float32)

    for r in range(rows):
        for c in range(cols):
            y0 = r * cell_h
            y1 = min((r + 1) * cell_h, h)
            x0 = c * cell_w
            x1 = min((c + 1) * cell_w, w)

            roi = gray[y0:y1, x0:x1]
            # Laplacian variance as sharpness metric
            laplacian = cv2.Laplacian(roi, cv2.CV_64F)
            focus_map[r, c] = laplacian.var()

    in_focus_mask = focus_map > threshold
    return focus_map, in_focus_mask


def draw_focus_overlay(frame, focus_map, in_focus_mask, grid_size=(8, 6), alpha=0.3):
    """Draw focus detection overlay on frame.

    Args:
        frame: Input BGR image (will be modified in-place)
        focus_map: 2D array of focus scores
        in_focus_mask: Boolean mask for in-focus regions
        grid_size: (cols, rows) for focus detection grid
        alpha: Overlay transparency (0=transparent, 1=opaque)
    """
    h, w = frame.shape[:2]
    cols, rows = grid_size
    cell_h = h // rows
    cell_w = w // cols

    overlay = frame.copy()

    for r in range(rows):
        for c in range(cols):
            y0 = r * cell_h
            y1 = min((r + 1) * cell_h, h)
            x0 = c * cell_w
            x1 = min((c + 1) * cell_w, w)

            # Color: green if in focus, red if blurry
            if in_focus_mask[r, c]:
                color = (0, 255, 0)  # Green
                thickness = 2
            else:
                color = (0, 0, 255)  # Red
                thickness = 1

            # Draw rectangle
            cv2.rectangle(overlay, (x0, y0), (x1, y1), color, thickness)

            # Draw focus score (normalized to 0-999)
            score = int(min(focus_map[r, c], 999))
            text_color = (0, 255, 0) if in_focus_mask[r, c] else (0, 100, 255)
            cv2.putText(overlay, str(score),
                       (x0 + 5, y0 + 20),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, text_color, 1)

    # Blend overlay with original
    cv2.addWeighted(overlay, alpha, frame, 1 - alpha, 0, frame)


def main():
    cfg = load_config()

    default_host = cfg.get("host", "192.168.100.6")
    default_base_port = cfg.get("base_port", 5550)
    default_cell_width = cfg.get("cell_width", 640)
    default_imu_port = cfg.get("imu_port", 5560)
    cam_configs = cfg.get("cameras", [])

    parser = argparse.ArgumentParser(description="ZMQ MJPEG + IMU Viewer")
    parser.add_argument("--host", default=default_host,
                        help=f"Board IP address (default: {default_host})")
    parser.add_argument("--base-port", type=int, default=default_base_port,
                        help=f"Base ZMQ port (default: {default_base_port})")
    parser.add_argument("--cameras", default=None,
                        help="Comma-separated camera indices (default: all enabled from config)")
    parser.add_argument("--cell-width", type=int, default=default_cell_width,
                        help=f"Width of each cell in the grid (default: {default_cell_width})")
    parser.add_argument("--imu-port", type=int, default=default_imu_port,
                        help=f"IMU ZMQ port (default: {default_imu_port})")
    parser.add_argument("--no-imu", action="store_true",
                        help="Disable IMU display")
    parser.add_argument("--control-port", type=int, default=5570,
                        help="Parameter control port (default: 5570)")
    parser.add_argument("--no-control", action="store_true",
                        help="Disable parameter control panel")
    args = parser.parse_args()

    # Determine camera indices and names from config
    if args.cameras is not None:
        cam_indices = [int(x) for x in args.cameras.split(",")]
    elif cam_configs:
        cam_indices = [c["index"] for c in cam_configs if c.get("enabled", True)]
    else:
        cam_indices = [0, 1, 2, 3]

    # Build camera info: index -> name
    cam_names = {}
    cfg_by_index = {c["index"]: c for c in cam_configs if "index" in c}
    for idx in cam_indices:
        if idx in cfg_by_index:
            cam_names[idx] = cfg_by_index[idx].get("name", f"cam{idx}")
        else:
            cam_names[idx] = f"cam{idx}"

    # Layout: top row = IMX334 cameras, bottom row = OV9281 cameras
    top_row = []
    bottom_row = []
    for idx in cam_indices:
        name = cam_names[idx].lower()
        if "ov" in name:
            bottom_row.append(idx)
        else:
            top_row.append(idx)

    grid_layout = top_row + bottom_row

    cols = 2
    rows = (len(grid_layout) + cols - 1) // cols

    # Cell size: all cells same dimensions
    cell_w = args.cell_width
    cell_h = int(cell_w * 9 / 16)  # 16:9 aspect ratio

    show_imu = not args.no_imu
    imu_panel_h = 60 if show_imu else 0

    win_w = cell_w * cols
    win_h = cell_h * rows + imu_panel_h

    print(f"Host: {args.host}, base_port: {args.base_port}")
    print(f"Grid: {cols}x{rows}, cell: {cell_w}x{cell_h}, window: {win_w}x{win_h}")
    print(f"Sync: timestamp matching enabled (buf={FRAME_BUF_SIZE})")

    # Initialize parameter controller
    param_ctrl = None
    cam_params = {}
    show_control = not args.no_control
    control_window = "Camera Controls"

    if show_control:
        try:
            param_ctrl = ParamController(args.host, args.control_port)
            print(f"Parameter control enabled on port {args.control_port}")

            # Query current parameters from device
            print("Querying current parameters from device...")
            device_params = param_ctrl.query_params()

            # Build device params lookup
            device_params_map = {}
            if device_params:
                for dp in device_params:
                    device_params_map[dp["index"]] = dp
                    print(f"  cam{dp['index']}: exp={dp['exposure_us']}us gain={dp['analogue_gain']} q={dp['jpeg_quality']} auto={dp['auto_exposure']}")
            else:
                print("  Warning: Could not query device parameters, using defaults")

            # Initialize camera parameters with device values
            for idx in cam_indices:
                if idx in device_params_map:
                    dp = device_params_map[idx]
                    cam_params[idx] = CameraParams(
                        idx, cam_names[idx],
                        auto_exp=dp["auto_exposure"],
                        exposure=dp["exposure_us"],
                        gain=dp["analogue_gain"],
                        quality=dp["jpeg_quality"]
                    )
                else:
                    # Fallback to defaults
                    cam_params[idx] = CameraParams(idx, cam_names[idx])

            # Create control window with trackbars for first camera
            cv2.namedWindow(control_window, cv2.WINDOW_NORMAL)
            cv2.resizeWindow(control_window, 800, 400)
            selected_cam = cam_indices[0]

            def update_control_display():
                """Update control window display with current parameters."""
                ctrl_img = np.zeros((400, 800, 3), dtype=np.uint8)
                cv2.rectangle(ctrl_img, (0, 0), (800, 60), (60, 60, 60), -1)
                cv2.putText(ctrl_img, "Camera Parameter Controls", (20, 40),
                           cv2.FONT_HERSHEY_DUPLEX, 1.0, (255, 255, 255), 2)
                p = cam_params[selected_cam]
                cv2.rectangle(ctrl_img, (10, 70), (790, 130), (40, 40, 40), -1)
                cam_text = f"Selected: {cam_names[selected_cam]} (Camera {selected_cam})"
                cv2.putText(ctrl_img, cam_text, (25, 100),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (100, 200, 255), 2)
                mode_text = "AUTO" if p.auto_exposure else "MANUAL"
                mode_color = (100, 255, 100) if p.auto_exposure else (100, 150, 255)
                cv2.putText(ctrl_img, f"Mode: {mode_text}", (500, 100),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, mode_color, 2)
                y_offset = 160
                params_text = [
                    f"Exposure: {p.exposure_us} us",
                    f"Gain: {p.gain}",
                    f"JPEG Quality: {p.jpeg_quality}"
                ]
                for i, text in enumerate(params_text):
                    cv2.putText(ctrl_img, text, (25, y_offset + i * 40),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
                cv2.imshow(control_window, ctrl_img)

            def on_cam_select(val):
                nonlocal selected_cam
                selected_cam = cam_indices[val]
                p = cam_params[selected_cam]
                cv2.setTrackbarPos("Auto Exp", control_window, 1 if p.auto_exposure else 0)
                # Clamp exposure to trackbar range
                cv2.setTrackbarPos("Exposure", control_window, min(p.exposure_us, 5000))
                cv2.setTrackbarPos("Gain", control_window, p.gain)
                cv2.setTrackbarPos("Quality", control_window, p.jpeg_quality)
                update_control_display()

            def on_auto_exp(val):
                p = cam_params[selected_cam]
                p.auto_exposure = bool(val)
                param_ctrl.send_params(selected_cam, auto_exp=p.auto_exposure)
                update_control_display()

            def on_exposure(val):
                if val < 1:
                    val = 1
                p = cam_params[selected_cam]
                p.exposure_us = val
                if not p.auto_exposure:
                    param_ctrl.send_params(selected_cam, exposure_us=val)
                update_control_display()

            def on_gain(val):
                if val < 1:
                    val = 1
                p = cam_params[selected_cam]
                p.gain = val
                if not p.auto_exposure:
                    param_ctrl.send_params(selected_cam, gain=val)
                update_control_display()

            def on_quality(val):
                if val < 1:
                    val = 1
                p = cam_params[selected_cam]
                p.jpeg_quality = val
                param_ctrl.send_params(selected_cam, quality=val)
                update_control_display()

            cv2.createTrackbar("Camera", control_window, 0, len(cam_indices) - 1, on_cam_select)
            cv2.createTrackbar("Auto Exp", control_window, 1 if cam_params[selected_cam].auto_exposure else 0, 1, on_auto_exp)
            cv2.createTrackbar("Exposure", control_window, min(cam_params[selected_cam].exposure_us, 5000), 5000, on_exposure)
            cv2.createTrackbar("Gain", control_window, cam_params[selected_cam].gain, 255, on_gain)
            cv2.createTrackbar("Quality", control_window, cam_params[selected_cam].jpeg_quality, 100, on_quality)

            # Display initial control panel
            update_control_display()

        except Exception as e:
            print(f"Warning: parameter control disabled: {e}")
            show_control = False

    # Start IMU receiver
    imu_recv = None
    if show_imu:
        imu_recv = ImuReceiver(args.host, args.imu_port)
        imu_recv.start()

    # Connect camera ZMQ sockets (no CONFLATE - we need frame buffering)
    context = zmq.Context()
    sockets = {}
    poller = zmq.Poller()

    for idx in cam_indices:
        port = args.base_port + idx
        addr = f"tcp://{args.host}:{port}"
        sock = context.socket(zmq.SUB)
        sock.setsockopt(zmq.RCVHWM, FRAME_BUF_SIZE)
        sock.setsockopt(zmq.SUBSCRIBE, b"")
        sock.connect(addr)
        sockets[idx] = sock
        poller.register(sock, zmq.POLLIN)
        print(f"  {cam_names[idx]} (cam{idx}) -> {addr}")

    # Per-camera frame ring buffers: deque of (ts_ns, frame)
    frame_bufs = {idx: deque(maxlen=FRAME_BUF_SIZE) for idx in cam_indices}
    frame_counts = {idx: 0 for idx in cam_indices}
    fps_start = time.time()

    # Current matched display set
    disp_frames = {}   # {cam_idx: frame}
    disp_cam_ts = {}   # {cam_idx: ts_ns}
    disp_imu = None    # (ts_ns, accel, gyro) or None
    match_diff_ms = 0.0  # max timestamp diff in matched set

    paused = False
    ref_idx = grid_layout[0]
    other_indices = grid_layout[1:]

    window_name = "Quad Camera Viewer"
    print(f"\nKeyboard shortcuts:")
    print(f"  q: Quit")
    print(f"  SPACE: Pause/resume")
    if show_control:
        print(f"  s: Save parameters to config.json")
        print(f"  c: Toggle control panel")
        print(f"  e: Manual input exposure (us)")
        print(f"  g: Manual input gain")
        print(f"  j: Manual input JPEG quality")
        print(f"  f: Toggle focus detection overlay")
        print(f"  t: Adjust focus threshold")
    print(f"Reference camera: {cam_names[ref_idx]} (cam{ref_idx})")

    control_visible = show_control

    # Focus detection state
    focus_enabled = False
    focus_threshold = 100.0  # Laplacian variance threshold
    focus_grid_size = (8, 6)  # (cols, rows)

    def manual_input_exposure():
        """Prompt user for manual exposure input."""
        p = cam_params[selected_cam]
        print(f"\n[Manual Input] Camera: {cam_names[selected_cam]}")
        print(f"Current exposure: {p.exposure_us}us")
        try:
            val = int(input("Enter new exposure (1-33000us): "))
            if val < 1 or val > 33000:
                print(f"✗ Invalid range. Must be 1-33000")
                return
            p.exposure_us = val
            # Update trackbar if within range
            if val <= 5000:
                cv2.setTrackbarPos("Exposure", control_window, val)
            if not p.auto_exposure:
                ok, msg = param_ctrl.send_params(selected_cam, exposure_us=val)
                if ok:
                    print(f"✓ Exposure set to {val}us")
                else:
                    print(f"✗ Failed: {msg}")
            update_control_display()
        except (ValueError, EOFError):
            print("✗ Invalid input")

    def manual_input_gain():
        """Prompt user for manual gain input."""
        p = cam_params[selected_cam]
        print(f"\n[Manual Input] Camera: {cam_names[selected_cam]}")
        print(f"Current gain: {p.gain}")
        try:
            val = int(input("Enter new gain (1-255): "))
            if val < 1 or val > 255:
                print(f"✗ Invalid range. Must be 1-255")
                return
            p.gain = val
            cv2.setTrackbarPos("Gain", control_window, val)
            if not p.auto_exposure:
                ok, msg = param_ctrl.send_params(selected_cam, gain=val)
                if ok:
                    print(f"✓ Gain set to {val}")
                else:
                    print(f"✗ Failed: {msg}")
            update_control_display()
        except (ValueError, EOFError):
            print("✗ Invalid input")

    def manual_input_quality():
        """Prompt user for manual JPEG quality input."""
        p = cam_params[selected_cam]
        print(f"\n[Manual Input] Camera: {cam_names[selected_cam]}")
        print(f"Current JPEG quality: {p.jpeg_quality}")
        try:
            val = int(input("Enter new quality (1-100): "))
            if val < 1 or val > 100:
                print(f"✗ Invalid range. Must be 1-100")
                return
            p.jpeg_quality = val
            cv2.setTrackbarPos("Quality", control_window, val)
            ok, msg = param_ctrl.send_params(selected_cam, quality=val)
            if ok:
                print(f"✓ Quality set to {val}")
            else:
                print(f"✗ Failed: {msg}")
            update_control_display()
        except (ValueError, EOFError):
            print("✗ Invalid input")


    try:
        while True:
            events = dict(poller.poll(timeout=100))

            # Drain all available messages, but only decode the latest frame per
            # camera this iteration. Decoding every queued JPEG cannot keep up
            # once the aggregate stream rate gets high enough.
            for idx, sock in sockets.items():
                if sock not in events:
                    continue
                latest_data = None
                while True:
                    try:
                        data = sock.recv(zmq.NOBLOCK)
                    except zmq.Again:
                        break
                    latest_data = data
                if latest_data is None or len(latest_data) <= 8:
                    continue
                ts_ns = struct.unpack("<Q", latest_data[:8])[0]
                jpeg_data = latest_data[8:]
                arr = np.frombuffer(jpeg_data, dtype=np.uint8)
                frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                if frame is not None:
                    if not paused:
                        frame_bufs[idx].append((ts_ns, frame))
                    frame_counts[idx] += 1

            # Run timestamp matching (only when not paused)
            if not paused:
                matched, diff_ms = match_frames(frame_bufs, ref_idx,
                                                other_indices)
                if matched:
                    disp_frames = {idx: f for idx, (_, f) in matched.items()}
                    disp_cam_ts = {idx: ts for idx, (ts, _) in matched.items()}
                    match_diff_ms = diff_ms
                    # Find IMU sample closest to reference camera timestamp
                    if imu_recv and ref_idx in disp_cam_ts:
                        disp_imu = imu_recv.find_closest(disp_cam_ts[ref_idx])
                    # Trim old frames to prevent memory growth
                    trim_bufs(frame_bufs, matched)

            # Build grid image
            grid = np.zeros((win_h, win_w, 3), dtype=np.uint8)

            for i, idx in enumerate(grid_layout):
                r, c = divmod(i, cols)
                x0 = c * cell_w
                y0 = r * cell_h

                if idx in disp_frames:
                    # Resize maintaining aspect ratio, center in cell
                    fh, fw = disp_frames[idx].shape[:2]
                    scale = min(cell_w / fw, cell_h / fh)
                    new_w = int(fw * scale)
                    new_h = int(fh * scale)
                    resized = cv2.resize(disp_frames[idx], (new_w, new_h))

                    # Apply focus detection overlay if enabled
                    if focus_enabled:
                        focus_map, in_focus_mask = compute_focus_map(
                            resized, focus_grid_size, focus_threshold
                        )
                        draw_focus_overlay(resized, focus_map, in_focus_mask, focus_grid_size)

                    pad_x = (cell_w - new_w) // 2
                    pad_y = (cell_h - new_h) // 2
                    grid[y0 + pad_y:y0 + pad_y + new_h,
                         x0 + pad_x:x0 + pad_x + new_w] = resized

                # Draw camera label (highlight reference)
                label = cam_names.get(idx, f"cam{idx}")
                if idx == ref_idx:
                    label += " [ref]"
                cv2.putText(grid, label, (x0 + 10, y0 + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

                # Draw timestamp + diff from reference
                if idx in disp_cam_ts:
                    ts_ns = disp_cam_ts[idx]
                    ts_str = f"ts: {ts_ns/1e9:.3f}s"
                    if idx != ref_idx and ref_idx in disp_cam_ts:
                        diff_ms = (ts_ns - disp_cam_ts[ref_idx]) / 1e6
                        ts_str += f"  (d={diff_ms:+.2f}ms)"
                    cv2.putText(grid, ts_str, (x0 + 10, y0 + 60),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55,
                                (200, 200, 0), 1)

                # Draw FPS
                elapsed = time.time() - fps_start
                if elapsed > 0:
                    fps_val = frame_counts[idx] / elapsed
                    cv2.putText(grid, f"{fps_val:.1f} fps",
                                (x0 + 10, y0 + cell_h - 15),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 200, 200), 1)

            # Draw grid lines
            for c in range(1, cols):
                cv2.line(grid, (c * cell_w, 0), (c * cell_w, win_h - imu_panel_h), (80, 80, 80), 1)
            for r in range(1, rows):
                cv2.line(grid, (0, r * cell_h), (win_w, r * cell_h), (80, 80, 80), 1)

            # Draw IMU panel
            if show_imu and imu_recv:
                imu_y0 = cell_h * rows
                cv2.line(grid, (0, imu_y0), (win_w, imu_y0), (80, 80, 80), 1)

                imu_show = disp_imu
                _, imu_rate = imu_recv.get_latest()

                if imu_show is not None:
                    ts_ns, accel, gyro = imu_show
                    imu_ts_str = f"ts:{ts_ns/1e9:.3f}s"
                    accel_str = f"Accel [{accel[0]:+7.2f}, {accel[1]:+7.2f}, {accel[2]:+7.2f}] m/s2"
                    gyro_str = f"Gyro  [{gyro[0]:+7.3f}, {gyro[1]:+7.3f}, {gyro[2]:+7.3f}] rad/s"
                    rate_str = f"{imu_rate:.0f} Hz"

                    cv2.putText(grid, f"IMU: {accel_str}   {rate_str}   {imu_ts_str}",
                                (10, imu_y0 + 25),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 200), 1)
                    cv2.putText(grid, f"     {gyro_str}",
                                (10, imu_y0 + 50),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 200, 200), 1)
                else:
                    cv2.putText(grid, "IMU: waiting for data...",
                                (10, imu_y0 + 35),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.55, (100, 100, 100), 1)

            # Draw sync status at top-right
            if len(disp_cam_ts) > 1:
                sync_color = (0, 255, 0) if match_diff_ms < 2.0 else (0, 180, 255)
                sync_text = f"sync spread: {match_diff_ms:.2f}ms"
                text_size = cv2.getTextSize(sync_text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 1)[0]
                cv2.putText(grid, sync_text,
                            (win_w - text_size[0] - 10, 25),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, sync_color, 1)

            # Draw focus detection status
            if focus_enabled:
                focus_text = f"Focus Detection: ON (threshold={focus_threshold:.0f})"
                cv2.putText(grid, focus_text, (10, win_h - imu_panel_h - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 255), 1)

            # Draw PAUSED indicator
            if paused:
                pause_text = "PAUSED (SPACE to resume)"
                text_size = cv2.getTextSize(pause_text, cv2.FONT_HERSHEY_SIMPLEX, 1.0, 2)[0]
                tx = (win_w - text_size[0]) // 2
                ty = win_h // 2
                cv2.rectangle(grid, (tx - 10, ty - text_size[1] - 10),
                              (tx + text_size[0] + 10, ty + 10), (0, 0, 0), -1)
                cv2.rectangle(grid, (tx - 10, ty - text_size[1] - 10),
                              (tx + text_size[0] + 10, ty + 10), (0, 0, 255), 2)
                cv2.putText(grid, pause_text, (tx, ty),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)

            cv2.imshow(window_name, grid)

            # Console FPS logging every 5 seconds
            elapsed = time.time() - fps_start
            if elapsed >= 5.0:
                fps_str = ", ".join(
                    f"{cam_names[idx]}: {frame_counts[idx]/elapsed:.1f}fps"
                    for idx in cam_indices
                )
                imu_info = ""
                if show_imu and imu_recv:
                    _, rate = imu_recv.get_latest()
                    imu_info = f", IMU: {rate:.0f}Hz"
                sync_info = f", sync:{match_diff_ms:.2f}ms"
                print(f"FPS: {fps_str}{imu_info}{sync_info}")
                for idx in cam_indices:
                    frame_counts[idx] = 0
                fps_start = time.time()

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            elif key == ord(" "):
                paused = not paused
                if paused:
                    print(">>> PAUSED - matched timestamps:")
                    for idx in grid_layout:
                        if idx in disp_cam_ts:
                            ts = disp_cam_ts[idx]
                            diff = ""
                            if idx != ref_idx and ref_idx in disp_cam_ts:
                                d = (ts - disp_cam_ts[ref_idx]) / 1e6
                                diff = f"  d={d:+.3f}ms"
                            print(f"    {cam_names[idx]}: {ts} ({ts/1e9:.6f}s){diff}")
                    if disp_imu:
                        print(f"    IMU: {disp_imu[0]} ({disp_imu[0]/1e9:.6f}s)")
                    print(f"    max spread: {match_diff_ms:.3f}ms")
                else:
                    print(">>> RESUMED")
            elif key == ord("s") and show_control and param_ctrl:
                # Save current parameters to config.json
                print("Saving parameters to config.json...")
                for idx in cam_indices:
                    p = cam_params[idx]
                    ok, msg = param_ctrl.send_params(
                        idx,
                        auto_exp=p.auto_exposure,
                        exposure_us=p.exposure_us,
                        gain=p.gain,
                        quality=p.jpeg_quality,
                        persist=True
                    )
                    if ok:
                        print(f"  ✓ {cam_names[idx]} saved")
                    else:
                        print(f"  ✗ {cam_names[idx]} failed: {msg}")
            elif key == ord("c") and show_control:
                # Toggle control panel visibility
                control_visible = not control_visible
                if control_visible:
                    cv2.namedWindow(control_window)
                    update_control_display()
                else:
                    cv2.destroyWindow(control_window)
            elif key == ord("e") and show_control and param_ctrl:
                # Manual exposure input
                manual_input_exposure()
            elif key == ord("g") and show_control and param_ctrl:
                # Manual gain input
                manual_input_gain()
            elif key == ord("j") and show_control and param_ctrl:
                # Manual JPEG quality input
                manual_input_quality()
            elif key == ord("f"):
                # Toggle focus detection
                focus_enabled = not focus_enabled
                status = "ON" if focus_enabled else "OFF"
                print(f">>> Focus detection: {status} (threshold={focus_threshold:.0f})")
            elif key == ord("t"):
                # Adjust focus threshold
                print(f"\n[Focus Threshold] Current: {focus_threshold:.0f}")
                print("Typical ranges: 50-150 (low light), 100-300 (normal), 200-500 (bright)")
                try:
                    val = float(input("Enter new threshold (10-1000): "))
                    if val < 10 or val > 1000:
                        print(f"✗ Invalid range. Must be 10-1000")
                    else:
                        focus_threshold = val
                        print(f"✓ Focus threshold set to {focus_threshold:.0f}")
                except (ValueError, EOFError):
                    print("✗ Invalid input")

    except KeyboardInterrupt:
        pass
    finally:
        if imu_recv:
            imu_recv.stop()
        if param_ctrl:
            param_ctrl.close()
        cv2.destroyAllWindows()
        for sock in sockets.values():
            sock.close()
        context.term()


if __name__ == "__main__":
    main()
