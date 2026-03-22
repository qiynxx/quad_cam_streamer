#!/usr/bin/env python3
"""
Quad Camera Calibration Tool
=============================

Calibrates 4 cameras (2x IMX334 + 2x OV9281) using ZMQ MJPEG streams.
Outputs EuRoC format sensor.yaml per camera with resolution-independent
normalized intrinsics.

Camera message format: [uint64_t timestamp_ns (8B LE)][JPEG data]

Modes:
    python quad_calibrate.py capture   - Capture calibration images via ZMQ
    python quad_calibrate.py calibrate - Calibrate from saved images
    python quad_calibrate.py           - Capture then calibrate interactively

Requirements:
    pip install -r requirements.txt
"""

import argparse
import json
import os
import struct
import sys
import time
from datetime import datetime
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml
import zmq


# ---------------------------------------------------------------------------
# Supported distortion models
# ---------------------------------------------------------------------------

DISTORTION_MODELS = {
    "radtan": {
        "cv_flags": cv2.CALIB_FIX_K3,
        "num_coeffs": 4,          # [k1, k2, p1, p2]
        "euroc_camera": "pinhole",
        "euroc_distortion": "radial-tangential",
        "subpix_window": (11, 11),
    },
    "rational": {
        "cv_flags": cv2.CALIB_RATIONAL_MODEL,
        "num_coeffs": 8,          # [k1, k2, p1, p2, k3, k4, k5, k6]
        "euroc_camera": "pinhole",
        "euroc_distortion": "rational",
        "subpix_window": (5, 5),
    },
    "fisheye": {
        "cv_flags": cv2.fisheye.CALIB_RECOMPUTE_EXTRINSIC,
        "num_coeffs": 4,          # [k1, k2, k3, k4] (KB4 / equidistant)
        "euroc_camera": "pinhole",  # still pinhole projection for EuRoC
        "euroc_distortion": "equidistant",
        "subpix_window": (5, 5),
    },
}


# ---------------------------------------------------------------------------
# BoardSpec
# ---------------------------------------------------------------------------

class BoardSpec:
    """Chessboard calibration target specification."""

    def __init__(self, cols: int, rows: int, square_size_mm: float):
        self.cols = cols
        self.rows = rows
        self.square_size_mm = square_size_mm
        self.pattern_size = (cols, rows)

    def object_points(self) -> np.ndarray:
        """Generate 3D object points in mm."""
        objp = np.zeros((self.cols * self.rows, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        objp *= self.square_size_mm
        return objp


# ---------------------------------------------------------------------------
# Chessboard detection
# ---------------------------------------------------------------------------

_CHESS_FLAGS = (cv2.CALIB_CB_ADAPTIVE_THRESH
                | cv2.CALIB_CB_NORMALIZE_IMAGE
                | cv2.CALIB_CB_FAST_CHECK)

_SUBPIX_CRITERIA = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
                    30, 0.001)


def detect_chessboard(
    img: np.ndarray,
    pattern_size: Tuple[int, int],
    max_width: int = 0,
    subpix_window: Tuple[int, int] = (11, 11),
) -> Tuple[bool, Optional[np.ndarray]]:
    """Detect chessboard corners with sub-pixel refinement.

    Args:
        img: Input image (BGR or grayscale).
        pattern_size: (cols, rows) inner corners.
        max_width: If > 0, downscale to this width before detection to speed
                   up findChessboardCorners. Corners are mapped back to
                   original resolution. Set to 0 to detect at full resolution.
        subpix_window: Half-size of the search window for cornerSubPix.
                       Smaller values (5,5) are better for high-distortion lenses.

    Returns (found, corners) where corners is Nx1x2 float32 or None.
    """
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY) if len(img.shape) == 3 else img
    h, w = gray.shape[:2]

    # Optionally downscale for faster detection
    if max_width > 0 and w > max_width:
        scale = max_width / w
        small = cv2.resize(gray, (max_width, int(h * scale)),
                           interpolation=cv2.INTER_AREA)
    else:
        scale = 1.0
        small = gray

    found, corners = cv2.findChessboardCorners(small, pattern_size,
                                               _CHESS_FLAGS)
    if found:
        if scale != 1.0:
            # Map corners back to original resolution
            corners = corners / scale
        # Sub-pixel refinement on full-resolution image
        corners = cv2.cornerSubPix(gray, corners, subpix_window, (-1, -1),
                                   _SUBPIX_CRITERIA)
    return found, corners


def draw_detection(img: np.ndarray, pattern_size: Tuple[int, int],
                   found: bool, corners: Optional[np.ndarray]) -> np.ndarray:
    """Draw chessboard detection overlay on a copy of the image."""
    vis = img.copy()
    if found and corners is not None:
        cv2.drawChessboardCorners(vis, pattern_size, corners, found)
    return vis


# ---------------------------------------------------------------------------
# QuadReceiver - ZMQ subscriber for 4 cameras
# ---------------------------------------------------------------------------

class QuadReceiver:
    """Receives JPEG frames from N ZMQ PUB sockets (CONFLATE=1)."""

    def __init__(self, host: str, base_port: int, cam_indices: List[int]):
        self.host = host
        self.base_port = base_port
        self.cam_indices = cam_indices

        self._ctx = zmq.Context()
        self._sockets: Dict[int, zmq.Socket] = {}
        self._poller = zmq.Poller()

        for idx in cam_indices:
            port = base_port + idx
            addr = f"tcp://{host}:{port}"
            sock = self._ctx.socket(zmq.SUB)
            sock.setsockopt(zmq.RCVHWM, 2)
            sock.setsockopt(zmq.SUBSCRIBE, b"")
            sock.setsockopt(zmq.CONFLATE, 1)
            sock.connect(addr)
            self._sockets[idx] = sock
            self._poller.register(sock, zmq.POLLIN)
            print(f"  cam{idx} -> {addr}")

    def poll(self, timeout_ms: int = 100) -> Dict[int, np.ndarray]:
        """Poll all sockets and return {cam_index: bgr_frame} for new data."""
        events = dict(self._poller.poll(timeout=timeout_ms))
        frames: Dict[int, np.ndarray] = {}
        for idx, sock in self._sockets.items():
            if sock not in events:
                continue
            try:
                data = sock.recv(zmq.NOBLOCK)
            except zmq.Again:
                continue
            if len(data) <= 8:
                continue
            jpeg_data = data[8:]  # skip 8-byte timestamp
            arr = np.frombuffer(jpeg_data, dtype=np.uint8)
            frame = cv2.imdecode(arr, cv2.IMREAD_COLOR)
            if frame is not None:
                frames[idx] = frame
        return frames

    def close(self):
        for sock in self._sockets.values():
            sock.close()
        self._ctx.term()


# ---------------------------------------------------------------------------
# Capture mode
# ---------------------------------------------------------------------------

def run_capture_mode(cfg: dict, args) -> str:
    """Live 2x2 preview with chessboard detection; SPACE saves all 4 frames.

    Returns the capture directory path.
    """
    host = args.host or cfg.get("host", "192.168.100.1")
    base_port = args.base_port or cfg.get("base_port", 5550)
    cam_cfgs = cfg.get("cameras", [])
    board_cfg = cfg.get("board", {})
    capture_dir = args.capture_dir or cfg.get("capture_dir", "calib_images")

    cam_indices = [c["index"] for c in cam_cfgs if c.get("enabled", True)]
    cam_names = {c["index"]: c.get("name", f"cam{c['index']}") for c in cam_cfgs}

    board = BoardSpec(
        cols=board_cfg.get("cols", 9),
        rows=board_cfg.get("rows", 6),
        square_size_mm=board_cfg.get("square_size_mm", 25.0),
    )

    # Create per-camera capture dirs
    for idx in cam_indices:
        os.makedirs(os.path.join(capture_dir, f"cam{idx}_{cam_names.get(idx, '')}"),
                    exist_ok=True)

    print(f"\nConnecting to {host}, base_port={base_port} ...")
    receiver = QuadReceiver(host, base_port, cam_indices)

    cell_w = args.cell_width
    cell_h = int(cell_w * 9 / 16)
    cols, rows_grid = 2, 2
    win_w, win_h = cell_w * cols, cell_h * rows_grid

    print(f"\nCapture preview: {win_w}x{win_h}  (cell {cell_w}x{cell_h})")
    print("Controls:")
    print("  SPACE - Save current frames from all cameras")
    print("  Q     - Quit capture mode")
    print()

    latest: Dict[int, np.ndarray] = {}
    detect_cache: Dict[int, Tuple[bool, Optional[np.ndarray]]] = {}
    frame_idx = 0
    detect_max_width = 640  # downscale for fast detection
    window_name = "Quad Calibration - Capture"

    try:
        while True:
            new_frames = receiver.poll(timeout_ms=50)
            latest.update(new_frames)

            # Build 2x2 grid
            grid = np.zeros((win_h, win_w, 3), dtype=np.uint8)
            for i, idx in enumerate(cam_indices[:4]):
                r, c = divmod(i, cols)
                x0 = c * cell_w
                y0 = r * cell_h

                if idx in latest:
                    fh, fw = latest[idx].shape[:2]
                    scale = min(cell_w / fw, cell_h / fh)
                    nw, nh = int(fw * scale), int(fh * scale)

                    # Only re-detect when we got a NEW frame for this camera
                    if idx in new_frames:
                        found, corners = detect_chessboard(
                            latest[idx], board.pattern_size,
                            max_width=detect_max_width)
                        detect_cache[idx] = (found, corners)
                    else:
                        found, corners = detect_cache.get(idx, (False, None))

                    vis = draw_detection(latest[idx], board.pattern_size,
                                         found, corners)
                    resized = cv2.resize(vis, (nw, nh))

                    px = (cell_w - nw) // 2
                    py = (cell_h - nh) // 2
                    grid[y0 + py:y0 + py + nh,
                         x0 + px:x0 + px + nw] = resized

                    # Status indicator
                    color = (0, 255, 0) if found else (0, 0, 255)
                    cv2.circle(grid, (x0 + cell_w - 20, y0 + 20), 8,
                               color, -1)

                name = cam_names.get(idx, f"cam{idx}")
                cv2.putText(grid, name, (x0 + 10, y0 + 30),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)

            # Grid lines
            cv2.line(grid, (cell_w, 0), (cell_w, win_h), (80, 80, 80), 1)
            cv2.line(grid, (0, cell_h), (win_w, cell_h), (80, 80, 80), 1)

            # Frame count
            cv2.putText(grid, f"Saved: {frame_idx}",
                        (win_w - 180, win_h - 15),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

            cv2.imshow(window_name, grid)
            key = cv2.waitKey(1) & 0xFF

            if key == ord(" "):
                # Save all current frames
                saved_any = False
                for idx in cam_indices:
                    if idx not in latest:
                        continue
                    name = cam_names.get(idx, f"cam{idx}")
                    cam_dir = os.path.join(capture_dir,
                                           f"cam{idx}_{name}")
                    fname = f"frame_{frame_idx:04d}.png"
                    cv2.imwrite(os.path.join(cam_dir, fname), latest[idx])
                    saved_any = True
                if saved_any:
                    print(f"  Saved frame set {frame_idx}")
                    frame_idx += 1
                else:
                    print("  No frames available to save")

            elif key == ord("q") or key == ord("Q"):
                break

    except KeyboardInterrupt:
        pass
    finally:
        cv2.destroyAllWindows()
        receiver.close()

    # Save capture metadata
    meta = {
        "num_frame_sets": frame_idx,
        "cameras": [{"index": idx, "name": cam_names.get(idx, "")}
                     for idx in cam_indices],
        "board": {
            "type": board_cfg.get("type", "chessboard"),
            "cols": board.cols,
            "rows": board.rows,
            "square_size_mm": board.square_size_mm,
        },
        "capture_date": datetime.now().isoformat(),
    }
    with open(os.path.join(capture_dir, "capture_meta.json"), "w") as f:
        json.dump(meta, f, indent=2)

    print(f"\nCapture complete: {frame_idx} frame sets in {capture_dir}/")
    return capture_dir


# ---------------------------------------------------------------------------
# Intrinsic calibration (per camera)
# ---------------------------------------------------------------------------

def calibrate_intrinsics(
    image_dir: str,
    board: BoardSpec,
    cam_idx: int,
    cam_name: str,
    distortion_model: str = "radtan",
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray],
           Optional[Tuple[int, int]], float, List[int]]:
    """Calibrate a single camera's intrinsics.

    Returns (K, D, image_size, rms, used_frame_indices) or Nones on failure.
    K is 3x3, D shape depends on distortion_model.
    """
    model_info = DISTORTION_MODELS[distortion_model]
    subpix_win = model_info["subpix_window"]

    cam_dir = os.path.join(image_dir, f"cam{cam_idx}_{cam_name}")
    if not os.path.isdir(cam_dir):
        print(f"  [cam{cam_idx}] Directory not found: {cam_dir}")
        return None, None, None, -1.0, []

    files = sorted(f for f in os.listdir(cam_dir)
                   if f.startswith("frame_") and f.endswith(".png"))
    if not files:
        print(f"  [cam{cam_idx}] No frame images found in {cam_dir}")
        return None, None, None, -1.0, []

    objp = board.object_points()
    obj_pts_list: List[np.ndarray] = []
    img_pts_list: List[np.ndarray] = []
    used_indices: List[int] = []
    image_size: Optional[Tuple[int, int]] = None

    for fname in files:
        # Extract index from frame_XXXX.png
        idx_str = fname.replace("frame_", "").replace(".png", "")
        try:
            fidx = int(idx_str)
        except ValueError:
            continue

        img = cv2.imread(os.path.join(cam_dir, fname))
        if img is None:
            continue
        if image_size is None:
            h, w = img.shape[:2]
            image_size = (w, h)

        found, corners = detect_chessboard(img, board.pattern_size,
                                           subpix_window=subpix_win)
        if found:
            obj_pts_list.append(objp)
            img_pts_list.append(corners)
            used_indices.append(fidx)

    print(f"  [cam{cam_idx}] Detected board in {len(used_indices)}/{len(files)} images")

    if len(obj_pts_list) < 5:
        print(f"  [cam{cam_idx}] Not enough detections (need >= 5)")
        return None, None, image_size, -1.0, used_indices

    if distortion_model == "fisheye":
        # cv2.fisheye requires (1, N, 3) float64 and (1, N, 2) float64
        obj_pts_fish = [o.reshape(1, -1, 3).astype(np.float64)
                        for o in obj_pts_list]
        img_pts_fish = [p.reshape(1, -1, 2).astype(np.float64)
                        for p in img_pts_list]
        K = np.eye(3, dtype=np.float64)
        D = np.zeros((4, 1), dtype=np.float64)
        flags = model_info["cv_flags"]
        rms, K, D, rvecs, tvecs = cv2.fisheye.calibrate(
            obj_pts_fish, img_pts_fish, image_size, K, D, flags=flags)
        D = D.flatten()[:4].reshape(1, 4)
    else:
        # Pinhole models: radtan or rational
        flags = model_info["cv_flags"]
        rms, K, D, rvecs, tvecs = cv2.calibrateCamera(
            obj_pts_list, img_pts_list, image_size, None, None, flags=flags)
        n = model_info["num_coeffs"]
        D = D.flatten()[:n].reshape(1, n)

    print(f"  [cam{cam_idx}] model={distortion_model} "
          f"Intrinsic RMS = {rms:.4f}  "
          f"fx={K[0,0]:.1f} fy={K[1,1]:.1f} "
          f"cx={K[0,2]:.1f} cy={K[1,2]:.1f}  "
          f"D=[{', '.join(f'{x:.4f}' for x in D.flatten())}]")
    return K, D, image_size, rms, used_indices


# ---------------------------------------------------------------------------
# Extrinsic calibration (cam0 <-> camN)
# ---------------------------------------------------------------------------

def _is_pinhole(model: str) -> bool:
    """Return True if the model uses pinhole (cv2.calibrateCamera) API."""
    return model in ("radtan", "rational")


def _solvepnp_pose(objp, corners, K, D, model):
    """Get (rvec, tvec) for one frame using the correct model API."""
    if model == "fisheye":
        pts_undist = cv2.fisheye.undistortPoints(corners, K, D)
        K_eye = np.eye(3, dtype=np.float64)
        D_zero = np.zeros(4)
        ok, rvec, tvec = cv2.solvePnP(objp, pts_undist, K_eye, D_zero)
    else:
        ok, rvec, tvec = cv2.solvePnP(objp, corners, K, D)
    return ok, rvec, tvec


def _stereo_via_solvepnp(
    obj_pts_list, img_pts_ref, img_pts_tgt,
    ref_K, ref_D, ref_model,
    tgt_K, tgt_D, tgt_model,
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], float]:
    """Estimate stereo extrinsics via per-frame solvePnP + relative transform.

    Used when models are mixed (e.g. fisheye + pinhole) since
    cv2.stereoCalibrate cannot mix distortion models.
    """
    Rs, Ts = [], []
    total_err = 0.0
    total_pts = 0

    for objp, pts_ref, pts_tgt in zip(obj_pts_list, img_pts_ref, img_pts_tgt):
        ok1, rv1, tv1 = _solvepnp_pose(objp, pts_ref, ref_K, ref_D, ref_model)
        ok2, rv2, tv2 = _solvepnp_pose(objp, pts_tgt, tgt_K, tgt_D, tgt_model)
        if not ok1 or not ok2:
            continue

        R1, _ = cv2.Rodrigues(rv1)
        R2, _ = cv2.Rodrigues(rv2)
        # T_ref_to_tgt = T_tgt_board * inv(T_ref_board)
        R_rel = R2 @ R1.T
        T_rel = tv2 - R_rel @ tv1
        Rs.append(R_rel)
        Ts.append(T_rel)

        # Reproject error estimate on target camera
        if tgt_model == "fisheye":
            proj, _ = cv2.fisheye.projectPoints(
                objp.reshape(1, -1, 3).astype(np.float64),
                rv2, tv2, tgt_K, tgt_D)
            proj = proj.reshape(-1, 1, 2)
        else:
            proj, _ = cv2.projectPoints(objp, rv2, tv2, tgt_K, tgt_D)
        err = np.linalg.norm(proj.reshape(-1, 2) - pts_tgt.reshape(-1, 2),
                             axis=1)
        total_err += np.sum(err ** 2)
        total_pts += len(err)

    if len(Rs) < 3:
        return None, None, -1.0

    # Average rotation via Rodrigues mean
    rvecs = [cv2.Rodrigues(R)[0] for R in Rs]
    mean_rvec = np.mean(rvecs, axis=0)
    R_mean, _ = cv2.Rodrigues(mean_rvec)
    T_mean = np.mean(Ts, axis=0)

    rms = float(np.sqrt(total_err / total_pts)) if total_pts > 0 else -1.0
    return R_mean, T_mean, rms


# ---------------------------------------------------------------------------
# Baseline verification helpers
# ---------------------------------------------------------------------------

def _undistort_points(pts: np.ndarray, K: np.ndarray, D: np.ndarray,
                      model: str) -> np.ndarray:
    """Undistort 2D points to normalized coordinates using the correct model.

    Args:
        pts: Nx1x2 or Nx2 array of image points.
        K: 3x3 camera matrix.
        D: distortion coefficients.
        model: "radtan", "rational", or "fisheye".

    Returns:
        Nx1x2 array of normalized (undistorted) points.
    """
    pts = pts.reshape(-1, 1, 2).astype(np.float64)
    if model == "fisheye":
        return cv2.fisheye.undistortPoints(pts, K, D)
    else:
        return cv2.undistortPoints(pts, K, D)


def _rotation_to_euler(R: np.ndarray) -> Tuple[float, float, float]:
    """Convert 3x3 rotation matrix to (roll, pitch, yaw) in degrees.

    Uses ZYX (Tait-Bryan) convention.
    """
    sy = np.sqrt(R[0, 0] ** 2 + R[1, 0] ** 2)
    singular = sy < 1e-6
    if not singular:
        roll = np.arctan2(R[2, 1], R[2, 2])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = np.arctan2(R[1, 0], R[0, 0])
    else:
        roll = np.arctan2(-R[1, 2], R[1, 1])
        pitch = np.arctan2(-R[2, 0], sy)
        yaw = 0.0
    return (float(np.degrees(roll)), float(np.degrees(pitch)),
            float(np.degrees(yaw)))


def _compute_epipolar_errors(
    pts_ref: np.ndarray, pts_tgt: np.ndarray,
    K_ref: np.ndarray, D_ref: np.ndarray, model_ref: str,
    K_tgt: np.ndarray, D_tgt: np.ndarray, model_tgt: str,
    R: np.ndarray, T: np.ndarray,
) -> Tuple[float, float, np.ndarray]:
    """Compute epipolar errors between matched point pairs.

    Undistorts both sets to normalized coordinates, computes Essential matrix
    from R,T, then measures Sampson distance. Converts to approximate pixel
    error by multiplying by average focal length.

    Returns (mean_error_px, max_error_px, per_point_errors_px).
    """
    pts_n_ref = _undistort_points(pts_ref, K_ref, D_ref, model_ref)
    pts_n_tgt = _undistort_points(pts_tgt, K_tgt, D_tgt, model_tgt)

    # Essential matrix from R, T
    Tx = np.array([[0, -T[2], T[1]],
                    [T[2], 0, -T[0]],
                    [-T[1], T[0], 0]], dtype=np.float64).reshape(3, 3)
    E = Tx @ R

    p1 = pts_n_ref.reshape(-1, 2)
    p2 = pts_n_tgt.reshape(-1, 2)

    # Epipolar constraint: p2^T * E * p1 = 0
    # Compute symmetric epipolar distance
    ones = np.ones((len(p1), 1), dtype=np.float64)
    p1h = np.hstack([p1, ones])  # Nx3
    p2h = np.hstack([p2, ones])  # Nx3

    # l2 = E @ p1 (epipolar line in image 2)
    l2 = (E @ p1h.T).T  # Nx3
    # l1 = E^T @ p2 (epipolar line in image 1)
    l1 = (E.T @ p2h.T).T  # Nx3

    # Sampson distance
    num = np.sum(p2h * (E @ p1h.T).T, axis=1) ** 2
    denom = l2[:, 0]**2 + l2[:, 1]**2 + l1[:, 0]**2 + l1[:, 1]**2
    denom = np.maximum(denom, 1e-12)
    sampson = np.sqrt(num / denom)

    # Convert normalized error to approximate pixel error
    avg_f_ref = (K_ref[0, 0] + K_ref[1, 1]) / 2.0
    avg_f_tgt = (K_tgt[0, 0] + K_tgt[1, 1]) / 2.0
    avg_f = (avg_f_ref + avg_f_tgt) / 2.0
    errors_px = sampson * avg_f

    return float(np.mean(errors_px)), float(np.max(errors_px)), errors_px


def _visualize_epipolar_lines(
    img_ref: np.ndarray, img_tgt: np.ndarray,
    pts_ref: np.ndarray, pts_tgt: np.ndarray,
    K_ref: np.ndarray, D_ref: np.ndarray, model_ref: str,
    K_tgt: np.ndarray, D_tgt: np.ndarray, model_tgt: str,
    R: np.ndarray, T: np.ndarray,
    errors_px: np.ndarray,
    save_path: str,
    ref_label: str = "cam_ref",
    tgt_label: str = "cam_tgt",
    error_threshold: float = 2.0,
):
    """Draw epipolar lines and matched corners on target image. Save to file.

    Green = good (error < threshold), Red = bad.
    """
    # Compute fundamental matrix for drawing lines in pixel space
    K_ref_inv = np.linalg.inv(K_ref)
    K_tgt_inv = np.linalg.inv(K_tgt)
    Tx = np.array([[0, -T[2], T[1]],
                    [T[2], 0, -T[0]],
                    [-T[1], T[0], 0]], dtype=np.float64).reshape(3, 3)
    E = Tx @ R

    # For visualization, undistort and re-project to pixel coords
    pts_n_ref = _undistort_points(pts_ref, K_ref, D_ref, model_ref)

    # Scale images to same height for side-by-side display
    h_ref, w_ref = img_ref.shape[:2]
    h_tgt, w_tgt = img_tgt.shape[:2]
    target_h = max(h_ref, h_tgt)

    if h_ref != target_h:
        scale_ref = target_h / h_ref
        vis_ref = cv2.resize(img_ref, (int(w_ref * scale_ref), target_h))
        s_ref = scale_ref
    else:
        vis_ref = img_ref.copy()
        s_ref = 1.0

    if h_tgt != target_h:
        scale_tgt = target_h / h_tgt
        vis_tgt = cv2.resize(img_tgt, (int(w_tgt * scale_tgt), target_h))
        s_tgt = scale_tgt
    else:
        vis_tgt = img_tgt.copy()
        s_tgt = 1.0

    vw_ref = vis_ref.shape[1]
    vw_tgt = vis_tgt.shape[1]

    canvas = np.zeros((target_h, vw_ref + vw_tgt, 3), dtype=np.uint8)
    canvas[:, :vw_ref] = vis_ref
    canvas[:, vw_ref:] = vis_tgt

    p_ref = pts_ref.reshape(-1, 2)
    p_tgt = pts_tgt.reshape(-1, 2)
    p_n_ref = pts_n_ref.reshape(-1, 2)

    for i in range(len(p_ref)):
        err = errors_px[i] if i < len(errors_px) else 0.0
        color = (0, 255, 0) if err < error_threshold else (0, 0, 255)

        # Draw point in reference image (left side)
        cx_r = int(p_ref[i, 0] * s_ref)
        cy_r = int(p_ref[i, 1] * s_ref)
        cv2.circle(canvas, (cx_r, cy_r), 3, color, -1)

        # Compute epipolar line in target image: l = E @ p1_normalized_homogeneous
        p1h = np.array([p_n_ref[i, 0], p_n_ref[i, 1], 1.0])
        l = E @ p1h  # epipolar line in normalized tgt coords
        # Convert to pixel coords: l_pix = K_tgt^{-T} @ l  (but for line: l_pix = K_tgt^{-T} is wrong)
        # Actually for F = K_tgt^{-T} E K_ref^{-1}, line in pixel: l_pix = F @ p1_pixel
        # Simpler: l_pix_tgt = K_tgt^{-T} @ l
        l_pix = np.linalg.inv(K_tgt).T @ l

        # Draw epipolar line on target image (right side)
        a, b, c = l_pix
        if abs(b) > 1e-8:
            x0_tgt = 0
            y0_tgt = int((-c - a * x0_tgt) / b * s_tgt)
            x1_tgt = w_tgt
            y1_tgt = int((-c - a * x1_tgt) / b * s_tgt)
        else:
            x0_tgt = int(-c / a) if abs(a) > 1e-8 else 0
            y0_tgt = 0
            x1_tgt = x0_tgt
            y1_tgt = target_h

        cv2.line(canvas,
                 (vw_ref + int(x0_tgt * s_tgt), y0_tgt),
                 (vw_ref + int(x1_tgt * s_tgt), y1_tgt),
                 color, 1)

        # Draw point in target image (right side)
        cx_t = vw_ref + int(p_tgt[i, 0] * s_tgt)
        cy_t = int(p_tgt[i, 1] * s_tgt)
        cv2.circle(canvas, (cx_t, cy_t), 3, color, -1)

    # Labels
    cv2.putText(canvas, ref_label, (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(canvas, tgt_label, (vw_ref + 10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    mean_err = float(np.mean(errors_px))
    max_err = float(np.max(errors_px))
    cv2.putText(canvas, f"Epi: mean={mean_err:.2f}px max={max_err:.2f}px",
                (10, target_h - 15),
                cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    cv2.imwrite(save_path, canvas)


def _visualize_stereo_rectification(
    img_ref: np.ndarray, img_tgt: np.ndarray,
    K_ref: np.ndarray, D_ref: np.ndarray,
    K_tgt: np.ndarray, D_tgt: np.ndarray,
    R: np.ndarray, T: np.ndarray,
    size_ref: Tuple[int, int], size_tgt: Tuple[int, int],
    model: str,
    save_path: str,
    ref_label: str = "cam_ref",
    tgt_label: str = "cam_tgt",
    num_lines: int = 20,
):
    """Compute stereo rectification and visualize with horizontal scanlines.

    Only works for same-model pairs (both pinhole or both fisheye).
    """
    if model == "fisheye":
        R1, R2, P1, P2, Q = cv2.fisheye.stereoRectify(
            K_ref, D_ref, K_tgt, D_tgt, size_ref, R, T.reshape(3, 1),
            flags=cv2.CALIB_ZERO_DISPARITY)
        map1_ref, map2_ref = cv2.fisheye.initUndistortRectifyMap(
            K_ref, D_ref, R1, P1, size_ref, cv2.CV_16SC2)
        map1_tgt, map2_tgt = cv2.fisheye.initUndistortRectifyMap(
            K_tgt, D_tgt, R2, P2, size_tgt, cv2.CV_16SC2)
    else:
        R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
            K_ref, D_ref, K_tgt, D_tgt, size_ref, R, T.reshape(3, 1),
            flags=cv2.CALIB_ZERO_DISPARITY)
        map1_ref, map2_ref = cv2.initUndistortRectifyMap(
            K_ref, D_ref, R1, P1, size_ref, cv2.CV_16SC2)
        map1_tgt, map2_tgt = cv2.initUndistortRectifyMap(
            K_tgt, D_tgt, R2, P2, size_tgt, cv2.CV_16SC2)

    rect_ref = cv2.remap(img_ref, map1_ref, map2_ref, cv2.INTER_LINEAR)
    rect_tgt = cv2.remap(img_tgt, map1_tgt, map2_tgt, cv2.INTER_LINEAR)

    # Scale to same height
    h_ref, w_ref = rect_ref.shape[:2]
    h_tgt, w_tgt = rect_tgt.shape[:2]
    target_h = max(h_ref, h_tgt)

    if h_ref != target_h:
        s = target_h / h_ref
        rect_ref = cv2.resize(rect_ref, (int(w_ref * s), target_h))
    if h_tgt != target_h:
        s = target_h / h_tgt
        rect_tgt = cv2.resize(rect_tgt, (int(w_tgt * s), target_h))

    vw_ref = rect_ref.shape[1]
    vw_tgt = rect_tgt.shape[1]

    canvas = np.zeros((target_h, vw_ref + vw_tgt, 3), dtype=np.uint8)
    canvas[:, :vw_ref] = rect_ref
    canvas[:, vw_ref:] = rect_tgt

    # Draw horizontal scanlines
    for i in range(num_lines):
        y = int(target_h * (i + 1) / (num_lines + 1))
        color = (0, 255, 255) if i % 2 == 0 else (255, 255, 0)
        cv2.line(canvas, (0, y), (vw_ref + vw_tgt, y), color, 1)

    cv2.putText(canvas, f"{ref_label} (rectified)", (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
    cv2.putText(canvas, f"{tgt_label} (rectified)", (vw_ref + 10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

    os.makedirs(os.path.dirname(save_path), exist_ok=True)
    cv2.imwrite(save_path, canvas)


def _load_calib_from_json(
    calib_data: dict, cfg: dict,
) -> Tuple[Dict[int, dict], Dict[int, dict], Dict[int, str]]:
    """Reconstruct K, D, R, T from calibration_full.json.

    Returns (intrinsics, extrinsics, cam_models) dicts keyed by camera index.
    """
    cameras = calib_data.get("cameras", {})
    cam_cfgs = cfg.get("cameras", [])
    ref_cam = cfg.get("reference_camera", 0)

    # Build name→index map from config
    name_to_cfg = {}
    for c in cam_cfgs:
        idx = c['index']
        name = c.get('name', f'cam{idx}')
        key = f"cam{idx}_{name}"
        name_to_cfg[key] = c

    intrinsics: Dict[int, dict] = {}
    extrinsics: Dict[int, dict] = {}
    cam_models: Dict[int, str] = {}

    for cam_key, sensor in cameras.items():
        # Find matching config entry
        c = name_to_cfg.get(cam_key)
        if c is None:
            continue
        idx = c["index"]
        name = c.get("name", f"cam{idx}")

        # Reconstruct model from distortion_model field
        dist_model_euroc = sensor.get("distortion_model", "radial-tangential")
        if dist_model_euroc == "equidistant":
            model = "fisheye"
        elif dist_model_euroc == "rational":
            model = "rational"
        else:
            model = "radtan"
        cam_models[idx] = model

        # Reconstruct K
        intr = sensor.get("intrinsics", [0, 0, 0, 0])
        K = np.array([[intr[0], 0, intr[2]],
                       [0, intr[1], intr[3]],
                       [0, 0, 1]], dtype=np.float64)

        # Reconstruct D
        d_coeffs = sensor.get("distortion_coefficients", [])
        D = np.array(d_coeffs, dtype=np.float64).reshape(1, -1)

        res = sensor.get("resolution", [0, 0])
        image_size = (res[0], res[1])

        intrinsics[idx] = {
            "K": K, "D": D, "image_size": image_size,
            "rms": sensor.get("intrinsic_rms_error", 0.0),
            "model": model,
        }

        # Reconstruct extrinsics if not reference
        if idx != ref_cam:
            t_bs = sensor.get("T_BS", {}).get("data", None)
            if t_bs and len(t_bs) == 16:
                M = np.array(t_bs, dtype=np.float64).reshape(4, 4)
                R = M[:3, :3]
                T_m = M[:3, 3]  # in meters
                T_mm = T_m * 1000.0  # convert back to mm
                extrinsics[idx] = {
                    "R": R, "T": T_mm.reshape(3, 1),
                    "rms": sensor.get("extrinsic_rms_error", 0.0),
                }

    return intrinsics, extrinsics, cam_models


def calibrate_extrinsics(
    image_dir: str,
    board: BoardSpec,
    ref_idx: int, ref_name: str,
    ref_K: np.ndarray, ref_D: np.ndarray, ref_size: Tuple[int, int],
    tgt_idx: int, tgt_name: str,
    tgt_K: np.ndarray, tgt_D: np.ndarray, tgt_size: Tuple[int, int],
    ref_model: str = "radtan",
    tgt_model: str = "radtan",
) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], float]:
    """Stereo calibrate cam_ref <-> cam_tgt with fixed intrinsics.

    Returns (R, T, rms) where R is 3x3 rotation and T is 3x1 translation
    (in mm, same units as square_size_mm). Returns Nones on failure.
    """
    ref_dir = os.path.join(image_dir, f"cam{ref_idx}_{ref_name}")
    tgt_dir = os.path.join(image_dir, f"cam{tgt_idx}_{tgt_name}")

    ref_info = DISTORTION_MODELS[ref_model]
    tgt_info = DISTORTION_MODELS[tgt_model]
    objp = board.object_points()

    # Find common frame indices where both cameras detected the board
    obj_pts_list = []
    img_pts_ref = []
    img_pts_tgt = []
    common_count = 0

    ref_files = sorted(f for f in os.listdir(ref_dir)
                       if f.startswith("frame_") and f.endswith(".png"))
    for fname in ref_files:
        tgt_path = os.path.join(tgt_dir, fname)
        ref_path = os.path.join(ref_dir, fname)
        if not os.path.exists(tgt_path):
            continue

        img_ref = cv2.imread(ref_path)
        img_tgt = cv2.imread(tgt_path)
        if img_ref is None or img_tgt is None:
            continue

        found_ref, corners_ref = detect_chessboard(
            img_ref, board.pattern_size,
            subpix_window=ref_info["subpix_window"])
        found_tgt, corners_tgt = detect_chessboard(
            img_tgt, board.pattern_size,
            subpix_window=tgt_info["subpix_window"])

        if found_ref and found_tgt:
            obj_pts_list.append(objp)
            img_pts_ref.append(corners_ref)
            img_pts_tgt.append(corners_tgt)
            common_count += 1

    print(f"  [cam{ref_idx}<->cam{tgt_idx}] Common detections: {common_count}")

    if common_count < 5:
        print(f"  [cam{ref_idx}<->cam{tgt_idx}] Not enough common frames (need >= 5)")
        return None, None, -1.0

    both_pinhole = _is_pinhole(ref_model) and _is_pinhole(tgt_model)
    both_fisheye = (ref_model == "fisheye") and (tgt_model == "fisheye")

    if both_pinhole:
        # Pad shorter D to match longer for stereoCalibrate
        ref_d = ref_D.flatten()
        tgt_d = tgt_D.flatten()
        max_len = max(len(ref_d), len(tgt_d))
        ref_d_pad = np.zeros(max_len, dtype=np.float64)
        tgt_d_pad = np.zeros(max_len, dtype=np.float64)
        ref_d_pad[:len(ref_d)] = ref_d
        tgt_d_pad[:len(tgt_d)] = tgt_d

        criteria = (cv2.TERM_CRITERIA_EPS | cv2.TERM_CRITERIA_MAX_ITER,
                    100, 1e-6)
        flags = cv2.CALIB_FIX_INTRINSIC
        if max_len > 5:
            flags |= cv2.CALIB_RATIONAL_MODEL

        rms, _, _, _, _, R, T, E, F = cv2.stereoCalibrate(
            obj_pts_list,
            img_pts_ref, img_pts_tgt,
            ref_K, ref_d_pad.reshape(1, -1),
            tgt_K, tgt_d_pad.reshape(1, -1),
            ref_size,
            criteria=criteria,
            flags=flags,
        )
    elif both_fisheye:
        obj_fish = [o.reshape(1, -1, 3) for o in obj_pts_list]
        img_ref_fish = [p.reshape(1, -1, 1, 2) for p in img_pts_ref]
        img_tgt_fish = [p.reshape(1, -1, 1, 2) for p in img_pts_tgt]
        flags = cv2.fisheye.CALIB_FIX_INTRINSIC
        rms, _, _, _, _, R, T = cv2.fisheye.stereoCalibrate(
            obj_fish,
            img_ref_fish, img_tgt_fish,
            ref_K, ref_D,
            tgt_K, tgt_D,
            ref_size,
            flags=flags,
        )
    else:
        # Mixed models: fall back to solvePnP-based estimation
        print(f"  [cam{ref_idx}<->cam{tgt_idx}] Mixed models "
              f"({ref_model}+{tgt_model}), using solvePnP fallback")
        R, T, rms = _stereo_via_solvepnp(
            obj_pts_list, img_pts_ref, img_pts_tgt,
            ref_K, ref_D, ref_model,
            tgt_K, tgt_D, tgt_model,
        )
        if R is not None:
            print(f"  [cam{ref_idx}<->cam{tgt_idx}] Stereo RMS = {rms:.4f}")
        return R, T, rms

    print(f"  [cam{ref_idx}<->cam{tgt_idx}] Stereo RMS = {rms:.4f}")
    return R, T, rms


# ---------------------------------------------------------------------------
# EuRoC YAML export
# ---------------------------------------------------------------------------

def _mat4x4_identity() -> List[float]:
    """Flat row-major 4x4 identity."""
    return [1, 0, 0, 0,
            0, 1, 0, 0,
            0, 0, 1, 0,
            0, 0, 0, 1]


def _T_BS_from_RT(R: np.ndarray, T: np.ndarray) -> List[float]:
    """Build flat row-major 4x4 SE(3) from 3x3 R and 3x1 T (mm -> m)."""
    M = np.eye(4)
    M[:3, :3] = R
    M[:3, 3] = (T.flatten() / 1000.0)  # mm to meters
    return M.flatten().tolist()


def export_euroc_yaml(
    output_dir: str,
    cam_idx: int,
    cam_name: str,
    K: np.ndarray,
    D: np.ndarray,
    image_size: Tuple[int, int],
    intrinsic_rms: float,
    R: Optional[np.ndarray] = None,
    T: Optional[np.ndarray] = None,
    extrinsic_rms: float = 0.0,
    is_reference: bool = False,
    rate_hz: int = 30,
    distortion_model: str = "radtan",
):
    """Write sensor.yaml in EuRoC format for one camera."""
    model_info = DISTORTION_MODELS[distortion_model]
    w, h = image_size
    fu, fv = float(K[0, 0]), float(K[1, 1])
    cu, cv_ = float(K[0, 2]), float(K[1, 2])
    d = D.flatten().tolist()

    if is_reference or R is None:
        t_bs = _mat4x4_identity()
        comment = f"{cam_name} - reference camera (body frame)"
    else:
        t_bs = _T_BS_from_RT(R, T)
        comment = f"{cam_name} - extrinsic relative to cam0"

    sensor = {
        "sensor_type": "camera",
        "comment": comment,
        "T_BS": {
            "cols": 4,
            "rows": 4,
            "data": t_bs,
        },
        "rate_hz": rate_hz,
        "resolution": [w, h],
        "camera_model": model_info["euroc_camera"],
        "intrinsics": [fu, fv, cu, cv_],
        "distortion_model": model_info["euroc_distortion"],
        "distortion_coefficients": d,
        "calibration_resolution": [w, h],
        "intrinsics_normalized": [fu / w, fv / h, cu / w, cv_ / h],
        "intrinsic_rms_error": float(round(intrinsic_rms, 6)),
    }
    if not is_reference and R is not None:
        sensor["extrinsic_rms_error"] = float(round(extrinsic_rms, 6))

    cam_dir = os.path.join(output_dir, f"cam{cam_idx}_{cam_name}")
    os.makedirs(cam_dir, exist_ok=True)
    yaml_path = os.path.join(cam_dir, "sensor.yaml")

    with open(yaml_path, "w") as f:
        yaml.dump(sensor, f, default_flow_style=False, sort_keys=False,
                  allow_unicode=True)

    print(f"  Written {yaml_path}")
    return sensor


# ---------------------------------------------------------------------------
# Full JSON dump
# ---------------------------------------------------------------------------

def export_full_json(output_dir: str, all_sensors: dict, board: BoardSpec):
    """Write calibration_full.json with everything."""
    data = {
        "calibration_date": datetime.now().isoformat(),
        "board": {
            "type": "chessboard",
            "cols": board.cols,
            "rows": board.rows,
            "square_size_mm": board.square_size_mm,
        },
        "cameras": all_sensors,
    }
    path = os.path.join(output_dir, "calibration_full.json")
    with open(path, "w") as f:
        json.dump(data, f, indent=2)
    print(f"  Written {path}")


# ---------------------------------------------------------------------------
# Undistortion visualization
# ---------------------------------------------------------------------------

def export_visualizations(
    image_dir: str,
    output_dir: str,
    cam_idx: int,
    cam_name: str,
    K: np.ndarray,
    D: np.ndarray,
    image_size: Tuple[int, int],
    distortion_model: str = "radtan",
):
    """Save undistorted comparison images for visual verification."""
    vis_dir = os.path.join(output_dir, "visualizations")
    os.makedirs(vis_dir, exist_ok=True)

    cam_dir = os.path.join(image_dir, f"cam{cam_idx}_{cam_name}")
    if not os.path.isdir(cam_dir):
        return

    files = sorted(f for f in os.listdir(cam_dir)
                   if f.startswith("frame_") and f.endswith(".png"))
    # Pick up to 3 sample images
    sample_files = files[:3] if len(files) >= 3 else files

    if distortion_model == "fisheye":
        new_K = cv2.fisheye.estimateNewCameraMatrixForUndistortRectify(
            K, D, image_size, np.eye(3), balance=0.0)
        # Sanity check: if focal length is degenerate, fall back to original K
        if new_K[0, 0] < 1.0 or new_K[1, 1] < 1.0:
            new_K = K.copy()
        map1, map2 = cv2.fisheye.initUndistortRectifyMap(
            K, D, np.eye(3), new_K, image_size, cv2.CV_16SC2)
    else:
        # Works for both radtan (4-param) and rational (8-param)
        new_K, roi = cv2.getOptimalNewCameraMatrix(
            K, D, image_size, 1, image_size)
        map1, map2 = None, None  # use cv2.undistort directly

    for fname in sample_files:
        img = cv2.imread(os.path.join(cam_dir, fname))
        if img is None:
            continue

        if distortion_model == "fisheye":
            undist = cv2.remap(img, map1, map2, cv2.INTER_LINEAR)
        else:
            undist = cv2.undistort(img, K, D, None, new_K)

        # Side-by-side comparison
        h_img, w_img = img.shape[:2]
        canvas = np.zeros((h_img, w_img * 2, 3), dtype=np.uint8)
        canvas[:, :w_img] = img
        canvas[:, w_img:] = undist

        cv2.putText(canvas, "Original", (10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
        cv2.putText(canvas, f"Undistorted ({distortion_model})",
                    (w_img + 10, 30),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)

        out_path = os.path.join(
            vis_dir, f"cam{cam_idx}_{cam_name}_{fname}")
        cv2.imwrite(out_path, canvas)

    print(f"  [cam{cam_idx}] Saved undistortion comparisons to {vis_dir}/")


# ---------------------------------------------------------------------------
# Baseline verification
# ---------------------------------------------------------------------------

def verify_baseline(
    capture_dir: str,
    output_dir: str,
    board: BoardSpec,
    intrinsics: Dict[int, dict],
    extrinsics: Dict[int, dict],
    cam_names: Dict[int, str],
    ref_cam: int = 0,
):
    """Verify stereo baseline quality with epipolar error analysis and visualizations.

    For each stereo pair (cam0<->camN):
    1. Compute baseline distance and euler angles
    2. Detect chessboard corners in shared frames
    3. Compute epipolar errors
    4. Save epipolar + rectification visualizations

    For same-model pairs (cam0<->cam1 pinhole, cam2<->cam3 fisheye):
    - Compute stereo rectification visualization
    """
    vis_dir = os.path.join(output_dir, "visualizations")
    os.makedirs(vis_dir, exist_ok=True)

    cam_indices = sorted(intrinsics.keys())
    ref = intrinsics[ref_cam]
    ref_name = cam_names.get(ref_cam, f"cam{ref_cam}")

    # Header
    print(f"\n  {'Pair':<16} {'Baseline':>10} {'Roll':>8} {'Pitch':>8} "
          f"{'Yaw':>8} {'Epi(mean)':>10} {'Epi(max)':>10}")
    print("  " + "-" * 82)

    pair_results = {}

    for idx in cam_indices:
        if idx == ref_cam:
            continue
        if idx not in extrinsics:
            continue

        tgt = intrinsics[idx]
        ext = extrinsics[idx]
        R = ext["R"]
        T = ext["T"]
        tgt_name = cam_names.get(idx, f"cam{idx}")

        # Baseline distance in mm
        baseline_mm = float(np.linalg.norm(T))

        # Euler angles
        roll, pitch, yaw = _rotation_to_euler(R)

        # Detect chessboard in shared frames
        ref_dir = os.path.join(capture_dir, f"cam{ref_cam}_{ref_name}")
        tgt_dir = os.path.join(capture_dir, f"cam{idx}_{tgt_name}")

        if not os.path.isdir(ref_dir) or not os.path.isdir(tgt_dir):
            print(f"  cam{ref_cam}<->cam{idx}: image dirs not found, skipping")
            continue

        ref_model_info = DISTORTION_MODELS[ref["model"]]
        tgt_model_info = DISTORTION_MODELS[tgt["model"]]

        ref_files = sorted(f for f in os.listdir(ref_dir)
                           if f.startswith("frame_") and f.endswith(".png"))

        all_pts_ref = []
        all_pts_tgt = []
        sample_img_ref = None
        sample_img_tgt = None

        for fname in ref_files:
            ref_path = os.path.join(ref_dir, fname)
            tgt_path = os.path.join(tgt_dir, fname)
            if not os.path.exists(tgt_path):
                continue

            img_ref = cv2.imread(ref_path)
            img_tgt = cv2.imread(tgt_path)
            if img_ref is None or img_tgt is None:
                continue

            found_ref, corners_ref = detect_chessboard(
                img_ref, board.pattern_size,
                subpix_window=ref_model_info["subpix_window"])
            found_tgt, corners_tgt = detect_chessboard(
                img_tgt, board.pattern_size,
                subpix_window=tgt_model_info["subpix_window"])

            if found_ref and found_tgt:
                all_pts_ref.append(corners_ref)
                all_pts_tgt.append(corners_tgt)
                if sample_img_ref is None:
                    sample_img_ref = img_ref
                    sample_img_tgt = img_tgt
                    sample_pts_ref = corners_ref
                    sample_pts_tgt = corners_tgt

        epi_mean = -1.0
        epi_max = -1.0

        if len(all_pts_ref) > 0:
            # Concatenate all points for aggregate epipolar error
            pts_ref_all = np.vstack(all_pts_ref)
            pts_tgt_all = np.vstack(all_pts_tgt)

            T_flat = T.flatten()
            epi_mean, epi_max, errors_all = _compute_epipolar_errors(
                pts_ref_all, pts_tgt_all,
                ref["K"], ref["D"], ref["model"],
                tgt["K"], tgt["D"], tgt["model"],
                R, T_flat,
            )

            # Visualize epipolar lines on sample frame
            if sample_img_ref is not None:
                _, _, sample_errors = _compute_epipolar_errors(
                    sample_pts_ref, sample_pts_tgt,
                    ref["K"], ref["D"], ref["model"],
                    tgt["K"], tgt["D"], tgt["model"],
                    R, T_flat,
                )
                epi_path = os.path.join(
                    vis_dir, f"epipolar_cam{ref_cam}_cam{idx}.png")
                _visualize_epipolar_lines(
                    sample_img_ref, sample_img_tgt,
                    sample_pts_ref, sample_pts_tgt,
                    ref["K"], ref["D"], ref["model"],
                    tgt["K"], tgt["D"], tgt["model"],
                    R, T_flat, sample_errors,
                    epi_path,
                    ref_label=f"cam{ref_cam}({ref_name})",
                    tgt_label=f"cam{idx}({tgt_name})",
                )

        pair_label = f"cam{ref_cam}<->cam{idx}"
        epi_mean_str = f"{epi_mean:.2f}px" if epi_mean >= 0 else "N/A"
        epi_max_str = f"{epi_max:.2f}px" if epi_max >= 0 else "N/A"
        print(f"  {pair_label:<16} {baseline_mm:>8.2f}mm {roll:>7.2f}° "
              f"{pitch:>7.2f}° {yaw:>7.2f}° {epi_mean_str:>10} {epi_max_str:>10}")

        pair_results[idx] = {
            "baseline_mm": baseline_mm,
            "roll": roll, "pitch": pitch, "yaw": yaw,
            "epi_mean": epi_mean, "epi_max": epi_max,
            "common_frames": len(all_pts_ref),
        }

    # Stereo rectification for same-model pairs
    # cam0 <-> cam1 (both pinhole/radtan)
    _try_stereo_rectify(
        capture_dir, vis_dir, board, intrinsics, extrinsics,
        cam_names, 0, 1, ref_cam)

    # cam2 <-> cam3 (both fisheye) — needs composed extrinsics
    _try_stereo_rectify(
        capture_dir, vis_dir, board, intrinsics, extrinsics,
        cam_names, 2, 3, ref_cam)

    return pair_results


def _try_stereo_rectify(
    capture_dir: str, vis_dir: str, board: BoardSpec,
    intrinsics: Dict[int, dict], extrinsics: Dict[int, dict],
    cam_names: Dict[int, str],
    idx_a: int, idx_b: int,
    ref_cam: int,
):
    """Try to compute and visualize stereo rectification for a same-model pair."""
    if idx_a not in intrinsics or idx_b not in intrinsics:
        return
    if intrinsics[idx_a]["model"] != intrinsics[idx_b]["model"]:
        return

    model = intrinsics[idx_a]["model"]
    K_a = intrinsics[idx_a]["K"]
    D_a = intrinsics[idx_a]["D"]
    K_b = intrinsics[idx_b]["K"]
    D_b = intrinsics[idx_b]["D"]
    size_a = intrinsics[idx_a]["image_size"]
    size_b = intrinsics[idx_b]["image_size"]
    name_a = cam_names.get(idx_a, f"cam{idx_a}")
    name_b = cam_names.get(idx_b, f"cam{idx_b}")

    # Compute R, T between idx_a and idx_b
    if idx_a == ref_cam and idx_b in extrinsics:
        R_ab = extrinsics[idx_b]["R"]
        T_ab = extrinsics[idx_b]["T"]
    elif idx_b == ref_cam and idx_a in extrinsics:
        R_ba = extrinsics[idx_a]["R"]
        T_ba = extrinsics[idx_a]["T"]
        R_ab = R_ba.T
        T_ab = -R_ab @ T_ba
    elif idx_a in extrinsics and idx_b in extrinsics:
        # Compose: R_ab = R_0b @ R_0a^T, T_ab = T_0b - R_ab @ T_0a
        R_0a = extrinsics[idx_a]["R"]
        T_0a = extrinsics[idx_a]["T"]
        R_0b = extrinsics[idx_b]["R"]
        T_0b = extrinsics[idx_b]["T"]
        R_ab = R_0b @ R_0a.T
        T_ab = T_0b - R_ab @ T_0a
    else:
        return

    # Find a sample frame where both cameras detected the board
    dir_a = os.path.join(capture_dir, f"cam{idx_a}_{name_a}")
    dir_b = os.path.join(capture_dir, f"cam{idx_b}_{name_b}")
    if not os.path.isdir(dir_a) or not os.path.isdir(dir_b):
        return

    files = sorted(f for f in os.listdir(dir_a)
                   if f.startswith("frame_") and f.endswith(".png"))
    for fname in files:
        path_a = os.path.join(dir_a, fname)
        path_b = os.path.join(dir_b, fname)
        if not os.path.exists(path_b):
            continue
        img_a = cv2.imread(path_a)
        img_b = cv2.imread(path_b)
        if img_a is None or img_b is None:
            continue

        # Verify both have detectable boards (ensures good frame)
        model_info = DISTORTION_MODELS[model]
        found_a, _ = detect_chessboard(
            img_a, board.pattern_size,
            subpix_window=model_info["subpix_window"])
        found_b, _ = detect_chessboard(
            img_b, board.pattern_size,
            subpix_window=model_info["subpix_window"])
        if not found_a or not found_b:
            continue

        rect_path = os.path.join(
            vis_dir, f"rectified_cam{idx_a}_cam{idx_b}.png")
        try:
            _visualize_stereo_rectification(
                img_a, img_b,
                K_a, D_a, K_b, D_b,
                R_ab, T_ab, size_a, size_b, model,
                rect_path,
                ref_label=f"cam{idx_a}({name_a})",
                tgt_label=f"cam{idx_b}({name_b})",
            )
            print(f"  Saved rectification: {rect_path}")
        except Exception as e:
            print(f"  Rectification failed for cam{idx_a}<->cam{idx_b}: {e}")
        return  # Only need one frame


# ---------------------------------------------------------------------------
# Calibrate mode
# ---------------------------------------------------------------------------

def run_calibrate_mode(cfg: dict, args) -> Optional[str]:
    """Run full calibration pipeline on saved images.

    Returns the output directory path or None on failure.
    """
    capture_dir = args.capture_dir or cfg.get("capture_dir", "calib_images")
    output_dir = args.output_dir or cfg.get("output_dir", "calib_output")
    ref_cam = cfg.get("reference_camera", 0)
    cam_cfgs = cfg.get("cameras", [])
    board_cfg = cfg.get("board", {})

    board = BoardSpec(
        cols=board_cfg.get("cols", 9),
        rows=board_cfg.get("rows", 6),
        square_size_mm=board_cfg.get("square_size_mm", 25.0),
    )

    cam_indices = [c["index"] for c in cam_cfgs if c.get("enabled", True)]
    cam_names = {c["index"]: c.get("name", f"cam{c['index']}") for c in cam_cfgs}

    # Build per-camera distortion model map (default: "radtan" for backward compat)
    cam_models: Dict[int, str] = {}
    for c in cam_cfgs:
        if not c.get("enabled", True):
            continue
        model = c.get("distortion_model", "radtan")
        if model not in DISTORTION_MODELS:
            print(f"Warning: unknown distortion_model '{model}' for "
                  f"cam{c['index']}, falling back to 'radtan'")
            model = "radtan"
        cam_models[c["index"]] = model

    if not os.path.isdir(capture_dir):
        print(f"Error: Capture directory not found: {capture_dir}")
        print("Run 'capture' mode first.")
        return None

    os.makedirs(output_dir, exist_ok=True)

    # ---- Step 1: Intrinsic calibration per camera ----
    print("\n" + "=" * 60)
    print("STEP 1: Intrinsic Calibration (per camera)")
    print("=" * 60)

    intrinsics: Dict[int, dict] = {}
    for idx in cam_indices:
        name = cam_names.get(idx, f"cam{idx}")
        model = cam_models.get(idx, "radtan")
        print(f"\nCalibrating cam{idx} ({name}) model={model}...")
        K, D, img_size, rms, used = calibrate_intrinsics(
            capture_dir, board, idx, name, distortion_model=model)
        if K is None:
            print(f"  FAILED - skipping cam{idx}")
            continue
        intrinsics[idx] = {
            "K": K, "D": D, "image_size": img_size,
            "rms": rms, "used_frames": used, "model": model,
        }

    if ref_cam not in intrinsics:
        print(f"\nError: Reference camera cam{ref_cam} calibration failed.")
        return None

    # ---- Step 2: Extrinsic calibration (cam0 <-> camN) ----
    print("\n" + "=" * 60)
    print("STEP 2: Extrinsic Calibration (relative to cam0)")
    print("=" * 60)

    extrinsics: Dict[int, dict] = {}
    ref = intrinsics[ref_cam]
    ref_name = cam_names.get(ref_cam, f"cam{ref_cam}")

    for idx in cam_indices:
        if idx == ref_cam:
            continue
        if idx not in intrinsics:
            continue
        name = cam_names.get(idx, f"cam{idx}")
        tgt = intrinsics[idx]
        print(f"\nStereo cam{ref_cam}<->cam{idx} ...")
        R, T, rms = calibrate_extrinsics(
            capture_dir, board,
            ref_cam, ref_name, ref["K"], ref["D"], ref["image_size"],
            idx, name, tgt["K"], tgt["D"], tgt["image_size"],
            ref_model=ref["model"], tgt_model=tgt["model"],
        )
        if R is not None:
            extrinsics[idx] = {"R": R, "T": T, "rms": rms}

    # ---- Step 3: Export ----
    print("\n" + "=" * 60)
    print("STEP 3: Export EuRoC sensor.yaml")
    print("=" * 60 + "\n")

    all_sensors: Dict[str, dict] = {}

    for idx in cam_indices:
        if idx not in intrinsics:
            continue
        name = cam_names.get(idx, f"cam{idx}")
        intr = intrinsics[idx]
        is_ref = (idx == ref_cam)

        ext = extrinsics.get(idx, {})
        R = ext.get("R")
        T = ext.get("T")
        ext_rms = ext.get("rms", 0.0)

        sensor = export_euroc_yaml(
            output_dir, idx, name,
            intr["K"], intr["D"], intr["image_size"],
            intr["rms"],
            R=R, T=T, extrinsic_rms=ext_rms,
            is_reference=is_ref,
            distortion_model=intr["model"],
        )
        all_sensors[f"cam{idx}_{name}"] = sensor

    export_full_json(output_dir, all_sensors, board)

    # ---- Step 4: Visualizations ----
    print("\n" + "=" * 60)
    print("STEP 4: Undistortion Visualizations")
    print("=" * 60 + "\n")

    for idx in cam_indices:
        if idx not in intrinsics:
            continue
        name = cam_names.get(idx, f"cam{idx}")
        intr = intrinsics[idx]
        export_visualizations(
            capture_dir, output_dir, idx, name,
            intr["K"], intr["D"], intr["image_size"],
            distortion_model=intr["model"],
        )

    # ---- Step 5: Baseline verification ----
    print("\n" + "=" * 60)
    print("STEP 5: Baseline Verification")
    print("=" * 60)

    verify_baseline(
        capture_dir, output_dir, board,
        intrinsics, extrinsics, cam_names,
        ref_cam=ref_cam,
    )

    # ---- Summary ----
    print("\n" + "=" * 60)
    print("CALIBRATION SUMMARY")
    print("=" * 60)
    for idx in cam_indices:
        if idx not in intrinsics:
            print(f"  cam{idx}: FAILED")
            continue
        intr = intrinsics[idx]
        w, h = intr["image_size"]
        K = intr["K"]
        n_fu = K[0, 0] / w
        n_fv = K[1, 1] / h
        n_cu = K[0, 2] / w
        n_cv = K[1, 2] / h
        ext_str = ""
        if idx != ref_cam and idx in extrinsics:
            T = extrinsics[idx]["T"].flatten() / 1000.0
            ext_str = (f"  T=[{T[0]:.4f}, {T[1]:.4f}, {T[2]:.4f}]m "
                       f"stereo_rms={extrinsics[idx]['rms']:.4f}")
        elif idx == ref_cam:
            ext_str = "  (reference / body frame)"
        name = cam_names.get(idx, f"cam{idx}")
        model = intr["model"]
        print(f"  cam{idx} ({name}): {w}x{h}  "
              f"model={model}  rms={intr['rms']:.4f}  "
              f"norm=[{n_fu:.4f}, {n_fv:.4f}, {n_cu:.4f}, {n_cv:.4f}]"
              f"{ext_str}")

    print(f"\nOutput: {output_dir}/")
    print("=" * 60)
    return output_dir


# ---------------------------------------------------------------------------
# Verify mode (standalone)
# ---------------------------------------------------------------------------

def run_verify_mode(cfg: dict, args) -> Optional[str]:
    """Run baseline verification from saved calibration data.

    Loads calibration_full.json and re-detects chessboard corners for
    epipolar error analysis.
    """
    capture_dir = args.capture_dir or cfg.get("capture_dir", "calib_images")
    output_dir = args.output_dir or cfg.get("output_dir", "calib_output")
    ref_cam = cfg.get("reference_camera", 0)
    board_cfg = cfg.get("board", {})

    # Load calibration data
    calib_path = os.path.join(output_dir, "calibration_full.json")
    if not os.path.exists(calib_path):
        print(f"Error: {calib_path} not found. Run 'calibrate' mode first.")
        return None

    with open(calib_path, "r") as f:
        calib_data = json.load(f)

    # Override board from calibration data if available
    board_data = calib_data.get("board", board_cfg)
    board = BoardSpec(
        cols=board_data.get("cols", 11),
        rows=board_data.get("rows", 8),
        square_size_mm=board_data.get("square_size_mm", 25.0),
    )

    cam_cfgs = cfg.get("cameras", [])
    cam_names = {c["index"]: c.get("name", f"cam{c['index']}") for c in cam_cfgs}

    intrinsics, extrinsics, cam_models = _load_calib_from_json(calib_data, cfg)

    if ref_cam not in intrinsics:
        print(f"Error: Reference camera cam{ref_cam} not found in calibration data.")
        return None

    print("\n" + "=" * 60)
    print("BASELINE VERIFICATION (standalone)")
    print("=" * 60)

    verify_baseline(
        capture_dir, output_dir, board,
        intrinsics, extrinsics, cam_names,
        ref_cam=ref_cam,
    )

    print(f"\nVisualizations saved to: {output_dir}/visualizations/")
    print("=" * 60)
    return output_dir


# ---------------------------------------------------------------------------
# Config loading
# ---------------------------------------------------------------------------

def load_config() -> dict:
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


# ---------------------------------------------------------------------------
# CLI
# ---------------------------------------------------------------------------

def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Quad Camera Calibration Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""\
Modes:
  capture   - Receive ZMQ streams, display 2x2 grid, save frames on SPACE
  calibrate - Run intrinsic + extrinsic calibration on saved images
  verify    - Verify baseline from saved calibration (epipolar errors + viz)
  (default) - Capture interactively, then calibrate

Examples:
  python quad_calibrate.py capture --host 192.168.100.1
  python quad_calibrate.py calibrate --capture-dir calib_images
  python quad_calibrate.py verify
  python quad_calibrate.py
""",
    )
    parser.add_argument("mode", nargs="?", default=None,
                        choices=["capture", "calibrate", "verify"],
                        help="Operating mode (default: capture then calibrate)")
    parser.add_argument("--host", default=None,
                        help="Board IP (overrides config)")
    parser.add_argument("--base-port", type=int, default=None,
                        help="Base ZMQ port (overrides config)")
    parser.add_argument("--capture-dir", default=None,
                        help="Directory for captured images")
    parser.add_argument("--output-dir", default=None,
                        help="Directory for calibration output")
    parser.add_argument("--cell-width", type=int, default=640,
                        help="Cell width for preview grid (default: 640)")
    return parser


def main():
    parser = build_parser()
    args = parser.parse_args()
    cfg = load_config()

    mode = args.mode

    if mode == "capture":
        run_capture_mode(cfg, args)

    elif mode == "calibrate":
        run_calibrate_mode(cfg, args)

    elif mode == "verify":
        run_verify_mode(cfg, args)

    else:
        # Default: capture then calibrate
        print("=== QUAD CAMERA CALIBRATION (interactive) ===\n")
        print("Step 1: Capture calibration images")
        capture_dir = run_capture_mode(cfg, args)

        ans = input("\nProceed to calibration? [Y/n] ").strip().lower()
        if ans in ("", "y", "yes"):
            print("\nStep 2: Run calibration")
            run_calibrate_mode(cfg, args)
        else:
            print("Calibration skipped. Run manually with:")
            print(f"  python {sys.argv[0]} calibrate "
                  f"--capture-dir {capture_dir}")


if __name__ == "__main__":
    main()
