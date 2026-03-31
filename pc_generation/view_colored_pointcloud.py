#!/usr/bin/env python3
"""
Capture one synchronized RGB + depth frame pair, build a colored point cloud
from the stereo calibration, and display it interactively with Matplotlib.

Usage (from repo root):
    arena python3 pc_generation/view_colored_pointcloud.py
    arena python3 pc_generation/view_colored_pointcloud.py --stereo-yaml path/to/stereo.yaml
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401 — registers the 3d projection

REPO_ROOT = Path(__file__).resolve().parents[1]
if str(Path(__file__).resolve().parent) not in sys.path:
    sys.path.insert(0, str(Path(__file__).resolve().parent))

from src.helios2 import Helios2Camera
from src.phoenix import PhoenixCamera


DEFAULT_STEREO_YAML = (
    REPO_ROOT / "calibration/results/combined_all_datasets/stereo_calibration.yaml"
)


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__)
    p.add_argument("--stereo-yaml", type=Path, default=DEFAULT_STEREO_YAML)
    p.add_argument("--helios-format", choices=["Coord3D_ABCY16", "Coord3D_ABCY16s"],
                   default="Coord3D_ABCY16")
    p.add_argument("--max-delta-sec", type=float, default=0.15,
                   help="Max timestamp delta between RGB and depth frames (s).")
    p.add_argument("--max-attempts", type=int, default=20,
                   help="Pairing attempts before giving up.")
    p.add_argument("--max-points", type=int, default=50_000,
                   help="Max points rendered in Matplotlib.")
    p.add_argument("--phoenix-index", type=int, default=0)
    p.add_argument("--helios-index", type=int, default=0)
    return p.parse_args()


def load_calibration(path: Path) -> dict[str, np.ndarray]:
    fs = cv2.FileStorage(str(path), cv2.FileStorage_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Cannot open calibration file: {path}")
    try:
        cal = {k: fs.getNode(k).mat() for k in
               ("rgb_camera_matrix", "rgb_dist_coeffs", "R", "T")}
    finally:
        fs.release()
    missing = [k for k, v in cal.items() if v is None or v.size == 0]
    if missing:
        raise RuntimeError(f"Calibration file missing fields: {missing}")
    return cal


def capture_pair(phoenix: PhoenixCamera, helios: Helios2Camera,
                 max_delta: float, max_attempts: int):
    for _ in range(max_attempts):
        rgb = phoenix.get_frame()
        depth = helios.get_frame()
        if abs(rgb.timestamp - depth.timestamp) <= max_delta:
            return rgb, depth
    raise RuntimeError(
        f"Could not capture a synchronized frame pair in {max_attempts} attempts."
    )


def build_colored_pointcloud(
    xyz_mm: np.ndarray,
    bgr: np.ndarray,
    K_rgb: np.ndarray,
    dist_rgb: np.ndarray,
    R: np.ndarray,
    T: np.ndarray,
) -> tuple[np.ndarray, np.ndarray]:
    """
    Project Helios depth points into the RGB image and sample colors.

    R, T are from OpenCV stereoCalibrate:  X_depth = R @ X_rgb + T
    So the inverse transform is:           X_rgb   = R.T @ (X_depth - T)

    Returns (points_m, colors_rgb) — both shape (N, 3).
    """
    valid = xyz_mm[:, :, 2] > 0
    pts_depth_m = xyz_mm[valid].astype(np.float64) / 1000.0

    T_flat = T.reshape(3)
    pts_rgb_m = (R.T @ (pts_depth_m - T_flat).T).T

    in_front = pts_rgb_m[:, 2] > 0
    pts_depth_m = pts_depth_m[in_front]
    pts_rgb_m = pts_rgb_m[in_front]

    if pts_rgb_m.size == 0:
        raise RuntimeError("No valid depth points in front of the RGB camera.")

    uv, _ = cv2.projectPoints(
        pts_rgb_m.reshape(-1, 1, 3),
        np.zeros((3, 1)), np.zeros((3, 1)),
        K_rgb, dist_rgb,
    )
    uv = uv.reshape(-1, 2)

    h, w = bgr.shape[:2]
    u = np.round(uv[:, 0]).astype(np.int32)
    v = np.round(uv[:, 1]).astype(np.int32)
    inside = (u >= 0) & (u < w) & (v >= 0) & (v < h)

    pts_depth_m = pts_depth_m[inside]
    colors_rgb = bgr[v[inside], u[inside]][:, ::-1]   # BGR → RGB

    return pts_depth_m, colors_rgb


def show(points_m: np.ndarray, colors_rgb: np.ndarray, max_points: int) -> None:
    step = max(1, len(points_m) // max_points)
    pts = points_m[::step]
    col = colors_rgb[::step].astype(np.float32) / 255.0

    fig = plt.figure("Colored Point Cloud")
    ax = fig.add_subplot(111, projection="3d")
    ax.scatter(pts[:, 0], pts[:, 1], pts[:, 2], c=col, s=0.5, depthshade=False)
    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_zlabel("Z (m)")
    ax.set_title("Helios point cloud colored by Phoenix RGB")
    ranges = [np.ptp(pts[:, i]) for i in range(3)]
    if all(r > 0 for r in ranges):
        ax.set_box_aspect(ranges)
    plt.tight_layout()
    plt.show()


def main() -> int:
    args = parse_args()

    print(f"Loading calibration: {args.stereo_yaml}")
    cal = load_calibration(args.stereo_yaml)

    phoenix = PhoenixCamera.from_model(index=args.phoenix_index,
                                       pixel_format="BayerRG8", binning=2)
    helios = Helios2Camera.from_model(index=args.helios_index,
                                      pixel_format=args.helios_format)
    try:
        phoenix.configure()
        helios.configure()
        phoenix.start_stream()
        helios.start_stream()

        print("Capturing synchronized frame pair…")
        rgb_frame, depth_frame = capture_pair(
            phoenix, helios, args.max_delta_sec, args.max_attempts
        )
        print(f"  RGB   timestamp: {rgb_frame.timestamp:.4f}")
        print(f"  Depth timestamp: {depth_frame.timestamp:.4f}")

        print("Building colored point cloud…")
        points_m, colors_rgb = build_colored_pointcloud(
            depth_frame.xyz,
            rgb_frame.image,
            cal["rgb_camera_matrix"],
            cal["rgb_dist_coeffs"],
            cal["R"],
            cal["T"],
        )
        print(f"  {len(points_m):,} colored points")

        show(points_m, colors_rgb, args.max_points)
    finally:
        phoenix.release()
        helios.release()

    return 0


if __name__ == "__main__":
    raise SystemExit(main())
