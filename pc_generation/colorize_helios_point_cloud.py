#!/usr/bin/env python3

from __future__ import annotations

import argparse
import os
import sys
import time
from pathlib import Path

# Prevent cv2's bundled Qt from conflicting with the system Qt / matplotlib.
os.environ.pop("QT_QPA_PLATFORM_PLUGIN_PATH", None)

import cv2
import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D  # noqa: F401
import numpy as np
from arena_api.system import system


REPO_ROOT = Path("/home/khoa/Projects/zabidou_pc_sync")
PC_GENERATION_ROOT = REPO_ROOT / "pc_generation"
if str(PC_GENERATION_ROOT) not in sys.path:
    sys.path.insert(0, str(PC_GENERATION_ROOT))

from src.helios2 import Helios2Camera
from src.phoenix import PhoenixCamera


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture a Helios point cloud and colorize it using Phoenix RGB plus stereo calibration."
    )
    parser.add_argument(
        "--stereo-yaml",
        type=Path,
        default=REPO_ROOT / "calibration/results/combined_all_datasets/stereo_calibration.yaml",
        help="Stereo calibration YAML path.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=REPO_ROOT / "calibration/results/combined_all_datasets/helios_colored_point_cloud.ply",
        help="Output PLY path.",
    )
    parser.add_argument("--phoenix-index", type=int, default=0, help="Phoenix device index.")
    parser.add_argument("--helios-index", type=int, default=0, help="Helios device index.")
    parser.add_argument(
        "--helios-format",
        choices=["Coord3D_ABCY16", "Coord3D_ABCY16s"],
        default="Coord3D_ABCY16",
        help="Helios pixel format.",
    )
    parser.add_argument(
        "--max-delta-sec",
        type=float,
        default=0.15,
        help="Max allowed RGB/Helios timestamp delta in seconds for pairing.",
    )
    parser.add_argument(
        "--max-attempts",
        type=int,
        default=20,
        help="Maximum pairing attempts before giving up.",
    )
    parser.add_argument(
        "--preview",
        action="store_true",
        help="Show an RGB overlay preview of projected Helios points.",
    )
    parser.add_argument(
        "--show-matplotlib",
        action="store_true",
        help="Show the colored point cloud interactively with Matplotlib.",
    )
    parser.add_argument(
        "--max-plot-points",
        type=int,
        default=50000,
        help="Maximum number of points to render in the Matplotlib view.",
    )
    parser.add_argument(
        "--elev",
        type=float,
        default=10.0,
        help="Initial 3D view elevation angle in degrees (default: 10).",
    )
    parser.add_argument(
        "--azim",
        type=float,
        default=0.0,
        help="Initial 3D view azimuth angle in degrees (default: 0).",
    )
    parser.add_argument(
        "--zoom",
        type=float,
        default=7.0,
        help="Initial 3D view zoom (camera distance, smaller = more zoomed in, default: 7).",
    )
    return parser.parse_args()


def load_stereo_calibration(path: Path) -> dict[str, np.ndarray]:
    fs = cv2.FileStorage(str(path), cv2.FileStorage_READ)
    if not fs.isOpened():
        raise RuntimeError(f"Failed to open stereo calibration file: {path}")

    try:
        data = {
            "rgb_camera_matrix": fs.getNode("rgb_camera_matrix").mat(),
            "rgb_dist_coeffs": fs.getNode("rgb_dist_coeffs").mat(),
            "R": fs.getNode("R").mat(),
            "T": fs.getNode("T").mat(),
        }
    finally:
        fs.release()

    for key, value in data.items():
        if value is None or value.size == 0:
            raise RuntimeError(f"Stereo calibration file is missing field: {key}")
    return data


def find_device_by_prefix(model_prefix: str, index: int):
    devices = system.create_device()
    matches = [
        device
        for device in devices
        if device.nodemap["DeviceModelName"].value.startswith(model_prefix)
    ]
    if index < 0 or index >= len(matches):
        available = [device.nodemap["DeviceModelName"].value for device in devices]
        raise RuntimeError(
            f"No device for prefix {model_prefix!r} at index {index}. Available: {available}"
        )
    return matches[index]


def open_cameras(args: argparse.Namespace):
    phoenix_device = find_device_by_prefix("PHX", args.phoenix_index)
    helios_device = find_device_by_prefix("HTP", args.helios_index)

    phoenix = PhoenixCamera(
        phoenix_device,
        pixel_format="BayerRG8",
        binning=2,
        binning_mode="Average",
    )
    helios = Helios2Camera(helios_device, pixel_format=args.helios_format)

    phoenix.configure()
    helios.configure()
    phoenix.start_stream()
    helios.start_stream()
    return phoenix, helios


def capture_paired_frames(
    phoenix: PhoenixCamera,
    helios: Helios2Camera,
    max_delta_sec: float,
    max_attempts: int,
):
    # Capture Helios first (slow, ~2 fps) then immediately grab Phoenix
    # (fast, ~26 fps). This keeps the delta to ~one Phoenix frame period.
    for _ in range(max_attempts):
        latest_depth = helios.get_frame()
        latest_rgb = phoenix.get_frame()
        delta = abs(latest_rgb.timestamp - latest_depth.timestamp)
        if delta <= max_delta_sec:
            return latest_rgb, latest_depth

    raise RuntimeError(
        f"Failed to capture a sufficiently close RGB/Helios frame pair "
        f"within {max_attempts} attempts (threshold: {max_delta_sec*1000:.0f} ms)."
    )


def colorize_point_cloud(
    helios_xyz_mm: np.ndarray,
    rgb_image_bgr: np.ndarray,
    rgb_camera_matrix: np.ndarray,
    rgb_dist_coeffs: np.ndarray,
    R_rgb_to_depth: np.ndarray,
    T_rgb_to_depth_m: np.ndarray,
):
    valid_mask = helios_xyz_mm[:, :, 2] > 0
    if not np.any(valid_mask):
        raise RuntimeError("No valid Helios depth points available.")

    points_depth_m = helios_xyz_mm[valid_mask].astype(np.float64) / 1000.0

    # OpenCV stereoCalibrate returns X_depth = R * X_rgb + T, so convert
    # Helios/depth-frame points into the RGB camera frame before projection.
    T_rgb_to_depth_m = T_rgb_to_depth_m.reshape(3)
    points_rgb_m = (R_rgb_to_depth.T @ (points_depth_m - T_rgb_to_depth_m).T).T

    in_front_mask = points_rgb_m[:, 2] > 0
    points_depth_m = points_depth_m[in_front_mask]
    points_rgb_m = points_rgb_m[in_front_mask]
    if points_rgb_m.size == 0:
        raise RuntimeError("No valid Helios points remain in front of the RGB camera.")

    projected, _ = cv2.projectPoints(
        points_rgb_m.reshape(-1, 1, 3),
        np.zeros((3, 1), dtype=np.float64),
        np.zeros((3, 1), dtype=np.float64),
        rgb_camera_matrix,
        rgb_dist_coeffs,
    )
    projected = projected.reshape(-1, 2)

    width = rgb_image_bgr.shape[1]
    height = rgb_image_bgr.shape[0]
    u = np.round(projected[:, 0]).astype(np.int32)
    v = np.round(projected[:, 1]).astype(np.int32)
    inside_mask = (u >= 0) & (u < width) & (v >= 0) & (v < height)

    points_depth_m = points_depth_m[inside_mask]
    u = u[inside_mask]
    v = v[inside_mask]
    colors_bgr = rgb_image_bgr[v, u]
    colors_rgb = colors_bgr[:, ::-1]
    return points_depth_m, colors_rgb, projected[inside_mask]


def save_ply(path: Path, points_m: np.ndarray, colors_rgb: np.ndarray) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("w", encoding="ascii") as handle:
        handle.write("ply\n")
        handle.write("format ascii 1.0\n")
        handle.write(f"element vertex {len(points_m)}\n")
        handle.write("property float x\n")
        handle.write("property float y\n")
        handle.write("property float z\n")
        handle.write("property uchar red\n")
        handle.write("property uchar green\n")
        handle.write("property uchar blue\n")
        handle.write("end_header\n")
        for point, color in zip(points_m, colors_rgb, strict=True):
            handle.write(
                f"{point[0]:.6f} {point[1]:.6f} {point[2]:.6f} "
                f"{int(color[0])} {int(color[1])} {int(color[2])}\n"
            )


def show_preview(rgb_image_bgr: np.ndarray, projected_xy: np.ndarray) -> None:
    preview = rgb_image_bgr.copy()
    for x, y in projected_xy[:: max(1, len(projected_xy) // 5000)]:
        cv2.circle(preview, (int(round(x)), int(round(y))), 1, (0, 255, 0), -1)
    cv2.imshow("RGB overlay", preview)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def update_matplotlib_point_cloud(
    axes,
    points_m: np.ndarray,
    colors_rgb: np.ndarray,
    max_plot_points: int,
) -> None:
    elev = axes.elev
    azim = axes.azim
    dist = axes.dist
    axes.cla()
    axes.view_init(elev=elev, azim=azim)
    axes.dist = dist

    if len(points_m) == 0:
        return

    step = max(1, len(points_m) // max_plot_points)
    sampled_points = points_m[::step]
    sampled_colors = colors_rgb[::step].astype(np.float32) / 255.0

    # Remap from camera frame (X right, Y down, Z forward) to display frame
    # where Y is up and Z points away from the viewer:
    #   plot X = X_cam  (left/right)
    #   plot Y = Z_cam  (depth, away from viewer)
    #   plot Z = -Y_cam (up, since camera Y is down)
    px = sampled_points[:, 0]
    py = sampled_points[:, 2]
    pz = -sampled_points[:, 1]

    axes.scatter(px, py, pz, c=sampled_colors, s=0.5, depthshade=False)
    axes.set_xlabel("X (m)")
    axes.set_ylabel("Z / depth (m)")
    axes.set_zlabel("Y (m)")
    axes.set_title("Helios point cloud colored by Phoenix RGB")
    ranges = [np.ptp(px), np.ptp(py), np.ptp(pz)]
    if all(r > 0 for r in ranges):
        axes.set_box_aspect(ranges)


def main() -> int:
    args = parse_args()
    calibration = load_stereo_calibration(args.stereo_yaml)

    plt.ion()
    figure = plt.figure("Helios Colored Point Cloud")
    axes = figure.add_subplot(111, projection="3d")
    axes.view_init(elev=args.elev, azim=args.azim)
    axes.dist = args.zoom
    plt.show(block=False)

    phoenix = None
    helios = None
    try:
        phoenix, helios = open_cameras(args)
        print("Streaming — close the Matplotlib window to stop.")

        while plt.fignum_exists(figure.number):
            try:
                rgb_frame, helios_frame = capture_paired_frames(
                    phoenix, helios, args.max_delta_sec, args.max_attempts
                )
            except RuntimeError as exc:
                print(f"[warn] {exc}")
                plt.pause(0.1)
                continue

            points_m, colors_rgb, projected_xy = colorize_point_cloud(
                helios_frame.xyz,
                rgb_frame.image,
                calibration["rgb_camera_matrix"],
                calibration["rgb_dist_coeffs"],
                calibration["R"],
                calibration["T"],
            )

            update_matplotlib_point_cloud(axes, points_m, colors_rgb, args.max_plot_points)
            figure.canvas.draw_idle()
            plt.pause(0.001)

            if args.preview:
                show_preview(rgb_frame.image, projected_xy)

        return 0
    finally:
        if phoenix is not None:
            phoenix.release()
        if helios is not None:
            helios.release()


if __name__ == "__main__":
    raise SystemExit(main())
