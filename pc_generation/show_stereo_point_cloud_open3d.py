#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np
from arena_api.system import system

try:
    import open3d as o3d
except ImportError as exc:  # pragma: no cover - depends on local environment
    raise SystemExit(
        "open3d is required for this script. Install it in the current environment "
        "with `pip install open3d`."
    ) from exc


REPO_ROOT = Path("/home/khoa/Projects/zabidou_pc_sync")
PC_GENERATION_ROOT = REPO_ROOT / "pc_generation"
if str(PC_GENERATION_ROOT) not in sys.path:
    sys.path.insert(0, str(PC_GENERATION_ROOT))

from src.helios2 import Helios2Camera
from src.phoenix import PhoenixCamera


VOXEL_SIZE_M = 0.0
STATISTICAL_OUTLIER_NB_NEIGHBORS = 10
STATISTICAL_OUTLIER_STD_RATIO = 5.0
RADIUS_OUTLIER_NB_POINTS = 10
RADIUS_OUTLIER_RADIUS_M = 0.01
HELIOS_OPERATING_MODE = "Distance5000mmMultiFreq"
HELIOS_HDR_MODE = "LowNoiseHDRX8"
HELIOS_EXPOSURE_TIME_SELECTOR = "Exp1000Us"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture one Phoenix RGB frame and one Helios depth frame, colorize the "
            "Helios point cloud with the stereo calibration, and display it in Open3D."
        )
    )
    parser.add_argument(
        "--stereo-yaml",
        type=Path,
        default=REPO_ROOT / "calibration/results/combined_all_datasets/stereo_calibration.yaml",
        help="Stereo calibration YAML path.",
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
        default=0.5,
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
        help="Show an RGB overlay preview of projected Helios points before the Open3D viewer.",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=150000,
        help="Maximum number of points to show in Open3D.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Optional path to save the colored point cloud as a PLY file.",
    )
    parser.add_argument(
        "--stream",
        action="store_true",
        help="Continuously capture and update the point cloud in the Open3D viewer.",
    )
    parser.add_argument(
        "--disable-hdr",
        action="store_true",
        help="Disable Helios HDR mode. HDR is enabled by default for this script.",
    )
    parser.add_argument(
        "--remove-outliers",
        action="store_true",
        help="Apply the built-in Open3D denoising and outlier-removal filters.",
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
        binning=1,
        binning_mode="Average",
    )
    helios = Helios2Camera(
        helios_device,
        pixel_format=args.helios_format,
        hdr_mode="Off" if args.disable_hdr else HELIOS_HDR_MODE,
        operating_mode=HELIOS_OPERATING_MODE,
        exposure_time_selector=HELIOS_EXPOSURE_TIME_SELECTOR,
    )

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


def maybe_downsample(
    points_m: np.ndarray,
    colors_rgb: np.ndarray,
    max_points: int,
) -> tuple[np.ndarray, np.ndarray]:
    if len(points_m) <= max_points:
        return points_m, colors_rgb

    step = max(1, len(points_m) // max_points)
    return points_m[::step], colors_rgb[::step]


def show_preview(rgb_image_bgr: np.ndarray, projected_xy: np.ndarray) -> None:
    preview = rgb_image_bgr.copy()
    for x, y in projected_xy[:: max(1, len(projected_xy) // 5000)]:
        cv2.circle(preview, (int(round(x)), int(round(y))), 1, (0, 255, 0), -1)
    cv2.imshow("RGB overlay", preview)
    cv2.waitKey(0)
    cv2.destroyAllWindows()


def build_open3d_point_cloud(points_m: np.ndarray, colors_rgb: np.ndarray) -> o3d.geometry.PointCloud:
    point_cloud = o3d.geometry.PointCloud()
    point_cloud.points = o3d.utility.Vector3dVector(points_m.astype(np.float64))
    point_cloud.colors = o3d.utility.Vector3dVector(
        colors_rgb.astype(np.float64) / 255.0
    )
    return point_cloud


def save_point_cloud(path: Path, point_cloud: o3d.geometry.PointCloud) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    if not o3d.io.write_point_cloud(str(path), point_cloud):
        raise RuntimeError(f"Failed to write point cloud to {path}")


def filter_point_cloud(point_cloud: o3d.geometry.PointCloud) -> o3d.geometry.PointCloud:
    filtered = point_cloud

    if VOXEL_SIZE_M > 0:
        filtered = filtered.voxel_down_sample(voxel_size=VOXEL_SIZE_M)

    if STATISTICAL_OUTLIER_NB_NEIGHBORS > 0 and STATISTICAL_OUTLIER_STD_RATIO > 0:
        _, inlier_indices = filtered.remove_statistical_outlier(
            nb_neighbors=STATISTICAL_OUTLIER_NB_NEIGHBORS,
            std_ratio=STATISTICAL_OUTLIER_STD_RATIO,
        )
        filtered = filtered.select_by_index(inlier_indices)

    if RADIUS_OUTLIER_NB_POINTS > 0 and RADIUS_OUTLIER_RADIUS_M > 0:
        _, inlier_indices = filtered.remove_radius_outlier(
            nb_points=RADIUS_OUTLIER_NB_POINTS,
            radius=RADIUS_OUTLIER_RADIUS_M,
        )
        filtered = filtered.select_by_index(inlier_indices)

    return filtered


def maybe_filter_point_cloud(
    point_cloud: o3d.geometry.PointCloud,
    remove_outliers: bool,
) -> o3d.geometry.PointCloud:
    if not remove_outliers:
        return point_cloud
    return filter_point_cloud(point_cloud)


def show_open3d_snapshot(point_cloud: o3d.geometry.PointCloud) -> None:
    o3d.visualization.draw_geometries(
        [point_cloud],
        window_name="Stereo Colored Point Cloud",
        width=1280,
        height=720,
    )


def stream_open3d_point_cloud(
    phoenix: PhoenixCamera,
    helios: Helios2Camera,
    calibration: dict[str, np.ndarray],
    args: argparse.Namespace,
) -> None:
    visualizer = o3d.visualization.Visualizer()
    if not visualizer.create_window(
        window_name="Stereo Colored Point Cloud Stream",
        width=1280,
        height=720,
    ):
        raise RuntimeError("Failed to create the Open3D visualization window.")

    point_cloud = o3d.geometry.PointCloud()
    visualizer.add_geometry(point_cloud)
    render_option = visualizer.get_render_option()
    if render_option is not None:
        render_option.point_size = 1.0

    captured_once = False
    try:
        while True:
            try:
                rgb_frame, helios_frame = capture_paired_frames(
                    phoenix, helios, args.max_delta_sec, args.max_attempts
                )
                points_m, colors_rgb, projected_xy = colorize_point_cloud(
                    helios_frame.xyz,
                    rgb_frame.image,
                    calibration["rgb_camera_matrix"],
                    calibration["rgb_dist_coeffs"],
                    calibration["R"],
                    calibration["T"],
                )
            except RuntimeError as exc:
                print(f"[warn] {exc}")
                if not visualizer.poll_events():
                    break
                visualizer.update_renderer()
                continue

            points_m, colors_rgb = maybe_downsample(points_m, colors_rgb, args.max_points)
            filtered_point_cloud = maybe_filter_point_cloud(
                build_open3d_point_cloud(points_m, colors_rgb),
                args.remove_outliers,
            )
            point_cloud.points = filtered_point_cloud.points
            point_cloud.colors = filtered_point_cloud.colors

            if args.output is not None and not captured_once:
                save_point_cloud(args.output, point_cloud)
                print(f"Saved first streamed point cloud to {args.output}")
                captured_once = True

            if args.preview:
                show_preview(rgb_frame.image, projected_xy)

            visualizer.update_geometry(point_cloud)
            if not visualizer.poll_events():
                break
            visualizer.update_renderer()
    finally:
        visualizer.destroy_window()


def main() -> int:
    args = parse_args()
    calibration = load_stereo_calibration(args.stereo_yaml)

    phoenix = None
    helios = None
    try:
        phoenix, helios = open_cameras(args)
        if args.stream:
            print("Streaming RGB/Helios point cloud in Open3D. Close the viewer window to stop.")
            stream_open3d_point_cloud(phoenix, helios, calibration, args)
            return 0

        print("Capturing one RGB/Helios pair...")
        rgb_frame, helios_frame = capture_paired_frames(
            phoenix, helios, args.max_delta_sec, args.max_attempts
        )

        print("Colorizing Helios point cloud with Phoenix RGB...")
        points_m, colors_rgb, projected_xy = colorize_point_cloud(
            helios_frame.xyz,
            rgb_frame.image,
            calibration["rgb_camera_matrix"],
            calibration["rgb_dist_coeffs"],
            calibration["R"],
            calibration["T"],
        )

        points_m, colors_rgb = maybe_downsample(points_m, colors_rgb, args.max_points)
        point_cloud = maybe_filter_point_cloud(
            build_open3d_point_cloud(points_m, colors_rgb),
            args.remove_outliers,
        )

        if args.output is not None:
            save_point_cloud(args.output, point_cloud)
            print(f"Saved colored point cloud to {args.output}")

        if args.preview:
            show_preview(rgb_frame.image, projected_xy)

        print(f"Displaying {len(points_m):,} colored points in Open3D.")
        show_open3d_snapshot(point_cloud)
        return 0
    finally:
        if phoenix is not None:
            phoenix.release()
        if helios is not None:
            helios.release()


if __name__ == "__main__":
    raise SystemExit(main())
