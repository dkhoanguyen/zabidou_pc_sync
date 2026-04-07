#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np

try:
    import open3d as o3d
except ImportError as exc:  # pragma: no cover - depends on local environment
    raise SystemExit(
        "open3d is required for this script. Install it in the current environment "
        "with `pip install open3d`."
    ) from exc

from show_stereo_point_cloud_open3d import (
    RADIUS_OUTLIER_NB_POINTS,
    RADIUS_OUTLIER_RADIUS_M,
    STATISTICAL_OUTLIER_NB_NEIGHBORS,
    STATISTICAL_OUTLIER_STD_RATIO,
    build_open3d_point_cloud,
    capture_paired_frames,
    load_stereo_calibration,
    maybe_filter_point_cloud,
    open_cameras,
)


REPO_ROOT = Path("/home/khoa/Projects/zabidou_pc_sync")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Capture one synchronized RGB/Helios frame pair, let the user choose a "
            "2D ROI on the Helios intensity or depth image, then display the cropped "
            "ROI point cloud with an optional bounding-box overlay in Open3D."
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
        "--disable-hdr",
        action="store_true",
        help="Disable Helios HDR mode. HDR is enabled by default for this script.",
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
        "--view",
        choices=["intensity", "depth"],
        default="intensity",
        help="Which Helios image to use for ROI selection before displaying the fused colored cloud.",
    )
    parser.add_argument(
        "--min-roi-points",
        type=int,
        default=300,
        help="Minimum number of valid 3D points required inside the selected ROI.",
    )
    parser.add_argument(
        "--remove-outliers",
        action="store_true",
        help="Apply the same Open3D denoising and outlier-removal filters as the stereo viewer.",
    )
    parser.add_argument(
        "--distance-threshold",
        type=float,
        default=0.01,
        help="Maximum inlier distance to the fitted plane, in meters.",
    )
    parser.add_argument(
        "--ransac-n",
        type=int,
        default=3,
        help="Number of points sampled per RANSAC iteration.",
    )
    parser.add_argument(
        "--num-iterations",
        type=int,
        default=1000,
        help="Number of RANSAC iterations used for plane fitting.",
    )
    parser.add_argument(
        "--min-plane-points",
        type=int,
        default=150,
        help="Minimum number of plane inliers required to accept the detected plane.",
    )
    parser.add_argument(
        "--min-object-points",
        type=int,
        default=50,
        help="Minimum number of non-plane points required to build an inflated object hull.",
    )
    parser.add_argument(
        "--inflate-distance",
        type=float,
        default=0.01,
        help="Distance in meters used to inflate the non-plane object region.",
    )
    parser.add_argument(
        "--object-outlier-mad-scale",
        type=float,
        default=3.0,
        help=(
            "Reject non-plane points whose distance from the object centroid is farther "
            "than median + scale * MAD before building the convex hull."
        ),
    )
    parser.add_argument(
        "--object-outlier-max-distance",
        type=float,
        default=0.0,
        help=(
            "Optional absolute maximum distance in meters from the object centroid. "
            "Use 0 to disable this extra cutoff."
        ),
    )
    parser.add_argument(
        "--output",
        default=None,
        help="Optional path to save the cropped ROI point cloud as a PLY file.",
    )
    parser.add_argument(
        "--show-ch",
        action="store_true",
        help="Overlay a simple bounding box centered on the convex-hull centroid.",
    )
    return parser.parse_args()


def build_intensity_view(intensity: np.ndarray) -> np.ndarray:
    intensity_8u = cv2.normalize(
        intensity,
        None,
        0,
        255,
        cv2.NORM_MINMAX,
        dtype=cv2.CV_8U,
    )
    return cv2.cvtColor(intensity_8u, cv2.COLOR_GRAY2BGR)


def build_depth_view(depth_mm: np.ndarray) -> np.ndarray:
    valid = depth_mm > 0
    if not np.any(valid):
        return np.zeros((*depth_mm.shape, 3), dtype=np.uint8)

    normalized = np.zeros_like(depth_mm, dtype=np.float32)
    depth_valid = depth_mm[valid]
    depth_min = float(depth_valid.min())
    depth_max = float(depth_valid.max())
    scale = max(depth_max - depth_min, 1.0)
    normalized[valid] = (depth_mm[valid] - depth_min) / scale
    depth_8u = np.clip(normalized * 255.0, 0, 255).astype(np.uint8)
    return cv2.applyColorMap(depth_8u, cv2.COLORMAP_TURBO)


def choose_roi(frame, view_name: str) -> tuple[int, int, int, int]:
    if view_name == "depth":
        preview = build_depth_view(frame.depth)
        window_title = "Select ROI on Helios depth image"
    else:
        preview = build_intensity_view(frame.intensity)
        window_title = "Select ROI on Helios intensity image"

    overlay = preview.copy()
    cv2.putText(
        overlay,
        "Drag a box and press ENTER or SPACE. Press C to cancel.",
        (10, 24),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.6,
        (255, 255, 255),
        2,
        cv2.LINE_AA,
    )
    roi = cv2.selectROI(window_title, overlay, showCrosshair=True, fromCenter=False)
    cv2.destroyWindow(window_title)
    x, y, w, h = map(int, roi)
    if w <= 0 or h <= 0:
        raise RuntimeError("No ROI selected.")
    return x, y, w, h


def choose_origin_point(frame) -> tuple[int, int]:
    preview = build_depth_view(frame.depth)
    window_title = "Select Origin On Helios Depth Image"
    selected = {"point": None}

    def on_mouse(event: int, x: int, y: int, _flags: int, _param) -> None:
        if event == cv2.EVENT_LBUTTONUP:
            selected["point"] = (int(x), int(y))

    cv2.namedWindow(window_title, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window_title, on_mouse)

    while True:
        overlay = preview.copy()
        cv2.putText(
            overlay,
            "Click origin point. Press ENTER or SPACE to confirm, C or ESC to cancel.",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.55,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        if selected["point"] is not None:
            px, py = selected["point"]
            cv2.drawMarker(
                overlay,
                (px, py),
                (0, 255, 255),
                markerType=cv2.MARKER_CROSS,
                markerSize=18,
                thickness=2,
            )
            cv2.circle(overlay, (px, py), 8, (0, 255, 255), 1)

        cv2.imshow(window_title, overlay)
        key = cv2.waitKey(20) & 0xFF
        if key in (13, 32):
            if selected["point"] is None:
                continue
            px, py = selected["point"]
            if frame.depth[py, px] <= 0:
                continue
            cv2.destroyWindow(window_title)
            return selected["point"]
        if key in (27, ord("c"), ord("C")):
            cv2.destroyWindow(window_title)
            raise RuntimeError("Origin point selection cancelled.")


def save_origin_point(path: Path, frame, point_xy: tuple[int, int]) -> None:
    x, y = point_xy
    depth_mm = float(frame.depth[y, x])
    xyz_mm = frame.xyz[y, x].astype(np.float64)

    path.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(path), cv2.FileStorage_WRITE)
    if not fs.isOpened():
        raise RuntimeError(f"Failed to open origin YAML file for writing: {path}")
    try:
        fs.write("origin_x_px", int(x))
        fs.write("origin_y_px", int(y))
        fs.write("origin_depth_mm", depth_mm)
        fs.write("origin_xyz_mm", xyz_mm.reshape(3, 1))
    finally:
        fs.release()


def write_center_data(
    path: Path,
    frame_index: int,
    origin_xy: tuple[int, int],
    object_center_xy: tuple[float, float],
    offset_xy: tuple[float, float],
    distance_px: float,
    distance_m: float,
    hull_center_xyz_m: np.ndarray,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(path), cv2.FileStorage_WRITE)
    if not fs.isOpened():
        raise RuntimeError(f"Failed to open center YAML file for writing: {path}")
    try:
        fs.write("status", "ok")
        fs.write("frame_index", int(frame_index))
        fs.write("origin_x_px", int(origin_xy[0]))
        fs.write("origin_y_px", int(origin_xy[1]))
        fs.write("center_x_px", float(object_center_xy[0]))
        fs.write("center_y_px", float(object_center_xy[1]))
        fs.write("offset_x_px", float(offset_xy[0]))
        fs.write("offset_y_px", float(offset_xy[1]))
        fs.write("distance_px", float(distance_px))
        fs.write("distance_m", float(distance_m))
        fs.write("center_xyz_m", hull_center_xyz_m.reshape(3, 1))
    finally:
        fs.release()


def write_center_error(path: Path, frame_index: int, message: str) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(path), cv2.FileStorage_WRITE)
    if not fs.isOpened():
        raise RuntimeError(f"Failed to open center YAML file for writing: {path}")
    try:
        fs.write("status", "error")
        fs.write("frame_index", int(frame_index))
        fs.write("message", message)
    finally:
        fs.release()


def colorize_roi_point_cloud(
    helios_xyz_mm: np.ndarray,
    rgb_image_bgr: np.ndarray,
    rgb_camera_matrix: np.ndarray,
    rgb_dist_coeffs: np.ndarray,
    R_rgb_to_depth: np.ndarray,
    T_rgb_to_depth_m: np.ndarray,
    roi: tuple[int, int, int, int],
) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
    x, y, w, h = roi
    valid_mask = helios_xyz_mm[:, :, 2] > 0
    roi_mask = np.zeros(valid_mask.shape, dtype=bool)
    roi_mask[y : y + h, x : x + w] = True
    selected_mask = valid_mask & roi_mask
    if not np.any(selected_mask):
        raise RuntimeError("The selected ROI contains no valid depth points.")

    selected_pixels_yx = np.argwhere(selected_mask)
    selected_pixels_xy = selected_pixels_yx[:, ::-1].astype(np.int32)
    points_depth_m = helios_xyz_mm[selected_mask].astype(np.float64) / 1000.0

    T_rgb_to_depth_m = T_rgb_to_depth_m.reshape(3)
    points_rgb_m = (R_rgb_to_depth.T @ (points_depth_m - T_rgb_to_depth_m).T).T

    in_front_mask = points_rgb_m[:, 2] > 0
    points_depth_m = points_depth_m[in_front_mask]
    points_rgb_m = points_rgb_m[in_front_mask]
    selected_pixels_xy = selected_pixels_xy[in_front_mask]
    if points_rgb_m.size == 0:
        raise RuntimeError("No ROI points remain in front of the RGB camera.")

    projected, _ = cv2.projectPoints(
        points_rgb_m.reshape(-1, 1, 3),
        np.zeros((3, 1), dtype=np.float64),
        np.zeros((3, 1), dtype=np.float64),
        rgb_camera_matrix,
        rgb_dist_coeffs,
    )
    projected = projected.reshape(-1, 2)

    height, width = rgb_image_bgr.shape[:2]
    u = np.round(projected[:, 0]).astype(np.int32)
    v = np.round(projected[:, 1]).astype(np.int32)
    inside_mask = (u >= 0) & (u < width) & (v >= 0) & (v < height)

    points_depth_m = points_depth_m[inside_mask]
    projected = projected[inside_mask]
    selected_pixels_xy = selected_pixels_xy[inside_mask]
    u = u[inside_mask]
    v = v[inside_mask]
    if points_depth_m.size == 0:
        raise RuntimeError("No ROI points project inside the RGB image.")

    colors_rgb = rgb_image_bgr[v, u][:, ::-1]
    return points_depth_m, colors_rgb, projected, selected_pixels_xy


def maybe_filter_point_cloud_and_pixels(
    point_cloud: o3d.geometry.PointCloud,
    pixels_xy: np.ndarray,
    remove_outliers: bool,
) -> tuple[o3d.geometry.PointCloud, np.ndarray]:
    if not remove_outliers:
        return point_cloud, pixels_xy

    filtered = point_cloud
    kept_indices = np.arange(len(pixels_xy))

    if STATISTICAL_OUTLIER_NB_NEIGHBORS > 0 and STATISTICAL_OUTLIER_STD_RATIO > 0:
        _, inlier_indices = filtered.remove_statistical_outlier(
            nb_neighbors=STATISTICAL_OUTLIER_NB_NEIGHBORS,
            std_ratio=STATISTICAL_OUTLIER_STD_RATIO,
        )
        inlier_indices = np.asarray(inlier_indices, dtype=np.int64)
        filtered = filtered.select_by_index(inlier_indices.tolist())
        kept_indices = kept_indices[inlier_indices]

    if RADIUS_OUTLIER_NB_POINTS > 0 and RADIUS_OUTLIER_RADIUS_M > 0:
        _, inlier_indices = filtered.remove_radius_outlier(
            nb_points=RADIUS_OUTLIER_NB_POINTS,
            radius=RADIUS_OUTLIER_RADIUS_M,
        )
        inlier_indices = np.asarray(inlier_indices, dtype=np.int64)
        filtered = filtered.select_by_index(inlier_indices.tolist())
        kept_indices = kept_indices[inlier_indices]

    return filtered, pixels_xy[kept_indices]


def save_point_cloud(path: str, point_cloud: o3d.geometry.PointCloud) -> None:
    if not o3d.io.write_point_cloud(path, point_cloud):
        raise RuntimeError(f"Failed to write point cloud to {path}")


def show_point_cloud(
    point_cloud: o3d.geometry.PointCloud,
    overlay_geometry: o3d.geometry.Geometry | None = None,
) -> None:
    geometries: list[o3d.geometry.Geometry] = [point_cloud]
    window_name = "Cropped ROI Point Cloud"
    if overlay_geometry is not None:
        geometries.append(overlay_geometry)
        window_name = "Cropped ROI Point Cloud With Bounding Box"
    o3d.visualization.draw_geometries(
        geometries,
        window_name=window_name,
        width=1280,
        height=720,
    )


def detect_plane(
    point_cloud: o3d.geometry.PointCloud,
    args: argparse.Namespace,
) -> tuple[o3d.geometry.PointCloud, o3d.geometry.PointCloud, np.ndarray, np.ndarray]:
    plane_model, inlier_indices = point_cloud.segment_plane(
        distance_threshold=args.distance_threshold,
        ransac_n=args.ransac_n,
        num_iterations=args.num_iterations,
    )
    if len(inlier_indices) < args.min_plane_points:
        raise RuntimeError(
            f"Detected plane has only {len(inlier_indices)} inliers; need at least "
            f"{args.min_plane_points}. Try choosing a tighter ROI or loosening the "
            "RANSAC thresholds."
        )
    inlier_indices = np.asarray(inlier_indices, dtype=np.int64)
    plane_cloud = point_cloud.select_by_index(inlier_indices)
    remainder_cloud = point_cloud.select_by_index(inlier_indices.tolist(), invert=True)
    return plane_cloud, remainder_cloud, plane_model, inlier_indices


def build_inflated_object_hull(
    point_cloud: o3d.geometry.PointCloud,
    pixels_xy: np.ndarray,
    min_object_points: int,
    inflate_distance: float,
    outlier_mad_scale: float,
    outlier_max_distance: float,
) -> tuple[o3d.geometry.TriangleMesh, np.ndarray]:
    if len(point_cloud.points) < min_object_points:
        raise RuntimeError(
            f"Only found {len(point_cloud.points)} non-plane points; need at least "
            f"{min_object_points} to build an inflated object hull."
        )

    filtered_cloud, filtered_pixels_xy = remove_far_object_outliers(
        point_cloud,
        pixels_xy,
        min_object_points=min_object_points,
        mad_scale=outlier_mad_scale,
        max_distance=outlier_max_distance,
    )

    hull_mesh, _ = filtered_cloud.compute_convex_hull()
    hull_mesh.compute_vertex_normals()

    vertices = np.asarray(hull_mesh.vertices)
    center = vertices.mean(axis=0)
    directions = vertices - center
    norms = np.linalg.norm(directions, axis=1, keepdims=True)
    valid = norms[:, 0] > 1e-9
    directions[valid] /= norms[valid]
    inflated_vertices = vertices.copy()
    inflated_vertices[valid] += directions[valid] * inflate_distance
    hull_mesh.vertices = o3d.utility.Vector3dVector(inflated_vertices)
    hull_mesh.compute_vertex_normals()
    hull_mesh.paint_uniform_color([1.0, 0.0, 0.0])
    return hull_mesh, filtered_pixels_xy


def build_centered_bounding_box(
    hull_mesh: o3d.geometry.TriangleMesh,
) -> o3d.geometry.LineSet:
    vertices = np.asarray(hull_mesh.vertices)
    if len(vertices) == 0:
        raise RuntimeError("Cannot build bounding box because the convex hull has no vertices.")

    centroid = vertices.mean(axis=0)
    half_extent = np.max(np.abs(vertices - centroid), axis=0)
    half_extent = np.maximum(half_extent, 1e-6)

    corners = np.array(
        [
            [-1, -1, -1],
            [1, -1, -1],
            [1, 1, -1],
            [-1, 1, -1],
            [-1, -1, 1],
            [1, -1, 1],
            [1, 1, 1],
            [-1, 1, 1],
        ],
        dtype=np.float64,
    )
    corners = centroid + corners * half_extent
    lines = [
        [0, 1], [1, 2], [2, 3], [3, 0],
        [4, 5], [5, 6], [6, 7], [7, 4],
        [0, 4], [1, 5], [2, 6], [3, 7],
    ]
    colors = [[1.0, 0.0, 0.0] for _ in lines]

    line_set = o3d.geometry.LineSet()
    line_set.points = o3d.utility.Vector3dVector(corners)
    line_set.lines = o3d.utility.Vector2iVector(lines)
    line_set.colors = o3d.utility.Vector3dVector(colors)
    return line_set


def remove_far_object_outliers(
    point_cloud: o3d.geometry.PointCloud,
    pixels_xy: np.ndarray,
    min_object_points: int,
    mad_scale: float,
    max_distance: float,
) -> tuple[o3d.geometry.PointCloud, np.ndarray]:
    points = np.asarray(point_cloud.points)
    if len(points) < min_object_points:
        return point_cloud, pixels_xy

    center = np.median(points, axis=0)
    distances = np.linalg.norm(points - center, axis=1)
    median_distance = float(np.median(distances))
    mad = float(np.median(np.abs(distances - median_distance)))

    keep_mask = np.ones(len(points), dtype=bool)
    if mad_scale > 0:
        if mad > 1e-9:
            keep_mask &= distances <= (median_distance + mad_scale * mad)
        else:
            keep_mask &= distances <= median_distance
    if max_distance > 0:
        keep_mask &= distances <= max_distance

    kept_indices = np.flatnonzero(keep_mask).tolist()
    if len(kept_indices) < min_object_points:
        return point_cloud, pixels_xy
    return point_cloud.select_by_index(kept_indices), pixels_xy[np.asarray(kept_indices, dtype=np.int64)]


def extract_points_inside_hull(
    source_cloud: o3d.geometry.PointCloud,
    hull_mesh: o3d.geometry.TriangleMesh,
    epsilon: float = 1e-6,
) -> o3d.geometry.PointCloud:
    points = np.asarray(source_cloud.points)
    if len(points) == 0:
        raise RuntimeError("Source point cloud is empty.")

    vertices = np.asarray(hull_mesh.vertices)
    triangles = np.asarray(hull_mesh.triangles)
    if len(vertices) == 0 or len(triangles) == 0:
        raise RuntimeError("Convex hull mesh is empty.")

    hull_center = vertices.mean(axis=0)
    inside_mask = np.ones(len(points), dtype=bool)

    for tri in triangles:
        p0, p1, p2 = vertices[tri]
        normal = np.cross(p1 - p0, p2 - p0)
        norm = np.linalg.norm(normal)
        if norm <= 1e-12:
            continue
        normal = normal / norm
        if np.dot(normal, hull_center - p0) > 0:
            normal = -normal
        signed_distance = (points - p0) @ normal
        inside_mask &= signed_distance <= epsilon

    inside_indices = np.flatnonzero(inside_mask).tolist()
    if not inside_indices:
        raise RuntimeError("No ROI points remain inside the convex hull.")
    return source_cloud.select_by_index(inside_indices)


def compute_pixel_offset(
    origin_xy: tuple[int, int],
    object_pixels_xy: np.ndarray,
) -> tuple[tuple[float, float], tuple[float, float]]:
    if len(object_pixels_xy) == 0:
        raise RuntimeError("Cannot compute pixel offset because the object has no retained pixels.")
    object_center_xy = object_pixels_xy.astype(np.float64).mean(axis=0)
    origin_xy_array = np.asarray(origin_xy, dtype=np.float64)
    offset_xy = object_center_xy - origin_xy_array
    return (float(object_center_xy[0]), float(object_center_xy[1])), (
        float(offset_xy[0]),
        float(offset_xy[1]),
    )


def compute_hull_center_xyz(object_hull: o3d.geometry.TriangleMesh) -> np.ndarray:
    vertices = np.asarray(object_hull.vertices)
    if len(vertices) == 0:
        raise RuntimeError("Cannot compute hull center because the convex hull has no vertices.")
    return vertices.mean(axis=0)


def main() -> int:
    args = parse_args()
    calibration = load_stereo_calibration(args.stereo_yaml)

    phoenix = None
    helios = None
    try:
        phoenix, helios = open_cameras(args)

        print("Capturing one synchronized RGB/Helios pair...")
        rgb_frame, helios_frame = capture_paired_frames(
            phoenix, helios, args.max_delta_sec, args.max_attempts
        )

        print(f"Choose a bounding box on the {args.view} image.")
        roi = choose_roi(helios_frame, args.view)
        x, y, w, h = roi
        print(f"Selected ROI: x={x}, y={y}, w={w}, h={h}")

        points_m, colors_rgb_u8, _projected_xy, roi_pixels_xy = colorize_roi_point_cloud(
            helios_frame.xyz,
            rgb_frame.image,
            calibration["rgb_camera_matrix"],
            calibration["rgb_dist_coeffs"],
            calibration["R"],
            calibration["T"],
            roi,
        )
        roi_cloud = build_open3d_point_cloud(points_m, colors_rgb_u8)
        roi_point_count = len(points_m)
        if roi_point_count < args.min_roi_points:
            raise RuntimeError(
                f"Only found {roi_point_count} valid 3D points in the ROI; need at least "
                f"{args.min_roi_points}."
            )

        roi_cloud, roi_pixels_xy = maybe_filter_point_cloud_and_pixels(
            roi_cloud,
            roi_pixels_xy,
            args.remove_outliers,
        )
        plane_cloud, non_plane_cloud, plane_model, plane_inlier_indices = detect_plane(
            roi_cloud,
            args,
        )
        non_plane_mask = np.ones(len(roi_pixels_xy), dtype=bool)
        non_plane_mask[plane_inlier_indices] = False
        non_plane_pixels_xy = roi_pixels_xy[non_plane_mask]
        object_hull, _object_pixels_xy = build_inflated_object_hull(
            non_plane_cloud,
            non_plane_pixels_xy,
            args.min_object_points,
            args.inflate_distance,
            args.object_outlier_mad_scale,
            args.object_outlier_max_distance,
        )
        object_box = build_centered_bounding_box(object_hull)

        if args.output is not None:
            save_point_cloud(args.output, roi_cloud)
            print(f"Saved cropped ROI point cloud to {args.output}")

        a, b, c, d = plane_model
        print(
            f"Plane inliers: {len(plane_cloud.points):,} | "
            f"Non-plane points: {len(non_plane_cloud.points):,} | "
            f"Plane: {a:.4f}x + {b:.4f}y + {c:.4f}z + {d:.4f} = 0"
        )
        if args.show_ch:
            print("Displaying the cropped ROI point cloud with the centered bounding box overlay in Open3D.")
            show_point_cloud(roi_cloud, object_box)
        else:
            print("Displaying the cropped ROI point cloud in Open3D.")
            show_point_cloud(roi_cloud)
        return 0
    finally:
        cv2.destroyAllWindows()
        if phoenix is not None:
            phoenix.release()
        if helios is not None:
            helios.release()


if __name__ == "__main__":
    raise SystemExit(main())
