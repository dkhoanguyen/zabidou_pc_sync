#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Run checkerboard detection on a single image across many board sizes."
    )
    parser.add_argument(
        "image",
        type=Path,
        help="Path to the input image.",
    )
    parser.add_argument(
        "--min-cols",
        type=int,
        default=3,
        help="Minimum number of inner corners along the checkerboard width.",
    )
    parser.add_argument(
        "--max-cols",
        type=int,
        default=12,
        help="Maximum number of inner corners along the checkerboard width.",
    )
    parser.add_argument(
        "--min-rows",
        type=int,
        default=3,
        help="Minimum number of inner corners along the checkerboard height.",
    )
    parser.add_argument(
        "--max-rows",
        type=int,
        default=12,
        help="Maximum number of inner corners along the checkerboard height.",
    )
    parser.add_argument(
        "--max",
        type=int,
        default=10,
        help="Maximum number of checkerboard polygon ROIs to select and process.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Optional path to save the visualization image.",
    )
    parser.add_argument(
        "--show",
        action="store_true",
        help="Show the detection result in an OpenCV window.",
    )
    parser.add_argument(
        "--roi-output",
        type=Path,
        default=None,
        help="Optional path prefix to save extracted ROI images.",
    )
    parser.add_argument(
        "--points-output",
        type=Path,
        default=None,
        help="Optional `.npz` path to save object/image point correspondences for calibration.",
    )
    parser.add_argument(
        "--offset",
        type=int,
        default=0,
        help="Integer offset added to all annotated object-point coordinates.",
    )
    return parser.parse_args()


def draw_axis_frame_overlay(
    image: np.ndarray,
    axis_frame: dict[str, tuple[int, int]] | None,
) -> np.ndarray:
    overlay = image.copy()
    if axis_frame is None:
        return overlay

    origin = axis_frame["origin"]
    colors = {
        "x": (0, 0, 255),
        "y": (0, 255, 0),
        "z": (255, 0, 0),
    }

    cv2.circle(overlay, origin, 6, (0, 255, 255), -1)
    cv2.putText(
        overlay,
        "O",
        (origin[0] + 6, origin[1] - 6),
        cv2.FONT_HERSHEY_SIMPLEX,
        0.7,
        (0, 255, 255),
        2,
        cv2.LINE_AA,
    )

    for axis_name in ("x", "y", "z"):
        point = axis_frame[axis_name]
        cv2.arrowedLine(
            overlay,
            origin,
            point,
            colors[axis_name],
            2,
            cv2.LINE_AA,
            tipLength=0.08,
        )
        cv2.circle(overlay, point, 5, colors[axis_name], -1)
        cv2.putText(
            overlay,
            axis_name.upper(),
            (point[0] + 6, point[1] - 6),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            colors[axis_name],
            2,
            cv2.LINE_AA,
        )

    return overlay


def select_axis_frame(image: np.ndarray) -> dict[str, tuple[int, int]]:
    window_name = "Select Axis Frame"
    labels = ["origin", "x", "y", "z"]
    points: list[tuple[int, int]] = []

    def on_mouse(event: int, x: int, y: int, _flags: int, _param) -> None:
        if event == cv2.EVENT_LBUTTONUP and len(points) < 4:
            points.append((int(x), int(y)))
        elif event == cv2.EVENT_RBUTTONUP and points:
            points.pop()

    cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
    cv2.setMouseCallback(window_name, on_mouse)

    while True:
        overlay = image.copy()
        for idx, point in enumerate(points):
            label = labels[idx]
            color = {
                "origin": (0, 255, 255),
                "x": (0, 0, 255),
                "y": (0, 255, 0),
                "z": (255, 0, 0),
            }[label]
            cv2.circle(overlay, point, 5, color, -1)
            cv2.putText(
                overlay,
                label.upper(),
                (point[0] + 6, point[1] - 6),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.65,
                color,
                2,
                cv2.LINE_AA,
            )
        if len(points) == 4:
            axis_frame = {
                "origin": points[0],
                "x": points[1],
                "y": points[2],
                "z": points[3],
            }
            overlay = draw_axis_frame_overlay(overlay, axis_frame)

        cv2.putText(
            overlay,
            "Click origin, +X, +Y, +Z. Right click undo.",
            (10, 24),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.putText(
            overlay,
            "Press Enter/Space to confirm after all 4 points are set.",
            (10, 50),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (255, 255, 255),
            2,
            cv2.LINE_AA,
        )
        cv2.imshow(window_name, overlay)
        key = cv2.waitKey(20) & 0xFF
        if key in (13, 32) and len(points) == 4:
            axis_frame = {
                "origin": points[0],
                "x": points[1],
                "y": points[2],
                "z": points[3],
            }
            cv2.destroyWindow(window_name)
            return axis_frame
        if key in (27, ord("c"), ord("C")):
            cv2.destroyWindow(window_name)
            raise SystemExit("Axis frame selection cancelled.")


def select_polygon_rois(image: np.ndarray, max_count: int) -> list[np.ndarray]:
    rois: list[np.ndarray] = []
    window_name = "Select Checkerboard Polygon ROI"

    for index in range(max_count):
        points: list[tuple[int, int]] = []

        def on_mouse(event: int, x: int, y: int, _flags: int, _param) -> None:
            if event == cv2.EVENT_LBUTTONUP and len(points) < 4:
                points.append((int(x), int(y)))
            elif event == cv2.EVENT_RBUTTONUP and points:
                points.pop()

        cv2.namedWindow(window_name, cv2.WINDOW_AUTOSIZE)
        cv2.setMouseCallback(window_name, on_mouse)

        while True:
            overlay = image.copy()
            for roi_idx, polygon in enumerate(rois, start=1):
                cv2.polylines(overlay, [polygon], isClosed=True, color=(255, 0, 255), thickness=2)
                x, y, w, h = cv2.boundingRect(polygon)
                cv2.putText(
                    overlay,
                    f"ROI {roi_idx}",
                    (x, max(20, y - 8)),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.7,
                    (255, 0, 255),
                    2,
                    cv2.LINE_AA,
                )

            for point_idx, point in enumerate(points, start=1):
                cv2.circle(overlay, point, 5, (0, 255, 255), -1)
                cv2.putText(
                    overlay,
                    str(point_idx),
                    (point[0] + 6, point[1] - 6),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.6,
                    (0, 255, 255),
                    2,
                    cv2.LINE_AA,
                )
            if len(points) >= 2:
                cv2.polylines(
                    overlay,
                    [np.array(points, dtype=np.int32)],
                    isClosed=(len(points) == 4),
                    color=(0, 255, 255),
                    thickness=2,
                )

            cv2.putText(
                overlay,
                f"Select 4 points for ROI {index + 1}/{max_count}. Enter/Space confirm, C/Esc stop.",
                (10, 24),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )
            cv2.putText(
                overlay,
                "Left click: add point | Right click: undo",
                (10, 50),
                cv2.FONT_HERSHEY_SIMPLEX,
                0.55,
                (255, 255, 255),
                2,
                cv2.LINE_AA,
            )

            cv2.imshow(window_name, overlay)
            key = cv2.waitKey(20) & 0xFF
            if key in (13, 32):
                if len(points) == 4:
                    rois.append(np.array(points, dtype=np.int32))
                    break
            if key in (27, ord("c"), ord("C")):
                cv2.destroyWindow(window_name)
                return rois

        cv2.destroyWindow(window_name)

    return rois


def extract_polygon_roi(
    image: np.ndarray,
    polygon: np.ndarray,
) -> tuple[np.ndarray, tuple[int, int], np.ndarray]:
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    cv2.fillPoly(mask, [polygon], 255)
    x, y, w, h = cv2.boundingRect(polygon)
    cropped_image = image[y : y + h, x : x + w].copy()
    cropped_mask = mask[y : y + h, x : x + w]
    cropped_image[cropped_mask == 0] = 0
    return cropped_image, (x, y), cropped_mask


def find_checkerboard(
    gray: np.ndarray,
    pattern_size: tuple[int, int],
) -> tuple[bool, np.ndarray | None]:
    if hasattr(cv2, "findChessboardCornersSB"):
        flags = cv2.CALIB_CB_NORMALIZE_IMAGE | cv2.CALIB_CB_EXHAUSTIVE
        found, corners = cv2.findChessboardCornersSB(gray, pattern_size, flags)
        return found, corners if found else None

    flags = (
        cv2.CALIB_CB_ADAPTIVE_THRESH
        | cv2.CALIB_CB_NORMALIZE_IMAGE
        | cv2.CALIB_CB_FAST_CHECK
    )
    found, corners = cv2.findChessboardCorners(gray, pattern_size, flags)
    if not found:
        return False, None
    criteria = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        30,
        0.001,
    )
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    return True, corners


def generate_pattern_sizes(args: argparse.Namespace) -> list[tuple[int, int]]:
    sizes: list[tuple[int, int]] = []
    for cols in range(args.min_cols, args.max_cols + 1):
        for rows in range(args.min_rows, args.max_rows + 1):
            sizes.append((cols, rows))
    sizes.sort(key=lambda size: size[0] * size[1], reverse=True)
    return sizes


def mask_detection(gray: np.ndarray, corners: np.ndarray) -> np.ndarray:
    masked = gray.copy()
    pts = corners.reshape(-1, 2).astype(np.float32)
    x, y, w, h = cv2.boundingRect(pts)
    pad = 10
    x0 = max(0, x - pad)
    y0 = max(0, y - pad)
    x1 = min(masked.shape[1], x + w + pad)
    y1 = min(masked.shape[0], y + h + pad)
    masked[y0:y1, x0:x1] = 127
    return masked


def detect_all_checkerboards(
    gray: np.ndarray,
    pattern_sizes: list[tuple[int, int]],
) -> list[dict[str, object]]:
    working_gray = gray.copy()
    detections: list[dict[str, object]] = []

    best_detection: dict[str, object] | None = None
    for pattern_size in pattern_sizes:
        found, corners = find_checkerboard(working_gray, pattern_size)
        if not found or corners is None:
            continue
        score = int(pattern_size[0] * pattern_size[1])
        best_detection = {
            "pattern_size": pattern_size,
            "corners": corners,
            "score": score,
        }
        break

    if best_detection is not None:
        detections.append(best_detection)
    return detections


def orientation_candidates(grid: np.ndarray) -> list[np.ndarray]:
    return [
        grid,
        grid[:, ::-1, :],
        grid[::-1, :, :],
        grid[::-1, ::-1, :],
    ]


def score_oriented_grid(
    grid: np.ndarray,
    axis_frame: dict[str, tuple[int, int]] | None,
    plane_name: str,
) -> tuple[float, float]:
    if axis_frame is None:
        return 0.0, 0.0

    rows, cols = grid.shape[:2]
    origin = np.asarray(axis_frame["origin"], dtype=np.float32)
    axis_vectors = {
        "x": normalize_vector(np.asarray(axis_frame["x"], dtype=np.float32) - origin),
        "y": normalize_vector(np.asarray(axis_frame["y"], dtype=np.float32) - origin),
        "z": normalize_vector(np.asarray(axis_frame["z"], dtype=np.float32) - origin),
    }
    plane_axes = {
        "xy": ("x", "y"),
        "xz": ("x", "z"),
        "yz": ("y", "z"),
    }
    col_axis, row_axis = plane_axes.get(plane_name, ("x", "y"))

    candidate_origin = grid[0, 0]
    if cols > 1:
        col_vector = normalize_vector(grid[0, 1] - candidate_origin)
    else:
        col_vector = np.zeros(2, dtype=np.float32)
    if rows > 1:
        row_vector = normalize_vector(grid[1, 0] - candidate_origin)
    else:
        row_vector = np.zeros(2, dtype=np.float32)

    origin_distance = float(np.linalg.norm(candidate_origin - origin))
    alignment = float(np.dot(col_vector, axis_vectors[col_axis]))
    alignment += float(np.dot(row_vector, axis_vectors[row_axis]))
    return origin_distance, alignment


def detect_best_checkerboard_for_plane(
    gray: np.ndarray,
    pattern_sizes: list[tuple[int, int]],
    axis_frame: dict[str, tuple[int, int]] | None,
    plane_name: str,
) -> list[dict[str, object]]:
    detections: list[dict[str, object]] = []
    for pattern_size in pattern_sizes:
        found, corners = find_checkerboard(gray, pattern_size)
        if not found or corners is None:
            continue
        detections.append(
            {
                "pattern_size": pattern_size,
                "corners": corners,
                "score": int(pattern_size[0] * pattern_size[1]),
            }
        )

    if not detections:
        return []

    def rank_detection(detection: dict[str, object]) -> tuple[int, float, float]:
        pattern_size = detection["pattern_size"]
        corners = detection["corners"]
        assert isinstance(pattern_size, tuple)
        assert isinstance(corners, np.ndarray)
        oriented = orient_corner_grid_for_plane(corners, pattern_size, axis_frame, plane_name)
        origin_distance, alignment = score_oriented_grid(oriented, axis_frame, plane_name)
        score = int(detection["score"])
        return score, -origin_distance, alignment

    best_detection = max(detections, key=rank_detection)
    return [best_detection]


def draw_detections(image: np.ndarray, detections: list[dict[str, object]]) -> np.ndarray:
    vis = image.copy()
    for detection in detections:
        pattern_size = detection["pattern_size"]
        corners = detection["corners"]
        axis_frame = detection.get("axis_frame")
        plane_name = str(detection.get("plane_name", "xy"))
        offset = int(detection.get("offset", 0))
        assert isinstance(pattern_size, tuple)
        assert isinstance(corners, np.ndarray)
        cv2.drawChessboardCorners(vis, pattern_size, corners, True)
        grid = orient_corner_grid_for_plane(corners, pattern_size, axis_frame, plane_name)
        cols, rows = pattern_size

        for row in range(rows):
            for col in range(cols):
                point = grid[row, col]
                px = int(round(float(point[0])))
                py = int(round(float(point[1])))
                object_point = object_point_for_plane(plane_name, col, row, cols, rows, offset)
                label = f"({object_point[0]},{object_point[1]},{object_point[2]})"
                cv2.circle(vis, (px, py), 3, (0, 0, 255), -1)
                cv2.putText(
                    vis,
                    label,
                    (px + 4, py - 4),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    0.35,
                    (0, 0, 255),
                    1,
                    cv2.LINE_AA,
                )
    return vis


def offset_detections(
    detections: list[dict[str, object]],
    offset_xy: tuple[int, int],
    axis_frame: dict[str, tuple[int, int]] | None = None,
    plane_name: str = "xy",
    object_point_offset: int = 0,
) -> list[dict[str, object]]:
    ox, oy = offset_xy
    shifted: list[dict[str, object]] = []
    for detection in detections:
        corners = detection["corners"]
        assert isinstance(corners, np.ndarray)
        shifted_axis_frame = None
        if axis_frame is not None:
            shifted_axis_frame = {
                key: (int(value[0]), int(value[1]))
                for key, value in axis_frame.items()
            }
        shifted.append(
            {
                "pattern_size": detection["pattern_size"],
                "corners": corners + np.array([[[ox, oy]]], dtype=np.float32),
                "score": detection["score"],
                "axis_frame": shifted_axis_frame,
                "plane_name": plane_name,
                "offset": object_point_offset,
            }
        )
    return shifted


def plane_name_for_index(index: int) -> str:
    if index == 1:
        return "xy"
    if index == 2:
        return "xz"
    if index == 3:
        return "yz"
    return "xy"


def normalize_vector(vector: np.ndarray) -> np.ndarray:
    norm = float(np.linalg.norm(vector))
    if norm <= 1e-9:
        return np.zeros_like(vector, dtype=np.float32)
    return (vector / norm).astype(np.float32)


def orient_corner_grid_for_plane(
    corners: np.ndarray,
    pattern_size: tuple[int, int],
    axis_frame: dict[str, tuple[int, int]] | None,
    plane_name: str,
) -> np.ndarray:
    pts = corners.reshape(-1, 2).astype(np.float32)
    cols, rows = pattern_size
    grid = pts.reshape(rows, cols, 2)

    if axis_frame is None:
        return grid

    candidates = orientation_candidates(grid)
    candidate_infos: list[tuple[float, float, np.ndarray]] = []

    for candidate in candidates:
        origin_distance, alignment = score_oriented_grid(candidate, axis_frame, plane_name)
        candidate_infos.append((origin_distance, alignment, candidate.copy()))

    min_origin_distance = min(info[0] for info in candidate_infos)
    distance_tolerance_px = 25.0
    near_origin_candidates = [
        info for info in candidate_infos if info[0] <= min_origin_distance + distance_tolerance_px
    ]
    best_origin_distance, best_alignment, best_grid = max(
        near_origin_candidates,
        key=lambda info: info[1],
    )
    return best_grid


def object_point_for_plane(
    plane_name: str,
    col: int,
    row: int,
    cols: int,
    rows: int,
    offset: int = 0,
) -> tuple[int, int, int]:
    if plane_name == "xy":
        return col + offset, row + offset, 0 + offset
    if plane_name == "xz":
        return col + offset, 0 + offset, row + offset
    if plane_name == "yz":
        return 0 + offset, row + offset, col + offset
    return col + offset, row + offset, 0 + offset


def build_calibration_arrays(
    detections: list[dict[str, object]],
) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    object_points: list[tuple[float, float, float]] = []
    image_points: list[tuple[float, float]] = []
    plane_ids: list[str] = []

    for detection in detections:
        pattern_size = detection["pattern_size"]
        corners = detection["corners"]
        axis_frame = detection.get("axis_frame")
        plane_name = str(detection.get("plane_name", "xy"))
        offset = int(detection.get("offset", 0))
        assert isinstance(pattern_size, tuple)
        assert isinstance(corners, np.ndarray)

        cols, rows = pattern_size
        grid = orient_corner_grid_for_plane(corners, pattern_size, axis_frame, plane_name)
        for row in range(rows):
            for col in range(cols):
                point = grid[row, col]
                object_point = object_point_for_plane(plane_name, col, row, cols, rows, offset)
                object_points.append(
                    (float(object_point[0]), float(object_point[1]), float(object_point[2]))
                )
                image_points.append((float(point[0]), float(point[1])))
                plane_ids.append(plane_name)

    return (
        np.asarray(object_points, dtype=np.float32),
        np.asarray(image_points, dtype=np.float32),
        np.asarray(plane_ids),
    )


def save_calibration_arrays(
    path: Path,
    object_points_xyz: np.ndarray,
    image_points_xy: np.ndarray,
    plane_ids: np.ndarray,
) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        path,
        object_points_xyz=object_points_xyz,
        image_points_xy=image_points_xy,
        plane_ids=plane_ids,
    )


def main() -> int:
    args = parse_args()

    if not args.image.is_file():
        raise SystemExit(f"Image not found: {args.image}")

    image = cv2.imread(str(args.image), cv2.IMREAD_COLOR)
    if image is None:
        raise SystemExit(f"Failed to read image: {args.image}")

    if args.min_cols > args.max_cols or args.min_rows > args.max_rows:
        raise SystemExit("Invalid checkerboard search range.")

    axis_frame = select_axis_frame(image)
    pattern_sizes = generate_pattern_sizes(args)
    rois = select_polygon_rois(image, args.max)
    detections: list[dict[str, object]] = []
    vis = draw_axis_frame_overlay(image, axis_frame)

    if not rois:
        raise SystemExit("No ROI selected.")

    for idx, roi in enumerate(rois, start=1):
        roi_image, roi_offset_xy, roi_mask = extract_polygon_roi(image, roi)
        gray = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY)
        gray[roi_mask == 0] = 127
        plane_name = plane_name_for_index(idx)
        roi_detections = detect_best_checkerboard_for_plane(
            gray,
            pattern_sizes,
            axis_frame,
            plane_name,
        )
        shifted = offset_detections(
            roi_detections,
            roi_offset_xy,
            axis_frame=axis_frame,
            plane_name=plane_name,
            object_point_offset=args.offset,
        )
        detections.extend(shifted)

        x, y, w, h = cv2.boundingRect(roi)
        cv2.polylines(vis, [roi], isClosed=True, color=(255, 0, 255), thickness=2)
        cv2.putText(
            vis,
            f"ROI {idx}",
            (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )

        if args.roi_output is not None:
            roi_output_path = args.roi_output.with_name(
                f"{args.roi_output.stem}_{idx}{args.roi_output.suffix or '.png'}"
            )
            roi_output_path.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(roi_output_path), roi_image):
                raise SystemExit(f"Failed to write ROI image: {roi_output_path}")
            print(f"Saved ROI image to {roi_output_path}")

    vis = draw_detections(vis, detections)

    if detections:
        print(f"Detected {len(detections)} checkerboard(s):")
        for idx, detection in enumerate(detections, start=1):
            pattern_size = detection["pattern_size"]
            corners = detection["corners"]
            assert isinstance(pattern_size, tuple)
            assert isinstance(corners, np.ndarray)
            print(f"  {idx}. {pattern_size[0]}x{pattern_size[1]} inner corners ({len(corners)} points)")
    else:
        print("No checkerboards detected")

    if detections:
        object_points_xyz, image_points_xy, plane_ids = build_calibration_arrays(detections)
        points_output = args.points_output
        if points_output is None:
            points_output = args.image.with_name(f"{args.image.stem}_calibration_points.npz")
        save_calibration_arrays(points_output, object_points_xyz, image_points_xy, plane_ids)
        print(
            f"Saved calibration arrays to {points_output} "
            f"(object_points_xyz: {object_points_xyz.shape}, image_points_xy: {image_points_xy.shape})"
        )

    if args.output is not None:
        args.output.parent.mkdir(parents=True, exist_ok=True)
        if not cv2.imwrite(str(args.output), vis):
            raise SystemExit(f"Failed to write output image: {args.output}")
        print(f"Saved visualization to {args.output}")

    if args.show:
        cv2.imshow("Checkerboard Detection", vis)
        cv2.waitKey(0)
        cv2.destroyAllWindows()

    return 0 if detections else 1


if __name__ == "__main__":
    raise SystemExit(main())
