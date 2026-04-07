#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
import numpy as np


SCRIPT_DIR = Path(__file__).resolve().parent
if str(SCRIPT_DIR) not in sys.path:
    sys.path.insert(0, str(SCRIPT_DIR))

from test_custom_calibration import (
    build_calibration_arrays,
    detect_best_checkerboard_for_plane,
    draw_axis_frame_overlay,
    draw_detections,
    extract_polygon_roi,
    generate_pattern_sizes,
    offset_detections,
    plane_name_for_index,
    select_axis_frame,
    select_polygon_rois,
)


REPO_ROOT = Path("/home/khoa/Projects/zabidou_pc_sync")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Walk an orthogonal-checkerboard dataset, manually select polygon ROIs for "
            "each RGB/intensity frame, detect checkerboards, and save all labeled "
            "point correspondences in a NumPy-friendly format."
        )
    )
    parser.add_argument(
        "--dataset-dir",
        type=Path,
        default=REPO_ROOT / "calibration" / "ortho_data",
        help="Dataset root or a specific session directory containing *_rgb.png and *_intensity.png files.",
    )
    parser.add_argument(
        "--session",
        type=str,
        default=None,
        help="Optional session subdirectory name under dataset-dir to process.",
    )
    parser.add_argument("--min-cols", type=int, default=3)
    parser.add_argument("--max-cols", type=int, default=12)
    parser.add_argument("--min-rows", type=int, default=3)
    parser.add_argument("--max-rows", type=int, default=12)
    parser.add_argument(
        "--offset",
        type=int,
        default=0,
        help="Integer offset added to all annotated object-point coordinates.",
    )
    parser.add_argument(
        "--max",
        type=int,
        default=3,
        help="Maximum number of checkerboard polygon ROIs to select per image.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Output .npz path. Defaults to <session_dir>/annotated_checkerboard_points.npz",
    )
    parser.add_argument(
        "--annotated-dir",
        type=Path,
        default=None,
        help="Optional directory to save annotated RGB/intensity images.",
    )
    return parser.parse_args()


def resolve_session_dir(args: argparse.Namespace) -> Path:
    base = args.dataset_dir
    if args.session is not None:
        session_dir = base / args.session
        if not session_dir.is_dir():
            raise SystemExit(f"Session directory not found: {session_dir}")
        return session_dir

    if base.is_dir() and any(base.glob("*_rgb.png")):
        return base

    session_dirs = sorted(path for path in base.iterdir() if path.is_dir())
    if len(session_dirs) == 1:
        return session_dirs[0]
    raise SystemExit(
        "Please specify `--session` or point `--dataset-dir` directly at one session directory."
    )


def find_capture_pairs(session_dir: Path) -> list[tuple[int, Path, Path]]:
    rgb_files = sorted(session_dir.glob("*_rgb.png"))
    captures: list[tuple[int, Path, Path]] = []
    for index, rgb_path in enumerate(rgb_files):
        stem = rgb_path.name[:-8]
        intensity_path = session_dir / f"{stem}_intensity.png"
        if intensity_path.is_file():
            captures.append((index, rgb_path, intensity_path))
    return captures


def process_image(
    image_path: Path,
    camera_name: str,
    capture_index: int,
    args: argparse.Namespace,
    pattern_sizes: list[tuple[int, int]],
    annotated_dir: Path | None,
) -> tuple[list[dict[str, object]], Path | None]:
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"Failed to read image: {image_path}")

    print(f"\n[{camera_name} capture {capture_index}] {image_path.name}")
    axis_frame = select_axis_frame(image)
    rois = select_polygon_rois(image, args.max)
    if not rois:
        print("  No ROI selected; skipping image.")
        return [], None

    detections: list[dict[str, object]] = []
    vis = draw_axis_frame_overlay(image, axis_frame)

    for roi_index, roi in enumerate(rois, start=1):
        roi_image, roi_offset_xy, roi_mask = extract_polygon_roi(image, roi)
        gray = cv2.cvtColor(roi_image, cv2.COLOR_BGR2GRAY)
        gray[roi_mask == 0] = 127
        plane_name = plane_name_for_index(roi_index)
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
            f"ROI {roi_index} ({plane_name})",
            (x, max(20, y - 8)),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.7,
            (255, 0, 255),
            2,
            cv2.LINE_AA,
        )

    vis = draw_detections(vis, detections)

    annotated_path: Path | None = None
    if annotated_dir is not None:
        annotated_dir.mkdir(parents=True, exist_ok=True)
        annotated_path = annotated_dir / f"{image_path.stem}_annotated.png"
        if not cv2.imwrite(str(annotated_path), vis):
            raise RuntimeError(f"Failed to save annotated image: {annotated_path}")
        print(f"  Saved annotated image to {annotated_path}")

    if detections:
        print(f"  Detected {len(detections)} checkerboard(s)")
    else:
        print("  No checkerboards detected in selected ROIs")

    return detections, annotated_path


def main() -> int:
    args = parse_args()

    if args.min_cols > args.max_cols or args.min_rows > args.max_rows:
        raise SystemExit("Invalid checkerboard search range.")

    session_dir = resolve_session_dir(args)
    pattern_sizes = generate_pattern_sizes(args)
    capture_pairs = find_capture_pairs(session_dir)
    if not capture_pairs:
        raise SystemExit(f"No RGB/intensity capture pairs found in {session_dir}")

    output_path = args.output
    if output_path is None:
        output_path = session_dir / "annotated_checkerboard_points.npz"

    annotated_dir = args.annotated_dir
    if annotated_dir is None:
        annotated_dir = session_dir / "annotated"

    all_object_points: list[np.ndarray] = []
    all_image_points: list[np.ndarray] = []
    all_plane_ids: list[np.ndarray] = []
    all_camera_ids: list[np.ndarray] = []
    all_capture_indices: list[np.ndarray] = []
    all_image_filenames: list[np.ndarray] = []

    for capture_index, rgb_path, intensity_path in capture_pairs:
        for camera_name, image_path in (("rgb", rgb_path), ("intensity", intensity_path)):
            detections, _ = process_image(
                image_path=image_path,
                camera_name=camera_name,
                capture_index=capture_index,
                args=args,
                pattern_sizes=pattern_sizes,
                annotated_dir=annotated_dir,
            )
            if not detections:
                continue

            object_points_xyz, image_points_xy, plane_ids = build_calibration_arrays(detections)
            count = len(object_points_xyz)
            all_object_points.append(object_points_xyz)
            all_image_points.append(image_points_xy)
            all_plane_ids.append(plane_ids)
            all_camera_ids.append(np.full((count,), camera_name))
            all_capture_indices.append(np.full((count,), capture_index, dtype=np.int32))
            all_image_filenames.append(np.full((count,), image_path.name))

    if not all_object_points:
        raise SystemExit("No checkerboard correspondences were collected.")

    object_points_xyz = np.concatenate(all_object_points, axis=0)
    image_points_xy = np.concatenate(all_image_points, axis=0)
    plane_ids = np.concatenate(all_plane_ids, axis=0)
    camera_ids = np.concatenate(all_camera_ids, axis=0)
    capture_indices = np.concatenate(all_capture_indices, axis=0)
    image_filenames = np.concatenate(all_image_filenames, axis=0)

    output_path.parent.mkdir(parents=True, exist_ok=True)
    np.savez(
        output_path,
        object_points_xyz=object_points_xyz,
        image_points_xy=image_points_xy,
        plane_ids=plane_ids,
        camera_ids=camera_ids,
        capture_indices=capture_indices,
        image_filenames=image_filenames,
        session_dir=str(session_dir),
    )

    print(f"\nSaved dataset annotations to {output_path}")
    print(f"  object_points_xyz: {object_points_xyz.shape}")
    print(f"  image_points_xy: {image_points_xy.shape}")
    print(f"  camera_ids: {camera_ids.shape}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
