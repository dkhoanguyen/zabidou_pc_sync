#!/usr/bin/env python3

from __future__ import annotations

import argparse
from pathlib import Path

import cv2
import numpy as np


REPO_ROOT = Path("/home/khoa/Projects/zabidou_pc_sync")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Load saved multi-plane checkerboard correspondences and run "
            "cv2.calibrateCamera separately for RGB and intensity images."
        )
    )
    parser.add_argument(
        "points_file",
        type=Path,
        help="Path to the .npz file produced by annotate_ortho_dataset.py",
    )
    parser.add_argument(
        "--session-dir",
        type=Path,
        default=None,
        help="Optional override for the session directory that contains the source images.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Optional YAML output path. Defaults next to the points file.",
    )
    parser.add_argument(
        "--capture-index",
        type=int,
        default=None,
        help="Optional capture index to use only one RGB/intensity pair from the saved dataset.",
    )
    parser.add_argument(
        "--reprojection-dir",
        type=Path,
        default=None,
        help="Optional directory to save reprojection overlay images. Defaults next to the points file.",
    )
    parser.add_argument(
        "--rgb-init",
        type=Path,
        default=REPO_ROOT / "calibration" / "results" / "combined_all_datasets" / "mono_rgb_calibration.yaml",
        help="Initial RGB calibration YAML used as the intrinsic guess.",
    )
    parser.add_argument(
        "--intensity-init",
        type=Path,
        default=REPO_ROOT / "calibration" / "results" / "combined_all_datasets" / "mono_depth_calibration.yaml",
        help="Initial intensity/depth calibration YAML used as the intrinsic guess.",
    )
    return parser.parse_args()


def load_points(points_file: Path) -> dict[str, np.ndarray]:
    if not points_file.is_file():
        raise SystemExit(f"Points file not found: {points_file}")

    with np.load(points_file, allow_pickle=False) as data:
        required = {
            "object_points_xyz",
            "image_points_xy",
            "plane_ids",
            "camera_ids",
            "capture_indices",
            "image_filenames",
        }
        missing = sorted(required - set(data.files))
        if missing:
            raise SystemExit(f"Points file is missing required arrays: {missing}")

        loaded = {name: data[name] for name in data.files}
    return loaded


def resolve_session_dir(args: argparse.Namespace, loaded: dict[str, np.ndarray]) -> Path:
    if args.session_dir is not None:
        return args.session_dir

    if "session_dir" in loaded:
        session_dir = Path(str(loaded["session_dir"].item()))
        if not session_dir.is_absolute():
            session_dir = REPO_ROOT / session_dir
        return session_dir

    return args.points_file.parent


def infer_image_size(session_dir: Path, filenames: np.ndarray) -> tuple[int, int]:
    unique_names = np.unique(filenames)
    if len(unique_names) == 0:
        raise SystemExit(f"No image filenames available to infer image size from {session_dir}")

    image_path = session_dir / str(unique_names[0])
    image = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if image is None:
        raise SystemExit(f"Failed to read image for size inference: {image_path}")
    height, width = image.shape[:2]
    return width, height


def build_camera_observations(
    loaded: dict[str, np.ndarray],
    camera_name: str,
    capture_index: int | None = None,
) -> tuple[list[np.ndarray], list[np.ndarray], np.ndarray]:
    object_points_xyz = loaded["object_points_xyz"].astype(np.float32)
    image_points_xy = loaded["image_points_xy"].astype(np.float32)
    camera_ids = loaded["camera_ids"].astype(str)
    image_filenames = loaded["image_filenames"].astype(str)
    capture_indices = loaded["capture_indices"].astype(np.int32)

    camera_mask = camera_ids == camera_name
    if not np.any(camera_mask):
        raise SystemExit(f"No saved correspondences found for camera '{camera_name}'")

    if capture_index is not None:
        camera_mask &= capture_indices == capture_index
        if not np.any(camera_mask):
            available = np.unique(capture_indices[camera_ids == camera_name])
            raise SystemExit(
                f"No saved correspondences found for camera '{camera_name}' "
                f"at capture index {capture_index}. Available: {available.tolist()}"
            )

    camera_object_points = object_points_xyz[camera_mask]
    camera_image_points = image_points_xy[camera_mask]
    camera_filenames = image_filenames[camera_mask]

    object_points_per_image: list[np.ndarray] = []
    image_points_per_image: list[np.ndarray] = []
    used_filenames: list[str] = []

    for filename in np.unique(camera_filenames):
        image_mask = camera_filenames == filename
        obj = camera_object_points[image_mask].reshape(-1, 1, 3).astype(np.float32)
        img = camera_image_points[image_mask].reshape(-1, 1, 2).astype(np.float32)
        if len(obj) == 0 or len(img) == 0:
            continue
        object_points_per_image.append(obj)
        image_points_per_image.append(img)
        used_filenames.append(filename)

    if not object_points_per_image:
        raise SystemExit(f"No grouped observations found for camera '{camera_name}'")

    return object_points_per_image, image_points_per_image, np.asarray(used_filenames)


def load_initial_intrinsics(path: Path) -> tuple[np.ndarray, np.ndarray]:
    if not path.is_file():
        raise SystemExit(f"Initial calibration file not found: {path}")

    fs = cv2.FileStorage(str(path), cv2.FileStorage_READ)
    if not fs.isOpened():
        raise SystemExit(f"Failed to open initial calibration file: {path}")
    try:
        camera_matrix = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("dist_coeffs").mat()
    finally:
        fs.release()

    if camera_matrix is None or camera_matrix.size == 0:
        raise SystemExit(f"Initial calibration file is missing camera_matrix: {path}")
    if dist_coeffs is None or dist_coeffs.size == 0:
        raise SystemExit(f"Initial calibration file is missing dist_coeffs: {path}")

    return camera_matrix.astype(np.float64), dist_coeffs.astype(np.float64)


def calibrate_camera_from_points(
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    image_size: tuple[int, int],
    initial_camera_matrix: np.ndarray,
    initial_dist_coeffs: np.ndarray,
) -> tuple[float, np.ndarray, np.ndarray, list[np.ndarray], list[np.ndarray]]:
    camera_matrix = initial_camera_matrix.copy()
    dist_coeffs = initial_dist_coeffs.copy()
    rms, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        object_points,
        image_points,
        image_size,
        camera_matrix,
        dist_coeffs,
        flags=cv2.CALIB_USE_INTRINSIC_GUESS,
    )
    return rms, camera_matrix, dist_coeffs, rvecs, tvecs


def save_results(
    output_path: Path,
    rgb_result: tuple[float, np.ndarray, np.ndarray],
    intensity_result: tuple[float, np.ndarray, np.ndarray],
    rgb_size: tuple[int, int],
    intensity_size: tuple[int, int],
    rgb_images: np.ndarray,
    intensity_images: np.ndarray,
) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    fs = cv2.FileStorage(str(output_path), cv2.FileStorage_WRITE)
    if not fs.isOpened():
        raise SystemExit(f"Failed to open output YAML for writing: {output_path}")

    rgb_rms, rgb_camera_matrix, rgb_dist_coeffs = rgb_result
    intensity_rms, intensity_camera_matrix, intensity_dist_coeffs = intensity_result

    try:
        fs.write("backend", "opencv_calibrateCamera_from_saved_points")
        fs.write("rgb_rms_error", float(rgb_rms))
        fs.write("rgb_camera_matrix", rgb_camera_matrix)
        fs.write("rgb_dist_coeffs", rgb_dist_coeffs)
        fs.write("rgb_image_width", int(rgb_size[0]))
        fs.write("rgb_image_height", int(rgb_size[1]))
        fs.write("rgb_image_count", int(len(rgb_images)))

        fs.write("intensity_rms_error", float(intensity_rms))
        fs.write("intensity_camera_matrix", intensity_camera_matrix)
        fs.write("intensity_dist_coeffs", intensity_dist_coeffs)
        fs.write("intensity_image_width", int(intensity_size[0]))
        fs.write("intensity_image_height", int(intensity_size[1]))
        fs.write("intensity_image_count", int(len(intensity_images)))
    finally:
        fs.release()


def draw_reprojection_overlay(
    image: np.ndarray,
    measured_points: np.ndarray,
    reprojected_points: np.ndarray,
) -> np.ndarray:
    vis = image.copy()
    for measured, reprojected in zip(measured_points.reshape(-1, 2), reprojected_points.reshape(-1, 2), strict=True):
        mx, my = int(round(float(measured[0]))), int(round(float(measured[1])))
        rx, ry = int(round(float(reprojected[0]))), int(round(float(reprojected[1])))
        cv2.circle(vis, (mx, my), 4, (0, 255, 0), -1)
        cv2.circle(vis, (rx, ry), 3, (0, 0, 255), -1)
        cv2.line(vis, (mx, my), (rx, ry), (255, 255, 0), 1, cv2.LINE_AA)
    return vis


def save_reprojection_images(
    session_dir: Path,
    reprojection_dir: Path,
    camera_name: str,
    filenames: np.ndarray,
    object_points: list[np.ndarray],
    image_points: list[np.ndarray],
    camera_matrix: np.ndarray,
    dist_coeffs: np.ndarray,
    rvecs: list[np.ndarray],
    tvecs: list[np.ndarray],
) -> list[float]:
    reprojection_dir.mkdir(parents=True, exist_ok=True)
    per_image_errors: list[float] = []

    for filename, obj, img, rvec, tvec in zip(
        filenames,
        object_points,
        image_points,
        rvecs,
        tvecs,
        strict=True,
    ):
        image_path = session_dir / str(filename)
        image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
        if image is None:
            raise SystemExit(f"Failed to read image for reprojection overlay: {image_path}")

        reprojected, _ = cv2.projectPoints(obj, rvec, tvec, camera_matrix, dist_coeffs)
        error = np.linalg.norm(
            img.reshape(-1, 2) - reprojected.reshape(-1, 2),
            axis=1,
        ).mean()
        per_image_errors.append(float(error))

        overlay = draw_reprojection_overlay(image, img, reprojected)
        cv2.putText(
            overlay,
            f"{camera_name} reprojection error: {error:.3f}px",
            (10, 30),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.8,
            (0, 255, 255),
            2,
            cv2.LINE_AA,
        )
        output_path = reprojection_dir / f"{Path(str(filename)).stem}_{camera_name}_reprojection.png"
        if not cv2.imwrite(str(output_path), overlay):
            raise SystemExit(f"Failed to save reprojection image: {output_path}")

    return per_image_errors


def main() -> int:
    args = parse_args()
    loaded = load_points(args.points_file)
    session_dir = resolve_session_dir(args, loaded)

    rgb_object_points, rgb_image_points, rgb_images = build_camera_observations(
        loaded,
        "rgb",
        capture_index=args.capture_index,
    )
    intensity_object_points, intensity_image_points, intensity_images = build_camera_observations(
        loaded,
        "intensity",
        capture_index=args.capture_index,
    )

    rgb_size = infer_image_size(session_dir, rgb_images)
    intensity_size = infer_image_size(session_dir, intensity_images)
    rgb_init_camera_matrix, rgb_init_dist_coeffs = load_initial_intrinsics(args.rgb_init)
    intensity_init_camera_matrix, intensity_init_dist_coeffs = load_initial_intrinsics(args.intensity_init)

    rgb_rms, rgb_camera_matrix, rgb_dist_coeffs, rgb_rvecs, rgb_tvecs = calibrate_camera_from_points(
        rgb_object_points,
        rgb_image_points,
        rgb_size,
        rgb_init_camera_matrix,
        rgb_init_dist_coeffs,
    )
    intensity_rms, intensity_camera_matrix, intensity_dist_coeffs, intensity_rvecs, intensity_tvecs = (
        calibrate_camera_from_points(
            intensity_object_points,
            intensity_image_points,
            intensity_size,
            intensity_init_camera_matrix,
            intensity_init_dist_coeffs,
        )
    )

    output_path = args.output
    if output_path is None:
        output_path = args.points_file.with_name(f"{args.points_file.stem}_camera_calibration.yaml")

    reprojection_dir = args.reprojection_dir
    if reprojection_dir is None:
        reprojection_dir = args.points_file.with_name(f"{args.points_file.stem}_reprojection")

    save_results(
        output_path,
        (rgb_rms, rgb_camera_matrix, rgb_dist_coeffs),
        (intensity_rms, intensity_camera_matrix, intensity_dist_coeffs),
        rgb_size,
        intensity_size,
        rgb_images,
        intensity_images,
    )

    rgb_errors = save_reprojection_images(
        session_dir,
        reprojection_dir,
        "rgb",
        rgb_images,
        rgb_object_points,
        rgb_image_points,
        rgb_camera_matrix,
        rgb_dist_coeffs,
        rgb_rvecs,
        rgb_tvecs,
    )
    intensity_errors = save_reprojection_images(
        session_dir,
        reprojection_dir,
        "intensity",
        intensity_images,
        intensity_object_points,
        intensity_image_points,
        intensity_camera_matrix,
        intensity_dist_coeffs,
        intensity_rvecs,
        intensity_tvecs,
    )

    print(f"Session dir: {session_dir}")
    if args.capture_index is not None:
        print(f"Capture index: {args.capture_index}")
    print(f"RGB init: {args.rgb_init}")
    print(f"Intensity init: {args.intensity_init}")
    print(f"RGB images used: {len(rgb_images)} | image size: {rgb_size} | RMS: {rgb_rms:.6f}")
    print(f"Intensity images used: {len(intensity_images)} | image size: {intensity_size} | RMS: {intensity_rms:.6f}")
    print(f"Saved calibration YAML to {output_path}")
    print(f"Saved reprojection overlays to {reprojection_dir}")
    print(f"Mean RGB reprojection error per image: {np.mean(rgb_errors):.3f}px")
    print(f"Mean intensity reprojection error per image: {np.mean(intensity_errors):.3f}px")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
