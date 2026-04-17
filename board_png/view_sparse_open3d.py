#!/usr/bin/env python3
"""Display a COLMAP sparse reconstruction with Open3D.

Usage:
    python3 view_sparse_open3d.py colmap_sparse/sparse/0

The script reads COLMAP sparse models in either binary form
(`cameras.bin`, `images.bin`, `points3D.bin`) or text form
(`cameras.txt`, `images.txt`, `points3D.txt`).
"""

from __future__ import annotations

import argparse
import math
import struct
from dataclasses import dataclass
from pathlib import Path

import numpy as np
import open3d as o3d


# COLMAP camera model IDs from src/colmap/sensor/models.h.
CAMERA_MODEL_PARAMS = {
    0: ("SIMPLE_PINHOLE", 3),
    1: ("PINHOLE", 4),
    2: ("SIMPLE_RADIAL", 4),
    3: ("RADIAL", 5),
    4: ("OPENCV", 8),
    5: ("OPENCV_FISHEYE", 8),
    6: ("FULL_OPENCV", 12),
    7: ("FOV", 5),
    8: ("SIMPLE_RADIAL_FISHEYE", 4),
    9: ("RADIAL_FISHEYE", 5),
    10: ("THIN_PRISM_FISHEYE", 12),
}

CAMERA_MODEL_NAME_TO_ID = {name: model_id for model_id, (name, _) in CAMERA_MODEL_PARAMS.items()}


@dataclass(frozen=True)
class Camera:
    camera_id: int
    model: str
    width: int
    height: int
    params: np.ndarray


@dataclass(frozen=True)
class Image:
    image_id: int
    qvec: np.ndarray
    tvec: np.ndarray
    camera_id: int
    name: str


@dataclass(frozen=True)
class Points3D:
    xyz: np.ndarray
    rgb: np.ndarray
    error: np.ndarray


def read_next_bytes(handle, num_bytes: int, fmt: str):
    data = handle.read(num_bytes)
    if len(data) != num_bytes:
        raise EOFError("Unexpected end of COLMAP binary file")
    return struct.unpack("<" + fmt, data)


def read_cameras_binary(path: Path) -> dict[int, Camera]:
    cameras: dict[int, Camera] = {}
    with path.open("rb") as handle:
        num_cameras = read_next_bytes(handle, 8, "Q")[0]
        for _ in range(num_cameras):
            camera_id, model_id, width, height = read_next_bytes(handle, 24, "iiQQ")
            if model_id not in CAMERA_MODEL_PARAMS:
                raise ValueError(f"Unsupported COLMAP camera model id: {model_id}")
            model_name, num_params = CAMERA_MODEL_PARAMS[model_id]
            params = np.array(read_next_bytes(handle, 8 * num_params, "d" * num_params))
            cameras[camera_id] = Camera(camera_id, model_name, width, height, params)
    return cameras


def read_images_binary(path: Path) -> dict[int, Image]:
    images: dict[int, Image] = {}
    with path.open("rb") as handle:
        num_images = read_next_bytes(handle, 8, "Q")[0]
        for _ in range(num_images):
            image_id = read_next_bytes(handle, 4, "i")[0]
            qvec = np.array(read_next_bytes(handle, 32, "dddd"))
            tvec = np.array(read_next_bytes(handle, 24, "ddd"))
            camera_id = read_next_bytes(handle, 4, "i")[0]

            name_bytes = bytearray()
            while True:
                char = handle.read(1)
                if char == b"\x00":
                    break
                if char == b"":
                    raise EOFError("Unexpected end of image name")
                name_bytes.extend(char)
            name = name_bytes.decode("utf-8", errors="replace")

            num_points2d = read_next_bytes(handle, 8, "Q")[0]
            handle.seek(num_points2d * 24, 1)
            images[image_id] = Image(image_id, qvec, tvec, camera_id, name)
    return images


def read_points3d_binary(path: Path) -> Points3D:
    xyz = []
    rgb = []
    error = []
    with path.open("rb") as handle:
        num_points = read_next_bytes(handle, 8, "Q")[0]
        for _ in range(num_points):
            read_next_bytes(handle, 8, "Q")
            point_xyz = read_next_bytes(handle, 24, "ddd")
            point_rgb = read_next_bytes(handle, 3, "BBB")
            point_error = read_next_bytes(handle, 8, "d")[0]
            track_length = read_next_bytes(handle, 8, "Q")[0]
            handle.seek(track_length * 8, 1)

            xyz.append(point_xyz)
            rgb.append(point_rgb)
            error.append(point_error)

    return Points3D(
        xyz=np.asarray(xyz, dtype=np.float64),
        rgb=np.asarray(rgb, dtype=np.float64) / 255.0,
        error=np.asarray(error, dtype=np.float64),
    )


def read_cameras_text(path: Path) -> dict[int, Camera]:
    cameras: dict[int, Camera] = {}
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            camera_id = int(parts[0])
            model = parts[1]
            width = int(parts[2])
            height = int(parts[3])
            params = np.array([float(value) for value in parts[4:]], dtype=np.float64)
            cameras[camera_id] = Camera(camera_id, model, width, height, params)
    return cameras


def read_images_text(path: Path) -> dict[int, Image]:
    images: dict[int, Image] = {}
    with path.open("r", encoding="utf-8") as handle:
        data_lines = [line.strip() for line in handle if line.strip() and not line.startswith("#")]

    for idx in range(0, len(data_lines), 2):
        parts = data_lines[idx].split()
        image_id = int(parts[0])
        qvec = np.array([float(value) for value in parts[1:5]], dtype=np.float64)
        tvec = np.array([float(value) for value in parts[5:8]], dtype=np.float64)
        camera_id = int(parts[8])
        name = " ".join(parts[9:])
        images[image_id] = Image(image_id, qvec, tvec, camera_id, name)
    return images


def read_points3d_text(path: Path) -> Points3D:
    xyz = []
    rgb = []
    error = []
    with path.open("r", encoding="utf-8") as handle:
        for line in handle:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            xyz.append([float(value) for value in parts[1:4]])
            rgb.append([float(value) / 255.0 for value in parts[4:7]])
            error.append(float(parts[7]))
    return Points3D(
        xyz=np.asarray(xyz, dtype=np.float64),
        rgb=np.asarray(rgb, dtype=np.float64),
        error=np.asarray(error, dtype=np.float64),
    )


def read_model(model_dir: Path) -> tuple[dict[int, Camera], dict[int, Image], Points3D]:
    binary_files = [model_dir / "cameras.bin", model_dir / "images.bin", model_dir / "points3D.bin"]
    text_files = [model_dir / "cameras.txt", model_dir / "images.txt", model_dir / "points3D.txt"]

    if all(path.exists() for path in binary_files):
        return (
            read_cameras_binary(binary_files[0]),
            read_images_binary(binary_files[1]),
            read_points3d_binary(binary_files[2]),
        )

    if all(path.exists() for path in text_files):
        return (
            read_cameras_text(text_files[0]),
            read_images_text(text_files[1]),
            read_points3d_text(text_files[2]),
        )

    raise FileNotFoundError(
        f"{model_dir} must contain cameras/images/points3D as either .bin or .txt files"
    )


def qvec_to_rotmat(qvec: np.ndarray) -> np.ndarray:
    qw, qx, qy, qz = qvec
    return np.array(
        [
            [
                1 - 2 * qy * qy - 2 * qz * qz,
                2 * qx * qy - 2 * qw * qz,
                2 * qz * qx + 2 * qw * qy,
            ],
            [
                2 * qx * qy + 2 * qw * qz,
                1 - 2 * qx * qx - 2 * qz * qz,
                2 * qy * qz - 2 * qw * qx,
            ],
            [
                2 * qz * qx - 2 * qw * qy,
                2 * qy * qz + 2 * qw * qx,
                1 - 2 * qx * qx - 2 * qy * qy,
            ],
        ],
        dtype=np.float64,
    )


def camera_center(image: Image) -> np.ndarray:
    rotation = qvec_to_rotmat(image.qvec)
    return -rotation.T @ image.tvec


def make_point_cloud(points3d: Points3D, max_error: float | None, max_points: int | None) -> o3d.geometry.PointCloud:
    points = points3d.xyz
    colors = points3d.rgb

    if max_error is not None:
        keep = points3d.error <= max_error
        points = points[keep]
        colors = colors[keep]

    if max_points is not None and len(points) > max_points:
        rng = np.random.default_rng(0)
        indices = rng.choice(len(points), size=max_points, replace=False)
        points = points[indices]
        colors = colors[indices]

    cloud = o3d.geometry.PointCloud()
    cloud.points = o3d.utility.Vector3dVector(points)
    cloud.colors = o3d.utility.Vector3dVector(colors)
    return cloud


def make_camera_frustums(
    cameras: dict[int, Camera],
    images: dict[int, Image],
    scale: float,
) -> o3d.geometry.LineSet:
    points = []
    lines = []
    colors = []

    for image in sorted(images.values(), key=lambda item: item.name):
        camera = cameras[image.camera_id]
        width = camera.width
        height = camera.height

        # Approximate focal length is enough for visualization.
        if camera.model in {"SIMPLE_PINHOLE", "SIMPLE_RADIAL", "RADIAL", "SIMPLE_RADIAL_FISHEYE", "RADIAL_FISHEYE"}:
            focal = camera.params[0]
        elif camera.model in {"PINHOLE", "OPENCV", "OPENCV_FISHEYE", "FULL_OPENCV", "FOV", "THIN_PRISM_FISHEYE"}:
            focal = 0.5 * (camera.params[0] + camera.params[1])
        else:
            focal = max(width, height)

        half_w = scale * width / focal
        half_h = scale * height / focal
        depth = scale

        corners_camera = np.array(
            [
                [0.0, 0.0, 0.0],
                [-half_w, -half_h, depth],
                [half_w, -half_h, depth],
                [half_w, half_h, depth],
                [-half_w, half_h, depth],
            ],
            dtype=np.float64,
        )

        rotation = qvec_to_rotmat(image.qvec)
        corners_world = (rotation.T @ (corners_camera - image.tvec).T).T

        start = len(points)
        points.extend(corners_world.tolist())
        lines.extend(
            [
                [start, start + 1],
                [start, start + 2],
                [start, start + 3],
                [start, start + 4],
                [start + 1, start + 2],
                [start + 2, start + 3],
                [start + 3, start + 4],
                [start + 4, start + 1],
            ]
        )
        colors.extend([[0.05, 0.45, 1.0]] * 8)

    frustums = o3d.geometry.LineSet()
    frustums.points = o3d.utility.Vector3dVector(np.asarray(points, dtype=np.float64))
    frustums.lines = o3d.utility.Vector2iVector(np.asarray(lines, dtype=np.int32))
    frustums.colors = o3d.utility.Vector3dVector(np.asarray(colors, dtype=np.float64))
    return frustums


def estimate_frustum_scale(points: np.ndarray) -> float:
    if len(points) == 0:
        return 1.0
    extent = np.ptp(points, axis=0)
    diagonal = np.linalg.norm(extent)
    if not math.isfinite(diagonal) or diagonal <= 0:
        return 1.0
    return diagonal * 0.03


def print_stats(cameras: dict[int, Camera], images: dict[int, Image], points3d: Points3D) -> None:
    print(f"Cameras: {len(cameras)}")
    print(f"Registered images: {len(images)}")
    print(f"Sparse points: {len(points3d.xyz)}")
    if len(points3d.xyz) > 0:
        print(f"Mean reprojection error: {points3d.error.mean():.4f}")
        print(f"Median reprojection error: {np.median(points3d.error):.4f}")
        print(f"Point bounds min: {points3d.xyz.min(axis=0)}")
        print(f"Point bounds max: {points3d.xyz.max(axis=0)}")


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "model_dir",
        nargs="?",
        default="colmap_sparse/sparse/0",
        type=Path,
        help="Path to COLMAP sparse model directory.",
    )
    parser.add_argument(
        "--max-error",
        type=float,
        default=None,
        help="Hide points with reprojection error above this threshold.",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=None,
        help="Randomly downsample to at most this many sparse points.",
    )
    parser.add_argument(
        "--frustum-scale",
        type=float,
        default=None,
        help="Camera frustum depth in reconstruction units. Defaults to 3%% of point-cloud diagonal.",
    )
    parser.add_argument(
        "--no-cameras",
        action="store_true",
        help="Display only sparse points.",
    )
    parser.add_argument(
        "--stats-only",
        action="store_true",
        help="Load the model and print summary statistics without opening a window.",
    )
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    cameras, images, points3d = read_model(args.model_dir)
    print_stats(cameras, images, points3d)

    if args.stats_only:
        return

    geometries: list[o3d.geometry.Geometry] = [
        make_point_cloud(points3d, args.max_error, args.max_points),
        o3d.geometry.TriangleMesh.create_coordinate_frame(
            size=estimate_frustum_scale(points3d.xyz) * 2.0,
            origin=[0.0, 0.0, 0.0],
        ),
    ]

    if not args.no_cameras:
        scale = args.frustum_scale or estimate_frustum_scale(points3d.xyz)
        geometries.append(make_camera_frustums(cameras, images, scale))

    o3d.visualization.draw_geometries(
        geometries,
        window_name=f"COLMAP sparse model: {args.model_dir}",
        point_show_normal=False,
    )


if __name__ == "__main__":
    main()
