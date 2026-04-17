#!/usr/bin/env python3
"""View a PLY point cloud or mesh with Open3D.

Usage:
    python3 view_ply_open3d.py
    python3 view_ply_open3d.py colmap_dense/fused.ply --point-size 2
    python3 view_ply_open3d.py colmap_dense/fused.ply --voxel-size 0.01
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import open3d as o3d


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "ply_path",
        nargs="?",
        default="colmap_dense/fused.ply",
        help="Path to the PLY file to view.",
    )
    parser.add_argument(
        "--voxel-size",
        type=float,
        default=0.0,
        help="Downsample point clouds with this voxel size. Use 0 to disable.",
    )
    parser.add_argument(
        "--point-size",
        type=float,
        default=2.0,
        help="Rendered point size.",
    )
    parser.add_argument(
        "--no-axes",
        action="store_true",
        help="Hide the coordinate frame.",
    )
    parser.add_argument(
        "--estimate-normals",
        action="store_true",
        help="Estimate normals before rendering a point cloud.",
    )
    return parser.parse_args()


def load_point_cloud(path: Path, voxel_size: float, estimate_normals: bool) -> o3d.geometry.PointCloud:
    point_cloud = o3d.io.read_point_cloud(str(path))
    if point_cloud.is_empty():
        raise ValueError(f"No points were loaded from {path}")

    if voxel_size > 0:
        before = len(point_cloud.points)
        point_cloud = point_cloud.voxel_down_sample(voxel_size)
        after = len(point_cloud.points)
        print(f"Downsampled: {before:,} -> {after:,} points")

    if not point_cloud.has_colors():
        point_cloud.paint_uniform_color([0.85, 0.85, 0.85])

    if estimate_normals and not point_cloud.has_normals():
        point_cloud.estimate_normals()
        point_cloud.orient_normals_consistent_tangent_plane(30)

    return point_cloud


def add_coordinate_frame(
    geometries: list[o3d.geometry.Geometry],
    point_cloud: o3d.geometry.PointCloud,
) -> None:
    bounds = point_cloud.get_axis_aligned_bounding_box()
    extent = bounds.get_extent()
    scale = max(float(np.max(extent)) * 0.12, 0.1)
    origin = bounds.get_center() - extent * 0.45
    axes = o3d.geometry.TriangleMesh.create_coordinate_frame(size=scale, origin=origin)
    geometries.append(axes)


def show_geometries(
    geometries: list[o3d.geometry.Geometry],
    title: str,
    point_size: float,
) -> None:
    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window(window_name=title, width=1280, height=800)
    for geometry in geometries:
        visualizer.add_geometry(geometry)

    render_options = visualizer.get_render_option()
    render_options.point_size = point_size
    render_options.background_color = np.asarray([0.03, 0.03, 0.035])

    visualizer.run()
    visualizer.destroy_window()


def main() -> None:
    args = parse_args()
    ply_path = Path(args.ply_path).expanduser().resolve()
    if not ply_path.exists():
        raise FileNotFoundError(f"PLY file does not exist: {ply_path}")

    point_cloud = load_point_cloud(ply_path, args.voxel_size, args.estimate_normals)
    bounds = point_cloud.get_axis_aligned_bounding_box()

    print(f"Loaded: {ply_path}")
    print(f"Points: {len(point_cloud.points):,}")
    print(f"Center: {bounds.get_center()}")
    print(f"Extent: {bounds.get_extent()}")

    geometries: list[o3d.geometry.Geometry] = [point_cloud]
    if not args.no_axes:
        add_coordinate_frame(geometries, point_cloud)

    show_geometries(geometries, ply_path.name, args.point_size)


if __name__ == "__main__":
    main()
