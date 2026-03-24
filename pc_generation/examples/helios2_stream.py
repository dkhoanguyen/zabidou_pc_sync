"""
Helios2 depth camera — live stream example.

Displays the intensity channel in an OpenCV window and prints the
closest and farthest valid points each frame.  Press 'q' to quit.

Usage
-----
    python examples/helios2_stream.py
    python examples/helios2_stream.py --format Coord3D_ABCY16s
"""

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

# Allow running from the project root without installing the package
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src import Helios2Camera


def parse_args():
    p = argparse.ArgumentParser(description='Helios2 live depth stream')
    p.add_argument(
        '--format',
        choices=['Coord3D_ABCY16', 'Coord3D_ABCY16s'],
        default='Coord3D_ABCY16',
        help='Pixel format (default: Coord3D_ABCY16)',
    )
    p.add_argument('--device-index', type=int, default=0,
                   help='Which Helios2 device to open (default: 0)')
    return p.parse_args()


def depth_colormap(depth_mm: np.ndarray) -> np.ndarray:
    """Convert a float32 depth map (mm) to a colour-mapped uint8 image."""
    valid = depth_mm > 0
    if not valid.any():
        return np.zeros((*depth_mm.shape, 3), dtype=np.uint8)
    vmin, vmax = depth_mm[valid].min(), depth_mm[valid].max()
    norm = np.zeros_like(depth_mm, dtype=np.float32)
    norm[valid] = (depth_mm[valid] - vmin) / max(vmax - vmin, 1.0)
    grey = (norm * 255).astype(np.uint8)
    return cv2.applyColorMap(grey, cv2.COLORMAP_JET)


def print_stats(frame):
    valid = frame.depth[frame.depth > 0]
    if valid.size == 0:
        print('  No valid depth points.')
        return
    idx_min = np.argmin(frame.depth[frame.depth > 0])
    idx_max = np.argmax(frame.depth[frame.depth > 0])
    z_min = valid.min()
    z_max = valid.max()
    print(f'  Closest : {z_min:.1f} mm  |  Farthest : {z_max:.1f} mm  '
          f'|  Valid pixels: {valid.size}/{frame.depth.size}')


def main():
    args = parse_args()

    print(f'Opening Helios2 camera (index {args.device_index}, format {args.format}) …')
    cam = Helios2Camera.from_model(index=args.device_index, pixel_format=args.format)
    print(f'  {cam}')

    cam.configure()
    cam.start_stream()

    try:
        prev_time = time.time()
        while True:
            frame = cam.get_frame()

            # ---- intensity overlay ----
            intensity_8u = cv2.normalize(
                frame.intensity, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U
            )
            intensity_bgr = cv2.cvtColor(intensity_8u, cv2.COLOR_GRAY2BGR)

            # ---- depth colormap ----
            depth_vis = depth_colormap(frame.depth)

            # ---- side-by-side display ----
            combined = np.hstack([intensity_bgr, depth_vis])
            cv2.imshow('Helios2  |  Intensity (left)  Depth (right)', combined)

            # ---- fps + stats ----
            now = time.time()
            fps = 1.0 / max(now - prev_time, 1e-6)
            prev_time = now
            print(f'FPS {fps:.1f}', end='  ')
            print_stats(frame)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cam.release()
        cv2.destroyAllWindows()
        print('Stream stopped.')


if __name__ == '__main__':
    main()
