#!/usr/bin/env python3
"""svg_to_ilda.py

Convert an SVG file into an ILDA frame and send it to a Helios DAC device.
The heavy‑lifting of SVG parsing is delegated to ``convert_svg_to_points.py``
which provides ``process_svg(svg_path)`` returning a list of ``(x, y, pen)``
where ``x`` and ``y`` are already scaled to the 0‑4095 range expected by the
DAC. ``pen`` is 1 when the laser should be on and 0 when it should be off.

Usage
-----
    python3 svg_to_ilda.py <svg_file>

The script loads the points, builds a ``HeliosPoint`` array and writes a
single frame to the first detected device using ``HeliosLib.WriteFrame``.
"""

import sys
import ctypes
from pathlib import Path

TARGET_MAX = 4095
HeliosLib = ctypes.cdll.LoadLibrary("./libHeliosDacAPI.so")

# Define the point structure expected by the library
class HeliosPoint(ctypes.Structure):
    _fields_ = [
        ("x", ctypes.c_uint16),
        ("y", ctypes.c_uint16),
        ("r", ctypes.c_uint8),
        ("g", ctypes.c_uint8),
        ("b", ctypes.c_uint8),
        ("i", ctypes.c_uint8),
    ]

# Import the SVG→points conversion function
sys.path.append(str(Path(__file__).parent))
try:
    from convert_svg_to_points import process_svg
except Exception as e:
    sys.stderr.write(f"Failed to import conversion module: {e}\n")
    sys.exit(1)

def mirror_vertical(points):
    """Mirror points vertically (top/bottom) and horizontally (left/right) by reflecting both coordinates across the centre of the ILDA coordinate space (0‑4095)."""
    return [(TARGET_MAX - x, TARGET_MAX - y, pen) for (x, y, pen) in points]

def simplify_points(points, max_points=1000, tolerance_degrees=5):
    """Reduce points by merging near‑collinear segments.
    The function keeps points when the pen state changes or when the angle
    between consecutive line segments differs by more than ``tolerance_degrees``.
    Angles are measured in degrees. If the resulting list is still larger than
    ``max_points`` we fall back to uniform down‑sampling.
    """
    import math
    if len(points) <= 2:
        return points
    kept = [points[0]]
    for i in range(1, len(points) - 1):
        x0, y0, p0 = kept[-1]
        x1, y1, p1 = points[i]
        x2, y2, p2 = points[i + 1]
        # Preserve pen toggles
        if p1 != p0:
            kept.append(points[i])
            continue
        # Vectors for two consecutive segments
        dx1, dy1 = x1 - x0, y1 - y0
        dx2, dy2 = x2 - x1, y2 - y1
        # If any segment has zero length, keep the point
        if dx1 == 0 and dy1 == 0 or dx2 == 0 and dy2 == 0:
            kept.append(points[i])
            continue
        # Compute angles in degrees
        ang1 = math.degrees(math.atan2(dy1, dx1))
        ang2 = math.degrees(math.atan2(dy2, dx2))
        diff = abs(ang2 - ang1)
        # Normalize to [0,180]
        if diff > 180:
            diff = 360 - diff
        if diff > tolerance_degrees:
            kept.append(points[i])
    kept.append(points[-1])
#    if len(kept) > max_points:
#        return downsample_points(kept, max_points)
    return kept

def downsample_points(points, max_points=1000):
    """Reduce the list of points to at most ``max_points``.
    Simple uniform decimation – picks every ``step``‑th point and always
    includes the first and last points to preserve start/end.
    """
    if len(points) <= max_points:
        return points
    step = max(1, len(points) // max_points)
    sampled = [points[i] for i in range(0, len(points), step)]
    if sampled[-1] != points[-1]:
        sampled.append(points[-1])
    return sampled[:max_points]

def build_helios_frame(points, r, g, b):
    """Create a ctypes array of ``HeliosPoint`` from a list of (x, y, pen).
    * ``x`` and ``y`` are already in the 0‑4095 range.
    * ``pen`` (0/1) determines the intensity channel – we keep a constant
      green colour (r=0, g=93, b=0) and set intensity to ``pen``.
    """
    FrameType = HeliosPoint * len(points)
    frame = FrameType()
    for idx, (x, y, pen) in enumerate(points):
        frame[idx] = HeliosPoint(int(x), int(y), r, g, b, int(pen))
        # if idx > 10:
        #     break
    # frame[0] = HeliosPoint(1000,1000,0,255,0,1)
    # frame[1] = HeliosPoint(2000,1000,0,255,0,1)
    # frame[2] = HeliosPoint(2000,2000,0,255,0,1)
    # frame[3] = HeliosPoint(1000,2000,0,255,0,1)
    # frame[4] = HeliosPoint(1000,1000,0,255,0,1)
    return frame

def parse_args():
    import argparse
    parser = argparse.ArgumentParser(description="Convert SVG to ILDA frame and stream to Helios DAC.")
    parser.add_argument('svg_file', help='Path to the input SVG file')
    parser.add_argument('--scale', '-s', type=float, default=100.0,
                        help='Scaling factor as a percentage (100 = original size)')
    parser.add_argument('--tolerance', '-t', type=float, default=5.0,
                        help='Angle tolerance in degrees for simplification (default 5)')
    parser.add_argument('--offset-x', type=float, default=0.0,
                        help='Horizontal offset applied to points after scaling (default 0)')
    parser.add_argument('--offset-y', type=float, default=0.0,
                        help='Vertical offset applied to points after scaling (default 0)')
    parser.add_argument('--red', type=int, default=0,
                        help='Red component (default 0)')
    parser.add_argument('--green', type=int, default=93,
                        help='Green component (default 93)')
    parser.add_argument('--blue', type=int, default=0,
                        help='Blue component (default 0)')

    return parser.parse_args()

def main():
    args = parse_args()
    svg_path = args.svg_file
    if not Path(svg_path).is_file():
        sys.stderr.write(f"File not found: {svg_path}\n")
        sys.exit(1)

    # Convert SVG to points
    points = process_svg(svg_path)
    if not points:
        sys.stderr.write("No points generated from SVG.\n")
        sys.exit(1)

    # Apply scaling factor (percentage)
    scale_factor = args.scale / 100.0
    if scale_factor != 1.0:
        scaled = []
        for x, y, pen in points:
            sx = int(round(x * scale_factor))
            sy = int(round(y * scale_factor))
            sx = max(0, min(TARGET_MAX, sx))
            sy = max(0, min(TARGET_MAX, sy))
            scaled.append((sx, sy, pen))
        points = scaled
        # Apply optional offsets
        if args.offset_x != 0.0 or args.offset_y != 0.0:
            offsetted = []
            for x, y, pen in points:
                ox = int(round(x + args.offset_x))
                oy = int(round(y + args.offset_y))
                ox = max(0, min(TARGET_MAX, ox))
                oy = max(0, min(TARGET_MAX, oy))
                offsetted.append((ox, oy, pen))
            points = offsetted

    # Mirror vertically and horizontally and simplify points
    mirrored = mirror_vertical(points)
    optimized_points = simplify_points(mirrored, max_points=1000, tolerance_degrees=args.tolerance)
    frame = build_helios_frame(optimized_points, args.red, args.green, args.blue)

    # Open devices
    num_devices = HeliosLib.OpenDevices()
    if num_devices == 0:
        sys.stderr.write("No Helios DAC devices found.\n")
        sys.exit(1)
    device_index = 0
    # Continuously write the single optimized frame (15000 cycles)
    for i in range(15000):
        statusAttempts = 0
        while (statusAttempts < 512 and HeliosLib.GetStatus(0) != 1):
            statusAttempts += 1
        HeliosLib.WriteFrame(
            device_index,
            ctypes.c_uint32(30000),  # duration in µs
            ctypes.c_uint32(0),
            ctypes.pointer(frame),
            ctypes.c_uint32(len(optimized_points)),
        )
    HeliosLib.CloseDevices()
    print(f"Wrote {len(optimized_points)} points (optimized) to device {device_index}.")

if __name__ == "__main__":
    main()

