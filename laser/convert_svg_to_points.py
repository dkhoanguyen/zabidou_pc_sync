#!/usr/bin/env python3
"""convert_svg_to_points.py

Utility script to read an SVG file (expected to contain <path> elements) and
output a sequence of points suitable for ILDA/laser‑plotter style drawing.
Each output line is:
    x,y,pen
where `x` and `y` are integer coordinates in the range 0‑4095 and `pen`
is 1 when the laser should be ON (drawing) and 0 when it should be OFF
(moving between sub‑paths).

The script scales the SVG coordinate system (derived from the viewBox) to the
0‑4095 range while keeping the aspect ratio.

Dependencies
------------
* ``svgpathtools`` – provides robust parsing of SVG path data.
  Install with ``pip install svgpathtools``.

Usage
-----
    python convert_svg_to_points.py input.svg > output.csv
"""

import sys
import argparse
from xml.etree import ElementTree as ET

try:
    # svgpathtools provides a convenient Path parser
    from svgpathtools import parse_path, Path
except ImportError:  # pragma: no cover
    sys.stderr.write(
        "Error: svgpathtools not installed. Install it with:\n"
        "    pip install svgpathtools\n"
    )
    sys.exit(1)

# Target coordinate range for the laser plotter
TARGET_MAX = 4095


def extract_viewbox(root: ET.Element) -> tuple[float, float, float, float]:
    """Return (min_x, min_y, width, height) from the SVG viewBox.
    If the attribute is missing, fall back to the width/height attributes.
    """
    viewbox = root.get("viewBox")
    if viewbox:
        parts = list(map(float, viewbox.strip().split()))
        if len(parts) == 4:
            return tuple(parts)
    # Fallback – use width/height attributes (assume 0,0 origin)
    width = float(root.get("width", "0"))
    height = float(root.get("height", "0"))
    return 0.0, 0.0, width, height


def scale_point(x: float, y: float, vb: tuple[float, float, float, float]) -> tuple[int, int]:
    """Scale a point from SVG coordinates to the 0‑4095 integer range.
    The scaling preserves the aspect ratio based on the viewBox dimensions.
    """
    _, _, w, h = vb
    # Avoid division by zero – if dimensions are zero, use a scale of 1
    scale_x = TARGET_MAX / w if w != 0 else 1.0
    scale_y = TARGET_MAX / h if h != 0 else 1.0
    # Use the same scale for both axes to keep the image square – pick the
    # smaller factor so nothing exceeds the target range.
    scale = min(scale_x, scale_y)
    sx = int(round(x * scale))
    sy = int(round(y * scale))
    # Clamp to valid range just in case rounding pushes a coordinate out of bounds
    sx = max(0, min(TARGET_MAX, sx))
    sy = max(0, min(TARGET_MAX, sy))
    return sx, sy


def path_to_points(d: str, vb: tuple[float, float, float, float]) -> list[tuple[int, int, int]]:
    """Convert a single SVG path ``d`` attribute into a list of (x, y, pen).
    The algorithm walks the parsed path and samples it at a fixed resolution.
    ``pen`` is 1 for drawing segments and 0 for the initial move.
    """
    path_obj: Path = parse_path(d)
    points: list[tuple[int, int, int]] = []
    # Number of sample points per unit length – higher values give smoother
    # curves at the cost of more output points.
    samples_per_unit = 1.0  # 1 point per SVG unit; adjust if needed

    for seg in path_obj:
        length = seg.length(error=1e-5)
        n_samples = max(1, int(round(length * samples_per_unit)))
        for i in range(n_samples + 1):
            t = i / n_samples
            pt = seg.point(t)
            sx, sy = scale_point(pt.real, pt.imag, vb)
            points.append((sx, sy, 1))
    return points


def process_svg(svg_path: str) -> list[tuple[int, int, int]]:
    """Parse the SVG file and return the full point list.
    The function respects multiple <path> elements. Between distinct paths the
    pen is lifted (pen = 0) and the next path starts with pen = 1.
    """
    tree = ET.parse(svg_path)
    root = tree.getroot()
    vb = extract_viewbox(root)
    all_points: list[tuple[int, int, int]] = []
    # SVG namespace handling – ElementTree includes the namespace in the tag
    ns = {'svg': 'http://www.w3.org/2000/svg'}
    for path_el in root.findall('.//svg:path', ns):
        d = path_el.get('d')
        if not d:
            continue
        # Lift pen before starting a new sub‑path (unless this is the very first)
        if all_points:
            last_x, last_y, _ = all_points[-1]
            all_points.append((last_x, last_y, 0))
        seg_points = path_to_points(d, vb)
        all_points.extend(seg_points)
    return all_points


def main() -> None:
    parser = argparse.ArgumentParser(description="Convert SVG to laser‑plotter points.")
    parser.add_argument('svg_file', help='Path to the input SVG file')
    parser.add_argument('--output', '-o', help='Output file (default: stdout)')
    args = parser.parse_args()

    points = process_svg(args.svg_file)
    out_stream = sys.stdout if not args.output else open(args.output, 'w')
    for x, y, pen in points:
        out_stream.write(f"{x},{y},{pen}\n")
    if args.output:
        out_stream.close()


if __name__ == "__main__":
    main()
