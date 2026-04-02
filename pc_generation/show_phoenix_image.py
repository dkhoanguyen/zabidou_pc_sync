#!/usr/bin/env python3

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import cv2
from arena_api.system import system


REPO_ROOT = Path("/home/khoa/Projects/zabidou_pc_sync")
PC_GENERATION_ROOT = REPO_ROOT / "pc_generation"
if str(PC_GENERATION_ROOT) not in sys.path:
    sys.path.insert(0, str(PC_GENERATION_ROOT))

from src.phoenix import PhoenixCamera


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture one debayered image from the Phoenix camera and display it."
    )
    parser.add_argument("--phoenix-index", type=int, default=0, help="Phoenix device index.")
    parser.add_argument(
        "--pixel-format",
        default="BayerRG8",
        choices=["BayerRG8", "BayerRG16", "RGB8", "BGR8", "Mono8", "Mono16"],
        help="Phoenix pixel format.",
    )
    parser.add_argument("--binning", type=int, default=1, help="Phoenix binning factor.")
    parser.add_argument(
        "--binning-mode",
        default="Average",
        choices=["Average", "Sum"],
        help="Phoenix binning mode.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="Optional path to save the captured image.",
    )
    return parser.parse_args()


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


def main() -> int:
    args = parse_args()

    phoenix = None
    try:
        phoenix_device = find_device_by_prefix("PHX", args.phoenix_index)
        phoenix = PhoenixCamera(
            phoenix_device,
            pixel_format=args.pixel_format,
            binning=args.binning,
            binning_mode=args.binning_mode,
        )
        phoenix.configure()
        phoenix.start_stream()

        frame = phoenix.get_frame()
        image = frame.image

        if args.output is not None:
            args.output.parent.mkdir(parents=True, exist_ok=True)
            if not cv2.imwrite(str(args.output), image):
                raise RuntimeError(f"Failed to write image to {args.output}")
            print(f"Saved image to {args.output}")

        window_name = "Phoenix Debayered Image"
        cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
        cv2.imshow(window_name, image)
        print("Showing debayered Phoenix image. Press any key in the image window to close.")
        cv2.waitKey(0)
        cv2.destroyAllWindows()
        return 0
    finally:
        if phoenix is not None:
            phoenix.release()


if __name__ == "__main__":
    raise SystemExit(main())
