"""
Phoenix RGB camera — live stream example.

Displays the colour feed in an OpenCV window.  Press 'q' to quit.

Usage
-----
    python examples/phoenix_stream.py
    python examples/phoenix_stream.py --format RGB8 --binning 2
"""

import argparse
import sys
import time
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src import PhoenixCamera


def parse_args():
    p = argparse.ArgumentParser(description='Phoenix live colour stream')
    p.add_argument(
        '--format',
        choices=['BayerRG8', 'BayerRG16', 'RGB8', 'BGR8', 'Mono8', 'Mono16'],
        default='BayerRG8',
        help='Pixel format (default: BayerRG8)',
    )
    p.add_argument('--binning', type=int, default=2,
                   help='Spatial binning factor (default: 1 = off)')
    p.add_argument('--binning-mode', choices=['Average', 'Sum'], default='Average',
                   help='Binning mode when --binning > 1 (default: Average)')
    p.add_argument('--device-index', type=int, default=0,
                   help='Which Phoenix device to open (default: 0)')
    return p.parse_args()


def main():
    args = parse_args()

    print(f'Opening Phoenix camera (index {args.device_index}, '
          f'format {args.format}, binning {args.binning}×) …')

    cam = PhoenixCamera.from_model(
        index=args.device_index,
        pixel_format=args.format,
        binning=args.binning,
        binning_mode=args.binning_mode,
    )
    print(f'  {cam}')

    cam.configure()
    cam.start_stream()

    try:
        prev_time = time.time()
        while True:
            frame = cam.get_frame()

            cv2.imshow('Phoenix RGB', frame.image)

            now = time.time()
            fps = 1.0 / max(now - prev_time, 1e-6)
            prev_time = now
            print(f'\rFPS {fps:.1f}  {frame.width}×{frame.height}    ', end='', flush=True)

            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

    finally:
        cam.release()
        cv2.destroyAllWindows()
        print('\nStream stopped.')


if __name__ == '__main__':
    main()
