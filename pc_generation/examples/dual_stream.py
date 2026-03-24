"""
Dual-camera example — Helios2 (depth) + Phoenix (RGB).

Opens each camera in a separate thread and prints the timestamp of every
captured frame.  Press Ctrl-C to stop.

Usage
-----
    python examples/dual_stream.py
"""

import sys
import threading
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from src import Helios2Camera, PhoenixCamera


stop_event = threading.Event()
print_lock = threading.Lock()


def helios2_thread(cam: Helios2Camera):
    cam.configure()
    cam.start_stream()
    try:
        while not stop_event.is_set():
            frame = cam.get_frame()
            with print_lock:
                print(f'[Helios2 ] ts={frame.timestamp:.6f}')
    finally:
        cam.release()


def phoenix_thread(cam: PhoenixCamera):
    cam.configure()
    cam.start_stream()
    try:
        while not stop_event.is_set():
            frame = cam.get_frame()
            with print_lock:
                print(f'[Phoenix ] ts={frame.timestamp:.6f}')
    finally:
        cam.release()


def main():
    print('Discovering cameras …')
    try:
        helios = Helios2Camera.from_model()
        print(f'  Helios2 : {helios}')
    except RuntimeError as e:
        print(f'  Helios2 not found: {e}')
        helios = None

    try:
        phoenix = PhoenixCamera.from_model()
        print(f'  Phoenix : {phoenix}')
    except RuntimeError as e:
        print(f'  Phoenix not found: {e}')
        phoenix = None

    if helios is None and phoenix is None:
        print('No supported cameras found. Exiting.')
        return

    threads = []
    if helios is not None:
        threads.append(threading.Thread(target=helios2_thread, args=(helios,), daemon=True))
    if phoenix is not None:
        threads.append(threading.Thread(target=phoenix_thread, args=(phoenix,), daemon=True))

    print('Streaming — press Ctrl-C to stop.')
    for t in threads:
        t.start()
    try:
        while any(t.is_alive() for t in threads):
            time.sleep(0.1)
    except KeyboardInterrupt:
        print('\nStopping …')
        stop_event.set()
    for t in threads:
        t.join()
    print('Done.')


if __name__ == '__main__':
    main()
