#!/usr/bin/env python3
"""
Record a Phoenix camera sequence in a TUM-style layout for ORB-SLAM3.

Output layout:
    pc_generation/dataset/<session>/
      rgb/
        000000.png
        000001.png
        ...
      rgb.txt
      times.txt
      orbslam3_phoenix.yaml
      metadata.yaml

For BayerRG8/BayerRG16 Phoenix polarization sensors, the script splits the 2x2
polarization mosaic first, debayers the 0 and 90 degree planes, then saves the
S0/intensity colour result. Use --raw only for debugging; raw Bayer/polarization
frames are not the recommended ORB-SLAM3 input.
"""

from __future__ import annotations

import argparse
import ctypes
import datetime as dt
import sys
import time
from pathlib import Path

import cv2
import numpy as np


REPO_ROOT = Path(__file__).resolve().parents[1]
PC_GENERATION_ROOT = REPO_ROOT / "pc_generation"
DEFAULT_DATASET_ROOT = PC_GENERATION_ROOT / "dataset"
DEFAULT_CALIBRATION = REPO_ROOT / "calibration" / "results" / "combined_all_datasets" / "mono_rgb_calibration.yaml"

if str(PC_GENERATION_ROOT) not in sys.path:
    sys.path.insert(0, str(PC_GENERATION_ROOT))


BAYER_CODES = {
    "BG": cv2.COLOR_BayerBG2BGR,
    "GB": cv2.COLOR_BayerGB2BGR,
    "GR": cv2.COLOR_BayerGR2BGR,
    "RG": cv2.COLOR_BayerRG2BGR,
}

RAW_BAYER_FORMATS = {"BayerRG8", "BayerRG16"}
MONO_FORMATS = {"Mono8", "Mono16"}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Capture Phoenix frames into a TUM-style ORB-SLAM3 monocular dataset."
    )
    parser.add_argument("--phoenix-index", type=int, default=0, help="Phoenix device index.")
    parser.add_argument(
        "--output-root",
        type=Path,
        default=DEFAULT_DATASET_ROOT,
        help="Parent directory for the created dataset session.",
    )
    parser.add_argument(
        "--session-name",
        default=None,
        help="Dataset session folder name. Defaults to data_<YYYYmmdd_HHMMSS>.",
    )
    parser.add_argument(
        "--num-frames",
        type=int,
        default=0,
        help="Frames to record. Use 0 to record until Ctrl+C or --duration expires.",
    )
    parser.add_argument(
        "--duration",
        type=float,
        default=None,
        help="Optional recording duration in seconds.",
    )
    parser.add_argument(
        "--warmup",
        type=int,
        default=5,
        help="Frames to discard before recording.",
    )
    parser.add_argument(
        "--pixel-format",
        choices=["BayerRG8", "BayerRG16", "RGB8", "BGR8", "Mono8", "Mono16"],
        default="BayerRG8",
        help="Camera pixel format to request.",
    )
    parser.add_argument("--binning", type=int, default=2, help="Spatial binning factor.")
    parser.add_argument(
        "--binning-mode",
        choices=["Average", "Sum"],
        default="Average",
        help="Binning mode when --binning is greater than 1.",
    )
    parser.add_argument(
        "--bayer-code",
        choices=sorted(BAYER_CODES),
        default="BG",
        help="Bayer conversion to use after splitting the Phoenix polarization mosaic.",
    )
    parser.add_argument(
        "--raw",
        action="store_true",
        help="Save the captured raw frame directly, without debayering. Not recommended for ORB-SLAM3.",
    )
    parser.add_argument(
        "--fps",
        type=float,
        default=30.0,
        help="Requested acquisition frame rate. Use 0 to leave the camera default unchanged.",
    )
    parser.add_argument(
        "--exposure-us",
        type=float,
        default=None,
        help="Optional fixed exposure time in microseconds. Shorter exposure can raise FPS.",
    )
    parser.add_argument(
        "--calibration-yaml",
        type=Path,
        default=DEFAULT_CALIBRATION,
        help="OpenCV calibration YAML used to generate the ORB-SLAM3 camera settings.",
    )
    parser.add_argument(
        "--image-format",
        choices=["png", "jpg"],
        default="png",
        help="Image file format for rgb/ frames.",
    )
    parser.add_argument(
        "--png-compression",
        type=int,
        default=1,
        help="PNG compression level. Lower is faster.",
    )
    parser.add_argument(
        "--jpg-quality",
        type=int,
        default=95,
        help="JPEG quality when --image-format jpg is selected.",
    )
    return parser.parse_args()


def find_phoenix(index: int):
    from arena_api.system import system

    devices = system.create_device()
    matches = [
        device
        for device in devices
        if device.nodemap["DeviceModelName"].value.upper().startswith("PHX")
    ]
    if index < 0 or index >= len(matches):
        available = [device.nodemap["DeviceModelName"].value for device in devices]
        raise RuntimeError(f"No Phoenix camera at index {index}. Available devices: {available}")
    return matches[index]


def try_set_node(nodemap, name: str, value) -> None:
    try:
        nodemap[name].value = value
    except Exception as exc:
        print(f"Warning: could not set {name}={value!r}: {exc}")


def configure_device(
    device,
    pixel_format: str,
    binning: int,
    binning_mode: str,
    fps: float,
    exposure_us: float | None,
) -> None:
    nodemap = device.nodemap
    nodemap["PixelFormat"].value = pixel_format

    if binning > 1:
        nodemap["BinningHorizontal"].value = binning
        nodemap["BinningVertical"].value = binning
        nodemap["BinningHorizontalMode"].value = binning_mode
        nodemap["BinningVerticalMode"].value = binning_mode
    else:
        nodemap["BinningHorizontal"].value = 1
        nodemap["BinningVertical"].value = 1

    if fps > 0:
        try_set_node(nodemap, "AcquisitionFrameRateEnable", True)
        try_set_node(nodemap, "AcquisitionFrameRate", fps)

    if exposure_us is not None:
        try_set_node(nodemap, "ExposureAuto", "Off")
        try_set_node(nodemap, "ExposureTime", exposure_us)

    stream_nodemap = device.tl_stream_nodemap
    stream_nodemap["StreamBufferHandlingMode"].value = "NewestOnly"
    stream_nodemap["StreamAutoNegotiatePacketSize"].value = True
    stream_nodemap["StreamPacketResendEnable"].value = True


def buffer_to_raw(buffer, pixel_format: str) -> np.ndarray:
    byte_count = buffer.width * buffer.height * buffer.bits_per_pixel // 8
    data = np.ctypeslib.as_array(
        ctypes.cast(buffer.pdata, ctypes.POINTER(ctypes.c_uint8)),
        shape=(byte_count,),
    )

    if pixel_format == "BayerRG8":
        return data.reshape(buffer.height, buffer.width)
    if pixel_format == "BayerRG16":
        return data.view(np.uint16).reshape(buffer.height, buffer.width)
    if pixel_format in {"RGB8", "BGR8"}:
        return data.reshape(buffer.height, buffer.width, 3)
    if pixel_format == "Mono8":
        return data.reshape(buffer.height, buffer.width)
    if pixel_format == "Mono16":
        return data.view(np.uint16).reshape(buffer.height, buffer.width)

    raise RuntimeError(f"Unsupported pixel format: {pixel_format}")


def normalize_to_u8(image: np.ndarray) -> np.ndarray:
    if image.dtype == np.uint8:
        return image.copy()
    return cv2.normalize(image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)


def debayer_polarized(raw: np.ndarray, bayer_code: str) -> np.ndarray:
    raw_u8 = normalize_to_u8(raw)
    even_h = raw_u8.shape[0] - (raw_u8.shape[0] % 2)
    even_w = raw_u8.shape[1] - (raw_u8.shape[1] % 2)
    raw_u8 = raw_u8[:even_h, :even_w]

    raw_90 = raw_u8[0::2, 0::2]
    raw_0 = raw_u8[1::2, 1::2]

    code = BAYER_CODES[bayer_code]
    bgr_90 = cv2.cvtColor(raw_90, code)
    bgr_0 = cv2.cvtColor(raw_0, code)
    return cv2.addWeighted(bgr_0, 0.5, bgr_90, 0.5, 0.0)


def raw_to_frame(raw: np.ndarray, pixel_format: str, bayer_code: str, save_raw: bool) -> np.ndarray:
    if save_raw:
        if pixel_format == "RGB8":
            return cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
        if pixel_format == "BGR8":
            return raw.copy()
        return normalize_to_u8(raw)

    if pixel_format in RAW_BAYER_FORMATS:
        return debayer_polarized(raw, bayer_code)
    if pixel_format == "RGB8":
        return cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
    if pixel_format == "BGR8":
        return raw.copy()
    if pixel_format in MONO_FORMATS:
        return normalize_to_u8(raw)
    raise RuntimeError(f"Unsupported pixel format: {pixel_format}")


def buffer_timestamp_seconds(buffer) -> float:
    for attr in ("timestamp_ns", "timestamp"):
        value = getattr(buffer, attr, None)
        if callable(value):
            value = value()
        if value is None:
            continue
        value = float(value)
        if attr == "timestamp_ns" or value > 1e12:
            return value / 1e9
        return value
    return time.time_ns() / 1e9


def create_session_dir(output_root: Path, session_name: str | None) -> Path:
    if session_name is None:
        session_name = "data_" + dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    session_dir = output_root / session_name
    rgb_dir = session_dir / "rgb"
    rgb_dir.mkdir(parents=True, exist_ok=False)
    return session_dir


def image_write_params(args: argparse.Namespace) -> tuple[str, list[int]]:
    if args.image_format == "png":
        return "png", [cv2.IMWRITE_PNG_COMPRESSION, args.png_compression]
    return "jpg", [cv2.IMWRITE_JPEG_QUALITY, args.jpg_quality]


def save_image(path: Path, image: np.ndarray, params: list[int]) -> None:
    if not cv2.imwrite(str(path), image, params):
        raise RuntimeError(f"Failed to write image: {path}")


def read_calibration(path: Path) -> dict[str, object] | None:
    if path is None or not path.exists():
        return None

    fs = cv2.FileStorage(str(path), cv2.FILE_STORAGE_READ)
    if not fs.isOpened():
        return None
    try:
        camera_matrix = fs.getNode("camera_matrix").mat()
        dist_coeffs = fs.getNode("dist_coeffs").mat()
        image_width = int(fs.getNode("image_width").real())
        image_height = int(fs.getNode("image_height").real())

        if camera_matrix is None or camera_matrix.size == 0:
            camera_matrix = fs.getNode("rgb_camera_matrix").mat()
            dist_coeffs = fs.getNode("rgb_dist_coeffs").mat()
            image_width = int(fs.getNode("rgb_image_width").real())
            image_height = int(fs.getNode("rgb_image_height").real())

        if camera_matrix is None or camera_matrix.size == 0:
            return None
        if dist_coeffs is None or dist_coeffs.size == 0:
            dist_coeffs = np.zeros((5, 1), dtype=np.float64)

        return {
            "camera_matrix": camera_matrix.astype(np.float64),
            "dist_coeffs": dist_coeffs.reshape(-1).astype(np.float64),
            "image_width": image_width,
            "image_height": image_height,
            "source": path,
        }
    finally:
        fs.release()


def default_calibration(width: int, height: int) -> dict[str, object]:
    fx = fy = float(max(width, height))
    cx = (width - 1) * 0.5
    cy = (height - 1) * 0.5
    return {
        "camera_matrix": np.array([[fx, 0.0, cx], [0.0, fy, cy], [0.0, 0.0, 1.0]], dtype=np.float64),
        "dist_coeffs": np.zeros(5, dtype=np.float64),
        "image_width": width,
        "image_height": height,
        "source": None,
    }


def scaled_calibration(calibration: dict[str, object], width: int, height: int) -> tuple[np.ndarray, np.ndarray]:
    camera_matrix = np.array(calibration["camera_matrix"], dtype=np.float64).copy()
    dist_coeffs = np.array(calibration["dist_coeffs"], dtype=np.float64).reshape(-1)
    source_width = int(calibration["image_width"])
    source_height = int(calibration["image_height"])

    if source_width > 0 and source_height > 0 and (source_width != width or source_height != height):
        scale_x = width / source_width
        scale_y = height / source_height
        camera_matrix[0, 0] *= scale_x
        camera_matrix[0, 2] *= scale_x
        camera_matrix[1, 1] *= scale_y
        camera_matrix[1, 2] *= scale_y

    return camera_matrix, dist_coeffs


def dist_value(dist_coeffs: np.ndarray, index: int) -> float:
    return float(dist_coeffs[index]) if index < dist_coeffs.size else 0.0


def write_orbslam3_settings(
    path: Path,
    calibration: dict[str, object],
    width: int,
    height: int,
    fps: float,
) -> None:
    camera_matrix, dist_coeffs = scaled_calibration(calibration, width, height)
    source = calibration.get("source")
    source_text = str(source) if source is not None else "none; placeholder intrinsics"

    text = f"""%YAML:1.0
---
# Generated for ORB-SLAM3 monocular/TUM input.
# Calibration source: {source_text}

File.version: "1.0"

Camera.type: "PinHole"

Camera.fx: {camera_matrix[0, 0]:.12g}
Camera.fy: {camera_matrix[1, 1]:.12g}
Camera.cx: {camera_matrix[0, 2]:.12g}
Camera.cy: {camera_matrix[1, 2]:.12g}

Camera.k1: {dist_value(dist_coeffs, 0):.12g}
Camera.k2: {dist_value(dist_coeffs, 1):.12g}
Camera.p1: {dist_value(dist_coeffs, 2):.12g}
Camera.p2: {dist_value(dist_coeffs, 3):.12g}
Camera.k3: {dist_value(dist_coeffs, 4):.12g}

Camera.width: {width}
Camera.height: {height}
Camera.fps: {fps:.12g}
Camera.RGB: 0

ORBextractor.nFeatures: 1250
ORBextractor.scaleFactor: 1.2
ORBextractor.nLevels: 8
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

Viewer.KeyFrameSize: 0.05
Viewer.KeyFrameLineWidth: 1.0
Viewer.GraphLineWidth: 0.9
Viewer.PointSize: 2.0
Viewer.CameraSize: 0.08
Viewer.CameraLineWidth: 3.0
Viewer.ViewpointX: 0.0
Viewer.ViewpointY: -0.7
Viewer.ViewpointZ: -1.8
Viewer.ViewpointF: 500.0
"""
    path.write_text(text, encoding="utf-8")


def write_orbslam3_readme(path: Path, session_dir: Path, settings_path: Path) -> None:
    text = f"""ORB-SLAM3 Monocular/TUM Dataset
================================

This session is organized for ORB-SLAM3's TUM monocular example.

Example command from an ORB-SLAM3 checkout:

    ./Examples/Monocular/mono_tum Vocabulary/ORBvoc.txt {settings_path} {session_dir}

Files:
    rgb/                 image frames
    rgb.txt              TUM-style timestamp/image list
    times.txt            timestamps only, useful for tooling
    orbslam3_phoenix.yaml camera and ORB extractor settings
    metadata.yaml        capture metadata

Use the debayered/default capture for SLAM. Datasets captured with --raw contain the raw
Bayer/polarization mosaic and are mainly useful for debugging.
"""
    path.write_text(text, encoding="utf-8")


def write_metadata(
    path: Path,
    args: argparse.Namespace,
    model: str,
    serial: str,
    width: int,
    height: int,
    saved_frames: int,
    measured_fps: float,
    calibration: dict[str, object],
) -> None:
    source = calibration.get("source")
    source_text = str(source) if source is not None else ""
    text = f"""%YAML:1.0
---
dataset_format: tum_monocular
intended_consumer: ORB-SLAM3 Examples/Monocular/mono_tum
camera_model: "{model}"
camera_serial: "{serial}"
pixel_format: "{args.pixel_format}"
raw_saved: {str(args.raw).lower()}
bayer_code: "{args.bayer_code}"
binning: {args.binning}
binning_mode: "{args.binning_mode}"
requested_fps: {args.fps}
measured_save_fps: {measured_fps:.6f}
exposure_us: {args.exposure_us if args.exposure_us is not None else -1}
image_width: {width}
image_height: {height}
saved_frames: {saved_frames}
image_format: "{args.image_format}"
rgb_txt: "rgb.txt"
times_txt: "times.txt"
orbslam3_settings: "orbslam3_phoenix.yaml"
calibration_source: "{source_text}"
"""
    path.write_text(text, encoding="utf-8")


def should_stop(saved_frames: int, start_time: float, args: argparse.Namespace) -> bool:
    if args.num_frames > 0 and saved_frames >= args.num_frames:
        return True
    if args.duration is not None and time.time() - start_time >= args.duration:
        return True
    return False


def main() -> int:
    args = parse_args()

    from arena_api.system import system

    session_dir = create_session_dir(args.output_root, args.session_name)
    rgb_dir = session_dir / "rgb"
    rgb_txt_path = session_dir / "rgb.txt"
    times_txt_path = session_dir / "times.txt"
    settings_path = session_dir / "orbslam3_phoenix.yaml"
    metadata_path = session_dir / "metadata.yaml"
    readme_path = session_dir / "README_ORBSLAM3.txt"
    extension, write_params = image_write_params(args)

    device = None
    streaming = False
    saved_frames = 0
    measured_fps = 0.0
    first_frame_shape = None
    interrupted = False

    try:
        device = find_phoenix(args.phoenix_index)
        model = device.nodemap["DeviceModelName"].value
        serial = device.nodemap["DeviceSerialNumber"].value
        print(f"Opening Phoenix camera: {model} serial={serial}")
        print(f"Recording ORB-SLAM3 dataset into: {session_dir}")

        configure_device(
            device,
            args.pixel_format,
            args.binning,
            args.binning_mode,
            args.fps,
            args.exposure_us,
        )
        device.start_stream(8)
        streaming = True

        for _ in range(max(0, args.warmup)):
            buffer = device.get_buffer()
            device.requeue_buffer(buffer)

        start_time = time.time()
        fps_time = start_time
        frames_since_report = 0

        with rgb_txt_path.open("w", encoding="utf-8") as rgb_txt, times_txt_path.open(
            "w", encoding="utf-8"
        ) as times_txt:
            rgb_txt.write("# timestamp filename\n")

            try:
                while not should_stop(saved_frames, start_time, args):
                    buffer = device.get_buffer()
                    try:
                        timestamp = buffer_timestamp_seconds(buffer)
                        raw = buffer_to_raw(buffer, args.pixel_format)
                        frame = raw_to_frame(raw, args.pixel_format, args.bayer_code, args.raw)
                    finally:
                        device.requeue_buffer(buffer)

                    if first_frame_shape is None:
                        first_frame_shape = frame.shape

                    image_name = f"{saved_frames:06d}.{extension}"
                    relative_path = f"rgb/{image_name}"
                    save_image(rgb_dir / image_name, frame, write_params)
                    rgb_txt.write(f"{timestamp:.9f} {relative_path}\n")
                    times_txt.write(f"{timestamp:.9f}\n")

                    saved_frames += 1
                    frames_since_report += 1
                    now = time.time()
                    elapsed = now - fps_time
                    if elapsed >= 1.0:
                        measured_fps = frames_since_report / elapsed
                        frames_since_report = 0
                        fps_time = now
                        print(f"\rSaved {saved_frames} frames  {measured_fps:5.1f} FPS", end="", flush=True)
            except KeyboardInterrupt:
                interrupted = True
                print("\nStopping recording and finalizing dataset...")

        if saved_frames == 0 or first_frame_shape is None:
            raise RuntimeError("No frames were recorded.")

        total_elapsed = max(time.time() - start_time, 1e-9)
        measured_fps = saved_frames / total_elapsed
        height, width = first_frame_shape[:2]

        calibration = read_calibration(args.calibration_yaml)
        if calibration is None:
            print(f"\nWarning: could not read calibration YAML: {args.calibration_yaml}")
            print("Writing placeholder intrinsics. Replace them before running serious SLAM.")
            calibration = default_calibration(width, height)

        write_orbslam3_settings(settings_path, calibration, width, height, args.fps or measured_fps)
        write_orbslam3_readme(readme_path, session_dir, settings_path)
        write_metadata(
            metadata_path,
            args,
            model,
            serial,
            width,
            height,
            saved_frames,
            measured_fps,
            calibration,
        )

        print()
        print(f"Saved {saved_frames} frames at {measured_fps:.2f} FPS.")
        print(f"Dataset: {session_dir}")
        print("ORB-SLAM3 mono_tum inputs:")
        print(f"  sequence path: {session_dir}")
        print(f"  settings:      {settings_path}")
        return 130 if interrupted else 0

    except KeyboardInterrupt:
        print("\nStopping recording...")
        return 130
    finally:
        if device is not None:
            try:
                if streaming:
                    device.stop_stream()
            finally:
                system.destroy_device(device)


if __name__ == "__main__":
    raise SystemExit(main())
