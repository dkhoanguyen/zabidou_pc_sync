import ctypes
import time
import numpy as np
from arena_api.enums import PixelFormat

from .camera_base import LucidCamera, FrameData


class Helios2Camera(LucidCamera):
    """
    Lucid Helios2 Time-of-Flight depth camera.

    Acquires ``Coord3D_ABCY16`` (unsigned) or ``Coord3D_ABCY16s`` (signed)
    frames and exposes them as metric (mm) XYZ point-cloud arrays plus an
    intensity channel.

    Pixel-format layout (4 channels × 16-bit per pixel)
    ----------------------------------------------------
    Channel A  →  X coordinate
    Channel B  →  Y coordinate
    Channel C  →  Z coordinate (depth)
    Channel Y  →  Intensity / amplitude
    """

    DEVICE_MODEL = 'HTP'   # Helios2+ model names start with "HTP"

    # Pixel formats this class supports (in preference order)
    _SUPPORTED_FORMATS = ('Coord3D_ABCY16', 'Coord3D_ABCY16s')

    def __init__(
        self,
        device,
        pixel_format: str = 'Coord3D_ABCY16',
        hdr_mode: str = '',
        operating_mode: str = '',
        exposure_time_selector: str = '',
    ):
        """
        Parameters
        ----------
        device
            Arena SDK device object.
        pixel_format
            ``'Coord3D_ABCY16'``  – unsigned 16-bit (default)
            ``'Coord3D_ABCY16s'`` – signed 16-bit
        """
        super().__init__(device)
        if pixel_format not in self._SUPPORTED_FORMATS:
            raise ValueError(
                f'pixel_format must be one of {self._SUPPORTED_FORMATS}, '
                f'got {pixel_format!r}'
            )
        self._pixel_format = pixel_format
        self._hdr_mode = hdr_mode
        self._operating_mode = operating_mode
        self._exposure_time_selector = exposure_time_selector

        # Populated during configure()
        self._scale_x: float = 1.0
        self._scale_y: float = 1.0
        self._scale_z: float = 1.0
        self._offset_x: float = 0.0
        self._offset_y: float = 0.0

    # ------------------------------------------------------------------
    # LucidCamera interface
    # ------------------------------------------------------------------

    def configure(self):
        """Set pixel format, read coordinate scales/offsets."""
        self._nodemap['PixelFormat'].value = self._pixel_format
        if self._operating_mode:
            try:
                self._nodemap['Scan3dOperatingMode'].value = self._operating_mode
            except Exception:
                # Operating-mode naming varies across some Helios firmware variants.
                pass
        if self._exposure_time_selector:
            try:
                self._nodemap['ExposureTimeSelector'].value = self._exposure_time_selector
            except Exception:
                # Exposure-time presets may vary by firmware or model.
                pass
        if self._hdr_mode:
            try:
                self._nodemap['Scan3dHDRMode'].value = self._hdr_mode
            except Exception:
                # HDR control naming varies across some Helios firmware variants.
                pass
        self._apply_stream_defaults()
        self._read_coordinate_metadata()

    def get_frame(self) -> FrameData:
        """
        Capture one depth frame.

        Returns
        -------
        FrameData
            ``xyz``       – float32 array of shape (H, W, 3), values in mm.
            ``depth``     – float32 array of shape (H, W), Z values in mm
                            (invalid pixels are 0).
            ``intensity`` – uint16 array of shape (H, W).
        """
        buffer = self._device.get_buffer()
        try:
            return self._buffer_to_frame(buffer)
        finally:
            self._device.requeue_buffer(buffer)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _read_coordinate_metadata(self):
        nm = self._nodemap
        nm['Scan3dCoordinateSelector'].value = 'CoordinateA'
        self._scale_x = nm['Scan3dCoordinateScale'].value
        self._offset_x = nm['Scan3dCoordinateOffset'].value

        nm['Scan3dCoordinateSelector'].value = 'CoordinateB'
        self._scale_y = nm['Scan3dCoordinateScale'].value
        self._offset_y = nm['Scan3dCoordinateOffset'].value

        nm['Scan3dCoordinateSelector'].value = 'CoordinateC'
        self._scale_z = nm['Scan3dCoordinateScale'].value

    def _buffer_to_frame(self, buffer) -> FrameData:
        h, w = buffer.height, buffer.width
        channels_per_pixel = buffer.bits_per_pixel // 16  # should be 4

        if buffer.pixel_format == PixelFormat.Coord3D_ABCY16s:
            raw = self._cast_buffer(buffer, ctypes.c_int16, h, w, channels_per_pixel)
            xyz, intensity = self._decode_signed(raw)
        elif buffer.pixel_format == PixelFormat.Coord3D_ABCY16:
            raw = self._cast_buffer(buffer, ctypes.c_uint16, h, w, channels_per_pixel)
            xyz, intensity = self._decode_unsigned(raw)
        else:
            raise RuntimeError(
                f'Unexpected pixel format: {buffer.pixel_format}. '
                f'Expected Coord3D_ABCY16 or Coord3D_ABCY16s.'
            )

        depth = xyz[:, :, 2].copy()

        return FrameData(
            timestamp=time.time(),
            width=w,
            height=h,
            xyz=xyz,
            depth=depth,
            intensity=intensity,
        )

    @staticmethod
    def _cast_buffer(buffer, ctype, h: int, w: int, cpp: int) -> np.ndarray:
        """Cast the Arena buffer pointer to a numpy array (H, W, cpp)."""
        ptr = ctypes.cast(buffer.pdata, ctypes.POINTER(ctype))
        flat = np.ctypeslib.as_array(ptr, shape=(h * w * cpp,))
        return flat.reshape(h, w, cpp)

    def _decode_signed(self, raw: np.ndarray):
        """
        Decode signed Coord3D_ABCY16s data.

        Returns (xyz float32 H×W×3 in mm, intensity uint16 H×W).
        """
        x_raw = raw[:, :, 0].astype(np.float32) * self._scale_x
        y_raw = raw[:, :, 1].astype(np.float32) * self._scale_y
        z_raw = raw[:, :, 2].astype(np.float32) * self._scale_z
        intensity = raw[:, :, 3].astype(np.uint16)

        # Zero out invalid (zero-depth) pixels
        valid = z_raw > 0
        x_raw[~valid] = 0.0
        y_raw[~valid] = 0.0
        z_raw[~valid] = 0.0

        xyz = np.stack([x_raw, y_raw, z_raw], axis=-1)
        return xyz, intensity

    def _decode_unsigned(self, raw: np.ndarray):
        """
        Decode unsigned Coord3D_ABCY16 data.

        Invalid pixels are marked with 0xFFFF in the Z channel; they are
        zeroed out in the output.

        Returns (xyz float32 H×W×3 in mm, intensity uint16 H×W).
        """
        INVALID = 0xFFFF

        x_raw = raw[:, :, 0].astype(np.float32)
        y_raw = raw[:, :, 1].astype(np.float32)
        z_raw = raw[:, :, 2].astype(np.uint32)   # keep uint to compare to 0xFFFF
        intensity = raw[:, :, 3].astype(np.uint16)

        valid = z_raw < INVALID

        x_mm = np.where(valid, x_raw * self._scale_x + self._offset_x, 0.0).astype(np.float32)
        y_mm = np.where(valid, y_raw * self._scale_y + self._offset_y, 0.0).astype(np.float32)
        z_mm = np.where(valid, z_raw.astype(np.float32) * self._scale_z, 0.0).astype(np.float32)

        xyz = np.stack([x_mm, y_mm, z_mm], axis=-1)
        return xyz, intensity

    # ------------------------------------------------------------------
    # Convenience accessors for the last frame's coordinate metadata
    # ------------------------------------------------------------------

    @property
    def coordinate_scales(self) -> tuple:
        """Return (scale_x, scale_y, scale_z) as configured on the device."""
        return self._scale_x, self._scale_y, self._scale_z

    @property
    def coordinate_offsets(self) -> tuple:
        """Return (offset_x, offset_y) as configured on the device."""
        return self._offset_x, self._offset_y
