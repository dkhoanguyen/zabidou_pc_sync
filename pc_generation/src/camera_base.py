from abc import ABC, abstractmethod
from dataclasses import dataclass, field
from typing import Optional
import numpy as np
from arena_api.system import system


@dataclass
class FrameData:
    """Generic container for a single captured frame."""
    timestamp: float = 0.0
    width: int = 0
    height: int = 0
    # Subclasses populate whichever fields are relevant
    image: Optional[np.ndarray] = None          # RGB/BGR image (H x W x 3)
    depth: Optional[np.ndarray] = None          # Depth map in mm (H x W)
    xyz: Optional[np.ndarray] = None            # 3-D coords in mm (H x W x 3)
    intensity: Optional[np.ndarray] = None      # Intensity / IR channel (H x W)


class LucidCamera(ABC):
    """
    Abstract base class for Lucid Vision cameras accessed via the Arena SDK.

    Usage pattern
    -------------
    cam = Helios2Camera(device)          # or PhoenixCamera(device)
    cam.configure()
    cam.start_stream()
    try:
        while True:
            frame = cam.get_frame()
            ...
    finally:
        cam.stop_stream()
        cam.release()
    """

    # Override in each subclass to enforce a model-name check.
    DEVICE_MODEL: Optional[str] = None

    def __init__(self, device):
        """
        Parameters
        ----------
        device
            An Arena SDK device object returned by ``system.create_device()``.
        """
        self._device = device
        self._streaming = False
        self._nodemap = device.nodemap
        self._tl_nodemap = device.tl_stream_nodemap

    # ------------------------------------------------------------------
    # Class-level helpers for device discovery
    # ------------------------------------------------------------------

    @classmethod
    def find_devices(cls, model_substring: Optional[str] = None):
        """
        Return Arena SDK device objects, optionally filtered by model name.

        Parameters
        ----------
        model_substring
            Case-insensitive substring to match against the device model string.
            Pass ``None`` to return all connected devices.
        """
        devices = system.create_device()
        if not devices:
            raise RuntimeError('No Lucid cameras detected on the network.')
        if model_substring is None:
            return devices
        filtered = [
            d for d in devices
            if model_substring.lower() in d.nodemap['DeviceModelName'].value.lower()
        ]
        if not filtered:
            available = [d.nodemap['DeviceModelName'].value for d in devices]
            raise RuntimeError(
                f"No device matching '{model_substring}' found. "
                f"Available models: {available}"
            )
        return filtered

    @classmethod
    def from_model(cls, model_substring: Optional[str] = None, index: int = 0, **kwargs):
        """
        Convenience factory: find a device by model name and return a camera instance.

        Parameters
        ----------
        model_substring
            Passed to ``find_devices``; defaults to ``cls.DEVICE_MODEL``.
        index
            Which matching device to use (0-based).
        """
        query = model_substring or cls.DEVICE_MODEL
        devices = cls.find_devices(query)
        return cls(devices[index], **kwargs)

    # ------------------------------------------------------------------
    # Properties
    # ------------------------------------------------------------------

    @property
    def model_name(self) -> str:
        return self._nodemap['DeviceModelName'].value

    @property
    def serial_number(self) -> str:
        return self._nodemap['DeviceSerialNumber'].value

    @property
    def is_streaming(self) -> bool:
        return self._streaming

    # ------------------------------------------------------------------
    # Stream transport settings (common to all Lucid cameras)
    # ------------------------------------------------------------------

    def _apply_stream_defaults(self):
        """Apply recommended Arena stream-transport settings."""
        self._tl_nodemap['StreamBufferHandlingMode'].value = 'NewestOnly'
        self._tl_nodemap['StreamAutoNegotiatePacketSize'].value = True
        self._tl_nodemap['StreamPacketResendEnable'].value = True

    # ------------------------------------------------------------------
    # Abstract interface
    # ------------------------------------------------------------------

    @abstractmethod
    def configure(self):
        """
        Apply sensor-specific nodemap settings (pixel format, binning, etc.).
        Must be called once before ``start_stream()``.
        """

    def start_stream(self, num_buffers: int = 1):
        """Start image acquisition."""
        if self._streaming:
            return
        self._device.start_stream(num_buffers)
        self._streaming = True

    def stop_stream(self):
        """Stop image acquisition."""
        if not self._streaming:
            return
        self._device.stop_stream()
        self._streaming = False

    def release(self):
        """Stop the stream (if running) and destroy the device."""
        self.stop_stream()
        system.destroy_device(self._device)

    @abstractmethod
    def get_frame(self) -> FrameData:
        """
        Acquire and return one frame from the camera.

        The caller must *not* hold onto the underlying Arena buffer between
        calls; the buffer is re-queued before this method returns.
        """

    # ------------------------------------------------------------------
    # Context-manager support
    # ------------------------------------------------------------------

    def __enter__(self):
        self.configure()
        self.start_stream()
        return self

    def __exit__(self, *_):
        self.release()

    def __repr__(self):
        return (
            f'<{self.__class__.__name__} model={self.model_name!r} '
            f'serial={self.serial_number!r} streaming={self._streaming}>'
        )
