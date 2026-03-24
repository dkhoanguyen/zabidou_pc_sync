import time
import numpy as np
import cv2

from .camera_base import LucidCamera, FrameData


class PhoenixCamera(LucidCamera):
    """
    Lucid Phoenix RGB camera (PHX series, IMX264/IMX265 sensor family).

    Captures colour frames and exposes them as BGR numpy arrays ready for
    OpenCV processing.

    Supported pixel formats
    -----------------------
    ``'BayerRG8'``   – raw 8-bit Bayer mosaic (default, lowest bandwidth)
    ``'BayerRG16'``  – raw 16-bit Bayer mosaic
    ``'RGB8'``       – on-camera ISP, 8-bit RGB
    ``'BGR8'``       – on-camera ISP, 8-bit BGR
    ``'Mono8'``      – monochrome 8-bit
    ``'Mono16'``     – monochrome 16-bit
    """

    DEVICE_MODEL = 'PHX'   # Phoenix model names start with "PHX"

    _BAYER_FORMATS = {
        'BayerRG8':  (cv2.COLOR_BayerBG2BGR,    np.uint8),
        'BayerRG16': (cv2.COLOR_BayerBG2BGR_EA,  np.uint16),
        # Arena SDK uses the GEV convention where "RG" at pixel (0,0) maps to
        # OpenCV's BayerBG pattern after a 180° rotation.  Adjust if your
        # specific sensor layout differs.
    }
    _RGB_FORMATS  = {'RGB8', 'BGR8'}
    _MONO_FORMATS = {'Mono8', 'Mono16'}

    def __init__(
        self,
        device,
        pixel_format: str = 'BayerRG8',
        binning: int = 2,
        binning_mode: str = 'Average',
    ):
        """
        Parameters
        ----------
        device
            Arena SDK device object.
        pixel_format
            Sensor output format (see class docstring).
        binning
            Spatial binning factor (1 = off, 2 = 2×2, …).
        binning_mode
            ``'Average'`` or ``'Sum'`` – only used when binning > 1.
        """
        super().__init__(device)
        self._pixel_format = pixel_format
        self._binning = binning
        self._binning_mode = binning_mode

    # ------------------------------------------------------------------
    # LucidCamera interface
    # ------------------------------------------------------------------

    def configure(self):
        """Apply pixel format, binning, and stream transport settings."""
        nm = self._nodemap

        nm['PixelFormat'].value = self._pixel_format

        if self._binning > 1:
            nm['BinningHorizontal'].value = self._binning
            nm['BinningVertical'].value = self._binning
            nm['BinningHorizontalMode'].value = self._binning_mode
            nm['BinningVerticalMode'].value = self._binning_mode
        else:
            # Reset to 1 in case a previous session changed it
            nm['BinningHorizontal'].value = 1
            nm['BinningVertical'].value = 1

        self._apply_stream_defaults()

    def get_frame(self) -> FrameData:
        """
        Capture one colour frame.

        Returns
        -------
        FrameData
            ``image`` – uint8 BGR array of shape (H, W, 3), or (H, W) for mono.
        """
        buffer = self._device.get_buffer()
        try:
            return self._buffer_to_frame(buffer)
        finally:
            self._device.requeue_buffer(buffer)

    # ------------------------------------------------------------------
    # Private helpers
    # ------------------------------------------------------------------

    def _buffer_to_frame(self, buffer) -> FrameData:
        h, w = buffer.height, buffer.width
        fmt = self._pixel_format

        data_u8 = np.array(buffer.data, dtype=np.uint8)

        if fmt in self._BAYER_FORMATS:
            bgr = self._debayer(data_u8, h, w, fmt)
        elif fmt == 'RGB8':
            raw = data_u8.reshape(h, w, 3)
            bgr = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
        elif fmt == 'BGR8':
            bgr = data_u8.reshape(h, w, 3).copy()
        elif fmt == 'Mono8':
            bgr = data_u8.reshape(h, w).copy()
        elif fmt == 'Mono16':
            bgr = data_u8.view(np.uint16).reshape(h, w).copy()
        else:
            raise RuntimeError(f'Unsupported pixel format: {fmt!r}')

        return FrameData(
            timestamp=time.time(),
            width=w,
            height=h,
            image=bgr,
        )

    def _debayer(self, data_u8: np.ndarray, h: int, w: int, fmt: str) -> np.ndarray:
        cvt_code, dtype = self._BAYER_FORMATS[fmt]
        raw = data_u8.view(dtype).reshape(h, w)
        if dtype == np.uint16:
            # Normalise to 8-bit for downstream OpenCV compatibility
            raw8 = cv2.normalize(raw, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)
            return cv2.cvtColor(raw8, cv2.COLOR_BayerBG2BGR)
        return cv2.cvtColor(raw, cvt_code)

    # ------------------------------------------------------------------
    # Convenience: live preview loop
    # ------------------------------------------------------------------

    def preview(self, window_name: str = 'Phoenix Preview', quit_key: str = 'q'):
        """
        Open an OpenCV window and stream frames until ``quit_key`` is pressed.

        Requires the camera to already be configured and streaming.
        """
        if not self._streaming:
            raise RuntimeError('Call start_stream() before preview().')

        while True:
            frame = self.get_frame()
            cv2.imshow(window_name, frame.image)
            if cv2.waitKey(1) & 0xFF == ord(quit_key):
                break

        cv2.destroyWindow(window_name)
