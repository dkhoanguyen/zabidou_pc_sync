import time
import numpy as np
import cv2

from .camera_base import LucidCamera, FrameData


class PhoenixCamera(LucidCamera):
    """
    Lucid Phoenix RGB polarization camera (PHX050S1-Q / Sony IMX250MYR family).

    For polarized color raw formats, the sensor output is a 2x2 polarization
    mosaic overlaid with Bayer color sampling. Therefore, raw Bayer conversion
    on the full frame is incorrect.

    This class returns the S0/intensity BGR image by default when using raw
    polarized Bayer formats. The four per-angle BGR images are also available
    in frame.metadata["pol_bgr"].
    """

    DEVICE_MODEL = "PHX"

    _POLAR_BAYER_FORMATS = {
        "BayerRG8": np.uint8,
        "BayerRG16": np.uint16,
    }
    _RGB_FORMATS = {"RGB8", "BGR8"}
    _MONO_FORMATS = {"Mono8", "Mono16"}

    def __init__(
        self,
        device,
        pixel_format: str = "BayerRG8",
        binning: int = 2,
        binning_mode: str = "Average",
    ):
        super().__init__(device)
        self._pixel_format = pixel_format
        self._binning = binning
        self._binning_mode = binning_mode

    def configure(self):
        nm = self._nodemap

        nm["PixelFormat"].value = self._pixel_format

        if self._binning > 1:
            nm["BinningHorizontal"].value = self._binning
            nm["BinningVertical"].value = self._binning
            nm["BinningHorizontalMode"].value = self._binning_mode
            nm["BinningVerticalMode"].value = self._binning_mode
        else:
            nm["BinningHorizontal"].value = 1
            nm["BinningVertical"].value = 1

        self._apply_stream_defaults()

    def get_frame(self) -> FrameData:
        buffer = self._device.get_buffer()
        try:
            return self._buffer_to_frame(buffer)
        finally:
            self._device.requeue_buffer(buffer)

    def _buffer_to_frame(self, buffer) -> FrameData:
        h, w = buffer.height, buffer.width
        fmt = self._pixel_format

        data_u8 = np.array(buffer.data, dtype=np.uint8)

        metadata = {}

        if fmt in self._POLAR_BAYER_FORMATS:
            bgr, pol_bgr = self._debayer_polarized_color(data_u8, h, w, fmt)
            metadata["pol_bgr"] = pol_bgr
        elif fmt == "RGB8":
            raw = data_u8.reshape(h, w, 3)
            bgr = cv2.cvtColor(raw, cv2.COLOR_RGB2BGR)
        elif fmt == "BGR8":
            bgr = data_u8.reshape(h, w, 3).copy()
        elif fmt == "Mono8":
            bgr = data_u8.reshape(h, w).copy()
        elif fmt == "Mono16":
            bgr = data_u8.view(np.uint16).reshape(h, w).copy()
        else:
            raise RuntimeError(f"Unsupported pixel format: {fmt!r}")

        frame = FrameData(
            timestamp=time.time(),
            width=w,
            height=h,
            image=bgr,
        )

        # Attach optional metadata if FrameData supports dynamic attributes.
        # Remove this block if FrameData is a frozen dataclass / namedtuple.
        try:
            frame.metadata = metadata
        except Exception:
            pass

        return frame

    def _debayer_polarized_color(
        self,
        data_u8: np.ndarray,
        h: int,
        w: int,
        fmt: str,
    ) -> tuple[np.ndarray, dict[str, np.ndarray]]:
        dtype = self._POLAR_BAYER_FORMATS[fmt]
        raw = data_u8.view(dtype).reshape(h, w)

        # For OpenCV display / debayer convenience, convert 16-bit to 8-bit.
        if dtype == np.uint16:
            raw = cv2.normalize(raw, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8U)

        # Split 2x2 polarization mosaic first.
        #
        # Assumed Sony Polarsens layout:
        #   [ 90°,  45° ]
        #   [135°,   0° ]
        #
        # If colors look wrong or AoLP later looks rotated, keep this split but
        # adjust only the angle labels after verifying against vendor output.
        raw_90  = raw[0::2, 0::2]
        raw_45  = raw[0::2, 1::2]
        raw_135 = raw[1::2, 0::2]
        raw_0   = raw[1::2, 1::2]

        # Each polarization plane is still Bayer.
        # After 2x2 subsampling, the Bayer phase can shift depending on sensor layout.
        # BayerBG is a common correct mapping here for Arena/LUCID output, but if
        # colors are swapped, try BayerRG/BayerGR/BayerGB.
        bgr_0   = cv2.cvtColor(raw_0,   cv2.COLOR_BayerBG2BGR)
        bgr_45  = cv2.cvtColor(raw_45,  cv2.COLOR_BayerBG2BGR)
        bgr_90  = cv2.cvtColor(raw_90,  cv2.COLOR_BayerBG2BGR)
        bgr_135 = cv2.cvtColor(raw_135, cv2.COLOR_BayerBG2BGR)

        # Build a default display image.
        # S0/intensity ~= (I0 + I90), computed per color channel.
        s0 = (
            bgr_0.astype(np.float32) +
            bgr_90.astype(np.float32)
        ) * 0.5
        s0 = np.clip(s0, 0, 255).astype(np.uint8)

        pol_bgr = {
            "0": bgr_0,
            "45": bgr_45,
            "90": bgr_90,
            "135": bgr_135,
        }

        return s0, pol_bgr

    def preview(self, window_name: str = "Phoenix Preview", quit_key: str = "q"):
        if not self._streaming:
            raise RuntimeError("Call start_stream() before preview().")

        while True:
            frame = self.get_frame()
            cv2.imshow(window_name, frame.image)
            if cv2.waitKey(1) & 0xFF == ord(quit_key):
                break

        cv2.destroyWindow(window_name)