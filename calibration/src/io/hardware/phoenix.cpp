#include "calibration/io/hardware/phoenix.hpp"

#include <string>
#include <stdexcept>
#include <utility>
#include <vector>

#include "calibration/io/common/lucid_device_utils.hpp"
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#if CALIBRATION_HAS_ARENA_SDK
#include "ArenaApi.h"
#endif

namespace calibration {

namespace {

#if CALIBRATION_HAS_ARENA_SDK
cv::Mat image_to_bgr(Arena::IImage* image, const std::string& pixel_format) {
    Arena::IImage* converted = nullptr;

    try {
        if (pixel_format == "RGB8" || pixel_format == "BGR8") {
            converted = Arena::ImageFactory::Convert(image, BGR8);
        } else if (pixel_format == "Mono8" || pixel_format == "Mono16") {
            converted = Arena::ImageFactory::Convert(image, Mono8);
        } else {
            converted = Arena::ImageFactory::Convert(image, BGR8);
        }

        const int width = static_cast<int>(converted->GetWidth());
        const int height = static_cast<int>(converted->GetHeight());
        const int type = (pixel_format == "Mono8" || pixel_format == "Mono16") ? CV_8UC1 : CV_8UC3;

        cv::Mat wrapped(
            height,
            width,
            type,
            const_cast<std::uint8_t*>(converted->GetData()));

        cv::Mat result = wrapped.clone();
        Arena::ImageFactory::Destroy(converted);
        return result;
    } catch (...) {
        if (converted != nullptr) {
            Arena::ImageFactory::Destroy(converted);
        }
        throw;
    }
}
#endif

}  // namespace

Phoenix::Phoenix(std::size_t device_index,
                 std::string pixel_format,
                 int binning,
                 std::string binning_mode,
                 double acquisition_frame_rate_hz,
                 std::size_t stream_buffer_count)
    : device_index_(device_index),
      pixel_format_(std::move(pixel_format)),
      binning_(binning),
      binning_mode_(std::move(binning_mode)),
      acquisition_frame_rate_hz_(acquisition_frame_rate_hz),
      stream_buffer_count_(stream_buffer_count) {}

Phoenix::~Phoenix() {
    close();
}

void Phoenix::open() {
#if CALIBRATION_HAS_ARENA_SDK
    if (is_open_) {
        return;
    }

    common::CameraInfo camera_info;
    try {
        device_ = common::create_lucid_device_by_prefix("PHX", device_index_, &camera_info);
        camera_label_ = common::format_camera_label(camera_info, "Phoenix");
        configure_device();
        device_->StartStream(static_cast<std::size_t>(stream_buffer_count_));
        is_open_ = true;
    } catch (...) {
        close();
        throw;
    }
#else
    const auto cameras = common::find_lucid_cameras();
    const auto camera_info = common::find_camera_by_prefix(cameras, "PHX", device_index_);
    if (camera_info.has_value()) {
        camera_label_ = common::format_camera_label(*camera_info, "Phoenix");
    } else {
        camera_label_ = "Phoenix[index=" + std::to_string(device_index_) + "]";
    }
    is_open_ = true;
#endif
}

void Phoenix::close() {
#if CALIBRATION_HAS_ARENA_SDK
    if (device_ != nullptr) {
        try {
            device_->StopStream();
        } catch (...) {
        }

        common::destroy_lucid_device(device_);
        device_ = nullptr;
    }
#endif

    is_open_ = false;
}

bool Phoenix::is_open() const {
    return is_open_;
}

bool Phoenix::grab(RgbFrame& out, std::uint32_t timeout_ms) {
    if (!is_open_) {
        return false;
    }

#if CALIBRATION_HAS_ARENA_SDK
    if (device_ == nullptr) {
        return false;
    }

    Arena::IImage* image = device_->GetImage(timeout_ms);
    try {
        out.bgr = image_to_bgr(image, pixel_format_);
        out.timestamp_ns = image->GetTimestampNs();
    } catch (...) {
        device_->RequeueBuffer(image);
        throw;
    }

    device_->RequeueBuffer(image);
    return true;
#else
    out.bgr = cv::Mat::zeros(480, 640, CV_8UC3);
    out.timestamp_ns = common::now_ns();
    return true;
#endif
}

std::string Phoenix::model_name() const {
    return camera_label_;
}

void Phoenix::preview(const std::string& window_name,
                      std::uint32_t timeout_ms,
                      char quit_key) {
    if (!is_open()) {
        open();
    }

    while (true) {
        RgbFrame frame;
        if (!grab(frame, timeout_ms)) {
            continue;
        }

        cv::imshow(window_name, frame.bgr);
        if ((cv::waitKey(1) & 0xFF) == static_cast<unsigned char>(quit_key)) {
            break;
        }
    }

    cv::destroyWindow(window_name);
}

void Phoenix::configure_device() {
#if CALIBRATION_HAS_ARENA_SDK
    if (device_ == nullptr) {
        throw std::runtime_error("Phoenix device is not initialized");
    }

    Arena::SetNodeValue<GenICam::gcstring>(device_->GetNodeMap(), "PixelFormat", pixel_format_.c_str());
    Arena::SetNodeValue<bool>(device_->GetNodeMap(), "AcquisitionFrameRateEnable", true);
    Arena::SetNodeValue<double>(device_->GetNodeMap(), "AcquisitionFrameRate", acquisition_frame_rate_hz_);

    if (binning_ > 1) {
        Arena::SetNodeValue<int64_t>(device_->GetNodeMap(), "BinningHorizontal", binning_);
        Arena::SetNodeValue<int64_t>(device_->GetNodeMap(), "BinningVertical", binning_);
        Arena::SetNodeValue<GenICam::gcstring>(
            device_->GetNodeMap(), "BinningHorizontalMode", binning_mode_.c_str());
        Arena::SetNodeValue<GenICam::gcstring>(
            device_->GetNodeMap(), "BinningVerticalMode", binning_mode_.c_str());
    } else {
        Arena::SetNodeValue<int64_t>(device_->GetNodeMap(), "BinningHorizontal", 1);
        Arena::SetNodeValue<int64_t>(device_->GetNodeMap(), "BinningVertical", 1);
    }

    Arena::SetNodeValue<GenICam::gcstring>(
        device_->GetTLStreamNodeMap(), "StreamBufferHandlingMode", stream_buffer_handling_mode_.c_str());
    Arena::SetNodeValue<bool>(
        device_->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", auto_negotiate_packet_size_);
    Arena::SetNodeValue<bool>(
        device_->GetTLStreamNodeMap(), "StreamPacketResendEnable", packet_resend_enable_);
#endif
}

}  // namespace calibration
