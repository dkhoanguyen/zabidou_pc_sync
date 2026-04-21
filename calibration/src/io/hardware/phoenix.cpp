#include "calibration/io/hardware/phoenix.hpp"

#include <cstdint>
#include <cstring>
#include <iostream>
#include <algorithm>
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
template <typename Value>
bool try_set_node_value(GenApi::INodeMap* node_map,
                        const char* node_name,
                        const Value& value,
                        const std::string& context = {}) {
    try {
        Arena::SetNodeValue<Value>(node_map, node_name, value);
        return true;
    } catch (const std::exception& ex) {
        std::cerr << "[Phoenix] Warning: could not set " << node_name;
        if (!context.empty()) {
            std::cerr << " (" << context << ")";
        }
        std::cerr << ": " << ex.what() << "\n";
        return false;
    }
}

void configure_frame_rate(GenApi::INodeMap* node_map, double requested_fps) {
    if (requested_fps <= 0.0) {
        return;
    }

    if (!try_set_node_value<bool>(node_map, "AcquisitionFrameRateEnable", true)) {
        return;
    }

    try {
        GenApi::CFloatPtr frame_rate_node = node_map->GetNode("AcquisitionFrameRate");
        if (frame_rate_node == nullptr || !GenApi::IsWritable(frame_rate_node)) {
            std::cerr << "[Phoenix] Warning: AcquisitionFrameRate is not writable; leaving camera default\n";
            return;
        }

        const double min_fps = frame_rate_node->GetMin();
        const double max_fps = frame_rate_node->GetMax();
        const double clamped_fps = std::clamp(requested_fps, min_fps, max_fps);
        if (clamped_fps != requested_fps) {
            std::cerr << "[Phoenix] Warning: requested FPS " << requested_fps
                      << " is outside camera range [" << min_fps << ", " << max_fps
                      << "]; using " << clamped_fps << "\n";
        }
        frame_rate_node->SetValue(clamped_fps);
    } catch (const std::exception& ex) {
        std::cerr << "[Phoenix] Warning: could not configure AcquisitionFrameRate="
                  << requested_fps << ": " << ex.what() << "\n";
    }
}

cv::Mat debayer_polarized_color(Arena::IImage* image, const std::string& pixel_format) {
    const int width = static_cast<int>(image->GetWidth());
    const int height = static_cast<int>(image->GetHeight());
    if (width <= 1 || height <= 1) {
        throw std::runtime_error("Phoenix image is too small to debayer polarized Bayer data");
    }

    cv::Mat raw;
    if (pixel_format == "BayerRG8") {
        raw = cv::Mat(height, width, CV_8UC1, const_cast<std::uint8_t*>(image->GetData())).clone();
    } else if (pixel_format == "BayerRG16") {
        cv::Mat raw16(height, width, CV_16UC1, const_cast<std::uint8_t*>(image->GetData()));
        cv::normalize(raw16, raw, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    } else {
        throw std::runtime_error("Unsupported polarized Bayer format: " + pixel_format);
    }

    const int sub_height = height / 2;
    const int sub_width = width / 2;
    cv::Mat raw_90(sub_height, sub_width, CV_8UC1);
    cv::Mat raw_45(sub_height, sub_width, CV_8UC1);
    cv::Mat raw_135(sub_height, sub_width, CV_8UC1);
    cv::Mat raw_0(sub_height, sub_width, CV_8UC1);

    for (int row = 0; row < sub_height; ++row) {
        const int src_row0 = row * 2;
        const int src_row1 = src_row0 + 1;
        for (int col = 0; col < sub_width; ++col) {
            const int src_col0 = col * 2;
            const int src_col1 = src_col0 + 1;
            raw_90.at<std::uint8_t>(row, col) = raw.at<std::uint8_t>(src_row0, src_col0);
            raw_45.at<std::uint8_t>(row, col) = raw.at<std::uint8_t>(src_row0, src_col1);
            raw_135.at<std::uint8_t>(row, col) = raw.at<std::uint8_t>(src_row1, src_col0);
            raw_0.at<std::uint8_t>(row, col) = raw.at<std::uint8_t>(src_row1, src_col1);
        }
    }

    cv::Mat bgr_0;
    cv::Mat bgr_45;
    cv::Mat bgr_90;
    cv::Mat bgr_135;
    cv::cvtColor(raw_0, bgr_0, cv::COLOR_BayerBG2BGR);
    cv::cvtColor(raw_45, bgr_45, cv::COLOR_BayerBG2BGR);
    cv::cvtColor(raw_90, bgr_90, cv::COLOR_BayerBG2BGR);
    cv::cvtColor(raw_135, bgr_135, cv::COLOR_BayerBG2BGR);

    cv::Mat s0;
    cv::addWeighted(bgr_0, 0.5, bgr_90, 0.5, 0.0, s0);
    return s0;
}

cv::Mat image_to_bgr(Arena::IImage* image, const std::string& pixel_format) {
    Arena::IImage* converted = nullptr;

    try {
        if (pixel_format == "BayerRG8" || pixel_format == "BayerRG16") {
            return debayer_polarized_color(image, pixel_format);
        }

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

    auto* node_map = device_->GetNodeMap();
    auto* stream_node_map = device_->GetTLStreamNodeMap();

    Arena::SetNodeValue<GenICam::gcstring>(node_map, "PixelFormat", pixel_format_.c_str());
    configure_frame_rate(node_map, acquisition_frame_rate_hz_);

    if (binning_ > 1) {
        const bool horizontal_ok = try_set_node_value<int64_t>(
            node_map, "BinningHorizontal", binning_, "falling back to 1 if rejected");
        const bool vertical_ok = try_set_node_value<int64_t>(
            node_map, "BinningVertical", binning_, "falling back to 1 if rejected");
        if (horizontal_ok && vertical_ok) {
            try_set_node_value<GenICam::gcstring>(
                node_map, "BinningHorizontalMode", binning_mode_.c_str());
            try_set_node_value<GenICam::gcstring>(
                node_map, "BinningVerticalMode", binning_mode_.c_str());
        } else {
            try_set_node_value<int64_t>(node_map, "BinningHorizontal", 1);
            try_set_node_value<int64_t>(node_map, "BinningVertical", 1);
        }
    } else {
        try_set_node_value<int64_t>(node_map, "BinningHorizontal", 1);
        try_set_node_value<int64_t>(node_map, "BinningVertical", 1);
    }

    try_set_node_value<GenICam::gcstring>(
        stream_node_map, "StreamBufferHandlingMode", stream_buffer_handling_mode_.c_str());
    try_set_node_value<bool>(
        stream_node_map, "StreamAutoNegotiatePacketSize", auto_negotiate_packet_size_);
    try_set_node_value<bool>(
        stream_node_map, "StreamPacketResendEnable", packet_resend_enable_);
#endif
}

}  // namespace calibration
