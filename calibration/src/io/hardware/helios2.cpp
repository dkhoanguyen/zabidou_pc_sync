#include "calibration/io/hardware/helios2.hpp"

#include <algorithm>
#include <array>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include "calibration/io/common/lucid_device_utils.hpp"
#include <opencv2/core.hpp>

#if CALIBRATION_HAS_ARENA_SDK
#include "ArenaApi.h"
#endif

namespace calibration {

namespace {

#if CALIBRATION_HAS_ARENA_SDK
template <typename T>
cv::Mat wrap_buffer_channel(const T* buffer_data,
                            int width,
                            int height,
                            int channel_index,
                            int channels_per_pixel,
                            int cv_type) {
    cv::Mat channel(height, width, cv_type);

    for (int row = 0; row < height; ++row) {
        const T* row_src = buffer_data + static_cast<std::size_t>(row) * width * channels_per_pixel;
        T* row_dst = channel.ptr<T>(row);
        for (int col = 0; col < width; ++col) {
            row_dst[col] = row_src[static_cast<std::size_t>(col) * channels_per_pixel + channel_index];
        }
    }

    return channel;
}

DepthFrame decode_signed_image(Arena::IImage* image,
                               float scale_x,
                               float scale_y,
                               float scale_z) {
    const int width = static_cast<int>(image->GetWidth());
    const int height = static_cast<int>(image->GetHeight());
    const int channels_per_pixel = static_cast<int>(image->GetBitsPerPixel() / 16);
    if (channels_per_pixel != 4) {
        throw std::runtime_error("Helios2 signed frame expected 4 channels per pixel");
    }

    const auto* data = reinterpret_cast<const std::int16_t*>(image->GetData());

    cv::Mat xyz(height, width, CV_32FC3, cv::Scalar(0.0F, 0.0F, 0.0F));
    cv::Mat intensity = wrap_buffer_channel<std::uint16_t>(
        reinterpret_cast<const std::uint16_t*>(data), width, height, 3, channels_per_pixel, CV_16UC1);

    for (int row = 0; row < height; ++row) {
        const auto* row_src =
            data + static_cast<std::size_t>(row) * width * channels_per_pixel;
        auto* row_xyz = xyz.ptr<cv::Vec3f>(row);

        for (int col = 0; col < width; ++col) {
            const std::size_t base = static_cast<std::size_t>(col) * channels_per_pixel;
            const float z_mm = static_cast<float>(row_src[base + 2]) * scale_z;
            if (z_mm <= 0.0F) {
                row_xyz[col] = cv::Vec3f(0.0F, 0.0F, 0.0F);
                continue;
            }

            row_xyz[col][0] = static_cast<float>(row_src[base + 0]) * scale_x;
            row_xyz[col][1] = static_cast<float>(row_src[base + 1]) * scale_y;
            row_xyz[col][2] = z_mm;
        }
    }

    return DepthFrame{xyz, intensity, image->GetTimestampNs()};
}

DepthFrame decode_unsigned_image(Arena::IImage* image,
                                 float scale_x,
                                 float scale_y,
                                 float scale_z,
                                 float offset_x,
                                 float offset_y) {
    const int width = static_cast<int>(image->GetWidth());
    const int height = static_cast<int>(image->GetHeight());
    const int channels_per_pixel = static_cast<int>(image->GetBitsPerPixel() / 16);
    if (channels_per_pixel != 4) {
        throw std::runtime_error("Helios2 unsigned frame expected 4 channels per pixel");
    }

    constexpr std::uint16_t kInvalidDepth = std::numeric_limits<std::uint16_t>::max();
    const auto* data = reinterpret_cast<const std::uint16_t*>(image->GetData());

    cv::Mat xyz(height, width, CV_32FC3, cv::Scalar(0.0F, 0.0F, 0.0F));
    cv::Mat intensity = wrap_buffer_channel<std::uint16_t>(
        data, width, height, 3, channels_per_pixel, CV_16UC1);

    for (int row = 0; row < height; ++row) {
        const auto* row_src =
            data + static_cast<std::size_t>(row) * width * channels_per_pixel;
        auto* row_xyz = xyz.ptr<cv::Vec3f>(row);

        for (int col = 0; col < width; ++col) {
            const std::size_t base = static_cast<std::size_t>(col) * channels_per_pixel;
            const std::uint16_t z_raw = row_src[base + 2];
            if (z_raw >= kInvalidDepth) {
                row_xyz[col] = cv::Vec3f(0.0F, 0.0F, 0.0F);
                continue;
            }

            row_xyz[col][0] = static_cast<float>(row_src[base + 0]) * scale_x + offset_x;
            row_xyz[col][1] = static_cast<float>(row_src[base + 1]) * scale_y + offset_y;
            row_xyz[col][2] = static_cast<float>(z_raw) * scale_z;
        }
    }

    return DepthFrame{xyz, intensity, image->GetTimestampNs()};
}
#endif

}  // namespace

Helios2::Helios2(std::size_t device_index, std::string pixel_format)
    : device_index_(device_index), pixel_format_(std::move(pixel_format)) {}

Helios2::~Helios2() {
    close();
}

void Helios2::open() {
#if CALIBRATION_HAS_ARENA_SDK
    if (is_open_) {
        return;
    }

    common::CameraInfo camera_info;
    try {
        device_ = common::create_lucid_device_by_prefix("HTP", device_index_, &camera_info);
        camera_label_ = common::format_camera_label(camera_info, "Helios2");
        configure_device();
        read_coordinate_metadata();
        device_->StartStream(1);
        is_open_ = true;
    } catch (...) {
        close();
        throw;
    }
#else
    const auto cameras = common::find_lucid_cameras();
    const auto camera_info = common::find_camera_by_prefix(cameras, "HTP", device_index_);
    if (camera_info.has_value()) {
        camera_label_ = common::format_camera_label(*camera_info, "Helios2");
    } else {
        camera_label_ = "Helios2[index=" + std::to_string(device_index_) + "]";
    }
    is_open_ = true;
#endif
}

void Helios2::close() {
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

bool Helios2::is_open() const {
    return is_open_;
}

bool Helios2::grab(DepthFrame& out, std::uint32_t timeout_ms) {
    if (!is_open_) {
        return false;
    }

#if CALIBRATION_HAS_ARENA_SDK
    if (device_ == nullptr) {
        return false;
    }

    Arena::IImage* image = device_->GetImage(timeout_ms);
    try {
        const auto pixel_format = image->GetPixelFormat();
        if (pixel_format == Coord3D_ABCY16s) {
            out = decode_signed_image(image, scale_x_, scale_y_, scale_z_);
        } else if (pixel_format == Coord3D_ABCY16) {
            out = decode_unsigned_image(
                image, scale_x_, scale_y_, scale_z_, offset_x_, offset_y_);
        } else {
            throw std::runtime_error(
                "Unexpected Helios2 pixel format: " + std::string(GetPixelFormatName(static_cast<PfncFormat>(pixel_format))));
        }
    } catch (...) {
        device_->RequeueBuffer(image);
        throw;
    }

    device_->RequeueBuffer(image);
    return true;
#else
    out.intensity = cv::Mat::zeros(480, 640, CV_16UC1);
    out.xyz = cv::Mat::zeros(480, 640, CV_32FC3);
    out.timestamp_ns = common::now_ns();
    return true;
#endif
}

std::string Helios2::model_name() const {
    return camera_label_;
}

void Helios2::configure_device() {
#if CALIBRATION_HAS_ARENA_SDK
    if (device_ == nullptr) {
        throw std::runtime_error("Helios2 device is not initialized");
    }

    if (pixel_format_ != "Coord3D_ABCY16" && pixel_format_ != "Coord3D_ABCY16s") {
        throw std::invalid_argument(
            "Helios2 pixel format must be Coord3D_ABCY16 or Coord3D_ABCY16s");
    }

    Arena::SetNodeValue<GenICam::gcstring>(device_->GetNodeMap(), "PixelFormat", pixel_format_.c_str());
    Arena::SetNodeValue<GenICam::gcstring>(
        device_->GetTLStreamNodeMap(), "StreamBufferHandlingMode", "NewestOnly");
    Arena::SetNodeValue<bool>(device_->GetTLStreamNodeMap(), "StreamAutoNegotiatePacketSize", true);
    Arena::SetNodeValue<bool>(device_->GetTLStreamNodeMap(), "StreamPacketResendEnable", true);
#endif
}

void Helios2::read_coordinate_metadata() {
#if CALIBRATION_HAS_ARENA_SDK
    if (device_ == nullptr) {
        throw std::runtime_error("Helios2 device is not initialized");
    }

    auto* node_map = device_->GetNodeMap();

    Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateA");
    scale_x_ = static_cast<float>(Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateScale"));
    offset_x_ = static_cast<float>(Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset"));

    Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateB");
    scale_y_ = static_cast<float>(Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateScale"));
    offset_y_ = static_cast<float>(Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset"));

    Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateC");
    scale_z_ = static_cast<float>(Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateScale"));
#endif
}

}  // namespace calibration
