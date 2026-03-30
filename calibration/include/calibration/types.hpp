#ifndef CALIBRATION__TYPES_HPP_
#define CALIBRATION__TYPES_HPP_

#include <cstdint>

#include <opencv2/core.hpp>

namespace calibration {

struct DepthFrame {
    cv::Mat xyz;
    cv::Mat intensity;
    std::uint64_t timestamp_ns{0};
};

struct RgbFrame {
    cv::Mat bgr;
    std::uint64_t timestamp_ns{0};
};

struct FramePair {
    RgbFrame rgb;
    DepthFrame depth;
};

struct CameraIntrinsics {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Size image_size;
};

struct CalibResult {
    CameraIntrinsics rgb_intrinsics;
    CameraIntrinsics depth_intrinsics;
    cv::Mat R;
    cv::Mat T;
    double rms_error{0.0};
};

}  // namespace calibration

#endif  // CALIBRATION__TYPES_HPP_
