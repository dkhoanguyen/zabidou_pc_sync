#include "calibration/opencv_calibration.hpp"

#include <stdexcept>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>

namespace calibration {

namespace {

std::vector<cv::Point3f> make_board_object_points(const cv::Size& board_size,
                                                  float square_size_meters) {
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<std::size_t>(board_size.width * board_size.height));

    for (int row = 0; row < board_size.height; ++row) {
        for (int col = 0; col < board_size.width; ++col) {
            points.emplace_back(static_cast<float>(col) * square_size_meters,
                                static_cast<float>(row) * square_size_meters,
                                0.0F);
        }
    }

    return points;
}

cv::Mat to_mono8(const cv::Mat& input) {
    if (input.empty()) {
        return {};
    }

    cv::Mat gray;
    if (input.channels() == 1) {
        gray = input;
    } else {
        cv::cvtColor(input, gray, cv::COLOR_BGR2GRAY);
    }

    if (gray.type() == CV_8UC1) {
        return gray;
    }

    cv::Mat normalized;
    cv::normalize(gray, normalized, 0, 255, cv::NORM_MINMAX);
    normalized.convertTo(normalized, CV_8UC1);
    return normalized;
}

bool detect_chessboard(const cv::Mat& image,
                       const cv::Size& board_size,
                       bool use_fast_check,
                       std::vector<cv::Point2f>& corners_out) {
    const int flags = cv::CALIB_CB_ADAPTIVE_THRESH |
                      cv::CALIB_CB_NORMALIZE_IMAGE |
                      (use_fast_check ? cv::CALIB_CB_FAST_CHECK : 0);

    corners_out.clear();
    const auto mono = to_mono8(image);
    if (mono.empty()) {
        return false;
    }

    const bool found = cv::findChessboardCorners(mono, board_size, corners_out, flags);
    if (!found) {
        return false;
    }

    cv::cornerSubPix(
        mono,
        corners_out,
        cv::Size(11, 11),
        cv::Size(-1, -1),
        cv::TermCriteria(cv::TermCriteria::EPS + cv::TermCriteria::COUNT, 30, 1e-3));

    return true;
}

}  // namespace

OpenCVCalibration::OpenCVCalibration(int board_rows,
                                     int board_cols,
                                     float square_size_meters,
                                     bool use_fast_check)
    : board_size_(board_cols, board_rows),
      square_size_meters_(square_size_meters),
      use_fast_check_(use_fast_check) {
    if (board_rows <= 0 || board_cols <= 0 || square_size_meters <= 0.0F) {
        throw std::invalid_argument("Invalid chessboard dimensions or square size");
    }
}

void OpenCVCalibration::add_frame(const FramePair& pair) {
    frames_.push_back(pair);
}

bool OpenCVCalibration::calibrate() {
    if (frames_.size() < 3) {
        return false;
    }

    std::vector<std::vector<cv::Point2f>> rgb_image_points;
    std::vector<std::vector<cv::Point2f>> depth_image_points;
    std::vector<std::vector<cv::Point3f>> object_points;

    cv::Size rgb_image_size;
    cv::Size depth_image_size;

    for (const auto& frame : frames_) {
        std::vector<cv::Point2f> rgb_corners;
        std::vector<cv::Point2f> depth_corners;

        const bool rgb_found = detect_chessboard(frame.rgb.bgr, board_size_, use_fast_check_, rgb_corners);
        const bool depth_found =
            detect_chessboard(frame.depth.intensity, board_size_, use_fast_check_, depth_corners);

        if (!(rgb_found && depth_found)) {
            continue;
        }

        if (rgb_image_size.empty()) {
            rgb_image_size = frame.rgb.bgr.size();
        }
        if (depth_image_size.empty()) {
            depth_image_size = frame.depth.intensity.size();
        }

        rgb_image_points.push_back(std::move(rgb_corners));
        depth_image_points.push_back(std::move(depth_corners));
        object_points.push_back(make_board_object_points(board_size_, square_size_meters_));
    }

    if (object_points.size() < 3) {
        return false;
    }

    cv::Mat rgb_camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat rgb_dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
    cv::Mat depth_camera_matrix = cv::Mat::eye(3, 3, CV_64F);
    cv::Mat depth_dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);

    std::vector<cv::Mat> rvecs;
    std::vector<cv::Mat> tvecs;

    cv::calibrateCamera(object_points,
                        rgb_image_points,
                        rgb_image_size,
                        rgb_camera_matrix,
                        rgb_dist_coeffs,
                        rvecs,
                        tvecs);

    cv::calibrateCamera(object_points,
                        depth_image_points,
                        depth_image_size,
                        depth_camera_matrix,
                        depth_dist_coeffs,
                        rvecs,
                        tvecs);

    cv::Mat R;
    cv::Mat T;
    cv::Mat E;
    cv::Mat F;

    const double stereo_rms = cv::stereoCalibrate(object_points,
                                                  rgb_image_points,
                                                  depth_image_points,
                                                  rgb_camera_matrix,
                                                  rgb_dist_coeffs,
                                                  depth_camera_matrix,
                                                  depth_dist_coeffs,
                                                  rgb_image_size,
                                                  R,
                                                  T,
                                                  E,
                                                  F,
                                                  cv::CALIB_FIX_INTRINSIC,
                                                  cv::TermCriteria(cv::TermCriteria::COUNT + cv::TermCriteria::EPS,
                                                                   100,
                                                                   1e-5));

    result_.rgb_intrinsics.camera_matrix = rgb_camera_matrix;
    result_.rgb_intrinsics.dist_coeffs = rgb_dist_coeffs;
    result_.rgb_intrinsics.image_size = rgb_image_size;

    result_.depth_intrinsics.camera_matrix = depth_camera_matrix;
    result_.depth_intrinsics.dist_coeffs = depth_dist_coeffs;
    result_.depth_intrinsics.image_size = depth_image_size;

    result_.R = R;
    result_.T = T;
    result_.rms_error = stereo_rms;

    return true;
}

CalibResult OpenCVCalibration::result() const {
    return result_;
}

void OpenCVCalibration::save(const std::string& path) const {
    cv::FileStorage fs(path, cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        throw std::runtime_error("Failed to open output path for calibration save: " + path);
    }

    fs << "backend" << name();
    fs << "rms_error" << result_.rms_error;

    fs << "rgb_camera_matrix" << result_.rgb_intrinsics.camera_matrix;
    fs << "rgb_dist_coeffs" << result_.rgb_intrinsics.dist_coeffs;
    fs << "rgb_image_width" << result_.rgb_intrinsics.image_size.width;
    fs << "rgb_image_height" << result_.rgb_intrinsics.image_size.height;

    fs << "depth_camera_matrix" << result_.depth_intrinsics.camera_matrix;
    fs << "depth_dist_coeffs" << result_.depth_intrinsics.dist_coeffs;
    fs << "depth_image_width" << result_.depth_intrinsics.image_size.width;
    fs << "depth_image_height" << result_.depth_intrinsics.image_size.height;

    fs << "R" << result_.R;
    fs << "T" << result_.T;
}

void OpenCVCalibration::load(const std::string& path) {
    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Failed to open calibration file: " + path);
    }

    fs["rms_error"] >> result_.rms_error;

    fs["rgb_camera_matrix"] >> result_.rgb_intrinsics.camera_matrix;
    fs["rgb_dist_coeffs"] >> result_.rgb_intrinsics.dist_coeffs;
    fs["rgb_image_width"] >> result_.rgb_intrinsics.image_size.width;
    fs["rgb_image_height"] >> result_.rgb_intrinsics.image_size.height;

    fs["depth_camera_matrix"] >> result_.depth_intrinsics.camera_matrix;
    fs["depth_dist_coeffs"] >> result_.depth_intrinsics.dist_coeffs;
    fs["depth_image_width"] >> result_.depth_intrinsics.image_size.width;
    fs["depth_image_height"] >> result_.depth_intrinsics.image_size.height;

    fs["R"] >> result_.R;
    fs["T"] >> result_.T;
}

std::string OpenCVCalibration::name() const {
    return "opencv";
}

}  // namespace calibration
