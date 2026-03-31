#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdint>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sensor_msgs/point_cloud2_iterator.hpp>

#include "calibration/io/hardware/helios2.hpp"
#include "calibration/io/hardware/phoenix.hpp"
#include "calibration/io/live_frame_source.hpp"

namespace {

struct StereoCalibrationData {
    cv::Mat rgb_camera_matrix;
    cv::Mat rgb_dist_coeffs;
    cv::Mat R;
    cv::Mat T;
};

struct ColoredPoint {
    cv::Vec3f xyz_m;
    cv::Vec3b rgb;
};

StereoCalibrationData load_stereo_calibration(const std::filesystem::path& path) {
    cv::FileStorage fs(path.string(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Failed to open stereo calibration file: " + path.string());
    }

    StereoCalibrationData data;
    fs["rgb_camera_matrix"] >> data.rgb_camera_matrix;
    fs["rgb_dist_coeffs"] >> data.rgb_dist_coeffs;
    fs["R"] >> data.R;
    fs["T"] >> data.T;

    if (data.rgb_camera_matrix.empty() || data.rgb_dist_coeffs.empty() || data.R.empty() ||
        data.T.empty()) {
        throw std::runtime_error("Stereo calibration file is missing required matrices");
    }

    return data;
}

std::vector<ColoredPoint> colorize_point_cloud(const calibration::FramePair& pair,
                                               const StereoCalibrationData& calibration) {
    std::vector<ColoredPoint> points;
    if (pair.depth.xyz.empty() || pair.rgb.bgr.empty()) {
        return points;
    }

    const cv::Mat& R = calibration.R;
    const cv::Mat& T = calibration.T;
    const cv::Mat& K = calibration.rgb_camera_matrix;
    const cv::Mat& D = calibration.rgb_dist_coeffs;

    std::vector<cv::Point3f> points_rgb_frame;
    std::vector<cv::Point3f> points_depth_frame_m;
    points_rgb_frame.reserve(static_cast<std::size_t>(pair.depth.xyz.rows * pair.depth.xyz.cols));
    points_depth_frame_m.reserve(static_cast<std::size_t>(pair.depth.xyz.rows * pair.depth.xyz.cols));

    for (int row = 0; row < pair.depth.xyz.rows; ++row) {
        const auto* row_xyz = pair.depth.xyz.ptr<cv::Vec3f>(row);
        for (int col = 0; col < pair.depth.xyz.cols; ++col) {
            const cv::Vec3f xyz_mm = row_xyz[col];
            if (xyz_mm[2] <= 0.0F) {
                continue;
            }

            const cv::Mat point_depth = (cv::Mat_<double>(3, 1)
                                         << xyz_mm[0] / 1000.0, xyz_mm[1] / 1000.0, xyz_mm[2] / 1000.0);
            const cv::Mat point_rgb = R.t() * (point_depth - T);
            const double z_rgb = point_rgb.at<double>(2, 0);
            if (z_rgb <= 0.0) {
                continue;
            }

            points_rgb_frame.emplace_back(static_cast<float>(point_rgb.at<double>(0, 0)),
                                          static_cast<float>(point_rgb.at<double>(1, 0)),
                                          static_cast<float>(z_rgb));
            points_depth_frame_m.emplace_back(
                xyz_mm[0] / 1000.0F, xyz_mm[1] / 1000.0F, xyz_mm[2] / 1000.0F);
        }
    }

    if (points_rgb_frame.empty()) {
        return points;
    }

    std::vector<cv::Point2f> projected;
    cv::projectPoints(points_rgb_frame,
                      cv::Vec3d(0.0, 0.0, 0.0),
                      cv::Vec3d(0.0, 0.0, 0.0),
                      K,
                      D,
                      projected);

    points.reserve(projected.size());
    for (std::size_t i = 0; i < projected.size(); ++i) {
        const int u = static_cast<int>(std::lround(projected[i].x));
        const int v = static_cast<int>(std::lround(projected[i].y));
        if (u < 0 || u >= pair.rgb.bgr.cols || v < 0 || v >= pair.rgb.bgr.rows) {
            continue;
        }

        points.push_back(ColoredPoint{
            points_depth_frame_m[i],
            pair.rgb.bgr.at<cv::Vec3b>(v, u),
        });
    }

    return points;
}

sensor_msgs::msg::PointCloud2 make_point_cloud_msg(const std::vector<ColoredPoint>& points,
                                                   const std::string& frame_id,
                                                   const rclcpp::Time& stamp) {
    sensor_msgs::msg::PointCloud2 msg;
    msg.header.frame_id = frame_id;
    msg.header.stamp = stamp;
    msg.height = 1;
    msg.width = static_cast<std::uint32_t>(points.size());
    msg.is_dense = false;
    msg.is_bigendian = false;

    sensor_msgs::PointCloud2Modifier modifier(msg);
    modifier.setPointCloud2FieldsByString(2, "xyz", "rgb");
    modifier.resize(points.size());

    sensor_msgs::PointCloud2Iterator<float> iter_x(msg, "x");
    sensor_msgs::PointCloud2Iterator<float> iter_y(msg, "y");
    sensor_msgs::PointCloud2Iterator<float> iter_z(msg, "z");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> iter_r(msg, "r");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> iter_g(msg, "g");
    sensor_msgs::PointCloud2Iterator<std::uint8_t> iter_b(msg, "b");

    for (const auto& point : points) {
        *iter_x = point.xyz_m[0];
        *iter_y = point.xyz_m[1];
        *iter_z = point.xyz_m[2];
        *iter_r = point.rgb[2];
        *iter_g = point.rgb[1];
        *iter_b = point.rgb[0];
        ++iter_x;
        ++iter_y;
        ++iter_z;
        ++iter_r;
        ++iter_g;
        ++iter_b;
    }

    return msg;
}

}  // namespace

class ColoredCloudPublisher final : public rclcpp::Node {
public:
    ColoredCloudPublisher()
        : rclcpp::Node("colored_cloud_publisher"),
          stereo_yaml_(declare_parameter<std::string>(
              "stereo_yaml", "../calibration/results/combined_all_datasets/stereo_calibration.yaml")),
          frame_id_(declare_parameter<std::string>("frame_id", "map")),
          topic_(declare_parameter<std::string>("topic", "/helios/colored_points")),
          helios_pixel_format_(declare_parameter<std::string>("helios_pixel_format", "Coord3D_ABCY16")),
          phoenix_index_(static_cast<std::size_t>(declare_parameter<int>("phoenix_index", 0))),
          helios_index_(static_cast<std::size_t>(declare_parameter<int>("helios_index", 0))),
          timeout_ms_(static_cast<std::uint32_t>(declare_parameter<int>("timeout_ms", 1000))),
          max_delta_ns_(static_cast<std::uint64_t>(declare_parameter<std::int64_t>(
              "max_delta_ns", static_cast<std::int64_t>(150000000)))),
          publish_period_ms_(declare_parameter<int>("publish_period_ms", 300)) {
        calibration_ = load_stereo_calibration(stereo_yaml_);

        auto qos = rclcpp::QoS(rclcpp::KeepLast(1));
        qos.reliable();
        publisher_ = create_publisher<sensor_msgs::msg::PointCloud2>(topic_, qos);
        phoenix_ = std::make_unique<calibration::Phoenix>(phoenix_index_, "BayerRG8", 2, "Average");
        helios_ = std::make_unique<calibration::Helios2>(helios_index_, helios_pixel_format_);
        source_ = std::make_unique<calibration::LiveFrameSource>(
            *phoenix_, *helios_, max_delta_ns_, timeout_ms_);
        source_->open();

        timer_ = create_wall_timer(
            std::chrono::milliseconds(publish_period_ms_),
            std::bind(&ColoredCloudPublisher::publish_once, this));

        RCLCPP_INFO(get_logger(),
                    "Publishing colored cloud on %s using stereo calibration %s",
                    topic_.c_str(),
                    stereo_yaml_.c_str());
    }

    ~ColoredCloudPublisher() override {
        if (source_ != nullptr) {
            source_->close();
        }
    }

private:
    void publish_once() {
        try {
            calibration::FramePair pair;
            if (!source_->next(pair)) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000, "No synchronized RGB/depth frame available yet");
                return;
            }

            const auto points = colorize_point_cloud(pair, calibration_);
            if (points.empty()) {
                RCLCPP_WARN_THROTTLE(
                    get_logger(), *get_clock(), 2000, "No valid colored points produced for this frame");
                return;
            }

            auto msg = make_point_cloud_msg(points, frame_id_, now());
            publisher_->publish(std::move(msg));

            RCLCPP_INFO_THROTTLE(get_logger(),
                                 *get_clock(),
                                 3000,
                                 "Published %zu colored points (rgb ts=%llu depth ts=%llu)",
                                 points.size(),
                                 static_cast<unsigned long long>(pair.rgb.timestamp_ns),
                                 static_cast<unsigned long long>(pair.depth.timestamp_ns));
        } catch (const std::exception& ex) {
            RCLCPP_ERROR_THROTTLE(
                get_logger(), *get_clock(), 2000, "Publish step failed: %s", ex.what());
        }
    }

    std::string stereo_yaml_;
    std::string frame_id_;
    std::string topic_;
    std::string helios_pixel_format_;
    std::size_t phoenix_index_{0};
    std::size_t helios_index_{0};
    std::uint32_t timeout_ms_{1000};
    std::uint64_t max_delta_ns_{150000000};
    int publish_period_ms_{300};
    StereoCalibrationData calibration_;
    rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<calibration::Phoenix> phoenix_;
    std::unique_ptr<calibration::Helios2> helios_;
    std::unique_ptr<calibration::LiveFrameSource> source_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<ColoredCloudPublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& ex) {
        std::fprintf(stderr, "colored_cloud_publisher failed: %s\n", ex.what());
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
