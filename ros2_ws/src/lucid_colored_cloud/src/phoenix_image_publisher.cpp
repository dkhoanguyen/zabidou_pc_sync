#include <algorithm>
#include <array>
#include <chrono>
#include <cstdint>
#include <exception>
#include <iostream>
#include <memory>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/core/persistence.hpp>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/camera_info.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "calibration/io/hardware/phoenix.hpp"
#include "calibration/types.hpp"

namespace {

struct CameraCalibration {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    int width{0};
    int height{0};
    bool valid{false};
};

CameraCalibration load_camera_calibration(const std::string& path) {
    CameraCalibration calibration;
    if (path.empty()) {
        return calibration;
    }

    cv::FileStorage fs(path, cv::FileStorage::READ);
    if (!fs.isOpened()) {
        return calibration;
    }

    fs["camera_matrix"] >> calibration.camera_matrix;
    fs["dist_coeffs"] >> calibration.dist_coeffs;
    calibration.width = static_cast<int>(fs["image_width"]);
    calibration.height = static_cast<int>(fs["image_height"]);

    if (calibration.camera_matrix.empty()) {
        fs["rgb_camera_matrix"] >> calibration.camera_matrix;
        fs["rgb_dist_coeffs"] >> calibration.dist_coeffs;
        calibration.width = static_cast<int>(fs["rgb_image_width"]);
        calibration.height = static_cast<int>(fs["rgb_image_height"]);
    }

    calibration.valid = !calibration.camera_matrix.empty();
    if (calibration.dist_coeffs.empty()) {
        calibration.dist_coeffs = cv::Mat::zeros(1, 5, CV_64F);
    }
    return calibration;
}

std::array<double, 9> scaled_k(const CameraCalibration& calibration,
                               int image_width,
                               int image_height) {
    std::array<double, 9> k{1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    if (!calibration.valid) {
        return k;
    }

    cv::Mat camera_matrix;
    calibration.camera_matrix.convertTo(camera_matrix, CV_64F);
    const double scale_x = calibration.width > 0 ? static_cast<double>(image_width) / calibration.width : 1.0;
    const double scale_y = calibration.height > 0 ? static_cast<double>(image_height) / calibration.height : 1.0;

    k = {
        camera_matrix.at<double>(0, 0) * scale_x,
        camera_matrix.at<double>(0, 1),
        camera_matrix.at<double>(0, 2) * scale_x,
        camera_matrix.at<double>(1, 0),
        camera_matrix.at<double>(1, 1) * scale_y,
        camera_matrix.at<double>(1, 2) * scale_y,
        camera_matrix.at<double>(2, 0),
        camera_matrix.at<double>(2, 1),
        camera_matrix.at<double>(2, 2),
    };
    return k;
}

std::vector<double> distortion(const CameraCalibration& calibration) {
    if (!calibration.valid || calibration.dist_coeffs.empty()) {
        return std::vector<double>{0.0, 0.0, 0.0, 0.0, 0.0};
    }

    cv::Mat dist;
    calibration.dist_coeffs.reshape(1, 1).convertTo(dist, CV_64F);
    std::vector<double> values;
    values.reserve(static_cast<std::size_t>(dist.cols));
    for (int col = 0; col < dist.cols; ++col) {
        values.push_back(dist.at<double>(0, col));
    }
    return values;
}

sensor_msgs::msg::Image make_image_msg(const calibration::RgbFrame& frame,
                                       const rclcpp::Time& stamp,
                                       const std::string& frame_id) {
    sensor_msgs::msg::Image msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.height = static_cast<std::uint32_t>(frame.bgr.rows);
    msg.width = static_cast<std::uint32_t>(frame.bgr.cols);
    msg.encoding = "bgr8";
    msg.is_bigendian = false;
    msg.step = static_cast<std::uint32_t>(frame.bgr.cols * frame.bgr.elemSize());

    const std::size_t byte_count = static_cast<std::size_t>(msg.step) * msg.height;
    msg.data.resize(byte_count);
    if (frame.bgr.isContinuous()) {
        std::copy(frame.bgr.datastart, frame.bgr.dataend, msg.data.begin());
    } else {
        std::uint8_t* dst = msg.data.data();
        for (int row = 0; row < frame.bgr.rows; ++row) {
            const std::uint8_t* src = frame.bgr.ptr<std::uint8_t>(row);
            std::copy(src, src + msg.step, dst);
            dst += msg.step;
        }
    }
    return msg;
}

sensor_msgs::msg::CameraInfo make_camera_info_msg(const CameraCalibration& calibration,
                                                  int width,
                                                  int height,
                                                  const rclcpp::Time& stamp,
                                                  const std::string& frame_id) {
    sensor_msgs::msg::CameraInfo msg;
    msg.header.stamp = stamp;
    msg.header.frame_id = frame_id;
    msg.width = static_cast<std::uint32_t>(width);
    msg.height = static_cast<std::uint32_t>(height);
    msg.distortion_model = "plumb_bob";
    msg.d = distortion(calibration);
    msg.k = scaled_k(calibration, width, height);
    msg.r = {1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0};
    msg.p = {
        msg.k[0], msg.k[1], msg.k[2], 0.0,
        msg.k[3], msg.k[4], msg.k[5], 0.0,
        msg.k[6], msg.k[7], msg.k[8], 0.0,
    };
    return msg;
}

}  // namespace

class PhoenixImagePublisher final : public rclcpp::Node {
public:
    PhoenixImagePublisher()
        : rclcpp::Node("phoenix_image_publisher") {
        phoenix_index_ = static_cast<std::size_t>(declare_parameter<int>("phoenix_index", 0));
        pixel_format_ = declare_parameter<std::string>("pixel_format", "BayerRG8");
        binning_ = declare_parameter<int>("binning", 2);
        binning_mode_ = declare_parameter<std::string>("binning_mode", "Average");
        fps_ = declare_parameter<double>("fps", 30.0);
        timeout_ms_ = static_cast<std::uint32_t>(declare_parameter<int>("timeout_ms", 1000));
        topic_ = declare_parameter<std::string>("image_topic", "/phoenix/image_raw");
        camera_info_topic_ = declare_parameter<std::string>("camera_info_topic", "/phoenix/camera_info");
        frame_id_ = declare_parameter<std::string>("frame_id", "phoenix_camera");
        calibration_yaml_ = declare_parameter<std::string>(
            "calibration_yaml",
            "/home/khoa/Projects/zabidou_pc_sync/calibration/results/combined_all_datasets/mono_rgb_calibration.yaml");

        calibration_ = load_camera_calibration(calibration_yaml_);
        if (calibration_.valid) {
            RCLCPP_INFO(get_logger(), "Loaded camera calibration from %s", calibration_yaml_.c_str());
        } else {
            RCLCPP_WARN(get_logger(), "No camera calibration loaded; publishing identity CameraInfo");
        }

        auto qos = rclcpp::SensorDataQoS();
        image_pub_ = create_publisher<sensor_msgs::msg::Image>(topic_, qos);
        camera_info_pub_ = create_publisher<sensor_msgs::msg::CameraInfo>(camera_info_topic_, qos);

        camera_ = std::make_unique<calibration::Phoenix>(
            phoenix_index_, pixel_format_, binning_, binning_mode_, fps_, 8);
        camera_->open();
        RCLCPP_INFO(get_logger(), "Opened %s", camera_->model_name().c_str());
        RCLCPP_INFO(get_logger(), "Publishing %s and %s", topic_.c_str(), camera_info_topic_.c_str());

        const double publish_fps = fps_ > 0.0 ? fps_ : 30.0;
        const auto period = std::chrono::duration<double>(1.0 / publish_fps);
        timer_ = create_wall_timer(std::chrono::duration_cast<std::chrono::nanoseconds>(period),
                                   [this]() { publish_frame(); });
    }

    ~PhoenixImagePublisher() override {
        if (camera_) {
            camera_->close();
        }
    }

private:
    void publish_frame() {
        calibration::RgbFrame frame;
        if (!camera_->grab(frame, timeout_ms_)) {
            RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 2000, "Timed out waiting for Phoenix frame");
            return;
        }

        const rclcpp::Time stamp = now();
        auto image_msg = make_image_msg(frame, stamp, frame_id_);
        auto camera_info_msg = make_camera_info_msg(calibration_, frame.bgr.cols, frame.bgr.rows, stamp, frame_id_);
        image_pub_->publish(std::move(image_msg));
        camera_info_pub_->publish(std::move(camera_info_msg));
    }

    std::size_t phoenix_index_{0};
    std::string pixel_format_;
    int binning_{2};
    std::string binning_mode_;
    double fps_{30.0};
    std::uint32_t timeout_ms_{1000};
    std::string topic_;
    std::string camera_info_topic_;
    std::string frame_id_;
    std::string calibration_yaml_;
    CameraCalibration calibration_;

    std::unique_ptr<calibration::Phoenix> camera_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_pub_;
    rclcpp::Publisher<sensor_msgs::msg::CameraInfo>::SharedPtr camera_info_pub_;
    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<PhoenixImagePublisher>();
        rclcpp::spin(node);
    } catch (const std::exception& ex) {
        std::cerr << "phoenix_image_publisher failed: " << ex.what() << "\n";
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
