#include <chrono>
#include <cstdint>
#include <exception>
#include <fstream>
#include <iostream>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <vector>

#include <Eigen/Geometry>
#include <opencv2/core.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/imgproc.hpp>

#include <geometry_msgs/msg/pose_stamped.hpp>
#include <nav_msgs/msg/path.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "MapPoint.h"
#include "System.h"

namespace {

double stamp_to_seconds(const builtin_interfaces::msg::Time& stamp) {
    return static_cast<double>(stamp.sec) + static_cast<double>(stamp.nanosec) * 1e-9;
}

cv::Mat image_to_cv_mat(const sensor_msgs::msg::Image& msg) {
    if (msg.encoding == "bgr8") {
        return cv::Mat(static_cast<int>(msg.height),
                       static_cast<int>(msg.width),
                       CV_8UC3,
                       const_cast<std::uint8_t*>(msg.data.data()),
                       msg.step);
    }

    if (msg.encoding == "rgb8") {
        cv::Mat rgb(static_cast<int>(msg.height),
                    static_cast<int>(msg.width),
                    CV_8UC3,
                    const_cast<std::uint8_t*>(msg.data.data()),
                    msg.step);
        cv::Mat bgr;
        cv::cvtColor(rgb, bgr, cv::COLOR_RGB2BGR);
        return bgr;
    }

    if (msg.encoding == "mono8" || msg.encoding == "8UC1") {
        return cv::Mat(static_cast<int>(msg.height),
                       static_cast<int>(msg.width),
                       CV_8UC1,
                       const_cast<std::uint8_t*>(msg.data.data()),
                       msg.step);
    }

    throw std::runtime_error("Unsupported image encoding: " + msg.encoding);
}

const char* tracking_state_name(int state) {
    switch (state) {
    case ORB_SLAM3::Tracking::SYSTEM_NOT_READY:
        return "SYSTEM_NOT_READY";
    case ORB_SLAM3::Tracking::NO_IMAGES_YET:
        return "NO_IMAGES_YET";
    case ORB_SLAM3::Tracking::NOT_INITIALIZED:
        return "NOT_INITIALIZED";
    case ORB_SLAM3::Tracking::OK:
        return "OK";
    case ORB_SLAM3::Tracking::RECENTLY_LOST:
        return "RECENTLY_LOST";
    case ORB_SLAM3::Tracking::LOST:
        return "LOST";
    case ORB_SLAM3::Tracking::OK_KLT:
        return "OK_KLT";
    default:
        return "UNKNOWN";
    }
}

geometry_msgs::msg::PoseStamped pose_from_tcw(const Sophus::SE3f& tcw,
                                              const rclcpp::Time& stamp,
                                              const std::string& frame_id) {
    const Eigen::Matrix4f twc = tcw.matrix().inverse();
    const Eigen::Matrix3f r_wc = twc.block<3, 3>(0, 0);
    const Eigen::Vector3f t_wc = twc.block<3, 1>(0, 3);
    Eigen::Quaternionf q_wc(r_wc);
    q_wc.normalize();

    geometry_msgs::msg::PoseStamped pose;
    pose.header.stamp = stamp;
    pose.header.frame_id = frame_id;
    pose.pose.position.x = t_wc.x();
    pose.pose.position.y = t_wc.y();
    pose.pose.position.z = t_wc.z();
    pose.pose.orientation.x = q_wc.x();
    pose.pose.orientation.y = q_wc.y();
    pose.pose.orientation.z = q_wc.z();
    pose.pose.orientation.w = q_wc.w();
    return pose;
}

}  // namespace

class PhoenixOrbSlam3Node final : public rclcpp::Node {
public:
    PhoenixOrbSlam3Node()
        : rclcpp::Node("phoenix_orbslam3_node") {
        vocabulary_path_ = declare_parameter<std::string>(
            "vocabulary_path",
            "/home/khoa/Projects/ORB_SLAM3/Vocabulary/ORBvoc.txt");
        settings_path_ = declare_parameter<std::string>(
            "settings_path",
            "/home/khoa/Projects/zabidou_pc_sync/ros2_ws/src/lucid_colored_cloud/config/orbslam3_phoenix_binning2.yaml");
        image_topic_ = declare_parameter<std::string>("image_topic", "/phoenix/image_raw");
        pose_topic_ = declare_parameter<std::string>("pose_topic", "/orbslam3/camera_pose");
        path_topic_ = declare_parameter<std::string>("path_topic", "/orbslam3/path");
        map_frame_id_ = declare_parameter<std::string>("map_frame_id", "map");
        trajectory_path_ = declare_parameter<std::string>(
            "trajectory_path",
            "/home/khoa/Projects/zabidou_pc_sync/KeyFrameTrajectory_live.txt");
        sparse_cloud_path_ = declare_parameter<std::string>(
            "sparse_cloud_path",
            "/home/khoa/Projects/zabidou_pc_sync/MapPoints_live.ply");
        use_viewer_ = declare_parameter<bool>("use_viewer", true);
        publish_path_ = declare_parameter<bool>("publish_path", true);
        save_sparse_cloud_ = declare_parameter<bool>("save_sparse_cloud", true);
        max_path_poses_ = declare_parameter<int>("max_path_poses", 5000);
        sparse_cloud_min_observations_ = declare_parameter<int>("sparse_cloud_min_observations", 2);
        diagnostics_enabled_ = declare_parameter<bool>("diagnostics_enabled", true);
        diagnostics_period_ms_ = declare_parameter<int>("diagnostics_period_ms", 2000);
        diagnostic_orb_features_ = declare_parameter<int>("diagnostic_orb_features", 2000);
        diagnostic_fast_threshold_ = declare_parameter<int>("diagnostic_fast_threshold", 5);

        RCLCPP_INFO(get_logger(), "Starting ORB-SLAM3 with vocabulary: %s", vocabulary_path_.c_str());
        RCLCPP_INFO(get_logger(), "Starting ORB-SLAM3 with settings:   %s", settings_path_.c_str());

        slam_ = std::make_unique<ORB_SLAM3::System>(
            vocabulary_path_,
            settings_path_,
            ORB_SLAM3::System::MONOCULAR,
            use_viewer_);

        pose_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>(pose_topic_, 10);
        path_pub_ = create_publisher<nav_msgs::msg::Path>(path_topic_, 10);
        path_msg_.header.frame_id = map_frame_id_;

        image_sub_ = create_subscription<sensor_msgs::msg::Image>(
            image_topic_,
            rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr msg) {
                handle_image(msg);
            });

        RCLCPP_INFO(get_logger(), "Subscribed to %s", image_topic_.c_str());
        RCLCPP_INFO(get_logger(), "Publishing pose on %s", pose_topic_.c_str());
    }

    ~PhoenixOrbSlam3Node() override {
        shutdown_slam();
    }

private:
    void handle_image(const sensor_msgs::msg::Image::ConstSharedPtr& msg) {
        if (!slam_) {
            return;
        }

        cv::Mat image;
        try {
            image = image_to_cv_mat(*msg);
        } catch (const std::exception& ex) {
            RCLCPP_ERROR_THROTTLE(get_logger(), *get_clock(), 2000, "%s", ex.what());
            return;
        }

        maybe_log_image_diagnostics(*msg, image);

        const double timestamp = stamp_to_seconds(msg->header.stamp);
        Sophus::SE3f tcw;
        {
            std::lock_guard<std::mutex> lock(slam_mutex_);
            tcw = slam_->TrackMonocular(image, timestamp);
        }

        const int tracking_state = slam_->GetTrackingState();
        if (tracking_state != ORB_SLAM3::Tracking::OK &&
            tracking_state != ORB_SLAM3::Tracking::OK_KLT) {
            RCLCPP_WARN_THROTTLE(
                get_logger(),
                *get_clock(),
                2000,
                "ORB-SLAM3 tracking state: %d (%s)",
                tracking_state,
                tracking_state_name(tracking_state));
            return;
        }

        remember_tracked_map_points();

        const rclcpp::Time stamp(msg->header.stamp);
        auto pose = pose_from_tcw(tcw, stamp, map_frame_id_);
        pose_pub_->publish(pose);

        if (publish_path_) {
            path_msg_.header.stamp = stamp;
            path_msg_.poses.push_back(pose);
            if (max_path_poses_ > 0 &&
                path_msg_.poses.size() > static_cast<std::size_t>(max_path_poses_)) {
                path_msg_.poses.erase(path_msg_.poses.begin());
            }
            path_pub_->publish(path_msg_);
        }
    }

    void remember_tracked_map_points() {
        const auto map_points = slam_->GetTrackedMapPoints();
        for (ORB_SLAM3::MapPoint* map_point : map_points) {
            if (map_point == nullptr || map_point->isBad()) {
                continue;
            }
            if (sparse_cloud_min_observations_ > 0 &&
                map_point->Observations() < sparse_cloud_min_observations_) {
                continue;
            }

            sparse_points_[map_point->mnId] = map_point->GetWorldPos();
        }
    }

    void save_sparse_cloud_ply() {
        if (!save_sparse_cloud_ || sparse_cloud_path_.empty()) {
            return;
        }

        std::ofstream out(sparse_cloud_path_);
        if (!out.is_open()) {
            RCLCPP_ERROR(get_logger(), "Failed to open sparse cloud PLY for writing: %s", sparse_cloud_path_.c_str());
            return;
        }

        out << "ply\n";
        out << "format ascii 1.0\n";
        out << "comment ORB-SLAM3 sparse map points accumulated from tracked frames\n";
        out << "element vertex " << sparse_points_.size() << "\n";
        out << "property float x\n";
        out << "property float y\n";
        out << "property float z\n";
        out << "property uchar red\n";
        out << "property uchar green\n";
        out << "property uchar blue\n";
        out << "end_header\n";

        for (const auto& [id, point] : sparse_points_) {
            (void)id;
            out << point.x() << ' ' << point.y() << ' ' << point.z() << " 255 255 255\n";
        }

        RCLCPP_INFO(
            get_logger(),
            "Saved %zu accumulated sparse map points to %s",
            sparse_points_.size(),
            sparse_cloud_path_.c_str());
    }

    void maybe_log_image_diagnostics(const sensor_msgs::msg::Image& msg,
                                     const cv::Mat& image) {
        if (!diagnostics_enabled_) {
            return;
        }

        const auto now_time = now();
        if (last_diagnostic_time_.nanoseconds() != 0) {
            const auto elapsed_ms = (now_time - last_diagnostic_time_).nanoseconds() / 1000000;
            if (elapsed_ms < diagnostics_period_ms_) {
                return;
            }
        }
        last_diagnostic_time_ = now_time;

        cv::Mat gray;
        if (image.channels() == 3) {
            cv::cvtColor(image, gray, cv::COLOR_BGR2GRAY);
        } else {
            gray = image;
        }

        cv::Scalar mean;
        cv::Scalar stddev;
        cv::meanStdDev(gray, mean, stddev);

        std::vector<cv::KeyPoint> keypoints;
        auto detector = cv::ORB::create(
            diagnostic_orb_features_,
            1.2f,
            8,
            31,
            0,
            2,
            cv::ORB::HARRIS_SCORE,
            31,
            diagnostic_fast_threshold_);
        detector->detect(gray, keypoints);

        RCLCPP_INFO(
            get_logger(),
            "Phoenix image diagnostics: %ux%u encoding=%s step=%u mean=%.1f stddev=%.1f orb_keypoints=%zu stamp=%.6f",
            msg.width,
            msg.height,
            msg.encoding.c_str(),
            msg.step,
            mean[0],
            stddev[0],
            keypoints.size(),
            stamp_to_seconds(msg.header.stamp));
    }

    void shutdown_slam() {
        std::lock_guard<std::mutex> lock(slam_mutex_);
        if (!slam_) {
            return;
        }

        RCLCPP_INFO(get_logger(), "Shutting down ORB-SLAM3");
        slam_->Shutdown();
        save_sparse_cloud_ply();
        if (!trajectory_path_.empty()) {
            slam_->SaveKeyFrameTrajectoryTUM(trajectory_path_);
            RCLCPP_INFO(get_logger(), "Saved keyframe trajectory to %s", trajectory_path_.c_str());
        }
        slam_.reset();
    }

    std::string vocabulary_path_;
    std::string settings_path_;
    std::string image_topic_;
    std::string pose_topic_;
    std::string path_topic_;
    std::string map_frame_id_;
    std::string trajectory_path_;
    std::string sparse_cloud_path_;
    bool use_viewer_{true};
    bool publish_path_{true};
    bool save_sparse_cloud_{true};
    int max_path_poses_{5000};
    int sparse_cloud_min_observations_{2};
    bool diagnostics_enabled_{true};
    int diagnostics_period_ms_{2000};
    int diagnostic_orb_features_{2000};
    int diagnostic_fast_threshold_{5};

    std::mutex slam_mutex_;
    std::unique_ptr<ORB_SLAM3::System> slam_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_sub_;
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr pose_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    nav_msgs::msg::Path path_msg_;
    rclcpp::Time last_diagnostic_time_{0, 0, RCL_ROS_TIME};
    std::unordered_map<unsigned long, Eigen::Vector3f> sparse_points_;
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    try {
        auto node = std::make_shared<PhoenixOrbSlam3Node>();
        rclcpp::spin(node);
    } catch (const std::exception& ex) {
        std::cerr << "phoenix_orbslam3_node failed: " << ex.what() << "\n";
        rclcpp::shutdown();
        return 1;
    }
    rclcpp::shutdown();
    return 0;
}
