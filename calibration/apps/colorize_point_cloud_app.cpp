#include <exception>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "calibration/io/hardware/helios2.hpp"
#include "calibration/io/hardware/phoenix.hpp"
#include "calibration/io/live_frame_source.hpp"

namespace {

struct CliArgs {
    std::filesystem::path stereo_yaml{
        "calibration/results/combined_all_datasets/stereo_calibration.yaml"};
    std::filesystem::path output_ply{
        "calibration/results/combined_all_datasets/helios_colored_point_cloud.ply"};
    std::size_t phoenix_index{0};
    std::size_t helios_index{0};
    std::uint32_t timeout_ms{1000};
    std::uint64_t max_delta_ns{20'000'000};
    std::string helios_pixel_format{"Coord3D_ABCY16"};
    bool preview{false};
};

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

CliArgs parse_args(int argc, char** argv) {
    CliArgs args;

    for (int i = 1; i < argc; ++i) {
        const std::string token(argv[i]);
        const auto require_value = [&](const std::string& name) -> std::string {
            if (i + 1 >= argc) {
                throw std::invalid_argument("Missing value for argument: " + name);
            }
            return std::string(argv[++i]);
        };

        if (token == "--stereo-yaml") {
            args.stereo_yaml = require_value(token);
        } else if (token == "--output") {
            args.output_ply = require_value(token);
        } else if (token == "--phoenix-index") {
            args.phoenix_index = static_cast<std::size_t>(std::stoul(require_value(token)));
        } else if (token == "--helios-index") {
            args.helios_index = static_cast<std::size_t>(std::stoul(require_value(token)));
        } else if (token == "--timeout-ms") {
            args.timeout_ms = static_cast<std::uint32_t>(std::stoul(require_value(token)));
        } else if (token == "--max-delta-ns") {
            args.max_delta_ns = static_cast<std::uint64_t>(std::stoull(require_value(token)));
        } else if (token == "--helios-format") {
            args.helios_pixel_format = require_value(token);
        } else if (token == "--preview") {
            args.preview = true;
        } else if (token == "--help" || token == "-h") {
            std::cout
                << "Usage: colorize_point_cloud_app [options]\n"
                << "  --stereo-yaml PATH\n"
                << "  --output PATH\n"
                << "  --phoenix-index N\n"
                << "  --helios-index N\n"
                << "  --timeout-ms N\n"
                << "  --max-delta-ns N\n"
                << "  --helios-format Coord3D_ABCY16|Coord3D_ABCY16s\n"
                << "  --preview\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("Unknown argument: " + token);
        }
    }

    return args;
}

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
                                               const StereoCalibrationData& calibration,
                                               cv::Mat* preview_out) {
    std::vector<ColoredPoint> points;
    if (pair.depth.xyz.empty() || pair.rgb.bgr.empty()) {
        return points;
    }

    cv::Mat preview;
    if (preview_out != nullptr) {
        preview = pair.rgb.bgr.clone();
    }

    const cv::Mat R = calibration.R;
    const cv::Mat T = calibration.T;
    const cv::Mat K = calibration.rgb_camera_matrix;
    const cv::Mat D = calibration.rgb_dist_coeffs;

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

            const cv::Mat point_depth =
                (cv::Mat_<double>(3, 1) << xyz_mm[0] / 1000.0, xyz_mm[1] / 1000.0, xyz_mm[2] / 1000.0);
            const cv::Mat point_rgb = R.t() * (point_depth - T);
            const double z_rgb = point_rgb.at<double>(2, 0);
            if (z_rgb <= 0.0) {
                continue;
            }

            points_rgb_frame.emplace_back(static_cast<float>(point_rgb.at<double>(0, 0)),
                                          static_cast<float>(point_rgb.at<double>(1, 0)),
                                          static_cast<float>(z_rgb));
            points_depth_frame_m.emplace_back(xyz_mm[0] / 1000.0F, xyz_mm[1] / 1000.0F, xyz_mm[2] / 1000.0F);
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

    for (std::size_t i = 0; i < projected.size(); ++i) {
        const int u = static_cast<int>(std::lround(projected[i].x));
        const int v = static_cast<int>(std::lround(projected[i].y));
        if (u < 0 || u >= pair.rgb.bgr.cols || v < 0 || v >= pair.rgb.bgr.rows) {
            continue;
        }

        ColoredPoint point;
        point.xyz_m = points_depth_frame_m[i];
        point.rgb = pair.rgb.bgr.at<cv::Vec3b>(v, u);
        points.push_back(point);

        if (preview_out != nullptr && (i % 200) == 0) {
            cv::circle(preview, cv::Point(u, v), 1, cv::Scalar(0, 255, 0), -1);
        }
    }

    if (preview_out != nullptr) {
        *preview_out = std::move(preview);
    }
    return points;
}

void write_ply(const std::filesystem::path& path, const std::vector<ColoredPoint>& points) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream stream(path);
    if (!stream.is_open()) {
        throw std::runtime_error("Failed to open PLY output path: " + path.string());
    }

    stream << "ply\n";
    stream << "format ascii 1.0\n";
    stream << "element vertex " << points.size() << "\n";
    stream << "property float x\n";
    stream << "property float y\n";
    stream << "property float z\n";
    stream << "property uchar red\n";
    stream << "property uchar green\n";
    stream << "property uchar blue\n";
    stream << "end_header\n";

    for (const auto& point : points) {
        stream << point.xyz_m[0] << " " << point.xyz_m[1] << " " << point.xyz_m[2] << " "
               << static_cast<int>(point.rgb[2]) << " " << static_cast<int>(point.rgb[1]) << " "
               << static_cast<int>(point.rgb[0]) << "\n";
    }
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const CliArgs args = parse_args(argc, argv);
        const StereoCalibrationData calibration = load_stereo_calibration(args.stereo_yaml);

        calibration::Phoenix phoenix(args.phoenix_index, "BayerRG8", 2, "Average");
        calibration::Helios2 helios(args.helios_index, args.helios_pixel_format);
        calibration::LiveFrameSource source(phoenix, helios, args.max_delta_ns, args.timeout_ms);

        source.open();
        calibration::FramePair pair;
        if (!source.next(pair)) {
            source.close();
            throw std::runtime_error("Failed to capture a paired RGB/Helios frame");
        }
        source.close();

        cv::Mat preview;
        const auto points = colorize_point_cloud(pair, calibration, args.preview ? &preview : nullptr);
        if (points.empty()) {
            throw std::runtime_error("No valid colored points were produced");
        }

        write_ply(args.output_ply, points);
        std::cout << "Saved colored point cloud: " << args.output_ply << "\n";
        std::cout << "Colored points: " << points.size() << "\n";
        std::cout << "RGB timestamp: " << pair.rgb.timestamp_ns
                  << " | Depth timestamp: " << pair.depth.timestamp_ns << "\n";

        if (args.preview) {
            cv::imshow("RGB overlay", preview);
            cv::waitKey(0);
            cv::destroyAllWindows();
        }

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "colorize_point_cloud_app failed: " << ex.what() << "\n";
        return 1;
    }
}
