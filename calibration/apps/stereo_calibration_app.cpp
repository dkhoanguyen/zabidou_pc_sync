#include <algorithm>
#include <cmath>
#include <filesystem>
#include <iostream>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "calibration/io/common/lucid_device_utils.hpp"

namespace {

struct CliArgs {
    std::filesystem::path session_dir;
    bool combine_all_datasets{false};
    std::filesystem::path data_root{"calibration/data"};
    std::filesystem::path mono_rgb_path;
    std::filesystem::path output_path;
    bool use_fast_check{false};
};

struct DatasetMetadata {
    std::string session_name;
    std::filesystem::path session_dir;
    int checkerboard_rows{0};
    int checkerboard_cols{0};
    float square_size_m{0.0F};
    std::string rgb_camera_label;
    std::string depth_camera_label;
    std::vector<std::string> rgb_filenames;
    std::vector<std::string> intensity_filenames;
};

struct MonoIntrinsics {
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Size image_size;
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

        if (token == "--session-dir") {
            args.session_dir = require_value(token);
        } else if (token == "--combine-all-datasets") {
            args.combine_all_datasets = true;
        } else if (token == "--data-root") {
            args.data_root = require_value(token);
        } else if (token == "--mono-rgb") {
            args.mono_rgb_path = require_value(token);
        } else if (token == "--out") {
            args.output_path = require_value(token);
        } else if (token == "--fast-check") {
            args.use_fast_check = true;
        } else if (token == "--help" || token == "-h") {
            std::cout
                << "Usage: stereo_calibration_app [--session-dir PATH | --combine-all-datasets] [options]\n"
                << "  --combine-all-datasets\n"
                << "  --data-root PATH\n"
                << "  --mono-rgb PATH\n"
                << "  --out PATH\n"
                << "  --fast-check\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("Unknown argument: " + token);
        }
    }

    const bool has_session_dir = !args.session_dir.empty();
    if (has_session_dir && args.combine_all_datasets) {
        throw std::invalid_argument(
            "Use either --session-dir PATH or --combine-all-datasets, not both");
    }
    if (!has_session_dir && !args.combine_all_datasets) {
        args.combine_all_datasets = true;
    }

    return args;
}

DatasetMetadata load_metadata(const std::filesystem::path& session_dir) {
    const auto metadata_path = session_dir / "metadata.yaml";
    cv::FileStorage fs(metadata_path.string(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Failed to open metadata file: " + metadata_path.string());
    }

    DatasetMetadata metadata;
    fs["session_name"] >> metadata.session_name;
    metadata.session_dir = session_dir;
    fs["checkerboard"]["rows"] >> metadata.checkerboard_rows;
    fs["checkerboard"]["cols"] >> metadata.checkerboard_cols;
    fs["checkerboard"]["square_size_m"] >> metadata.square_size_m;
    fs["rgb_camera"]["label"] >> metadata.rgb_camera_label;
    fs["depth_camera"]["label"] >> metadata.depth_camera_label;

    const cv::FileNode saved_frames = fs["saved_frames"];
    for (const auto& node : saved_frames) {
        std::string rgb_filename;
        std::string intensity_filename;
        node["rgb_filename"] >> rgb_filename;
        node["intensity_filename"] >> intensity_filename;
        metadata.rgb_filenames.push_back(std::move(rgb_filename));
        metadata.intensity_filenames.push_back(std::move(intensity_filename));
    }

    if (metadata.checkerboard_rows <= 0 || metadata.checkerboard_cols <= 0 ||
        metadata.square_size_m <= 0.0F) {
        throw std::runtime_error(
            "metadata.yaml checkerboard section must be populated before stereo calibration");
    }
    if (metadata.rgb_filenames.size() != metadata.intensity_filenames.size()) {
        throw std::runtime_error("Saved frame lists for RGB and intensity must be the same length");
    }

    return metadata;
}

std::vector<DatasetMetadata> load_all_metadata(const std::filesystem::path& data_root) {
    if (!std::filesystem::exists(data_root)) {
        throw std::runtime_error("Data root does not exist: " + data_root.string());
    }

    std::vector<DatasetMetadata> datasets;
    for (const auto& entry : std::filesystem::directory_iterator(data_root)) {
        if (!entry.is_directory()) {
            continue;
        }

        const auto metadata_path = entry.path() / "metadata.yaml";
        if (!std::filesystem::exists(metadata_path)) {
            continue;
        }

        datasets.push_back(load_metadata(entry.path()));
    }

    if (datasets.empty()) {
        throw std::runtime_error("No dataset sessions with metadata.yaml found under: " + data_root.string());
    }

    std::sort(datasets.begin(),
              datasets.end(),
              [](const DatasetMetadata& lhs, const DatasetMetadata& rhs) {
                  return lhs.session_name < rhs.session_name;
              });
    return datasets;
}

MonoIntrinsics load_mono_intrinsics(const std::filesystem::path& yaml_path) {
    cv::FileStorage fs(yaml_path.string(), cv::FileStorage::READ);
    if (!fs.isOpened()) {
        throw std::runtime_error("Failed to open mono calibration file: " + yaml_path.string());
    }

    MonoIntrinsics intrinsics;
    fs["camera_matrix"] >> intrinsics.camera_matrix;
    fs["dist_coeffs"] >> intrinsics.dist_coeffs;
    fs["image_width"] >> intrinsics.image_size.width;
    fs["image_height"] >> intrinsics.image_size.height;

    if (intrinsics.camera_matrix.empty() || intrinsics.dist_coeffs.empty() ||
        intrinsics.image_size.width <= 0 || intrinsics.image_size.height <= 0) {
        throw std::runtime_error("Mono calibration file is missing required fields: " + yaml_path.string());
    }

    return intrinsics;
}

std::vector<cv::Point3f> make_board_object_points(const cv::Size& board_size,
                                                  float square_size_m) {
    std::vector<cv::Point3f> points;
    points.reserve(static_cast<std::size_t>(board_size.width * board_size.height));

    for (int row = 0; row < board_size.height; ++row) {
        for (int col = 0; col < board_size.width; ++col) {
            points.emplace_back(static_cast<float>(col) * square_size_m,
                                static_cast<float>(row) * square_size_m,
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

    const cv::Mat mono = to_mono8(image);
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

void save_stereo_calibration(const std::filesystem::path& output_path,
                             const DatasetMetadata& reference_dataset,
                             const std::vector<std::string>& session_names,
                             const MonoIntrinsics& rgb_intrinsics,
                             const MonoIntrinsics& depth_intrinsics,
                             const cv::Mat& R,
                             const cv::Mat& T,
                             const cv::Mat& E,
                             const cv::Mat& F,
                             double rms_error,
                             int used_pairs,
                             int total_pairs) {
    cv::FileStorage fs(output_path.string(), cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        throw std::runtime_error("Failed to open stereo calibration output path: " + output_path.string());
    }

    fs << "backend" << "opencv_stereo";
    fs << "session_name" << reference_dataset.session_name;
    fs << "session_dir" << reference_dataset.session_dir.string();
    fs << "combined_session_count" << static_cast<int>(session_names.size());
    fs << "combined_sessions" << "[";
    for (const auto& session_name : session_names) {
        fs << session_name;
    }
    fs << "]";
    fs << "rgb_camera_label" << reference_dataset.rgb_camera_label;
    fs << "depth_camera_label" << reference_dataset.depth_camera_label;
    fs << "checkerboard_rows" << reference_dataset.checkerboard_rows;
    fs << "checkerboard_cols" << reference_dataset.checkerboard_cols;
    fs << "square_size_m" << reference_dataset.square_size_m;
    fs << "used_pairs" << used_pairs;
    fs << "total_pairs" << total_pairs;
    fs << "rms_error" << rms_error;
    fs << "rgb_image_width" << rgb_intrinsics.image_size.width;
    fs << "rgb_image_height" << rgb_intrinsics.image_size.height;
    fs << "depth_image_width" << depth_intrinsics.image_size.width;
    fs << "depth_image_height" << depth_intrinsics.image_size.height;
    fs << "rgb_camera_matrix" << rgb_intrinsics.camera_matrix;
    fs << "rgb_dist_coeffs" << rgb_intrinsics.dist_coeffs;
    fs << "depth_camera_matrix" << depth_intrinsics.camera_matrix;
    fs << "depth_dist_coeffs" << depth_intrinsics.dist_coeffs;
    fs << "R" << R;
    fs << "T" << T;
    fs << "E" << E;
    fs << "F" << F;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const CliArgs args = parse_args(argc, argv);
        std::vector<DatasetMetadata> datasets = args.combine_all_datasets
                                                    ? load_all_metadata(args.data_root)
                                                    : std::vector<DatasetMetadata>{load_metadata(args.session_dir)};
        const DatasetMetadata reference_dataset = datasets.front();
        const std::filesystem::path result_root =
            std::filesystem::path("calibration/results") /
            (args.combine_all_datasets ? "combined_all_datasets" : reference_dataset.session_name);
        const std::filesystem::path mono_rgb_path =
            args.mono_rgb_path.empty() ? (result_root / "mono_rgb_calibration.yaml") : args.mono_rgb_path;
        const std::filesystem::path output_path =
            args.output_path.empty() ? (result_root / "stereo_calibration.yaml") : args.output_path;

        std::filesystem::create_directories(output_path.parent_path());

        const MonoIntrinsics rgb_intrinsics = load_mono_intrinsics(mono_rgb_path);
        const auto helios_intrinsics =
            calibration::common::read_helios_factory_intrinsics(0);
        MonoIntrinsics depth_intrinsics;
        depth_intrinsics.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        depth_intrinsics.camera_matrix.at<double>(0, 0) = helios_intrinsics.focal_length_x;
        depth_intrinsics.camera_matrix.at<double>(1, 1) = helios_intrinsics.focal_length_y;
        depth_intrinsics.camera_matrix.at<double>(0, 2) = helios_intrinsics.optical_center_x;
        depth_intrinsics.camera_matrix.at<double>(1, 2) = helios_intrinsics.optical_center_y;
        depth_intrinsics.dist_coeffs =
            cv::Mat::zeros(1, static_cast<int>(helios_intrinsics.lens_distortion_values.size()), CV_64F);
        for (int i = 0; i < depth_intrinsics.dist_coeffs.cols; ++i) {
            depth_intrinsics.dist_coeffs.at<double>(0, i) =
                helios_intrinsics.lens_distortion_values[static_cast<std::size_t>(i)];
        }
        const cv::Size board_size(reference_dataset.checkerboard_cols, reference_dataset.checkerboard_rows);
        const auto object_template =
            make_board_object_points(board_size, reference_dataset.square_size_m);

        std::vector<std::vector<cv::Point3f>> object_points;
        std::vector<std::vector<cv::Point2f>> rgb_image_points;
        std::vector<std::vector<cv::Point2f>> depth_image_points;
        std::vector<std::string> session_names;
        int total_pairs = 0;

        for (const auto& dataset : datasets) {
            if (dataset.checkerboard_rows != reference_dataset.checkerboard_rows ||
                dataset.checkerboard_cols != reference_dataset.checkerboard_cols ||
                std::abs(dataset.square_size_m - reference_dataset.square_size_m) > 1e-6F) {
                throw std::runtime_error(
                    "All combined datasets must share the same checkerboard definition");
            }

            session_names.push_back(dataset.session_name);
            total_pairs += static_cast<int>(dataset.rgb_filenames.size());

            for (std::size_t i = 0; i < dataset.rgb_filenames.size(); ++i) {
                const auto rgb_path = dataset.session_dir / dataset.rgb_filenames[i];
                const auto depth_path = dataset.session_dir / dataset.intensity_filenames[i];
                const cv::Mat rgb_image = cv::imread(rgb_path.string(), cv::IMREAD_COLOR);
                const cv::Mat depth_image = cv::imread(depth_path.string(), cv::IMREAD_UNCHANGED);
                if (rgb_image.empty() || depth_image.empty()) {
                    std::cerr << "Skipping unreadable pair: " << rgb_path << " | " << depth_path << "\n";
                    continue;
                }

                if (rgb_image.size() != rgb_intrinsics.image_size) {
                    throw std::runtime_error(
                        "RGB image size does not match mono RGB calibration image size");
                }
                if (depth_intrinsics.image_size.empty()) {
                    depth_intrinsics.image_size = depth_image.size();
                } else if (depth_image.size() != depth_intrinsics.image_size) {
                    throw std::runtime_error(
                        "All depth images must share the same size across the combined dataset");
                }

                std::vector<cv::Point2f> rgb_corners;
                std::vector<cv::Point2f> depth_corners;
                const bool rgb_found =
                    detect_chessboard(rgb_image, board_size, args.use_fast_check, rgb_corners);
                const bool depth_found =
                    detect_chessboard(depth_image, board_size, args.use_fast_check, depth_corners);
                if (!(rgb_found && depth_found)) {
                    continue;
                }

                object_points.push_back(object_template);
                rgb_image_points.push_back(std::move(rgb_corners));
                depth_image_points.push_back(std::move(depth_corners));
            }
        }

        if (object_points.size() < 3) {
            throw std::runtime_error("Need at least 3 valid paired checkerboard detections for stereo calibration");
        }

        cv::Mat rgb_camera_matrix = rgb_intrinsics.camera_matrix.clone();
        cv::Mat rgb_dist_coeffs = rgb_intrinsics.dist_coeffs.clone();
        cv::Mat depth_camera_matrix = depth_intrinsics.camera_matrix.clone();
        cv::Mat depth_dist_coeffs = depth_intrinsics.dist_coeffs.clone();
        cv::Mat R;
        cv::Mat T;
        cv::Mat E;
        cv::Mat F;

        const double rms_error = cv::stereoCalibrate(object_points,
                                                     rgb_image_points,
                                                     depth_image_points,
                                                     rgb_camera_matrix,
                                                     rgb_dist_coeffs,
                                                     depth_camera_matrix,
                                                     depth_dist_coeffs,
                                                     rgb_intrinsics.image_size,
                                                     R,
                                                     T,
                                                     E,
                                                     F,
                                                     cv::CALIB_FIX_INTRINSIC,
                                                     cv::TermCriteria(cv::TermCriteria::COUNT +
                                                                          cv::TermCriteria::EPS,
                                                                      100,
                                                                      1e-5));

        save_stereo_calibration(output_path,
                                reference_dataset,
                                session_names,
                                rgb_intrinsics,
                                depth_intrinsics,
                                R,
                                T,
                                E,
                                F,
                                rms_error,
                                static_cast<int>(object_points.size()),
                                total_pairs);

        std::cout << "Stereo calibration completed\n";
        std::cout << "Datasets: " << session_names.size() << "\n";
        std::cout << "Used pairs: " << object_points.size() << " / " << total_pairs << "\n";
        std::cout << "RMS error: " << rms_error << "\n";
        std::cout << "Saved result to: " << output_path << "\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "stereo_calibration_app failed: " << ex.what() << "\n";
        return 1;
    }
}
