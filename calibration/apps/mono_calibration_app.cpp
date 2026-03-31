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

namespace {

struct CliArgs {
    std::filesystem::path session_dir;
    bool combine_all_datasets{false};
    std::filesystem::path data_root{"calibration/data"};
    std::string modality{"rgb"};
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
        } else if (token == "--modality") {
            args.modality = require_value(token);
        } else if (token == "--out") {
            args.output_path = require_value(token);
        } else if (token == "--fast-check") {
            args.use_fast_check = true;
        } else if (token == "--help" || token == "-h") {
            std::cout
                << "Usage: mono_calibration_app [--session-dir PATH | --combine-all-datasets] [options]\n"
                << "  --modality rgb|depth\n"
                << "  --combine-all-datasets\n"
                << "  --data-root PATH\n"
                << "  --out PATH\n"
                << "  --fast-check\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("Unknown argument: " + token);
        }
    }

    const bool has_session_dir = !args.session_dir.empty();
    if (has_session_dir == args.combine_all_datasets) {
        throw std::invalid_argument(
            "Provide exactly one of --session-dir PATH or --combine-all-datasets");
    }
    if (args.modality != "rgb" && args.modality != "depth") {
        throw std::invalid_argument("--modality must be rgb or depth");
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
            "metadata.yaml checkerboard section must be populated before mono calibration");
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

cv::Mat load_image_for_modality(const std::filesystem::path& image_path, const std::string& modality) {
    if (modality == "rgb") {
        return cv::imread(image_path.string(), cv::IMREAD_COLOR);
    }
    return cv::imread(image_path.string(), cv::IMREAD_UNCHANGED);
}

void save_calibration(const std::filesystem::path& output_path,
                      const std::string& modality,
                      const DatasetMetadata& metadata,
                      const std::vector<std::string>& session_names,
                      const cv::Size& image_size,
                      const cv::Mat& camera_matrix,
                      const cv::Mat& dist_coeffs,
                      double rms_error,
                      int used_images,
                      int total_images) {
    cv::FileStorage fs(output_path.string(), cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        throw std::runtime_error("Failed to open mono calibration output path: " + output_path.string());
    }

    fs << "backend" << "opencv_mono";
    fs << "session_name" << metadata.session_name;
    fs << "session_dir" << metadata.session_dir.string();
    fs << "combined_session_count" << static_cast<int>(session_names.size());
    fs << "combined_sessions" << "[";
    for (const auto& session_name : session_names) {
        fs << session_name;
    }
    fs << "]";
    fs << "modality" << modality;
    fs << "camera_label" << (modality == "rgb" ? metadata.rgb_camera_label : metadata.depth_camera_label);
    fs << "checkerboard_rows" << metadata.checkerboard_rows;
    fs << "checkerboard_cols" << metadata.checkerboard_cols;
    fs << "square_size_m" << metadata.square_size_m;
    fs << "image_width" << image_size.width;
    fs << "image_height" << image_size.height;
    fs << "used_images" << used_images;
    fs << "total_images" << total_images;
    fs << "rms_error" << rms_error;
    fs << "camera_matrix" << camera_matrix;
    fs << "dist_coeffs" << dist_coeffs;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const CliArgs args = parse_args(argc, argv);
        std::vector<DatasetMetadata> datasets = args.combine_all_datasets
                                                    ? load_all_metadata(args.data_root)
                                                    : std::vector<DatasetMetadata>{load_metadata(args.session_dir)};
        const DatasetMetadata metadata = datasets.front();
        const std::filesystem::path output_path =
            args.output_path.empty()
                ? (std::filesystem::path("calibration/results") /
                   (args.combine_all_datasets ? "combined_all_datasets" : metadata.session_name) /
                   (args.modality == "rgb" ? "mono_rgb_calibration.yaml"
                                           : "mono_depth_calibration.yaml"))
                : args.output_path;
        std::filesystem::create_directories(output_path.parent_path());
        const cv::Size board_size(metadata.checkerboard_cols, metadata.checkerboard_rows);
        const auto object_template =
            make_board_object_points(board_size, metadata.square_size_m);

        std::vector<std::vector<cv::Point3f>> object_points;
        std::vector<std::vector<cv::Point2f>> image_points;
        cv::Size image_size;
        int used_images = 0;
        int total_images = 0;
        std::vector<std::string> session_names;

        for (const auto& dataset : datasets) {
            if (dataset.checkerboard_rows != metadata.checkerboard_rows ||
                dataset.checkerboard_cols != metadata.checkerboard_cols ||
                std::abs(dataset.square_size_m - metadata.square_size_m) > 1e-6F) {
                throw std::runtime_error(
                    "All combined datasets must share the same checkerboard definition");
            }

            session_names.push_back(dataset.session_name);
            const std::vector<std::string>& filenames =
                (args.modality == "rgb") ? dataset.rgb_filenames : dataset.intensity_filenames;
            total_images += static_cast<int>(filenames.size());

            for (const auto& filename : filenames) {
                const auto image_path = dataset.session_dir / filename;
                const cv::Mat image = load_image_for_modality(image_path, args.modality);
                if (image.empty()) {
                    std::cerr << "Skipping unreadable image: " << image_path << "\n";
                    continue;
                }

                std::vector<cv::Point2f> corners;
                if (!detect_chessboard(image, board_size, args.use_fast_check, corners)) {
                    continue;
                }

                if (image_size.empty()) {
                    image_size = image.size();
                } else if (image.size() != image_size) {
                    throw std::runtime_error(
                        "All combined datasets must share the same image size for the chosen modality");
                }

                image_points.push_back(std::move(corners));
                object_points.push_back(object_template);
                ++used_images;
            }
        }

        if (used_images < 3) {
            throw std::runtime_error("Need at least 3 valid checkerboard detections for mono calibration");
        }

        cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        cv::Mat dist_coeffs = cv::Mat::zeros(8, 1, CV_64F);
        std::vector<cv::Mat> rvecs;
        std::vector<cv::Mat> tvecs;

        const double rms_error = cv::calibrateCamera(object_points,
                                                     image_points,
                                                     image_size,
                                                     camera_matrix,
                                                     dist_coeffs,
                                                     rvecs,
                                                     tvecs);

        save_calibration(output_path,
                         args.modality,
                         metadata,
                         session_names,
                         image_size,
                         camera_matrix,
                         dist_coeffs,
                         rms_error,
                         used_images,
                         total_images);

        std::cout << "Mono calibration completed for modality: " << args.modality << "\n";
        std::cout << "Datasets: " << session_names.size() << "\n";
        std::cout << "Used images: " << used_images << " / " << total_images << "\n";
        std::cout << "RMS error: " << rms_error << "\n";
        std::cout << "Saved result to: " << output_path << "\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "mono_calibration_app failed: " << ex.what() << "\n";
        return 1;
    }
}
