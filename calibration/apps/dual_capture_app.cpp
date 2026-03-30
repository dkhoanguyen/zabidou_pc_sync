#include <exception>
#include <filesystem>
#include <chrono>
#include <iomanip>
#include <iostream>
#include <ctime>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>

#include "calibration/io/hardware/helios2.hpp"
#include "calibration/io/hardware/phoenix.hpp"
#include "calibration/io/live_frame_source.hpp"

namespace {

struct CliArgs {
    std::size_t phoenix_index{0};
    std::size_t helios_index{0};
    std::uint32_t timeout_ms{1000};
    std::uint64_t max_delta_ns{150'000'000};
    std::string helios_pixel_format{"Coord3D_ABCY16"};
    std::filesystem::path output_root{"calibration"};
};

struct SavedFrameRecord {
    std::uint64_t rgb_timestamp_ns{0};
    std::uint64_t depth_timestamp_ns{0};
    std::string rgb_filename;
    std::string intensity_filename;
    int rgb_width{0};
    int rgb_height{0};
    int intensity_width{0};
    int intensity_height{0};
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

        if (token == "--phoenix-index") {
            args.phoenix_index = static_cast<std::size_t>(std::stoul(require_value(token)));
        } else if (token == "--helios-index") {
            args.helios_index = static_cast<std::size_t>(std::stoul(require_value(token)));
        } else if (token == "--timeout-ms") {
            args.timeout_ms = static_cast<std::uint32_t>(std::stoul(require_value(token)));
        } else if (token == "--max-delta-ns") {
            args.max_delta_ns = static_cast<std::uint64_t>(std::stoull(require_value(token)));
        } else if (token == "--helios-format") {
            args.helios_pixel_format = require_value(token);
        } else if (token == "--output-dir") {
            args.output_root = require_value(token);
        } else if (token == "--help" || token == "-h") {
            std::cout
                << "Usage: dual_capture_app [options]\n"
                << "  --phoenix-index N\n"
                << "  --helios-index N\n"
                << "  --timeout-ms N\n"
                << "  --max-delta-ns N\n"
                << "  --helios-format Coord3D_ABCY16|Coord3D_ABCY16s\n"
                << "  --output-dir PATH (parent directory for a new data_<timestamp> session folder)\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("Unknown argument: " + token);
        }
    }

    return args;
}

cv::Mat make_intensity_visualization(const calibration::DepthFrame& depth_frame) {
    if (depth_frame.intensity.empty()) {
        return cv::Mat::zeros(480, 640, CV_8UC3);
    }

    cv::Mat intensity_source;
    depth_frame.intensity.convertTo(intensity_source, CV_32F);

    cv::Mat valid_mask = intensity_source > 0;
    double min_value = 0.0;
    double max_value = 0.0;
    cv::minMaxLoc(intensity_source, &min_value, &max_value, nullptr, nullptr, valid_mask);

    cv::Mat normalized(intensity_source.size(), CV_8UC1, cv::Scalar(0));
    if (max_value > min_value) {
        cv::Mat scaled;
        intensity_source.convertTo(
            scaled, CV_8UC1, 255.0 / (max_value - min_value), -min_value * 255.0 / (max_value - min_value));
        scaled.copyTo(normalized, valid_mask);
    }

    cv::Mat intensity_bgr;
    cv::cvtColor(normalized, intensity_bgr, cv::COLOR_GRAY2BGR);
    intensity_bgr.setTo(cv::Scalar(0, 0, 0), ~valid_mask);
    return intensity_bgr;
}

cv::Mat make_side_by_side_view(const calibration::FramePair& pair, cv::Mat& intensity_vis_out) {
    intensity_vis_out = make_intensity_visualization(pair.depth);
    if (intensity_vis_out.empty()) {
        throw std::runtime_error("Intensity visualization frame is empty");
    }

    cv::Mat rgb_view;
    if (pair.rgb.bgr.empty()) {
        rgb_view = cv::Mat::zeros(intensity_vis_out.rows, intensity_vis_out.cols, CV_8UC3);
    } else {
        rgb_view = pair.rgb.bgr;
    }

    const int output_height = std::max(rgb_view.rows, intensity_vis_out.rows);
    cv::Mat rgb_padded(output_height, rgb_view.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat intensity_padded(output_height, intensity_vis_out.cols, CV_8UC3, cv::Scalar(0, 0, 0));

    rgb_view.copyTo(rgb_padded(cv::Rect(0, 0, rgb_view.cols, rgb_view.rows)));
    intensity_vis_out.copyTo(
        intensity_padded(cv::Rect(0, 0, intensity_vis_out.cols, intensity_vis_out.rows)));

    cv::Mat combined;
    cv::hconcat(rgb_padded, intensity_padded, combined);
    return combined;
}

std::string make_frame_stem(const calibration::FramePair& pair) {
    std::ostringstream stream;
    stream << "pair_rgb_" << pair.rgb.timestamp_ns
           << "_depth_" << pair.depth.timestamp_ns;
    return stream.str();
}

std::string make_session_timestamp() {
    const auto now = std::chrono::system_clock::now();
    const std::time_t now_time = std::chrono::system_clock::to_time_t(now);
    std::tm local_tm{};
#if defined(_WIN32)
    localtime_s(&local_tm, &now_time);
#else
    localtime_r(&now_time, &local_tm);
#endif

    std::ostringstream stream;
    stream << std::put_time(&local_tm, "%Y%m%d_%H%M%S");
    return stream.str();
}

SavedFrameRecord save_current_pair(const std::filesystem::path& session_dir,
                                   const calibration::FramePair& pair,
                                   const cv::Mat& intensity_vis) {
    const std::string stem = make_frame_stem(pair);
    const auto rgb_path = session_dir / (stem + "_rgb.png");
    const auto intensity_path = session_dir / (stem + "_intensity.png");

    if (pair.rgb.bgr.empty()) {
        throw std::runtime_error("Cannot save empty RGB frame");
    }
    if (intensity_vis.empty()) {
        throw std::runtime_error("Cannot save empty intensity frame");
    }

    if (!cv::imwrite(rgb_path.string(), pair.rgb.bgr)) {
        throw std::runtime_error("Failed to write RGB image: " + rgb_path.string());
    }
    if (!cv::imwrite(intensity_path.string(), intensity_vis)) {
        throw std::runtime_error("Failed to write intensity image: " + intensity_path.string());
    }

    SavedFrameRecord record;
    record.rgb_timestamp_ns = pair.rgb.timestamp_ns;
    record.depth_timestamp_ns = pair.depth.timestamp_ns;
    record.rgb_filename = rgb_path.filename().string();
    record.intensity_filename = intensity_path.filename().string();
    record.rgb_width = pair.rgb.bgr.cols;
    record.rgb_height = pair.rgb.bgr.rows;
    record.intensity_width = intensity_vis.cols;
    record.intensity_height = intensity_vis.rows;
    return record;
}

void write_metadata(const std::filesystem::path& metadata_path,
                    const std::string& session_name,
                    const std::filesystem::path& session_dir,
                    const CliArgs& args,
                    const calibration::Phoenix& phoenix,
                    const calibration::Helios2& helios,
                    const std::vector<SavedFrameRecord>& saved_frames) {
    cv::FileStorage fs(metadata_path.string(), cv::FileStorage::WRITE);
    if (!fs.isOpened()) {
        throw std::runtime_error("Failed to open metadata file for write: " + metadata_path.string());
    }

    fs << "session_name" << session_name;
    fs << "session_dir" << session_dir.string();
    fs << "timeout_ms" << static_cast<int>(args.timeout_ms);
    fs << "max_delta_ns" << std::to_string(args.max_delta_ns);
    fs << "saved_pair_count" << static_cast<int>(saved_frames.size());

    fs << "rgb_camera" << "{";
    fs << "label" << phoenix.model_name();
    fs << "device_index" << static_cast<int>(args.phoenix_index);
    fs << "pixel_format" << "BayerRG8";
    fs << "binning" << 2;
    fs << "binning_mode" << "Average";
    fs << "target_acquisition_frame_rate_hz" << 10.0;
    fs << "}";

    fs << "depth_camera" << "{";
    fs << "label" << helios.model_name();
    fs << "device_index" << static_cast<int>(args.helios_index);
    fs << "pixel_format" << args.helios_pixel_format;
    fs << "target_acquisition_frame_rate_hz" << 10.0;
    fs << "saved_representation" << "intensity";
    fs << "}";

    fs << "calibration_notes" << "{";
    fs << "rgb_images_pattern" << "*_rgb.png";
    fs << "depth_images_pattern" << "*_intensity.png";
    fs << "pairing_source" << "live_frame_source_timestamp_matching";
    fs << "pairing_timestamp_tolerance_ns" << std::to_string(args.max_delta_ns);
    fs << "mono_calibration_ready" << 1;
    fs << "stereo_calibration_ready" << 1;
    fs << "}";

    fs << "saved_frames" << "[";
    for (const auto& frame : saved_frames) {
        fs << "{";
        fs << "rgb_timestamp_ns" << std::to_string(frame.rgb_timestamp_ns);
        fs << "depth_timestamp_ns" << std::to_string(frame.depth_timestamp_ns);
        fs << "rgb_filename" << frame.rgb_filename;
        fs << "intensity_filename" << frame.intensity_filename;
        fs << "rgb_width" << frame.rgb_width;
        fs << "rgb_height" << frame.rgb_height;
        fs << "intensity_width" << frame.intensity_width;
        fs << "intensity_height" << frame.intensity_height;
        fs << "}";
    }
    fs << "]";
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const CliArgs args = parse_args(argc, argv);
        std::filesystem::create_directories(args.output_root);
        const std::string session_name = "data_" + make_session_timestamp();
        const std::filesystem::path session_dir = args.output_root / session_name;
        std::filesystem::create_directories(session_dir);
        const std::filesystem::path metadata_path = session_dir / "metadata.yaml";

        calibration::Phoenix phoenix(args.phoenix_index, "BayerRG8", 2, "Average");
        calibration::Helios2 helios(args.helios_index, args.helios_pixel_format);
        calibration::LiveFrameSource source(phoenix, helios, args.max_delta_ns, args.timeout_ms);

        source.open();

        std::cout << "Opened: " << source.description() << "\n";
        std::cout << "Showing Phoenix (left) and Helios intensity (right).\n";
        std::cout << "Saving session into " << session_dir << "\n";
        std::cout << "Press SPACE to save the current pair into that folder\n";
        std::cout << "Press 'q' to quit.\n";

        const std::string window_name = "Dual Capture";
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);

        cv::Mat waiting_frame(480, 960, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::putText(waiting_frame,
                    "Waiting for paired Phoenix + Helios frames...",
                    cv::Point(24, 220),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.8,
                    cv::Scalar(255, 255, 255),
                    2,
                    cv::LINE_AA);
        cv::putText(waiting_frame,
                    "Press SPACE to save once frames appear",
                    cv::Point(24, 270),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.7,
                    cv::Scalar(200, 200, 200),
                    2,
                    cv::LINE_AA);
        cv::imshow(window_name, waiting_frame);
        cv::waitKey(1);

        calibration::FramePair last_pair;
        cv::Mat last_intensity_vis;
        std::vector<SavedFrameRecord> saved_frames;
        write_metadata(metadata_path, session_name, session_dir, args, phoenix, helios, saved_frames);

        while (source.has_next()) {
            calibration::FramePair pair;
            if (!source.next(pair)) {
                continue;
            }

            cv::Mat intensity_vis;
            cv::Mat combined = make_side_by_side_view(pair, intensity_vis);
            cv::imshow(window_name, combined);

            last_pair = pair;
            last_intensity_vis = intensity_vis;

            const int key = cv::waitKey(1) & 0xFF;
            if (key == static_cast<int>('q')) {
                break;
            }
            if (key == static_cast<int>(' ')) {
                saved_frames.push_back(save_current_pair(session_dir, last_pair, last_intensity_vis));
                write_metadata(metadata_path, session_name, session_dir, args, phoenix, helios, saved_frames);
                std::cout << "Saved frame pair rgb_ts=" << last_pair.rgb.timestamp_ns
                          << " depth_ts=" << last_pair.depth.timestamp_ns
                          << " to " << session_dir << "\n";
            }
        }

        source.close();
        cv::destroyWindow(window_name);
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "dual_capture_app failed: " << exception.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "dual_capture_app failed: unknown non-std exception\n";
        return 1;
    }
}
