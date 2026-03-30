#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/highgui.hpp>
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
        } else if (token == "--help" || token == "-h") {
            std::cout
                << "Usage: dual_camera_test [options]\n"
                << "  --phoenix-index N\n"
                << "  --helios-index N\n"
                << "  --timeout-ms N\n"
                << "  --max-delta-ns N\n"
                << "  --helios-format Coord3D_ABCY16|Coord3D_ABCY16s\n";
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

cv::Mat make_side_by_side_view(const calibration::FramePair& pair) {
    const cv::Mat intensity_vis = make_intensity_visualization(pair.depth);
    if (intensity_vis.empty()) {
        throw std::runtime_error("Intensity visualization frame is empty");
    }

    cv::Mat rgb_view;
    if (pair.rgb.bgr.empty()) {
        rgb_view = cv::Mat::zeros(intensity_vis.rows, intensity_vis.cols, CV_8UC3);
    } else {
        rgb_view = pair.rgb.bgr;
    }

    const int output_height = std::max(rgb_view.rows, intensity_vis.rows);
    cv::Mat rgb_padded(output_height, rgb_view.cols, CV_8UC3, cv::Scalar(0, 0, 0));
    cv::Mat intensity_padded(output_height, intensity_vis.cols, CV_8UC3, cv::Scalar(0, 0, 0));

    rgb_view.copyTo(rgb_padded(cv::Rect(0, 0, rgb_view.cols, rgb_view.rows)));
    intensity_vis.copyTo(
        intensity_padded(cv::Rect(0, 0, intensity_vis.cols, intensity_vis.rows)));

    cv::Mat combined;
    cv::hconcat(rgb_padded, intensity_padded, combined);
    return combined;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const CliArgs args = parse_args(argc, argv);

        calibration::Phoenix phoenix(args.phoenix_index, "BayerRG8", 2, "Average");
        calibration::Helios2 helios(args.helios_index, args.helios_pixel_format);
        calibration::LiveFrameSource source(phoenix, helios, args.max_delta_ns, args.timeout_ms);

        source.open();

        std::cout << "Opened: " << source.description() << "\n";
        std::cout << "Showing Phoenix (left) and Helios intensity (right). Press 'q' to quit.\n";

        const std::string window_name = "Phoenix + Helios";
        cv::namedWindow(window_name, cv::WINDOW_NORMAL);
        cv::Mat waiting_frame(480, 960, CV_8UC3, cv::Scalar(0, 0, 0));
        cv::putText(waiting_frame,
                    "Waiting for paired Phoenix + Helios frames...",
                    cv::Point(24, 240),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.8,
                    cv::Scalar(255, 255, 255),
                    2,
                    cv::LINE_AA);
        cv::imshow(window_name, waiting_frame);
        cv::waitKey(1);

        while (source.has_next()) {
            calibration::FramePair pair;
            if (!source.next(pair)) {
                continue;
            }

            cv::Mat combined = make_side_by_side_view(pair);
            cv::imshow(window_name, combined);

            const int key = cv::waitKey(1) & 0xFF;
            if (key == static_cast<int>('q')) {
                break;
            }
        }

        source.close();
        cv::destroyWindow(window_name);
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "dual_camera_test failed: " << exception.what() << "\n";
        return 1;
    }
}
