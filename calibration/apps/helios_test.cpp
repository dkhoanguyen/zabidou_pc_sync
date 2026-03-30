#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include "calibration/io/hardware/helios2.hpp"

namespace {

struct CliArgs {
    std::size_t helios_index{0};
    std::uint32_t timeout_ms{1000};
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

        if (token == "--helios-index") {
            args.helios_index = static_cast<std::size_t>(std::stoul(require_value(token)));
        } else if (token == "--timeout-ms") {
            args.timeout_ms = static_cast<std::uint32_t>(std::stoul(require_value(token)));
        } else if (token == "--helios-format") {
            args.helios_pixel_format = require_value(token);
        } else if (token == "--help" || token == "-h") {
            std::cout
                << "Usage: helios_test [options]\n"
                << "  --helios-index N\n"
                << "  --timeout-ms N\n"
                << "  --helios-format Coord3D_ABCY16|Coord3D_ABCY16s\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("Unknown argument: " + token);
        }
    }

    return args;
}

cv::Mat make_depth_visualization(const calibration::DepthFrame& depth_frame) {
    cv::Mat depth_source;
    if (!depth_frame.xyz.empty()) {
        cv::extractChannel(depth_frame.xyz, depth_source, 2);
    } else if (!depth_frame.intensity.empty()) {
        depth_frame.intensity.convertTo(depth_source, CV_32F);
    }

    if (depth_source.empty()) {
        return cv::Mat::zeros(480, 640, CV_8UC3);
    }

    cv::Mat valid_mask = depth_source > 0;
    double min_value = 0.0;
    double max_value = 0.0;
    cv::minMaxLoc(depth_source, &min_value, &max_value, nullptr, nullptr, valid_mask);

    cv::Mat normalized(depth_source.size(), CV_8UC1, cv::Scalar(0));
    if (max_value > min_value) {
        cv::Mat scaled;
        depth_source.convertTo(
            scaled, CV_8UC1, 255.0 / (max_value - min_value), -min_value * 255.0 / (max_value - min_value));
        scaled.copyTo(normalized, valid_mask);
    }

    cv::Mat colored;
    cv::applyColorMap(normalized, colored, cv::COLORMAP_TURBO);
    colored.setTo(cv::Scalar(0, 0, 0), ~valid_mask);
    return colored;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const CliArgs args = parse_args(argc, argv);

        calibration::Helios2 camera(args.helios_index, args.helios_pixel_format);
        camera.open();

        std::cout << "Opened camera: " << camera.model_name() << "\n";
        std::cout << "Showing Helios depth view. Press 'q' to quit.\n";

        const std::string window_name = "Helios Preview";
        while (true) {
            calibration::DepthFrame frame;
            if (!camera.grab(frame, args.timeout_ms)) {
                continue;
            }

            cv::Mat preview = make_depth_visualization(frame);
            cv::imshow(window_name, preview);

            const int key = cv::waitKey(1) & 0xFF;
            if (key == static_cast<int>('q')) {
                break;
            }
        }

        camera.close();
        cv::destroyWindow(window_name);
        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "helios_test failed: " << exception.what() << "\n";
        return 1;
    }
}
