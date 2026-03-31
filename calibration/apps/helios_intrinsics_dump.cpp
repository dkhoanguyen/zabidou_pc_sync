#include <exception>
#include <iostream>
#include <stdexcept>
#include <string>

#include <opencv2/core.hpp>

#include "calibration/io/common/lucid_device_utils.hpp"

namespace {

struct CliArgs {
    std::size_t helios_index{0};
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
        } else if (token == "--help" || token == "-h") {
            std::cout
                << "Usage: helios_intrinsics_dump [options]\n"
                << "  --helios-index N\n";
            std::exit(0);
        } else {
            throw std::invalid_argument("Unknown argument: " + token);
        }
    }

    return args;
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const CliArgs args = parse_args(argc, argv);
        const auto intrinsics = calibration::common::read_helios_factory_intrinsics(args.helios_index);

        cv::Mat camera_matrix = cv::Mat::eye(3, 3, CV_64F);
        camera_matrix.at<double>(0, 0) = intrinsics.focal_length_x;
        camera_matrix.at<double>(1, 1) = intrinsics.focal_length_y;
        camera_matrix.at<double>(0, 2) = intrinsics.optical_center_x;
        camera_matrix.at<double>(1, 2) = intrinsics.optical_center_y;

        cv::Mat dist_coeffs =
            cv::Mat::zeros(1, static_cast<int>(intrinsics.lens_distortion_values.size()), CV_64F);
        for (int i = 0; i < dist_coeffs.cols; ++i) {
            dist_coeffs.at<double>(0, i) = intrinsics.lens_distortion_values[static_cast<std::size_t>(i)];
        }

        std::cout << "Helios camera: "
                  << calibration::common::format_camera_label(intrinsics.camera_info, "Helios2") << "\n";
        std::cout << "camera_matrix:\n" << camera_matrix << "\n\n";
        std::cout << "dist_coeffs:\n" << dist_coeffs << "\n\n";
        std::cout << "coordinate_scales:\n"
                  << "  x: " << intrinsics.scale_x << "\n"
                  << "  y: " << intrinsics.scale_y << "\n"
                  << "  z: " << intrinsics.scale_z << "\n\n";
        std::cout << "coordinate_offsets:\n"
                  << "  x: " << intrinsics.offset_x << "\n"
                  << "  y: " << intrinsics.offset_y << "\n"
                  << "  z: " << intrinsics.offset_z << "\n";
        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "helios_intrinsics_dump failed: " << ex.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "helios_intrinsics_dump failed: unknown non-std exception\n";
        return 1;
    }
}
