#include <cstdlib>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "calibration/calibration_base.hpp"
#include "calibration/io/dataset_frame_source.hpp"
#include "calibration/io/frame_source.hpp"
#include "calibration/io/hardware/helios2.hpp"
#include "calibration/io/hardware/phoenix.hpp"
#include "calibration/io/live_frame_source.hpp"
#include "calibration/opencv_calibration.hpp"
#include "calibration/rgbd_calibration.hpp"

namespace {

struct CliArgs {
    std::string backend{"opencv"};
    int rows{9};
    int cols{6};
    float square_size{0.025F};
    std::string out{"calibration_result.yaml"};
    std::string dataset{};
    bool preview{false};
    int max_frames{50};
};

bool parse_bool_flag(const std::string& token, const std::string& long_name) {
    return token == long_name;
}

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

        if (token == "--backend") {
            args.backend = require_value(token);
        } else if (token == "--rows") {
            args.rows = std::stoi(require_value(token));
        } else if (token == "--cols") {
            args.cols = std::stoi(require_value(token));
        } else if (token == "--square-size") {
            args.square_size = std::stof(require_value(token));
        } else if (token == "--out") {
            args.out = require_value(token);
        } else if (token == "--dataset") {
            args.dataset = require_value(token);
        } else if (token == "--max-frames") {
            args.max_frames = std::stoi(require_value(token));
        } else if (parse_bool_flag(token, "--preview")) {
            args.preview = true;
        } else if (token == "--help" || token == "-h") {
            std::cout
                << "Usage: run_calibration [options]\n"
                << "  --backend opencv|rgbd\n"
                << "  --rows N\n"
                << "  --cols N\n"
                << "  --square-size METERS\n"
                << "  --out PATH\n"
                << "  --dataset PATH (optional: use offline dataset source)\n"
                << "  --max-frames N (live mode only)\n"
                << "  --preview\n";
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

        std::unique_ptr<calibration::ICalibration> calibrator;
        if (args.backend == "opencv") {
            calibrator = std::make_unique<calibration::OpenCVCalibration>(
                args.rows, args.cols, args.square_size);
        } else if (args.backend == "rgbd") {
            calibrator = std::make_unique<calibration::RgbdCalibration>();
        } else {
            throw std::invalid_argument("Unsupported backend: " + args.backend);
        }

        std::unique_ptr<calibration::IFrameSource> frame_source;
        calibration::Phoenix phoenix_camera;
        calibration::Helios2 helios_camera;

        if (!args.dataset.empty()) {
            frame_source = std::make_unique<calibration::DatasetFrameSource>(args.dataset);
        } else {
            frame_source = std::make_unique<calibration::LiveFrameSource>(phoenix_camera, helios_camera);
        }

        frame_source->open();
        std::cout << "Source: " << frame_source->description() << "\n";

        calibration::FramePair pair;
        int captured_frames = 0;
        const bool is_live_source = args.dataset.empty();

        while (frame_source->has_next()) {
            if (!frame_source->next(pair)) {
                if (!is_live_source) {
                    continue;
                }
                break;
            }

            calibrator->add_frame(pair);
            ++captured_frames;

            if (is_live_source && captured_frames >= args.max_frames) {
                break;
            }
        }

        frame_source->close();

        if (!calibrator->calibrate()) {
            std::cerr << "Calibration failed: insufficient valid chessboard observations\n";
            return 2;
        }

        calibrator->save(args.out);
        const auto calibration_result = calibrator->result();

        std::cout << "Calibration backend: " << calibrator->name() << "\n";
        std::cout << "Captured frames: " << captured_frames << "\n";
        std::cout << "RMS error: " << calibration_result.rms_error << "\n";
        std::cout << "Saved result to: " << args.out << "\n";

        if (args.preview) {
            std::cout << "--preview requested; visualization hook is reserved for future iteration.\n";
        }

        return 0;
    } catch (const std::exception& ex) {
        std::cerr << "run_calibration error: " << ex.what() << "\n";
        return 1;
    }
}
