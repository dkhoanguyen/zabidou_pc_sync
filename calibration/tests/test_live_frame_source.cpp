#include <chrono>
#include <cstdint>
#include <iostream>
#include <string>
#include <thread>

#include <opencv2/core.hpp>

#include "calibration/io/live_frame_source.hpp"

namespace {

class FakeRgbCamera final : public calibration::IRgbCamera {
public:
    void open() override { is_open_ = true; }
    void close() override { is_open_ = false; }
    bool is_open() const override { return is_open_; }

    bool grab(calibration::RgbFrame& out, std::uint32_t /*timeout_ms*/) override {
        if (!is_open_) {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(2));
        out.bgr = cv::Mat::ones(4, 4, CV_8UC3);
        out.timestamp_ns = next_timestamp_ns_;
        next_timestamp_ns_ += 10'000'000;
        return true;
    }

    std::string model_name() const override { return "FakeRgb"; }

private:
    bool is_open_{false};
    std::uint64_t next_timestamp_ns_{1'000'000'000};
};

class FakeDepthCamera final : public calibration::IDepthCamera {
public:
    void open() override { is_open_ = true; }
    void close() override { is_open_ = false; }
    bool is_open() const override { return is_open_; }

    bool grab(calibration::DepthFrame& out, std::uint32_t /*timeout_ms*/) override {
        if (!is_open_) {
            return false;
        }

        std::this_thread::sleep_for(std::chrono::milliseconds(3));
        out.intensity = cv::Mat::ones(4, 4, CV_16UC1);
        out.xyz = cv::Mat::zeros(4, 4, CV_32FC3);
        out.timestamp_ns = next_timestamp_ns_;
        next_timestamp_ns_ += 10'000'000;
        return true;
    }

    std::string model_name() const override { return "FakeDepth"; }

private:
    bool is_open_{false};
    std::uint64_t next_timestamp_ns_{1'001'000'000};
};

}  // namespace

int main() {
    FakeRgbCamera rgb_camera;
    FakeDepthCamera depth_camera;
    calibration::LiveFrameSource source(rgb_camera, depth_camera, 5'000'000, 50);

    source.open();

    calibration::FramePair pair;
    if (!source.next(pair)) {
        std::cerr << "Expected threaded live frame source to produce a paired frame\n";
        source.close();
        return 1;
    }

    if (pair.rgb.bgr.empty() || pair.depth.intensity.empty() || pair.depth.xyz.empty()) {
        std::cerr << "Expected non-empty paired RGB/depth frames\n";
        source.close();
        return 1;
    }

    const auto delta = (pair.rgb.timestamp_ns > pair.depth.timestamp_ns)
                           ? (pair.rgb.timestamp_ns - pair.depth.timestamp_ns)
                           : (pair.depth.timestamp_ns - pair.rgb.timestamp_ns);
    if (delta > 5'000'000) {
        std::cerr << "Expected paired frames to be synchronized within threshold\n";
        source.close();
        return 1;
    }

    source.close();

    std::cout << "LiveFrameSource threaded pairing test passed\n";
    return 0;
}
