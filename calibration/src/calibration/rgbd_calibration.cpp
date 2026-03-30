#include "calibration/rgbd_calibration.hpp"

#include <stdexcept>

namespace calibration {

RgbdCalibration::RgbdCalibration() {
    throw std::logic_error(
        "RgbdCalibration is not implemented yet. Planned: depth polynomial undistortion + Ceres joint optimization.");
}

void RgbdCalibration::add_frame(const FramePair& pair) {
    frames_.push_back(pair);
}

bool RgbdCalibration::calibrate() {
    throw std::logic_error("RgbdCalibration::calibrate is not implemented yet");
}

CalibResult RgbdCalibration::result() const {
    return result_;
}

void RgbdCalibration::save(const std::string& /*path*/) const {
    throw std::logic_error("RgbdCalibration::save is not implemented yet");
}

void RgbdCalibration::load(const std::string& /*path*/) {
    throw std::logic_error("RgbdCalibration::load is not implemented yet");
}

std::string RgbdCalibration::name() const {
    return "rgbd";
}

}  // namespace calibration
