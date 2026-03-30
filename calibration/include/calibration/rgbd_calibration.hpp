#ifndef CALIBRATION__RGBD_CALIBRATION_HPP_
#define CALIBRATION__RGBD_CALIBRATION_HPP_

#include <string>

#include "calibration/calibration_base.hpp"

namespace calibration {

class RgbdCalibration final : public ICalibration {
public:
    RgbdCalibration();

    void add_frame(const FramePair& pair) override;
    bool calibrate() override;
    CalibResult result() const override;
    void save(const std::string& path) const override;
    void load(const std::string& path) override;
    std::string name() const override;
};

}  // namespace calibration

#endif  // CALIBRATION__RGBD_CALIBRATION_HPP_
