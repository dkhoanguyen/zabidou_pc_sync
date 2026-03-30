#ifndef CALIBRATION__CALIBRATION_BASE_HPP_
#define CALIBRATION__CALIBRATION_BASE_HPP_

#include <string>
#include <vector>

#include "calibration/types.hpp"

namespace calibration {

class ICalibration {
public:
    virtual ~ICalibration() = default;

    virtual void add_frame(const FramePair& pair) = 0;
    virtual bool calibrate() = 0;
    virtual CalibResult result() const = 0;
    virtual void save(const std::string& path) const = 0;
    virtual void load(const std::string& path) = 0;
    virtual std::string name() const = 0;

protected:
    std::vector<FramePair> frames_;
    CalibResult result_;
};

}  // namespace calibration

#endif  // CALIBRATION__CALIBRATION_BASE_HPP_
