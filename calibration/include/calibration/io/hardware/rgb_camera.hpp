#ifndef CALIBRATION__IO__HARDWARE__RGB_CAMERA_HPP_
#define CALIBRATION__IO__HARDWARE__RGB_CAMERA_HPP_

#include <cstdint>
#include <string>

#include "calibration/types.hpp"

namespace calibration {

class IRgbCamera {
public:
    virtual ~IRgbCamera() = default;

    virtual void open() = 0;
    virtual void close() = 0;
    virtual bool is_open() const = 0;
    virtual bool grab(RgbFrame& out, std::uint32_t timeout_ms) = 0;
    virtual std::string model_name() const = 0;
};

}  // namespace calibration

#endif  // CALIBRATION__IO__HARDWARE__RGB_CAMERA_HPP_
