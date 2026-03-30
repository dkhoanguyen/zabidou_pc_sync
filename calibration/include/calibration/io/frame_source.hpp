#ifndef CALIBRATION__IO__FRAME_SOURCE_HPP_
#define CALIBRATION__IO__FRAME_SOURCE_HPP_

#include <string>

#include "calibration/types.hpp"

namespace calibration {

class IFrameSource {
public:
    virtual ~IFrameSource() = default;

    virtual void open() = 0;
    virtual void close() = 0;
    virtual bool has_next() const = 0;
    virtual bool next(FramePair& out) = 0;
    virtual void reset() = 0;
    virtual std::string description() const = 0;
};

}  // namespace calibration

#endif  // CALIBRATION__IO__FRAME_SOURCE_HPP_
