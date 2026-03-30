#ifndef CALIBRATION__OPENCV_CALIBRATION_HPP_
#define CALIBRATION__OPENCV_CALIBRATION_HPP_

#include <string>

#include <opencv2/core.hpp>

#include "calibration/calibration_base.hpp"

namespace calibration {

class OpenCVCalibration final : public ICalibration {
public:
    OpenCVCalibration(int board_rows,
                      int board_cols,
                      float square_size_meters,
                      bool use_fast_check = true);

    void add_frame(const FramePair& pair) override;
    bool calibrate() override;
    CalibResult result() const override;
    void save(const std::string& path) const override;
    void load(const std::string& path) override;
    std::string name() const override;

private:
    cv::Size board_size_;
    float square_size_meters_;
    bool use_fast_check_{true};
};

}  // namespace calibration

#endif  // CALIBRATION__OPENCV_CALIBRATION_HPP_
