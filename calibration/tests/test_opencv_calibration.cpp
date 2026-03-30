#include <iostream>

#include "calibration/opencv_calibration.hpp"

int main() {
    calibration::OpenCVCalibration calibration(9, 6, 0.025F);
    const bool calibrated = calibration.calibrate();

    if (calibrated) {
        std::cerr << "Expected calibrate() to fail without any input frames\n";
        return 1;
    }

    std::cout << "OpenCVCalibration smoke test passed\n";
    return 0;
}
