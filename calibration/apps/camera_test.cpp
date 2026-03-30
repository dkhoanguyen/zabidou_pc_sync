#include <exception>
#include <iostream>

#include "calibration/io/hardware/phoenix.hpp"

int main() {
    try {
        calibration::Phoenix camera;
        camera.open();

        std::cout << "Opened camera: " << camera.model_name() << "\n";
        std::cout << "Starting preview. Press 'q' to quit.\n";

        camera.preview("Phoenix Preview", 1000, 'q');
        camera.close();

        return 0;
    } catch (const std::exception& exception) {
        std::cerr << "camera_test failed: " << exception.what() << "\n";
        return 1;
    }
}
