#ifndef CALIBRATION__IO__HARDWARE__PHOENIX_HPP_
#define CALIBRATION__IO__HARDWARE__PHOENIX_HPP_

#include <cstdint>
#include <cstddef>
#include <string>

#include "calibration/io/hardware/rgb_camera.hpp"

namespace Arena {
class IDevice;
}  // namespace Arena

namespace calibration {

class Phoenix final : public IRgbCamera {
public:
    explicit Phoenix(std::size_t device_index = 0,
                     std::string pixel_format = "BayerRG8",
                     int binning = 2,
                     std::string binning_mode = "Average");

    ~Phoenix() override;

    void open() override;
    void close() override;
    bool is_open() const override;
    bool grab(RgbFrame& out, std::uint32_t timeout_ms) override;
    std::string model_name() const override;
    void preview(const std::string& window_name = "Phoenix Preview",
                 std::uint32_t timeout_ms = 1000,
                 char quit_key = 'q');

private:
    void configure_device();

    std::size_t device_index_{0};
    std::string pixel_format_{"BayerRG8"};
    int binning_{2};
    std::string binning_mode_{"Average"};
    std::string camera_label_{"Phoenix"};
    Arena::IDevice* device_{nullptr};
    bool is_open_{false};
};

}  // namespace calibration

#endif  // CALIBRATION__IO__HARDWARE__PHOENIX_HPP_
