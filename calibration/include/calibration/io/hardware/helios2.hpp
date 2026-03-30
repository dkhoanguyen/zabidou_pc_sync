#ifndef CALIBRATION__IO__HARDWARE__HELIOS2_HPP_
#define CALIBRATION__IO__HARDWARE__HELIOS2_HPP_

#include <cstdint>
#include <cstddef>
#include <string>

#include "calibration/io/hardware/depth_camera.hpp"

namespace Arena {
class IDevice;
}  // namespace Arena

namespace calibration {

class Helios2 final : public IDepthCamera {
public:
    explicit Helios2(std::size_t device_index = 0,
                     std::string pixel_format = "Coord3D_ABCY16");

    ~Helios2() override;

    void open() override;
    void close() override;
    bool is_open() const override;
    bool grab(DepthFrame& out, std::uint32_t timeout_ms) override;
    std::string model_name() const override;

private:
    void configure_device();
    void read_coordinate_metadata();

    std::size_t device_index_{0};
    std::string pixel_format_{"Coord3D_ABCY16"};
    std::string camera_label_{"Helios2"};
    Arena::IDevice* device_{nullptr};
    float scale_x_{1.0F};
    float scale_y_{1.0F};
    float scale_z_{1.0F};
    float offset_x_{0.0F};
    float offset_y_{0.0F};
    bool is_open_{false};
};

}  // namespace calibration

#endif  // CALIBRATION__IO__HARDWARE__HELIOS2_HPP_
