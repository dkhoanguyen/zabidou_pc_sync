#ifndef CALIBRATION__IO__COMMON__LUCID_DEVICE_UTILS_HPP_
#define CALIBRATION__IO__COMMON__LUCID_DEVICE_UTILS_HPP_

#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace Arena {
class IDevice;
class ISystem;
}  // namespace Arena

namespace calibration::common {

struct CameraInfo {
    std::size_t device_index{0};
    std::string vendor_name;
    std::string model_name;
    std::string serial_number;
};

struct HeliosFactoryIntrinsics {
    CameraInfo camera_info;
    double focal_length_x{0.0};
    double focal_length_y{0.0};
    double optical_center_x{0.0};
    double optical_center_y{0.0};
    std::vector<double> lens_distortion_values;
    double scale_x{0.0};
    double scale_y{0.0};
    double scale_z{0.0};
    double offset_x{0.0};
    double offset_y{0.0};
    double offset_z{0.0};
};

std::uint64_t now_ns();

std::vector<CameraInfo> find_lucid_cameras();

std::optional<CameraInfo> find_camera_by_prefix(const std::vector<CameraInfo>& cameras,
                                                const std::string& model_prefix,
                                                std::size_t device_index);

Arena::IDevice* create_lucid_device_by_prefix(const std::string& model_prefix,
                                              std::size_t device_index,
                                              CameraInfo* camera_info_out = nullptr);

void destroy_lucid_device(Arena::IDevice* device);

HeliosFactoryIntrinsics read_helios_factory_intrinsics(std::size_t device_index = 0);

std::string format_camera_label(const CameraInfo& camera_info,
                                const std::string& fallback_name);

}  // namespace calibration::common

#endif  // CALIBRATION__IO__COMMON__LUCID_DEVICE_UTILS_HPP_
