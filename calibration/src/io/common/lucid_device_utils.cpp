#include "calibration/io/common/lucid_device_utils.hpp"

#include <chrono>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>

#if CALIBRATION_HAS_ARENA_SDK
#include "ArenaApi.h"
#endif

namespace calibration::common {

namespace {

#if CALIBRATION_HAS_ARENA_SDK
std::mutex& arena_system_mutex() {
    static std::mutex mutex;
    return mutex;
}

Arena::ISystem*& shared_arena_system() {
    static Arena::ISystem* system = nullptr;
    return system;
}

std::size_t& shared_arena_system_refcount() {
    static std::size_t refcount = 0;
    return refcount;
}

Arena::ISystem* acquire_arena_system() {
    std::scoped_lock lock(arena_system_mutex());
    auto*& system = shared_arena_system();
    auto& refcount = shared_arena_system_refcount();

    if (system == nullptr) {
        system = Arena::OpenSystem();
    }
    ++refcount;
    return system;
}

void release_arena_system() {
    std::scoped_lock lock(arena_system_mutex());
    auto*& system = shared_arena_system();
    auto& refcount = shared_arena_system_refcount();

    if (system == nullptr || refcount == 0) {
        return;
    }

    --refcount;
    if (refcount == 0) {
        Arena::CloseSystem(system);
        system = nullptr;
    }
}

std::vector<Arena::DeviceInfo> get_device_infos(Arena::ISystem* system, const std::uint32_t timeout_ms = 100) {
    system->UpdateDevices(timeout_ms);
    return system->GetDevices();
}
#endif

}  // namespace

std::uint64_t now_ns() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

std::vector<CameraInfo> find_lucid_cameras() {
#if CALIBRATION_HAS_ARENA_SDK
    std::vector<CameraInfo> cameras;

    Arena::ISystem* system = acquire_arena_system();
    try {
        auto device_infos = get_device_infos(system);
        cameras.reserve(device_infos.size());

        for (std::size_t i = 0; i < device_infos.size(); ++i) {
            auto& device_info = device_infos[i];
            CameraInfo camera_info;
            camera_info.device_index = i;
            camera_info.vendor_name = device_info.VendorName();
            camera_info.model_name = device_info.ModelName();
            camera_info.serial_number = device_info.SerialNumber();
            cameras.push_back(std::move(camera_info));
        }
    } catch (...) {
        release_arena_system();
        throw;
    }

    release_arena_system();
    return cameras;
#else
    return {};
#endif
}

std::optional<CameraInfo> find_camera_by_prefix(const std::vector<CameraInfo>& cameras,
                                                const std::string& model_prefix,
                                                std::size_t device_index) {
    std::size_t matched_index = 0;
    for (const auto& camera_info : cameras) {
        if (camera_info.model_name.rfind(model_prefix, 0) != 0) {
            continue;
        }

        if (matched_index == device_index) {
            return camera_info;
        }
        ++matched_index;
    }

    return std::nullopt;
}

Arena::IDevice* create_lucid_device_by_prefix(const std::string& model_prefix,
                                              std::size_t device_index,
                                              CameraInfo* camera_info_out) {
#if CALIBRATION_HAS_ARENA_SDK
    Arena::ISystem* system = acquire_arena_system();
    try {
        auto device_infos = get_device_infos(system);

        std::size_t matched_index = 0;
        for (std::size_t i = 0; i < device_infos.size(); ++i) {
            auto& device_info = device_infos[i];
            const std::string current_model = device_info.ModelName().c_str();
            if (current_model.rfind(model_prefix, 0) != 0) {
                continue;
            }

            if (matched_index == device_index) {
                if (camera_info_out != nullptr) {
                    camera_info_out->device_index = i;
                    camera_info_out->vendor_name = device_info.VendorName();
                    camera_info_out->model_name = device_info.ModelName();
                    camera_info_out->serial_number = device_info.SerialNumber();
                }

                return system->CreateDevice(device_info);
            }

            ++matched_index;
        }
    } catch (...) {
        release_arena_system();
        throw;
    }

    release_arena_system();
    throw std::runtime_error("No Lucid camera found for prefix " + model_prefix +
                             " and device index " + std::to_string(device_index));
#else
    (void)model_prefix;
    (void)device_index;
    (void)camera_info_out;
    return nullptr;
#endif
}

void destroy_lucid_device(Arena::IDevice* device) {
#if CALIBRATION_HAS_ARENA_SDK
    {
        std::scoped_lock lock(arena_system_mutex());
        Arena::ISystem* system = shared_arena_system();
        if (device != nullptr && system != nullptr) {
            system->DestroyDevice(device);
        }
    }
    release_arena_system();
#else
    (void)device;
#endif
}

HeliosFactoryIntrinsics read_helios_factory_intrinsics(std::size_t device_index) {
#if CALIBRATION_HAS_ARENA_SDK
    CameraInfo camera_info;
    Arena::IDevice* device = create_lucid_device_by_prefix("HTP", device_index, &camera_info);

    try {
        auto* node_map = device->GetNodeMap();
        HeliosFactoryIntrinsics intrinsics;
        intrinsics.camera_info = camera_info;
        intrinsics.focal_length_x = Arena::GetNodeValue<double>(node_map, "CalibFocalLengthX");
        intrinsics.focal_length_y = Arena::GetNodeValue<double>(node_map, "CalibFocalLengthY");
        intrinsics.optical_center_x = Arena::GetNodeValue<double>(node_map, "CalibOpticalCenterX");
        intrinsics.optical_center_y = Arena::GetNodeValue<double>(node_map, "CalibOpticalCenterY");

        intrinsics.lens_distortion_values.reserve(5);
        for (int i = 0; i < 5; ++i) {
            const std::string selector = "Value" + std::to_string(i);
            Arena::SetNodeValue<GenICam::gcstring>(
                node_map, "CalibLensDistortionValueSelector", selector.c_str());
            intrinsics.lens_distortion_values.push_back(
                Arena::GetNodeValue<double>(node_map, "CalibLensDistortionValue"));
        }

        Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateA");
        intrinsics.scale_x = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateScale");
        intrinsics.offset_x = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset");

        Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateB");
        intrinsics.scale_y = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateScale");
        intrinsics.offset_y = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset");

        Arena::SetNodeValue<GenICam::gcstring>(node_map, "Scan3dCoordinateSelector", "CoordinateC");
        intrinsics.scale_z = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateScale");
        intrinsics.offset_z = Arena::GetNodeValue<double>(node_map, "Scan3dCoordinateOffset");

        destroy_lucid_device(device);
        return intrinsics;
    } catch (...) {
        destroy_lucid_device(device);
        throw;
    }
#else
    (void)device_index;
    throw std::runtime_error("read_helios_factory_intrinsics requires CALIBRATION_HAS_ARENA_SDK=1");
#endif
}

std::string format_camera_label(const CameraInfo& camera_info,
                                const std::string& fallback_name) {
    std::ostringstream stream;
    stream << (camera_info.model_name.empty() ? fallback_name : camera_info.model_name);

    if (!camera_info.serial_number.empty()) {
        stream << "[serial=" << camera_info.serial_number << "]";
    } else {
        stream << "[index=" << camera_info.device_index << "]";
    }

    return stream.str();
}

}  // namespace calibration::common
