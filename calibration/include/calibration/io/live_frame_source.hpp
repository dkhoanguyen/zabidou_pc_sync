#ifndef CALIBRATION__IO__LIVE_FRAME_SOURCE_HPP_
#define CALIBRATION__IO__LIVE_FRAME_SOURCE_HPP_

#include <condition_variable>
#include <cstdint>
#include <chrono>
#include <deque>
#include <mutex>
#include <string>
#include <thread>

#include "calibration/io/frame_source.hpp"
#include "calibration/io/hardware/depth_camera.hpp"
#include "calibration/io/hardware/rgb_camera.hpp"

namespace calibration {

class LiveFrameSource final : public IFrameSource {
public:
    LiveFrameSource(IRgbCamera& rgb_camera,
                    IDepthCamera& depth_camera,
                    std::uint64_t max_timestamp_delta_ns = 20'000'000,
                    std::uint32_t timeout_ms = 1000);

    ~LiveFrameSource() override;

    void open() override;
    void close() override;
    bool has_next() const override;
    bool next(FramePair& out) override;
    void reset() override;
    std::string description() const override;

private:
    struct QueuedRgbFrame {
        RgbFrame frame;
        std::uint64_t host_timestamp_ns{0};
    };

    struct QueuedDepthFrame {
        DepthFrame frame;
        std::uint64_t host_timestamp_ns{0};
    };

    void rgb_capture_loop();
    void depth_capture_loop();
    bool try_make_pair(FramePair& out);
    void trim_queues_locked();
    void clear_queues_locked();
    bool should_log(std::chrono::steady_clock::time_point& last_log_time,
                    std::chrono::milliseconds period) const;

    IRgbCamera& rgb_camera_;
    IDepthCamera& depth_camera_;
    std::uint64_t max_timestamp_delta_ns_;
    std::uint32_t timeout_ms_;
    std::size_t max_queue_size_{8};

    mutable std::mutex mutex_;
    std::condition_variable queue_cv_;
    std::deque<QueuedRgbFrame> rgb_queue_;
    std::deque<QueuedDepthFrame> depth_queue_;
    std::thread rgb_thread_;
    std::thread depth_thread_;
    std::string capture_error_;
    bool stop_requested_{false};
    bool rgb_thread_done_{false};
    bool depth_thread_done_{false};
    bool is_open_{false};
    std::chrono::steady_clock::time_point last_rgb_frame_log_{};
    std::chrono::steady_clock::time_point last_depth_frame_log_{};
    std::chrono::steady_clock::time_point last_rgb_timeout_log_{};
    std::chrono::steady_clock::time_point last_depth_timeout_log_{};
    std::chrono::steady_clock::time_point last_pair_log_{};
};

}  // namespace calibration

#endif  // CALIBRATION__IO__LIVE_FRAME_SOURCE_HPP_
