#include "calibration/io/live_frame_source.hpp"

#include <exception>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace calibration {

LiveFrameSource::LiveFrameSource(IRgbCamera& rgb_camera,
                                 IDepthCamera& depth_camera,
                                 std::uint64_t max_timestamp_delta_ns,
                                 std::uint32_t timeout_ms)
    : rgb_camera_(rgb_camera),
      depth_camera_(depth_camera),
      max_timestamp_delta_ns_(max_timestamp_delta_ns),
      timeout_ms_(timeout_ms) {}

LiveFrameSource::~LiveFrameSource() {
    close();
}

void LiveFrameSource::open() {
    {
        std::scoped_lock lock(mutex_);
        if (is_open_) {
            return;
        }
    }

    bool opened_rgb_here = false;
    bool opened_depth_here = false;
    try {
        if (!rgb_camera_.is_open()) {
            rgb_camera_.open();
            opened_rgb_here = true;
        }
        if (!depth_camera_.is_open()) {
            depth_camera_.open();
            opened_depth_here = true;
        }
    } catch (...) {
        if (opened_rgb_here && rgb_camera_.is_open()) {
            rgb_camera_.close();
        }
        if (opened_depth_here && depth_camera_.is_open()) {
            depth_camera_.close();
        }
        throw;
    }

    std::scoped_lock lock(mutex_);
    capture_error_.clear();
    stop_requested_ = false;
    rgb_thread_done_ = false;
    depth_thread_done_ = false;
    clear_queues_locked();
    is_open_ = true;
    rgb_thread_ = std::thread(&LiveFrameSource::rgb_capture_loop, this);
    depth_thread_ = std::thread(&LiveFrameSource::depth_capture_loop, this);
}

void LiveFrameSource::close() {
    {
        std::scoped_lock lock(mutex_);
        if (!is_open_ && !rgb_thread_.joinable() && !depth_thread_.joinable()) {
            return;
        }

        stop_requested_ = true;
        is_open_ = false;
        queue_cv_.notify_all();
    }

    if (rgb_thread_.joinable()) {
        rgb_thread_.join();
    }
    if (depth_thread_.joinable()) {
        depth_thread_.join();
    }

    if (rgb_camera_.is_open()) {
        rgb_camera_.close();
    }
    if (depth_camera_.is_open()) {
        depth_camera_.close();
    }

    std::scoped_lock lock(mutex_);
    clear_queues_locked();
}

bool LiveFrameSource::has_next() const {
    std::scoped_lock lock(mutex_);
    return is_open_ || !rgb_queue_.empty() || !depth_queue_.empty();
}

bool LiveFrameSource::next(FramePair& out) {
    out = FramePair{};
    std::unique_lock<std::mutex> lock(mutex_);
    while (true) {
        if (try_make_pair(out)) {
            return true;
        }

        if (!capture_error_.empty()) {
            throw std::runtime_error("Live capture failed: " + capture_error_);
        }

        const bool capture_finished = rgb_thread_done_ && depth_thread_done_;
        if ((!is_open_ || capture_finished) && (rgb_queue_.empty() || depth_queue_.empty())) {
            return false;
        }

        queue_cv_.wait(lock, [this, &out] {
            return !capture_error_.empty() || try_make_pair(out) ||
                   ((!is_open_ || (rgb_thread_done_ && depth_thread_done_)) &&
                    (rgb_queue_.empty() || depth_queue_.empty()));
        });

        if (!capture_error_.empty()) {
            throw std::runtime_error("Live capture failed: " + capture_error_);
        }

        if (!out.rgb.bgr.empty() && !out.depth.intensity.empty()) {
            return true;
        }

        if ((!is_open_ || (rgb_thread_done_ && depth_thread_done_)) &&
            (rgb_queue_.empty() || depth_queue_.empty())) {
            return false;
        }
    }
}

void LiveFrameSource::reset() {
    // No-op for live source.
}

std::string LiveFrameSource::description() const {
    std::ostringstream oss;
    oss << "LiveFrameSource(threaded,rgb=" << rgb_camera_.model_name()
        << ", depth=" << depth_camera_.model_name() << ")";
    return oss.str();
}

void LiveFrameSource::rgb_capture_loop() {
    try {
        while (true) {
            {
                std::scoped_lock lock(mutex_);
                if (stop_requested_) {
                    break;
                }
            }

            RgbFrame frame;
            if (!rgb_camera_.grab(frame, timeout_ms_)) {
                continue;
            }

            {
                std::scoped_lock lock(mutex_);
                if (stop_requested_) {
                    break;
                }
                rgb_queue_.push_back(std::move(frame));
                trim_queues_locked();
            }
            queue_cv_.notify_all();
        }
    } catch (const std::exception& ex) {
        std::scoped_lock lock(mutex_);
        if (capture_error_.empty()) {
            capture_error_ = ex.what();
        }
    } catch (...) {
        std::scoped_lock lock(mutex_);
        if (capture_error_.empty()) {
            capture_error_ = "unknown RGB capture error";
        }
    }

    {
        std::scoped_lock lock(mutex_);
        rgb_thread_done_ = true;
    }
    queue_cv_.notify_all();
}

void LiveFrameSource::depth_capture_loop() {
    try {
        while (true) {
            {
                std::scoped_lock lock(mutex_);
                if (stop_requested_) {
                    break;
                }
            }

            DepthFrame frame;
            if (!depth_camera_.grab(frame, timeout_ms_)) {
                continue;
            }

            {
                std::scoped_lock lock(mutex_);
                if (stop_requested_) {
                    break;
                }
                depth_queue_.push_back(std::move(frame));
                trim_queues_locked();
            }
            queue_cv_.notify_all();
        }
    } catch (const std::exception& ex) {
        std::scoped_lock lock(mutex_);
        if (capture_error_.empty()) {
            capture_error_ = ex.what();
        }
    } catch (...) {
        std::scoped_lock lock(mutex_);
        if (capture_error_.empty()) {
            capture_error_ = "unknown depth capture error";
        }
    }

    {
        std::scoped_lock lock(mutex_);
        depth_thread_done_ = true;
    }
    queue_cv_.notify_all();
}

bool LiveFrameSource::try_make_pair(FramePair& out) {
    while (!rgb_queue_.empty() && !depth_queue_.empty()) {
        const auto& rgb = rgb_queue_.front();
        const auto& depth = depth_queue_.front();
        const auto delta = (rgb.timestamp_ns > depth.timestamp_ns)
                               ? (rgb.timestamp_ns - depth.timestamp_ns)
                               : (depth.timestamp_ns - rgb.timestamp_ns);

        if (delta <= max_timestamp_delta_ns_) {
            out.rgb = std::move(rgb_queue_.front());
            out.depth = std::move(depth_queue_.front());
            rgb_queue_.pop_front();
            depth_queue_.pop_front();
            return true;
        }

        if (rgb.timestamp_ns < depth.timestamp_ns) {
            rgb_queue_.pop_front();
        } else {
            depth_queue_.pop_front();
        }
    }

    return false;
}

void LiveFrameSource::trim_queues_locked() {
    while (rgb_queue_.size() > max_queue_size_) {
        rgb_queue_.pop_front();
    }
    while (depth_queue_.size() > max_queue_size_) {
        depth_queue_.pop_front();
    }
}

void LiveFrameSource::clear_queues_locked() {
    rgb_queue_.clear();
    depth_queue_.clear();
}

}  // namespace calibration
