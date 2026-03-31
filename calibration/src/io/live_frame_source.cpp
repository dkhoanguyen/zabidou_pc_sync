#include "calibration/io/live_frame_source.hpp"

#include <limits>
#include <iostream>
#include <exception>
#include <sstream>
#include <stdexcept>
#include <utility>

namespace calibration {

namespace {

std::uint64_t now_ns() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

}  // namespace

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
    const auto wait_step = std::chrono::milliseconds(timeout_ms_);
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

        const bool woke_for_event = queue_cv_.wait_for(lock, wait_step, [this, &out] {
            return !capture_error_.empty() || try_make_pair(out) ||
                   ((!is_open_ || (rgb_thread_done_ && depth_thread_done_)) &&
                    (rgb_queue_.empty() || depth_queue_.empty()));
        });

        if (!capture_error_.empty()) {
            throw std::runtime_error("Live capture failed: " + capture_error_);
        }

        if (!woke_for_event) {
            return false;
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

bool LiveFrameSource::should_log(std::chrono::steady_clock::time_point& last_log_time,
                                 std::chrono::milliseconds period) const {
    const auto now = std::chrono::steady_clock::now();
    if (last_log_time.time_since_epoch().count() == 0 || now - last_log_time >= period) {
        last_log_time = now;
        return true;
    }
    return false;
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
                std::scoped_lock lock(mutex_);
                if (should_log(last_rgb_timeout_log_, std::chrono::milliseconds(2000))) {
                    std::clog << "[LiveFrameSource] RGB grab timeout after " << timeout_ms_
                              << " ms\n";
                }
                continue;
            }

            {
                std::scoped_lock lock(mutex_);
                if (stop_requested_) {
                    break;
                }
                rgb_queue_.push_back(QueuedRgbFrame{std::move(frame), now_ns()});
                trim_queues_locked();
                if (should_log(last_rgb_frame_log_, std::chrono::milliseconds(2000))) {
                    const auto& latest = rgb_queue_.back().frame;
                    std::clog << "[LiveFrameSource] RGB frame received ts=" << latest.timestamp_ns
                              << " queue=" << rgb_queue_.size() << "\n";
                }
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
                std::scoped_lock lock(mutex_);
                if (should_log(last_depth_timeout_log_, std::chrono::milliseconds(2000))) {
                    std::clog << "[LiveFrameSource] Depth grab timeout after " << timeout_ms_
                              << " ms\n";
                }
                continue;
            }

            {
                std::scoped_lock lock(mutex_);
                if (stop_requested_) {
                    break;
                }
                depth_queue_.push_back(QueuedDepthFrame{std::move(frame), now_ns()});
                trim_queues_locked();
                if (should_log(last_depth_frame_log_, std::chrono::milliseconds(2000))) {
                    const auto& latest = depth_queue_.back().frame;
                    std::clog << "[LiveFrameSource] Depth frame received ts=" << latest.timestamp_ns
                              << " queue=" << depth_queue_.size() << "\n";
                }
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
        std::size_t best_rgb_index = rgb_queue_.size();
        std::size_t best_depth_index = depth_queue_.size();
        std::uint64_t best_delta = std::numeric_limits<std::uint64_t>::max();

        for (std::size_t rgb_index = 0; rgb_index < rgb_queue_.size(); ++rgb_index) {
            const auto rgb_timestamp = rgb_queue_[rgb_index].host_timestamp_ns;
            for (std::size_t depth_index = 0; depth_index < depth_queue_.size(); ++depth_index) {
                const auto depth_timestamp = depth_queue_[depth_index].host_timestamp_ns;
                const auto delta = (rgb_timestamp > depth_timestamp)
                                       ? (rgb_timestamp - depth_timestamp)
                                       : (depth_timestamp - rgb_timestamp);
                if (delta < best_delta) {
                    best_delta = delta;
                    best_rgb_index = rgb_index;
                    best_depth_index = depth_index;
                }
            }
        }

        if (best_rgb_index < rgb_queue_.size() &&
            best_depth_index < depth_queue_.size() &&
            best_delta <= max_timestamp_delta_ns_) {
            out.rgb = std::move(rgb_queue_[best_rgb_index].frame);
            out.depth = std::move(depth_queue_[best_depth_index].frame);
            rgb_queue_.erase(rgb_queue_.begin() + static_cast<std::ptrdiff_t>(best_rgb_index));
            depth_queue_.erase(depth_queue_.begin() + static_cast<std::ptrdiff_t>(best_depth_index));
            if (should_log(last_pair_log_, std::chrono::milliseconds(2000))) {
                std::clog << "[LiveFrameSource] Paired frame rgb_ts=" << out.rgb.timestamp_ns
                          << " depth_ts=" << out.depth.timestamp_ns
                          << " delta_ms=" << (static_cast<double>(best_delta) / 1e6)
                          << " rgb_queue=" << rgb_queue_.size()
                          << " depth_queue=" << depth_queue_.size() << "\n";
            }
            return true;
        }

        const auto& rgb = rgb_queue_.front();
        const auto& depth = depth_queue_.front();
        if (rgb.host_timestamp_ns + max_timestamp_delta_ns_ < depth.host_timestamp_ns) {
            rgb_queue_.pop_front();
        } else if (depth.host_timestamp_ns + max_timestamp_delta_ns_ < rgb.host_timestamp_ns) {
            depth_queue_.pop_front();
        } else {
            break;
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
