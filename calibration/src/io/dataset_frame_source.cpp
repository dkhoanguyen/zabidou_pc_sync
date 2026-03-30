#include "calibration/io/dataset_frame_source.hpp"

#include <algorithm>
#include <chrono>
#include <stdexcept>

#include <opencv2/imgcodecs.hpp>

namespace calibration {

namespace {

std::uint64_t now_ns() {
    return static_cast<std::uint64_t>(
        std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::steady_clock::now().time_since_epoch())
            .count());
}

}  // namespace

DatasetFrameSource::DatasetFrameSource(std::filesystem::path root_dir,
                                       std::string rgb_glob,
                                       std::string depth_glob)
    : root_dir_(std::move(root_dir)),
      rgb_glob_(std::move(rgb_glob)),
      depth_glob_(std::move(depth_glob)) {}

void DatasetFrameSource::open() {
    if (!std::filesystem::exists(root_dir_)) {
        throw std::runtime_error("Dataset root does not exist: " + root_dir_.string());
    }

    const auto [rgb_prefix, rgb_extension] = split_simple_glob(rgb_glob_);
    const auto [depth_prefix, depth_extension] = split_simple_glob(depth_glob_);

    rgb_files_ = collect_files(root_dir_, rgb_prefix, rgb_extension);
    depth_files_ = collect_files(root_dir_, depth_prefix, depth_extension);

    if (rgb_files_.empty() || depth_files_.empty()) {
        throw std::runtime_error("Dataset frame source found no RGB/depth files under: " + root_dir_.string());
    }

    if (rgb_files_.size() != depth_files_.size()) {
        throw std::runtime_error("Dataset frame source expected matching pair counts for RGB/depth files");
    }

    index_ = 0;
    is_open_ = true;
}

void DatasetFrameSource::close() {
    rgb_files_.clear();
    depth_files_.clear();
    index_ = 0;
    is_open_ = false;
}

bool DatasetFrameSource::has_next() const {
    return is_open_ && index_ < rgb_files_.size() && index_ < depth_files_.size();
}

bool DatasetFrameSource::next(FramePair& out) {
    if (!has_next()) {
        return false;
    }

    const auto& rgb_path = rgb_files_[index_];
    const auto& depth_path = depth_files_[index_];

    out.rgb.bgr = cv::imread(rgb_path.string(), cv::IMREAD_COLOR);
    out.depth.intensity = cv::imread(depth_path.string(), cv::IMREAD_UNCHANGED);

    if (out.rgb.bgr.empty() || out.depth.intensity.empty()) {
        ++index_;
        return false;
    }

    out.rgb.timestamp_ns = now_ns();
    out.depth.timestamp_ns = out.rgb.timestamp_ns;
    ++index_;
    return true;
}

void DatasetFrameSource::reset() {
    index_ = 0;
}

std::string DatasetFrameSource::description() const {
    return "DatasetFrameSource(root=" + root_dir_.string() + ")";
}

std::vector<std::filesystem::path> DatasetFrameSource::collect_files(
    const std::filesystem::path& root,
    const std::string& pattern_prefix,
    const std::string& extension) {
    std::vector<std::filesystem::path> files;

    for (const auto& entry : std::filesystem::directory_iterator(root)) {
        if (!entry.is_regular_file()) {
            continue;
        }

        const auto path = entry.path();
        if (!extension.empty() && path.extension().string() != extension) {
            continue;
        }

        const auto stem = path.stem().string();
        if (!pattern_prefix.empty() && stem.rfind(pattern_prefix, 0) != 0) {
            continue;
        }

        files.push_back(path);
    }

    std::sort(files.begin(), files.end());
    return files;
}

std::pair<std::string, std::string> DatasetFrameSource::split_simple_glob(const std::string& glob) {
    const auto star_pos = glob.find('*');
    if (star_pos == std::string::npos) {
        const auto dot_pos = glob.find_last_of('.');
        if (dot_pos == std::string::npos) {
            return {glob, std::string{}};
        }
        return {glob.substr(0, dot_pos), glob.substr(dot_pos)};
    }

    const auto dot_pos = glob.find('.', star_pos);
    std::string extension;
    if (dot_pos != std::string::npos) {
        extension = glob.substr(dot_pos);
    }

    return {glob.substr(0, star_pos), extension};
}

}  // namespace calibration
