#ifndef CALIBRATION__IO__DATASET_FRAME_SOURCE_HPP_
#define CALIBRATION__IO__DATASET_FRAME_SOURCE_HPP_

#include <filesystem>
#include <string>
#include <utility>
#include <vector>

#include "calibration/io/frame_source.hpp"

namespace calibration {

class DatasetFrameSource final : public IFrameSource {
public:
    explicit DatasetFrameSource(std::filesystem::path root_dir,
                                std::string rgb_glob = "rgb_*.png",
                                std::string depth_glob = "depth_*.png");

    void open() override;
    void close() override;
    bool has_next() const override;
    bool next(FramePair& out) override;
    void reset() override;
    std::string description() const override;

private:
    static std::vector<std::filesystem::path> collect_files(const std::filesystem::path& root,
                                                            const std::string& pattern_prefix,
                                                            const std::string& extension);
    static std::pair<std::string, std::string> split_simple_glob(const std::string& glob);

    std::filesystem::path root_dir_;
    std::string rgb_glob_;
    std::string depth_glob_;

    std::vector<std::filesystem::path> rgb_files_;
    std::vector<std::filesystem::path> depth_files_;
    std::size_t index_{0};
    bool is_open_{false};
};

}  // namespace calibration

#endif  // CALIBRATION__IO__DATASET_FRAME_SOURCE_HPP_
