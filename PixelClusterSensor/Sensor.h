#pragma once

#include <librealsense2/rs2.hpp>
#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>
#include <cstdint>
#include <filesystem>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

namespace pcs {
using Json = nlohmann::json;
namespace fs = std::filesystem;

struct Error : std::runtime_error {
    std::string code;
    Error(std::string c, const std::string& message) : std::runtime_error(message), code(std::move(c)) {}
};

inline void require(bool ok, const std::string& code, const std::string& message) {
    if (!ok) throw Error(code, message);
}

constexpr int MaxPixels = 1280 * 720;
constexpr int MaxClusters = 32768;
constexpr size_t MaxPacketBytes = 96 * 1024 * 1024;
int64_t nowMs();
std::string utf8(const fs::path& path);
Json readJson(const fs::path& path, size_t limit = 4 * 1024 * 1024);
Json parseJson(const std::string& text);
void writeJson(const fs::path& path, const Json& value);
std::vector<uint8_t> readBytes(const fs::path& path, size_t limit);
std::string sha256(const std::vector<uint8_t>& bytes);
void onlyKeys(const Json& value, std::initializer_list<const char*> allowed);
int boundedInt(const Json& value, int minimum, int maximum, const std::string& code);
Json intrinsicsJson(const rs2_intrinsics& value);
rs2_intrinsics parseIntrinsics(const Json& value);
Json extrinsicsJson(const rs2_extrinsics& value);
rs2_extrinsics parseExtrinsics(const Json& value);

struct ProcessingConfig {
    float nearM = 0.3f;
    float farM = 3.5f;
    float depthGapM = 0.04f;
    float depthRelativeGap = 0.02f;
    float colorDelta = 20.0f;
    bool fillEnabled = true;
    int fillMaxPixels = 4;
    int fillMinSamples = 3;
    float fillDepthSpreadM = 0.03f;
    float fillColorDelta = 20.0f;
    Json json() const;
    static ProcessingConfig parse(const Json& value);
};

struct RawFrame {
    cv::Mat colorBgr;
    cv::Mat depth16;
    rs2_intrinsics colorIntrinsics{};
    rs2_intrinsics depthIntrinsics{};
    rs2_extrinsics depthToColor{};
    float depthScale = 0.001f;
    Json source;
};

struct Observation {
    cv::Mat labels;           // CV_32S, positive frame-local IDs; zero is not assigned.
    cv::Mat ownership;        // 0 unresolved, 1 depth-supported, 2 image-only, 3 inherited boundary support.
    cv::Mat depthM;           // Color-camera optical-axis Z, not source-camera Z.
    cv::Mat depthState;       // 0 absent, 1 usable under configured range, 2 out of configured range.
    cv::Mat depthSourceIndex; // CV_32S, source depth linear index; -1 means no sample.
    cv::Mat filledM;
    cv::Mat fillState;        // 0 absent, 1 current-frame color-guided interpolation.
    std::vector<int32_t> contourPoints; // x, y, boundary-reason bits, pixel-center convention.
    Json clusters;
    Json metrics;
};

Observation process(const RawFrame& frame, const ProcessingConfig& config);
Json publish(const fs::path& root, const std::string& session, uint64_t sequence,
             const RawFrame& frame, const Observation& observation,
             const ProcessingConfig& config, uint64_t revision, const Json& control,
             size_t byteLimit = MaxPacketBytes);

class Source {
public:
    virtual ~Source() = default;
    virtual RawFrame next() = 0;
    virtual Json description() const = 0;
};
Json deviceCapabilities();
std::unique_ptr<Source> openSource(const Json& parameters);
int probeStartup(const fs::path& root, int frames, int sessions, const Json& sourceParameters);
}
