#include <librealsense2/rs2.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <memory>
#include <optional>
#include <set>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
struct ProbeConfig
{
    std::filesystem::path captureDir;
    std::filesystem::path replayDir;
    std::filesystem::path outputDir;
    int frameCount = 600;
    int warmupFrames = 300;
    int settleFrames = 30;
    int ignoreFirstFrames = 0;
    int repeatFrame = -1;
    int repeatCount = 100;
    bool lockControls = false;
    bool saveFrames = true;
    int intensityDiffThreshold = 5;
    int depthDiffThresholdMm = 20;
    double disparityDiffThresholdPx = 1.0;
    double maxStreamTimestampDeltaMs = 50.0;
    std::optional<float> emitterEnabled;
    std::optional<float> laserPower;
};

struct FrameBundle
{
    cv::Mat colorBgr;
    cv::Mat depthMm16;
    cv::Mat irLeft;
    cv::Mat irRight;
    uint64_t frameNumber = 0;
    double timestampMs = 0.0;
};

struct ProcessedFrame
{
    cv::Mat colorGray;
    cv::Mat colorEdges;
    cv::Mat leftEdges;
    cv::Mat rightEdges;
    cv::Mat depthValidMask;
    cv::Mat disparity32f;
    cv::Mat disparityValidMask;
    uint64_t rawHash = 0;
    uint64_t derivedHash = 0;
};

struct DifferenceStats
{
    double meanAbsolute = 0.0;
    double p95Absolute = 0.0;
    double changedPercent = 0.0;
    double validFlickerPercent = 0.0;
};

struct FrameMetrics
{
    int frameIndex = 0;
    uint64_t sourceFrameNumber = 0;
    double timestampMs = 0.0;
    double colorMean = 0.0;
    double irLeftMean = 0.0;
    double irRightMean = 0.0;
    double depthValidPercent = 0.0;
    double depthMeanMm = 0.0;
    double disparityValidPercent = 0.0;
    double disparityMeanPx = 0.0;
    DifferenceStats colorVsReference;
    DifferenceStats colorVsPrevious;
    DifferenceStats colorEdgeVsReference;
    DifferenceStats irLeftVsReference;
    DifferenceStats irLeftVsPrevious;
    DifferenceStats irRightVsReference;
    DifferenceStats irRightVsPrevious;
    DifferenceStats depthVsReference;
    DifferenceStats depthVsPrevious;
    DifferenceStats disparityVsReference;
    DifferenceStats disparityVsPrevious;
    uint64_t rawHash = 0;
    uint64_t derivedHash = 0;
    bool derivedMatchesReference = false;
};

std::string jsonEscape(const std::string& value)
{
    std::ostringstream out;
    for (const unsigned char ch : value)
    {
        switch (ch)
        {
        case '\\': out << "\\\\"; break;
        case '"': out << "\\\""; break;
        case '\n': out << "\\n"; break;
        case '\r': out << "\\r"; break;
        case '\t': out << "\\t"; break;
        default:
            if (ch < 0x20)
            {
                out << "\\u" << std::hex << std::setw(4) << std::setfill('0')
                    << static_cast<int>(ch) << std::dec;
            }
            else
            {
                out << ch;
            }
        }
    }
    return out.str();
}

std::string hexHash(uint64_t value)
{
    std::ostringstream out;
    out << std::hex << std::setw(16) << std::setfill('0') << value;
    return out.str();
}

uint64_t hashBytes(uint64_t hash, const unsigned char* data, size_t size)
{
    constexpr uint64_t prime = 1099511628211ULL;
    for (size_t index = 0; index < size; ++index)
    {
        hash ^= static_cast<uint64_t>(data[index]);
        hash *= prime;
    }
    return hash;
}

uint64_t hashMat(uint64_t hash, const cv::Mat& image)
{
    const std::array<int, 3> header = { image.rows, image.cols, image.type() };
    hash = hashBytes(hash, reinterpret_cast<const unsigned char*>(header.data()), sizeof(header));
    if (image.empty())
    {
        return hash;
    }
    const size_t rowBytes = static_cast<size_t>(image.cols) * image.elemSize();
    for (int row = 0; row < image.rows; ++row)
    {
        hash = hashBytes(hash, image.ptr<unsigned char>(row), rowBytes);
    }
    return hash;
}

uint64_t hashFrameBundle(const FrameBundle& frame)
{
    uint64_t hash = 1469598103934665603ULL;
    hash = hashMat(hash, frame.colorBgr);
    hash = hashMat(hash, frame.depthMm16);
    hash = hashMat(hash, frame.irLeft);
    return hashMat(hash, frame.irRight);
}

std::optional<std::string> optionValue(int argc, char** argv, const std::string& prefix)
{
    for (int index = 1; index < argc; ++index)
    {
        const std::string argument = argv[index];
        if (argument.rfind(prefix, 0) == 0)
        {
            return argument.substr(prefix.size());
        }
    }
    return std::nullopt;
}

bool hasFlag(int argc, char** argv, const std::string& flag)
{
    for (int index = 1; index < argc; ++index)
    {
        if (argv[index] == flag)
        {
            return true;
        }
    }
    return false;
}

int parseInt(const std::optional<std::string>& text, int fallback, const char* name)
{
    if (!text)
    {
        return fallback;
    }
    try
    {
        size_t consumed = 0;
        const int value = std::stoi(*text, &consumed);
        if (consumed != text->size())
        {
            throw std::invalid_argument("trailing characters");
        }
        return value;
    }
    catch (const std::exception&)
    {
        throw std::runtime_error(std::string("invalid ") + name + ": " + *text);
    }
}

double parseDouble(const std::optional<std::string>& text, double fallback, const char* name)
{
    if (!text)
    {
        return fallback;
    }
    try
    {
        size_t consumed = 0;
        const double value = std::stod(*text, &consumed);
        if (consumed != text->size())
        {
            throw std::invalid_argument("trailing characters");
        }
        return value;
    }
    catch (const std::exception&)
    {
        throw std::runtime_error(std::string("invalid ") + name + ": " + *text);
    }
}

ProbeConfig parseConfig(int argc, char** argv)
{
    ProbeConfig config;
    if (const auto value = optionValue(argc, argv, "--capture-dir=")) config.captureDir = *value;
    if (const auto value = optionValue(argc, argv, "--replay-dir=")) config.replayDir = *value;
    if (const auto value = optionValue(argc, argv, "--out-dir=")) config.outputDir = *value;
    config.frameCount = std::max(1, parseInt(optionValue(argc, argv, "--frames="), config.frameCount, "--frames"));
    config.warmupFrames = std::max(0, parseInt(optionValue(argc, argv, "--warmup-frames="), config.warmupFrames, "--warmup-frames"));
    config.settleFrames = std::max(0, parseInt(optionValue(argc, argv, "--settle-frames="), config.settleFrames, "--settle-frames"));
    config.ignoreFirstFrames = std::max(0, parseInt(optionValue(argc, argv, "--ignore-first-frames="), config.ignoreFirstFrames, "--ignore-first-frames"));
    config.repeatFrame = parseInt(optionValue(argc, argv, "--repeat-frame="), config.repeatFrame, "--repeat-frame");
    config.repeatCount = std::max(1, parseInt(optionValue(argc, argv, "--repeat-count="), config.repeatCount, "--repeat-count"));
    config.intensityDiffThreshold = std::max(0, parseInt(optionValue(argc, argv, "--intensity-diff-threshold="), config.intensityDiffThreshold, "--intensity-diff-threshold"));
    config.depthDiffThresholdMm = std::max(0, parseInt(optionValue(argc, argv, "--depth-diff-threshold-mm="), config.depthDiffThresholdMm, "--depth-diff-threshold-mm"));
    config.disparityDiffThresholdPx = std::max(0.0, parseDouble(optionValue(argc, argv, "--disparity-diff-threshold-px="), config.disparityDiffThresholdPx, "--disparity-diff-threshold-px"));
    config.maxStreamTimestampDeltaMs = std::max(0.0, parseDouble(optionValue(argc, argv, "--max-stream-timestamp-delta-ms="), config.maxStreamTimestampDeltaMs, "--max-stream-timestamp-delta-ms"));
    config.lockControls = hasFlag(argc, argv, "--lock-controls");
    config.saveFrames = !hasFlag(argc, argv, "--no-save-frames");
    if (const auto value = optionValue(argc, argv, "--emitter-enabled="))
    {
        config.emitterEnabled = static_cast<float>(parseDouble(value, 1.0, "--emitter-enabled"));
    }
    if (const auto value = optionValue(argc, argv, "--laser-power="))
    {
        config.laserPower = static_cast<float>(parseDouble(value, 0.0, "--laser-power"));
    }
    if (config.captureDir.empty() == config.replayDir.empty())
    {
        throw std::runtime_error("specify exactly one of --capture-dir or --replay-dir");
    }
    if (config.repeatFrame >= 0 && config.replayDir.empty())
    {
        throw std::runtime_error("--repeat-frame requires --replay-dir");
    }
    if (config.outputDir.empty())
    {
        config.outputDir = config.captureDir.empty()
            ? std::filesystem::path("analysis_runs") / "static_stability_probe"
            : config.captureDir / "analysis";
    }
    return config;
}

void printHelp()
{
    std::cout
        << "StaticStabilityProbe - stateless D455 frame stability measurement\n\n"
        << "Capture:\n"
        << "  StaticStabilityProbe.exe --capture-dir=datasets\\static_stability_auto_001 "
           "--out-dir=analysis_runs\\static_stability_auto_001 --frames=600\n"
        << "  Add --lock-controls to freeze current exposure/gain/white-balance after warmup.\n\n"
        << "Replay:\n"
        << "  StaticStabilityProbe.exe --replay-dir=datasets\\static_stability_auto_001 "
           "--out-dir=analysis_runs\\static_stability_replay_001 --frames=600 --ignore-first-frames=0\n\n"
        << "Same-frame repeatability:\n"
        << "  StaticStabilityProbe.exe --replay-dir=datasets\\static_stability_auto_001 "
           "--repeat-frame=0 --repeat-count=100 --out-dir=analysis_runs\\static_stability_repeat_001\n";
    std::cout
        << "\nMaintenance:\n"
        << "  StaticStabilityProbe.exe --reset-auto-controls\n";
}

cv::Mat frameToMat(const rs2::video_frame& frame, int type)
{
    if (!frame)
    {
        return cv::Mat();
    }
    return cv::Mat(
        cv::Size(frame.get_width(), frame.get_height()),
        type,
        const_cast<void*>(frame.get_data()),
        cv::Mat::AUTO_STEP).clone();
}

double frameMetadataOrNaN(const rs2::frame& frame, rs2_frame_metadata_value field)
{
    try
    {
        if (frame && frame.supports_frame_metadata(field))
        {
            return static_cast<double>(frame.get_frame_metadata(field));
        }
    }
    catch (const rs2::error&)
    {
    }
    return std::numeric_limits<double>::quiet_NaN();
}

cv::Mat depthToMillimeters(const rs2::depth_frame& frame, float depthScale)
{
    cv::Mat units = frameToMat(frame, CV_16UC1);
    if (units.empty())
    {
        return units;
    }
    cv::Mat millimeters32f;
    units.convertTo(millimeters32f, CV_32F, depthScale * 1000.0f);
    cv::Mat millimeters16;
    millimeters32f.convertTo(millimeters16, CV_16U);
    return millimeters16;
}

double validMean(const cv::Mat& image, const cv::Mat& validMask)
{
    if (image.empty() || validMask.empty() || cv::countNonZero(validMask) == 0)
    {
        return 0.0;
    }
    return cv::mean(image, validMask)[0];
}

double percentOfPixels(const cv::Mat& mask)
{
    if (mask.empty())
    {
        return 0.0;
    }
    return 100.0 * static_cast<double>(cv::countNonZero(mask)) /
        static_cast<double>(std::max(1, mask.rows * mask.cols));
}

double percentile(std::vector<double> values, double percentileValue)
{
    values.erase(
        std::remove_if(values.begin(), values.end(), [](double value) { return !std::isfinite(value); }),
        values.end());
    if (values.empty())
    {
        return 0.0;
    }
    std::sort(values.begin(), values.end());
    const double position = std::clamp(percentileValue, 0.0, 100.0) *
        static_cast<double>(values.size() - 1) / 100.0;
    const size_t lower = static_cast<size_t>(std::floor(position));
    const size_t upper = static_cast<size_t>(std::ceil(position));
    const double fraction = position - static_cast<double>(lower);
    return values[lower] * (1.0 - fraction) + values[upper] * fraction;
}

DifferenceStats compareU8(const cv::Mat& current, const cv::Mat& reference, int threshold)
{
    DifferenceStats stats;
    if (current.empty() || reference.empty() || current.size() != reference.size())
    {
        return stats;
    }
    cv::Mat difference;
    cv::absdiff(current, reference, difference);
    stats.meanAbsolute = cv::mean(difference)[0];
    std::array<int, 256> histogram{};
    for (int row = 0; row < difference.rows; ++row)
    {
        const unsigned char* values = difference.ptr<unsigned char>(row);
        for (int column = 0; column < difference.cols; ++column)
        {
            ++histogram[values[column]];
        }
    }
    const int target = static_cast<int>(std::ceil(difference.total() * 0.95));
    int cumulative = 0;
    for (int value = 0; value < static_cast<int>(histogram.size()); ++value)
    {
        cumulative += histogram[static_cast<size_t>(value)];
        if (cumulative >= target)
        {
            stats.p95Absolute = static_cast<double>(value);
            break;
        }
    }
    cv::Mat changed;
    cv::compare(difference, threshold, changed, cv::CMP_GT);
    stats.changedPercent = percentOfPixels(changed);
    return stats;
}

DifferenceStats compareBinary(const cv::Mat& current, const cv::Mat& reference)
{
    DifferenceStats stats;
    if (current.empty() || reference.empty() || current.size() != reference.size())
    {
        return stats;
    }
    cv::Mat different;
    cv::bitwise_xor(current, reference, different);
    stats.changedPercent = percentOfPixels(different);
    stats.meanAbsolute = stats.changedPercent;
    stats.p95Absolute = stats.changedPercent;
    return stats;
}

DifferenceStats compareDepth(
    const cv::Mat& current,
    const cv::Mat& currentValid,
    const cv::Mat& reference,
    const cv::Mat& referenceValid,
    int thresholdMm)
{
    DifferenceStats stats;
    if (current.empty() || reference.empty() || current.size() != reference.size())
    {
        return stats;
    }
    cv::Mat commonValid;
    cv::bitwise_and(currentValid, referenceValid, commonValid);
    cv::Mat validityDifference;
    cv::bitwise_xor(currentValid, referenceValid, validityDifference);
    stats.validFlickerPercent = percentOfPixels(validityDifference);
    std::vector<double> differences;
    differences.reserve(static_cast<size_t>(cv::countNonZero(commonValid)));
    int changed = 0;
    double total = 0.0;
    for (int row = 0; row < current.rows; ++row)
    {
        const uint16_t* currentValues = current.ptr<uint16_t>(row);
        const uint16_t* referenceValues = reference.ptr<uint16_t>(row);
        const unsigned char* valid = commonValid.ptr<unsigned char>(row);
        for (int column = 0; column < current.cols; ++column)
        {
            if (valid[column] == 0)
            {
                continue;
            }
            const double difference = std::abs(
                static_cast<double>(currentValues[column]) -
                static_cast<double>(referenceValues[column]));
            differences.push_back(difference);
            total += difference;
            if (difference > static_cast<double>(thresholdMm))
            {
                ++changed;
            }
        }
    }
    if (!differences.empty())
    {
        stats.meanAbsolute = total / static_cast<double>(differences.size());
        stats.p95Absolute = percentile(differences, 95.0);
        stats.changedPercent = 100.0 * static_cast<double>(changed) /
            static_cast<double>(differences.size());
    }
    return stats;
}

DifferenceStats compareDisparity(
    const cv::Mat& current,
    const cv::Mat& currentValid,
    const cv::Mat& reference,
    const cv::Mat& referenceValid,
    double thresholdPx)
{
    DifferenceStats stats;
    if (current.empty() || reference.empty() || current.size() != reference.size())
    {
        return stats;
    }
    cv::Mat commonValid;
    cv::bitwise_and(currentValid, referenceValid, commonValid);
    cv::Mat validityDifference;
    cv::bitwise_xor(currentValid, referenceValid, validityDifference);
    stats.validFlickerPercent = percentOfPixels(validityDifference);
    std::vector<double> differences;
    differences.reserve(static_cast<size_t>(cv::countNonZero(commonValid)));
    int changed = 0;
    double total = 0.0;
    for (int row = 0; row < current.rows; ++row)
    {
        const float* currentValues = current.ptr<float>(row);
        const float* referenceValues = reference.ptr<float>(row);
        const unsigned char* valid = commonValid.ptr<unsigned char>(row);
        for (int column = 0; column < current.cols; ++column)
        {
            if (valid[column] == 0)
            {
                continue;
            }
            const double difference = std::abs(
                static_cast<double>(currentValues[column]) -
                static_cast<double>(referenceValues[column]));
            differences.push_back(difference);
            total += difference;
            if (difference > thresholdPx)
            {
                ++changed;
            }
        }
    }
    if (!differences.empty())
    {
        stats.meanAbsolute = total / static_cast<double>(differences.size());
        stats.p95Absolute = percentile(differences, 95.0);
        stats.changedPercent = 100.0 * static_cast<double>(changed) /
            static_cast<double>(differences.size());
    }
    return stats;
}

ProcessedFrame processFrameStateless(const FrameBundle& frame)
{
    ProcessedFrame processed;
    processed.rawHash = hashFrameBundle(frame);
    if (!frame.colorBgr.empty())
    {
        cv::cvtColor(frame.colorBgr, processed.colorGray, cv::COLOR_BGR2GRAY);
        cv::Canny(processed.colorGray, processed.colorEdges, 60, 120);
    }
    if (!frame.irLeft.empty())
    {
        cv::Canny(frame.irLeft, processed.leftEdges, 60, 120);
    }
    if (!frame.irRight.empty())
    {
        cv::Canny(frame.irRight, processed.rightEdges, 60, 120);
    }
    if (!frame.depthMm16.empty())
    {
        cv::compare(frame.depthMm16, 0, processed.depthValidMask, cv::CMP_GT);
    }
    if (!frame.irLeft.empty() && !frame.irRight.empty() && frame.irLeft.size() == frame.irRight.size())
    {
        cv::Ptr<cv::StereoSGBM> matcher = cv::StereoSGBM::create(0, 128, 5);
        matcher->setP1(8 * 5 * 5);
        matcher->setP2(32 * 5 * 5);
        matcher->setPreFilterCap(31);
        matcher->setUniquenessRatio(10);
        matcher->setSpeckleWindowSize(100);
        matcher->setSpeckleRange(2);
        matcher->setDisp12MaxDiff(1);
        matcher->setMode(cv::StereoSGBM::MODE_SGBM_3WAY);
        cv::Mat disparity16s;
        matcher->compute(frame.irLeft, frame.irRight, disparity16s);
        disparity16s.convertTo(processed.disparity32f, CV_32F, 1.0 / 16.0);
        cv::compare(processed.disparity32f, 0.5, processed.disparityValidMask, cv::CMP_GT);
    }
    uint64_t hash = 1469598103934665603ULL;
    hash = hashMat(hash, processed.colorGray);
    hash = hashMat(hash, processed.colorEdges);
    hash = hashMat(hash, processed.leftEdges);
    hash = hashMat(hash, processed.rightEdges);
    hash = hashMat(hash, processed.depthValidMask);
    hash = hashMat(hash, processed.disparity32f);
    processed.derivedHash = hashMat(hash, processed.disparityValidMask);
    return processed;
}

FrameMetrics measureFrame(
    int frameIndex,
    const FrameBundle& frame,
    const ProcessedFrame& processed,
    const FrameBundle& referenceFrame,
    const ProcessedFrame& reference,
    const FrameBundle& previousFrame,
    const ProcessedFrame& previous,
    const ProbeConfig& config)
{
    FrameMetrics metrics;
    metrics.frameIndex = frameIndex;
    metrics.sourceFrameNumber = frame.frameNumber;
    metrics.timestampMs = frame.timestampMs;
    metrics.colorMean = processed.colorGray.empty() ? 0.0 : cv::mean(processed.colorGray)[0];
    metrics.irLeftMean = frame.irLeft.empty() ? 0.0 : cv::mean(frame.irLeft)[0];
    metrics.irRightMean = frame.irRight.empty() ? 0.0 : cv::mean(frame.irRight)[0];
    metrics.depthValidPercent = percentOfPixels(processed.depthValidMask);
    metrics.depthMeanMm = validMean(frame.depthMm16, processed.depthValidMask);
    metrics.disparityValidPercent = percentOfPixels(processed.disparityValidMask);
    metrics.disparityMeanPx = validMean(processed.disparity32f, processed.disparityValidMask);
    metrics.colorVsReference = compareU8(processed.colorGray, reference.colorGray, config.intensityDiffThreshold);
    metrics.colorVsPrevious = compareU8(processed.colorGray, previous.colorGray, config.intensityDiffThreshold);
    metrics.colorEdgeVsReference = compareBinary(processed.colorEdges, reference.colorEdges);
    metrics.irLeftVsReference = compareU8(frame.irLeft, referenceFrame.irLeft, config.intensityDiffThreshold);
    metrics.irLeftVsPrevious = compareU8(frame.irLeft, previousFrame.irLeft, config.intensityDiffThreshold);
    metrics.irRightVsReference = compareU8(frame.irRight, referenceFrame.irRight, config.intensityDiffThreshold);
    metrics.irRightVsPrevious = compareU8(frame.irRight, previousFrame.irRight, config.intensityDiffThreshold);
    metrics.depthVsReference = compareDepth(
        frame.depthMm16,
        processed.depthValidMask,
        referenceFrame.depthMm16,
        reference.depthValidMask,
        config.depthDiffThresholdMm);
    metrics.depthVsPrevious = compareDepth(
        frame.depthMm16,
        processed.depthValidMask,
        previousFrame.depthMm16,
        previous.depthValidMask,
        config.depthDiffThresholdMm);
    metrics.disparityVsReference = compareDisparity(
        processed.disparity32f,
        processed.disparityValidMask,
        reference.disparity32f,
        reference.disparityValidMask,
        config.disparityDiffThresholdPx);
    metrics.disparityVsPrevious = compareDisparity(
        processed.disparity32f,
        processed.disparityValidMask,
        previous.disparity32f,
        previous.disparityValidMask,
        config.disparityDiffThresholdPx);
    metrics.rawHash = processed.rawHash;
    metrics.derivedHash = processed.derivedHash;
    metrics.derivedMatchesReference = processed.derivedHash == reference.derivedHash;
    return metrics;
}

class StabilityAnalyzer
{
public:
    explicit StabilityAnalyzer(const ProbeConfig& config)
        : config_(config)
    {
    }

    void add(const FrameBundle& frame)
    {
        const ProcessedFrame processed = processFrameStateless(frame);
        if (!initialized_)
        {
            referenceFrame_ = frame;
            reference_ = processed;
            previousFrame_ = frame;
            previous_ = processed;
            initialized_ = true;
        }
        metrics_.push_back(measureFrame(
            static_cast<int>(metrics_.size()),
            frame,
            processed,
            referenceFrame_,
            reference_,
            previousFrame_,
            previous_,
            config_));
        previousFrame_ = frame;
        previous_ = processed;
    }

    const std::vector<FrameMetrics>& metrics() const
    {
        return metrics_;
    }

private:
    const ProbeConfig& config_;
    bool initialized_ = false;
    FrameBundle referenceFrame_;
    ProcessedFrame reference_;
    FrameBundle previousFrame_;
    ProcessedFrame previous_;
    std::vector<FrameMetrics> metrics_;
};

void writeDifferenceColumns(std::ostream& out, const DifferenceStats& stats)
{
    out << ',' << stats.meanAbsolute
        << ',' << stats.p95Absolute
        << ',' << stats.changedPercent
        << ',' << stats.validFlickerPercent;
}

void writeMetricsCsv(const std::filesystem::path& path, const std::vector<FrameMetrics>& metrics)
{
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out)
    {
        throw std::runtime_error("cannot write " + path.string());
    }
    out << "frame_index,source_frame_number,timestamp_ms,color_mean,ir_left_mean,ir_right_mean,"
           "depth_valid_percent,depth_mean_mm,disparity_valid_percent,disparity_mean_px";
    const std::array<const char*, 11> prefixes = {
        "color_ref", "color_prev", "color_edge_ref", "ir_left_ref", "ir_left_prev",
        "ir_right_ref", "ir_right_prev", "depth_ref", "depth_prev", "disparity_ref", "disparity_prev"
    };
    for (const char* prefix : prefixes)
    {
        out << ',' << prefix << "_mean_abs"
            << ',' << prefix << "_p95_abs"
            << ',' << prefix << "_changed_percent"
            << ',' << prefix << "_valid_flicker_percent";
    }
    out << ",raw_hash,derived_hash,derived_matches_reference\n";
    out << std::fixed << std::setprecision(6);
    for (const FrameMetrics& row : metrics)
    {
        out << row.frameIndex
            << ',' << row.sourceFrameNumber
            << ',' << row.timestampMs
            << ',' << row.colorMean
            << ',' << row.irLeftMean
            << ',' << row.irRightMean
            << ',' << row.depthValidPercent
            << ',' << row.depthMeanMm
            << ',' << row.disparityValidPercent
            << ',' << row.disparityMeanPx;
        writeDifferenceColumns(out, row.colorVsReference);
        writeDifferenceColumns(out, row.colorVsPrevious);
        writeDifferenceColumns(out, row.colorEdgeVsReference);
        writeDifferenceColumns(out, row.irLeftVsReference);
        writeDifferenceColumns(out, row.irLeftVsPrevious);
        writeDifferenceColumns(out, row.irRightVsReference);
        writeDifferenceColumns(out, row.irRightVsPrevious);
        writeDifferenceColumns(out, row.depthVsReference);
        writeDifferenceColumns(out, row.depthVsPrevious);
        writeDifferenceColumns(out, row.disparityVsReference);
        writeDifferenceColumns(out, row.disparityVsPrevious);
        out << ',' << hexHash(row.rawHash)
            << ',' << hexHash(row.derivedHash)
            << ',' << (row.derivedMatchesReference ? 1 : 0)
            << '\n';
    }
}

struct SummaryStats
{
    double p50 = 0.0;
    double p95 = 0.0;
    double maximum = 0.0;
};

template <typename Getter>
SummaryStats summarize(const std::vector<FrameMetrics>& metrics, Getter getter)
{
    std::vector<double> values;
    values.reserve(metrics.size());
    for (const FrameMetrics& metric : metrics)
    {
        values.push_back(getter(metric));
    }
    SummaryStats result;
    result.p50 = percentile(values, 50.0);
    result.p95 = percentile(values, 95.0);
    result.maximum = values.empty() ? 0.0 : *std::max_element(values.begin(), values.end());
    return result;
}

void writeSummaryMetric(std::ostream& out, const char* name, const SummaryStats& stats, bool trailingComma)
{
    out << "    \"" << name << "\": {\"p50\": " << stats.p50
        << ", \"p95\": " << stats.p95
        << ", \"max\": " << stats.maximum << "}";
    if (trailingComma) out << ',';
    out << '\n';
}

void writeSummary(
    const std::filesystem::path& path,
    const ProbeConfig& config,
    const std::vector<FrameMetrics>& metrics,
    const std::string& mode)
{
    std::set<uint64_t> rawHashes;
    std::set<uint64_t> derivedHashes;
    int matchingReference = 0;
    for (const FrameMetrics& metric : metrics)
    {
        rawHashes.insert(metric.rawHash);
        derivedHashes.insert(metric.derivedHash);
        matchingReference += metric.derivedMatchesReference ? 1 : 0;
    }
    const double exactMatchPercent = metrics.empty()
        ? 0.0
        : 100.0 * static_cast<double>(matchingReference) / static_cast<double>(metrics.size());
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    if (!out)
    {
        throw std::runtime_error("cannot write " + path.string());
    }
    out << std::fixed << std::setprecision(6);
    out << "{\n"
        << "  \"schema_version\": 1,\n"
        << "  \"mode\": \"" << jsonEscape(mode) << "\",\n"
        << "  \"frame_count\": " << metrics.size() << ",\n"
        << "  \"unique_raw_hash_count\": " << rawHashes.size() << ",\n"
        << "  \"unique_derived_hash_count\": " << derivedHashes.size() << ",\n"
        << "  \"derived_exact_match_percent\": " << exactMatchPercent << ",\n"
        << "  \"same_frame_output_identical\": ";
    if (config.repeatFrame < 0)
    {
        out << "null";
    }
    else
    {
        out << (derivedHashes.size() == 1 ? "true" : "false");
    }
    out << ",\n"
        << "  \"ignored_first_frames\": " << config.ignoreFirstFrames << ",\n"
        << "  \"reference_source_frame_number\": "
        << (metrics.empty() ? 0 : metrics.front().sourceFrameNumber) << ",\n"
        << "  \"thresholds\": {\n"
        << "    \"intensity_difference\": " << config.intensityDiffThreshold << ",\n"
        << "    \"depth_difference_mm\": " << config.depthDiffThresholdMm << ",\n"
        << "    \"disparity_difference_px\": " << config.disparityDiffThresholdPx << "\n"
        << "  },\n"
        << "  \"metrics\": {\n";
    writeSummaryMetric(out, "color_reference_changed_percent", summarize(metrics, [](const FrameMetrics& value) { return value.colorVsReference.changedPercent; }), true);
    writeSummaryMetric(out, "color_previous_changed_percent", summarize(metrics, [](const FrameMetrics& value) { return value.colorVsPrevious.changedPercent; }), true);
    writeSummaryMetric(out, "color_edge_reference_changed_percent", summarize(metrics, [](const FrameMetrics& value) { return value.colorEdgeVsReference.changedPercent; }), true);
    writeSummaryMetric(out, "ir_left_reference_changed_percent", summarize(metrics, [](const FrameMetrics& value) { return value.irLeftVsReference.changedPercent; }), true);
    writeSummaryMetric(out, "ir_left_previous_changed_percent", summarize(metrics, [](const FrameMetrics& value) { return value.irLeftVsPrevious.changedPercent; }), true);
    writeSummaryMetric(out, "ir_right_reference_changed_percent", summarize(metrics, [](const FrameMetrics& value) { return value.irRightVsReference.changedPercent; }), true);
    writeSummaryMetric(out, "ir_right_previous_changed_percent", summarize(metrics, [](const FrameMetrics& value) { return value.irRightVsPrevious.changedPercent; }), true);
    writeSummaryMetric(out, "depth_reference_mean_abs_mm", summarize(metrics, [](const FrameMetrics& value) { return value.depthVsReference.meanAbsolute; }), true);
    writeSummaryMetric(out, "depth_reference_p95_abs_mm", summarize(metrics, [](const FrameMetrics& value) { return value.depthVsReference.p95Absolute; }), true);
    writeSummaryMetric(out, "depth_valid_flicker_percent", summarize(metrics, [](const FrameMetrics& value) { return value.depthVsReference.validFlickerPercent; }), true);
    writeSummaryMetric(out, "depth_previous_mean_abs_mm", summarize(metrics, [](const FrameMetrics& value) { return value.depthVsPrevious.meanAbsolute; }), true);
    writeSummaryMetric(out, "depth_previous_p95_abs_mm", summarize(metrics, [](const FrameMetrics& value) { return value.depthVsPrevious.p95Absolute; }), true);
    writeSummaryMetric(out, "depth_previous_valid_flicker_percent", summarize(metrics, [](const FrameMetrics& value) { return value.depthVsPrevious.validFlickerPercent; }), true);
    writeSummaryMetric(out, "disparity_reference_mean_abs_px", summarize(metrics, [](const FrameMetrics& value) { return value.disparityVsReference.meanAbsolute; }), true);
    writeSummaryMetric(out, "disparity_reference_p95_abs_px", summarize(metrics, [](const FrameMetrics& value) { return value.disparityVsReference.p95Absolute; }), true);
    writeSummaryMetric(out, "disparity_valid_flicker_percent", summarize(metrics, [](const FrameMetrics& value) { return value.disparityVsReference.validFlickerPercent; }), true);
    writeSummaryMetric(out, "disparity_previous_mean_abs_px", summarize(metrics, [](const FrameMetrics& value) { return value.disparityVsPrevious.meanAbsolute; }), true);
    writeSummaryMetric(out, "disparity_previous_p95_abs_px", summarize(metrics, [](const FrameMetrics& value) { return value.disparityVsPrevious.p95Absolute; }), true);
    writeSummaryMetric(out, "disparity_previous_valid_flicker_percent", summarize(metrics, [](const FrameMetrics& value) { return value.disparityVsPrevious.validFlickerPercent; }), false);
    out << "  },\n"
        << "  \"interpretation\": {\n"
        << "    \"software_repeatability_status\": \""
        << (config.repeatFrame < 0 ? "not_evaluated" : (derivedHashes.size() == 1 ? "pass" : "fail"))
        << "\",\n"
        << "    \"note\": \"Thresholds are descriptive in the first round; do not treat sensor noise as a software failure.\"\n"
        << "  }\n"
        << "}\n";
}

std::string frameStem(int frameIndex)
{
    std::ostringstream out;
    out << std::setw(6) << std::setfill('0') << frameIndex;
    return out.str();
}

FrameBundle loadReplayFrame(const std::filesystem::path& framesDir, int frameIndex)
{
    const std::string stem = frameStem(frameIndex);
    FrameBundle frame;
    frame.colorBgr = cv::imread((framesDir / (stem + "_color.png")).string(), cv::IMREAD_COLOR);
    frame.depthMm16 = cv::imread((framesDir / (stem + "_depth16.png")).string(), cv::IMREAD_UNCHANGED);
    frame.irLeft = cv::imread((framesDir / (stem + "_ir_left.png")).string(), cv::IMREAD_GRAYSCALE);
    frame.irRight = cv::imread((framesDir / (stem + "_ir_right.png")).string(), cv::IMREAD_GRAYSCALE);
    frame.frameNumber = static_cast<uint64_t>(frameIndex);
    if (frame.colorBgr.empty())
    {
        throw std::runtime_error("missing replay color frame: " + (framesDir / (stem + "_color.png")).string());
    }
    if (!frame.depthMm16.empty() && frame.depthMm16.type() != CV_16UC1)
    {
        throw std::runtime_error("replay depth frame is not uint16: " + stem);
    }
    return frame;
}

int countReplayFrames(const std::filesystem::path& framesDir)
{
    int count = 0;
    while (std::filesystem::exists(framesDir / (frameStem(count) + "_color.png")))
    {
        ++count;
    }
    return count;
}

void saveFrameBundle(const std::filesystem::path& framesDir, int frameIndex, const FrameBundle& frame)
{
    const std::string stem = frameStem(frameIndex);
    cv::imwrite((framesDir / (stem + "_color.png")).string(), frame.colorBgr);
    cv::imwrite((framesDir / (stem + "_depth16.png")).string(), frame.depthMm16);
    cv::imwrite((framesDir / (stem + "_ir_left.png")).string(), frame.irLeft);
    cv::imwrite((framesDir / (stem + "_ir_right.png")).string(), frame.irRight);
}

std::vector<rs2_option> observedOptions()
{
    return {
        RS2_OPTION_ENABLE_AUTO_EXPOSURE,
        RS2_OPTION_EXPOSURE,
        RS2_OPTION_GAIN,
        RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,
        RS2_OPTION_WHITE_BALANCE,
        RS2_OPTION_EMITTER_ENABLED,
        RS2_OPTION_LASER_POWER,
        RS2_OPTION_ASIC_TEMPERATURE,
        RS2_OPTION_PROJECTOR_TEMPERATURE
    };
}

std::vector<rs2_option> writableControlOptions()
{
    return {
        RS2_OPTION_ENABLE_AUTO_EXPOSURE,
        RS2_OPTION_EXPOSURE,
        RS2_OPTION_GAIN,
        RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE,
        RS2_OPTION_WHITE_BALANCE,
        RS2_OPTION_EMITTER_ENABLED,
        RS2_OPTION_LASER_POWER
    };
}

class SensorOptionRestoreGuard
{
public:
    explicit SensorOptionRestoreGuard(const rs2::device& device)
    {
        for (rs2::sensor sensor : device.query_sensors())
        {
            SensorSnapshot snapshot;
            snapshot.sensor = sensor;
            for (const rs2_option option : writableControlOptions())
            {
                try
                {
                    if (sensor.supports(option) && !sensor.is_option_read_only(option))
                    {
                        snapshot.values.emplace_back(option, sensor.get_option(option));
                    }
                }
                catch (const rs2::error&)
                {
                }
            }
            snapshots_.push_back(std::move(snapshot));
        }
    }

    ~SensorOptionRestoreGuard()
    {
        restore();
    }

    void restore() noexcept
    {
        if (restored_)
        {
            return;
        }
        for (SensorSnapshot& snapshot : snapshots_)
        {
            const auto savedValue = [&](rs2_option wanted) -> std::optional<float>
            {
                for (const auto& [option, value] : snapshot.values)
                {
                    if (option == wanted) return value;
                }
                return std::nullopt;
            };
            const auto restoreOption = [&](rs2_option option)
            {
                const std::optional<float> value = savedValue(option);
                if (!value) return;
                try { snapshot.sensor.set_option(option, *value); } catch (...) {}
            };

            restoreOption(RS2_OPTION_EMITTER_ENABLED);
            restoreOption(RS2_OPTION_LASER_POWER);

            const std::optional<float> autoExposure = savedValue(RS2_OPTION_ENABLE_AUTO_EXPOSURE);
            if (autoExposure && *autoExposure < 0.5f)
            {
                restoreOption(RS2_OPTION_EXPOSURE);
                restoreOption(RS2_OPTION_GAIN);
            }
            restoreOption(RS2_OPTION_ENABLE_AUTO_EXPOSURE);

            const std::optional<float> autoWhiteBalance = savedValue(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE);
            if (autoWhiteBalance && *autoWhiteBalance < 0.5f)
            {
                restoreOption(RS2_OPTION_WHITE_BALANCE);
            }
            restoreOption(RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE);
        }
        restored_ = true;
    }

private:
    struct SensorSnapshot
    {
        rs2::sensor sensor;
        std::vector<std::pair<rs2_option, float>> values;
    };

    std::vector<SensorSnapshot> snapshots_;
    bool restored_ = false;
};

void writeSensorOptions(const std::filesystem::path& path, const rs2::device& device)
{
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    out << "{\n  \"device\": {\n";
    const auto writeInfo = [&](rs2_camera_info info, const char* name, bool trailing)
    {
        out << "    \"" << name << "\": \"";
        if (device.supports(info)) out << jsonEscape(device.get_info(info));
        out << "\"" << (trailing ? "," : "") << "\n";
    };
    writeInfo(RS2_CAMERA_INFO_NAME, "name", true);
    writeInfo(RS2_CAMERA_INFO_SERIAL_NUMBER, "serial", true);
    writeInfo(RS2_CAMERA_INFO_FIRMWARE_VERSION, "firmware", false);
    out << "  },\n  \"sensors\": [\n";
    const std::vector<rs2::sensor> sensors = device.query_sensors();
    for (size_t sensorIndex = 0; sensorIndex < sensors.size(); ++sensorIndex)
    {
        const rs2::sensor& sensor = sensors[sensorIndex];
        out << "    {\n      \"name\": \"";
        if (sensor.supports(RS2_CAMERA_INFO_NAME)) out << jsonEscape(sensor.get_info(RS2_CAMERA_INFO_NAME));
        out << "\",\n      \"options\": {\n";
        bool first = true;
        for (const rs2_option option : observedOptions())
        {
            if (!sensor.supports(option)) continue;
            if (!first) out << ",\n";
            first = false;
            out << "        \"" << jsonEscape(rs2_option_to_string(option)) << "\": ";
            try
            {
                out << sensor.get_option(option);
            }
            catch (const rs2::error&)
            {
                out << "null";
            }
        }
        out << "\n      }\n    }";
        if (sensorIndex + 1 < sensors.size()) out << ',';
        out << '\n';
    }
    out << "  ]\n}\n";
}

void setOptionIfSupported(rs2::sensor& sensor, rs2_option option, float value)
{
    if (!sensor.supports(option)) return;
    try
    {
        const rs2::option_range range = sensor.get_option_range(option);
        sensor.set_option(option, std::clamp(value, range.min, range.max));
    }
    catch (const rs2::error& error)
    {
        std::cerr << "Warning: cannot set " << rs2_option_to_string(option)
            << " on sensor: " << error.what() << '\n';
    }
}

void applyRequestedControls(const rs2::device& device, const ProbeConfig& config)
{
    for (rs2::sensor sensor : device.query_sensors())
    {
        std::optional<float> exposure;
        std::optional<float> gain;
        std::optional<float> whiteBalance;
        try { if (sensor.supports(RS2_OPTION_EXPOSURE)) exposure = sensor.get_option(RS2_OPTION_EXPOSURE); } catch (...) {}
        try { if (sensor.supports(RS2_OPTION_GAIN)) gain = sensor.get_option(RS2_OPTION_GAIN); } catch (...) {}
        try { if (sensor.supports(RS2_OPTION_WHITE_BALANCE)) whiteBalance = sensor.get_option(RS2_OPTION_WHITE_BALANCE); } catch (...) {}
        if (config.lockControls)
        {
            setOptionIfSupported(sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, 0.0f);
            setOptionIfSupported(sensor, RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 0.0f);
            if (exposure) setOptionIfSupported(sensor, RS2_OPTION_EXPOSURE, *exposure);
            if (gain) setOptionIfSupported(sensor, RS2_OPTION_GAIN, *gain);
            if (whiteBalance) setOptionIfSupported(sensor, RS2_OPTION_WHITE_BALANCE, *whiteBalance);
        }
        if (config.emitterEnabled) setOptionIfSupported(sensor, RS2_OPTION_EMITTER_ENABLED, *config.emitterEnabled);
        if (config.laserPower) setOptionIfSupported(sensor, RS2_OPTION_LASER_POWER, *config.laserPower);
    }
}

int resetAutoControls()
{
    rs2::context context;
    const rs2::device_list devices = context.query_devices();
    if (devices.size() == 0)
    {
        throw std::runtime_error("no RealSense device found");
    }
    const rs2::device device = devices.front();
    for (rs2::sensor sensor : device.query_sensors())
    {
        setOptionIfSupported(sensor, RS2_OPTION_ENABLE_AUTO_EXPOSURE, 1.0f);
        setOptionIfSupported(sensor, RS2_OPTION_ENABLE_AUTO_WHITE_BALANCE, 1.0f);
    }
    std::cout << "Restored supported auto exposure and auto white balance controls.\n";
    return 0;
}

void writeIntrinsics(std::ostream& out, const rs2_intrinsics& value)
{
    out << "{\"width\": " << value.width
        << ", \"height\": " << value.height
        << ", \"ppx\": " << value.ppx
        << ", \"ppy\": " << value.ppy
        << ", \"fx\": " << value.fx
        << ", \"fy\": " << value.fy
        << ", \"model\": " << static_cast<int>(value.model)
        << ", \"coeffs\": [";
    for (int index = 0; index < 5; ++index)
    {
        if (index > 0) out << ", ";
        out << value.coeffs[index];
    }
    out << "]}";
}

void writeExtrinsics(std::ostream& out, const rs2_extrinsics& value)
{
    out << "{\"rotation\": [";
    for (int index = 0; index < 9; ++index)
    {
        if (index > 0) out << ", ";
        out << value.rotation[index];
    }
    out << "], \"translation_m\": [";
    for (int index = 0; index < 3; ++index)
    {
        if (index > 0) out << ", ";
        out << value.translation[index];
    }
    out << "]}";
}

void writeCalibration(const std::filesystem::path& path, const rs2::pipeline_profile& profile)
{
    const auto color = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>();
    const auto depth = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>();
    const auto left = profile.get_stream(RS2_STREAM_INFRARED, 1).as<rs2::video_stream_profile>();
    const auto right = profile.get_stream(RS2_STREAM_INFRARED, 2).as<rs2::video_stream_profile>();
    std::ofstream out(path, std::ios::out | std::ios::trunc);
    out << std::setprecision(10) << "{\n  \"intrinsics\": {\n    \"color\": ";
    writeIntrinsics(out, color.get_intrinsics());
    out << ",\n    \"depth\": ";
    writeIntrinsics(out, depth.get_intrinsics());
    out << ",\n    \"ir_left\": ";
    writeIntrinsics(out, left.get_intrinsics());
    out << ",\n    \"ir_right\": ";
    writeIntrinsics(out, right.get_intrinsics());
    out << "\n  },\n  \"extrinsics\": {\n    \"ir_left_to_ir_right\": ";
    writeExtrinsics(out, left.get_extrinsics_to(right));
    out << ",\n    \"ir_left_to_color\": ";
    writeExtrinsics(out, left.get_extrinsics_to(color));
    out << ",\n    \"depth_to_color\": ";
    writeExtrinsics(out, depth.get_extrinsics_to(color));
    out << "\n  }\n}\n";
}

float depthScale(const rs2::device& device)
{
    for (const rs2::sensor& sensor : device.query_sensors())
    {
        if (const auto depthSensor = sensor.as<rs2::depth_sensor>())
        {
            return depthSensor.get_depth_scale();
        }
    }
    throw std::runtime_error("D455 depth sensor not found");
}

std::vector<FrameMetrics> captureAndAnalyze(const ProbeConfig& config)
{
    rs2::pipeline pipeline;
    rs2::config streamConfig;
    streamConfig.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    streamConfig.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    streamConfig.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    streamConfig.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
    const rs2::pipeline_profile profile = pipeline.start(streamConfig);
    const rs2::device device = profile.get_device();
    const float scale = depthScale(device);
    std::filesystem::create_directories(config.captureDir / "frames");
    std::filesystem::create_directories(config.outputDir);
    writeCalibration(config.captureDir / "calibration.json", profile);

    for (int index = 0; index < config.warmupFrames; ++index)
    {
        pipeline.wait_for_frames();
    }
    writeSensorOptions(config.captureDir / "sensor_options_before_lock.json", device);
    std::unique_ptr<SensorOptionRestoreGuard> restoreGuard;
    if (config.lockControls || config.emitterEnabled || config.laserPower)
    {
        restoreGuard = std::make_unique<SensorOptionRestoreGuard>(device);
        applyRequestedControls(device, config);
        for (int index = 0; index < config.settleFrames; ++index)
        {
            pipeline.wait_for_frames();
        }
    }
    writeSensorOptions(config.captureDir / "sensor_options_capture.json", device);

    StabilityAnalyzer analyzer(config);
    std::ofstream frameMetadata(config.captureDir / "frame_metadata.csv", std::ios::out | std::ios::trunc);
    frameMetadata
        << "capture_index,depth_frame_number,color_frame_number,ir_left_frame_number,ir_right_frame_number,"
           "depth_timestamp_ms,color_timestamp_ms,ir_left_timestamp_ms,ir_right_timestamp_ms,sync_timestamp_span_ms,"
           "depth_actual_exposure,color_actual_exposure,ir_left_actual_exposure,ir_right_actual_exposure,"
           "depth_gain,color_gain,ir_left_gain,ir_right_gain,color_white_balance,depth_laser_power\n";
    frameMetadata << std::fixed << std::setprecision(6);
    int captured = 0;
    int skippedUnsynchronized = 0;
    uint64_t lastDepthFrameNumber = 0;
    uint64_t lastColorFrameNumber = 0;
    uint64_t lastLeftFrameNumber = 0;
    uint64_t lastRightFrameNumber = 0;
    while (captured < config.frameCount)
    {
        const rs2::frameset frameSet = pipeline.wait_for_frames();
        const rs2::video_frame color = frameSet.get_color_frame();
        const rs2::depth_frame depth = frameSet.get_depth_frame();
        const rs2::video_frame left = frameSet.get_infrared_frame(1);
        const rs2::video_frame right = frameSet.get_infrared_frame(2);
        if (!color || !depth || !left || !right)
        {
            continue;
        }
        const uint64_t depthFrameNumber = depth.get_frame_number();
        const uint64_t colorFrameNumber = color.get_frame_number();
        const uint64_t leftFrameNumber = left.get_frame_number();
        const uint64_t rightFrameNumber = right.get_frame_number();
        const std::array<double, 4> timestamps = {
            depth.get_timestamp(), color.get_timestamp(), left.get_timestamp(), right.get_timestamp()
        };
        const auto [minimumTimestamp, maximumTimestamp] = std::minmax_element(timestamps.begin(), timestamps.end());
        const double timestampSpanMs = *maximumTimestamp - *minimumTimestamp;
        const bool streamsAdvanced =
            depthFrameNumber > lastDepthFrameNumber &&
            colorFrameNumber > lastColorFrameNumber &&
            leftFrameNumber > lastLeftFrameNumber &&
            rightFrameNumber > lastRightFrameNumber;
        const bool stereoPairAligned = leftFrameNumber == rightFrameNumber;
        if (!streamsAdvanced || !stereoPairAligned || timestampSpanMs > config.maxStreamTimestampDeltaMs)
        {
            ++skippedUnsynchronized;
            continue;
        }
        lastDepthFrameNumber = depthFrameNumber;
        lastColorFrameNumber = colorFrameNumber;
        lastLeftFrameNumber = leftFrameNumber;
        lastRightFrameNumber = rightFrameNumber;
        FrameBundle frame;
        frame.colorBgr = frameToMat(color, CV_8UC3);
        frame.depthMm16 = depthToMillimeters(depth, scale);
        frame.irLeft = frameToMat(left, CV_8UC1);
        frame.irRight = frameToMat(right, CV_8UC1);
        frame.frameNumber = depthFrameNumber;
        frame.timestampMs = depth.get_timestamp();
        frameMetadata
            << captured
            << ',' << depth.get_frame_number()
            << ',' << color.get_frame_number()
            << ',' << left.get_frame_number()
            << ',' << right.get_frame_number()
            << ',' << depth.get_timestamp()
            << ',' << color.get_timestamp()
            << ',' << left.get_timestamp()
            << ',' << right.get_timestamp()
            << ',' << timestampSpanMs
            << ',' << frameMetadataOrNaN(depth, RS2_FRAME_METADATA_ACTUAL_EXPOSURE)
            << ',' << frameMetadataOrNaN(color, RS2_FRAME_METADATA_ACTUAL_EXPOSURE)
            << ',' << frameMetadataOrNaN(left, RS2_FRAME_METADATA_ACTUAL_EXPOSURE)
            << ',' << frameMetadataOrNaN(right, RS2_FRAME_METADATA_ACTUAL_EXPOSURE)
            << ',' << frameMetadataOrNaN(depth, RS2_FRAME_METADATA_GAIN_LEVEL)
            << ',' << frameMetadataOrNaN(color, RS2_FRAME_METADATA_GAIN_LEVEL)
            << ',' << frameMetadataOrNaN(left, RS2_FRAME_METADATA_GAIN_LEVEL)
            << ',' << frameMetadataOrNaN(right, RS2_FRAME_METADATA_GAIN_LEVEL)
            << ',' << frameMetadataOrNaN(color, RS2_FRAME_METADATA_WHITE_BALANCE)
            << ',' << frameMetadataOrNaN(depth, RS2_FRAME_METADATA_FRAME_LASER_POWER)
            << '\n';
        if (config.saveFrames)
        {
            saveFrameBundle(config.captureDir / "frames", captured, frame);
        }
        analyzer.add(frame);
        ++captured;
    }
    writeSensorOptions(config.captureDir / "sensor_options_after_capture.json", device);
    if (restoreGuard)
    {
        restoreGuard->restore();
        for (int index = 0; index < 3; ++index)
        {
            pipeline.wait_for_frames();
        }
        writeSensorOptions(config.captureDir / "sensor_options_restored.json", device);
    }
    pipeline.stop();

    std::ofstream manifest(config.captureDir / "case_manifest.json", std::ios::out | std::ios::trunc);
    manifest << "{\n"
        << "  \"case_id\": \"" << jsonEscape(config.captureDir.filename().string()) << "\",\n"
        << "  \"format\": \"d455_static_stability_v1\",\n"
        << "  \"frame_count\": " << captured << ",\n"
        << "  \"raw_unaligned_streams\": true,\n"
        << "  \"controls_locked\": " << (config.lockControls ? "true" : "false") << ",\n"
        << "  \"warmup_frames\": " << config.warmupFrames << ",\n"
        << "  \"settle_frames\": " << config.settleFrames << ",\n"
        << "  \"skipped_unsynchronized_framesets\": " << skippedUnsynchronized << ",\n"
        << "  \"max_stream_timestamp_delta_ms\": " << config.maxStreamTimestampDeltaMs << ",\n"
        << "  \"calibration_file\": \"calibration.json\",\n"
        << "  \"frame_metadata_file\": \"frame_metadata.csv\",\n"
        << "  \"sensor_options_file\": \"sensor_options_capture.json\"\n"
        << "}\n";
    return analyzer.metrics();
}

std::vector<FrameMetrics> analyzeReplay(const ProbeConfig& config)
{
    const std::filesystem::path framesDir = config.replayDir / "frames";
    const int available = countReplayFrames(framesDir);
    if (available <= 0)
    {
        throw std::runtime_error("no replay frames found in " + framesDir.string());
    }
    StabilityAnalyzer analyzer(config);
    if (config.repeatFrame >= 0)
    {
        if (config.repeatFrame >= available)
        {
            throw std::runtime_error("--repeat-frame is outside replay frame range");
        }
        const FrameBundle source = loadReplayFrame(framesDir, config.repeatFrame);
        for (int index = 0; index < config.repeatCount; ++index)
        {
            analyzer.add(source);
        }
        return analyzer.metrics();
    }
    const int start = std::min(config.ignoreFirstFrames, available);
    const int count = std::min(config.frameCount, available - start);
    for (int index = 0; index < count; ++index)
    {
        analyzer.add(loadReplayFrame(framesDir, start + index));
    }
    return analyzer.metrics();
}
}

int main(int argc, char** argv)
{
    try
    {
        if (hasFlag(argc, argv, "--help") || hasFlag(argc, argv, "-h"))
        {
            printHelp();
            return 0;
        }
        if (hasFlag(argc, argv, "--reset-auto-controls"))
        {
            return resetAutoControls();
        }
        const ProbeConfig config = parseConfig(argc, argv);
        std::filesystem::create_directories(config.outputDir);
        const std::vector<FrameMetrics> metrics = config.captureDir.empty()
            ? analyzeReplay(config)
            : captureAndAnalyze(config);
        const std::string mode = config.repeatFrame >= 0
            ? "same_frame_repeat"
            : (config.captureDir.empty() ? "replay_sequence" : "live_capture");
        writeMetricsCsv(config.outputDir / "frame_metrics.csv", metrics);
        writeSummary(config.outputDir / "stability_summary.json", config, metrics, mode);
        std::cout << "Static stability probe complete: mode=" << mode
            << " frames=" << metrics.size()
            << " output=" << config.outputDir.string() << '\n';
        return 0;
    }
    catch (const rs2::error& error)
    {
        std::cerr << "RealSense error: " << error.what() << '\n';
        return 2;
    }
    catch (const cv::Exception& error)
    {
        std::cerr << "OpenCV error: " << error.what() << '\n';
        return 3;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Error: " << error.what() << '\n';
        return 1;
    }
}
