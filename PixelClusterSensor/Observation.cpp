#include "Sensor.h"
#include <librealsense2/rsutil.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <algorithm>
#include <array>
#include <bit>
#include <chrono>
#include <climits>
#include <cmath>
#include <fstream>
#include <limits>
#include <map>
#include <numeric>
#include <set>

namespace pcs {
Json ProcessingConfig::json() const {
    return {{"聚簇模式", clusteringMode}, {"可用深度近界米", nearM}, {"可用深度远界米", farM}, {"邻接深度差米", depthGapM},
        {"邻接深度相对差", depthRelativeGap}, {"归属色差", colorDelta},
        {"深度拆分最小支持像素", depthSplitMinSupportPixels},
        {"强制吸收深度噪点最大像素", depthNoiseMaxPixels},
        {"跨颜色合并最小连续边界像素", crossColorMergeMinBoundaryPixels},
        {"允许同彩图区域跨缺测全局合并", mergeCompatibleDepthSeedsAcrossMissing}, {"启用补全", fillEnabled},
        {"补全最大缺测连通像素", fillMaxPixels}, {"补全最少样本", fillMinSamples},
        {"补全深度跨度米", fillDepthSpreadM}, {"补全色差", fillColorDelta}};
}
ProcessingConfig ProcessingConfig::parse(const Json& j) {
    onlyKeys(j, {"聚簇模式", "可用深度近界米", "可用深度远界米", "邻接深度差米", "邻接深度相对差", "归属色差",
        "深度拆分最小支持像素", "强制吸收深度噪点最大像素", "跨颜色合并最小连续边界像素",
        "允许同彩图区域跨缺测全局合并",
        "启用补全", "补全最大缺测连通像素", "补全最少样本", "补全深度跨度米", "补全色差"});
    ProcessingConfig c;
    if (j.contains("聚簇模式")) require(j.at("聚簇模式").is_string(), "invalid_config", "聚簇模式必须是字符串");
    c.clusteringMode = j.value("聚簇模式", c.clusteringMode);
    c.nearM = j.value("可用深度近界米", c.nearM); c.farM = j.value("可用深度远界米", c.farM);
    c.depthGapM = j.value("邻接深度差米", c.depthGapM); c.depthRelativeGap = j.value("邻接深度相对差", c.depthRelativeGap);
    c.colorDelta = j.value("归属色差", c.colorDelta); c.fillEnabled = j.value("启用补全", c.fillEnabled);
    for (const auto* key : {"深度拆分最小支持像素", "强制吸收深度噪点最大像素", "跨颜色合并最小连续边界像素",
                            "补全最大缺测连通像素", "补全最少样本"})
        if (j.contains(key)) require(j.at(key).is_number_integer(), "invalid_config", "计数配置必须是整数");
    c.depthSplitMinSupportPixels = boundedInt(j.value("深度拆分最小支持像素", Json(c.depthSplitMinSupportPixels)), 1, 100000, "invalid_config");
    c.depthNoiseMaxPixels = boundedInt(j.value("强制吸收深度噪点最大像素", Json(c.depthNoiseMaxPixels)), 1, 1024, "invalid_config");
    c.crossColorMergeMinBoundaryPixels = boundedInt(j.value("跨颜色合并最小连续边界像素", Json(c.crossColorMergeMinBoundaryPixels)), 1, 10000, "invalid_config");
    if (j.contains("允许同彩图区域跨缺测全局合并"))
        require(j.at("允许同彩图区域跨缺测全局合并").is_boolean(), "invalid_config", "跨缺测全局合并开关必须是布尔值");
    c.mergeCompatibleDepthSeedsAcrossMissing = j.value("允许同彩图区域跨缺测全局合并", c.mergeCompatibleDepthSeedsAcrossMissing);
    c.fillMaxPixels = boundedInt(j.value("补全最大缺测连通像素", Json(c.fillMaxPixels)), 1, 16, "invalid_config");
    c.fillMinSamples = boundedInt(j.value("补全最少样本", Json(c.fillMinSamples)), 3, 8, "invalid_config");
    c.fillDepthSpreadM = j.value("补全深度跨度米", c.fillDepthSpreadM);
    c.fillColorDelta = j.value("补全色差", c.fillColorDelta);
    for (float value : {c.nearM, c.farM, c.depthGapM, c.depthRelativeGap, c.colorDelta, c.fillDepthSpreadM, c.fillColorDelta})
        require(std::isfinite(value), "invalid_config", "配置必须为有限数值");
    require(c.nearM > 0 && c.farM > c.nearM && c.farM <= 100 && c.depthGapM > 0 && c.depthGapM <= 0.5 &&
        c.depthRelativeGap >= 0 && c.depthRelativeGap <= 0.2 && c.colorDelta > 0 && c.colorDelta <= 100 &&
        c.depthSplitMinSupportPixels >= 1 && c.depthSplitMinSupportPixels <= 100000 &&
        c.depthNoiseMaxPixels >= 1 && c.depthNoiseMaxPixels <= 1024 &&
        c.crossColorMergeMinBoundaryPixels >= 1 && c.crossColorMergeMinBoundaryPixels <= 10000 &&
        c.fillMaxPixels >= 1 && c.fillMaxPixels <= 16 && c.fillMinSamples >= 3 && c.fillMinSamples <= 8 &&
        c.fillDepthSpreadM > 0 && c.fillDepthSpreadM <= 0.2 && c.fillColorDelta > 0 && c.fillColorDelta <= 100,
        "invalid_config", "处理配置越界");
    require(c.clusteringMode == "轮廓主导" || c.clusteringMode == "深度主导", "invalid_config", "未知聚簇模式");
    return c;
}

namespace {
const std::array<cv::Point, 4> Neighbors{cv::Point(-1, 0), cv::Point(1, 0), cv::Point(0, -1), cv::Point(0, 1)};
bool inside(const cv::Size& size, int x, int y) { return x >= 0 && y >= 0 && x < size.width && y < size.height; }
float deltaSquared(const cv::Mat& lab, cv::Point a, cv::Point b) {
    const auto d = lab.at<cv::Vec3f>(a) - lab.at<cv::Vec3f>(b);
    return d.dot(d);
}
void validateFrame(const RawFrame& f) {
    require(f.colorBgr.type() == CV_8UC3 && f.depth16.type() == CV_16UC1 && !f.colorBgr.empty() && !f.depth16.empty(),
            "invalid_frame", "必须提供完整RGB与depth16材料");
    require(f.colorBgr.total() <= MaxPixels && f.depth16.total() <= MaxPixels,
            "resource_limit", "输入像素数超限");
    parseIntrinsics(intrinsicsJson(f.colorIntrinsics)); parseIntrinsics(intrinsicsJson(f.depthIntrinsics));
    parseExtrinsics(extrinsicsJson(f.depthToColor));
    require(f.colorBgr.size() == cv::Size(f.colorIntrinsics.width, f.colorIntrinsics.height) &&
            f.depth16.size() == cv::Size(f.depthIntrinsics.width, f.depthIntrinsics.height),
            "calibration_mismatch", "图像与标定不相容");
    require(std::isfinite(f.depthScale) && f.depthScale > 0 && f.depthScale <= 1,
            "invalid_calibration", "深度单位非法");
    require(f.source.at("彩图时间域") == f.source.at("深度时间域") &&
        std::abs(f.source.at("彩图时间戳毫秒").get<double>() - f.source.at("深度时间戳毫秒").get<double>()) <= 50,
        "unsynchronized_frame", "双流时间差超出50ms原型门槛");
}
void alignDepth(const RawFrame& f, Observation& out, const ProcessingConfig& config) {
    const auto size = f.colorBgr.size();
    out.depthM = cv::Mat::zeros(size, CV_32F);
    out.depthState = cv::Mat::zeros(size, CV_8U);
    out.depthSourceIndex = cv::Mat(size, CV_32S, cv::Scalar(-1));
    uint64_t rejected = 0, collisions = 0;
    for (int y = 0; y < f.depth16.rows; ++y) {
        for (int x = 0; x < f.depth16.cols; ++x) {
            const uint16_t raw = f.depth16.at<uint16_t>(y, x);
            if (raw == 0 || raw == 65535) { ++rejected; continue; }
            const float pixel[2]{static_cast<float>(x), static_cast<float>(y)};
            float p[3], transformed[3], projected[2];
            rs2_deproject_pixel_to_point(p, &f.depthIntrinsics, pixel, raw * f.depthScale);
            rs2_transform_point_to_point(transformed, &f.depthToColor, p);
            if (!std::isfinite(transformed[2]) || transformed[2] <= 0) { ++rejected; continue; }
            rs2_project_point_to_pixel(projected, &f.colorIntrinsics, transformed);
            if (!std::isfinite(projected[0]) || !std::isfinite(projected[1]) ||
                projected[0] < -0.5f || projected[1] < -0.5f || projected[0] >= size.width - 0.5f || projected[1] >= size.height - 0.5f) {
                ++rejected; continue;
            }
            const int u = static_cast<int>(std::floor(projected[0] + 0.5f));
            const int v = static_cast<int>(std::floor(projected[1] + 0.5f));
            float& z = out.depthM.at<float>(v, u);
            if (z > 0) ++collisions;
            if (z > 0 && z <= transformed[2]) continue;
            z = transformed[2];
            out.depthSourceIndex.at<int>(v, u) = y * f.depth16.cols + x;
            out.depthState.at<uint8_t>(v, u) = z >= config.nearM && z <= config.farM ? 1 : 2;
        }
    }
    out.metrics["配准丢弃源采样数"] = rejected;
    out.metrics["配准竞争采样数"] = collisions;
}

void partitionDepthFirst(const cv::Mat& lab, Observation& out, const ProcessingConfig& c) {
    const auto size = lab.size();
    out.labels = cv::Mat::zeros(size, CV_32S);
    out.ownership = cv::Mat::zeros(size, CV_8U);
    std::vector<cv::Point> queue;
    queue.reserve(lab.total());
    int next = 1;
    // Only current usable depth can join geometric anchors. Missing pixels never union two anchors.
    for (int y = 0; y < size.height; ++y) for (int x = 0; x < size.width; ++x) {
        if (out.depthState.at<uint8_t>(y, x) != 1 || out.labels.at<int>(y, x)) continue;
        require(next <= MaxClusters, "resource_limit", "簇数量超限，未发布残缺帧");
        const int label = next++;
        queue.clear(); queue.emplace_back(x, y); out.labels.at<int>(y, x) = label;
        for (size_t i = 0; i < queue.size(); ++i) {
            const auto p = queue[i]; out.ownership.at<uint8_t>(p) = 1;
            for (const auto d : Neighbors) {
                const auto q = p + d;
                if (!inside(size, q.x, q.y) || out.labels.at<int>(q) || out.depthState.at<uint8_t>(q) != 1) continue;
                const float a = out.depthM.at<float>(p), b = out.depthM.at<float>(q);
                if (std::abs(a - b) > c.depthGapM + c.depthRelativeGap * std::min(a, b)) continue;
                out.labels.at<int>(q) = label; queue.push_back(q);
            }
        }
    }
    cv::Mat visited = cv::Mat::zeros(size, CV_8U);
    for (int y = 0; y < size.height; ++y) for (int x = 0; x < size.width; ++x) {
        if (out.labels.at<int>(y, x) || visited.at<uint8_t>(y, x)) continue;
        queue.clear(); queue.emplace_back(x, y); visited.at<uint8_t>(y, x) = 1;
        std::set<int> anchors;
        bool touchesBorder = false, containsRangeRejected = false;
        for (size_t i = 0; i < queue.size(); ++i) {
            const auto p = queue[i];
            containsRangeRejected |= out.depthState.at<uint8_t>(p) == 2;
            for (const auto d : Neighbors) {
                const auto q = p + d;
                if (!inside(size, q.x, q.y)) { touchesBorder = true; continue; }
                if (deltaSquared(lab, p, q) > c.colorDelta * c.colorDelta) continue;
                if (out.depthState.at<uint8_t>(q) == 1) anchors.insert(out.labels.at<int>(q));
                else if (!visited.at<uint8_t>(q) && !out.labels.at<int>(q)) {
                    visited.at<uint8_t>(q) = 1; queue.push_back(q);
                }
            }
        }
        const bool inherit = anchors.size() == 1 && !touchesBorder && !containsRangeRejected;
        require(inherit || next <= MaxClusters, "resource_limit", "簇数量超限，未发布残缺帧");
        const int label = inherit ? *anchors.begin() : next++;
        for (const auto p : queue) {
            out.labels.at<int>(p) = label;
            out.ownership.at<uint8_t>(p) = inherit ? 3 : 2;
        }
    }
    std::vector<int> remap(static_cast<size_t>(next), 0);
    int canonical = 0;
    for (int y = 0; y < size.height; ++y) for (int x = 0; x < size.width; ++x) {
        int& id = out.labels.at<int>(y, x);
        if (!remap[id]) remap[id] = ++canonical;
        id = remap[id];
    }
    out.metrics["簇数量"] = canonical;
    out.metrics["聚簇模式"] = "深度主导";
}

bool depthContinuous(const Observation& out, const ProcessingConfig& c, cv::Point a, cv::Point b) {
    if (out.depthState.at<uint8_t>(a) != 1 || out.depthState.at<uint8_t>(b) != 1) return false;
    const float first = out.depthM.at<float>(a), second = out.depthM.at<float>(b);
    return std::abs(first - second) <= c.depthGapM + c.depthRelativeGap * std::min(first, second);
}

class DisjointSet {
public:
    explicit DisjointSet(int count) : parent_(static_cast<size_t>(count + 1)) {
        std::iota(parent_.begin(), parent_.end(), 0);
    }
    int find(int value) {
        int root = value;
        while (parent_[static_cast<size_t>(root)] != root) root = parent_[static_cast<size_t>(root)];
        while (parent_[static_cast<size_t>(value)] != value) {
            const int next = parent_[static_cast<size_t>(value)];
            parent_[static_cast<size_t>(value)] = root;
            value = next;
        }
        return root;
    }
    bool merge(int first, int second) {
        first = find(first); second = find(second);
        if (first == second) return false;
        if (first > second) std::swap(first, second);
        parent_[static_cast<size_t>(second)] = first;
        return true;
    }
private:
    std::vector<int> parent_;
};

void partitionContourFirst(const cv::Mat& lab, Observation& out, const ProcessingConfig& c) {
    const auto size = lab.size();
    cv::Mat colorLabels = cv::Mat::zeros(size, CV_32S);
    cv::Mat depthSeeds = cv::Mat::zeros(size, CV_32S);
    std::vector<std::vector<cv::Point>> colorPixels(1);
    std::vector<cv::Point> queue;
    queue.reserve(lab.total());

    int colorCount = 0;
    for (int y = 0; y < size.height; ++y) for (int x = 0; x < size.width; ++x) {
        if (colorLabels.at<int>(y, x)) continue;
        require(colorCount < MaxClusters, "resource_limit", "彩图候选数量超限");
        const int label = ++colorCount;
        colorPixels.emplace_back();
        queue.clear(); queue.emplace_back(x, y); colorLabels.at<int>(y, x) = label;
        for (size_t index = 0; index < queue.size(); ++index) {
            const auto point = queue[index];
            colorPixels[static_cast<size_t>(label)].push_back(point);
            for (const auto delta : Neighbors) {
                const auto neighbor = point + delta;
                if (!inside(size, neighbor.x, neighbor.y) || colorLabels.at<int>(neighbor)) continue;
                if (deltaSquared(lab, point, neighbor) > c.colorDelta * c.colorDelta) continue;
                colorLabels.at<int>(neighbor) = label;
                queue.push_back(neighbor);
            }
        }
    }

    std::vector<double> seedDepthSum(1, 0.0);
    std::vector<int> seedDepthCount(1, 0);
    int seedCount = 0;
    for (int y = 0; y < size.height; ++y) for (int x = 0; x < size.width; ++x) {
        if (out.depthState.at<uint8_t>(y, x) != 1 || depthSeeds.at<int>(y, x)) continue;
        require(seedCount < MaxClusters, "resource_limit", "深度区域数量超限");
        const int seed = ++seedCount;
        seedDepthSum.push_back(0.0); seedDepthCount.push_back(0);
        queue.clear(); queue.emplace_back(x, y); depthSeeds.at<int>(y, x) = seed;
        for (size_t index = 0; index < queue.size(); ++index) {
            const auto point = queue[index];
            seedDepthSum[static_cast<size_t>(seed)] += out.depthM.at<float>(point);
            ++seedDepthCount[static_cast<size_t>(seed)];
            for (const auto delta : Neighbors) {
                const auto neighbor = point + delta;
                if (!inside(size, neighbor.x, neighbor.y) || depthSeeds.at<int>(neighbor) ||
                    out.depthState.at<uint8_t>(neighbor) != 1 ||
                    colorLabels.at<int>(neighbor) != colorLabels.at<int>(point) ||
                    !depthContinuous(out, c, point, neighbor)) continue;
                depthSeeds.at<int>(neighbor) = seed;
                queue.push_back(neighbor);
            }
        }
    }

    std::map<std::pair<int, int>, int> depthSeedBoundary;
    for (int y = 0; y < size.height; ++y) for (int x = 0; x < size.width; ++x) {
        const cv::Point point(x, y);
        for (const cv::Point delta : {cv::Point(1, 0), cv::Point(0, 1)}) {
            const auto neighbor = point + delta;
            if (!inside(size, neighbor.x, neighbor.y) || colorLabels.at<int>(point) != colorLabels.at<int>(neighbor)) continue;
            int first = depthSeeds.at<int>(point), second = depthSeeds.at<int>(neighbor);
            if (!first || !second || first == second) continue;
            if (first > second) std::swap(first, second);
            ++depthSeedBoundary[{first, second}];
        }
    }

    DisjointSet sets(seedCount);
    int depthMergesAcrossColor = 0, depthMergesAcrossMissing = 0, ignoredSmallDepthSeeds = 0;
    int retainedUnsupportedDepthSeeds = 0, forcedIncompatibleNoiseMerges = 0;
    for (int color = 1; color <= colorCount; ++color) {
        std::set<int> unique;
        for (const auto point : colorPixels[static_cast<size_t>(color)]) {
            const int seed = depthSeeds.at<int>(point);
            if (seed) unique.insert(seed);
        }
        std::vector<int> supported;
        for (const int seed : unique) {
            if (seedDepthCount[static_cast<size_t>(seed)] >= c.depthSplitMinSupportPixels) supported.push_back(seed);
        }
        if (supported.empty() && !unique.empty()) {
            supported.push_back(*std::max_element(unique.begin(), unique.end(), [&](int first, int second) {
                return seedDepthCount[static_cast<size_t>(first)] < seedDepthCount[static_cast<size_t>(second)];
            }));
        }
        const auto meanDepth = [&](int seed) {
            return seedDepthSum[static_cast<size_t>(seed)] / seedDepthCount[static_cast<size_t>(seed)];
        };
        for (const int seed : unique) {
            if (std::find(supported.begin(), supported.end(), seed) != supported.end()) continue;
            int nearest = 0, bestBoundary = 0, fallback = 0, fallbackBoundary = 0;
            for (const int candidate : supported) {
                const auto roots = std::minmax(seed, candidate);
                const auto found = depthSeedBoundary.find({roots.first, roots.second});
                const int boundary = found == depthSeedBoundary.end() ? 0 : found->second;
                if (boundary > fallbackBoundary) {
                    fallback = candidate;
                    fallbackBoundary = boundary;
                }
                const double first = meanDepth(candidate), second = meanDepth(seed);
                const bool compatible = std::abs(first - second) <=
                    c.depthGapM + c.depthRelativeGap * static_cast<float>(std::min(first, second));
                if (!compatible) continue;
                if (boundary > bestBoundary || (boundary == bestBoundary && boundary > 0 &&
                    std::abs(first - second) < std::abs(meanDepth(nearest) - second))) {
                    nearest = candidate;
                    bestBoundary = boundary;
                }
            }
            if (!nearest && seedDepthCount[static_cast<size_t>(seed)] <= c.depthNoiseMaxPixels && fallback) {
                nearest = fallback;
                ++forcedIncompatibleNoiseMerges;
            }
            if (nearest) {
                ++ignoredSmallDepthSeeds;
                sets.merge(seed, nearest);
            } else {
                ++retainedUnsupportedDepthSeeds;
            }
        }
        std::sort(supported.begin(), supported.end(), [&](int first, int second) {
            return seedDepthSum[static_cast<size_t>(first)] / seedDepthCount[static_cast<size_t>(first)] <
                seedDepthSum[static_cast<size_t>(second)] / seedDepthCount[static_cast<size_t>(second)];
        });
        size_t begin = 0;
        while (c.mergeCompatibleDepthSeedsAcrossMissing && begin < supported.size()) {
            const double reference = seedDepthSum[static_cast<size_t>(supported[begin])] / seedDepthCount[static_cast<size_t>(supported[begin])];
            size_t end = begin + 1;
            while (end < supported.size()) {
                const double value = seedDepthSum[static_cast<size_t>(supported[end])] / seedDepthCount[static_cast<size_t>(supported[end])];
                if (value - reference > c.depthGapM + c.depthRelativeGap * static_cast<float>(std::min(reference, value))) break;
                if (sets.merge(supported[begin], supported[end])) ++depthMergesAcrossMissing;
                ++end;
            }
            begin = end;
        }
    }
    std::map<std::pair<int, int>, int> continuousCrossColorBoundary;
    for (int y = 0; y < size.height; ++y) for (int x = 0; x < size.width; ++x) {
        const cv::Point point(x, y);
        for (const cv::Point delta : {cv::Point(1, 0), cv::Point(0, 1)}) {
            const auto neighbor = point + delta;
            if (!inside(size, neighbor.x, neighbor.y) || colorLabels.at<int>(point) == colorLabels.at<int>(neighbor) ||
                !depthContinuous(out, c, point, neighbor)) continue;
            int first = sets.find(depthSeeds.at<int>(point)), second = sets.find(depthSeeds.at<int>(neighbor));
            if (first == second) continue;
            if (first > second) std::swap(first, second);
            ++continuousCrossColorBoundary[{first, second}];
        }
    }
    int rejectedShortCrossColorBoundaries = 0;
    for (const auto& [roots, support] : continuousCrossColorBoundary) {
        if (support < c.crossColorMergeMinBoundaryPixels) {
            ++rejectedShortCrossColorBoundaries;
            continue;
        }
        if (sets.merge(roots.first, roots.second)) ++depthMergesAcrossColor;
    }

    out.labels = cv::Mat::zeros(size, CV_32S);
    out.ownership = cv::Mat::zeros(size, CV_8U);
    for (int y = 0; y < size.height; ++y) for (int x = 0; x < size.width; ++x) {
        const int seed = depthSeeds.at<int>(y, x);
        if (!seed) continue;
        out.labels.at<int>(y, x) = sets.find(seed);
        out.ownership.at<uint8_t>(y, x) = out.depthState.at<uint8_t>(y, x) == 1 ? 1 : 2;
    }

    int nextLabel = seedCount + 1;
    std::vector<std::vector<cv::Point>> imageComponents(static_cast<size_t>(nextLabel));
    for (int color = 1; color <= colorCount; ++color) {
        std::set<int> roots;
        for (const auto point : colorPixels[static_cast<size_t>(color)]) {
            const int label = out.labels.at<int>(point);
            if (label) roots.insert(label);
        }
        if (roots.size() == 1) {
            const int root = *roots.begin();
            for (const auto point : colorPixels[static_cast<size_t>(color)]) {
                if (out.labels.at<int>(point)) continue;
                out.labels.at<int>(point) = root;
                out.ownership.at<uint8_t>(point) = 3;
            }
            continue;
        }
        for (const auto start : colorPixels[static_cast<size_t>(color)]) {
            if (out.labels.at<int>(start)) continue;
            require(nextLabel <= MaxClusters, "resource_limit", "图像区域数量超限");
            const int imageLabel = nextLabel++;
            imageComponents.emplace_back();
            queue.clear(); queue.push_back(start); out.labels.at<int>(start) = imageLabel;
            for (size_t index = 0; index < queue.size(); ++index) {
                const auto point = queue[index];
                imageComponents[static_cast<size_t>(imageLabel)].push_back(point);
                out.ownership.at<uint8_t>(point) = 2;
                for (const auto delta : Neighbors) {
                    const auto neighbor = point + delta;
                    if (!inside(size, neighbor.x, neighbor.y) || out.labels.at<int>(neighbor) ||
                        colorLabels.at<int>(neighbor) != color) continue;
                    out.labels.at<int>(neighbor) = imageLabel;
                    queue.push_back(neighbor);
                }
            }
        }
    }

    std::vector<uint8_t> hasCurrentDepth(static_cast<size_t>(nextLabel), 0);
    for (int y = 0; y < size.height; ++y) for (int x = 0; x < size.width; ++x)
        if (out.depthState.at<uint8_t>(y, x) == 1) hasCurrentDepth[static_cast<size_t>(out.labels.at<int>(y, x))] = 1;
    int enclosedInheritedPixels = 0;
    for (int imageLabel = seedCount + 1; imageLabel < nextLabel; ++imageLabel) {
        const auto& pixels = imageComponents[static_cast<size_t>(imageLabel)];
        if (pixels.empty()) continue;
        bool touchesBorder = false;
        std::set<int> surrounding;
        for (const auto point : pixels) {
            touchesBorder |= point.x == 0 || point.y == 0 || point.x + 1 == size.width || point.y + 1 == size.height;
            for (const auto delta : Neighbors) {
                const auto neighbor = point + delta;
                if (!inside(size, neighbor.x, neighbor.y)) continue;
                const int label = out.labels.at<int>(neighbor);
                if (label != imageLabel && label < static_cast<int>(hasCurrentDepth.size()) && hasCurrentDepth[static_cast<size_t>(label)])
                    surrounding.insert(label);
            }
        }
        if (touchesBorder || surrounding.size() != 1) continue;
        const int inherited = *surrounding.begin();
        for (const auto point : pixels) {
            out.labels.at<int>(point) = inherited;
            out.ownership.at<uint8_t>(point) = 3;
            ++enclosedInheritedPixels;
        }
    }

    std::vector<int> remap(static_cast<size_t>(nextLabel), 0);
    int canonical = 0;
    for (int y = 0; y < size.height; ++y) for (int x = 0; x < size.width; ++x) {
        int& label = out.labels.at<int>(y, x);
        if (!remap[static_cast<size_t>(label)]) remap[static_cast<size_t>(label)] = ++canonical;
        label = remap[static_cast<size_t>(label)];
    }
    out.metrics["簇数量"] = canonical;
    out.metrics["聚簇模式"] = "轮廓主导";
    out.metrics["彩图初始区域数"] = colorCount;
    out.metrics["当前深度种子区域数"] = seedCount;
    out.metrics["跨颜色连续深度合并数"] = depthMergesAcrossColor;
    out.metrics["跨缺测相容深度合并数"] = depthMergesAcrossMissing;
    out.metrics["允许同彩图区域跨缺测全局合并"] = c.mergeCompatibleDepthSeedsAcrossMissing;
    out.metrics["深度拆分忽略小种子数"] = ignoredSmallDepthSeeds;
    out.metrics["深度拆分无局部邻接保留小种子数"] = retainedUnsupportedDepthSeeds;
    out.metrics["强制吸收不相容深度噪点数"] = forcedIncompatibleNoiseMerges;
    out.metrics["跨颜色连续边界候选数"] = continuousCrossColorBoundary.size();
    out.metrics["跨颜色短边界拒绝合并数"] = rejectedShortCrossColorBoundaries;
    out.metrics["封闭缺深度继承像素数"] = enclosedInheritedPixels;
}

void fillDepth(const cv::Mat& lab, Observation& out, const ProcessingConfig& c) {
    const auto size = lab.size();
    out.filledM = cv::Mat::zeros(size, CV_32F);
    out.fillState = cv::Mat::zeros(size, CV_8U);
    int filled = 0, refusedSize = 0, refusedEvidence = 0;
    cv::Mat seen = cv::Mat::zeros(size, CV_8U);
    std::vector<cv::Point> queue;
    for (int y = 0; c.fillEnabled && y < size.height; ++y) for (int x = 0; x < size.width; ++x) {
        if (out.depthState.at<uint8_t>(y, x) || seen.at<uint8_t>(y, x)) continue;
        queue.clear(); queue.emplace_back(x, y); seen.at<uint8_t>(y, x) = 1;
        bool border = false;
        for (size_t i = 0; i < queue.size(); ++i) for (const auto d : Neighbors) {
            const auto q = queue[i] + d;
            if (!inside(size, q.x, q.y)) { border = true; continue; }
            if (!seen.at<uint8_t>(q) && !out.depthState.at<uint8_t>(q)) {
                seen.at<uint8_t>(q) = 1; queue.push_back(q);
            }
        }
        if (border || queue.size() > static_cast<size_t>(c.fillMaxPixels)) { refusedSize += static_cast<int>(queue.size()); continue; }
        for (const auto p : queue) {
            double sum = 0, weights = 0;
            float lo = std::numeric_limits<float>::max(), hi = 0;
            int count = 0;
            bool conflictingNeighbor = false;
            for (int dy = -1; dy <= 1; ++dy) for (int dx = -1; dx <= 1; ++dx) {
                const cv::Point q = p + cv::Point(dx, dy);
                if ((!dx && !dy) || !inside(size, q.x, q.y)) continue;
                if (out.depthState.at<uint8_t>(q) != 1) continue;
                // A depth-supported foreign label is a boundary veto, even when its color is similar.
                if (out.labels.at<int>(q) != out.labels.at<int>(p)) { conflictingNeighbor = true; continue; }
                const float delta = deltaSquared(lab, p, q);
                if (delta > c.fillColorDelta * c.fillColorDelta) continue;
                const float depth = out.depthM.at<float>(q);
                const double weight = std::exp(-0.5 * (dx * dx + dy * dy)) *
                    std::exp(-delta / (2.0 * c.fillColorDelta * c.fillColorDelta));
                sum += weight * depth; weights += weight;
                lo = std::min(lo, depth); hi = std::max(hi, depth); ++count;
            }
            if (conflictingNeighbor || count < c.fillMinSamples || hi - lo > c.fillDepthSpreadM || weights <= 0) {
                ++refusedEvidence; continue;
            }
            out.filledM.at<float>(p) = static_cast<float>(sum / weights);
            out.fillState.at<uint8_t>(p) = 1; ++filled;
        }
    }
    out.metrics["插值像素数"] = filled;
    out.metrics["面积或边界拒绝插值像素数"] = refusedSize;
    out.metrics["证据不足拒绝插值像素数"] = refusedEvidence;
}

void contours(Observation& out) {
    struct Accumulator {
        int minX = INT_MAX, minY = INT_MAX, maxX = -1, maxY = -1;
        int count = 0, measured = 0, rejected = 0, filled = 0;
    };
    const int count = out.metrics.at("簇数量").get<int>();
    std::vector<Accumulator> stats(static_cast<size_t>(count + 1));
    for (int y = 0; y < out.labels.rows; ++y) for (int x = 0; x < out.labels.cols; ++x) {
        auto& a = stats[out.labels.at<int>(y, x)];
        a.minX = std::min(a.minX, x); a.maxX = std::max(a.maxX, x);
        a.minY = std::min(a.minY, y); a.maxY = std::max(a.maxY, y);
        ++a.count; a.measured += out.depthState.at<uint8_t>(y, x) == 1;
        a.rejected += out.depthState.at<uint8_t>(y, x) == 2;
        a.filled += out.fillState.at<uint8_t>(y, x) == 1;
    }
    out.clusters = Json::array();
    int64_t examinedPixels = 0;
    for (int id = 1; id <= count; ++id) {
        const auto& a = stats[id];
        const cv::Rect box(a.minX, a.minY, a.maxX - a.minX + 1, a.maxY - a.minY + 1);
        examinedPixels += box.area();
        require(examinedPixels <= int64_t(out.labels.total()) * 16, "resource_limit", "轮廓扫描预算超限");
        cv::Mat mask = out.labels(box) == id;
        std::vector<std::vector<cv::Point>> paths;
        std::vector<cv::Vec4i> hierarchy;
        cv::findContours(mask, paths, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_NONE, box.tl());
        Json rings = Json::array();
        for (size_t index = 0; index < paths.size(); ++index) {
            int depth = 0;
            for (int parent = hierarchy[index][3]; parent >= 0; parent = hierarchy[parent][3]) ++depth;
            const size_t start = out.contourPoints.size() / 3;
            for (const auto p : paths[index]) {
                int reason = 0;
                for (const auto d : Neighbors) {
                    const auto q = p + d;
                    if (!inside(out.labels.size(), q.x, q.y)) reason |= 4;
                    else if (out.labels.at<int>(q) != id) {
                        reason |= 1;
                        if (!out.depthState.at<uint8_t>(q)) reason |= 2;
                    }
                }
                out.contourPoints.insert(out.contourPoints.end(), {p.x, p.y, reason});
            }
            require(out.contourPoints.size() <= out.labels.total() * 12, "resource_limit", "轮廓点预算超限");
            rings.push_back({{"起始点", start}, {"点数", paths[index].size()}, {"父轮廓索引", hierarchy[index][3]},
                {"内环", depth % 2 == 1}, {"闭合区域边界", true}, {"物理孔洞确认", false}});
        }
        out.clusters.push_back({{"本帧簇编号", id}, {"范围XYWH", {box.x, box.y, box.width, box.height}},
            {"像素数", a.count}, {"当前可用深度像素数", a.measured}, {"当前范围外深度像素数", a.rejected},
            {"补全像素数", a.filled}, {"未解决深度像素数", a.count - a.measured - a.rejected - a.filled},
            {"观察模式", a.measured ? "深度支撑区域候选" : "图像区域候选"}, {"轮廓", rings}});
    }
}
}

Observation process(const RawFrame& frame, const ProcessingConfig& config) {
    validateFrame(frame);
    Observation result;
    result.metrics = Json::object();
    const auto start = std::chrono::steady_clock::now();
    alignDepth(frame, result, config);
    cv::Mat floatBgr, lab;
    frame.colorBgr.convertTo(floatBgr, CV_32F, 1.0 / 255.0);
    cv::cvtColor(floatBgr, lab, cv::COLOR_BGR2Lab);
    if (config.clusteringMode == "轮廓主导") partitionContourFirst(lab, result, config);
    else partitionDepthFirst(lab, result, config);
    fillDepth(lab, result, config);
    contours(result);
    result.metrics["处理毫秒"] = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - start).count();
    result.metrics["像素账本记录数"] = frame.colorBgr.total();
    result.metrics["当前可用深度像素数"] = cv::countNonZero(result.depthState == 1);
    result.metrics["当前范围外深度像素数"] = cv::countNonZero(result.depthState == 2);
    result.metrics["物理归属正确率"] = nullptr;
    return result;
}

Json publish(const fs::path& root, const std::string& session, uint64_t sequence,
             const RawFrame& frame, const Observation& out, const ProcessingConfig& config, uint64_t revision,
             const Json& control, size_t byteLimit) {
    static_assert(std::endian::native == std::endian::little);
    static_assert(sizeof(float) == 4 && sizeof(int) == 4);
    const auto base = root / session;
    fs::create_directories(base);
    const auto target = base / ("frame_" + std::to_string(sequence));
    const auto staging = base / ("pending_" + std::to_string(sequence));
    require(!fs::exists(target) && !fs::exists(staging), "publication_conflict", "禁止覆盖已存在观察材料");
    fs::create_directory(staging);
    Json materials = Json::object();
    size_t total = 0;
    auto save = [&](const std::string& key, const std::string& name, const std::vector<uint8_t>& bytes,
                    const std::string& type, const Json& shape) {
        total += bytes.size();
        require(total <= std::min(MaxPacketBytes, byteLimit), "resource_limit", "观察包超过字节预算");
        std::ofstream file(staging / name, std::ios::binary | std::ios::trunc);
        require(file.is_open(), "io_error", "无法创建数组材料");
        file.write(reinterpret_cast<const char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
        file.close(); require(!file.fail(), "io_error", "数组材料写入失败");
        materials[key] = {{"文件", name}, {"编码", type}, {"形状", shape}, {"字节数", bytes.size()}, {"SHA256", sha256(bytes)}};
    };
    auto saveMat = [&](const std::string& key, const std::string& name, const cv::Mat& value, const std::string& type) {
        const auto m = value.isContinuous() ? value : value.clone();
        const size_t bytes = m.total() * m.elemSize();
        save(key, name, std::vector<uint8_t>(m.data, m.data + bytes), type, {m.rows, m.cols});
    };
    std::vector<uint8_t> encoded;
    require(cv::imencode(".png", frame.colorBgr, encoded), "io_error", "彩图编码失败");
    save("颜色图", "color.png", encoded, "PNG_RGB8", {frame.colorBgr.rows, frame.colorBgr.cols, 3});
    require(cv::imencode(".png", frame.depth16, encoded), "io_error", "原始深度编码失败");
    save("原始深度", "source_depth.png", encoded, "PNG_U16", {frame.depth16.rows, frame.depth16.cols});
    saveMat("簇归属图", "labels.bin", out.labels, "uint32_le");
    saveMat("归属状态", "ownership.bin", out.ownership, "uint8");
    saveMat("当前观测深度米", "depth_m.bin", out.depthM, "float32_le");
    saveMat("当前深度状态", "depth_state.bin", out.depthState, "uint8");
    saveMat("源深度像素索引", "depth_source_index.bin", out.depthSourceIndex, "int32_le");
    saveMat("补全深度米", "filled_m.bin", out.filledM, "float32_le");
    saveMat("补全状态", "fill_state.bin", out.fillState, "uint8");
    const auto* contourBytes = reinterpret_cast<const uint8_t*>(out.contourPoints.data());
    save("轮廓点", "contours.bin", std::vector<uint8_t>(contourBytes, contourBytes + out.contourPoints.size() * sizeof(int32_t)),
         "int32_le", {out.contourPoints.size() / 3, 3});
    Json manifest = {{"格式", "PCS.Observation/1"}, {"发布状态", "完整"}, {"会话标识", session},
        {"输出序号", std::to_string(sequence)}, {"源信息", frame.source}, {"输出Unix毫秒", nowMs()},
        {"配置版本", std::to_string(revision)}, {"控制来源", control},
        {"处理算法", config.clusteringMode == "轮廓主导" ? "contour-owner-depth-constraint/1" : "depth-anchor-color-owner/1"},
        {"RealSenseSDK版本", RS2_API_VERSION_STR}, {"OpenCV版本", CV_VERSION},
        {"配准算法", "source-center-project-zbuffer/1"}, {"处理配置", config.json()},
        {"图像尺寸WH", {frame.colorBgr.cols, frame.colorBgr.rows}},
        {"标定", {{"彩图内参", intrinsicsJson(frame.colorIntrinsics)}, {"深度内参", intrinsicsJson(frame.depthIntrinsics)},
            {"深度到彩图外参", extrinsicsJson(frame.depthToColor)}, {"源深度单位米", frame.depthScale},
            {"源深度无效码", {0, 65535}}, {"输出深度含义", "彩图相机光轴Z，米"},
            {"坐标轴", "X右Y下Z前"}, {"像素坐标", "左上像素中心为(0,0)，整数u向右v向下"}}},
        {"状态编码", {{"归属", {{"0", "未解决"}, {"1", "当前深度支撑候选"}, {"2", "图像候选"}, {"3", "单一周边锚点继承候选"}}},
            {"深度", {{"0", "缺测"}, {"1", "配置范围内观测，不代表校准精度保证"}, {"2", "配置范围外观测"}}},
            {"补全", {{"0", "未补全"}, {"1", "本帧颜色引导插值"}}},
            {"轮廓原因位", {{"1", "算法区域分界"}, {"2", "相邻深度缺测"}, {"4", "视野截断"}}}}},
        {"轮廓格式", "OpenCV像素中心链，8方向相邻，RETR_TREE，CHAIN_APPROX_NONE；归属图为像素成员权威"},
        {"零深度含义", "无效槽位，必须同时读取对应状态图"},
        {"体素", false}, {"已确认存在身份", false}, {"材料", materials}, {"簇目录", out.clusters}, {"指标", out.metrics}};
    const std::string manifestText = manifest.dump(2) + "\n";
    require(total + manifestText.size() <= std::min(MaxPacketBytes, byteLimit), "resource_limit", "元数据导致观察包超限");
    writeJson(staging / "frame.json", manifest);
    // Only a complete directory is published. Failures leave an explicitly unpublished pending directory.
    fs::rename(staging, target);
    return {{"材料路径", utf8(fs::absolute(target / "frame.json"))}, {"源帧号", frame.source.at("源帧号")},
        {"输出序号", std::to_string(sequence)}, {"会话标识", session}, {"配置版本", std::to_string(revision)},
        {"清单SHA256", sha256(std::vector<uint8_t>(manifestText.begin(), manifestText.end()))},
        {"字节数", total + manifestText.size()}, {"来源类型", frame.source.at("类型")}, {"指标", out.metrics}};
}
}
