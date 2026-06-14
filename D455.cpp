#include <librealsense2/rs2.hpp>

#include <opencv2/core.hpp>
#include <opencv2/core/utils/logger.hpp>
#include <opencv2/dnn.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/videoio.hpp>

#include <pcl/point_cloud.h>
#include <pcl/PointIndices.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdint>
#include <ctime>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <limits>
#include <map>
#include <sstream>
#include <stdexcept>
#include <string>
#include <vector>

namespace
{
constexpr const char* kRawWindow = "D455 raw frame";
constexpr const char* kSegmentedWindow = "D455 stable contour information";
constexpr const char* kMosaicWindow = "D455 stable contour color mosaic";
constexpr const char* kOutsideMosaicWindow = "D455 black-area color view";
constexpr const char* kBoundaryDiagnosticsWindow = "D455 IR-D boundary diagnostics";
constexpr const char* kIndoorPlaneWindow = "D455 indoor plane diagnostics";
constexpr const char* kStableContourWindow = "D455 stable contour stability";

struct SegmentationConfig
{
    int minDepthMm = 250;
    int maxDepthMm = 3500;
    int minAreaPixels = 900;
    int maxAreaPercent = 24;
    int maxRoiAreaPercent = 55;
    int maxBorderAreaPercent = 8;
    int foregroundKeepDepthMm = 1800;
    int foregroundMaxAreaPercent = 70;
    int foregroundMaxRoiAreaPercent = 90;
    int maxMaterialAreaPercent = 35;
    int maxMaterialRoiAreaPercent = 65;
    int depthSliceMm = 300;
    int maxMaterials = 24;
    int minEdgePixels = 18;
    int depthCannyLow = 10;
    int depthCannyHigh = 35;
    int groupGapPixels = 24;
    int groupDepthGapMm = 450;
    int morphKernelSize = 5;
    int contourLinePixels = 1;
    int colorCannyLow = 70;
    int colorCannyHigh = 160;
    int infraredCannyLow = 45;
    int infraredCannyHigh = 130;
    int colorDepthSupportPixels = 5;
    int contourDepthConfirmRadiusPixels = 4;
    int contourDepthConfirmMinRangeMm = 25;
    int contourDepthConfirmMinValidPixels = 8;
    int depthHoleEdgePixels = 3;
    int cueMinReliableEdgePixels = 8;
    int cueMinConfirmedRgbEdgePixels = 6;
    int cueMaxTextureOnlyPercent = 75;
    int cueStrongAnchorMultiplierPercent = 180;
    int indoorPlaneFrameInterval = 10;
    int indoorPlaneSampleStepPixels = 10;
    int indoorPlaneNormalNeighborPixels = 10;
    int indoorPlaneMinAreaPercent = 4;
    int indoorPlaneForegroundDilatePixels = 9;
    int indoorPlaneHorizontalNormalMinPercent = 65;
    int indoorPlaneVerticalNormalMaxPercent = 65;
    int indoorPlaneCeilingBandPercent = 30;
    int splitBoundaryPixels = 3;
    int spatialClusterGapMm = 120;
    int trackConfirmFrames = 3;
    int trackMissFrames = 5;
    int trackCenterGapPixels = 48;
    int trackDepthGapMm = 300;
    int trackIouPercent = 20;
    int trackSmoothPercent = 65;
    int pclClusterToleranceMm = 35;
    int pclMinClusterPoints = 80;
    int pclMaxClusterPoints = 200000;
    int pclSampleStepPixels = 3;
    int pclMaxInputPoints = 7000;
    int pclFrameInterval = 2;
    int colorContourCompletionPaddingPixels = 12;
    int colorContourCompletionMinIouPercent = 72;
    int colorContourCompletionMaxAreaDeltaPercent = 24;
    int colorContourCompletionMaxCenterShiftPixels = 16;
    int colorContourCompletionClosePixels = 3;
    int colorContourCompletionOtherGuardPixels = 7;
    int colorContourCompletionMaxOtherOverlapPixels = 4;
    bool showLabels = false;
    bool showCenters = false;
    bool showRegionContours = false;
    bool boundarySplit = true;
    bool colorSplit = true;
    bool contourDepthConfirmSplit = true;
    bool depthHoleSplit = false;
    bool cueSelection = true;
    bool colorEdges = false;
    bool infraredSegmentation = true;
    bool boundaryDiagnostics = false;
    bool indoorPlaneDiagnostics = false;
    bool spatialClusterCheck = true;
    bool historyTracking = true;
    bool pclClustering = true;
    bool colorContourCompletion = true;
    bool showPartNumbers = false;
    double contourApproxRatio = 0.0015;
};

struct ObservationMaterial
{
    uint64_t sourceFrameId = 0;
    uint64_t observationId = 0;
    cv::Rect roi;
    cv::Point center;
    int pixelCount = 0;
    int depthMinMm = 0;
    int depthMaxMm = 0;
    int observedDepthMinMm = 0;
    int observedDepthMaxMm = 0;
    int meanDepthMm = 0;
    int groupId = 0;
    int pclClusterId = 0;
    double contourArea = 0.0;
    bool hasPointCloudBounds = false;
    cv::Point3f minPointMeters;
    cv::Point3f maxPointMeters;
    std::vector<cv::Point> contour;
};

struct ObservationGroup
{
    int groupId = 0;
    cv::Point center;
    cv::Rect roi;
    int pixelCount = 0;
    std::vector<size_t> materialIndexes;
};

struct BoundaryAnalysis
{
    cv::Mat validMask;
    cv::Mat depthStepEdges;
    cv::Mat depthHoleEdges;
    cv::Mat colorEdges;
    cv::Mat depthSupportedColorEdges;
    cv::Mat depthConfirmedColorEdges;
    cv::Mat rawBoundaryMask;
    cv::Mat splitBoundaryMask;
    bool edgeSourceIsInfrared = false;
};

struct CandidateCueEvidence
{
    int contourEdgePixels = 0;
    int depthEdgePixels = 0;
    int supportedRgbEdgePixels = 0;
    int confirmedRgbEdgePixels = 0;
    int rgbEdgePixels = 0;
    int textureOnlyRgbEdgePixels = 0;
    int anchorCount = 0;
    double textureOnlyPercent = 0.0;
};

struct CueSelectionSummary
{
    int inputCandidates = 0;
    int acceptedCandidates = 0;
    int rejectedTextureCandidates = 0;
    int acceptedByDepth = 0;
    int acceptedByRgb = 0;
    int acceptedByAnchor = 0;
    int acceptedByFallback = 0;
    double textureOnlyPercentSum = 0.0;
};

struct IndoorPlaneAnalysis
{
    cv::Mat structuralPlaneMask;
    cv::Mat wallMask;
    cv::Mat ceilingMask;
    cv::Mat supportMask;
    int structuralPlanePixels = 0;
    int wallPixels = 0;
    int ceilingPixels = 0;
    int supportPixels = 0;
    int components = 0;
    bool reusedFromCache = false;
};

struct ColorContourCompletionStats
{
    int inputContours = 0;
    int adoptedContours = 0;
    int rejectedContours = 0;
};

struct FrameTimingStats
{
    double depthPostMs = 0.0;
    double frameConvertMs = 0.0;
    double grayPrepareMs = 0.0;
    double anchorMs = 0.0;
    double boundaryMs = 0.0;
    double extractMs = 0.0;
    double pclMs = 0.0;
    double calibrateMs = 0.0;
    double cueMs = 0.0;
    double trackerMs = 0.0;
    double supportMs = 0.0;
    double renderMs = 0.0;
    double completionMs = 0.0;
    double diagnosticsMs = 0.0;
    double displayMs = 0.0;
    double recordMs = 0.0;
};

struct RgbDepthAccuracyConfig
{
    int sampleFrames = 20;
    int warmupFrames = 30;
    int evalStepPixels = 1;
    int anchorStepPixels = 4;
    int anchorNeighborhoodPixels = 2;
    int anchorMaxNeighborRangeMm = 35;
    int anchorMaxRawFilteredGapMm = 60;
    int anchorEdgeDilatePixels = 3;
    int anchorHoldoutPercent = 30;
    int stableContourMinAnchors = 16;
    int stableContourTopCount = 12;
    bool fitEachFrame = false;
    bool anchorOnly = false;
    bool anchorCorrection = false;
    bool stableContourTest = false;
    bool stableContourShow = false;
    bool stableContourVideo = false;
    std::string stableContourSavePath = "stable_contours_last.png";
    std::string onnxPath;
};

struct VideoRecordingConfig
{
    bool enabled = false;
    int fps = 30;
    int everyN = 1;
    int scalePercent = 100;
    std::string outputPath;
};

struct AcceptanceMetricsConfig
{
    bool enabled = false;
    bool recordVideo = true;
    std::string label = "acceptance_baseline";
    std::string csvPath;
};

struct InverseDepthCalibration
{
    double intercept = 0.0;
    double slope = 0.0;
    int sampleCount = 0;
    bool valid = false;
};

class DepthPostProcessor
{
public:
    DepthPostProcessor()
        : depthToDisparity_(true),
          disparityToDepth_(false)
    {
        spatial_.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.50f);
        spatial_.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
        spatial_.set_option(RS2_OPTION_HOLES_FILL, 2.0f);
        temporal_.set_option(RS2_OPTION_FILTER_SMOOTH_ALPHA, 0.40f);
        temporal_.set_option(RS2_OPTION_FILTER_SMOOTH_DELTA, 20.0f);
    }

    rs2::depth_frame process(const rs2::depth_frame& depthFrame)
    {
        rs2::frame filtered = depthFrame;
        filtered = depthToDisparity_.process(filtered);
        filtered = spatial_.process(filtered);
        filtered = temporal_.process(filtered);
        filtered = disparityToDepth_.process(filtered);
        filtered = holeFilling_.process(filtered);
        return filtered.as<rs2::depth_frame>();
    }

private:
    rs2::disparity_transform depthToDisparity_;
    rs2::spatial_filter spatial_;
    rs2::temporal_filter temporal_;
    rs2::disparity_transform disparityToDepth_;
    rs2::hole_filling_filter holeFilling_;
};

bool parseIntOption(const std::string& arg, const std::string& prefix, int& value)
{
    if (arg.rfind(prefix, 0) != 0)
    {
        return false;
    }

    value = std::stoi(arg.substr(prefix.size()));
    return true;
}

bool parseStringOption(const std::string& arg, const std::string& prefix, std::string& value)
{
    if (arg.rfind(prefix, 0) != 0)
    {
        return false;
    }

    value = arg.substr(prefix.size());
    return true;
}

SegmentationConfig parseConfig(int argc, char** argv)
{
    SegmentationConfig config;

    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (parseIntOption(arg, "--min-depth-mm=", config.minDepthMm) ||
            parseIntOption(arg, "--max-depth-mm=", config.maxDepthMm) ||
            parseIntOption(arg, "--min-area-px=", config.minAreaPixels) ||
            parseIntOption(arg, "--max-area-percent=", config.maxAreaPercent) ||
            parseIntOption(arg, "--max-roi-area-percent=", config.maxRoiAreaPercent) ||
            parseIntOption(arg, "--max-border-area-percent=", config.maxBorderAreaPercent) ||
            parseIntOption(arg, "--foreground-keep-depth-mm=", config.foregroundKeepDepthMm) ||
            parseIntOption(arg, "--foreground-max-area-percent=", config.foregroundMaxAreaPercent) ||
            parseIntOption(arg, "--foreground-max-roi-area-percent=", config.foregroundMaxRoiAreaPercent) ||
            parseIntOption(arg, "--max-material-area-percent=", config.maxMaterialAreaPercent) ||
            parseIntOption(arg, "--max-material-roi-area-percent=", config.maxMaterialRoiAreaPercent) ||
            parseIntOption(arg, "--depth-slice-mm=", config.depthSliceMm) ||
            parseIntOption(arg, "--max-materials=", config.maxMaterials) ||
            parseIntOption(arg, "--min-edge-px=", config.minEdgePixels) ||
            parseIntOption(arg, "--depth-canny-low=", config.depthCannyLow) ||
            parseIntOption(arg, "--depth-canny-high=", config.depthCannyHigh) ||
            parseIntOption(arg, "--group-gap-px=", config.groupGapPixels) ||
            parseIntOption(arg, "--group-depth-gap-mm=", config.groupDepthGapMm) ||
            parseIntOption(arg, "--contour-line-px=", config.contourLinePixels) ||
            parseIntOption(arg, "--color-canny-low=", config.colorCannyLow) ||
            parseIntOption(arg, "--color-canny-high=", config.colorCannyHigh) ||
            parseIntOption(arg, "--infrared-canny-low=", config.infraredCannyLow) ||
            parseIntOption(arg, "--infrared-canny-high=", config.infraredCannyHigh) ||
            parseIntOption(arg, "--color-depth-support-px=", config.colorDepthSupportPixels) ||
            parseIntOption(arg, "--contour-depth-confirm-radius-px=", config.contourDepthConfirmRadiusPixels) ||
            parseIntOption(arg, "--contour-depth-confirm-min-range-mm=", config.contourDepthConfirmMinRangeMm) ||
            parseIntOption(arg, "--contour-depth-confirm-min-valid-px=", config.contourDepthConfirmMinValidPixels) ||
            parseIntOption(arg, "--depth-hole-edge-px=", config.depthHoleEdgePixels) ||
            parseIntOption(arg, "--cue-min-reliable-edge-px=", config.cueMinReliableEdgePixels) ||
            parseIntOption(arg, "--cue-min-confirmed-rgb-edge-px=", config.cueMinConfirmedRgbEdgePixels) ||
            parseIntOption(arg, "--cue-min-confirmed-gray-edge-px=", config.cueMinConfirmedRgbEdgePixels) ||
            parseIntOption(arg, "--cue-max-texture-only-percent=", config.cueMaxTextureOnlyPercent) ||
            parseIntOption(arg, "--cue-strong-anchor-multiplier-percent=", config.cueStrongAnchorMultiplierPercent) ||
            parseIntOption(arg, "--indoor-plane-frame-interval=", config.indoorPlaneFrameInterval) ||
            parseIntOption(arg, "--indoor-plane-sample-step-px=", config.indoorPlaneSampleStepPixels) ||
            parseIntOption(arg, "--indoor-plane-normal-neighbor-px=", config.indoorPlaneNormalNeighborPixels) ||
            parseIntOption(arg, "--indoor-plane-min-area-percent=", config.indoorPlaneMinAreaPercent) ||
            parseIntOption(arg, "--indoor-plane-foreground-dilate-px=", config.indoorPlaneForegroundDilatePixels) ||
            parseIntOption(arg, "--indoor-plane-horizontal-normal-min-percent=", config.indoorPlaneHorizontalNormalMinPercent) ||
            parseIntOption(arg, "--indoor-plane-vertical-normal-max-percent=", config.indoorPlaneVerticalNormalMaxPercent) ||
            parseIntOption(arg, "--indoor-plane-ceiling-band-percent=", config.indoorPlaneCeilingBandPercent) ||
            parseIntOption(arg, "--split-boundary-px=", config.splitBoundaryPixels) ||
            parseIntOption(arg, "--spatial-cluster-gap-mm=", config.spatialClusterGapMm) ||
            parseIntOption(arg, "--track-confirm-frames=", config.trackConfirmFrames) ||
            parseIntOption(arg, "--track-miss-frames=", config.trackMissFrames) ||
            parseIntOption(arg, "--track-center-gap-px=", config.trackCenterGapPixels) ||
            parseIntOption(arg, "--track-depth-gap-mm=", config.trackDepthGapMm) ||
            parseIntOption(arg, "--track-iou-percent=", config.trackIouPercent) ||
            parseIntOption(arg, "--track-smooth-percent=", config.trackSmoothPercent) ||
            parseIntOption(arg, "--pcl-cluster-tolerance-mm=", config.pclClusterToleranceMm) ||
            parseIntOption(arg, "--pcl-min-cluster-points=", config.pclMinClusterPoints) ||
            parseIntOption(arg, "--pcl-max-cluster-points=", config.pclMaxClusterPoints) ||
            parseIntOption(arg, "--pcl-sample-step-px=", config.pclSampleStepPixels) ||
            parseIntOption(arg, "--pcl-max-input-points=", config.pclMaxInputPoints) ||
            parseIntOption(arg, "--pcl-frame-interval=", config.pclFrameInterval) ||
            parseIntOption(arg, "--color-contour-completion-padding-px=", config.colorContourCompletionPaddingPixels) ||
            parseIntOption(arg, "--color-contour-completion-min-iou-percent=", config.colorContourCompletionMinIouPercent) ||
            parseIntOption(arg, "--color-contour-completion-max-area-delta-percent=", config.colorContourCompletionMaxAreaDeltaPercent) ||
            parseIntOption(arg, "--color-contour-completion-max-center-shift-px=", config.colorContourCompletionMaxCenterShiftPixels) ||
            parseIntOption(arg, "--color-contour-completion-close-px=", config.colorContourCompletionClosePixels) ||
            parseIntOption(arg, "--color-contour-completion-other-guard-px=", config.colorContourCompletionOtherGuardPixels) ||
            parseIntOption(arg, "--color-contour-completion-max-other-overlap-px=", config.colorContourCompletionMaxOtherOverlapPixels))
        {
            continue;
        }
        if (arg == "--no-color-split")
        {
            config.colorSplit = false;
            continue;
        }
        if (arg == "--no-gray-split")
        {
            config.colorSplit = false;
            continue;
        }
        if (arg == "--no-infrared-segmentation")
        {
            config.infraredSegmentation = false;
            continue;
        }
        if (arg == "--no-contour-depth-confirm-split")
        {
            config.contourDepthConfirmSplit = false;
            continue;
        }
        if (arg == "--depth-hole-split")
        {
            config.depthHoleSplit = true;
            continue;
        }
        if (arg == "--no-depth-hole-split")
        {
            config.depthHoleSplit = false;
            continue;
        }
        if (arg == "--no-cue-selection")
        {
            config.cueSelection = false;
            continue;
        }
        if (arg == "--no-boundary-split")
        {
            config.boundarySplit = false;
            continue;
        }
        if (arg == "--show-regions")
        {
            config.showRegionContours = true;
            continue;
        }
        if (arg == "--color-edges")
        {
            config.colorEdges = true;
            continue;
        }
        if (arg == "--gray-edges")
        {
            config.colorEdges = true;
            continue;
        }
        if (arg == "--boundary-diagnostics")
        {
            config.boundaryDiagnostics = true;
            continue;
        }
        if (arg == "--indoor-plane-diagnostics")
        {
            config.indoorPlaneDiagnostics = true;
            continue;
        }
        if (arg == "--show-part-numbers")
        {
            config.showPartNumbers = true;
            continue;
        }
        if (arg == "--show-labels")
        {
            config.showLabels = true;
            continue;
        }
        if (arg == "--show-centers")
        {
            config.showCenters = true;
            continue;
        }
        if (arg == "--no-spatial-cluster-check")
        {
            config.spatialClusterCheck = false;
            continue;
        }
        if (arg == "--no-history-tracking")
        {
            config.historyTracking = false;
            continue;
        }
        if (arg == "--no-pcl-clustering")
        {
            config.pclClustering = false;
            continue;
        }
        if (arg == "--color-contour-completion")
        {
            config.colorContourCompletion = true;
            continue;
        }
        if (arg == "--no-color-contour-completion")
        {
            config.colorContourCompletion = false;
            continue;
        }
        if (arg == "--probe-only" || arg == "--help" || arg == "-h")
        {
            continue;
        }
        if (arg.rfind("--max-frames=", 0) == 0)
        {
            continue;
        }
        if (arg == "--record-video" ||
            arg == "--no-record-video" ||
            arg.rfind("--record-video=", 0) == 0 ||
            arg.rfind("--record-fps=", 0) == 0 ||
            arg.rfind("--record-every-n=", 0) == 0 ||
            arg.rfind("--record-scale-percent=", 0) == 0 ||
            arg == "--no-display")
        {
            continue;
        }
        if (arg == "--acceptance-baseline" ||
            arg == "--acceptance-no-record" ||
            arg.rfind("--acceptance-label=", 0) == 0 ||
            arg.rfind("--acceptance-csv=", 0) == 0)
        {
            continue;
        }
        if (arg == "--depth-stability-test" ||
            arg.rfind("--depth-stability-frames=", 0) == 0 ||
            arg.rfind("--depth-stability-warmup=", 0) == 0 ||
            arg.rfind("--depth-stability-relative-permille=", 0) == 0)
        {
            continue;
        }
        if (arg == "--rgb-depth-accuracy-test" ||
            arg == "--rgb-depth-fit-each-frame" ||
            arg == "--rgb-depth-anchor-only" ||
            arg == "--rgb-depth-anchor-correction" ||
            arg == "--stable-contour-test" ||
            arg == "--stable-contour-show" ||
            arg == "--stable-contour-video" ||
            arg.rfind("--rgb-depth-frames=", 0) == 0 ||
            arg.rfind("--rgb-depth-warmup=", 0) == 0 ||
            arg.rfind("--rgb-depth-eval-step-px=", 0) == 0 ||
            arg.rfind("--rgb-depth-anchor-step-px=", 0) == 0 ||
            arg.rfind("--rgb-depth-anchor-neighborhood-px=", 0) == 0 ||
            arg.rfind("--rgb-depth-anchor-max-neighbor-range-mm=", 0) == 0 ||
            arg.rfind("--rgb-depth-anchor-max-raw-filtered-gap-mm=", 0) == 0 ||
            arg.rfind("--rgb-depth-anchor-edge-dilate-px=", 0) == 0 ||
            arg.rfind("--rgb-depth-anchor-holdout-percent=", 0) == 0 ||
            arg.rfind("--stable-contour-min-anchors=", 0) == 0 ||
            arg.rfind("--stable-contour-top=", 0) == 0 ||
            arg.rfind("--stable-contour-save=", 0) == 0 ||
            arg.rfind("--rgb-depth-onnx=", 0) == 0)
        {
            continue;
        }

        std::cerr << "Ignored unknown option: " << arg << '\n';
    }

    if (config.minDepthMm < 1)
    {
        config.minDepthMm = 1;
    }
    if (config.maxDepthMm <= config.minDepthMm)
    {
        config.maxDepthMm = config.minDepthMm + 1;
    }
    config.minAreaPixels = std::max(1, config.minAreaPixels);
    config.maxAreaPercent = std::clamp(config.maxAreaPercent, 1, 100);
    config.maxRoiAreaPercent = std::clamp(config.maxRoiAreaPercent, 1, 100);
    config.maxBorderAreaPercent = std::clamp(config.maxBorderAreaPercent, 1, 100);
    config.foregroundKeepDepthMm = std::clamp(config.foregroundKeepDepthMm, config.minDepthMm, config.maxDepthMm);
    config.foregroundMaxAreaPercent = std::clamp(config.foregroundMaxAreaPercent, config.maxAreaPercent, 100);
    config.foregroundMaxRoiAreaPercent = std::clamp(config.foregroundMaxRoiAreaPercent, config.maxRoiAreaPercent, 100);
    config.maxMaterialAreaPercent = std::clamp(config.maxMaterialAreaPercent, 1, 100);
    config.maxMaterialRoiAreaPercent = std::clamp(config.maxMaterialRoiAreaPercent, 1, 100);
    config.depthSliceMm = std::max(25, config.depthSliceMm);
    config.maxMaterials = std::max(1, config.maxMaterials);
    config.minEdgePixels = std::max(1, config.minEdgePixels);
    config.depthCannyLow = std::clamp(config.depthCannyLow, 1, 255);
    config.depthCannyHigh = std::clamp(config.depthCannyHigh, config.depthCannyLow + 1, 255);
    config.groupGapPixels = std::clamp(config.groupGapPixels, 0, 96);
    config.groupDepthGapMm = std::clamp(config.groupDepthGapMm, 0, 2000);
    config.contourLinePixels = std::clamp(config.contourLinePixels, 1, 4);
    config.colorCannyLow = std::clamp(config.colorCannyLow, 1, 255);
    config.colorCannyHigh = std::clamp(config.colorCannyHigh, config.colorCannyLow + 1, 255);
    config.infraredCannyLow = std::clamp(config.infraredCannyLow, 1, 255);
    config.infraredCannyHigh = std::clamp(config.infraredCannyHigh, config.infraredCannyLow + 1, 255);
    config.colorDepthSupportPixels = std::clamp(config.colorDepthSupportPixels, 0, 32);
    config.contourDepthConfirmRadiusPixels = std::clamp(config.contourDepthConfirmRadiusPixels, 1, 16);
    config.contourDepthConfirmMinRangeMm = std::clamp(config.contourDepthConfirmMinRangeMm, 1, 500);
    config.contourDepthConfirmMinValidPixels = std::clamp(config.contourDepthConfirmMinValidPixels, 1, 256);
    config.depthHoleEdgePixels = std::clamp(config.depthHoleEdgePixels, 1, 15);
    config.cueMinReliableEdgePixels = std::clamp(config.cueMinReliableEdgePixels, 0, 512);
    config.cueMinConfirmedRgbEdgePixels = std::clamp(config.cueMinConfirmedRgbEdgePixels, 0, 512);
    config.cueMaxTextureOnlyPercent = std::clamp(config.cueMaxTextureOnlyPercent, 1, 100);
    config.cueStrongAnchorMultiplierPercent = std::clamp(config.cueStrongAnchorMultiplierPercent, 100, 500);
    config.indoorPlaneFrameInterval = std::clamp(config.indoorPlaneFrameInterval, 1, 60);
    config.indoorPlaneSampleStepPixels = std::clamp(config.indoorPlaneSampleStepPixels, 4, 32);
    config.indoorPlaneNormalNeighborPixels = std::clamp(config.indoorPlaneNormalNeighborPixels, 2, 32);
    config.indoorPlaneMinAreaPercent = std::clamp(config.indoorPlaneMinAreaPercent, 1, 60);
    config.indoorPlaneForegroundDilatePixels = std::clamp(config.indoorPlaneForegroundDilatePixels, 0, 63);
    config.indoorPlaneHorizontalNormalMinPercent = std::clamp(config.indoorPlaneHorizontalNormalMinPercent, 1, 100);
    config.indoorPlaneVerticalNormalMaxPercent = std::clamp(config.indoorPlaneVerticalNormalMaxPercent, 0, 100);
    config.indoorPlaneCeilingBandPercent = std::clamp(config.indoorPlaneCeilingBandPercent, 5, 60);
    config.splitBoundaryPixels = std::clamp(config.splitBoundaryPixels, 1, 11);
    config.spatialClusterGapMm = std::clamp(config.spatialClusterGapMm, 0, 2000);
    config.trackConfirmFrames = std::clamp(config.trackConfirmFrames, 1, 30);
    config.trackMissFrames = std::clamp(config.trackMissFrames, 0, 120);
    config.trackCenterGapPixels = std::clamp(config.trackCenterGapPixels, 1, 480);
    config.trackDepthGapMm = std::clamp(config.trackDepthGapMm, 1, 3000);
    config.trackIouPercent = std::clamp(config.trackIouPercent, 0, 100);
    config.trackSmoothPercent = std::clamp(config.trackSmoothPercent, 0, 95);
    config.pclClusterToleranceMm = std::clamp(config.pclClusterToleranceMm, 1, 1000);
    config.pclMinClusterPoints = std::clamp(config.pclMinClusterPoints, 1, 1000000);
    config.pclMaxClusterPoints = std::max(config.pclMinClusterPoints, config.pclMaxClusterPoints);
    config.pclSampleStepPixels = std::clamp(config.pclSampleStepPixels, 1, 8);
    config.pclMaxInputPoints = std::clamp(config.pclMaxInputPoints, config.pclMinClusterPoints, 500000);
    config.pclFrameInterval = std::clamp(config.pclFrameInterval, 1, 30);
    config.colorContourCompletionPaddingPixels = std::clamp(config.colorContourCompletionPaddingPixels, 2, 64);
    config.colorContourCompletionMinIouPercent = std::clamp(config.colorContourCompletionMinIouPercent, 0, 100);
    config.colorContourCompletionMaxAreaDeltaPercent =
        std::clamp(config.colorContourCompletionMaxAreaDeltaPercent, 0, 200);
    config.colorContourCompletionMaxCenterShiftPixels =
        std::clamp(config.colorContourCompletionMaxCenterShiftPixels, 0, 160);
    config.colorContourCompletionClosePixels = std::clamp(config.colorContourCompletionClosePixels, 1, 31);
    config.colorContourCompletionOtherGuardPixels =
        std::clamp(config.colorContourCompletionOtherGuardPixels, 0, 64);
    config.colorContourCompletionMaxOtherOverlapPixels =
        std::clamp(config.colorContourCompletionMaxOtherOverlapPixels, 0, 1000);

    return config;
}

bool hasFlag(int argc, char** argv, const std::string& flag)
{
    for (int i = 1; i < argc; ++i)
    {
        if (argv[i] == flag)
        {
            return true;
        }
    }

    return false;
}

int parseIntOptionOrDefault(int argc, char** argv, const std::string& prefix, int defaultValue)
{
    int value = defaultValue;
    for (int i = 1; i < argc; ++i)
    {
        parseIntOption(argv[i], prefix, value);
    }

    return value;
}

VideoRecordingConfig parseVideoRecordingConfig(int argc, char** argv, bool defaultEnabled = false)
{
    VideoRecordingConfig config;
    config.enabled = defaultEnabled;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--no-record-video")
        {
            config.enabled = false;
            continue;
        }
        if (arg == "--record-video")
        {
            config.enabled = true;
            continue;
        }
        if (parseStringOption(arg, "--record-video=", config.outputPath))
        {
            config.enabled = true;
            continue;
        }
        parseIntOption(arg, "--record-fps=", config.fps);
        parseIntOption(arg, "--record-every-n=", config.everyN);
        parseIntOption(arg, "--record-scale-percent=", config.scalePercent);
    }

    config.fps = std::clamp(config.fps, 1, 120);
    config.everyN = std::clamp(config.everyN, 1, 300);
    config.scalePercent = std::clamp(config.scalePercent, 10, 100);
    return config;
}

AcceptanceMetricsConfig parseAcceptanceMetricsConfig(int argc, char** argv)
{
    AcceptanceMetricsConfig config;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--acceptance-baseline")
        {
            config.enabled = true;
            continue;
        }
        if (arg == "--acceptance-no-record")
        {
            config.recordVideo = false;
            continue;
        }
        if (parseStringOption(arg, "--acceptance-label=", config.label))
        {
            config.enabled = true;
            continue;
        }
        if (parseStringOption(arg, "--acceptance-csv=", config.csvPath))
        {
            config.enabled = true;
            continue;
        }
    }

    if (config.label.empty())
    {
        config.label = "acceptance_baseline";
    }
    for (char& character : config.label)
    {
        const unsigned char value = static_cast<unsigned char>(character);
        if (!std::isalnum(value) && character != '_' && character != '-')
        {
            character = '_';
        }
    }

    return config;
}

RgbDepthAccuracyConfig parseRgbDepthAccuracyConfig(int argc, char** argv)
{
    RgbDepthAccuracyConfig config;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        parseIntOption(arg, "--rgb-depth-frames=", config.sampleFrames);
        parseIntOption(arg, "--rgb-depth-warmup=", config.warmupFrames);
        parseIntOption(arg, "--rgb-depth-eval-step-px=", config.evalStepPixels);
        parseIntOption(arg, "--rgb-depth-anchor-step-px=", config.anchorStepPixels);
        parseIntOption(arg, "--rgb-depth-anchor-neighborhood-px=", config.anchorNeighborhoodPixels);
        parseIntOption(arg, "--rgb-depth-anchor-max-neighbor-range-mm=", config.anchorMaxNeighborRangeMm);
        parseIntOption(arg, "--rgb-depth-anchor-max-raw-filtered-gap-mm=", config.anchorMaxRawFilteredGapMm);
        parseIntOption(arg, "--rgb-depth-anchor-edge-dilate-px=", config.anchorEdgeDilatePixels);
        parseIntOption(arg, "--rgb-depth-anchor-holdout-percent=", config.anchorHoldoutPercent);
        parseIntOption(arg, "--stable-contour-min-anchors=", config.stableContourMinAnchors);
        parseIntOption(arg, "--stable-contour-top=", config.stableContourTopCount);
        parseStringOption(arg, "--stable-contour-save=", config.stableContourSavePath);
        parseStringOption(arg, "--rgb-depth-onnx=", config.onnxPath);
        if (arg == "--rgb-depth-fit-each-frame")
        {
            config.fitEachFrame = true;
        }
        if (arg == "--rgb-depth-anchor-only")
        {
            config.anchorOnly = true;
        }
        if (arg == "--rgb-depth-anchor-correction")
        {
            config.anchorCorrection = true;
        }
        if (arg == "--stable-contour-test")
        {
            config.stableContourTest = true;
        }
        if (arg == "--stable-contour-show")
        {
            config.stableContourShow = true;
        }
        if (arg == "--stable-contour-video")
        {
            config.stableContourTest = true;
            config.stableContourShow = true;
            config.stableContourVideo = true;
        }
    }

    config.sampleFrames = std::clamp(config.sampleFrames, 1, 200);
    config.warmupFrames = std::clamp(config.warmupFrames, 0, 300);
    config.evalStepPixels = std::clamp(config.evalStepPixels, 1, 16);
    config.anchorStepPixels = std::clamp(config.anchorStepPixels, 1, 32);
    config.anchorNeighborhoodPixels = std::clamp(config.anchorNeighborhoodPixels, 1, 8);
    config.anchorMaxNeighborRangeMm = std::clamp(config.anchorMaxNeighborRangeMm, 1, 500);
    config.anchorMaxRawFilteredGapMm = std::clamp(config.anchorMaxRawFilteredGapMm, 0, 1000);
    config.anchorEdgeDilatePixels = std::clamp(config.anchorEdgeDilatePixels, 0, 15);
    config.anchorHoldoutPercent = std::clamp(config.anchorHoldoutPercent, 0, 90);
    config.stableContourMinAnchors = std::clamp(config.stableContourMinAnchors, 1, 10000);
    config.stableContourTopCount = std::clamp(config.stableContourTopCount, 1, 100);
    return config;
}

float findDepthScale(const rs2::pipeline_profile& profile)
{
    const rs2::device device = profile.get_device();
    for (const rs2::sensor& sensor : device.query_sensors())
    {
        if (rs2::depth_sensor depthSensor = sensor.as<rs2::depth_sensor>())
        {
            return depthSensor.get_depth_scale();
        }
    }

    return 0.001f;
}

cv::Mat colorFrameToBgr(const rs2::video_frame& frame)
{
    cv::Mat view(
        cv::Size(frame.get_width(), frame.get_height()),
        CV_8UC3,
        const_cast<void*>(frame.get_data()),
        cv::Mat::AUTO_STEP);

    return view.clone();
}

cv::Mat videoFrameToGray8(const rs2::video_frame& frame)
{
    cv::Mat view(
        cv::Size(frame.get_width(), frame.get_height()),
        CV_8UC1,
        const_cast<void*>(frame.get_data()),
        cv::Mat::AUTO_STEP);

    return view.clone();
}

cv::Mat depthFrameToMat(const rs2::depth_frame& frame)
{
    cv::Mat view(
        cv::Size(frame.get_width(), frame.get_height()),
        CV_16UC1,
        const_cast<void*>(frame.get_data()),
        cv::Mat::AUTO_STEP);

    return view.clone();
}

int depthUnitsFromMm(int depthMm, float depthScale)
{
    return std::max(1, static_cast<int>((depthMm / 1000.0f) / depthScale + 0.5f));
}

cv::Point3f deprojectPixelMeters(
    int x,
    int y,
    uint16_t depthUnits,
    float depthScale,
    const rs2_intrinsics& intrinsics)
{
    const float z = static_cast<float>(depthUnits) * depthScale;
    return cv::Point3f(
        (static_cast<float>(x) - intrinsics.ppx) / intrinsics.fx * z,
        (static_cast<float>(y) - intrinsics.ppy) / intrinsics.fy * z,
        z);
}

void fillPointCloudBounds(
    ObservationMaterial& material,
    const cv::Mat& componentMask,
    const cv::Mat& depth16,
    float depthScale,
    const rs2_intrinsics& intrinsics)
{
    if (intrinsics.fx <= 0.0f || intrinsics.fy <= 0.0f)
    {
        return;
    }

    cv::Point3f minPoint(
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max());
    cv::Point3f maxPoint(
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest());

    int pointCount = 0;
    for (int y = 0; y < componentMask.rows; ++y)
    {
        const uchar* maskRow = componentMask.ptr<uchar>(y);
        const uint16_t* depthRow = depth16.ptr<uint16_t>(y);
        for (int x = 0; x < componentMask.cols; ++x)
        {
            if (maskRow[x] == 0 || depthRow[x] == 0)
            {
                continue;
            }

            const cv::Point3f point = deprojectPixelMeters(x, y, depthRow[x], depthScale, intrinsics);
            minPoint.x = std::min(minPoint.x, point.x);
            minPoint.y = std::min(minPoint.y, point.y);
            minPoint.z = std::min(minPoint.z, point.z);
            maxPoint.x = std::max(maxPoint.x, point.x);
            maxPoint.y = std::max(maxPoint.y, point.y);
            maxPoint.z = std::max(maxPoint.z, point.z);
            ++pointCount;
        }
    }

    if (pointCount > 0)
    {
        material.hasPointCloudBounds = true;
        material.minPointMeters = minPoint;
        material.maxPointMeters = maxPoint;
    }
}

cv::Mat cleanBinaryMask(const cv::Mat& inputMask, const SegmentationConfig& config)
{
    cv::Mat mask = inputMask.clone();

    const int kernelSize = std::max(3, config.morphKernelSize | 1);
    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(kernelSize, kernelSize));

    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    return mask;
}

cv::Mat makeDepthMask(
    const cv::Mat& depth16,
    const SegmentationConfig& config,
    float depthScale)
{
    cv::Mat mask;
    cv::inRange(
        depth16,
        depthUnitsFromMm(config.minDepthMm, depthScale),
        depthUnitsFromMm(config.maxDepthMm, depthScale),
        mask);

    return cleanBinaryMask(mask, config);
}

struct ComponentDepthSummary
{
    bool valid = false;
    int observedDepthMinMm = 0;
    int observedDepthMaxMm = 0;
    int meanDepthMm = 0;
};

ComponentDepthSummary summarizeComponentDepth(
    const cv::Mat& componentMask,
    const cv::Mat& depth16,
    float depthScale)
{
    ComponentDepthSummary summary;
    if (cv::countNonZero(componentMask) == 0)
    {
        return summary;
    }

    const cv::Scalar meanDepthUnits = cv::mean(depth16, componentMask);
    double observedMinUnits = 0.0;
    double observedMaxUnits = 0.0;
    cv::minMaxLoc(depth16, &observedMinUnits, &observedMaxUnits, nullptr, nullptr, componentMask);

    summary.valid = observedMaxUnits > 0.0;
    summary.meanDepthMm = static_cast<int>(meanDepthUnits[0] * depthScale * 1000.0f + 0.5f);
    summary.observedDepthMinMm = static_cast<int>(observedMinUnits * depthScale * 1000.0f + 0.5f);
    summary.observedDepthMaxMm = static_cast<int>(observedMaxUnits * depthScale * 1000.0f + 0.5f);
    return summary;
}

bool isNearForegroundComponent(
    const ComponentDepthSummary& depthSummary,
    const SegmentationConfig& config)
{
    if (!depthSummary.valid)
    {
        return false;
    }

    return depthSummary.observedDepthMinMm <= config.foregroundKeepDepthMm ||
        depthSummary.meanDepthMm <= config.foregroundKeepDepthMm;
}

bool shouldRejectBorderComponent(
    const cv::Rect& roi,
    int area,
    const cv::Size& frameSize,
    int maxBorderAreaPixels,
    bool isNearForeground)
{
    const bool touchesLeft = roi.x <= 0;
    const bool touchesTop = roi.y <= 0;
    const bool touchesRight = roi.x + roi.width >= frameSize.width;
    const bool touchesBottom = roi.y + roi.height >= frameSize.height;
    const int touchedBorders =
        (touchesLeft ? 1 : 0) +
        (touchesTop ? 1 : 0) +
        (touchesRight ? 1 : 0) +
        (touchesBottom ? 1 : 0);

    if (isNearForeground)
    {
        return false;
    }

    if ((touchesLeft && touchesRight) || (touchesTop && touchesBottom))
    {
        return true;
    }

    return touchedBorders > 0 && area > maxBorderAreaPixels;
}

struct LocalComponentMeasurement
{
    bool validDepth = false;
    int meanDepthMm = 0;
    int observedDepthMinMm = 0;
    int observedDepthMaxMm = 0;
    bool hasPointCloudBounds = false;
    cv::Point3f minPointMeters;
    cv::Point3f maxPointMeters;
};

LocalComponentMeasurement measureLocalComponent(
    const cv::Mat& localMask,
    const cv::Rect& roi,
    const cv::Mat& depth16,
    float depthScale,
    const rs2_intrinsics& intrinsics,
    int pixelCount)
{
    LocalComponentMeasurement measurement;
    if (pixelCount <= 0)
    {
        return measurement;
    }

    int64_t depthSumUnits = 0;
    uint16_t minDepthUnits = std::numeric_limits<uint16_t>::max();
    uint16_t maxDepthUnits = 0;
    cv::Point3f minPoint(
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max());
    cv::Point3f maxPoint(
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest());

    int pointCount = 0;
    const bool canDeproject = intrinsics.fx > 0.0f && intrinsics.fy > 0.0f;
    for (int localY = 0; localY < localMask.rows; ++localY)
    {
        const uchar* maskRow = localMask.ptr<uchar>(localY);
        const int y = roi.y + localY;
        const uint16_t* depthRow = depth16.ptr<uint16_t>(y);
        for (int localX = 0; localX < localMask.cols; ++localX)
        {
            if (maskRow[localX] == 0)
            {
                continue;
            }

            const int x = roi.x + localX;
            const uint16_t depthUnits = depthRow[x];
            depthSumUnits += depthUnits;
            minDepthUnits = std::min(minDepthUnits, depthUnits);
            maxDepthUnits = std::max(maxDepthUnits, depthUnits);

            if (depthUnits == 0 || !canDeproject)
            {
                continue;
            }

            const cv::Point3f point = deprojectPixelMeters(x, y, depthUnits, depthScale, intrinsics);
            minPoint.x = std::min(minPoint.x, point.x);
            minPoint.y = std::min(minPoint.y, point.y);
            minPoint.z = std::min(minPoint.z, point.z);
            maxPoint.x = std::max(maxPoint.x, point.x);
            maxPoint.y = std::max(maxPoint.y, point.y);
            maxPoint.z = std::max(maxPoint.z, point.z);
            ++pointCount;
        }
    }

    measurement.validDepth = maxDepthUnits > 0;
    measurement.meanDepthMm =
        static_cast<int>((depthSumUnits / static_cast<double>(pixelCount)) * depthScale * 1000.0f + 0.5f);
    measurement.observedDepthMinMm = static_cast<int>(minDepthUnits * depthScale * 1000.0f + 0.5f);
    measurement.observedDepthMaxMm = static_cast<int>(maxDepthUnits * depthScale * 1000.0f + 0.5f);
    if (pointCount > 0)
    {
        measurement.hasPointCloudBounds = true;
        measurement.minPointMeters = minPoint;
        measurement.maxPointMeters = maxPoint;
    }
    return measurement;
}

void appendObservationMaterialsFromMask(
    const cv::Mat& mask,
    const cv::Mat& depth16,
    const SegmentationConfig& config,
    float depthScale,
    const rs2_intrinsics& intrinsics,
    uint64_t sourceFrameId,
    int depthMinMm,
    int depthMaxMm,
    uint64_t& nextObservationId,
    std::vector<ObservationMaterial>& materials)
{
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int labelCount = cv::connectedComponentsWithStats(mask, labels, stats, centroids, 8, CV_32S);

    const int frameArea = mask.rows * mask.cols;
    const int maxAreaPixels = frameArea * config.maxAreaPercent / 100;
    const int maxRoiAreaPixels = frameArea * config.maxRoiAreaPercent / 100;
    const int maxBorderAreaPixels = frameArea * config.maxBorderAreaPercent / 100;
    const int foregroundMaxAreaPixels = frameArea * config.foregroundMaxAreaPercent / 100;
    const int foregroundMaxRoiAreaPixels = frameArea * config.foregroundMaxRoiAreaPercent / 100;
    const int maxMaterialAreaPixels = frameArea * config.maxMaterialAreaPercent / 100;
    const int maxMaterialRoiAreaPixels = frameArea * config.maxMaterialRoiAreaPercent / 100;

    for (int label = 1; label < labelCount; ++label)
    {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < config.minAreaPixels)
        {
            continue;
        }

        const cv::Rect roi(
            stats.at<int>(label, cv::CC_STAT_LEFT),
            stats.at<int>(label, cv::CC_STAT_TOP),
            stats.at<int>(label, cv::CC_STAT_WIDTH),
            stats.at<int>(label, cv::CC_STAT_HEIGHT));

        cv::Mat localMask;
        cv::compare(labels(roi), label, localMask, cv::CMP_EQ);
        const LocalComponentMeasurement measurement =
            measureLocalComponent(localMask, roi, depth16, depthScale, intrinsics, area);
        const ComponentDepthSummary depthSummary{
            measurement.validDepth,
            measurement.observedDepthMinMm,
            measurement.observedDepthMaxMm,
            measurement.meanDepthMm};
        const bool isNearForeground = isNearForegroundComponent(depthSummary, config);

        if (area > maxAreaPixels && (!isNearForeground || area > foregroundMaxAreaPixels))
        {
            continue;
        }
        if (roi.area() > maxRoiAreaPixels && (!isNearForeground || roi.area() > foregroundMaxRoiAreaPixels))
        {
            continue;
        }
        if (shouldRejectBorderComponent(roi, area, mask.size(), maxBorderAreaPixels, isNearForeground))
        {
            continue;
        }

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(localMask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
        if (contours.empty())
        {
            continue;
        }

        const auto largestContourIt = std::max_element(
            contours.begin(),
            contours.end(),
            [](const auto& lhs, const auto& rhs)
            {
                return cv::contourArea(lhs) < cv::contourArea(rhs);
            });

        const double contourArea = cv::contourArea(*largestContourIt);
        if (contourArea < static_cast<double>(config.minAreaPixels))
        {
            continue;
        }
        if (contourArea > static_cast<double>(maxMaterialAreaPixels) || roi.area() > maxMaterialRoiAreaPixels)
        {
            continue;
        }

        std::vector<cv::Point> preciseContour;
        const double epsilon = cv::arcLength(*largestContourIt, true) * config.contourApproxRatio;
        if (epsilon >= 0.5)
        {
            cv::approxPolyDP(*largestContourIt, preciseContour, epsilon, true);
        }
        if (preciseContour.size() < 3)
        {
            preciseContour = *largestContourIt;
        }
        for (cv::Point& point : preciseContour)
        {
            point.x += roi.x;
            point.y += roi.y;
        }

        ObservationMaterial material;
        material.sourceFrameId = sourceFrameId;
        material.observationId = sourceFrameId * 1000ULL + nextObservationId++;
        material.roi = roi;
        material.center = cv::Point(
            static_cast<int>(centroids.at<double>(label, 0) + 0.5),
            static_cast<int>(centroids.at<double>(label, 1) + 0.5));
        material.pixelCount = area;
        material.depthMinMm = depthMinMm;
        material.depthMaxMm = depthMaxMm;
        material.meanDepthMm = depthSummary.meanDepthMm;
        material.observedDepthMinMm = depthSummary.observedDepthMinMm;
        material.observedDepthMaxMm = depthSummary.observedDepthMaxMm;
        material.contourArea = contourArea;
        material.hasPointCloudBounds = measurement.hasPointCloudBounds;
        material.minPointMeters = measurement.minPointMeters;
        material.maxPointMeters = measurement.maxPointMeters;
        material.contour = std::move(preciseContour);

        materials.push_back(std::move(material));
    }
}

std::vector<ObservationMaterial> extractObservationMaterials(
    const cv::Mat& depth16,
    const cv::Mat& splitBoundaryMask,
    const SegmentationConfig& config,
    float depthScale,
    const rs2_intrinsics& intrinsics,
    uint64_t sourceFrameId,
    cv::Mat* acceptedMask)
{
    std::vector<ObservationMaterial> materials;
    uint64_t nextObservationId = 1;

    for (int depthMinMm = config.minDepthMm; depthMinMm < config.maxDepthMm; depthMinMm += config.depthSliceMm)
    {
        const int depthMaxMm = std::min(config.maxDepthMm, depthMinMm + config.depthSliceMm);

        cv::Mat sliceMask;
        cv::inRange(
            depth16,
            depthUnitsFromMm(depthMinMm, depthScale),
            depthUnitsFromMm(depthMaxMm, depthScale),
            sliceMask);

        sliceMask = cleanBinaryMask(sliceMask, config);
        if (!splitBoundaryMask.empty())
        {
            sliceMask.setTo(0, splitBoundaryMask);
        }

        appendObservationMaterialsFromMask(
            sliceMask,
            depth16,
            config,
            depthScale,
            intrinsics,
            sourceFrameId,
            depthMinMm,
            depthMaxMm,
            nextObservationId,
            materials);
    }

    std::sort(
        materials.begin(),
        materials.end(),
        [](const ObservationMaterial& lhs, const ObservationMaterial& rhs)
        {
            return lhs.contourArea > rhs.contourArea;
        });

    if (materials.size() > static_cast<size_t>(config.maxMaterials))
    {
        materials.resize(static_cast<size_t>(config.maxMaterials));
    }

    if (acceptedMask != nullptr)
    {
        *acceptedMask = cv::Mat::zeros(depth16.size(), CV_8UC1);
        for (const ObservationMaterial& material : materials)
        {
            const std::vector<std::vector<cv::Point>> contours{material.contour};
            cv::drawContours(*acceptedMask, contours, -1, cv::Scalar(255), cv::FILLED);
        }
    }

    return materials;
}

cv::Mat makeMaterialsMask(
    const cv::Size& frameSize,
    const std::vector<ObservationMaterial>& materials)
{
    cv::Mat mask = cv::Mat::zeros(frameSize, CV_8UC1);
    for (const ObservationMaterial& material : materials)
    {
        if (material.contour.size() < 3)
        {
            continue;
        }

        const std::vector<std::vector<cv::Point>> contours{material.contour};
        cv::drawContours(mask, contours, -1, cv::Scalar(255), cv::FILLED);
    }

    return mask;
}

cv::Scalar palette(size_t index)
{
    static const cv::Scalar colors[] = {
        cv::Scalar(64, 190, 255),
        cv::Scalar(96, 220, 96),
        cv::Scalar(255, 176, 64),
        cv::Scalar(220, 96, 255),
        cv::Scalar(64, 220, 220),
        cv::Scalar(255, 96, 96),
        cv::Scalar(180, 220, 64),
        cv::Scalar(192, 160, 255),
    };

    return colors[index % (sizeof(colors) / sizeof(colors[0]))];
}

class DisjointSet
{
public:
    explicit DisjointSet(size_t size)
        : parent_(size),
          rank_(size, 0)
    {
        for (size_t i = 0; i < size; ++i)
        {
            parent_[i] = i;
        }
    }

    size_t find(size_t value)
    {
        if (parent_[value] != value)
        {
            parent_[value] = find(parent_[value]);
        }
        return parent_[value];
    }

    void unite(size_t lhs, size_t rhs)
    {
        size_t rootLhs = find(lhs);
        size_t rootRhs = find(rhs);
        if (rootLhs == rootRhs)
        {
            return;
        }

        if (rank_[rootLhs] < rank_[rootRhs])
        {
            std::swap(rootLhs, rootRhs);
        }
        parent_[rootRhs] = rootLhs;
        if (rank_[rootLhs] == rank_[rootRhs])
        {
            ++rank_[rootLhs];
        }
    }

private:
    std::vector<size_t> parent_;
    std::vector<int> rank_;
};

cv::Rect expandedRect(const cv::Rect& rect, int padding, const cv::Size& frameSize)
{
    const cv::Rect expanded(
        rect.x - padding,
        rect.y - padding,
        rect.width + padding * 2,
        rect.height + padding * 2);

    return expanded & cv::Rect(0, 0, frameSize.width, frameSize.height);
}

cv::Mat dilateMask(const cv::Mat& inputMask, int sizePixels);

cv::Mat contourMaskInRoi(
    const ObservationMaterial& material,
    const cv::Rect& roi)
{
    cv::Mat mask = cv::Mat::zeros(roi.size(), CV_8UC1);
    std::vector<cv::Point> localContour;
    localContour.reserve(material.contour.size());
    for (const cv::Point& point : material.contour)
    {
        localContour.emplace_back(point.x - roi.x, point.y - roi.y);
    }

    const std::vector<std::vector<cv::Point>> localContours{localContour};
    cv::drawContours(mask, localContours, -1, cv::Scalar(255), cv::FILLED, cv::LINE_AA);
    return mask;
}

bool buildObservationMaterialFromLocalMask(
    const cv::Mat& localMask,
    const cv::Rect& roi,
    const cv::Mat& depth16,
    const SegmentationConfig& config,
    float depthScale,
    const rs2_intrinsics& intrinsics,
    uint64_t sourceFrameId,
    int pclClusterId,
    uint64_t& nextObservationId,
    ObservationMaterial& material)
{
    const int pixelCount = cv::countNonZero(localMask);
    if (pixelCount < config.minAreaPixels)
    {
        return false;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(localMask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    if (contours.empty())
    {
        return false;
    }

    const auto largestContourIt = std::max_element(
        contours.begin(),
        contours.end(),
        [](const auto& lhs, const auto& rhs)
        {
            return cv::contourArea(lhs) < cv::contourArea(rhs);
        });

    const double contourArea = cv::contourArea(*largestContourIt);
    if (contourArea < static_cast<double>(config.minAreaPixels))
    {
        return false;
    }
    const int frameArea = depth16.rows * depth16.cols;
    const int maxMaterialAreaPixels = frameArea * config.maxMaterialAreaPercent / 100;
    const int maxMaterialRoiAreaPixels = frameArea * config.maxMaterialRoiAreaPercent / 100;
    if (contourArea > static_cast<double>(maxMaterialAreaPixels) || roi.area() > maxMaterialRoiAreaPixels)
    {
        return false;
    }

    std::vector<cv::Point> preciseContour;
    const double epsilon = cv::arcLength(*largestContourIt, true) * config.contourApproxRatio;
    if (epsilon >= 0.5)
    {
        cv::approxPolyDP(*largestContourIt, preciseContour, epsilon, true);
    }
    if (preciseContour.size() < 3)
    {
        preciseContour = *largestContourIt;
    }
    for (cv::Point& point : preciseContour)
    {
        point.x += roi.x;
        point.y += roi.y;
    }

    int validDepthCount = 0;
    int64_t depthSumUnits = 0;
    uint16_t minDepthUnits = std::numeric_limits<uint16_t>::max();
    uint16_t maxDepthUnits = 0;
    cv::Point3f minPoint(
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max(),
        std::numeric_limits<float>::max());
    cv::Point3f maxPoint(
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest(),
        std::numeric_limits<float>::lowest());

    for (int localY = 0; localY < localMask.rows; ++localY)
    {
        const uchar* maskRow = localMask.ptr<uchar>(localY);
        const int y = roi.y + localY;
        const uint16_t* depthRow = depth16.ptr<uint16_t>(y);
        for (int localX = 0; localX < localMask.cols; ++localX)
        {
            if (maskRow[localX] == 0)
            {
                continue;
            }

            const int x = roi.x + localX;
            const uint16_t depthUnits = depthRow[x];
            if (depthUnits == 0)
            {
                continue;
            }

            minDepthUnits = std::min(minDepthUnits, depthUnits);
            maxDepthUnits = std::max(maxDepthUnits, depthUnits);
            depthSumUnits += depthUnits;
            ++validDepthCount;

            const cv::Point3f point = deprojectPixelMeters(x, y, depthUnits, depthScale, intrinsics);
            minPoint.x = std::min(minPoint.x, point.x);
            minPoint.y = std::min(minPoint.y, point.y);
            minPoint.z = std::min(minPoint.z, point.z);
            maxPoint.x = std::max(maxPoint.x, point.x);
            maxPoint.y = std::max(maxPoint.y, point.y);
            maxPoint.z = std::max(maxPoint.z, point.z);
        }
    }

    if (validDepthCount == 0)
    {
        return false;
    }

    const cv::Moments moments = cv::moments(*largestContourIt);
    cv::Point center(roi.x + roi.width / 2, roi.y + roi.height / 2);
    if (std::abs(moments.m00) > 1e-6)
    {
        center = cv::Point(
            roi.x + static_cast<int>(moments.m10 / moments.m00 + 0.5),
            roi.y + static_cast<int>(moments.m01 / moments.m00 + 0.5));
    }

    material = ObservationMaterial{};
    material.sourceFrameId = sourceFrameId;
    material.observationId = sourceFrameId * 1000ULL + nextObservationId++;
    material.roi = cv::boundingRect(preciseContour);
    material.center = center;
    material.pixelCount = pixelCount;
    material.depthMinMm = static_cast<int>(minDepthUnits * depthScale * 1000.0f + 0.5f);
    material.depthMaxMm = static_cast<int>(maxDepthUnits * depthScale * 1000.0f + 0.5f);
    material.observedDepthMinMm = material.depthMinMm;
    material.observedDepthMaxMm = material.depthMaxMm;
    material.meanDepthMm = static_cast<int>((depthSumUnits / static_cast<double>(validDepthCount)) * depthScale * 1000.0f + 0.5f);
    material.pclClusterId = pclClusterId;
    material.contourArea = contourArea;
    material.hasPointCloudBounds = true;
    material.minPointMeters = minPoint;
    material.maxPointMeters = maxPoint;
    material.contour = std::move(preciseContour);
    return true;
}

std::vector<ObservationMaterial> refineObservationMaterialsWithPclClusters(
    const std::vector<ObservationMaterial>& materials,
    const cv::Mat& depth16,
    const SegmentationConfig& config,
    float depthScale,
    const rs2_intrinsics& intrinsics,
    uint64_t sourceFrameId)
{
    if (!config.pclClustering || materials.empty() || intrinsics.fx <= 0.0f || intrinsics.fy <= 0.0f)
    {
        return materials;
    }

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());
    std::vector<cv::Point> pointPixels;
    std::vector<size_t> pointMaterialIndexes;
    const cv::Rect imageBounds(0, 0, depth16.cols, depth16.rows);
    const int minDepthUnits = depthUnitsFromMm(config.minDepthMm, depthScale);
    const int maxDepthUnits = depthUnitsFromMm(config.maxDepthMm, depthScale);
    const int sampleStep = std::max(1, config.pclSampleStepPixels);

    for (size_t materialIndex = 0; materialIndex < materials.size(); ++materialIndex)
    {
        const ObservationMaterial& material = materials[materialIndex];
        const cv::Rect roi = material.roi & imageBounds;
        if (roi.empty())
        {
            continue;
        }

        const cv::Mat localMask = contourMaskInRoi(material, roi);
        for (int localY = 0; localY < roi.height; localY += sampleStep)
        {
            const uchar* maskRow = localMask.ptr<uchar>(localY);
            const int y = roi.y + localY;
            const uint16_t* depthRow = depth16.ptr<uint16_t>(y);
            for (int localX = 0; localX < roi.width; localX += sampleStep)
            {
                if (maskRow[localX] == 0)
                {
                    continue;
                }

                const int x = roi.x + localX;
                const uint16_t depthUnits = depthRow[x];
                if (depthUnits < minDepthUnits || depthUnits > maxDepthUnits)
                {
                    continue;
                }

                const cv::Point3f point = deprojectPixelMeters(x, y, depthUnits, depthScale, intrinsics);
                cloud->points.emplace_back(point.x, point.y, point.z);
                pointPixels.emplace_back(x, y);
                pointMaterialIndexes.push_back(materialIndex);
            }
        }
    }

    if (cloud->points.size() < static_cast<size_t>(config.pclMinClusterPoints))
    {
        return materials;
    }

    if (cloud->points.size() > static_cast<size_t>(config.pclMaxInputPoints))
    {
        const size_t stride =
            (cloud->points.size() + static_cast<size_t>(config.pclMaxInputPoints) - 1) /
            static_cast<size_t>(config.pclMaxInputPoints);
        pcl::PointCloud<pcl::PointXYZ>::Ptr sampledCloud(new pcl::PointCloud<pcl::PointXYZ>());
        std::vector<cv::Point> sampledPixels;
        std::vector<size_t> sampledMaterialIndexes;
        sampledCloud->points.reserve(static_cast<size_t>(config.pclMaxInputPoints));
        sampledPixels.reserve(static_cast<size_t>(config.pclMaxInputPoints));
        sampledMaterialIndexes.reserve(static_cast<size_t>(config.pclMaxInputPoints));

        for (size_t index = 0; index < cloud->points.size(); index += stride)
        {
            sampledCloud->points.push_back(cloud->points[index]);
            sampledPixels.push_back(pointPixels[index]);
            sampledMaterialIndexes.push_back(pointMaterialIndexes[index]);
        }

        cloud = sampledCloud;
        pointPixels = std::move(sampledPixels);
        pointMaterialIndexes = std::move(sampledMaterialIndexes);
    }

    cloud->width = static_cast<uint32_t>(cloud->points.size());
    cloud->height = 1;
    cloud->is_dense = false;

    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> extraction;
    extraction.setClusterTolerance(config.pclClusterToleranceMm / 1000.0);
    extraction.setMinClusterSize(config.pclMinClusterPoints);
    extraction.setMaxClusterSize(config.pclMaxClusterPoints);
    extraction.setSearchMethod(tree);
    extraction.setInputCloud(cloud);
    extraction.extract(clusterIndices);
    if (clusterIndices.empty())
    {
        return materials;
    }

    std::vector<int> bestClusterId(materials.size(), 0);
    std::vector<int> bestClusterVotes(materials.size(), 0);
    int clusterId = 1;
    for (const pcl::PointIndices& cluster : clusterIndices)
    {
        std::vector<int> materialVotes(materials.size(), 0);
        for (const int pointIndex : cluster.indices)
        {
            if (pointIndex < 0 || static_cast<size_t>(pointIndex) >= pointMaterialIndexes.size())
            {
                continue;
            }

            const size_t materialIndex = pointMaterialIndexes[static_cast<size_t>(pointIndex)];
            if (materialIndex < materialVotes.size())
            {
                ++materialVotes[materialIndex];
            }
        }

        for (size_t materialIndex = 0; materialIndex < materialVotes.size(); ++materialIndex)
        {
            if (materialVotes[materialIndex] > bestClusterVotes[materialIndex])
            {
                bestClusterVotes[materialIndex] = materialVotes[materialIndex];
                bestClusterId[materialIndex] = clusterId;
            }
        }

        ++clusterId;
    }

    std::vector<ObservationMaterial> refinedMaterials = materials;
    for (size_t materialIndex = 0; materialIndex < refinedMaterials.size(); ++materialIndex)
    {
        refinedMaterials[materialIndex].sourceFrameId = sourceFrameId;
        refinedMaterials[materialIndex].pclClusterId = bestClusterId[materialIndex];
    }

    return refinedMaterials;
}

bool areDepthCompatible(
    const ObservationMaterial& lhs,
    const ObservationMaterial& rhs,
    const SegmentationConfig& config)
{
    const int rangeGap = std::max(
        0,
        std::max(lhs.observedDepthMinMm, rhs.observedDepthMinMm) -
        std::min(lhs.observedDepthMaxMm, rhs.observedDepthMaxMm));
    const int meanGap = std::abs(lhs.meanDepthMm - rhs.meanDepthMm);
    return rangeGap <= config.groupDepthGapMm || meanGap <= config.groupDepthGapMm;
}

float axisGap(float minA, float maxA, float minB, float maxB)
{
    if (maxA < minB)
    {
        return minB - maxA;
    }
    if (maxB < minA)
    {
        return minA - maxB;
    }
    return 0.0f;
}

bool areSpatialClusterCompatible(
    const ObservationMaterial& lhs,
    const ObservationMaterial& rhs,
    const SegmentationConfig& config)
{
    if (!config.spatialClusterCheck || !lhs.hasPointCloudBounds || !rhs.hasPointCloudBounds)
    {
        return true;
    }

    const float gapX = axisGap(lhs.minPointMeters.x, lhs.maxPointMeters.x, rhs.minPointMeters.x, rhs.maxPointMeters.x);
    const float gapY = axisGap(lhs.minPointMeters.y, lhs.maxPointMeters.y, rhs.minPointMeters.y, rhs.maxPointMeters.y);
    const float gapZ = axisGap(lhs.minPointMeters.z, lhs.maxPointMeters.z, rhs.minPointMeters.z, rhs.maxPointMeters.z);
    const int gapMm = static_cast<int>(std::sqrt(gapX * gapX + gapY * gapY + gapZ * gapZ) * 1000.0f + 0.5f);
    return gapMm <= config.spatialClusterGapMm;
}

bool arePclClusterCompatible(
    const ObservationMaterial& lhs,
    const ObservationMaterial& rhs,
    const SegmentationConfig& config)
{
    if (!config.pclClustering || lhs.pclClusterId <= 0 || rhs.pclClusterId <= 0)
    {
        return true;
    }

    return lhs.pclClusterId == rhs.pclClusterId;
}

double rectIou(const cv::Rect& lhs, const cv::Rect& rhs)
{
    const cv::Rect intersection = lhs & rhs;
    if (intersection.empty())
    {
        return 0.0;
    }

    const double intersectionArea = static_cast<double>(intersection.area());
    const double unionArea =
        static_cast<double>(lhs.area()) +
        static_cast<double>(rhs.area()) -
        intersectionArea;
    return unionArea <= 0.0 ? 0.0 : intersectionArea / unionArea;
}

double centerDistancePixels(const cv::Point& lhs, const cv::Point& rhs)
{
    const double dx = static_cast<double>(lhs.x - rhs.x);
    const double dy = static_cast<double>(lhs.y - rhs.y);
    return std::sqrt(dx * dx + dy * dy);
}

double areaRatio(const ObservationMaterial& lhs, const ObservationMaterial& rhs)
{
    const double lhsArea = std::max(1.0, lhs.contourArea);
    const double rhsArea = std::max(1.0, rhs.contourArea);
    return std::min(lhsArea, rhsArea) / std::max(lhsArea, rhsArea);
}

int smoothIntValue(int previous, int current, double previousWeight)
{
    const double currentWeight = 1.0 - previousWeight;
    return static_cast<int>(previous * previousWeight + current * currentWeight + 0.5);
}

double smoothDoubleValue(double previous, double current, double previousWeight)
{
    const double currentWeight = 1.0 - previousWeight;
    return previous * previousWeight + current * currentWeight;
}

cv::Rect smoothRectValue(const cv::Rect& previous, const cv::Rect& current, double previousWeight)
{
    return cv::Rect(
        smoothIntValue(previous.x, current.x, previousWeight),
        smoothIntValue(previous.y, current.y, previousWeight),
        std::max(1, smoothIntValue(previous.width, current.width, previousWeight)),
        std::max(1, smoothIntValue(previous.height, current.height, previousWeight)));
}

ObservationMaterial smoothTrackedMaterial(
    const ObservationMaterial& previous,
    const ObservationMaterial& current,
    const SegmentationConfig& config,
    uint64_t trackId)
{
    const double previousWeight = config.trackSmoothPercent / 100.0;
    ObservationMaterial result = current;
    result.observationId = trackId;
    result.groupId = 0;
    result.center = cv::Point(
        smoothIntValue(previous.center.x, current.center.x, previousWeight),
        smoothIntValue(previous.center.y, current.center.y, previousWeight));

    const int dx = result.center.x - current.center.x;
    const int dy = result.center.y - current.center.y;
    for (cv::Point& point : result.contour)
    {
        point.x += dx;
        point.y += dy;
    }

    if (result.contour.size() >= 3)
    {
        result.roi = cv::boundingRect(result.contour);
    }
    else
    {
        result.roi = smoothRectValue(previous.roi, current.roi, previousWeight);
    }

    result.pixelCount = smoothIntValue(previous.pixelCount, current.pixelCount, previousWeight);
    result.meanDepthMm = smoothIntValue(previous.meanDepthMm, current.meanDepthMm, previousWeight);
    result.observedDepthMinMm = smoothIntValue(previous.observedDepthMinMm, current.observedDepthMinMm, previousWeight);
    result.observedDepthMaxMm = smoothIntValue(previous.observedDepthMaxMm, current.observedDepthMaxMm, previousWeight);
    result.contourArea = smoothDoubleValue(previous.contourArea, current.contourArea, previousWeight);
    return result;
}

struct CandidateTrackMatch
{
    size_t trackIndex = 0;
    size_t candidateIndex = 0;
    double score = 0.0;
};

struct SegmentationTrack
{
    uint64_t trackId = 0;
    ObservationMaterial material;
    int hits = 0;
    int misses = 0;
    uint64_t lastFrameId = 0;
    bool matched = false;
};

double materialTrackMatchScore(
    const ObservationMaterial& trackMaterial,
    const ObservationMaterial& candidate,
    const SegmentationConfig& config)
{
    const int depthGap = std::abs(trackMaterial.meanDepthMm - candidate.meanDepthMm);
    if (depthGap > config.trackDepthGapMm)
    {
        return -1.0;
    }

    const double iou = rectIou(trackMaterial.roi, candidate.roi);
    const double centerDistance = centerDistancePixels(trackMaterial.center, candidate.center);
    const double area = areaRatio(trackMaterial, candidate);
    const double minIou = config.trackIouPercent / 100.0;

    if (area < 0.15)
    {
        return -1.0;
    }
    if (iou < minIou && centerDistance > static_cast<double>(config.trackCenterGapPixels))
    {
        return -1.0;
    }

    const double centerScore = std::max(0.0, 1.0 - centerDistance / std::max(1, config.trackCenterGapPixels));
    const double depthScore = std::max(0.0, 1.0 - static_cast<double>(depthGap) / std::max(1, config.trackDepthGapMm));
    return iou * 4.0 + centerScore * 2.0 + depthScore + area;
}

class SegmentationTracker
{
public:
    std::vector<ObservationMaterial> update(
        const std::vector<ObservationMaterial>& candidates,
        const SegmentationConfig& config,
        uint64_t frameId)
    {
        if (!config.historyTracking)
        {
            return candidates;
        }

        for (SegmentationTrack& track : tracks_)
        {
            track.matched = false;
        }

        std::vector<bool> candidateUsed(candidates.size(), false);
        std::vector<CandidateTrackMatch> matches;
        for (size_t trackIndex = 0; trackIndex < tracks_.size(); ++trackIndex)
        {
            for (size_t candidateIndex = 0; candidateIndex < candidates.size(); ++candidateIndex)
            {
                const double score = materialTrackMatchScore(
                    tracks_[trackIndex].material,
                    candidates[candidateIndex],
                    config);
                if (score >= 0.0)
                {
                    matches.push_back(CandidateTrackMatch{trackIndex, candidateIndex, score});
                }
            }
        }

        std::sort(
            matches.begin(),
            matches.end(),
            [](const CandidateTrackMatch& lhs, const CandidateTrackMatch& rhs)
            {
                return lhs.score > rhs.score;
            });

        for (const CandidateTrackMatch& match : matches)
        {
            SegmentationTrack& track = tracks_[match.trackIndex];
            if (track.matched || candidateUsed[match.candidateIndex])
            {
                continue;
            }

            const ObservationMaterial& candidate = candidates[match.candidateIndex];
            track.material = smoothTrackedMaterial(track.material, candidate, config, track.trackId);
            track.material.sourceFrameId = frameId;
            track.hits = std::min(track.hits + 1, config.trackConfirmFrames + config.trackMissFrames + 1);
            track.misses = 0;
            track.lastFrameId = frameId;
            track.matched = true;
            candidateUsed[match.candidateIndex] = true;
        }

        for (SegmentationTrack& track : tracks_)
        {
            if (!track.matched)
            {
                ++track.misses;
                track.material.sourceFrameId = frameId;
            }
        }

        for (size_t candidateIndex = 0; candidateIndex < candidates.size(); ++candidateIndex)
        {
            if (candidateUsed[candidateIndex])
            {
                continue;
            }

            SegmentationTrack track;
            track.trackId = nextTrackId_++;
            track.material = candidates[candidateIndex];
            track.material.observationId = track.trackId;
            track.material.groupId = 0;
            track.hits = 1;
            track.misses = 0;
            track.lastFrameId = frameId;
            track.matched = true;
            tracks_.push_back(std::move(track));
        }

        tracks_.erase(
            std::remove_if(
                tracks_.begin(),
                tracks_.end(),
                [&config](const SegmentationTrack& track)
                {
                    return track.misses > config.trackMissFrames;
                }),
            tracks_.end());

        std::vector<ObservationMaterial> stableMaterials;
        stableMaterials.reserve(tracks_.size());
        for (const SegmentationTrack& track : tracks_)
        {
            if (track.hits >= config.trackConfirmFrames && track.misses <= config.trackMissFrames)
            {
                stableMaterials.push_back(track.material);
            }
        }

        if (stableMaterials.size() > static_cast<size_t>(config.maxMaterials))
        {
            stableMaterials.resize(static_cast<size_t>(config.maxMaterials));
        }

        return stableMaterials;
    }

private:
    std::vector<SegmentationTrack> tracks_;
    uint64_t nextTrackId_ = 1;
};

std::vector<ObservationGroup> buildObservationGroups(
    std::vector<ObservationMaterial>& materials,
    const cv::Size& frameSize,
    const SegmentationConfig& config)
{
    if (materials.empty())
    {
        return {};
    }

    const int kernelSize = std::max(1, config.groupGapPixels * 2 + 1);
    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(kernelSize, kernelSize));

    DisjointSet sets(materials.size());
    for (size_t i = 0; i < materials.size(); ++i)
    {
        const cv::Rect expandedRoiI = expandedRect(materials[i].roi, config.groupGapPixels, frameSize);

        for (size_t j = i + 1; j < materials.size(); ++j)
        {
            if (!arePclClusterCompatible(materials[i], materials[j], config))
            {
                continue;
            }
            if (!areDepthCompatible(materials[i], materials[j], config))
            {
                continue;
            }
            if (!areSpatialClusterCompatible(materials[i], materials[j], config))
            {
                continue;
            }

            const cv::Rect expandedRoiJ = expandedRect(materials[j].roi, config.groupGapPixels, frameSize);
            const cv::Rect testRoi = expandedRoiI & expandedRoiJ;
            if (testRoi.empty())
            {
                continue;
            }

            cv::Mat maskI = contourMaskInRoi(materials[i], testRoi);
            cv::Mat maskJ = contourMaskInRoi(materials[j], testRoi);
            if (config.groupGapPixels > 0)
            {
                cv::dilate(maskI, maskI, kernel);
                cv::dilate(maskJ, maskJ, kernel);
            }

            cv::Mat intersection;
            cv::bitwise_and(maskI, maskJ, intersection);
            if (cv::countNonZero(intersection) > 0)
            {
                sets.unite(i, j);
            }
        }
    }

    std::vector<size_t> roots;
    std::vector<ObservationGroup> groups;
    for (size_t i = 0; i < materials.size(); ++i)
    {
        const size_t root = sets.find(i);
        auto rootIt = std::find(roots.begin(), roots.end(), root);
        if (rootIt == roots.end())
        {
            roots.push_back(root);
            ObservationGroup group;
            group.groupId = static_cast<int>(groups.size() + 1);
            group.roi = materials[i].roi;
            groups.push_back(std::move(group));
            rootIt = roots.end() - 1;
        }

        ObservationGroup& group = groups[static_cast<size_t>(std::distance(roots.begin(), rootIt))];
        group.materialIndexes.push_back(i);
        group.roi |= materials[i].roi;
        group.pixelCount += materials[i].pixelCount;
        materials[i].groupId = group.groupId;
    }

    for (ObservationGroup& group : groups)
    {
        int64_t weightedX = 0;
        int64_t weightedY = 0;
        int64_t totalPixels = 0;
        for (const size_t materialIndex : group.materialIndexes)
        {
            const ObservationMaterial& material = materials[materialIndex];
            weightedX += material.center.x * material.pixelCount;
            weightedY += material.center.y * material.pixelCount;
            totalPixels += material.pixelCount;
        }

        if (totalPixels > 0)
        {
            group.center = cv::Point(
                static_cast<int>(weightedX / totalPixels),
                static_cast<int>(weightedY / totalPixels));
        }
        else
        {
            group.center = cv::Point(group.roi.x + group.roi.width / 2, group.roi.y + group.roi.height / 2);
        }
    }

    return groups;
}

cv::Mat makeRawDepthRangeMask(
    const cv::Mat& depth16,
    const SegmentationConfig& config,
    float depthScale)
{
    cv::Mat mask;
    cv::inRange(
        depth16,
        depthUnitsFromMm(config.minDepthMm, depthScale),
        depthUnitsFromMm(config.maxDepthMm, depthScale),
        mask);
    return mask;
}

cv::Mat makeDepthEdgeMask(
    const cv::Mat& depth16,
    const SegmentationConfig& config,
    float depthScale)
{
    const int minDepthUnits = depthUnitsFromMm(config.minDepthMm, depthScale);
    const int maxDepthUnits = depthUnitsFromMm(config.maxDepthMm, depthScale);
    const double scale = 255.0 / std::max(1, maxDepthUnits - minDepthUnits);
    const double shift = -static_cast<double>(minDepthUnits) * scale;

    cv::Mat validMask = makeRawDepthRangeMask(depth16, config, depthScale);
    cv::Mat depth8;
    depth16.convertTo(depth8, CV_8UC1, scale, shift);
    depth8.setTo(0, ~validMask);

    cv::medianBlur(depth8, depth8, 5);

    cv::Mat edges;
    cv::Canny(depth8, edges, config.depthCannyLow, config.depthCannyHigh, 3, true);

    cv::Mat erodedValid;
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    cv::erode(validMask, erodedValid, kernel);
    cv::bitwise_and(edges, erodedValid, edges);
    return edges;
}

cv::Mat makeDepthHoleEdgeMask(
    const cv::Mat& validMask,
    const SegmentationConfig& config)
{
    if (validMask.empty())
    {
        return {};
    }

    const int kernelSize = std::max(1, config.depthHoleEdgePixels | 1);
    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(kernelSize, kernelSize));

    cv::Mat erodedValid;
    cv::erode(validMask, erodedValid, kernel);

    cv::Mat validSideBoundary;
    cv::bitwise_xor(validMask, erodedValid, validSideBoundary);

    if (validSideBoundary.cols > 2 && validSideBoundary.rows > 2)
    {
        validSideBoundary.row(0).setTo(0);
        validSideBoundary.row(validSideBoundary.rows - 1).setTo(0);
        validSideBoundary.col(0).setTo(0);
        validSideBoundary.col(validSideBoundary.cols - 1).setTo(0);
    }

    return validSideBoundary;
}

cv::Mat makeGrayEdgeMask(
    const cv::Mat& grayOrBgr,
    const cv::Mat& validMask,
    int cannyLow,
    int cannyHigh)
{
    cv::Mat gray;
    if (grayOrBgr.channels() == 1)
    {
        gray = grayOrBgr.clone();
    }
    else
    {
        cv::cvtColor(grayOrBgr, gray, cv::COLOR_BGR2GRAY);
    }

    if (gray.size() != validMask.size())
    {
        cv::resize(gray, gray, validMask.size(), 0.0, 0.0, cv::INTER_LINEAR);
    }
    cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0.0);

    cv::Mat edges;
    cv::Canny(gray, edges, cannyLow, cannyHigh, 3, true);
    cv::bitwise_and(edges, validMask, edges);
    return edges;
}

cv::Mat makeColorEdgeMask(
    const cv::Mat& colorBgr,
    const cv::Mat& validMask,
    const SegmentationConfig& config)
{
    return makeGrayEdgeMask(colorBgr, validMask, config.colorCannyLow, config.colorCannyHigh);
}

cv::Mat dilateMask(const cv::Mat& inputMask, int sizePixels)
{
    if (sizePixels <= 1)
    {
        return inputMask.clone();
    }

    const int kernelSize = std::max(1, sizePixels | 1);
    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(kernelSize, kernelSize));
    cv::Mat outputMask;
    cv::dilate(inputMask, outputMask, kernel);
    return outputMask;
}

cv::Mat makeDepthConfirmedColorEdgeMask(
    const cv::Mat& colorEdges,
    const cv::Mat& depth16,
    const SegmentationConfig& config,
    float depthScale)
{
    cv::Mat confirmed = cv::Mat::zeros(colorEdges.size(), CV_8UC1);
    if (colorEdges.empty() || depth16.empty())
    {
        return confirmed;
    }

    const int minDepthUnits = depthUnitsFromMm(config.minDepthMm, depthScale);
    const int maxDepthUnits = depthUnitsFromMm(config.maxDepthMm, depthScale);
    const int minRangeUnits = depthUnitsFromMm(config.contourDepthConfirmMinRangeMm, depthScale);
    const int radius = config.contourDepthConfirmRadiusPixels;

    for (int y = 0; y < colorEdges.rows; ++y)
    {
        const uint8_t* edgeRow = colorEdges.ptr<uint8_t>(y);
        uint8_t* confirmedRow = confirmed.ptr<uint8_t>(y);
        for (int x = 0; x < colorEdges.cols; ++x)
        {
            if (edgeRow[x] == 0)
            {
                continue;
            }

            int validCount = 0;
            int localMin = std::numeric_limits<int>::max();
            int localMax = 0;
            const int y0 = std::max(0, y - radius);
            const int y1 = std::min(depth16.rows - 1, y + radius);
            const int x0 = std::max(0, x - radius);
            const int x1 = std::min(depth16.cols - 1, x + radius);

            for (int yy = y0; yy <= y1; ++yy)
            {
                const uint16_t* depthRow = depth16.ptr<uint16_t>(yy);
                for (int xx = x0; xx <= x1; ++xx)
                {
                    const int depth = depthRow[xx];
                    if (depth < minDepthUnits || depth > maxDepthUnits)
                    {
                        continue;
                    }

                    ++validCount;
                    localMin = std::min(localMin, depth);
                    localMax = std::max(localMax, depth);
                }
            }

            if (validCount >= config.contourDepthConfirmMinValidPixels &&
                localMax - localMin >= minRangeUnits)
            {
                confirmedRow[x] = 255;
            }
        }
    }

    return confirmed;
}

BoundaryAnalysis makeBoundaryAnalysis(
    const cv::Mat& edgeGrayOrBgr,
    const cv::Mat& depth16,
    const SegmentationConfig& config,
    float depthScale,
    bool edgeSourceIsInfrared)
{
    BoundaryAnalysis analysis;
    analysis.edgeSourceIsInfrared = edgeSourceIsInfrared;
    if (!config.boundarySplit)
    {
        return analysis;
    }

    analysis.validMask = makeRawDepthRangeMask(depth16, config, depthScale);
    analysis.depthStepEdges = makeDepthEdgeMask(depth16, config, depthScale);
    analysis.rawBoundaryMask = analysis.depthStepEdges.clone();

    if (config.depthHoleSplit || config.boundaryDiagnostics)
    {
        analysis.depthHoleEdges = makeDepthHoleEdgeMask(analysis.validMask, config);
        if (config.depthHoleSplit)
        {
            cv::bitwise_or(analysis.rawBoundaryMask, analysis.depthHoleEdges, analysis.rawBoundaryMask);
        }
    }

    if (config.colorSplit)
    {
        const int edgeCannyLow = edgeSourceIsInfrared ? config.infraredCannyLow : config.colorCannyLow;
        const int edgeCannyHigh = edgeSourceIsInfrared ? config.infraredCannyHigh : config.colorCannyHigh;
        analysis.colorEdges = makeGrayEdgeMask(edgeGrayOrBgr, analysis.validMask, edgeCannyLow, edgeCannyHigh);
        cv::Mat depthSupportSource = analysis.depthStepEdges.clone();
        if (config.depthHoleSplit && !analysis.depthHoleEdges.empty())
        {
            cv::bitwise_or(depthSupportSource, analysis.depthHoleEdges, depthSupportSource);
        }
        const cv::Mat depthSupport = dilateMask(depthSupportSource, config.colorDepthSupportPixels);

        cv::bitwise_and(analysis.colorEdges, depthSupport, analysis.depthSupportedColorEdges);
        cv::bitwise_or(analysis.rawBoundaryMask, analysis.depthSupportedColorEdges, analysis.rawBoundaryMask);
        if (config.contourDepthConfirmSplit)
        {
            analysis.depthConfirmedColorEdges =
                makeDepthConfirmedColorEdgeMask(analysis.colorEdges, depth16, config, depthScale);
            cv::bitwise_or(analysis.rawBoundaryMask, analysis.depthConfirmedColorEdges, analysis.rawBoundaryMask);
        }
    }

    analysis.splitBoundaryMask = dilateMask(analysis.rawBoundaryMask, config.splitBoundaryPixels);
    cv::bitwise_and(analysis.splitBoundaryMask, analysis.validMask, analysis.splitBoundaryMask);
    return analysis;
}

cv::Mat makeSplitBoundaryMask(
    const cv::Mat& edgeGrayOrBgr,
    const cv::Mat& depth16,
    const SegmentationConfig& config,
    float depthScale,
    bool edgeSourceIsInfrared = false)
{
    return makeBoundaryAnalysis(edgeGrayOrBgr, depth16, config, depthScale, edgeSourceIsInfrared).splitBoundaryMask;
}

int maskPixelCount(const cv::Mat& mask)
{
    return mask.empty() ? 0 : cv::countNonZero(mask);
}

void paintMask(cv::Mat& view, const cv::Mat& mask, const cv::Scalar& color)
{
    if (mask.empty())
    {
        return;
    }

    view.setTo(color, mask);
}

cv::Mat buildBoundaryDiagnosticsView(
    const cv::Mat& colorBgr,
    const BoundaryAnalysis& analysis)
{
    cv::Mat view;
    colorBgr.convertTo(view, -1, 0.48, 0.0);

    paintMask(view, analysis.colorEdges, cv::Scalar(255, 80, 0));
    paintMask(view, analysis.depthStepEdges, cv::Scalar(0, 0, 255));
    paintMask(view, analysis.depthHoleEdges, cv::Scalar(255, 0, 255));
    paintMask(view, analysis.depthSupportedColorEdges, cv::Scalar(255, 255, 0));
    paintMask(view, analysis.depthConfirmedColorEdges, cv::Scalar(0, 255, 0));
    paintMask(view, analysis.splitBoundaryMask, cv::Scalar(0, 255, 255));

    const std::string edgeSource = analysis.edgeSourceIsInfrared ? "ir" : "gray";
    const std::string title =
        "P1 boundary: red=depth magenta=hole blue=" + edgeSource +
        " cyan=supported green=confirmed yellow=final";
    cv::putText(
        view,
        title,
        cv::Point(10, 24),
        cv::FONT_HERSHEY_SIMPLEX,
        0.42,
        cv::Scalar(0, 0, 0),
        3,
        cv::LINE_AA);
    cv::putText(
        view,
        title,
        cv::Point(10, 24),
        cv::FONT_HERSHEY_SIMPLEX,
        0.42,
        cv::Scalar(255, 255, 255),
        1,
        cv::LINE_AA);

    const std::string counts =
        "D" + std::to_string(maskPixelCount(analysis.depthStepEdges)) +
        " H" + std::to_string(maskPixelCount(analysis.depthHoleEdges)) +
        " G" + std::to_string(maskPixelCount(analysis.colorEdges)) +
        " S" + std::to_string(maskPixelCount(analysis.depthSupportedColorEdges)) +
        " C" + std::to_string(maskPixelCount(analysis.depthConfirmedColorEdges)) +
        " F" + std::to_string(maskPixelCount(analysis.splitBoundaryMask));
    cv::putText(
        view,
        counts,
        cv::Point(10, 46),
        cv::FONT_HERSHEY_SIMPLEX,
        0.42,
        cv::Scalar(0, 0, 0),
        3,
        cv::LINE_AA);
    cv::putText(
        view,
        counts,
        cv::Point(10, 46),
        cv::FONT_HERSHEY_SIMPLEX,
        0.42,
        cv::Scalar(255, 255, 255),
        1,
        cv::LINE_AA);

    return view;
}

cv::Mat contourLineMaskInRoi(
    const ObservationMaterial& material,
    const cv::Rect& roi,
    int thickness)
{
    cv::Mat mask = cv::Mat::zeros(roi.size(), CV_8UC1);
    if (material.contour.size() < 3)
    {
        return mask;
    }

    std::vector<cv::Point> localContour;
    localContour.reserve(material.contour.size());
    for (const cv::Point& point : material.contour)
    {
        localContour.emplace_back(point.x - roi.x, point.y - roi.y);
    }

    const std::vector<std::vector<cv::Point>> localContours{localContour};
    cv::drawContours(mask, localContours, -1, cv::Scalar(255), std::max(1, thickness), cv::LINE_8);
    return mask;
}

int countMaskOverlapInRoi(
    const cv::Mat& globalMask,
    const cv::Rect& roi,
    const cv::Mat& localMask)
{
    if (globalMask.empty() || roi.empty() || localMask.empty())
    {
        return 0;
    }

    cv::Mat overlap;
    cv::bitwise_and(globalMask(roi), localMask, overlap);
    return cv::countNonZero(overlap);
}

CandidateCueEvidence measureCandidateCueEvidence(
    const ObservationMaterial& material,
    const BoundaryAnalysis& boundaryAnalysis,
    const SegmentationConfig& config,
    const cv::Size& frameSize)
{
    CandidateCueEvidence evidence;
    evidence.anchorCount = material.groupId;
    if (material.contour.size() < 3)
    {
        return evidence;
    }

    const cv::Rect frameRect(0, 0, frameSize.width, frameSize.height);
    const cv::Rect roi = cv::boundingRect(material.contour) & frameRect;
    if (roi.empty())
    {
        return evidence;
    }

    const int lineThickness = std::max(2, config.contourLinePixels + 2);
    const cv::Mat contourLineMask = contourLineMaskInRoi(material, roi, lineThickness);
    evidence.contourEdgePixels = cv::countNonZero(contourLineMask);
    evidence.depthEdgePixels = countMaskOverlapInRoi(boundaryAnalysis.depthStepEdges, roi, contourLineMask);
    evidence.supportedRgbEdgePixels = countMaskOverlapInRoi(
        boundaryAnalysis.depthSupportedColorEdges,
        roi,
        contourLineMask);
    evidence.confirmedRgbEdgePixels = countMaskOverlapInRoi(
        boundaryAnalysis.depthConfirmedColorEdges,
        roi,
        contourLineMask);
    evidence.rgbEdgePixels = countMaskOverlapInRoi(boundaryAnalysis.colorEdges, roi, contourLineMask);

    if (!boundaryAnalysis.colorEdges.empty())
    {
        cv::Mat reliableRgb = cv::Mat::zeros(roi.size(), CV_8UC1);
        if (!boundaryAnalysis.depthSupportedColorEdges.empty())
        {
            cv::bitwise_or(reliableRgb, boundaryAnalysis.depthSupportedColorEdges(roi), reliableRgb);
        }
        if (!boundaryAnalysis.depthConfirmedColorEdges.empty())
        {
            cv::bitwise_or(reliableRgb, boundaryAnalysis.depthConfirmedColorEdges(roi), reliableRgb);
        }

        cv::Mat unreliableRgb;
        cv::bitwise_not(reliableRgb, unreliableRgb);
        cv::Mat textureOnly;
        cv::bitwise_and(boundaryAnalysis.colorEdges(roi), unreliableRgb, textureOnly);
        cv::bitwise_and(textureOnly, contourLineMask, textureOnly);
        evidence.textureOnlyRgbEdgePixels = cv::countNonZero(textureOnly);
    }

    evidence.textureOnlyPercent = evidence.contourEdgePixels == 0
        ? 0.0
        : 100.0 * static_cast<double>(evidence.textureOnlyRgbEdgePixels) /
            static_cast<double>(evidence.contourEdgePixels);
    return evidence;
}

std::vector<ObservationMaterial> selectCandidatesByCue(
    const std::vector<ObservationMaterial>& candidates,
    const BoundaryAnalysis& boundaryAnalysis,
    const SegmentationConfig& config,
    const cv::Size& frameSize,
    int minAnchorCount,
    CueSelectionSummary& summary)
{
    summary = CueSelectionSummary{};
    summary.inputCandidates = static_cast<int>(candidates.size());
    if (!config.cueSelection)
    {
        summary.acceptedCandidates = summary.inputCandidates;
        return candidates;
    }

    std::vector<ObservationMaterial> selected;
    selected.reserve(candidates.size());
    const int strongAnchorCount = std::max(
        minAnchorCount,
        minAnchorCount * config.cueStrongAnchorMultiplierPercent / 100);

    for (const ObservationMaterial& candidate : candidates)
    {
        const CandidateCueEvidence evidence =
            measureCandidateCueEvidence(candidate, boundaryAnalysis, config, frameSize);
        summary.textureOnlyPercentSum += evidence.textureOnlyPercent;

        const bool depthCue = evidence.depthEdgePixels >= config.cueMinReliableEdgePixels;
        const bool rgbCue =
            evidence.confirmedRgbEdgePixels >= config.cueMinConfirmedRgbEdgePixels ||
            evidence.supportedRgbEdgePixels >= config.cueMinReliableEdgePixels;
        const bool anchorCue = evidence.anchorCount >= strongAnchorCount;
        const int reliableEdgePixels =
            evidence.depthEdgePixels +
            evidence.supportedRgbEdgePixels +
            evidence.confirmedRgbEdgePixels;
        const bool textureDominant =
            evidence.textureOnlyRgbEdgePixels >= config.cueMinReliableEdgePixels &&
            evidence.textureOnlyPercent >= static_cast<double>(config.cueMaxTextureOnlyPercent);
        const bool rejectAsTexture =
            textureDominant &&
            reliableEdgePixels < config.cueMinReliableEdgePixels &&
            !anchorCue;

        if (rejectAsTexture)
        {
            ++summary.rejectedTextureCandidates;
            continue;
        }

        if (depthCue)
        {
            ++summary.acceptedByDepth;
        }
        if (rgbCue)
        {
            ++summary.acceptedByRgb;
        }
        if (anchorCue)
        {
            ++summary.acceptedByAnchor;
        }
        if (!depthCue && !rgbCue && !anchorCue)
        {
            ++summary.acceptedByFallback;
        }

        selected.push_back(candidate);
    }

    summary.acceptedCandidates = static_cast<int>(selected.size());
    return selected;
}

void drawCueSelectionOverlay(cv::Mat& view, const CueSelectionSummary& summary)
{
    const double meanTexturePercent = summary.inputCandidates == 0
        ? 0.0
        : summary.textureOnlyPercentSum / static_cast<double>(summary.inputCandidates);
    std::ostringstream line;
    line << "cue in=" << summary.inputCandidates
        << " keep=" << summary.acceptedCandidates
        << " reject_texture=" << summary.rejectedTextureCandidates
        << " D=" << summary.acceptedByDepth
        << " IR=" << summary.acceptedByRgb
        << " A=" << summary.acceptedByAnchor
        << " tex%=" << std::fixed << std::setprecision(1) << meanTexturePercent;

    cv::putText(
        view,
        line.str(),
        cv::Point(12, std::max(20, view.rows - 18)),
        cv::FONT_HERSHEY_SIMPLEX,
        0.42,
        cv::Scalar(0, 0, 0),
        3,
        cv::LINE_AA);
    cv::putText(
        view,
        line.str(),
        cv::Point(12, std::max(20, view.rows - 18)),
        cv::FONT_HERSHEY_SIMPLEX,
        0.42,
        cv::Scalar(255, 255, 255),
        1,
        cv::LINE_AA);
}

cv::Point3f deprojectDepthPoint(
    int x,
    int y,
    uint16_t depthUnits,
    float depthScale,
    const rs2_intrinsics& intrinsics)
{
    const float z = static_cast<float>(depthUnits) * depthScale;
    return cv::Point3f(
        (static_cast<float>(x) - intrinsics.ppx) / intrinsics.fx * z,
        (static_cast<float>(y) - intrinsics.ppy) / intrinsics.fy * z,
        z);
}

cv::Point3f crossProduct(const cv::Point3f& lhs, const cv::Point3f& rhs)
{
    return cv::Point3f(
        lhs.y * rhs.z - lhs.z * rhs.y,
        lhs.z * rhs.x - lhs.x * rhs.z,
        lhs.x * rhs.y - lhs.y * rhs.x);
}

double vectorLength(const cv::Point3f& value)
{
    return std::sqrt(
        static_cast<double>(value.x) * value.x +
        static_cast<double>(value.y) * value.y +
        static_cast<double>(value.z) * value.z);
}

IndoorPlaneAnalysis buildIndoorPlaneAnalysis(
    const cv::Mat& depth16,
    const cv::Mat& stableForegroundMask,
    const SegmentationConfig& config,
    float depthScale,
    const rs2_intrinsics& intrinsics)
{
    IndoorPlaneAnalysis analysis;
    if (depth16.empty())
    {
        return analysis;
    }

    cv::Mat validMask = makeRawDepthRangeMask(depth16, config, depthScale);
    cv::Mat foregroundMask;
    if (!stableForegroundMask.empty())
    {
        foregroundMask = stableForegroundMask.clone();
        if (config.indoorPlaneForegroundDilatePixels > 0)
        {
            foregroundMask = dilateMask(foregroundMask, config.indoorPlaneForegroundDilatePixels);
        }
        validMask.setTo(0, foregroundMask);
    }

    cv::Mat horizontalNormalMask = cv::Mat::zeros(depth16.size(), CV_8UC1);
    cv::Mat verticalNormalMask = cv::Mat::zeros(depth16.size(), CV_8UC1);
    const int sampleStep = config.indoorPlaneSampleStepPixels;
    const int neighborStep = config.indoorPlaneNormalNeighborPixels;
    const int minDepthUnits = depthUnitsFromMm(config.minDepthMm, depthScale);
    const int maxDepthUnits = depthUnitsFromMm(config.maxDepthMm, depthScale);
    const double horizontalMin = config.indoorPlaneHorizontalNormalMinPercent / 100.0;
    const double verticalMax = config.indoorPlaneVerticalNormalMaxPercent / 100.0;

    for (int y = neighborStep; y + neighborStep < depth16.rows; y += sampleStep)
    {
        for (int x = neighborStep; x + neighborStep < depth16.cols; x += sampleStep)
        {
            if (validMask.at<uint8_t>(y, x) == 0)
            {
                continue;
            }

            const uint16_t centerDepth = depth16.at<uint16_t>(y, x);
            const uint16_t rightDepth = depth16.at<uint16_t>(y, x + neighborStep);
            const uint16_t downDepth = depth16.at<uint16_t>(y + neighborStep, x);
            if (centerDepth < minDepthUnits || centerDepth > maxDepthUnits ||
                rightDepth < minDepthUnits || rightDepth > maxDepthUnits ||
                downDepth < minDepthUnits || downDepth > maxDepthUnits)
            {
                continue;
            }

            const cv::Point3f center = deprojectDepthPoint(x, y, centerDepth, depthScale, intrinsics);
            const cv::Point3f right = deprojectDepthPoint(x + neighborStep, y, rightDepth, depthScale, intrinsics);
            const cv::Point3f down = deprojectDepthPoint(x, y + neighborStep, downDepth, depthScale, intrinsics);
            const cv::Point3f dx = right - center;
            const cv::Point3f dy = down - center;
            const cv::Point3f normal = crossProduct(dx, dy);
            const double length = vectorLength(normal);
            if (length < 1e-6)
            {
                continue;
            }

            const double ny = std::abs(static_cast<double>(normal.y) / length);
            const cv::Rect block(
                std::max(0, x - sampleStep / 2),
                std::max(0, y - sampleStep / 2),
                std::min(sampleStep, depth16.cols - std::max(0, x - sampleStep / 2)),
                std::min(sampleStep, depth16.rows - std::max(0, y - sampleStep / 2)));
            if (block.empty())
            {
                continue;
            }

            if (ny >= horizontalMin)
            {
                horizontalNormalMask(block).setTo(255);
            }
            else if (ny <= verticalMax)
            {
                verticalNormalMask(block).setTo(255);
            }
        }
    }

    const cv::Mat morphKernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(7, 7));
    cv::morphologyEx(horizontalNormalMask, horizontalNormalMask, cv::MORPH_CLOSE, morphKernel);
    cv::morphologyEx(verticalNormalMask, verticalNormalMask, cv::MORPH_CLOSE, morphKernel);
    horizontalNormalMask.setTo(0, ~validMask);
    verticalNormalMask.setTo(0, ~validMask);

    cv::Mat candidateMask;
    cv::bitwise_or(horizontalNormalMask, verticalNormalMask, candidateMask);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int labelCount = cv::connectedComponentsWithStats(candidateMask, labels, stats, centroids, 8, CV_32S);
    const int minAreaPixels =
        std::max(1, depth16.rows * depth16.cols * config.indoorPlaneMinAreaPercent / 100);

    analysis.structuralPlaneMask = cv::Mat::zeros(depth16.size(), CV_8UC1);
    analysis.wallMask = cv::Mat::zeros(depth16.size(), CV_8UC1);
    analysis.ceilingMask = cv::Mat::zeros(depth16.size(), CV_8UC1);
    analysis.supportMask = cv::Mat::zeros(depth16.size(), CV_8UC1);

    for (int label = 1; label < labelCount; ++label)
    {
        const int area = stats.at<int>(label, cv::CC_STAT_AREA);
        if (area < minAreaPixels)
        {
            continue;
        }

        cv::Mat componentMask;
        cv::compare(labels, label, componentMask, cv::CMP_EQ);
        cv::bitwise_and(componentMask, validMask, componentMask);
        const int componentPixels = cv::countNonZero(componentMask);
        if (componentPixels < minAreaPixels)
        {
            continue;
        }

        cv::Mat componentHorizontal;
        cv::Mat componentVertical;
        cv::bitwise_and(componentMask, horizontalNormalMask, componentHorizontal);
        cv::bitwise_and(componentMask, verticalNormalMask, componentVertical);
        const int horizontalPixels = cv::countNonZero(componentHorizontal);
        const int verticalPixels = cv::countNonZero(componentVertical);
        const double centerY = centroids.at<double>(label, 1);

        analysis.structuralPlaneMask.setTo(255, componentMask);
        if (horizontalPixels >= verticalPixels)
        {
            if (centerY < depth16.rows * 0.45)
            {
                analysis.ceilingMask.setTo(255, componentMask);
            }
            else
            {
                analysis.supportMask.setTo(255, componentMask);
            }
        }
        else
        {
            const int top = stats.at<int>(label, cv::CC_STAT_TOP);
            const int height = stats.at<int>(label, cv::CC_STAT_HEIGHT);
            const bool touchesTop = top <= std::max(1, depth16.rows / 12);
            const bool spansUpperRoom = height >= std::max(1, depth16.rows / 5);
            if (touchesTop && spansUpperRoom)
            {
                const int ceilingRows = std::max(
                    1,
                    depth16.rows * config.indoorPlaneCeilingBandPercent / 100);
                cv::Mat topBand = cv::Mat::zeros(depth16.size(), CV_8UC1);
                topBand(cv::Rect(0, 0, depth16.cols, ceilingRows)).setTo(255);

                cv::Mat ceilingPart;
                cv::bitwise_and(componentMask, topBand, ceilingPart);
                const int ceilingPartPixels = cv::countNonZero(ceilingPart);
                if (ceilingPartPixels >= std::max(1, minAreaPixels / 3))
                {
                    analysis.ceilingMask.setTo(255, ceilingPart);
                    cv::Mat wallPart = componentMask.clone();
                    wallPart.setTo(0, ceilingPart);
                    if (cv::countNonZero(wallPart) >= std::max(1, minAreaPixels / 3))
                    {
                        analysis.wallMask.setTo(255, wallPart);
                    }
                }
                else
                {
                    analysis.wallMask.setTo(255, componentMask);
                }
            }
            else
            {
                analysis.wallMask.setTo(255, componentMask);
            }
        }
        ++analysis.components;
    }

    analysis.structuralPlanePixels = cv::countNonZero(analysis.structuralPlaneMask);
    analysis.wallPixels = cv::countNonZero(analysis.wallMask);
    analysis.ceilingPixels = cv::countNonZero(analysis.ceilingMask);
    analysis.supportPixels = cv::countNonZero(analysis.supportMask);
    return analysis;
}

cv::Mat buildIndoorPlaneDiagnosticsView(
    const cv::Mat& colorBgr,
    const IndoorPlaneAnalysis& analysis,
    const cv::Mat& stableMask)
{
    cv::Mat view;
    colorBgr.convertTo(view, -1, 0.54, 0.0);
    paintMask(view, analysis.wallMask, cv::Scalar(255, 170, 40));
    paintMask(view, analysis.ceilingMask, cv::Scalar(220, 80, 255));
    paintMask(view, analysis.supportMask, cv::Scalar(0, 180, 255));

    if (!stableMask.empty())
    {
        cv::Mat stableEdge;
        cv::Canny(stableMask, stableEdge, 80, 160);
        paintMask(view, stableEdge, cv::Scalar(80, 255, 80));
    }

    const std::string title =
        "P3 indoor plane: blue=wall magenta=ceiling orange=support green=stable-mask edge";
    cv::putText(
        view,
        title,
        cv::Point(10, 24),
        cv::FONT_HERSHEY_SIMPLEX,
        0.42,
        cv::Scalar(0, 0, 0),
        3,
        cv::LINE_AA);
    cv::putText(
        view,
        title,
        cv::Point(10, 24),
        cv::FONT_HERSHEY_SIMPLEX,
        0.42,
        cv::Scalar(255, 255, 255),
        1,
        cv::LINE_AA);

    const std::string counts =
        "plane=" + std::to_string(analysis.structuralPlanePixels) +
        " wall=" + std::to_string(analysis.wallPixels) +
        " ceiling=" + std::to_string(analysis.ceilingPixels) +
        " support=" + std::to_string(analysis.supportPixels) +
        " comps=" + std::to_string(analysis.components) +
        (analysis.reusedFromCache ? " cached" : " fresh");
    cv::putText(
        view,
        counts,
        cv::Point(10, 46),
        cv::FONT_HERSHEY_SIMPLEX,
        0.42,
        cv::Scalar(0, 0, 0),
        3,
        cv::LINE_AA);
    cv::putText(
        view,
        counts,
        cv::Point(10, 46),
        cv::FONT_HERSHEY_SIMPLEX,
        0.42,
        cv::Scalar(255, 255, 255),
        1,
        cv::LINE_AA);
    return view;
}

int drawEdgeComponents(
    cv::Mat& view,
    const cv::Mat& edgeMask,
    const SegmentationConfig& config)
{
    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int labelCount = cv::connectedComponentsWithStats(edgeMask, labels, stats, centroids, 8, CV_32S);

    int drawnCount = 0;
    for (int label = 1; label < labelCount; ++label)
    {
        const int pixels = stats.at<int>(label, cv::CC_STAT_AREA);
        if (pixels < config.minEdgePixels)
        {
            continue;
        }

        cv::Mat componentMask;
        cv::compare(labels, label, componentMask, cv::CMP_EQ);
        if (config.contourLinePixels > 1)
        {
            const cv::Mat kernel = cv::getStructuringElement(
                cv::MORPH_ELLIPSE,
                cv::Size(config.contourLinePixels, config.contourLinePixels));
            cv::dilate(componentMask, componentMask, kernel);
        }

        view.setTo(palette(static_cast<size_t>(drawnCount)), componentMask);
        ++drawnCount;
    }

    return drawnCount;
}

cv::Mat buildSegmentationView(
    const cv::Mat& colorBgr,
    const cv::Mat& depth16,
    float depthScale,
    const cv::Mat& acceptedMask,
    const cv::Mat& splitBoundaryMask,
    const std::vector<ObservationMaterial>& materials,
    const SegmentationConfig& config)
{
    cv::Mat view;
    colorBgr.convertTo(view, -1, 0.62, 0.0);

    cv::Mat validMask = makeRawDepthRangeMask(depth16, config, depthScale);
    cv::Mat edgeMask = splitBoundaryMask.empty()
        ? makeDepthEdgeMask(depth16, config, depthScale)
        : splitBoundaryMask.clone();
    if (config.colorEdges)
    {
        cv::Mat colorEdges = makeColorEdgeMask(colorBgr, validMask, config);
        cv::bitwise_or(edgeMask, colorEdges, edgeMask);
    }

    const int edgeCount = drawEdgeComponents(view, edgeMask, config);

    if (config.showRegionContours)
    {
        colorBgr.copyTo(view, acceptedMask);
        for (size_t i = 0; i < materials.size(); ++i)
        {
            const ObservationMaterial& material = materials[i];
            const cv::Scalar color = palette(
                material.groupId > 0
                    ? static_cast<size_t>(material.groupId - 1)
                    : i);
            const std::vector<std::vector<cv::Point>> contours{material.contour};

            cv::drawContours(view, contours, -1, color, config.contourLinePixels, cv::LINE_AA);

            if (config.showCenters)
            {
                cv::circle(view, material.center, 3, color, cv::FILLED, cv::LINE_AA);
            }

            if (config.showLabels)
            {
                const std::string label =
                    "#" + std::to_string(i + 1) +
                    " z=" + std::to_string(material.depthMinMm) +
                    "-" + std::to_string(material.depthMaxMm) +
                    " px=" + std::to_string(material.pixelCount);
                const int labelY = std::max(16, material.roi.y - 6);
                cv::putText(
                    view,
                    label,
                    cv::Point(material.roi.x, labelY),
                    cv::FONT_HERSHEY_SIMPLEX,
                    0.45,
                    color,
                    1,
                    cv::LINE_AA);
            }
        }
    }

    const std::string summary =
        "edges=" + std::to_string(edgeCount) +
        " regions=" + std::to_string(materials.size()) +
        " depth=" + std::to_string(config.minDepthMm) +
        "-" + std::to_string(config.maxDepthMm) + "mm" +
        " slice=" + std::to_string(config.depthSliceMm) + "mm" +
        " split=" + (config.boundarySplit ? (config.colorSplit ? "gray+depth" : "depth") : "off") +
        " pcl=" + (config.pclClustering
            ? std::to_string(config.pclClusterToleranceMm) + "mm/" +
                std::to_string(config.pclSampleStepPixels) + "px/" +
                std::to_string(config.pclFrameInterval) + "f"
            : "off") +
        " hist=" + (config.historyTracking ? "on" : "off") +
        " cluster=" + (config.spatialClusterCheck
            ? std::to_string(config.spatialClusterGapMm) + "mm"
            : "off");

    cv::rectangle(view, cv::Rect(0, 0, view.cols, 28), cv::Scalar(0, 0, 0), cv::FILLED);
    cv::putText(
        view,
        summary,
        cv::Point(8, 20),
        cv::FONT_HERSHEY_SIMPLEX,
        0.50,
        cv::Scalar(255, 255, 255),
        1,
        cv::LINE_AA);

    return view;
}

cv::Mat buildContourColorMosaic(
    const cv::Mat& colorBgr,
    const std::vector<ObservationMaterial>& materials,
    const std::vector<ObservationGroup>& groups,
    const SegmentationConfig& config)
{
    cv::Mat mosaic = cv::Mat::zeros(colorBgr.size(), colorBgr.type());

    for (size_t i = 0; i < materials.size(); ++i)
    {
        const ObservationMaterial& material = materials[i];
        const cv::Rect imageBounds(0, 0, colorBgr.cols, colorBgr.rows);
        const cv::Rect roi = material.roi & imageBounds;
        if (roi.empty())
        {
            continue;
        }

        std::vector<cv::Point> localContour;
        localContour.reserve(material.contour.size());
        for (const cv::Point& point : material.contour)
        {
            localContour.emplace_back(point.x - roi.x, point.y - roi.y);
        }

        cv::Mat localMask = cv::Mat::zeros(roi.size(), CV_8UC1);
        const std::vector<std::vector<cv::Point>> localContours{localContour};
        cv::drawContours(localMask, localContours, -1, cv::Scalar(255), cv::FILLED, cv::LINE_AA);

        const cv::Mat sourceCrop = colorBgr(roi);
        cv::Mat targetCrop = mosaic(roi);
        sourceCrop.copyTo(targetCrop, localMask);

        if (config.showRegionContours)
        {
            const std::vector<std::vector<cv::Point>> contours{material.contour};
            const size_t paletteIndex = material.groupId > 0
                ? static_cast<size_t>(material.groupId - 1)
                : i;
            cv::drawContours(mosaic, contours, -1, palette(paletteIndex), config.contourLinePixels, cv::LINE_AA);
        }

        if (config.showPartNumbers)
        {
            const std::string indexText =
                std::to_string(i + 1) + "/" + std::to_string(material.meanDepthMm);
            int baseline = 0;
            const double fontScale = 0.45;
            const int textThickness = 1;
            const cv::Size textSize = cv::getTextSize(
                indexText,
                cv::FONT_HERSHEY_SIMPLEX,
                fontScale,
                textThickness,
                &baseline);

            cv::Point textOrigin(
                material.center.x - textSize.width / 2,
                material.center.y + textSize.height / 2);
            textOrigin.x = std::clamp(textOrigin.x, 2, std::max(2, mosaic.cols - textSize.width - 2));
            textOrigin.y = std::clamp(textOrigin.y, textSize.height + 2, std::max(textSize.height + 2, mosaic.rows - 2));

            cv::putText(
                mosaic,
                indexText,
                textOrigin,
                cv::FONT_HERSHEY_SIMPLEX,
                fontScale,
                cv::Scalar(0, 0, 0),
                4,
                cv::LINE_AA);
            cv::putText(
                mosaic,
                indexText,
                textOrigin,
                cv::FONT_HERSHEY_SIMPLEX,
                fontScale,
                cv::Scalar(220, 220, 220),
                textThickness,
                cv::LINE_AA);
        }
    }

    for (const ObservationGroup& group : groups)
    {
        const std::string indexText = "G" + std::to_string(group.groupId);
        int baseline = 0;
        const double fontScale = 0.75;
        const int textThickness = 2;
        const cv::Size textSize = cv::getTextSize(
            indexText,
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            textThickness,
            &baseline);

        cv::Point textOrigin(
            group.center.x - textSize.width / 2,
            group.center.y + textSize.height / 2);
        textOrigin.x = std::clamp(textOrigin.x, 2, std::max(2, mosaic.cols - textSize.width - 2));
        textOrigin.y = std::clamp(textOrigin.y, textSize.height + 2, std::max(textSize.height + 2, mosaic.rows - 2));

        cv::putText(
            mosaic,
            indexText,
            textOrigin,
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            cv::Scalar(0, 0, 0),
            5,
            cv::LINE_AA);
        cv::putText(
            mosaic,
            indexText,
            textOrigin,
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            cv::Scalar(255, 255, 255),
            textThickness,
            cv::LINE_AA);
    }

    return mosaic;
}

cv::Mat buildStableContourMask(
    const cv::Size& frameSize,
    const std::vector<ObservationMaterial>& stableMaterials)
{
    cv::Mat stableMask = cv::Mat::zeros(frameSize, CV_8UC1);

    for (const ObservationMaterial& material : stableMaterials)
    {
        if (material.contour.size() < 3)
        {
            continue;
        }

        const std::vector<std::vector<cv::Point>> contours{material.contour};
        cv::drawContours(stableMask, contours, -1, cv::Scalar(255), cv::FILLED, cv::LINE_8);
    }

    return stableMask;
}

bool buildColorCompletedMaterial(
    const cv::Mat& colorBgr,
    const cv::Mat& colorEdges,
    const cv::Mat& stableUnionMask,
    const ObservationMaterial& material,
    const SegmentationConfig& config,
    ObservationMaterial& completedMaterial)
{
    if (colorBgr.empty() || colorEdges.empty() || stableUnionMask.empty() || material.contour.size() < 3)
    {
        return false;
    }

    const cv::Rect frameRect(0, 0, colorBgr.cols, colorBgr.rows);
    const cv::Rect sourceBounds = cv::boundingRect(material.contour) & frameRect;
    if (sourceBounds.empty())
    {
        return false;
    }

    const cv::Rect roi = expandedRect(
        sourceBounds,
        config.colorContourCompletionPaddingPixels,
        colorBgr.size());
    if (roi.empty())
    {
        return false;
    }

    const cv::Mat originalMask = contourMaskInRoi(material, roi);
    const int originalArea = cv::countNonZero(originalMask);
    if (originalArea < config.minAreaPixels)
    {
        return false;
    }

    cv::Mat searchMask = dilateMask(
        originalMask,
        config.colorContourCompletionPaddingPixels * 2 + 1);
    cv::Mat otherStableMask = stableUnionMask(roi).clone();
    otherStableMask.setTo(0, originalMask);
    cv::Mat otherGuardMask = dilateMask(
        otherStableMask,
        config.colorContourCompletionOtherGuardPixels * 2 + 1);
    otherGuardMask.setTo(0, originalMask);
    searchMask.setTo(0, otherGuardMask);
    cv::bitwise_or(searchMask, originalMask, searchMask);
    if (cv::countNonZero(searchMask) == 0)
    {
        return false;
    }

    cv::Mat sureForeground;
    const int erodeSize = std::max(3, std::min(9, (config.colorContourCompletionClosePixels | 1)));
    const cv::Mat erodeKernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(erodeSize, erodeSize));
    cv::erode(originalMask, sureForeground, erodeKernel);
    if (cv::countNonZero(sureForeground) == 0)
    {
        sureForeground = originalMask.clone();
    }

    cv::Mat edgeBarrier = colorEdges(roi).clone();
    if (cv::countNonZero(edgeBarrier) == 0)
    {
        return false;
    }
    edgeBarrier = dilateMask(edgeBarrier, 3);

    cv::Mat allowedMask = searchMask.clone();
    allowedMask.setTo(0, edgeBarrier);
    allowedMask.setTo(0, otherGuardMask);
    cv::bitwise_or(allowedMask, sureForeground, allowedMask);

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int labelCount =
        cv::connectedComponentsWithStats(allowedMask, labels, stats, centroids, 8, CV_32S);
    cv::Mat foregroundMask = cv::Mat::zeros(roi.size(), CV_8UC1);
    const int minOverlap = std::max(6, std::min(80, originalArea / 250));
    for (int label = 1; label < labelCount; ++label)
    {
        cv::Mat componentMask;
        cv::compare(labels, label, componentMask, cv::CMP_EQ);
        cv::Mat overlapMask;
        cv::bitwise_and(componentMask, originalMask, overlapMask);
        if (cv::countNonZero(overlapMask) >= minOverlap)
        {
            foregroundMask.setTo(255, componentMask);
        }
    }
    cv::bitwise_or(foregroundMask, sureForeground, foregroundMask);

    if (config.colorContourCompletionClosePixels > 1)
    {
        const int closeSize = config.colorContourCompletionClosePixels | 1;
        const cv::Mat closeKernel = cv::getStructuringElement(
            cv::MORPH_ELLIPSE,
            cv::Size(closeSize, closeSize));
        cv::morphologyEx(foregroundMask, foregroundMask, cv::MORPH_CLOSE, closeKernel);
        cv::bitwise_and(foregroundMask, searchMask, foregroundMask);
        foregroundMask.setTo(0, otherGuardMask);
        cv::bitwise_or(foregroundMask, originalMask, foregroundMask);
    }

    cv::Mat componentLabels;
    cv::Mat componentStats;
    cv::Mat componentCentroids;
    const int componentLabelCount =
        cv::connectedComponentsWithStats(
            foregroundMask,
            componentLabels,
            componentStats,
            componentCentroids,
            8,
            CV_32S);
    int bestLabel = -1;
    int bestOverlap = 0;
    int bestArea = 0;
    for (int label = 1; label < componentLabelCount; ++label)
    {
        cv::Mat componentMask;
        cv::compare(componentLabels, label, componentMask, cv::CMP_EQ);
        cv::Mat overlapMask;
        cv::bitwise_and(componentMask, originalMask, overlapMask);
        const int overlap = cv::countNonZero(overlapMask);
        const int area = componentStats.at<int>(label, cv::CC_STAT_AREA);
        if (overlap > bestOverlap || (overlap == bestOverlap && area > bestArea))
        {
            bestLabel = label;
            bestOverlap = overlap;
            bestArea = area;
        }
    }
    if (bestLabel < 0 || bestOverlap == 0)
    {
        return false;
    }

    cv::Mat candidateMask;
    cv::compare(componentLabels, bestLabel, candidateMask, cv::CMP_EQ);
    cv::Mat otherOverlapMask;
    cv::bitwise_and(candidateMask, otherStableMask, otherOverlapMask);
    if (cv::countNonZero(otherOverlapMask) > config.colorContourCompletionMaxOtherOverlapPixels)
    {
        return false;
    }
    const int candidateArea = cv::countNonZero(candidateMask);
    if (candidateArea < config.minAreaPixels)
    {
        return false;
    }

    cv::Mat intersectionMask;
    cv::Mat unionMask;
    cv::bitwise_and(originalMask, candidateMask, intersectionMask);
    cv::bitwise_or(originalMask, candidateMask, unionMask);
    const int unionArea = cv::countNonZero(unionMask);
    const int intersectionArea = cv::countNonZero(intersectionMask);
    const double iouPercent = unionArea > 0
        ? 100.0 * static_cast<double>(intersectionArea) / static_cast<double>(unionArea)
        : 0.0;
    const double areaDeltaPercent =
        100.0 * std::abs(candidateArea - originalArea) / static_cast<double>(originalArea);

    const cv::Moments originalMoments = cv::moments(originalMask, true);
    const cv::Moments candidateMoments = cv::moments(candidateMask, true);
    if (std::abs(originalMoments.m00) <= 1e-6 || std::abs(candidateMoments.m00) <= 1e-6)
    {
        return false;
    }

    const cv::Point2d originalCenter(
        originalMoments.m10 / originalMoments.m00,
        originalMoments.m01 / originalMoments.m00);
    const cv::Point2d candidateCenter(
        candidateMoments.m10 / candidateMoments.m00,
        candidateMoments.m01 / candidateMoments.m00);
    const double centerShift = std::hypot(
        candidateCenter.x - originalCenter.x,
        candidateCenter.y - originalCenter.y);

    if (iouPercent < static_cast<double>(config.colorContourCompletionMinIouPercent) ||
        areaDeltaPercent > static_cast<double>(config.colorContourCompletionMaxAreaDeltaPercent) ||
        centerShift > static_cast<double>(config.colorContourCompletionMaxCenterShiftPixels))
    {
        return false;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(candidateMask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
    if (contours.empty())
    {
        return false;
    }

    const auto largestContourIt = std::max_element(
        contours.begin(),
        contours.end(),
        [](const auto& lhs, const auto& rhs)
        {
            return cv::contourArea(lhs) < cv::contourArea(rhs);
        });
    if (cv::contourArea(*largestContourIt) < static_cast<double>(config.minAreaPixels))
    {
        return false;
    }

    std::vector<cv::Point> preciseContour;
    const double epsilon = cv::arcLength(*largestContourIt, true) * config.contourApproxRatio;
    if (epsilon >= 0.5)
    {
        cv::approxPolyDP(*largestContourIt, preciseContour, epsilon, true);
    }
    if (preciseContour.size() < 3)
    {
        preciseContour = *largestContourIt;
    }
    if (preciseContour.size() < 3)
    {
        return false;
    }
    for (cv::Point& point : preciseContour)
    {
        point.x += roi.x;
        point.y += roi.y;
    }

    completedMaterial = material;
    completedMaterial.roi = cv::boundingRect(preciseContour) & frameRect;
    completedMaterial.center = cv::Point(
        static_cast<int>(std::lround(candidateCenter.x + roi.x)),
        static_cast<int>(std::lround(candidateCenter.y + roi.y)));
    completedMaterial.pixelCount = candidateArea;
    completedMaterial.contourArea = cv::contourArea(preciseContour);
    completedMaterial.contour = std::move(preciseContour);
    return true;
}

std::vector<ObservationMaterial> buildDisplayStableMaterials(
    const cv::Mat& colorBgr,
    const std::vector<ObservationMaterial>& stableMaterials,
    const SegmentationConfig& config,
    ColorContourCompletionStats& completionStats)
{
    completionStats = {};
    std::vector<ObservationMaterial> displayMaterials = stableMaterials;
    if (!config.colorContourCompletion)
    {
        return displayMaterials;
    }

    cv::Mat allPixelsMask(colorBgr.size(), CV_8UC1, cv::Scalar(255));
    const cv::Mat colorEdges =
        makeGrayEdgeMask(colorBgr, allPixelsMask, config.colorCannyLow, config.colorCannyHigh);
    const cv::Mat stableUnionMask = buildStableContourMask(colorBgr.size(), stableMaterials);

    for (size_t index = 0; index < displayMaterials.size(); ++index)
    {
        if (displayMaterials[index].contour.size() < 3)
        {
            continue;
        }

        ++completionStats.inputContours;
        ObservationMaterial completedMaterial;
        if (buildColorCompletedMaterial(
                colorBgr,
                colorEdges,
                stableUnionMask,
                displayMaterials[index],
                config,
                completedMaterial))
        {
            displayMaterials[index] = std::move(completedMaterial);
            ++completionStats.adoptedContours;
        }
        else
        {
            ++completionStats.rejectedContours;
        }
    }

    return displayMaterials;
}

void drawColorContourCompletionOverlay(
    cv::Mat& mosaic,
    const ColorContourCompletionStats& completionStats,
    bool enabled)
{
    if (!enabled || mosaic.empty())
    {
        return;
    }

    const std::string text =
        "P5 RGB contour completion adopted=" +
        std::to_string(completionStats.adoptedContours) +
        " rejected=" + std::to_string(completionStats.rejectedContours) +
        " unknown=black";
    cv::putText(
        mosaic,
        text,
        cv::Point(10, 24),
        cv::FONT_HERSHEY_SIMPLEX,
        0.46,
        cv::Scalar(0, 0, 0),
        3,
        cv::LINE_AA);
    cv::putText(
        mosaic,
        text,
        cv::Point(10, 24),
        cv::FONT_HERSHEY_SIMPLEX,
        0.46,
        cv::Scalar(255, 255, 255),
        1,
        cv::LINE_AA);
}

cv::Mat buildStableContourColorMosaic(
    const cv::Mat& colorBgr,
    const std::vector<ObservationMaterial>& stableMaterials)
{
    cv::Mat mosaic = cv::Mat::zeros(colorBgr.size(), colorBgr.type());
    cv::Mat stableMask = buildStableContourMask(colorBgr.size(), stableMaterials);
    colorBgr.copyTo(mosaic, stableMask);

    for (size_t index = 0; index < stableMaterials.size(); ++index)
    {
        const ObservationMaterial& material = stableMaterials[index];
        if (material.contour.size() < 3)
        {
            continue;
        }

        const std::vector<std::vector<cv::Point>> contours{material.contour};
        cv::drawContours(mosaic, contours, -1, palette(index), 1, cv::LINE_AA);
    }

    cv::Mat outsideMask;
    cv::compare(stableMask, 0, outsideMask, cv::CMP_EQ);
    mosaic.setTo(cv::Scalar(0, 0, 0), outsideMask);
    return mosaic;
}

cv::Mat buildOutsideStableContourColorImage(
    const cv::Mat& colorBgr,
    const std::vector<ObservationMaterial>& stableMaterials)
{
    cv::Mat outsideColor = cv::Mat::zeros(colorBgr.size(), colorBgr.type());
    const cv::Mat stableMask = buildStableContourMask(colorBgr.size(), stableMaterials);

    cv::Mat outsideMask;
    cv::compare(stableMask, 0, outsideMask, cv::CMP_EQ);
    colorBgr.copyTo(outsideColor, outsideMask);
    return outsideColor;
}

std::string timestampForFilename()
{
    const auto now = std::chrono::system_clock::now();
    const std::time_t nowTime = std::chrono::system_clock::to_time_t(now);
    std::tm localTime{};
#ifdef _WIN32
    localtime_s(&localTime, &nowTime);
#else
    localtime_r(&nowTime, &localTime);
#endif

    std::ostringstream stream;
    stream << std::put_time(&localTime, "%Y%m%d_%H%M%S");
    return stream.str();
}

std::filesystem::path projectRootForOutput()
{
    std::filesystem::path current = std::filesystem::current_path();
    if (std::filesystem::exists(current / "D455.cpp"))
    {
        return current;
    }

    if ((current.filename() == "Debug" || current.filename() == "Release") &&
        current.parent_path().filename() == "x64")
    {
        const std::filesystem::path projectRoot = current.parent_path().parent_path();
        if (std::filesystem::exists(projectRoot / "D455.cpp"))
        {
            return projectRoot;
        }
    }

    for (std::filesystem::path probe = current; !probe.empty(); probe = probe.parent_path())
    {
        if (std::filesystem::exists(probe / "D455.cpp") &&
            std::filesystem::exists(probe / "D455.vcxproj"))
        {
            return probe;
        }
        if (probe == probe.root_path())
        {
            break;
        }
    }

    return current;
}

std::string defaultRecordingPath()
{
    const std::filesystem::path path =
        projectRootForOutput() /
        "recordings" /
        ("d455_record_" + timestampForFilename() + ".avi");
    return path.string();
}

std::string defaultAcceptanceRecordingPath(const std::string& label)
{
    const std::filesystem::path path =
        projectRootForOutput() /
        "recordings" /
        (label + "_" + timestampForFilename() + ".avi");
    return path.string();
}

std::string defaultAcceptanceMetricsPath(const std::string& label)
{
    std::filesystem::path path = defaultAcceptanceRecordingPath(label);
    path.replace_extension(".csv");
    return path.string();
}

std::filesystem::path resolveProjectOutputPath(
    const std::filesystem::path& requestedPath,
    const std::string& defaultExtension)
{
    std::filesystem::path output = requestedPath;
    if (output.is_relative())
    {
        output = projectRootForOutput() / output;
    }
    if (output.extension().empty())
    {
        output += defaultExtension;
    }
    return std::filesystem::absolute(output);
}

std::string companionCsvPathForVideo(const std::string& videoPath)
{
    std::filesystem::path output = resolveProjectOutputPath(videoPath, ".avi");
    output.replace_extension(".csv");
    return output.string();
}

cv::Mat ensureBgrFrame(const cv::Mat& source)
{
    if (source.empty())
    {
        return {};
    }
    if (source.type() == CV_8UC3)
    {
        return source.clone();
    }
    if (source.type() == CV_8UC1)
    {
        cv::Mat bgr;
        cv::cvtColor(source, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }
    if (source.type() == CV_8UC4)
    {
        cv::Mat bgr;
        cv::cvtColor(source, bgr, cv::COLOR_BGRA2BGR);
        return bgr;
    }

    cv::Mat converted;
    source.convertTo(converted, CV_8U);
    if (converted.channels() == 1)
    {
        cv::Mat bgr;
        cv::cvtColor(converted, bgr, cv::COLOR_GRAY2BGR);
        return bgr;
    }
    return converted;
}

cv::Mat buildRecordingFrame(const std::vector<cv::Mat>& views)
{
    if (views.empty())
    {
        return {};
    }

    std::vector<cv::Mat> panels;
    panels.reserve(views.size());
    for (const cv::Mat& view : views)
    {
        cv::Mat panel = ensureBgrFrame(view);
        if (panel.empty())
        {
            return {};
        }
        panels.push_back(panel);
    }

    const cv::Size panelSize = panels.front().size();
    for (size_t index = 1; index < panels.size(); ++index)
    {
        if (panels[index].size() != panelSize)
        {
            cv::resize(panels[index], panels[index], panelSize, 0.0, 0.0, cv::INTER_AREA);
        }
    }

    const int panelCount = static_cast<int>(panels.size());
    cv::Mat frame(panelSize.height, panelSize.width * panelCount, CV_8UC3, cv::Scalar(0, 0, 0));
    for (int index = 0; index < panelCount; ++index)
    {
        panels[static_cast<size_t>(index)].copyTo(
            frame(cv::Rect(panelSize.width * index, 0, panelSize.width, panelSize.height)));
    }
    return frame;
}

class VideoRecorder
{
public:
    explicit VideoRecorder(const VideoRecordingConfig& config)
        : config_(config)
    {
    }

    ~VideoRecorder()
    {
        close();
    }

    void write(const cv::Mat& frame)
    {
        if (!config_.enabled || frame.empty())
        {
            return;
        }
        const int64_t submittedFrame = submittedFrames_++;
        if (submittedFrame % static_cast<int64_t>(config_.everyN) != 0)
        {
            return;
        }

        cv::Mat bgrFrame = ensureBgrFrame(frame);
        if (bgrFrame.empty())
        {
            return;
        }
        if (config_.scalePercent < 100)
        {
            cv::Mat scaledFrame;
            const double scale = static_cast<double>(config_.scalePercent) / 100.0;
            cv::resize(bgrFrame, scaledFrame, cv::Size(), scale, scale, cv::INTER_AREA);
            bgrFrame = std::move(scaledFrame);
        }

        if (!writer_.isOpened())
        {
            open(bgrFrame.size());
        }
        if (bgrFrame.size() != frameSize_)
        {
            cv::resize(bgrFrame, bgrFrame, frameSize_, 0.0, 0.0, cv::INTER_AREA);
        }

        writer_.write(bgrFrame);
        ++writtenFrames_;
    }

    void close()
    {
        if (writer_.isOpened())
        {
            writer_.release();
            std::cout << "Recording saved: " << outputPath_
                << " frames=" << writtenFrames_ << '\n';
        }
    }

private:
    void open(const cv::Size& frameSize)
    {
        const std::filesystem::path output = resolveProjectOutputPath(
            config_.outputPath.empty()
                ? std::filesystem::path(defaultRecordingPath())
                : std::filesystem::path(config_.outputPath),
            ".avi");
        if (output.has_parent_path())
        {
            std::filesystem::create_directories(output.parent_path());
        }

        frameSize_ = frameSize;
        outputPath_ = output.string();
        bool opened = writer_.open(
            outputPath_,
            cv::CAP_OPENCV_MJPEG,
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            static_cast<double>(config_.fps),
            frameSize_,
            true);
        if (!opened)
        {
            opened = writer_.open(
                outputPath_,
                cv::CAP_FFMPEG,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                static_cast<double>(config_.fps),
                frameSize_,
                true);
        }
        if (!opened)
        {
            opened = writer_.open(
                outputPath_,
                cv::CAP_MSMF,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                static_cast<double>(config_.fps),
                frameSize_,
                true);
        }
        if (!opened)
        {
            opened = writer_.open(
                outputPath_,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                static_cast<double>(config_.fps),
                frameSize_,
                true);
        }
        if (!opened || !writer_.isOpened())
        {
            throw std::runtime_error("Failed to open video recording file: " + outputPath_);
        }

        std::cout << "Recording video: " << outputPath_
            << " fps=" << config_.fps
            << " every=" << config_.everyN
            << " scale=" << config_.scalePercent << "%"
            << " size=" << frameSize_.width << "x" << frameSize_.height << '\n';
    }

    VideoRecordingConfig config_;
    cv::VideoWriter writer_;
    cv::Size frameSize_;
    std::string outputPath_;
    int64_t submittedFrames_ = 0;
    int64_t writtenFrames_ = 0;
};

std::vector<cv::Point> shiftedContourForRoi(
    const std::vector<cv::Point>& contour,
    const cv::Rect& roi)
{
    std::vector<cv::Point> shifted;
    shifted.reserve(contour.size());
    for (const cv::Point& point : contour)
    {
        shifted.emplace_back(point.x - roi.x, point.y - roi.y);
    }
    return shifted;
}

cv::Rect contourBoundsInFrame(
    const ObservationMaterial& material,
    const cv::Size& frameSize)
{
    if (material.contour.empty())
    {
        return {};
    }

    const cv::Rect frameRect(0, 0, frameSize.width, frameSize.height);
    return cv::boundingRect(material.contour) & frameRect;
}

double contourMaskIou(
    const ObservationMaterial& current,
    const ObservationMaterial& previous,
    const cv::Size& frameSize)
{
    const cv::Rect currentBounds = contourBoundsInFrame(current, frameSize);
    const cv::Rect previousBounds = contourBoundsInFrame(previous, frameSize);
    if (currentBounds.empty() || previousBounds.empty())
    {
        return 0.0;
    }

    const cv::Rect roi = (currentBounds | previousBounds) & cv::Rect(0, 0, frameSize.width, frameSize.height);
    if (roi.empty())
    {
        return 0.0;
    }

    cv::Mat currentMask = cv::Mat::zeros(roi.size(), CV_8UC1);
    cv::Mat previousMask = cv::Mat::zeros(roi.size(), CV_8UC1);
    const std::vector<std::vector<cv::Point>> currentContour{shiftedContourForRoi(current.contour, roi)};
    const std::vector<std::vector<cv::Point>> previousContour{shiftedContourForRoi(previous.contour, roi)};
    cv::drawContours(currentMask, currentContour, -1, cv::Scalar(255), cv::FILLED, cv::LINE_8);
    cv::drawContours(previousMask, previousContour, -1, cv::Scalar(255), cv::FILLED, cv::LINE_8);

    cv::Mat intersectionMask;
    cv::Mat unionMask;
    cv::bitwise_and(currentMask, previousMask, intersectionMask);
    cv::bitwise_or(currentMask, previousMask, unionMask);
    const int unionPixels = cv::countNonZero(unionMask);
    if (unionPixels == 0)
    {
        return 0.0;
    }
    return static_cast<double>(cv::countNonZero(intersectionMask)) / static_cast<double>(unionPixels);
}

double contourBoundaryDistancePx(
    const ObservationMaterial& current,
    const ObservationMaterial& previous,
    const cv::Size& frameSize)
{
    const cv::Rect currentBounds = contourBoundsInFrame(current, frameSize);
    const cv::Rect previousBounds = contourBoundsInFrame(previous, frameSize);
    if (currentBounds.empty() || previousBounds.empty() || current.contour.empty())
    {
        return 0.0;
    }

    const cv::Rect roi = (currentBounds | previousBounds) & cv::Rect(0, 0, frameSize.width, frameSize.height);
    if (roi.empty())
    {
        return 0.0;
    }

    cv::Mat distanceSource(roi.size(), CV_8UC1, cv::Scalar(255));
    const std::vector<std::vector<cv::Point>> previousContour{shiftedContourForRoi(previous.contour, roi)};
    cv::drawContours(distanceSource, previousContour, -1, cv::Scalar(0), 1, cv::LINE_8);

    cv::Mat distance;
    cv::distanceTransform(distanceSource, distance, cv::DIST_L2, 3);

    double totalDistance = 0.0;
    int sampleCount = 0;
    for (const cv::Point& point : current.contour)
    {
        const int x = point.x - roi.x;
        const int y = point.y - roi.y;
        if (x < 0 || x >= distance.cols || y < 0 || y >= distance.rows)
        {
            continue;
        }

        totalDistance += static_cast<double>(distance.at<float>(y, x));
        ++sampleCount;
    }

    return sampleCount == 0 ? 0.0 : totalDistance / static_cast<double>(sampleCount);
}

class AcceptanceMetricsWriter
{
public:
    explicit AcceptanceMetricsWriter(const AcceptanceMetricsConfig& config)
        : config_(config)
    {
    }

    ~AcceptanceMetricsWriter()
    {
        close();
    }

    void write(
        uint64_t frameId,
        double frameMs,
        int candidateCount,
        int anchorSupportedCount,
        const std::vector<ObservationMaterial>& stableMaterials,
        const cv::Mat& stableMask,
        const BoundaryAnalysis& boundaryAnalysis,
        const CueSelectionSummary& cueSummary,
        const IndoorPlaneAnalysis& indoorPlaneAnalysis,
        int anchorCount,
        int anchorPointsOnCandidates,
        const ColorContourCompletionStats& completionStats,
        const FrameTimingStats& timingStats)
    {
        if (!config_.enabled)
        {
            return;
        }
        if (!stream_.is_open())
        {
            open();
        }

        const int framePixels = std::max(1, stableMask.rows * stableMask.cols);
        const int stablePixels = cv::countNonZero(stableMask);
        const double stableAreaPercent = 100.0 * static_cast<double>(stablePixels) / static_cast<double>(framePixels);
        const double blackAreaPercent = 100.0 - stableAreaPercent;

        int matchedStableCount = 0;
        double iouSum = 0.0;
        double boundaryJitterSum = 0.0;
        for (const ObservationMaterial& material : stableMaterials)
        {
            const auto previous = previousStableById_.find(material.observationId);
            if (previous == previousStableById_.end())
            {
                continue;
            }

            ++matchedStableCount;
            iouSum += contourMaskIou(material, previous->second, stableMask.size());
            boundaryJitterSum += contourBoundaryDistancePx(material, previous->second, stableMask.size());
        }

        const double averageIou = matchedStableCount == 0
            ? 0.0
            : iouSum / static_cast<double>(matchedStableCount);
        const double boundaryJitterPx = matchedStableCount == 0
            ? 0.0
            : boundaryJitterSum / static_cast<double>(matchedStableCount);

        stream_
            << frameId << ','
            << std::fixed << std::setprecision(3) << frameMs << ','
            << candidateCount << ','
            << anchorSupportedCount << ','
            << stableMaterials.size() << ','
            << matchedStableCount << ','
            << std::setprecision(3) << stableAreaPercent << ','
            << std::setprecision(3) << blackAreaPercent << ','
            << std::setprecision(5) << averageIou << ','
            << std::setprecision(3) << boundaryJitterPx << ','
            << maskPixelCount(boundaryAnalysis.depthStepEdges) << ','
            << maskPixelCount(boundaryAnalysis.depthHoleEdges) << ','
            << maskPixelCount(boundaryAnalysis.colorEdges) << ','
            << maskPixelCount(boundaryAnalysis.depthSupportedColorEdges) << ','
            << maskPixelCount(boundaryAnalysis.depthConfirmedColorEdges) << ','
            << maskPixelCount(boundaryAnalysis.splitBoundaryMask) << ','
            << cueSummary.inputCandidates << ','
            << cueSummary.acceptedCandidates << ','
            << cueSummary.rejectedTextureCandidates << ','
            << cueSummary.acceptedByDepth << ','
            << cueSummary.acceptedByRgb << ','
            << cueSummary.acceptedByAnchor << ','
            << cueSummary.acceptedByFallback << ','
            << std::setprecision(3) << (cueSummary.inputCandidates == 0
                ? 0.0
                : cueSummary.textureOnlyPercentSum / static_cast<double>(cueSummary.inputCandidates)) << ','
            << indoorPlaneAnalysis.structuralPlanePixels << ','
            << indoorPlaneAnalysis.wallPixels << ','
            << indoorPlaneAnalysis.ceilingPixels << ','
            << indoorPlaneAnalysis.supportPixels << ','
            << indoorPlaneAnalysis.components << ','
            << (indoorPlaneAnalysis.reusedFromCache ? 1 : 0) << ','
            << anchorCount << ','
            << anchorPointsOnCandidates << ','
            << completionStats.inputContours << ','
            << completionStats.adoptedContours << ','
            << completionStats.rejectedContours << ','
            << std::setprecision(3)
            << timingStats.depthPostMs << ','
            << timingStats.frameConvertMs << ','
            << timingStats.grayPrepareMs << ','
            << timingStats.anchorMs << ','
            << timingStats.boundaryMs << ','
            << timingStats.extractMs << ','
            << timingStats.pclMs << ','
            << timingStats.calibrateMs << ','
            << timingStats.cueMs << ','
            << timingStats.trackerMs << ','
            << timingStats.supportMs << ','
            << timingStats.renderMs << ','
            << timingStats.completionMs << ','
            << timingStats.diagnosticsMs << ','
            << timingStats.displayMs << ','
            << timingStats.recordMs << '\n';

        previousStableById_.clear();
        for (const ObservationMaterial& material : stableMaterials)
        {
            previousStableById_[material.observationId] = material;
        }
    }

    void close()
    {
        if (stream_.is_open())
        {
            stream_.close();
            std::cout << "Acceptance metrics saved: " << outputPath_ << '\n';
        }
    }

private:
    void open()
    {
        const std::filesystem::path output = resolveProjectOutputPath(
            config_.csvPath.empty()
                ? std::filesystem::path(defaultAcceptanceMetricsPath(config_.label))
                : std::filesystem::path(config_.csvPath),
            ".csv");
        if (output.has_parent_path())
        {
            std::filesystem::create_directories(output.parent_path());
        }

        outputPath_ = output.string();
        stream_.open(outputPath_, std::ios::out | std::ios::trunc);
        if (!stream_.is_open())
        {
            throw std::runtime_error("Failed to open acceptance metrics file: " + outputPath_);
        }

        stream_
            << "frame_index,frame_ms,candidate_count,anchor_supported_count,stable_count,"
            << "matched_stable_count,stable_area_percent,black_area_percent,avg_contour_iou,"
            << "boundary_jitter_px,depth_step_edge_px,depth_hole_edge_px,gray_edge_px,"
            << "gray_depth_supported_edge_px,gray_depth_confirmed_edge_px,split_boundary_px,"
            << "cue_input_count,cue_accepted_count,cue_rejected_texture_count,"
            << "cue_accepted_by_depth,cue_accepted_by_gray,cue_accepted_by_anchor,"
            << "cue_accepted_by_fallback,cue_mean_texture_only_percent,"
            << "indoor_plane_px,indoor_wall_px,indoor_ceiling_px,indoor_support_px,"
            << "indoor_plane_components,indoor_plane_cached,"
            << "anchor_count,anchor_points_on_candidates,"
            << "color_completion_input,color_completion_adopted,color_completion_rejected,"
            << "depth_post_ms,frame_convert_ms,gray_prepare_ms,anchor_ms,boundary_ms,"
            << "extract_ms,pcl_ms,calibrate_ms,cue_ms,tracker_ms,support_ms,"
            << "render_ms,completion_ms,diagnostics_ms,display_ms,record_ms\n";
        std::cout << "Acceptance metrics: " << outputPath_ << '\n';
    }

    AcceptanceMetricsConfig config_;
    std::ofstream stream_;
    std::string outputPath_;
    std::map<uint64_t, ObservationMaterial> previousStableById_;
};

cv::Mat normalizeFloat01(const cv::Mat& source)
{
    cv::Mat source32;
    source.convertTo(source32, CV_32F);
    double minValue = 0.0;
    double maxValue = 0.0;
    cv::minMaxLoc(source32, &minValue, &maxValue);
    if (!std::isfinite(minValue) || !std::isfinite(maxValue) || maxValue - minValue < 1e-6)
    {
        return cv::Mat::zeros(source.size(), CV_32F);
    }

    cv::Mat normalized;
    source32.convertTo(
        normalized,
        CV_32F,
        1.0 / (maxValue - minValue),
        -minValue / (maxValue - minValue));
    return normalized;
}

cv::Mat estimateHeuristicRelativeInverseDepth(const cv::Mat& colorBgr)
{
    cv::Mat gray8;
    cv::cvtColor(colorBgr, gray8, cv::COLOR_BGR2GRAY);

    cv::Mat gray32;
    gray8.convertTo(gray32, CV_32F, 1.0 / 255.0);

    cv::Mat gradX;
    cv::Mat gradY;
    cv::Sobel(gray32, gradX, CV_32F, 1, 0, 3);
    cv::Sobel(gray32, gradY, CV_32F, 0, 1, 3);
    cv::Mat gradientMagnitude;
    cv::magnitude(gradX, gradY, gradientMagnitude);
    cv::Mat gradientCue = normalizeFloat01(gradientMagnitude);

    cv::Mat hsv;
    cv::cvtColor(colorBgr, hsv, cv::COLOR_BGR2HSV);
    std::vector<cv::Mat> hsvChannels;
    cv::split(hsv, hsvChannels);
    cv::Mat saturationCue;
    hsvChannels[1].convertTo(saturationCue, CV_32F, 1.0 / 255.0);

    cv::Mat verticalCue(colorBgr.rows, colorBgr.cols, CV_32F);
    const float rowScale = colorBgr.rows <= 1 ? 0.0f : 1.0f / static_cast<float>(colorBgr.rows - 1);
    for (int y = 0; y < colorBgr.rows; ++y)
    {
        verticalCue.row(y).setTo(static_cast<float>(y) * rowScale);
    }

    const cv::Mat darkCue = cv::Scalar(1.0f) - gray32;
    cv::Mat relativeInverseDepth =
        verticalCue * 0.50f +
        gradientCue * 0.25f +
        saturationCue * 0.15f +
        darkCue * 0.10f;
    cv::GaussianBlur(relativeInverseDepth, relativeInverseDepth, cv::Size(9, 9), 0.0);
    return relativeInverseDepth;
}

class RgbDepthEstimator
{
public:
    explicit RgbDepthEstimator(const std::string& onnxPath)
    {
        if (!onnxPath.empty())
        {
            net_ = cv::dnn::readNetFromONNX(onnxPath);
            if (net_.empty())
            {
                throw std::runtime_error("Failed to load RGB depth ONNX model: " + onnxPath);
            }

            methodLabel_ = "midas_onnx";
        }
        else
        {
            methodLabel_ = "rgb_heuristic";
        }
    }

    const std::string& methodLabel() const
    {
        return methodLabel_;
    }

    cv::Mat estimateRelativeInverseDepth(const cv::Mat& colorBgr)
    {
        if (net_.empty())
        {
            return estimateHeuristicRelativeInverseDepth(colorBgr);
        }

        cv::Mat resizedBgr;
        cv::resize(colorBgr, resizedBgr, cv::Size(256, 256), 0.0, 0.0, cv::INTER_AREA);

        cv::Mat rgb32;
        cv::cvtColor(resizedBgr, rgb32, cv::COLOR_BGR2RGB);
        rgb32.convertTo(rgb32, CV_32F, 1.0 / 255.0);

        std::vector<cv::Mat> channels;
        cv::split(rgb32, channels);
        const std::array<float, 3> mean{0.485f, 0.456f, 0.406f};
        const std::array<float, 3> stddev{0.229f, 0.224f, 0.225f};
        for (int channel = 0; channel < 3; ++channel)
        {
            channels[channel] = (channels[channel] - mean[channel]) / stddev[channel];
        }
        cv::merge(channels, rgb32);

        cv::Mat blob = cv::dnn::blobFromImage(rgb32);
        net_.setInput(blob);
        cv::Mat output = net_.forward();

        cv::Mat output2d;
        if (output.dims == 4)
        {
            output2d = cv::Mat(output.size[2], output.size[3], CV_32F, output.ptr<float>()).clone();
        }
        else if (output.dims == 3)
        {
            output2d = cv::Mat(output.size[1], output.size[2], CV_32F, output.ptr<float>()).clone();
        }
        else if (output.dims == 2)
        {
            output2d = output.clone();
        }
        else
        {
            throw std::runtime_error("Unexpected RGB depth ONNX output dimensions");
        }

        cv::Mat relativeInverseDepth;
        cv::resize(output2d, relativeInverseDepth, colorBgr.size(), 0.0, 0.0, cv::INTER_CUBIC);
        cv::patchNaNs(relativeInverseDepth, 0.0);
        return relativeInverseDepth;
    }

private:
    cv::dnn::Net net_;
    std::string methodLabel_;
};

struct RgbDepthFitSample
{
    double cue = 0.0;
    double referenceInverseDepth = 0.0;
};

int percentileFromSorted(const std::vector<int>& sortedValues, double percentile);

struct RgbDepthAnchorStats
{
    int64_t gridCandidates = 0;
    int64_t inRange = 0;
    int64_t rawFilteredConsistent = 0;
    int64_t edgeSafe = 0;
    int64_t neighborhoodStable = 0;
    int64_t selected = 0;
    int64_t train = 0;
    int64_t holdout = 0;
    std::vector<int> selectedDepthsMm;

    void merge(const RgbDepthAnchorStats& other)
    {
        gridCandidates += other.gridCandidates;
        inRange += other.inRange;
        rawFilteredConsistent += other.rawFilteredConsistent;
        edgeSafe += other.edgeSafe;
        neighborhoodStable += other.neighborhoodStable;
        selected += other.selected;
        train += other.train;
        holdout += other.holdout;
        selectedDepthsMm.insert(
            selectedDepthsMm.end(),
            other.selectedDepthsMm.begin(),
            other.selectedDepthsMm.end());
    }
};

bool isHoldoutAnchorPoint(int x, int y, int holdoutPercent)
{
    if (holdoutPercent <= 0)
    {
        return false;
    }

    const uint32_t hash =
        static_cast<uint32_t>(x) * 73856093u ^
        static_cast<uint32_t>(y) * 19349663u;
    return static_cast<int>(hash % 100u) < holdoutPercent;
}

bool isNeighborhoodDepthStable(
    const cv::Mat& depth16,
    int centerX,
    int centerY,
    int radius,
    int minDepthUnits,
    int maxDepthUnits,
    int maxRangeUnits)
{
    int validCount = 0;
    int minUnits = std::numeric_limits<int>::max();
    int maxUnits = 0;

    for (int y = centerY - radius; y <= centerY + radius; ++y)
    {
        const uint16_t* depthRow = depth16.ptr<uint16_t>(y);
        for (int x = centerX - radius; x <= centerX + radius; ++x)
        {
            const int depthUnits = depthRow[x];
            if (depthUnits < minDepthUnits || depthUnits > maxDepthUnits)
            {
                continue;
            }

            ++validCount;
            minUnits = std::min(minUnits, depthUnits);
            maxUnits = std::max(maxUnits, depthUnits);
        }
    }

    const int kernelSide = radius * 2 + 1;
    const int minValidCount = std::max(3, kernelSide * kernelSide * 2 / 3);
    return validCount >= minValidCount && maxUnits - minUnits <= maxRangeUnits;
}

cv::Mat makeReliableDepthAnchorMask(
    const cv::Mat& rawDepth16,
    const cv::Mat& filteredDepth16,
    float depthScale,
    const SegmentationConfig& config,
    const RgbDepthAccuracyConfig& rgbDepthConfig,
    RgbDepthAnchorStats& stats)
{
    cv::Mat anchorMask = cv::Mat::zeros(filteredDepth16.size(), CV_8UC1);
    cv::Mat depthEdges = makeDepthEdgeMask(filteredDepth16, config, depthScale);
    if (rgbDepthConfig.anchorEdgeDilatePixels > 0)
    {
        depthEdges = dilateMask(depthEdges, rgbDepthConfig.anchorEdgeDilatePixels);
    }

    const int minDepthUnits = depthUnitsFromMm(config.minDepthMm, depthScale);
    const int maxDepthUnits = depthUnitsFromMm(config.maxDepthMm, depthScale);
    const int maxRawFilteredGapUnits = depthUnitsFromMm(
        std::max(1, rgbDepthConfig.anchorMaxRawFilteredGapMm),
        depthScale);
    const int maxNeighborRangeUnits = depthUnitsFromMm(
        std::max(1, rgbDepthConfig.anchorMaxNeighborRangeMm),
        depthScale);
    const int radius = rgbDepthConfig.anchorNeighborhoodPixels;

    for (int y = radius; y < filteredDepth16.rows - radius; y += rgbDepthConfig.anchorStepPixels)
    {
        const uint16_t* rawRow = rawDepth16.ptr<uint16_t>(y);
        const uint16_t* filteredRow = filteredDepth16.ptr<uint16_t>(y);
        const uint8_t* edgeRow = depthEdges.ptr<uint8_t>(y);
        uint8_t* anchorRow = anchorMask.ptr<uint8_t>(y);
        for (int x = radius; x < filteredDepth16.cols - radius; x += rgbDepthConfig.anchorStepPixels)
        {
            ++stats.gridCandidates;
            const int rawDepth = rawRow[x];
            const int filteredDepth = filteredRow[x];
            if (rawDepth < minDepthUnits || rawDepth > maxDepthUnits ||
                filteredDepth < minDepthUnits || filteredDepth > maxDepthUnits)
            {
                continue;
            }
            ++stats.inRange;

            if (std::abs(rawDepth - filteredDepth) > maxRawFilteredGapUnits)
            {
                continue;
            }
            ++stats.rawFilteredConsistent;

            if (edgeRow[x] != 0)
            {
                continue;
            }
            ++stats.edgeSafe;

            if (!isNeighborhoodDepthStable(
                    filteredDepth16,
                    x,
                    y,
                    radius,
                    minDepthUnits,
                    maxDepthUnits,
                    maxNeighborRangeUnits))
            {
                continue;
            }
            ++stats.neighborhoodStable;

            anchorRow[x] = 255;
            ++stats.selected;
            stats.selectedDepthsMm.push_back(
                static_cast<int>(filteredDepth * depthScale * 1000.0f + 0.5f));
        }
    }

    return anchorMask;
}

void splitAnchorMask(
    const cv::Mat& anchorMask,
    int holdoutPercent,
    cv::Mat& trainMask,
    cv::Mat& holdoutMask,
    RgbDepthAnchorStats& stats)
{
    trainMask = cv::Mat::zeros(anchorMask.size(), CV_8UC1);
    holdoutMask = cv::Mat::zeros(anchorMask.size(), CV_8UC1);

    for (int y = 0; y < anchorMask.rows; ++y)
    {
        const uint8_t* anchorRow = anchorMask.ptr<uint8_t>(y);
        uint8_t* trainRow = trainMask.ptr<uint8_t>(y);
        uint8_t* holdoutRow = holdoutMask.ptr<uint8_t>(y);
        for (int x = 0; x < anchorMask.cols; ++x)
        {
            if (anchorRow[x] == 0)
            {
                continue;
            }

            if (isHoldoutAnchorPoint(x, y, holdoutPercent))
            {
                holdoutRow[x] = 255;
                ++stats.holdout;
            }
            else
            {
                trainRow[x] = 255;
                ++stats.train;
            }
        }
    }
}

void printRgbDepthAnchorStats(
    const std::string& label,
    const RgbDepthAnchorStats& stats)
{
    std::vector<int> depths = stats.selectedDepthsMm;
    std::sort(depths.begin(), depths.end());

    auto percent = [](int64_t count, int64_t total)
    {
        return total == 0 ? 0.0 : count * 100.0 / static_cast<double>(total);
    };

    std::cout << label
        << " grid_candidates=" << stats.gridCandidates
        << " in_range=" << stats.inRange << " (" << std::fixed << std::setprecision(2)
        << percent(stats.inRange, stats.gridCandidates) << "%)"
        << " raw_filtered_ok=" << stats.rawFilteredConsistent << " ("
        << percent(stats.rawFilteredConsistent, stats.gridCandidates) << "%)"
        << " edge_safe=" << stats.edgeSafe << " ("
        << percent(stats.edgeSafe, stats.gridCandidates) << "%)"
        << " neighborhood_stable=" << stats.neighborhoodStable << " ("
        << percent(stats.neighborhoodStable, stats.gridCandidates) << "%)"
        << " selected=" << stats.selected << " ("
        << percent(stats.selected, stats.gridCandidates) << "%)"
        << " train=" << stats.train
        << " holdout=" << stats.holdout << '\n';
    std::cout << "  selected_depth_mm_percentiles p50=" << percentileFromSorted(depths, 0.50)
        << " p90=" << percentileFromSorted(depths, 0.90)
        << " p95=" << percentileFromSorted(depths, 0.95)
        << " p99=" << percentileFromSorted(depths, 0.99) << '\n';
}

struct StableContourSupport
{
    int anchorCount = 0;
    int meanAnchorDepthMm = 0;
    cv::Point anchorCenter;
};

StableContourSupport measureStableContourSupport(
    const ObservationMaterial& material,
    const cv::Mat& anchorMask,
    const cv::Mat& depth16,
    float depthScale)
{
    StableContourSupport support;
    const cv::Rect imageBounds(0, 0, anchorMask.cols, anchorMask.rows);
    const cv::Rect roi = material.roi & imageBounds;
    if (roi.empty() || material.contour.size() < 3)
    {
        return support;
    }

    const cv::Mat localContourMask = contourMaskInRoi(material, roi);
    int64_t sumX = 0;
    int64_t sumY = 0;
    int64_t sumDepthUnits = 0;
    for (int localY = 0; localY < roi.height; ++localY)
    {
        const uint8_t* contourRow = localContourMask.ptr<uint8_t>(localY);
        const uint8_t* anchorRow = anchorMask.ptr<uint8_t>(roi.y + localY);
        const uint16_t* depthRow = depth16.ptr<uint16_t>(roi.y + localY);
        for (int localX = 0; localX < roi.width; ++localX)
        {
            if (contourRow[localX] == 0 || anchorRow[roi.x + localX] == 0)
            {
                continue;
            }

            const int x = roi.x + localX;
            const int y = roi.y + localY;
            ++support.anchorCount;
            sumX += x;
            sumY += y;
            sumDepthUnits += depthRow[x];
        }
    }

    if (support.anchorCount > 0)
    {
        support.anchorCenter = cv::Point(
            static_cast<int>(sumX / support.anchorCount),
            static_cast<int>(sumY / support.anchorCount));
        support.meanAnchorDepthMm = static_cast<int>(
            (sumDepthUnits / static_cast<double>(support.anchorCount)) * depthScale * 1000.0f + 0.5f);
    }

    return support;
}

std::vector<ObservationMaterial> calibrateMaterialsWithStableAnchors(
    const std::vector<ObservationMaterial>& materials,
    const cv::Mat& anchorMask,
    const cv::Mat& depth16,
    float depthScale,
    int minAnchorCount,
    int& totalAnchorPoints)
{
    std::vector<ObservationMaterial> calibrated;
    calibrated.reserve(materials.size());
    totalAnchorPoints = 0;

    for (const ObservationMaterial& material : materials)
    {
        const StableContourSupport support =
            measureStableContourSupport(material, anchorMask, depth16, depthScale);
        if (support.anchorCount < minAnchorCount)
        {
            continue;
        }

        ObservationMaterial adjusted = material;
        adjusted.center = support.anchorCenter;
        adjusted.meanDepthMm = support.meanAnchorDepthMm;
        adjusted.groupId = support.anchorCount;
        totalAnchorPoints += support.anchorCount;
        calibrated.push_back(std::move(adjusted));
    }

    return calibrated;
}

struct StableContourTrackAggregate
{
    uint64_t trackId = 0;
    int observations = 0;
    int firstFrame = 0;
    int lastFrame = 0;
    int anchorSum = 0;
    double centerXSum = 0.0;
    double centerYSum = 0.0;
    double areaSum = 0.0;
    int minDepthMm = std::numeric_limits<int>::max();
    int maxDepthMm = 0;
    double minArea = std::numeric_limits<double>::max();
    double maxArea = 0.0;
    std::vector<cv::Point> centers;
    cv::Rect lastRoi;
    bool hasLastRoi = false;
    double consecutiveIouSum = 0.0;
    int consecutiveIouCount = 0;
};

void addStableContourObservation(
    StableContourTrackAggregate& aggregate,
    const ObservationMaterial& material,
    const StableContourSupport& support,
    int frameIndex)
{
    if (aggregate.observations == 0)
    {
        aggregate.trackId = material.observationId;
        aggregate.firstFrame = frameIndex;
    }

    aggregate.lastFrame = frameIndex;
    ++aggregate.observations;
    aggregate.anchorSum += support.anchorCount;
    aggregate.centerXSum += support.anchorCenter.x;
    aggregate.centerYSum += support.anchorCenter.y;
    aggregate.areaSum += material.contourArea;
    aggregate.minArea = std::min(aggregate.minArea, material.contourArea);
    aggregate.maxArea = std::max(aggregate.maxArea, material.contourArea);
    aggregate.minDepthMm = std::min(aggregate.minDepthMm, support.meanAnchorDepthMm);
    aggregate.maxDepthMm = std::max(aggregate.maxDepthMm, support.meanAnchorDepthMm);
    aggregate.centers.push_back(support.anchorCenter);

    if (aggregate.hasLastRoi)
    {
        aggregate.consecutiveIouSum += rectIou(aggregate.lastRoi, material.roi);
        ++aggregate.consecutiveIouCount;
    }
    aggregate.lastRoi = material.roi;
    aggregate.hasLastRoi = true;
}

double maxCenterRadiusPixels(const StableContourTrackAggregate& aggregate)
{
    if (aggregate.observations == 0)
    {
        return 0.0;
    }

    const double meanX = aggregate.centerXSum / aggregate.observations;
    const double meanY = aggregate.centerYSum / aggregate.observations;
    double maxRadius = 0.0;
    for (const cv::Point& center : aggregate.centers)
    {
        const double dx = center.x - meanX;
        const double dy = center.y - meanY;
        maxRadius = std::max(maxRadius, std::sqrt(dx * dx + dy * dy));
    }

    return maxRadius;
}

double contourStabilityScore(const StableContourTrackAggregate& aggregate, int sampleFrames)
{
    if (aggregate.observations == 0)
    {
        return 0.0;
    }

    const double presenceScore = aggregate.observations * 100.0 / std::max(1, sampleFrames);
    const double meanIou = aggregate.consecutiveIouCount == 0
        ? 0.0
        : aggregate.consecutiveIouSum / aggregate.consecutiveIouCount;
    const double meanArea = aggregate.areaSum / aggregate.observations;
    const double areaRangePercent = meanArea <= 1.0
        ? 100.0
        : (aggregate.maxArea - aggregate.minArea) * 100.0 / meanArea;
    const double centerScore = std::max(0.0, 100.0 - maxCenterRadiusPixels(aggregate) * 4.0);
    const double areaScore = std::max(0.0, 100.0 - areaRangePercent);
    const double depthScore = std::max(0.0, 100.0 - (aggregate.maxDepthMm - aggregate.minDepthMm) / 3.0);

    return
        presenceScore * 0.35 +
        meanIou * 100.0 * 0.25 +
        centerScore * 0.15 +
        areaScore * 0.15 +
        depthScore * 0.10;
}

void printStableContourAggregate(
    const std::map<uint64_t, StableContourTrackAggregate>& tracks,
    int sampleFrames,
    int topCount)
{
    std::vector<const StableContourTrackAggregate*> sortedTracks;
    sortedTracks.reserve(tracks.size());
    for (const auto& [trackId, aggregate] : tracks)
    {
        (void)trackId;
        sortedTracks.push_back(&aggregate);
    }

    std::sort(
        sortedTracks.begin(),
        sortedTracks.end(),
        [sampleFrames](const StableContourTrackAggregate* lhs, const StableContourTrackAggregate* rhs)
        {
            return contourStabilityScore(*lhs, sampleFrames) > contourStabilityScore(*rhs, sampleFrames);
        });

    std::cout << "\n[stable contour track aggregate]\n";
    std::cout << "tracks=" << sortedTracks.size()
        << " sample_frames=" << sampleFrames
        << " top=" << std::min<int>(topCount, static_cast<int>(sortedTracks.size())) << '\n';

    const int count = std::min<int>(topCount, static_cast<int>(sortedTracks.size()));
    for (int index = 0; index < count; ++index)
    {
        const StableContourTrackAggregate& aggregate = *sortedTracks[static_cast<size_t>(index)];
        const double meanIou = aggregate.consecutiveIouCount == 0
            ? 0.0
            : aggregate.consecutiveIouSum / aggregate.consecutiveIouCount;
        const double meanArea = aggregate.observations == 0 ? 0.0 : aggregate.areaSum / aggregate.observations;
        const double areaRangePercent = meanArea <= 1.0
            ? 0.0
            : (aggregate.maxArea - aggregate.minArea) * 100.0 / meanArea;
        const double meanAnchors = aggregate.observations == 0
            ? 0.0
            : aggregate.anchorSum / static_cast<double>(aggregate.observations);

        std::cout << "track=" << aggregate.trackId
            << " score=" << std::fixed << std::setprecision(2)
            << contourStabilityScore(aggregate, sampleFrames)
            << " frames=" << aggregate.observations << "/" << sampleFrames
            << " frame_span=" << aggregate.firstFrame << "-" << aggregate.lastFrame
            << " mean_iou=" << meanIou
            << " center_max_radius_px=" << maxCenterRadiusPixels(aggregate)
            << " area_range_pct=" << areaRangePercent
            << " depth_range_mm=" << (aggregate.maxDepthMm - aggregate.minDepthMm)
            << " mean_anchor_points=" << meanAnchors << '\n';
    }
}

cv::Mat buildStableContourVisualization(
    const cv::Mat& colorBgr,
    const cv::Mat& anchorMask,
    const cv::Mat& depth16,
    float depthScale,
    const std::vector<ObservationMaterial>& anchorSupportedCandidates,
    const std::vector<ObservationMaterial>& stableMaterials,
    const std::map<uint64_t, StableContourTrackAggregate>& trackAggregates,
    const RgbDepthAccuracyConfig& stableConfig,
    int frameIndex)
{
    cv::Mat view;
    colorBgr.convertTo(view, -1, 0.68, 0.0);

    for (int y = 0; y < anchorMask.rows; ++y)
    {
        const uint8_t* anchorRow = anchorMask.ptr<uint8_t>(y);
        cv::Vec3b* viewRow = view.ptr<cv::Vec3b>(y);
        for (int x = 0; x < anchorMask.cols; ++x)
        {
            if (anchorRow[x] != 0)
            {
                viewRow[x] = cv::Vec3b(80, 255, 80);
            }
        }
    }

    for (const ObservationMaterial& material : anchorSupportedCandidates)
    {
        if (material.contour.size() < 3)
        {
            continue;
        }
        const std::vector<std::vector<cv::Point>> contours{material.contour};
        cv::drawContours(view, contours, -1, cv::Scalar(0, 210, 255), 1, cv::LINE_AA);
    }

    for (size_t index = 0; index < stableMaterials.size(); ++index)
    {
        const ObservationMaterial& material = stableMaterials[index];
        if (material.contour.size() < 3)
        {
            continue;
        }

        const StableContourSupport support =
            measureStableContourSupport(material, anchorMask, depth16, depthScale);
        const cv::Scalar color = palette(index);
        const std::vector<std::vector<cv::Point>> contours{material.contour};
        cv::drawContours(view, contours, -1, color, 3, cv::LINE_AA);
        cv::circle(view, support.anchorCenter, 4, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_AA);
        cv::circle(view, support.anchorCenter, 6, color, 2, cv::LINE_AA);

        std::string scoreLabel;
        const auto aggregateIt = trackAggregates.find(material.observationId);
        if (aggregateIt != trackAggregates.end())
        {
            const int score = static_cast<int>(std::round(contourStabilityScore(
                aggregateIt->second,
                std::max(1, frameIndex))));
            scoreLabel = " S" + std::to_string(score);
        }

        const std::string label =
            "T" + std::to_string(material.observationId) + scoreLabel +
            " A" + std::to_string(support.anchorCount) +
            " D" + std::to_string(support.meanAnchorDepthMm);
        cv::Point labelOrigin(support.anchorCenter.x + 8, support.anchorCenter.y - 8);
        labelOrigin.x = std::clamp(labelOrigin.x, 2, std::max(2, view.cols - 230));
        labelOrigin.y = std::clamp(labelOrigin.y, 46, std::max(46, view.rows - 4));
        cv::putText(
            view,
            label,
            labelOrigin,
            cv::FONT_HERSHEY_SIMPLEX,
            0.42,
            cv::Scalar(0, 0, 0),
            3,
            cv::LINE_AA);
        cv::putText(
            view,
            label,
            labelOrigin,
            cv::FONT_HERSHEY_SIMPLEX,
            0.42,
            cv::Scalar(255, 255, 255),
            1,
            cv::LINE_AA);
    }

    const std::string title =
        "F" + std::to_string(frameIndex) +
        " green=anchors yellow=candidates color=stable" +
        (stableConfig.stableContourVideo ? " q/Esc=exit" : "");
    cv::putText(
        view,
        title,
        cv::Point(12, 24),
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        cv::Scalar(0, 0, 0),
        3,
        cv::LINE_AA);
    cv::putText(
        view,
        title,
        cv::Point(12, 24),
        cv::FONT_HERSHEY_SIMPLEX,
        0.55,
        cv::Scalar(255, 255, 255),
        1,
        cv::LINE_AA);

    const std::string footer =
        "tracked=" + std::to_string(trackAggregates.size()) +
        " visible=" + std::to_string(stableMaterials.size()) +
        " supported=" + std::to_string(anchorSupportedCandidates.size());
    const cv::Point footerOrigin(12, std::max(24, view.rows - 14));
    cv::putText(
        view,
        footer,
        footerOrigin,
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(0, 0, 0),
        3,
        cv::LINE_AA);
    cv::putText(
        view,
        footer,
        footerOrigin,
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(255, 255, 255),
        1,
        cv::LINE_AA);

    return view;
}

bool solveInverseDepthFit(
    const std::vector<RgbDepthFitSample>& samples,
    InverseDepthCalibration& calibration)
{
    if (samples.size() < 100)
    {
        return false;
    }

    double sumX = 0.0;
    double sumY = 0.0;
    double sumXX = 0.0;
    double sumXY = 0.0;
    for (const RgbDepthFitSample& sample : samples)
    {
        sumX += sample.cue;
        sumY += sample.referenceInverseDepth;
        sumXX += sample.cue * sample.cue;
        sumXY += sample.cue * sample.referenceInverseDepth;
    }

    const double count = static_cast<double>(samples.size());
    const double denominator = count * sumXX - sumX * sumX;
    if (std::abs(denominator) < 1e-12)
    {
        return false;
    }

    calibration.slope = (count * sumXY - sumX * sumY) / denominator;
    calibration.intercept = (sumY - calibration.slope * sumX) / count;
    calibration.sampleCount = static_cast<int>(samples.size());
    calibration.valid =
        std::isfinite(calibration.intercept) &&
        std::isfinite(calibration.slope);
    return calibration.valid;
}

bool fitInverseDepthCalibration(
    const cv::Mat& relativeInverseDepth,
    const cv::Mat& referenceDepth16,
    float depthScale,
    const SegmentationConfig& config,
    int stepPixels,
    InverseDepthCalibration& calibration,
    const cv::Mat* sampleMask = nullptr)
{
    const int minDepthUnits = depthUnitsFromMm(config.minDepthMm, depthScale);
    const int maxDepthUnits = depthUnitsFromMm(config.maxDepthMm, depthScale);
    const int sampleStepPixels = sampleMask == nullptr ? stepPixels : 1;

    std::vector<RgbDepthFitSample> samples;
    samples.reserve(static_cast<size_t>(referenceDepth16.total() / std::max(1, sampleStepPixels * sampleStepPixels)));
    for (int y = 0; y < referenceDepth16.rows; y += sampleStepPixels)
    {
        const uint16_t* depthRow = referenceDepth16.ptr<uint16_t>(y);
        const float* cueRow = relativeInverseDepth.ptr<float>(y);
        const uint8_t* maskRow = sampleMask == nullptr ? nullptr : sampleMask->ptr<uint8_t>(y);
        for (int x = 0; x < referenceDepth16.cols; x += sampleStepPixels)
        {
            if (maskRow != nullptr && maskRow[x] == 0)
            {
                continue;
            }

            const uint16_t depthUnits = depthRow[x];
            if (depthUnits < minDepthUnits || depthUnits > maxDepthUnits)
            {
                continue;
            }

            const float cue = cueRow[x];
            if (!std::isfinite(cue))
            {
                continue;
            }

            const double referenceDepthMm =
                static_cast<double>(depthUnits) * static_cast<double>(depthScale) * 1000.0;
            samples.push_back({static_cast<double>(cue), 1.0 / referenceDepthMm});
        }
    }

    InverseDepthCalibration initialCalibration;
    if (!solveInverseDepthFit(samples, initialCalibration))
    {
        return false;
    }

    std::vector<double> residuals;
    residuals.reserve(samples.size());
    for (const RgbDepthFitSample& sample : samples)
    {
        residuals.push_back(std::abs(
            sample.referenceInverseDepth -
            (initialCalibration.intercept + initialCalibration.slope * sample.cue)));
    }

    std::vector<double> sortedResiduals = residuals;
    std::sort(sortedResiduals.begin(), sortedResiduals.end());
    const double trimThreshold = sortedResiduals[
        std::min(
            sortedResiduals.size() - 1,
            static_cast<size_t>((sortedResiduals.size() - 1) * 0.85 + 0.5))];

    std::vector<RgbDepthFitSample> trimmedSamples;
    trimmedSamples.reserve(samples.size());
    for (size_t index = 0; index < samples.size(); ++index)
    {
        if (residuals[index] <= trimThreshold)
        {
            trimmedSamples.push_back(samples[index]);
        }
    }

    return solveInverseDepthFit(trimmedSamples, calibration);
}

struct RgbDepthAccuracyStats
{
    int64_t referenceValidPixels = 0;
    int64_t evaluatedPixels = 0;
    double sumAbsErrorMm = 0.0;
    double sumSquaredErrorMm = 0.0;
    double sumAbsRelativeError = 0.0;
    int64_t within50Mm = 0;
    int64_t within100Mm = 0;
    int64_t within200Mm = 0;
    int64_t within500Mm = 0;
    int64_t within5Percent = 0;
    int64_t within10Percent = 0;
    int64_t within20Percent = 0;
    std::vector<int> absErrorsMm;

    void add(double predictedDepthMm, double referenceDepthMm)
    {
        const double absError = std::abs(predictedDepthMm - referenceDepthMm);
        const double relativeError = absError / std::max(1.0, referenceDepthMm);
        ++evaluatedPixels;
        sumAbsErrorMm += absError;
        sumSquaredErrorMm += absError * absError;
        sumAbsRelativeError += relativeError;
        absErrorsMm.push_back(static_cast<int>(absError + 0.5));
        if (absError <= 50.0)
        {
            ++within50Mm;
        }
        if (absError <= 100.0)
        {
            ++within100Mm;
        }
        if (absError <= 200.0)
        {
            ++within200Mm;
        }
        if (absError <= 500.0)
        {
            ++within500Mm;
        }
        if (relativeError <= 0.05)
        {
            ++within5Percent;
        }
        if (relativeError <= 0.10)
        {
            ++within10Percent;
        }
        if (relativeError <= 0.20)
        {
            ++within20Percent;
        }
    }

    void merge(const RgbDepthAccuracyStats& other)
    {
        referenceValidPixels += other.referenceValidPixels;
        evaluatedPixels += other.evaluatedPixels;
        sumAbsErrorMm += other.sumAbsErrorMm;
        sumSquaredErrorMm += other.sumSquaredErrorMm;
        sumAbsRelativeError += other.sumAbsRelativeError;
        within50Mm += other.within50Mm;
        within100Mm += other.within100Mm;
        within200Mm += other.within200Mm;
        within500Mm += other.within500Mm;
        within5Percent += other.within5Percent;
        within10Percent += other.within10Percent;
        within20Percent += other.within20Percent;
        absErrorsMm.insert(absErrorsMm.end(), other.absErrorsMm.begin(), other.absErrorsMm.end());
    }
};

RgbDepthAccuracyStats evaluateRgbDepthPrediction(
    const cv::Mat& relativeInverseDepth,
    const cv::Mat& referenceDepth16,
    float depthScale,
    const SegmentationConfig& config,
    int stepPixels,
    const InverseDepthCalibration& calibration,
    const cv::Mat* evalMask = nullptr)
{
    RgbDepthAccuracyStats stats;
    const int minDepthUnits = depthUnitsFromMm(config.minDepthMm, depthScale);
    const int maxDepthUnits = depthUnitsFromMm(config.maxDepthMm, depthScale);
    const double minInverseDepth = 1.0 / static_cast<double>(config.maxDepthMm);
    const double maxInverseDepth = 1.0 / static_cast<double>(config.minDepthMm);
    const int sampleStepPixels = evalMask == nullptr ? stepPixels : 1;

    for (int y = 0; y < referenceDepth16.rows; y += sampleStepPixels)
    {
        const uint16_t* depthRow = referenceDepth16.ptr<uint16_t>(y);
        const float* cueRow = relativeInverseDepth.ptr<float>(y);
        const uint8_t* maskRow = evalMask == nullptr ? nullptr : evalMask->ptr<uint8_t>(y);
        for (int x = 0; x < referenceDepth16.cols; x += sampleStepPixels)
        {
            if (maskRow != nullptr && maskRow[x] == 0)
            {
                continue;
            }

            const uint16_t depthUnits = depthRow[x];
            if (depthUnits < minDepthUnits || depthUnits > maxDepthUnits)
            {
                continue;
            }
            ++stats.referenceValidPixels;

            const float cue = cueRow[x];
            if (!std::isfinite(cue))
            {
                continue;
            }

            double predictedInverseDepth = calibration.intercept + calibration.slope * static_cast<double>(cue);
            if (!std::isfinite(predictedInverseDepth))
            {
                continue;
            }
            predictedInverseDepth = std::clamp(predictedInverseDepth, minInverseDepth, maxInverseDepth);

            const double predictedDepthMm = 1.0 / predictedInverseDepth;
            const double referenceDepthMm =
                static_cast<double>(depthUnits) * static_cast<double>(depthScale) * 1000.0;
            if (!std::isfinite(predictedDepthMm))
            {
                continue;
            }

            stats.add(predictedDepthMm, referenceDepthMm);
        }
    }

    return stats;
}

int percentileFromSorted(const std::vector<int>& sortedValues, double percentile)
{
    if (sortedValues.empty())
    {
        return 0;
    }

    const size_t index = std::min(
        sortedValues.size() - 1,
        static_cast<size_t>((sortedValues.size() - 1) * percentile + 0.5));
    return sortedValues[index];
}

void printRgbDepthAccuracyStats(
    const std::string& label,
    const RgbDepthAccuracyStats& stats)
{
    std::vector<int> sortedErrors = stats.absErrorsMm;
    std::sort(sortedErrors.begin(), sortedErrors.end());

    auto percentOfEvaluated = [&stats](int64_t count)
    {
        return stats.evaluatedPixels == 0 ? 0.0 : count * 100.0 / static_cast<double>(stats.evaluatedPixels);
    };
    auto percentOfReference = [&stats](int64_t count)
    {
        return stats.referenceValidPixels == 0 ? 0.0 : count * 100.0 / static_cast<double>(stats.referenceValidPixels);
    };

    const double mae = stats.evaluatedPixels == 0 ? 0.0 : stats.sumAbsErrorMm / stats.evaluatedPixels;
    const double rmse = stats.evaluatedPixels == 0 ? 0.0 : std::sqrt(stats.sumSquaredErrorMm / stats.evaluatedPixels);
    const double absRelPercent = stats.evaluatedPixels == 0 ? 0.0 : stats.sumAbsRelativeError * 100.0 / stats.evaluatedPixels;

    std::cout << label
        << " reference_valid=" << stats.referenceValidPixels
        << " evaluated=" << stats.evaluatedPixels
        << " coverage=" << std::fixed << std::setprecision(2) << percentOfReference(stats.evaluatedPixels) << "%"
        << " mae_mm=" << mae
        << " rmse_mm=" << rmse
        << " abs_rel=" << absRelPercent << "%\n";
    std::cout << "  abs_error_mm_percentiles p50=" << percentileFromSorted(sortedErrors, 0.50)
        << " p90=" << percentileFromSorted(sortedErrors, 0.90)
        << " p95=" << percentileFromSorted(sortedErrors, 0.95)
        << " p99=" << percentileFromSorted(sortedErrors, 0.99) << '\n';
    std::cout << "  within_mm <=50=" << percentOfEvaluated(stats.within50Mm)
        << "% <=100=" << percentOfEvaluated(stats.within100Mm)
        << "% <=200=" << percentOfEvaluated(stats.within200Mm)
        << "% <=500=" << percentOfEvaluated(stats.within500Mm) << "%\n";
    std::cout << "  within_relative <=5%=" << percentOfEvaluated(stats.within5Percent)
        << "% <=10%=" << percentOfEvaluated(stats.within10Percent)
        << "% <=20%=" << percentOfEvaluated(stats.within20Percent) << "%\n";
}

int runRgbDepthAnchorOnlyTest(
    rs2::pipeline& pipeline,
    float depthScale,
    const SegmentationConfig& config,
    const RgbDepthAccuracyConfig& rgbDepthConfig)
{
    std::cout << "RGB depth anchor-only line: sample_frames=" << rgbDepthConfig.sampleFrames
        << " warmup_frames=" << rgbDepthConfig.warmupFrames
        << " anchor_step_px=" << rgbDepthConfig.anchorStepPixels
        << " anchor_neighborhood_px=" << rgbDepthConfig.anchorNeighborhoodPixels
        << " anchor_neighbor_range_mm=" << rgbDepthConfig.anchorMaxNeighborRangeMm
        << " anchor_raw_filtered_gap_mm=" << rgbDepthConfig.anchorMaxRawFilteredGapMm
        << " anchor_edge_dilate_px=" << rgbDepthConfig.anchorEdgeDilatePixels
        << " holdout_percent=" << rgbDepthConfig.anchorHoldoutPercent
        << '\n';

    rs2::align alignToColor(RS2_STREAM_COLOR);
    DepthPostProcessor depthPostProcessor;
    for (int frame = 0; frame < rgbDepthConfig.warmupFrames; ++frame)
    {
        rs2::frameset frames = pipeline.wait_for_frames();
        frames = alignToColor.process(frames);
        const rs2::depth_frame depthFrame = frames.get_depth_frame();
        if (depthFrame)
        {
            (void)depthPostProcessor.process(depthFrame);
        }
    }

    RgbDepthAnchorStats aggregateStats;
    int collectedFrames = 0;
    int attempts = 0;
    const int maxAttempts = rgbDepthConfig.sampleFrames * 10;
    while (collectedFrames < rgbDepthConfig.sampleFrames && attempts < maxAttempts)
    {
        ++attempts;
        rs2::frameset frames = pipeline.wait_for_frames();
        frames = alignToColor.process(frames);

        const rs2::video_frame colorFrame = frames.get_color_frame();
        const rs2::depth_frame depthFrame = frames.get_depth_frame();
        if (!colorFrame || !depthFrame)
        {
            continue;
        }

        const rs2::depth_frame filteredDepth = depthPostProcessor.process(depthFrame);
        cv::Mat colorBgr = colorFrameToBgr(colorFrame);
        cv::Mat rawDepth16 = depthFrameToMat(depthFrame);
        cv::Mat filteredDepth16 = depthFrameToMat(filteredDepth);
        if (rawDepth16.size() != colorBgr.size())
        {
            cv::resize(rawDepth16, rawDepth16, colorBgr.size(), 0.0, 0.0, cv::INTER_NEAREST);
        }
        if (filteredDepth16.size() != colorBgr.size())
        {
            cv::resize(filteredDepth16, filteredDepth16, colorBgr.size(), 0.0, 0.0, cv::INTER_NEAREST);
        }

        RgbDepthAnchorStats frameStats;
        const cv::Mat anchorMask = makeReliableDepthAnchorMask(
            rawDepth16,
            filteredDepth16,
            depthScale,
            config,
            rgbDepthConfig,
            frameStats);
        cv::Mat trainMask;
        cv::Mat holdoutMask;
        splitAnchorMask(anchorMask, rgbDepthConfig.anchorHoldoutPercent, trainMask, holdoutMask, frameStats);

        aggregateStats.merge(frameStats);
        ++collectedFrames;
        printRgbDepthAnchorStats("anchor_frame_" + std::to_string(collectedFrames), frameStats);
    }

    if (collectedFrames == 0)
    {
        std::cerr << "RGB depth anchor-only line collected no valid frames.\n";
        return 4;
    }
    if (collectedFrames < rgbDepthConfig.sampleFrames)
    {
        std::cerr << "RGB depth anchor-only line collected only " << collectedFrames
            << " frames after " << attempts << " attempts.\n";
    }

    std::cout << "\n[RGB depth anchor-only aggregate]\n";
    printRgbDepthAnchorStats("all_anchor_frames", aggregateStats);
    return collectedFrames == rgbDepthConfig.sampleFrames ? 0 : 5;
}

int runStableContourTest(
    rs2::pipeline& pipeline,
    float depthScale,
    const SegmentationConfig& config,
    const RgbDepthAccuracyConfig& stableConfig,
    const rs2_intrinsics& colorIntrinsics,
    const VideoRecordingConfig& recordingConfig)
{
    const bool videoMode = stableConfig.stableContourVideo;
    const bool showWindow = stableConfig.stableContourShow || videoMode;

    std::cout << "Stable contour test: mode=" << (videoMode ? "video" : "sample")
        << " sample_frames=" << stableConfig.sampleFrames
        << " warmup_frames=" << stableConfig.warmupFrames
        << " min_anchor_points=" << stableConfig.stableContourMinAnchors
        << " anchor_step_px=" << stableConfig.anchorStepPixels
        << " anchor_neighborhood_px=" << stableConfig.anchorNeighborhoodPixels
        << " anchor_neighbor_range_mm=" << stableConfig.anchorMaxNeighborRangeMm
        << " anchor_raw_filtered_gap_mm=" << stableConfig.anchorMaxRawFilteredGapMm
        << " anchor_edge_dilate_px=" << stableConfig.anchorEdgeDilatePixels
        << '\n';

    rs2::align alignToColor(RS2_STREAM_COLOR);
    DepthPostProcessor depthPostProcessor;
    for (int frame = 0; frame < stableConfig.warmupFrames; ++frame)
    {
        rs2::frameset frames = pipeline.wait_for_frames();
        frames = alignToColor.process(frames);
        const rs2::depth_frame depthFrame = frames.get_depth_frame();
        if (depthFrame)
        {
            (void)depthPostProcessor.process(depthFrame);
        }
    }

    SegmentationTracker tracker;
    std::vector<ObservationMaterial> pclCandidateCache;
    std::map<uint64_t, StableContourTrackAggregate> trackAggregates;
    RgbDepthAnchorStats aggregateAnchorStats;
    cv::Mat lastStableContourView;
    VideoRecorder videoRecorder(recordingConfig);

    int collectedFrames = 0;
    int attempts = 0;
    const int maxAttempts = videoMode
        ? std::numeric_limits<int>::max()
        : stableConfig.sampleFrames * 10;
    uint64_t frameId = 0;
    while ((videoMode || collectedFrames < stableConfig.sampleFrames) && attempts < maxAttempts)
    {
        ++attempts;
        rs2::frameset frames = pipeline.wait_for_frames();
        frames = alignToColor.process(frames);

        const rs2::video_frame colorFrame = frames.get_color_frame();
        const rs2::depth_frame depthFrame = frames.get_depth_frame();
        if (!colorFrame || !depthFrame)
        {
            continue;
        }

        const rs2::depth_frame filteredDepth = depthPostProcessor.process(depthFrame);
        cv::Mat colorBgr = colorFrameToBgr(colorFrame);
        cv::Mat rawDepth16 = depthFrameToMat(depthFrame);
        cv::Mat depth16 = depthFrameToMat(filteredDepth);
        if (rawDepth16.size() != colorBgr.size())
        {
            cv::resize(rawDepth16, rawDepth16, colorBgr.size(), 0.0, 0.0, cv::INTER_NEAREST);
        }
        if (depth16.size() != colorBgr.size())
        {
            cv::resize(depth16, depth16, colorBgr.size(), 0.0, 0.0, cv::INTER_NEAREST);
        }

        cv::Mat segmentationGray;
        bool edgeSourceIsInfrared = false;
        if (config.infraredSegmentation)
        {
            const rs2::video_frame infraredFrame = frames.get_infrared_frame(1);
            if (infraredFrame)
            {
                segmentationGray = videoFrameToGray8(infraredFrame);
                edgeSourceIsInfrared = true;
            }
        }
        if (segmentationGray.empty())
        {
            cv::cvtColor(colorBgr, segmentationGray, cv::COLOR_BGR2GRAY);
        }
        if (segmentationGray.size() != colorBgr.size())
        {
            cv::resize(segmentationGray, segmentationGray, colorBgr.size(), 0.0, 0.0, cv::INTER_LINEAR);
        }
        cv::Mat segmentationBgr;
        cv::cvtColor(segmentationGray, segmentationBgr, cv::COLOR_GRAY2BGR);

        RgbDepthAnchorStats frameAnchorStats;
        const cv::Mat anchorMask = makeReliableDepthAnchorMask(
            rawDepth16,
            depth16,
            depthScale,
            config,
            stableConfig,
            frameAnchorStats);
        aggregateAnchorStats.merge(frameAnchorStats);

        const cv::Mat splitBoundaryMask =
            makeSplitBoundaryMask(segmentationGray, depth16, config, depthScale, edgeSourceIsInfrared);
        std::vector<ObservationMaterial> candidateMaterials =
            extractObservationMaterials(
                depth16,
                splitBoundaryMask,
                config,
                depthScale,
                colorIntrinsics,
                frameId,
                nullptr);

        if (config.pclClustering)
        {
            if (candidateMaterials.empty())
            {
                pclCandidateCache.clear();
            }
            else if (pclCandidateCache.empty() || frameId % static_cast<uint64_t>(config.pclFrameInterval) == 0)
            {
                pclCandidateCache = refineObservationMaterialsWithPclClusters(
                    candidateMaterials,
                    depth16,
                    config,
                    depthScale,
                    colorIntrinsics,
                    frameId);
            }

            candidateMaterials = pclCandidateCache;
            for (ObservationMaterial& material : candidateMaterials)
            {
                material.sourceFrameId = frameId;
            }
        }

        int anchorPointsOnCandidates = 0;
        std::vector<ObservationMaterial> anchorSupportedCandidates =
            calibrateMaterialsWithStableAnchors(
                candidateMaterials,
                anchorMask,
                depth16,
                depthScale,
                stableConfig.stableContourMinAnchors,
                anchorPointsOnCandidates);

        const std::vector<ObservationMaterial> stableMaterials =
            tracker.update(anchorSupportedCandidates, config, frameId);

        int measuredStableContours = 0;
        int stableAnchorPoints = 0;
        for (const ObservationMaterial& stableMaterial : stableMaterials)
        {
            const StableContourSupport support =
                measureStableContourSupport(stableMaterial, anchorMask, depth16, depthScale);
            if (support.anchorCount < stableConfig.stableContourMinAnchors)
            {
                continue;
            }

            StableContourTrackAggregate& aggregate = trackAggregates[stableMaterial.observationId];
            addStableContourObservation(aggregate, stableMaterial, support, collectedFrames + 1);
            ++measuredStableContours;
            stableAnchorPoints += support.anchorCount;
        }

        const cv::Mat stableContourView = buildStableContourVisualization(
            segmentationBgr,
            anchorMask,
            depth16,
            depthScale,
            anchorSupportedCandidates,
            stableMaterials,
            trackAggregates,
            stableConfig,
            collectedFrames + 1);
        if (!stableConfig.stableContourSavePath.empty())
        {
            lastStableContourView = stableContourView.clone();
        }
        videoRecorder.write(stableContourView);

        bool exitRequested = false;
        if (showWindow)
        {
            cv::imshow(kStableContourWindow, stableContourView);
            const int key = cv::waitKey(1);
            if (key == 27 || key == 'q' || key == 'Q')
            {
                exitRequested = true;
            }
        }

        ++collectedFrames;
        ++frameId;

        std::cout << "stable_contour_frame_" << collectedFrames
            << " anchors=" << frameAnchorStats.selected
            << " candidates=" << candidateMaterials.size()
            << " anchor_supported_candidates=" << anchorSupportedCandidates.size()
            << " stable_tracked_contours=" << stableMaterials.size()
            << " measured_stable_contours=" << measuredStableContours
            << " candidate_anchor_points=" << anchorPointsOnCandidates
            << " stable_anchor_points=" << stableAnchorPoints << '\n';

        if (exitRequested)
        {
            break;
        }
    }

    if (collectedFrames == 0)
    {
        std::cerr << "Stable contour test collected no valid frames.\n";
        return 4;
    }
    if (!stableConfig.stableContourSavePath.empty() && !lastStableContourView.empty())
    {
        if (!cv::imwrite(stableConfig.stableContourSavePath, lastStableContourView))
        {
            std::cerr << "Failed to save stable contour visualization: "
                << stableConfig.stableContourSavePath << '\n';
        }
    }

    if (!videoMode && collectedFrames < stableConfig.sampleFrames)
    {
        std::cerr << "Stable contour test collected only " << collectedFrames
            << " frames after " << attempts << " attempts.\n";
    }

    std::cout << "\n[stable contour anchor aggregate]\n";
    printRgbDepthAnchorStats("all_anchor_frames", aggregateAnchorStats);
    printStableContourAggregate(
        trackAggregates,
        collectedFrames,
        stableConfig.stableContourTopCount);
    videoRecorder.close();
    if (showWindow)
    {
        cv::destroyWindow(kStableContourWindow);
    }
    return (videoMode || collectedFrames == stableConfig.sampleFrames) ? 0 : 5;
}

int runRgbDepthAccuracyTest(
    rs2::pipeline& pipeline,
    float depthScale,
    const SegmentationConfig& config,
    const RgbDepthAccuracyConfig& rgbDepthConfig)
{
    std::cout << "RGB depth accuracy test: sample_frames=" << rgbDepthConfig.sampleFrames
        << " warmup_frames=" << rgbDepthConfig.warmupFrames
        << " eval_step_px=" << rgbDepthConfig.evalStepPixels
        << " fit_scope=" << ((rgbDepthConfig.fitEachFrame || rgbDepthConfig.anchorCorrection) ? "each_frame" : "first_frame")
        << " anchor_correction=" << (rgbDepthConfig.anchorCorrection ? "on" : "off")
        << '\n';
    if (rgbDepthConfig.anchorCorrection)
    {
        std::cout << "anchor settings: step_px=" << rgbDepthConfig.anchorStepPixels
            << " neighborhood_px=" << rgbDepthConfig.anchorNeighborhoodPixels
            << " neighbor_range_mm=" << rgbDepthConfig.anchorMaxNeighborRangeMm
            << " raw_filtered_gap_mm=" << rgbDepthConfig.anchorMaxRawFilteredGapMm
            << " edge_dilate_px=" << rgbDepthConfig.anchorEdgeDilatePixels
            << " holdout_percent=" << rgbDepthConfig.anchorHoldoutPercent
            << '\n';
    }

    RgbDepthEstimator estimator(rgbDepthConfig.onnxPath);
    std::cout << "RGB depth estimator=" << estimator.methodLabel();
    if (!rgbDepthConfig.onnxPath.empty())
    {
        std::cout << " model=" << rgbDepthConfig.onnxPath;
    }
    std::cout << '\n';
    if (rgbDepthConfig.onnxPath.empty())
    {
        std::cout
            << "Note: no ONNX model was supplied, so this run uses a weak RGB-only heuristic baseline.\n";
    }

    rs2::align alignToColor(RS2_STREAM_COLOR);
    DepthPostProcessor depthPostProcessor;
    for (int frame = 0; frame < rgbDepthConfig.warmupFrames; ++frame)
    {
        rs2::frameset frames = pipeline.wait_for_frames();
        frames = alignToColor.process(frames);
        const rs2::depth_frame depthFrame = frames.get_depth_frame();
        if (depthFrame)
        {
            (void)depthPostProcessor.process(depthFrame);
        }
    }

    InverseDepthCalibration calibration;
    RgbDepthAccuracyStats aggregateStats;
    RgbDepthAccuracyStats aggregateHoldoutAnchorStats;
    RgbDepthAnchorStats aggregateAnchorStats;
    int collectedFrames = 0;
    int attempts = 0;
    const int maxAttempts = rgbDepthConfig.sampleFrames * 10;
    while (collectedFrames < rgbDepthConfig.sampleFrames && attempts < maxAttempts)
    {
        ++attempts;
        rs2::frameset frames = pipeline.wait_for_frames();
        frames = alignToColor.process(frames);

        const rs2::video_frame colorFrame = frames.get_color_frame();
        const rs2::depth_frame depthFrame = frames.get_depth_frame();
        if (!colorFrame || !depthFrame)
        {
            continue;
        }

        const rs2::depth_frame filteredDepth = depthPostProcessor.process(depthFrame);
        cv::Mat colorBgr = colorFrameToBgr(colorFrame);
        cv::Mat rawDepth16 = depthFrameToMat(depthFrame);
        cv::Mat referenceDepth16 = depthFrameToMat(filteredDepth);
        if (rawDepth16.size() != colorBgr.size())
        {
            cv::resize(rawDepth16, rawDepth16, colorBgr.size(), 0.0, 0.0, cv::INTER_NEAREST);
        }
        if (referenceDepth16.size() != colorBgr.size())
        {
            cv::resize(referenceDepth16, referenceDepth16, colorBgr.size(), 0.0, 0.0, cv::INTER_NEAREST);
        }

        cv::Mat trainAnchorMask;
        cv::Mat holdoutAnchorMask;
        const cv::Mat* calibrationMask = nullptr;
        const cv::Mat* holdoutEvalMask = nullptr;
        if (rgbDepthConfig.anchorCorrection)
        {
            RgbDepthAnchorStats frameAnchorStats;
            const cv::Mat anchorMask = makeReliableDepthAnchorMask(
                rawDepth16,
                referenceDepth16,
                depthScale,
                config,
                rgbDepthConfig,
                frameAnchorStats);
            splitAnchorMask(
                anchorMask,
                rgbDepthConfig.anchorHoldoutPercent,
                trainAnchorMask,
                holdoutAnchorMask,
                frameAnchorStats);
            aggregateAnchorStats.merge(frameAnchorStats);
            printRgbDepthAnchorStats(
                "anchor_frame_" + std::to_string(collectedFrames + 1),
                frameAnchorStats);

            calibrationMask = &trainAnchorMask;
            if (frameAnchorStats.holdout > 0)
            {
                holdoutEvalMask = &holdoutAnchorMask;
            }
        }

        cv::Mat relativeInverseDepth = estimator.estimateRelativeInverseDepth(colorBgr);
        if (relativeInverseDepth.size() != colorBgr.size())
        {
            cv::resize(relativeInverseDepth, relativeInverseDepth, colorBgr.size(), 0.0, 0.0, cv::INTER_CUBIC);
        }
        if (relativeInverseDepth.type() != CV_32F)
        {
            relativeInverseDepth.convertTo(relativeInverseDepth, CV_32F);
        }

        if (!calibration.valid || rgbDepthConfig.fitEachFrame || rgbDepthConfig.anchorCorrection)
        {
            InverseDepthCalibration frameCalibration;
            if (!fitInverseDepthCalibration(
                    relativeInverseDepth,
                    referenceDepth16,
                    depthScale,
                    config,
                    rgbDepthConfig.evalStepPixels,
                    frameCalibration,
                    calibrationMask))
            {
                std::cerr << "RGB depth calibration failed on frame attempt " << attempts << '\n';
                continue;
            }
            calibration = frameCalibration;
            std::cout << std::defaultfloat << std::setprecision(6)
                << "calibration frame=" << (collectedFrames + 1)
                << " samples=" << calibration.sampleCount
                << " inverse_depth= " << calibration.intercept
                << " + " << calibration.slope << " * rgb_cue\n";
        }

        RgbDepthAccuracyStats frameStats = evaluateRgbDepthPrediction(
            relativeInverseDepth,
            referenceDepth16,
            depthScale,
            config,
            rgbDepthConfig.evalStepPixels,
            calibration);
        aggregateStats.merge(frameStats);
        if (holdoutEvalMask != nullptr)
        {
            RgbDepthAccuracyStats holdoutFrameStats = evaluateRgbDepthPrediction(
                relativeInverseDepth,
                referenceDepth16,
                depthScale,
                config,
                rgbDepthConfig.evalStepPixels,
                calibration,
                holdoutEvalMask);
            aggregateHoldoutAnchorStats.merge(holdoutFrameStats);
            printRgbDepthAccuracyStats(
                "heldout_anchor_frame_" + std::to_string(collectedFrames + 1),
                holdoutFrameStats);
        }
        ++collectedFrames;

        printRgbDepthAccuracyStats(
            "frame_" + std::to_string(collectedFrames),
            frameStats);
    }

    if (collectedFrames == 0)
    {
        std::cerr << "RGB depth accuracy test collected no valid frames.\n";
        return 4;
    }
    if (collectedFrames < rgbDepthConfig.sampleFrames)
    {
        std::cerr << "RGB depth accuracy test collected only " << collectedFrames
            << " frames after " << attempts << " attempts.\n";
    }

    std::cout << "\n[RGB depth accuracy aggregate]\n";
    printRgbDepthAccuracyStats("all_frames", aggregateStats);
    if (rgbDepthConfig.anchorCorrection)
    {
        std::cout << "\n[RGB depth anchor aggregate]\n";
        printRgbDepthAnchorStats("all_anchor_frames", aggregateAnchorStats);
        std::cout << "\n[RGB depth held-out anchor accuracy aggregate]\n";
        printRgbDepthAccuracyStats("heldout_anchor_frames", aggregateHoldoutAnchorStats);
    }
    return collectedFrames == rgbDepthConfig.sampleFrames ? 0 : 5;
}

struct DepthStabilityAccumulator
{
    std::vector<uint16_t> minUnits;
    std::vector<uint16_t> maxUnits;
    std::vector<uint8_t> validCounts;
    int width = 0;
    int height = 0;

    void reset(int newWidth, int newHeight)
    {
        width = newWidth;
        height = newHeight;
        const size_t size = static_cast<size_t>(width) * static_cast<size_t>(height);
        minUnits.assign(size, std::numeric_limits<uint16_t>::max());
        maxUnits.assign(size, 0);
        validCounts.assign(size, 0);
    }

    void addFrame(const cv::Mat& depth16)
    {
        if (width == 0 || height == 0)
        {
            reset(depth16.cols, depth16.rows);
        }

        for (int y = 0; y < depth16.rows; ++y)
        {
            const uint16_t* depthRow = depth16.ptr<uint16_t>(y);
            for (int x = 0; x < depth16.cols; ++x)
            {
                const uint16_t depth = depthRow[x];
                if (depth == 0)
                {
                    continue;
                }

                const size_t index = static_cast<size_t>(y) * static_cast<size_t>(width) + static_cast<size_t>(x);
                minUnits[index] = std::min(minUnits[index], depth);
                maxUnits[index] = std::max(maxUnits[index], depth);
                if (validCounts[index] < std::numeric_limits<uint8_t>::max())
                {
                    ++validCounts[index];
                }
            }
        }
    }
};

void printDepthStabilityReport(
    const std::string& label,
    const DepthStabilityAccumulator& accumulator,
    int sampleFrames,
    float depthScale,
    int minDepthMm,
    int maxDepthMm,
    int relativePermille)
{
    const std::vector<int> thresholdsMm{0, 2, 5, 10, 20, 50};
    std::vector<int> stableCounts(thresholdsMm.size(), 0);
    std::vector<int> stableInRangeCounts(thresholdsMm.size(), 0);
    std::vector<int> rangesMm;
    std::vector<int> inRangeRangesMm;
    int validAtLeastOne = 0;
    int validAll = 0;
    int inRangeAll = 0;
    int stableRelative = 0;
    int stableRelativeInRange = 0;
    const int totalPixels = accumulator.width * accumulator.height;
    const int minDepthUnits = depthUnitsFromMm(minDepthMm, depthScale);
    const int maxDepthUnits = depthUnitsFromMm(maxDepthMm, depthScale);

    for (size_t index = 0; index < accumulator.validCounts.size(); ++index)
    {
        if (accumulator.validCounts[index] > 0)
        {
            ++validAtLeastOne;
        }
        if (accumulator.validCounts[index] != sampleFrames)
        {
            continue;
        }

        ++validAll;
        const int rangeMm = static_cast<int>(
            (accumulator.maxUnits[index] - accumulator.minUnits[index]) * depthScale * 1000.0f + 0.5f);
        rangesMm.push_back(rangeMm);
        const bool inRange =
            accumulator.minUnits[index] >= minDepthUnits &&
            accumulator.maxUnits[index] <= maxDepthUnits;
        const double centerDepthMm =
            (static_cast<double>(accumulator.minUnits[index]) +
                static_cast<double>(accumulator.maxUnits[index])) *
            0.5 * depthScale * 1000.0;
        const double relativeThresholdMm = centerDepthMm * relativePermille / 1000.0;
        const bool relativeStable = rangeMm <= relativeThresholdMm;
        if (relativeStable)
        {
            ++stableRelative;
        }
        if (inRange)
        {
            ++inRangeAll;
            inRangeRangesMm.push_back(rangeMm);
            if (relativeStable)
            {
                ++stableRelativeInRange;
            }
        }
        for (size_t thresholdIndex = 0; thresholdIndex < thresholdsMm.size(); ++thresholdIndex)
        {
            if (rangeMm <= thresholdsMm[thresholdIndex])
            {
                ++stableCounts[thresholdIndex];
            }
            if (inRange && rangeMm <= thresholdsMm[thresholdIndex])
            {
                ++stableInRangeCounts[thresholdIndex];
            }
        }
    }

    std::sort(rangesMm.begin(), rangesMm.end());
    std::sort(inRangeRangesMm.begin(), inRangeRangesMm.end());
    auto percentile = [](const std::vector<int>& values, double p)
    {
        if (values.empty())
        {
            return 0;
        }
        const size_t index = std::min(
            values.size() - 1,
            static_cast<size_t>((values.size() - 1) * p + 0.5));
        return values[index];
    };

    auto percentOfTotal = [totalPixels](int count)
    {
        return totalPixels == 0 ? 0.0 : count * 100.0 / totalPixels;
    };
    auto percentOfValidAll = [validAll](int count)
    {
        return validAll == 0 ? 0.0 : count * 100.0 / validAll;
    };
    auto percentOfInRangeAll = [inRangeAll](int count)
    {
        return inRangeAll == 0 ? 0.0 : count * 100.0 / inRangeAll;
    };

    std::cout << "\n[" << label << "]\n";
    std::cout << "pixels=" << totalPixels
        << " valid_at_least_one=" << validAtLeastOne << " (" << std::fixed << std::setprecision(2)
        << percentOfTotal(validAtLeastOne) << "%)"
        << " valid_all_" << sampleFrames << "_frames=" << validAll << " ("
        << percentOfTotal(validAll) << "%)\n";
    std::cout << "in_range_all_" << minDepthMm << "_" << maxDepthMm << "mm="
        << inRangeAll << " (" << percentOfTotal(inRangeAll) << "%)\n";
    std::cout << "range_mm_percentiles among valid_all: p50=" << percentile(rangesMm, 0.50)
        << " p90=" << percentile(rangesMm, 0.90)
        << " p95=" << percentile(rangesMm, 0.95)
        << " p99=" << percentile(rangesMm, 0.99) << '\n';
    std::cout << "range_mm_percentiles among in_range_all: p50=" << percentile(inRangeRangesMm, 0.50)
        << " p90=" << percentile(inRangeRangesMm, 0.90)
        << " p95=" << percentile(inRangeRangesMm, 0.95)
        << " p99=" << percentile(inRangeRangesMm, 0.99) << '\n';
    std::cout << "stable_relative_" << relativePermille << "permille="
        << stableRelative << " total=" << percentOfTotal(stableRelative)
        << "% valid_all=" << percentOfValidAll(stableRelative)
        << "% in_range=" << stableRelativeInRange
        << " in_range_all=" << percentOfInRangeAll(stableRelativeInRange) << "%\n";

    for (size_t thresholdIndex = 0; thresholdIndex < thresholdsMm.size(); ++thresholdIndex)
    {
        const int count = stableCounts[thresholdIndex];
        const int inRangeCount = stableInRangeCounts[thresholdIndex];
        std::cout << "stable_range_le_" << thresholdsMm[thresholdIndex] << "mm="
            << count << " total=" << percentOfTotal(count)
            << "% valid_all=" << percentOfValidAll(count)
            << "% in_range=" << inRangeCount
            << " in_range_all=" << percentOfInRangeAll(inRangeCount) << "%\n";
    }
}

int runDepthStabilityTest(
    rs2::pipeline& pipeline,
    float depthScale,
    const SegmentationConfig& config,
    int sampleFrames,
    int warmupFrames,
    int relativePermille)
{
    sampleFrames = std::clamp(sampleFrames, 1, 200);
    warmupFrames = std::clamp(warmupFrames, 0, 300);
    relativePermille = std::clamp(relativePermille, 1, 500);

    std::cout << "Depth stability test: sample_frames=" << sampleFrames
        << " warmup_frames=" << warmupFrames
        << " relative_permille=" << relativePermille << '\n';

    rs2::align alignToColor(RS2_STREAM_COLOR);
    DepthPostProcessor depthPostProcessor;
    DepthStabilityAccumulator rawAccumulator;
    DepthStabilityAccumulator filteredAccumulator;

    for (int frame = 0; frame < warmupFrames; ++frame)
    {
        rs2::frameset frames = pipeline.wait_for_frames();
        frames = alignToColor.process(frames);
        const rs2::depth_frame depthFrame = frames.get_depth_frame();
        if (depthFrame)
        {
            (void)depthPostProcessor.process(depthFrame);
        }
    }

    int collectedFrames = 0;
    while (collectedFrames < sampleFrames)
    {
        rs2::frameset frames = pipeline.wait_for_frames();
        frames = alignToColor.process(frames);
        const rs2::depth_frame depthFrame = frames.get_depth_frame();
        if (!depthFrame)
        {
            continue;
        }

        const rs2::depth_frame filteredDepth = depthPostProcessor.process(depthFrame);
        cv::Mat rawDepth16 = depthFrameToMat(depthFrame);
        cv::Mat filteredDepth16 = depthFrameToMat(filteredDepth);
        if (filteredDepth16.size() != rawDepth16.size())
        {
            cv::resize(filteredDepth16, filteredDepth16, rawDepth16.size(), 0.0, 0.0, cv::INTER_NEAREST);
        }

        rawAccumulator.addFrame(rawDepth16);
        filteredAccumulator.addFrame(filteredDepth16);
        ++collectedFrames;
    }

    printDepthStabilityReport(
        "raw_aligned_depth",
        rawAccumulator,
        collectedFrames,
        depthScale,
        config.minDepthMm,
        config.maxDepthMm,
        relativePermille);
    printDepthStabilityReport(
        "filtered_aligned_depth",
        filteredAccumulator,
        collectedFrames,
        depthScale,
        config.minDepthMm,
        config.maxDepthMm,
        relativePermille);
    return 0;
}

void printUsage()
{
    std::cout
        << "D455 four-window observation-material demo\n"
        << "Keys: q or Esc exits.\n"
        << "Options:\n"
        << "  --probe-only\n"
        << "  --depth-stability-test\n"
        << "  --depth-stability-frames=20\n"
        << "  --depth-stability-warmup=30\n"
        << "  --depth-stability-relative-permille=10\n"
        << "  --rgb-depth-accuracy-test\n"
        << "  --rgb-depth-anchor-only\n"
        << "  --rgb-depth-anchor-correction\n"
        << "  --stable-contour-test\n"
        << "  --rgb-depth-frames=20\n"
        << "  --rgb-depth-warmup=30\n"
        << "  --rgb-depth-eval-step-px=1\n"
        << "  --rgb-depth-anchor-step-px=4\n"
        << "  --rgb-depth-anchor-neighborhood-px=2\n"
        << "  --rgb-depth-anchor-max-neighbor-range-mm=35\n"
        << "  --rgb-depth-anchor-max-raw-filtered-gap-mm=60\n"
        << "  --rgb-depth-anchor-edge-dilate-px=3\n"
        << "  --rgb-depth-anchor-holdout-percent=30\n"
        << "  --stable-contour-min-anchors=16\n"
        << "  --stable-contour-top=12\n"
        << "  --stable-contour-save=stable_contours_last.png\n"
        << "  --stable-contour-show\n"
        << "  --stable-contour-video\n"
        << "  --rgb-depth-onnx=models\\midas\\model-small.onnx\n"
        << "  --rgb-depth-fit-each-frame\n"
        << "  --record-video\n"
        << "  --record-video=recordings\\d455_record.avi\n"
        << "  --no-record-video\n"
        << "  --record-fps=30\n"
        << "  --record-every-n=1\n"
        << "  --record-scale-percent=100\n"
        << "  --acceptance-baseline\n"
        << "  --acceptance-no-record\n"
        << "  --acceptance-label=acceptance_baseline\n"
        << "  --acceptance-csv=recordings\\acceptance_baseline.csv\n"
        << "  --no-display\n"
        << "  --max-frames=0\n"
        << "  --min-depth-mm=250\n"
        << "  --max-depth-mm=3500\n"
        << "  --min-area-px=900\n"
        << "  --depth-slice-mm=300\n"
        << "  --max-area-percent=24\n"
        << "  --max-roi-area-percent=55\n"
        << "  --max-border-area-percent=8\n"
        << "  --foreground-keep-depth-mm=1800\n"
        << "  --foreground-max-area-percent=70\n"
        << "  --foreground-max-roi-area-percent=90\n"
        << "  --max-material-area-percent=35\n"
        << "  --max-material-roi-area-percent=65\n"
        << "  --min-edge-px=18\n"
        << "  --depth-canny-low=10\n"
        << "  --depth-canny-high=35\n"
        << "  --color-canny-low=70\n"
        << "  --color-canny-high=160\n"
        << "  --infrared-canny-low=45\n"
        << "  --infrared-canny-high=130\n"
        << "  --color-depth-support-px=5\n"
        << "  --contour-depth-confirm-radius-px=4\n"
        << "  --contour-depth-confirm-min-range-mm=25\n"
        << "  --contour-depth-confirm-min-valid-px=8\n"
        << "  --depth-hole-edge-px=3\n"
        << "  --cue-min-reliable-edge-px=8\n"
        << "  --cue-min-confirmed-rgb-edge-px=6\n"
        << "  --cue-min-confirmed-gray-edge-px=6\n"
        << "  --cue-max-texture-only-percent=75\n"
        << "  --cue-strong-anchor-multiplier-percent=180\n"
        << "  --indoor-plane-frame-interval=10\n"
        << "  --indoor-plane-sample-step-px=10\n"
        << "  --indoor-plane-normal-neighbor-px=10\n"
        << "  --indoor-plane-min-area-percent=4\n"
        << "  --indoor-plane-foreground-dilate-px=9\n"
        << "  --indoor-plane-horizontal-normal-min-percent=65\n"
        << "  --indoor-plane-vertical-normal-max-percent=65\n"
        << "  --indoor-plane-ceiling-band-percent=30\n"
        << "  --split-boundary-px=3\n"
        << "  --group-gap-px=24\n"
        << "  --group-depth-gap-mm=450\n"
        << "  --spatial-cluster-gap-mm=120\n"
        << "  --track-confirm-frames=3\n"
        << "  --track-miss-frames=5\n"
        << "  --track-center-gap-px=48\n"
        << "  --track-depth-gap-mm=300\n"
        << "  --track-iou-percent=20\n"
        << "  --track-smooth-percent=65\n"
        << "  --pcl-cluster-tolerance-mm=35\n"
        << "  --pcl-min-cluster-points=80\n"
        << "  --pcl-max-cluster-points=200000\n"
        << "  --pcl-sample-step-px=3\n"
        << "  --pcl-max-input-points=7000\n"
        << "  --pcl-frame-interval=2\n"
        << "  --color-contour-completion\n"
        << "  --no-color-contour-completion\n"
        << "  --color-contour-completion-padding-px=12\n"
        << "  --color-contour-completion-min-iou-percent=72\n"
        << "  --color-contour-completion-max-area-delta-percent=24\n"
        << "  --color-contour-completion-max-center-shift-px=16\n"
        << "  --color-contour-completion-close-px=3\n"
        << "  --color-contour-completion-other-guard-px=7\n"
        << "  --color-contour-completion-max-other-overlap-px=4\n"
        << "  --contour-line-px=1\n"
        << "  --max-materials=24\n"
        << "  --no-boundary-split\n"
        << "  --no-color-split\n"
        << "  --no-gray-split\n"
        << "  --no-infrared-segmentation\n"
        << "  --no-contour-depth-confirm-split\n"
        << "  --depth-hole-split\n"
        << "  --no-depth-hole-split\n"
        << "  --no-cue-selection\n"
        << "  --no-pcl-clustering\n"
        << "  --no-spatial-cluster-check\n"
        << "  --no-history-tracking\n"
        << "  --boundary-diagnostics\n"
        << "  --indoor-plane-diagnostics\n"
        << "  --show-regions\n"
        << "  --color-edges\n"
        << "  --gray-edges\n"
        << "  --show-part-numbers\n"
        << "  --show-labels\n"
        << "  --show-centers\n";
}
}

int main(int argc, char** argv)
{
    const SegmentationConfig config = parseConfig(argc, argv);
    const int maxFrames = std::max(0, parseIntOptionOrDefault(argc, argv, "--max-frames=", 0));
    const bool displayEnabled = !hasFlag(argc, argv, "--no-display");
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);
    printUsage();
    if (hasFlag(argc, argv, "--help") || hasFlag(argc, argv, "-h"))
    {
        return 0;
    }

    try
    {
        rs2::context context;
        const rs2::device_list devices = context.query_devices();
        std::cout << "RealSense devices: " << devices.size() << '\n';
        for (rs2::device device : devices)
        {
            const char* name = device.supports(RS2_CAMERA_INFO_NAME)
                ? device.get_info(RS2_CAMERA_INFO_NAME)
                : "unknown";
            const char* serial = device.supports(RS2_CAMERA_INFO_SERIAL_NUMBER)
                ? device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)
                : "unknown";
            std::cout << "  " << name << " serial=" << serial << '\n';
        }

        if (hasFlag(argc, argv, "--probe-only"))
        {
            return devices.size() == 0 ? 1 : 0;
        }

        if (devices.size() == 0)
        {
            std::cerr << "No RealSense device found. Connect D455 and run again.\n";
            return 1;
        }

        rs2::pipeline pipeline;
        rs2::config rsConfig;
        rsConfig.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        rsConfig.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        rsConfig.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);

        const rs2::pipeline_profile profile = pipeline.start(rsConfig);
        const float depthScale = findDepthScale(profile);
        const rs2_intrinsics colorIntrinsics =
            profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
        if (hasFlag(argc, argv, "--depth-stability-test"))
        {
            const int sampleFrames = std::max(
                1,
                parseIntOptionOrDefault(argc, argv, "--depth-stability-frames=", 20));
            const int warmupFrames = std::max(
                0,
                parseIntOptionOrDefault(argc, argv, "--depth-stability-warmup=", 30));
            const int relativePermille = std::max(
                1,
                parseIntOptionOrDefault(argc, argv, "--depth-stability-relative-permille=", 10));
            return runDepthStabilityTest(
                pipeline,
                depthScale,
                config,
                sampleFrames,
                warmupFrames,
                relativePermille);
        }
        if (hasFlag(argc, argv, "--rgb-depth-anchor-only"))
        {
            return runRgbDepthAnchorOnlyTest(
                pipeline,
                depthScale,
                config,
                parseRgbDepthAccuracyConfig(argc, argv));
        }
        if (hasFlag(argc, argv, "--stable-contour-test") ||
            hasFlag(argc, argv, "--stable-contour-video"))
        {
            return runStableContourTest(
                pipeline,
                depthScale,
                config,
                parseRgbDepthAccuracyConfig(argc, argv),
                colorIntrinsics,
                parseVideoRecordingConfig(argc, argv, false));
        }
        if (hasFlag(argc, argv, "--rgb-depth-accuracy-test"))
        {
            return runRgbDepthAccuracyTest(
                pipeline,
                depthScale,
                config,
                parseRgbDepthAccuracyConfig(argc, argv));
        }

        rs2::align alignToColor(RS2_STREAM_COLOR);
        DepthPostProcessor depthPostProcessor;
        RgbDepthAccuracyConfig stableDisplayConfig = parseRgbDepthAccuracyConfig(argc, argv);
        stableDisplayConfig.stableContourVideo = true;
        VideoRecordingConfig recordingConfig = parseVideoRecordingConfig(argc, argv, true);
        AcceptanceMetricsConfig acceptanceConfig = parseAcceptanceMetricsConfig(argc, argv);
        if (acceptanceConfig.enabled)
        {
            if (acceptanceConfig.recordVideo && recordingConfig.enabled)
            {
                if (recordingConfig.outputPath.empty())
                {
                    recordingConfig.outputPath = defaultAcceptanceRecordingPath(acceptanceConfig.label);
                }
                if (acceptanceConfig.csvPath.empty())
                {
                    acceptanceConfig.csvPath = companionCsvPathForVideo(recordingConfig.outputPath);
                }
                std::cout << "Acceptance baseline enabled: video=" << recordingConfig.outputPath
                    << " csv=" << acceptanceConfig.csvPath << '\n';
            }
            else
            {
                recordingConfig.enabled = false;
                if (acceptanceConfig.csvPath.empty())
                {
                    acceptanceConfig.csvPath = defaultAcceptanceMetricsPath(acceptanceConfig.label);
                }
                std::cout << "Acceptance baseline enabled: video=disabled"
                    << " csv=" << acceptanceConfig.csvPath << '\n';
            }
        }

        if (displayEnabled)
        {
            cv::namedWindow(kRawWindow, cv::WINDOW_AUTOSIZE);
            cv::namedWindow(kSegmentedWindow, cv::WINDOW_AUTOSIZE);
            cv::namedWindow(kMosaicWindow, cv::WINDOW_AUTOSIZE);
            cv::namedWindow(kOutsideMosaicWindow, cv::WINDOW_AUTOSIZE);
            if (config.boundaryDiagnostics)
            {
                cv::namedWindow(kBoundaryDiagnosticsWindow, cv::WINDOW_AUTOSIZE);
            }
            if (config.indoorPlaneDiagnostics)
            {
                cv::namedWindow(kIndoorPlaneWindow, cv::WINDOW_AUTOSIZE);
            }
        }

        SegmentationTracker tracker;
        std::vector<ObservationMaterial> pclCandidateCache;
        std::map<uint64_t, StableContourTrackAggregate> stableTrackAggregates;
        IndoorPlaneAnalysis cachedIndoorPlaneAnalysis;
        bool hasIndoorPlaneCache = false;
        const bool needsViews = displayEnabled || recordingConfig.enabled;

        VideoRecorder videoRecorder(recordingConfig);
        AcceptanceMetricsWriter acceptanceMetrics(acceptanceConfig);
        uint64_t frameId = 0;
        while (true)
        {
            rs2::frameset frames = pipeline.wait_for_frames();
            frames = alignToColor.process(frames);

            const rs2::video_frame colorFrame = frames.get_color_frame();
            const rs2::depth_frame depthFrame = frames.get_depth_frame();
            if (!colorFrame || !depthFrame)
            {
                continue;
            }

            const auto frameStart = std::chrono::steady_clock::now();
            FrameTimingStats timingStats;
            auto sectionStart = frameStart;
            auto takeSectionMs = [&sectionStart]()
            {
                const auto sectionEnd = std::chrono::steady_clock::now();
                const double elapsedMs =
                    std::chrono::duration<double, std::milli>(sectionEnd - sectionStart).count();
                sectionStart = sectionEnd;
                return elapsedMs;
            };
            const rs2::depth_frame filteredDepth = depthPostProcessor.process(depthFrame);
            timingStats.depthPostMs = takeSectionMs();
            cv::Mat colorBgr = colorFrameToBgr(colorFrame);
            cv::Mat rawDepth16 = depthFrameToMat(depthFrame);
            cv::Mat depth16 = depthFrameToMat(filteredDepth);

            if (rawDepth16.size() != colorBgr.size())
            {
                cv::resize(rawDepth16, rawDepth16, colorBgr.size(), 0.0, 0.0, cv::INTER_NEAREST);
            }
            if (depth16.size() != colorBgr.size())
            {
                cv::resize(depth16, depth16, colorBgr.size(), 0.0, 0.0, cv::INTER_NEAREST);
            }
            timingStats.frameConvertMs = takeSectionMs();

            cv::Mat segmentationGray;
            bool edgeSourceIsInfrared = false;
            if (config.infraredSegmentation)
            {
                const rs2::video_frame infraredFrame = frames.get_infrared_frame(1);
                if (infraredFrame)
                {
                    segmentationGray = videoFrameToGray8(infraredFrame);
                    edgeSourceIsInfrared = true;
                }
            }
            if (segmentationGray.empty())
            {
                cv::cvtColor(colorBgr, segmentationGray, cv::COLOR_BGR2GRAY);
            }
            if (segmentationGray.size() != colorBgr.size())
            {
                cv::resize(segmentationGray, segmentationGray, colorBgr.size(), 0.0, 0.0, cv::INTER_LINEAR);
            }
            timingStats.grayPrepareMs = takeSectionMs();

            RgbDepthAnchorStats frameAnchorStats;
            const cv::Mat anchorMask = makeReliableDepthAnchorMask(
                rawDepth16,
                depth16,
                depthScale,
                config,
                stableDisplayConfig,
                frameAnchorStats);
            timingStats.anchorMs = takeSectionMs();

            BoundaryAnalysis boundaryAnalysis =
                makeBoundaryAnalysis(segmentationGray, depth16, config, depthScale, edgeSourceIsInfrared);
            cv::Mat splitBoundaryMask = boundaryAnalysis.splitBoundaryMask;
            timingStats.boundaryMs = takeSectionMs();
            std::vector<ObservationMaterial> candidateMaterials =
                extractObservationMaterials(
                    depth16,
                    splitBoundaryMask,
                    config,
                    depthScale,
                    colorIntrinsics,
                    frameId,
                    nullptr);
            timingStats.extractMs = takeSectionMs();
            if (config.pclClustering)
            {
                if (candidateMaterials.empty())
                {
                    pclCandidateCache.clear();
                }
                else if (pclCandidateCache.empty() || frameId % static_cast<uint64_t>(config.pclFrameInterval) == 0)
                {
                    pclCandidateCache = refineObservationMaterialsWithPclClusters(
                        candidateMaterials,
                        depth16,
                        config,
                        depthScale,
                        colorIntrinsics,
                        frameId);
                }

                candidateMaterials = pclCandidateCache;
                for (ObservationMaterial& material : candidateMaterials)
                {
                    material.sourceFrameId = frameId;
                }
            }
            timingStats.pclMs = takeSectionMs();

            int anchorPointsOnCandidates = 0;
            std::vector<ObservationMaterial> anchorSupportedCandidates =
                calibrateMaterialsWithStableAnchors(
                    candidateMaterials,
                    anchorMask,
                    depth16,
                    depthScale,
                    stableDisplayConfig.stableContourMinAnchors,
                    anchorPointsOnCandidates);
            timingStats.calibrateMs = takeSectionMs();

            CueSelectionSummary cueSummary;
            std::vector<ObservationMaterial> cueSelectedCandidates =
                selectCandidatesByCue(
                    anchorSupportedCandidates,
                    boundaryAnalysis,
                    config,
                    colorBgr.size(),
                    stableDisplayConfig.stableContourMinAnchors,
                    cueSummary);
            timingStats.cueMs = takeSectionMs();

            std::vector<ObservationMaterial> stableMaterials =
                tracker.update(cueSelectedCandidates, config, frameId);
            timingStats.trackerMs = takeSectionMs();

            for (const ObservationMaterial& stableMaterial : stableMaterials)
            {
                const StableContourSupport support =
                    measureStableContourSupport(stableMaterial, anchorMask, depth16, depthScale);
                if (support.anchorCount < stableDisplayConfig.stableContourMinAnchors)
                {
                    continue;
                }

                StableContourTrackAggregate& aggregate = stableTrackAggregates[stableMaterial.observationId];
                addStableContourObservation(
                    aggregate,
                    stableMaterial,
                    support,
                    static_cast<int>(frameId + 1));
            }
            timingStats.supportMs = takeSectionMs();

            cv::Mat stableMaskForFrame;
            if (config.indoorPlaneDiagnostics || acceptanceConfig.enabled)
            {
                stableMaskForFrame = buildStableContourMask(colorBgr.size(), stableMaterials);
            }
            IndoorPlaneAnalysis indoorPlaneAnalysis;
            if (config.indoorPlaneDiagnostics)
            {
                const bool refreshIndoorPlanes =
                    !hasIndoorPlaneCache ||
                    frameId % static_cast<uint64_t>(config.indoorPlaneFrameInterval) == 0;
                if (refreshIndoorPlanes)
                {
                    cachedIndoorPlaneAnalysis = buildIndoorPlaneAnalysis(
                        depth16,
                        stableMaskForFrame,
                        config,
                        depthScale,
                        colorIntrinsics);
                    cachedIndoorPlaneAnalysis.reusedFromCache = false;
                    hasIndoorPlaneCache = true;
                }
                indoorPlaneAnalysis = cachedIndoorPlaneAnalysis;
                indoorPlaneAnalysis.reusedFromCache = !refreshIndoorPlanes;
            }
            timingStats.diagnosticsMs = takeSectionMs();
            ColorContourCompletionStats completionStats;
            cv::Mat segmentedView;
            cv::Mat mosaicView;
            cv::Mat outsideColorView;
            cv::Mat boundaryDiagnosticsView;
            cv::Mat indoorPlaneDiagnosticsView;
            if (needsViews)
            {
                cv::Mat segmentationBgr;
                cv::cvtColor(segmentationGray, segmentationBgr, cv::COLOR_GRAY2BGR);
                segmentedView = buildStableContourVisualization(
                    segmentationBgr,
                    anchorMask,
                    depth16,
                    depthScale,
                    cueSelectedCandidates,
                    stableMaterials,
                    stableTrackAggregates,
                    stableDisplayConfig,
                    static_cast<int>(frameId + 1));
                if (config.cueSelection)
                {
                    drawCueSelectionOverlay(segmentedView, cueSummary);
                }
                timingStats.renderMs = takeSectionMs();

                const std::vector<ObservationMaterial> displayStableMaterials =
                    buildDisplayStableMaterials(colorBgr, stableMaterials, config, completionStats);
                timingStats.completionMs = takeSectionMs();

                mosaicView = buildStableContourColorMosaic(colorBgr, displayStableMaterials);
                drawColorContourCompletionOverlay(mosaicView, completionStats, config.colorContourCompletion);
                outsideColorView = buildOutsideStableContourColorImage(colorBgr, displayStableMaterials);
                timingStats.renderMs += takeSectionMs();

                if (config.boundaryDiagnostics)
                {
                    boundaryDiagnosticsView = buildBoundaryDiagnosticsView(segmentationBgr, boundaryAnalysis);
                }
                if (config.indoorPlaneDiagnostics)
                {
                    indoorPlaneDiagnosticsView =
                        buildIndoorPlaneDiagnosticsView(colorBgr, indoorPlaneAnalysis, stableMaskForFrame);
                }
                timingStats.diagnosticsMs += takeSectionMs();
            }
            (void)frameAnchorStats;
            (void)anchorPointsOnCandidates;

            int key = -1;
            if (displayEnabled)
            {
                cv::imshow(kRawWindow, colorBgr);
                cv::imshow(kSegmentedWindow, segmentedView);
                cv::imshow(kMosaicWindow, mosaicView);
                cv::imshow(kOutsideMosaicWindow, outsideColorView);
                if (config.boundaryDiagnostics)
                {
                    cv::imshow(kBoundaryDiagnosticsWindow, boundaryDiagnosticsView);
                }
                if (config.indoorPlaneDiagnostics)
                {
                    cv::imshow(kIndoorPlaneWindow, indoorPlaneDiagnosticsView);
                }
                key = cv::waitKey(1);
            }
            timingStats.displayMs = takeSectionMs();

            if (recordingConfig.enabled)
            {
                std::vector<cv::Mat> recordingViews{colorBgr, segmentedView, mosaicView, outsideColorView};
                if (config.boundaryDiagnostics)
                {
                    recordingViews.push_back(boundaryDiagnosticsView);
                }
                if (config.indoorPlaneDiagnostics)
                {
                    recordingViews.push_back(indoorPlaneDiagnosticsView);
                }
                videoRecorder.write(buildRecordingFrame(recordingViews));
            }
            timingStats.recordMs = takeSectionMs();

            const auto frameEnd = std::chrono::steady_clock::now();
            const double frameMs =
                std::chrono::duration<double, std::milli>(frameEnd - frameStart).count();
            if (acceptanceConfig.enabled)
            {
                const cv::Mat stableMask = stableMaskForFrame.empty()
                    ? buildStableContourMask(colorBgr.size(), stableMaterials)
                    : stableMaskForFrame;
                acceptanceMetrics.write(
                    frameId,
                    frameMs,
                    static_cast<int>(candidateMaterials.size()),
                    static_cast<int>(anchorSupportedCandidates.size()),
                    stableMaterials,
                    stableMask,
                    boundaryAnalysis,
                    cueSummary,
                    indoorPlaneAnalysis,
                    cv::countNonZero(anchorMask),
                    anchorPointsOnCandidates,
                    completionStats,
                    timingStats);
            }

            if (displayEnabled && (key == 27 || key == 'q' || key == 'Q'))
            {
                break;
            }

            ++frameId;
            if (maxFrames > 0 && frameId >= static_cast<uint64_t>(maxFrames))
            {
                break;
            }
        }

        acceptanceMetrics.close();
        videoRecorder.close();
        pipeline.stop();
        cv::destroyAllWindows();
        return 0;
    }
    catch (const rs2::error& error)
    {
        std::cerr << "RealSense error: " << error.what() << '\n'
                  << "Function: " << error.get_failed_function() << '\n'
                  << "Args: " << error.get_failed_args() << '\n';
        return 2;
    }
    catch (const std::exception& error)
    {
        std::cerr << "Error: " << error.what() << '\n';
        return 3;
    }
}
