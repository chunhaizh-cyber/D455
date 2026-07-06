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
#include <condition_variable>
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
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#ifdef _WIN32
#ifndef NOMINMAX
#define NOMINMAX
#endif
#ifndef WIN32_LEAN_AND_MEAN
#define WIN32_LEAN_AND_MEAN
#endif
#include <conio.h>
#include <windows.h>
#endif

namespace
{
constexpr const char* kObservationDashboardWindow = "D455 observation dashboard five-panel";
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
    int stablePlaneMergeAnchorGapPixels = 14;
    int stablePlaneMergeMaskGapPixels = 21;
    int stablePlaneMergeMinAnchors = 24;
    int stablePlaneMergeMaxAreaPercent = 85;
    int stablePlaneMergeMaxRoiAreaPercent = 98;
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
    int realtimePclFrameInterval = 6;
    int realtimeAnchorStepPixels = 6;
    int realtimeStableContourMinAnchors = 10;
    int realtimeDepthSliceMm = 450;
    int colorContourCompletionPaddingPixels = 12;
    int colorContourCompletionMinIouPercent = 72;
    int colorContourCompletionMaxAreaDeltaPercent = 24;
    int colorContourCompletionMaxCenterShiftPixels = 16;
    int colorContourCompletionClosePixels = 3;
    int colorContourCompletionOtherGuardPixels = 7;
    int colorContourCompletionMaxOtherOverlapPixels = 4;
    int colorContourPrimaryMinAnchors = 24;
    int colorContourPrimaryMinOverlapPercent = 45;
    int colorContourPrimaryMaxAreaDeltaPercent = 220;
    int colorContourPrimaryMaxCenterShiftPixels = 96;
    int colorContourPrimaryMaxAreaPercent = 85;
    int farMaxDepthMm = 12000;
    int farIntervalMm = 1500;
    int farMinAreaPixels = 600;
    int farMaxMaterials = 8;
    int farMorphKernelSize = 7;
    int nearPlaneMaxDepthMm = 2200;
    int nearPlaneMinAreaPercent = 3;
    int nearPlaneMaxMaterials = 2;
    int nearPlaneNormalMinPercent = 60;
    int nearPlaneMinCenterYPercent = 30;
    int nearPlaneSampleStepPixels = 16;
    int nearPlaneMorphKernelSize = 9;
    int nearPlaneFrameInterval = 10;
    int clusterMapVisualMinAreaPixels = 700;
    int clusterMapVisualMorphKernelSize = 7;
    int clusterMapMaxVisualRegions = 24;
    int mosaicMaxMaterialAreaPercent = 32;
    int mosaicBorderRejectAreaPercent = 5;
    int mosaicMinTrackObservations = 45;
    int mosaicMinTrackScore = 45;
    int mosaicNearMinTrackObservations = 3;
    int mosaicNearMinTrackScore = 0;
    int mosaicMaxTrackStaleFrames = 2;
    int colorSegmentationMinAreaPixels = 700;
    int colorSegmentationMorphKernelSize = 7;
    int colorSegmentationMaxRegions = 32;
    int colorSegmentationMaxRoiAreaPercent = 55;
    int colorSegmentationBorderRejectAreaPercent = 28;
    int colorSegmentationColorBins = 6;
    int colorSegmentationMeanShiftSpatial = 9;
    int colorSegmentationMeanShiftColor = 18;
    int colorRefineMinOverlapPercent = 18;
    int colorRefineMaxAreaDeltaPercent = 280;
    int nonPreciseColorOwnershipMinPercent = 10;
    int analysisExportEveryN = 0;
    int colorContourFrameInterval = 1;
    int colorContourRefreshMinGapFrames = 0;
    int colorContourRefreshRoiPaddingPixels = 48;
    int colorContourRefreshMaxRoiAreaPercent = 35;
    int colorContourRefreshMotionDeltaPercent = 25;
    int colorContourRefreshUnknownPercent = 5;
    int stereoContourMinDisparityTenthsPx = 5;
    int stereoContourMaxVerticalShiftPixels = 12;
    int stereoContourSearchMarginPixels = 48;
    int stereoContourMaxRegionsPerFrame = 32;
    int stereoContourMaxRoiAreaPercent = 100;
    int stereoContourBaselineMm = 95;
    int processedViewScalePercent = 75;
    int overlapTrimMinSupportPercent = 3;
    int overlapTrimPaddingPixels = 6;
    int overlapTrimExtraCropPixels = 2;
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
    bool stablePlaneMerge = false;
    bool historyTracking = true;
    bool pclClustering = true;
    bool realtime30 = true;
    bool realtimeSkipDepthPost = true;
    bool realtimeFastDepthPost = true;
    bool realtimeLiteVisualization = true;
    bool realtimeDisablePcl = true;
    bool realtimeDisableDepthConfirmSplit = true;
    bool realtimeDepthOnlyBoundary = true;
    bool colorContourCompletion = false;
    bool colorContourPrimary = false;
    bool farDistanceIntervals = true;
    bool nearPlaneDisplay = true;
    bool extraCandidatesInMosaic = false;
    bool overlapTrim = true;
    bool clusterMap = false;
    bool mosaicForegroundGate = true;
    bool qualitySegmentation = true;
    bool colorSegmentation = true;
    bool colorRefineDepthMasks = true;
    bool stereoContourDistance = true;
    bool stereoContourReuseCachedOnRefresh = false;
    bool asyncColorContourRefresh = false;
    bool asyncColorContourLowPriority = false;
    bool colorContourRefreshMotionRoi = false;
    bool colorContourRefreshRoiDropStereoFailed = false;
    bool colorContourRefreshOnMotion = false;
    bool colorContourRefreshOnUnknownSpike = false;
    bool colorContourRefreshOnFarLoss = false;
    bool showPartNumbers = false;
    double contourApproxRatio = 0.0015;
    std::string clusterMapExportPath;
    std::string finalSegmentationExportPath;
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

struct FarDistanceMaterial
{
    cv::Rect roi;
    cv::Point center;
    int pixelCount = 0;
    int observedDepthMinMm = 0;
    int observedDepthMaxMm = 0;
    int medianDepthMm = 0;
    int intervalMinMm = 0;
    int intervalMaxMm = 0;
    int rankNearFirst = 0;
    double contourArea = 0.0;
    std::vector<cv::Point> contour;
};

struct ColorContourRegion
{
    int id = 0;
    cv::Rect roi;
    cv::Point center;
    int pixelCount = 0;
    double contourArea = 0.0;
    int estimatedDistanceMm = 0;
    int distanceUncertaintyMm = 0;
    int estimatedWidthMm = 0;
    int estimatedHeightMm = 0;
    double medianDisparityPx = 0.0;
    int matchedStereoPoints = 0;
    double contourConfidence = 0.0;
    double distanceConfidence = 0.0;
    std::vector<cv::Point> contour;
};

struct FinalSegmentationFrame
{
    cv::Mat idMap;
    cv::Mat overlay;
    std::vector<ObservationMaterial> depthMaterials;
    std::vector<ColorContourRegion> colorRegions;
};

enum class ClusterSpatialMode
{
    PreciseDepth3D,
    ApproxStereoContour,
    ImageOnlyContour,
    BackgroundPlane,
    FarBackground,
    DepthHoleCandidate,
    Unknown
};

struct ClusterInfo
{
    int id = 0;
    ClusterSpatialMode mode = ClusterSpatialMode::Unknown;
    cv::Rect bbox;
    cv::Point2f center;
    int pixelCount = 0;
    int depthMinMm = 0;
    int depthMeanMm = 0;
    int depthMaxMm = 0;
    double confidence = 0.0;
    std::string source;
};

struct ClusterMapFrame
{
    cv::Mat clusterIdMap;
    std::vector<ClusterInfo> clusters;
    double assignmentCoveragePercent = 0.0;
    double clusterCoveragePercent = 0.0;
    double unknownPercent = 0.0;
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
    double captureWaitAlignMs = 0.0;
    double depthPostMs = 0.0;
    double frameConvertMs = 0.0;
    double motionMs = 0.0;
    double grayPrepareMs = 0.0;
    double anchorMs = 0.0;
    double boundaryMs = 0.0;
    double extractMs = 0.0;
    double farExtractMs = 0.0;
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

struct ColorContourRefreshStats
{
    bool refreshed = false;
    bool cacheReused = false;
    bool reasonStartup = false;
    bool reasonInterval = false;
    bool reasonMotion = false;
    bool reasonUnknownSpike = false;
    bool reasonFarLoss = false;
    double motionDeltaPercent = 0.0;
    int regionCount = 0;
    int stereoValidCount = 0;
    int stereoReuseCount = 0;
    bool asyncSubmitted = false;
    bool asyncApplied = false;
    bool asyncDropped = false;
    bool asyncPending = false;
    bool cooldownSkipped = false;
    bool roiRefresh = false;
    int roiPixels = 0;
    int roiCandidatePixels = 0;
    bool roiRejectedEmpty = false;
    bool roiRejectedLarge = false;
    int roiRefreshedRegionCount = 0;
    int roiStereoReuseCount = 0;
    int roiStereoBuiltCount = 0;
    int roiStereoFailedCount = 0;
    int roiPreservedStereoCount = 0;
    int cacheAgeFrames = 0;
    double asyncWorkerMs = 0.0;
};

struct PoseState
{
    bool enabled = false;
    bool accelStreamEnabled = false;
    bool gyroStreamEnabled = false;
    bool accelValid = false;
    bool gyroValid = false;
    bool orientationValid = false;
    uint64_t accelFrames = 0;
    uint64_t gyroFrames = 0;
    double accelTimestampMs = 0.0;
    double gyroTimestampMs = 0.0;
    double rollDeg = 0.0;
    double pitchDeg = 0.0;
    double yawDeg = 0.0;
    cv::Vec3d accelMps2{0.0, 0.0, 0.0};
    cv::Vec3d gyroRadPerSec{0.0, 0.0, 0.0};
};

struct MotionState
{
    bool enabled = false;
    bool visualValid = false;
    bool imuAccelValid = false;
    bool imuGyroValid = false;
    uint64_t visualFrames = 0;
    int level = 0;
    double score = 0.0;
    double visualShiftXPixels = 0.0;
    double visualShiftYPixels = 0.0;
    double visualShiftPixels = 0.0;
    double visualResponse = 0.0;
    double visualShiftSmoothPixels = 0.0;
    double gyroRadPerSec = 0.0;
    double gyroSmoothRadPerSec = 0.0;
    double accelDeltaMps2 = 0.0;
    double accelDeltaSmoothMps2 = 0.0;
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
    bool commandControl = true;
    bool commandControlExplicit = false;
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

struct ProfileCsvConfig
{
    bool enabled = false;
    std::string csvPath;
};

struct PoseReadConfig
{
    bool enabled = false;
    bool overlay = true;
    bool gravityLine = true;
    bool logConsole = false;
    int logEveryFrames = 30;
    int smoothPercent = 20;
};

struct ImuGravityCheckConfig
{
    bool enabled = false;
    int sampleFrames = 120;
    int warmupFrames = 30;
    int zMaxAngleDeg = 10;
    int zMinDominancePercent = 90;
    int gravityNormTolerancePercent = 15;
    int maxAccelStdMilliMps2 = 250;
    int maxGyroMilliRadps = 50;
    std::string csvPath;
};

struct MotionDiagnosticsConfig
{
    bool enabled = false;
    bool overlay = true;
    bool logConsole = false;
    int logEveryFrames = 30;
    int smoothPercent = 35;
    int visualDownscaleWidthPixels = 160;
    int visualMinResponsePercent = 12;
    int visualMoveMilliPixels = 2000;
    int visualShakeMilliPixels = 8000;
    int gyroMoveMilliRadps = 80;
    int gyroShakeMilliRadps = 250;
    int accelDeltaMoveMilliMps2 = 450;
    int accelDeltaShakeMilliMps2 = 1400;
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
    explicit DepthPostProcessor(bool fastMode = false)
        : fastMode_(fastMode),
          depthToDisparity_(true),
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
        if (fastMode_)
        {
            filtered = spatial_.process(filtered);
            return filtered.as<rs2::depth_frame>();
        }

        filtered = depthToDisparity_.process(filtered);
        filtered = spatial_.process(filtered);
        filtered = temporal_.process(filtered);
        filtered = disparityToDepth_.process(filtered);
        filtered = holeFilling_.process(filtered);
        return filtered.as<rs2::depth_frame>();
    }

private:
    bool fastMode_ = false;
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
            parseIntOption(arg, "--stable-plane-merge-anchor-gap-px=", config.stablePlaneMergeAnchorGapPixels) ||
            parseIntOption(arg, "--stable-plane-merge-mask-gap-px=", config.stablePlaneMergeMaskGapPixels) ||
            parseIntOption(arg, "--stable-plane-merge-min-anchors=", config.stablePlaneMergeMinAnchors) ||
            parseIntOption(arg, "--stable-plane-merge-max-area-percent=", config.stablePlaneMergeMaxAreaPercent) ||
            parseIntOption(arg, "--stable-plane-merge-max-roi-area-percent=", config.stablePlaneMergeMaxRoiAreaPercent) ||
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
            parseIntOption(arg, "--realtime-pcl-frame-interval=", config.realtimePclFrameInterval) ||
            parseIntOption(arg, "--realtime-anchor-step-px=", config.realtimeAnchorStepPixels) ||
            parseIntOption(arg, "--realtime-stable-contour-min-anchors=", config.realtimeStableContourMinAnchors) ||
            parseIntOption(arg, "--realtime-depth-slice-mm=", config.realtimeDepthSliceMm) ||
            parseIntOption(arg, "--color-contour-completion-padding-px=", config.colorContourCompletionPaddingPixels) ||
            parseIntOption(arg, "--color-contour-completion-min-iou-percent=", config.colorContourCompletionMinIouPercent) ||
            parseIntOption(arg, "--color-contour-completion-max-area-delta-percent=", config.colorContourCompletionMaxAreaDeltaPercent) ||
            parseIntOption(arg, "--color-contour-completion-max-center-shift-px=", config.colorContourCompletionMaxCenterShiftPixels) ||
            parseIntOption(arg, "--color-contour-completion-close-px=", config.colorContourCompletionClosePixels) ||
            parseIntOption(arg, "--color-contour-completion-other-guard-px=", config.colorContourCompletionOtherGuardPixels) ||
            parseIntOption(arg, "--color-contour-completion-max-other-overlap-px=", config.colorContourCompletionMaxOtherOverlapPixels) ||
            parseIntOption(arg, "--color-contour-primary-min-anchors=", config.colorContourPrimaryMinAnchors) ||
            parseIntOption(arg, "--color-contour-primary-min-overlap-percent=", config.colorContourPrimaryMinOverlapPercent) ||
            parseIntOption(arg, "--color-contour-primary-max-area-delta-percent=", config.colorContourPrimaryMaxAreaDeltaPercent) ||
            parseIntOption(arg, "--color-contour-primary-max-center-shift-px=", config.colorContourPrimaryMaxCenterShiftPixels) ||
            parseIntOption(arg, "--color-contour-primary-max-area-percent=", config.colorContourPrimaryMaxAreaPercent) ||
            parseIntOption(arg, "--far-max-depth-mm=", config.farMaxDepthMm) ||
            parseIntOption(arg, "--far-interval-mm=", config.farIntervalMm) ||
            parseIntOption(arg, "--far-min-area-px=", config.farMinAreaPixels) ||
            parseIntOption(arg, "--far-max-materials=", config.farMaxMaterials) ||
            parseIntOption(arg, "--far-morph-kernel-px=", config.farMorphKernelSize) ||
            parseIntOption(arg, "--near-plane-max-depth-mm=", config.nearPlaneMaxDepthMm) ||
            parseIntOption(arg, "--near-plane-min-area-percent=", config.nearPlaneMinAreaPercent) ||
            parseIntOption(arg, "--near-plane-max-materials=", config.nearPlaneMaxMaterials) ||
            parseIntOption(arg, "--near-plane-normal-min-percent=", config.nearPlaneNormalMinPercent) ||
            parseIntOption(arg, "--near-plane-min-center-y-percent=", config.nearPlaneMinCenterYPercent) ||
            parseIntOption(arg, "--near-plane-sample-step-px=", config.nearPlaneSampleStepPixels) ||
            parseIntOption(arg, "--near-plane-morph-kernel-px=", config.nearPlaneMorphKernelSize) ||
            parseIntOption(arg, "--near-plane-frame-interval=", config.nearPlaneFrameInterval) ||
            parseIntOption(arg, "--cluster-map-visual-min-area-px=", config.clusterMapVisualMinAreaPixels) ||
            parseIntOption(arg, "--cluster-map-visual-morph-kernel-px=", config.clusterMapVisualMorphKernelSize) ||
            parseIntOption(arg, "--cluster-map-max-visual-regions=", config.clusterMapMaxVisualRegions) ||
            parseIntOption(arg, "--mosaic-max-material-area-percent=", config.mosaicMaxMaterialAreaPercent) ||
            parseIntOption(arg, "--mosaic-border-reject-area-percent=", config.mosaicBorderRejectAreaPercent) ||
            parseIntOption(arg, "--mosaic-min-track-observations=", config.mosaicMinTrackObservations) ||
            parseIntOption(arg, "--mosaic-min-track-score=", config.mosaicMinTrackScore) ||
            parseIntOption(arg, "--mosaic-near-min-track-observations=", config.mosaicNearMinTrackObservations) ||
            parseIntOption(arg, "--mosaic-near-min-track-score=", config.mosaicNearMinTrackScore) ||
            parseIntOption(arg, "--mosaic-max-track-stale-frames=", config.mosaicMaxTrackStaleFrames) ||
            parseIntOption(arg, "--color-segmentation-min-area-px=", config.colorSegmentationMinAreaPixels) ||
            parseIntOption(arg, "--color-segmentation-morph-kernel-px=", config.colorSegmentationMorphKernelSize) ||
            parseIntOption(arg, "--color-segmentation-max-regions=", config.colorSegmentationMaxRegions) ||
            parseIntOption(arg, "--color-segmentation-max-roi-area-percent=", config.colorSegmentationMaxRoiAreaPercent) ||
            parseIntOption(arg, "--color-segmentation-border-reject-area-percent=", config.colorSegmentationBorderRejectAreaPercent) ||
            parseIntOption(arg, "--color-segmentation-color-bins=", config.colorSegmentationColorBins) ||
            parseIntOption(arg, "--color-segmentation-mean-shift-spatial=", config.colorSegmentationMeanShiftSpatial) ||
            parseIntOption(arg, "--color-segmentation-mean-shift-color=", config.colorSegmentationMeanShiftColor) ||
            parseIntOption(arg, "--color-refine-min-overlap-percent=", config.colorRefineMinOverlapPercent) ||
            parseIntOption(arg, "--color-refine-max-area-delta-percent=", config.colorRefineMaxAreaDeltaPercent) ||
            parseIntOption(arg, "--non-precise-color-ownership-min-percent=", config.nonPreciseColorOwnershipMinPercent) ||
            parseIntOption(arg, "--d455-precision-min-far-depth-support-percent=", config.nonPreciseColorOwnershipMinPercent) ||
            parseIntOption(arg, "--d455-precision-min-color-depth-support-percent=", config.nonPreciseColorOwnershipMinPercent) ||
            parseIntOption(arg, "--analysis-export-every-n=", config.analysisExportEveryN) ||
            parseIntOption(arg, "--color-contour-frame-interval=", config.colorContourFrameInterval) ||
            parseIntOption(arg, "--color-contour-refresh-min-gap-frames=", config.colorContourRefreshMinGapFrames) ||
            parseIntOption(arg, "--color-contour-refresh-roi-padding-px=", config.colorContourRefreshRoiPaddingPixels) ||
            parseIntOption(arg, "--color-contour-refresh-max-roi-area-percent=", config.colorContourRefreshMaxRoiAreaPercent) ||
            parseIntOption(arg, "--color-contour-refresh-motion-delta-percent=", config.colorContourRefreshMotionDeltaPercent) ||
            parseIntOption(arg, "--color-contour-refresh-unknown-percent=", config.colorContourRefreshUnknownPercent) ||
            parseIntOption(arg, "--stereo-contour-min-disparity-tenths-px=", config.stereoContourMinDisparityTenthsPx) ||
            parseIntOption(arg, "--stereo-contour-max-vertical-shift-px=", config.stereoContourMaxVerticalShiftPixels) ||
            parseIntOption(arg, "--stereo-contour-search-margin-px=", config.stereoContourSearchMarginPixels) ||
            parseIntOption(arg, "--stereo-contour-max-regions-per-frame=", config.stereoContourMaxRegionsPerFrame) ||
            parseIntOption(arg, "--stereo-contour-max-roi-area-percent=", config.stereoContourMaxRoiAreaPercent) ||
            parseIntOption(arg, "--stereo-contour-baseline-mm=", config.stereoContourBaselineMm) ||
            parseIntOption(arg, "--processed-view-scale-percent=", config.processedViewScalePercent) ||
            parseIntOption(arg, "--overlap-trim-min-support-percent=", config.overlapTrimMinSupportPercent) ||
            parseIntOption(arg, "--overlap-trim-padding-px=", config.overlapTrimPaddingPixels) ||
            parseIntOption(arg, "--overlap-trim-extra-crop-px=", config.overlapTrimExtraCropPixels))
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
        if (arg == "--stable-plane-merge")
        {
            config.stablePlaneMerge = true;
            continue;
        }
        if (arg == "--no-stable-plane-merge")
        {
            config.stablePlaneMerge = false;
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
        if (arg == "--pcl-clustering")
        {
            config.pclClustering = true;
            continue;
        }
        if (arg == "--realtime-30")
        {
            config.realtime30 = true;
            continue;
        }
        if (arg == "--no-realtime-30")
        {
            config.realtime30 = false;
            continue;
        }
        if (arg == "--realtime-skip-depth-post")
        {
            config.realtimeSkipDepthPost = true;
            continue;
        }
        if (arg == "--no-realtime-skip-depth-post")
        {
            config.realtimeSkipDepthPost = false;
            continue;
        }
        if (arg == "--realtime-fast-depth-post")
        {
            config.realtimeFastDepthPost = true;
            continue;
        }
        if (arg == "--no-realtime-fast-depth-post")
        {
            config.realtimeFastDepthPost = false;
            continue;
        }
        if (arg == "--realtime-lite-visualization")
        {
            config.realtimeLiteVisualization = true;
            continue;
        }
        if (arg == "--no-realtime-lite-visualization")
        {
            config.realtimeLiteVisualization = false;
            continue;
        }
        if (arg == "--realtime-disable-pcl")
        {
            config.realtimeDisablePcl = true;
            continue;
        }
        if (arg == "--no-realtime-disable-pcl")
        {
            config.realtimeDisablePcl = false;
            continue;
        }
        if (arg == "--realtime-disable-depth-confirm-split")
        {
            config.realtimeDisableDepthConfirmSplit = true;
            continue;
        }
        if (arg == "--no-realtime-disable-depth-confirm-split")
        {
            config.realtimeDisableDepthConfirmSplit = false;
            continue;
        }
        if (arg == "--realtime-depth-only-boundary")
        {
            config.realtimeDepthOnlyBoundary = true;
            continue;
        }
        if (arg == "--no-realtime-depth-only-boundary")
        {
            config.realtimeDepthOnlyBoundary = false;
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
        if (arg == "--color-contour-primary")
        {
            config.colorContourPrimary = true;
            continue;
        }
        if (arg == "--no-color-contour-primary")
        {
            config.colorContourPrimary = false;
            continue;
        }
        if (arg == "--far-distance-intervals")
        {
            config.farDistanceIntervals = true;
            continue;
        }
        if (arg == "--no-far-distance-intervals")
        {
            config.farDistanceIntervals = false;
            continue;
        }
        if (arg == "--near-plane-display")
        {
            config.nearPlaneDisplay = true;
            continue;
        }
        if (arg == "--no-near-plane-display")
        {
            config.nearPlaneDisplay = false;
            continue;
        }
        if (arg == "--extra-candidates-in-mosaic")
        {
            config.extraCandidatesInMosaic = true;
            continue;
        }
        if (arg == "--no-extra-candidates-in-mosaic")
        {
            config.extraCandidatesInMosaic = false;
            continue;
        }
        if (arg == "--overlap-trim")
        {
            config.overlapTrim = true;
            continue;
        }
        if (arg == "--no-overlap-trim")
        {
            config.overlapTrim = false;
            continue;
        }
        if (arg == "--cluster-map")
        {
            config.clusterMap = true;
            continue;
        }
        if (arg == "--no-cluster-map")
        {
            config.clusterMap = false;
            continue;
        }
        if (parseStringOption(arg, "--cluster-map-export=", config.clusterMapExportPath))
        {
            config.clusterMap = true;
            continue;
        }
        if (arg == "--mosaic-foreground-gate")
        {
            config.mosaicForegroundGate = true;
            continue;
        }
        if (arg == "--no-mosaic-foreground-gate")
        {
            config.mosaicForegroundGate = false;
            continue;
        }
        if (arg == "--quality-segmentation")
        {
            config.qualitySegmentation = true;
            continue;
        }
        if (arg == "--no-quality-segmentation")
        {
            config.qualitySegmentation = false;
            continue;
        }
        if (arg == "--color-segmentation")
        {
            config.colorSegmentation = true;
            continue;
        }
        if (arg == "--no-color-segmentation")
        {
            config.colorSegmentation = false;
            continue;
        }
        if (arg == "--color-refine-depth-masks")
        {
            config.colorRefineDepthMasks = true;
            continue;
        }
        if (arg == "--no-color-refine-depth-masks")
        {
            config.colorRefineDepthMasks = false;
            continue;
        }
        if (arg == "--stereo-contour-distance")
        {
            config.stereoContourDistance = true;
            continue;
        }
        if (arg == "--no-stereo-contour-distance")
        {
            config.stereoContourDistance = false;
            continue;
        }
        if (arg == "--stereo-contour-reuse-cached-on-refresh")
        {
            config.stereoContourReuseCachedOnRefresh = true;
            continue;
        }
        if (arg == "--no-stereo-contour-reuse-cached-on-refresh")
        {
            config.stereoContourReuseCachedOnRefresh = false;
            continue;
        }
        if (arg == "--async-color-contour-refresh")
        {
            config.asyncColorContourRefresh = true;
            continue;
        }
        if (arg == "--no-async-color-contour-refresh")
        {
            config.asyncColorContourRefresh = false;
            continue;
        }
        if (arg == "--async-color-contour-low-priority")
        {
            config.asyncColorContourLowPriority = true;
            continue;
        }
        if (arg == "--no-async-color-contour-low-priority")
        {
            config.asyncColorContourLowPriority = false;
            continue;
        }
        if (arg == "--color-contour-refresh-motion-roi")
        {
            config.colorContourRefreshMotionRoi = true;
            continue;
        }
        if (arg == "--no-color-contour-refresh-motion-roi")
        {
            config.colorContourRefreshMotionRoi = false;
            continue;
        }
        if (arg == "--color-contour-refresh-roi-drop-stereo-failed")
        {
            config.colorContourRefreshRoiDropStereoFailed = true;
            continue;
        }
        if (arg == "--no-color-contour-refresh-roi-drop-stereo-failed")
        {
            config.colorContourRefreshRoiDropStereoFailed = false;
            continue;
        }
        if (arg == "--color-contour-refresh-on-motion")
        {
            config.colorContourRefreshOnMotion = true;
            continue;
        }
        if (arg == "--no-color-contour-refresh-on-motion")
        {
            config.colorContourRefreshOnMotion = false;
            continue;
        }
        if (arg == "--color-contour-refresh-on-unknown-spike")
        {
            config.colorContourRefreshOnUnknownSpike = true;
            continue;
        }
        if (arg == "--no-color-contour-refresh-on-unknown-spike")
        {
            config.colorContourRefreshOnUnknownSpike = false;
            continue;
        }
        if (arg == "--color-contour-refresh-on-far-loss")
        {
            config.colorContourRefreshOnFarLoss = true;
            continue;
        }
        if (arg == "--no-color-contour-refresh-on-far-loss")
        {
            config.colorContourRefreshOnFarLoss = false;
            continue;
        }
        if (parseStringOption(arg, "--final-segmentation-export=", config.finalSegmentationExportPath))
        {
            config.qualitySegmentation = true;
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
        if (arg.rfind("--replay-dir=", 0) == 0)
        {
            continue;
        }
        if (arg.rfind("--capture-replay-dir=", 0) == 0 ||
            arg.rfind("--capture-replay-frames=", 0) == 0 ||
            arg.rfind("--capture-replay-warmup=", 0) == 0)
        {
            continue;
        }
        if (arg == "--record-video" ||
            arg == "--no-record-video" ||
            arg == "--record-command-control" ||
            arg == "--no-record-command-control" ||
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
        if (arg == "--profile-csv" ||
            arg.rfind("--profile-csv=", 0) == 0)
        {
            continue;
        }
        if (arg == "--pose-read" ||
            arg == "--no-pose-read" ||
            arg == "--pose-overlay" ||
            arg == "--no-pose-overlay" ||
            arg == "--gravity-line" ||
            arg == "--no-gravity-line" ||
            arg == "--pose-log" ||
            arg.rfind("--pose-log-every-n=", 0) == 0 ||
            arg.rfind("--pose-smooth-percent=", 0) == 0)
        {
            continue;
        }
        if (arg == "--imu-gravity-check" ||
            arg.rfind("--imu-gravity-frames=", 0) == 0 ||
            arg.rfind("--imu-gravity-warmup=", 0) == 0 ||
            arg.rfind("--imu-gravity-z-max-angle-deg=", 0) == 0 ||
            arg.rfind("--imu-gravity-z-min-dominance-percent=", 0) == 0 ||
            arg.rfind("--imu-gravity-norm-tolerance-percent=", 0) == 0 ||
            arg.rfind("--imu-gravity-max-accel-std-milli-mps2=", 0) == 0 ||
            arg.rfind("--imu-gravity-max-gyro-milliradps=", 0) == 0 ||
            arg.rfind("--imu-gravity-csv=", 0) == 0)
        {
            continue;
        }
        if (arg == "--motion-diagnostics" ||
            arg == "--no-motion-diagnostics" ||
            arg == "--motion-overlay" ||
            arg == "--no-motion-overlay" ||
            arg == "--motion-log" ||
            arg.rfind("--motion-log-every-n=", 0) == 0 ||
            arg.rfind("--motion-smooth-percent=", 0) == 0 ||
            arg.rfind("--motion-visual-downscale-width-px=", 0) == 0 ||
            arg.rfind("--motion-visual-min-response-percent=", 0) == 0 ||
            arg.rfind("--motion-visual-move-millipx=", 0) == 0 ||
            arg.rfind("--motion-visual-shake-millipx=", 0) == 0 ||
            arg.rfind("--motion-gyro-move-milliradps=", 0) == 0 ||
            arg.rfind("--motion-gyro-shake-milliradps=", 0) == 0 ||
            arg.rfind("--motion-accel-delta-move-milli-mps2=", 0) == 0 ||
            arg.rfind("--motion-accel-delta-shake-milli-mps2=", 0) == 0)
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
    config.stablePlaneMergeAnchorGapPixels = std::clamp(config.stablePlaneMergeAnchorGapPixels, 1, 64);
    config.stablePlaneMergeMaskGapPixels = std::clamp(config.stablePlaneMergeMaskGapPixels, 1, 127);
    config.stablePlaneMergeMinAnchors = std::clamp(config.stablePlaneMergeMinAnchors, 1, 1000000);
    config.stablePlaneMergeMaxAreaPercent = std::clamp(config.stablePlaneMergeMaxAreaPercent, 1, 100);
    config.stablePlaneMergeMaxRoiAreaPercent = std::clamp(config.stablePlaneMergeMaxRoiAreaPercent, 1, 100);
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
    config.realtimePclFrameInterval = std::clamp(config.realtimePclFrameInterval, 1, 60);
    config.realtimeAnchorStepPixels = std::clamp(config.realtimeAnchorStepPixels, 1, 32);
    config.realtimeStableContourMinAnchors = std::clamp(config.realtimeStableContourMinAnchors, 1, 10000);
    config.realtimeDepthSliceMm = std::clamp(config.realtimeDepthSliceMm, 50, 2000);
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
    config.colorContourPrimaryMinAnchors = std::clamp(config.colorContourPrimaryMinAnchors, 1, 1000000);
    config.colorContourPrimaryMinOverlapPercent = std::clamp(config.colorContourPrimaryMinOverlapPercent, 1, 100);
    config.colorContourPrimaryMaxAreaDeltaPercent =
        std::clamp(config.colorContourPrimaryMaxAreaDeltaPercent, 0, 1000);
    config.colorContourPrimaryMaxCenterShiftPixels =
        std::clamp(config.colorContourPrimaryMaxCenterShiftPixels, 1, 640);
    config.colorContourPrimaryMaxAreaPercent = std::clamp(config.colorContourPrimaryMaxAreaPercent, 1, 100);
    config.farMaxDepthMm = std::max(config.maxDepthMm + 1, config.farMaxDepthMm);
    config.farIntervalMm = std::clamp(config.farIntervalMm, 100, 10000);
    config.farMinAreaPixels = std::max(1, config.farMinAreaPixels);
    config.farMaxMaterials = std::clamp(config.farMaxMaterials, 1, 32);
    config.farMorphKernelSize = std::clamp(config.farMorphKernelSize | 1, 3, 31);
    config.nearPlaneMaxDepthMm = std::clamp(config.nearPlaneMaxDepthMm, config.minDepthMm, config.maxDepthMm);
    config.nearPlaneMinAreaPercent = std::clamp(config.nearPlaneMinAreaPercent, 1, 80);
    config.nearPlaneMaxMaterials = std::clamp(config.nearPlaneMaxMaterials, 1, 8);
    config.nearPlaneNormalMinPercent = std::clamp(config.nearPlaneNormalMinPercent, 1, 100);
    config.nearPlaneMinCenterYPercent = std::clamp(config.nearPlaneMinCenterYPercent, 0, 95);
    config.nearPlaneSampleStepPixels = std::clamp(config.nearPlaneSampleStepPixels, 2, 32);
    config.nearPlaneMorphKernelSize = std::clamp(config.nearPlaneMorphKernelSize | 1, 3, 31);
    config.nearPlaneFrameInterval = std::clamp(config.nearPlaneFrameInterval, 1, 120);
    config.clusterMapVisualMinAreaPixels = std::clamp(config.clusterMapVisualMinAreaPixels, 1, 200000);
    config.clusterMapVisualMorphKernelSize = std::clamp(config.clusterMapVisualMorphKernelSize | 1, 3, 31);
    config.clusterMapMaxVisualRegions = std::clamp(config.clusterMapMaxVisualRegions, 1, 200);
    config.mosaicMaxMaterialAreaPercent = std::clamp(config.mosaicMaxMaterialAreaPercent, 1, 100);
    config.mosaicBorderRejectAreaPercent = std::clamp(config.mosaicBorderRejectAreaPercent, 1, 100);
    config.mosaicMinTrackObservations = std::clamp(config.mosaicMinTrackObservations, 1, 300);
    config.mosaicMinTrackScore = std::clamp(config.mosaicMinTrackScore, 0, 100);
    config.mosaicNearMinTrackObservations = std::clamp(config.mosaicNearMinTrackObservations, 1, 120);
    config.mosaicNearMinTrackScore = std::clamp(config.mosaicNearMinTrackScore, 0, 100);
    config.mosaicMaxTrackStaleFrames = std::clamp(config.mosaicMaxTrackStaleFrames, 0, 120);
    config.colorSegmentationMinAreaPixels = std::clamp(config.colorSegmentationMinAreaPixels, 1, 200000);
    config.colorSegmentationMorphKernelSize = std::clamp(config.colorSegmentationMorphKernelSize | 1, 3, 31);
    config.colorSegmentationMaxRegions = std::clamp(config.colorSegmentationMaxRegions, 1, 200);
    config.colorSegmentationMaxRoiAreaPercent = std::clamp(config.colorSegmentationMaxRoiAreaPercent, 1, 100);
    config.colorSegmentationBorderRejectAreaPercent =
        std::clamp(config.colorSegmentationBorderRejectAreaPercent, 1, 100);
    config.colorSegmentationColorBins = std::clamp(config.colorSegmentationColorBins, 2, 12);
    config.colorSegmentationMeanShiftSpatial = std::clamp(config.colorSegmentationMeanShiftSpatial, 0, 40);
    config.colorSegmentationMeanShiftColor = std::clamp(config.colorSegmentationMeanShiftColor, 0, 80);
    config.colorRefineMinOverlapPercent = std::clamp(config.colorRefineMinOverlapPercent, 1, 100);
    config.colorRefineMaxAreaDeltaPercent = std::clamp(config.colorRefineMaxAreaDeltaPercent, 0, 1000);
    config.nonPreciseColorOwnershipMinPercent =
        std::clamp(config.nonPreciseColorOwnershipMinPercent, 1, 100);
    config.colorContourFrameInterval = std::clamp(config.colorContourFrameInterval, 1, 600);
    config.colorContourRefreshMotionDeltaPercent =
        std::clamp(config.colorContourRefreshMotionDeltaPercent, 1, 100);
    config.colorContourRefreshUnknownPercent =
        std::clamp(config.colorContourRefreshUnknownPercent, 1, 100);
    config.stereoContourMinDisparityTenthsPx = std::clamp(config.stereoContourMinDisparityTenthsPx, 1, 200);
    config.stereoContourMaxVerticalShiftPixels = std::clamp(config.stereoContourMaxVerticalShiftPixels, 0, 120);
    config.stereoContourSearchMarginPixels = std::clamp(config.stereoContourSearchMarginPixels, 0, 240);
    config.stereoContourMaxRegionsPerFrame = std::clamp(config.stereoContourMaxRegionsPerFrame, 1, 200);
    config.stereoContourMaxRoiAreaPercent = std::clamp(config.stereoContourMaxRoiAreaPercent, 1, 100);
    config.stereoContourBaselineMm = std::clamp(config.stereoContourBaselineMm, 1, 500);
    config.processedViewScalePercent = std::clamp(config.processedViewScalePercent, 25, 100);
    config.overlapTrimMinSupportPercent = std::clamp(config.overlapTrimMinSupportPercent, 1, 80);
    config.overlapTrimPaddingPixels = std::clamp(config.overlapTrimPaddingPixels, 0, 128);
    config.overlapTrimExtraCropPixels = std::clamp(config.overlapTrimExtraCropPixels, 0, 32);

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

bool hasOptionPrefix(int argc, char** argv, const std::string& prefix)
{
    for (int i = 1; i < argc; ++i)
    {
        if (std::string(argv[i]).rfind(prefix, 0) == 0)
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
        if (arg == "--record-command-control")
        {
            config.commandControl = true;
            config.commandControlExplicit = true;
            continue;
        }
        if (arg == "--no-record-command-control")
        {
            config.commandControl = false;
            config.commandControlExplicit = true;
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

rs2::frameset waitForLatestRgbdFrames(rs2::pipeline& pipeline)
{
    rs2::frameset latest = pipeline.wait_for_frames();
    rs2::frameset polled;
    int drained = 0;
    while (drained < 8 && pipeline.poll_for_frames(&polled))
    {
        if (polled.get_color_frame() && polled.get_depth_frame())
        {
            latest = polled;
        }
        ++drained;
    }

    return latest;
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

ProfileCsvConfig parseProfileCsvConfig(int argc, char** argv)
{
    ProfileCsvConfig config;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--profile-csv")
        {
            config.enabled = true;
            continue;
        }
        if (parseStringOption(arg, "--profile-csv=", config.csvPath))
        {
            config.enabled = true;
            continue;
        }
    }

    return config;
}

PoseReadConfig parsePoseReadConfig(int argc, char** argv)
{
    PoseReadConfig config;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--pose-read")
        {
            config.enabled = true;
            continue;
        }
        if (arg == "--no-pose-read")
        {
            config.enabled = false;
            continue;
        }
        if (arg == "--pose-overlay")
        {
            config.overlay = true;
            continue;
        }
        if (arg == "--no-pose-overlay")
        {
            config.overlay = false;
            continue;
        }
        if (arg == "--gravity-line")
        {
            config.gravityLine = true;
            continue;
        }
        if (arg == "--no-gravity-line")
        {
            config.gravityLine = false;
            continue;
        }
        if (arg == "--pose-log")
        {
            config.logConsole = true;
            continue;
        }
        parseIntOption(arg, "--pose-log-every-n=", config.logEveryFrames);
        parseIntOption(arg, "--pose-smooth-percent=", config.smoothPercent);
    }

    config.logEveryFrames = std::clamp(config.logEveryFrames, 1, 100000);
    config.smoothPercent = std::clamp(config.smoothPercent, 1, 100);
    return config;
}

ImuGravityCheckConfig parseImuGravityCheckConfig(int argc, char** argv)
{
    ImuGravityCheckConfig config;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--imu-gravity-check")
        {
            config.enabled = true;
            continue;
        }
        parseIntOption(arg, "--imu-gravity-frames=", config.sampleFrames);
        parseIntOption(arg, "--imu-gravity-warmup=", config.warmupFrames);
        parseIntOption(arg, "--imu-gravity-z-max-angle-deg=", config.zMaxAngleDeg);
        parseIntOption(arg, "--imu-gravity-z-min-dominance-percent=", config.zMinDominancePercent);
        parseIntOption(arg, "--imu-gravity-norm-tolerance-percent=", config.gravityNormTolerancePercent);
        parseIntOption(arg, "--imu-gravity-max-accel-std-milli-mps2=", config.maxAccelStdMilliMps2);
        parseIntOption(arg, "--imu-gravity-max-gyro-milliradps=", config.maxGyroMilliRadps);
        parseStringOption(arg, "--imu-gravity-csv=", config.csvPath);
    }

    config.sampleFrames = std::clamp(config.sampleFrames, 10, 2000);
    config.warmupFrames = std::clamp(config.warmupFrames, 0, 2000);
    config.zMaxAngleDeg = std::clamp(config.zMaxAngleDeg, 1, 45);
    config.zMinDominancePercent = std::clamp(config.zMinDominancePercent, 50, 100);
    config.gravityNormTolerancePercent = std::clamp(config.gravityNormTolerancePercent, 1, 50);
    config.maxAccelStdMilliMps2 = std::clamp(config.maxAccelStdMilliMps2, 1, 5000);
    config.maxGyroMilliRadps = std::clamp(config.maxGyroMilliRadps, 1, 1000);
    return config;
}

MotionDiagnosticsConfig parseMotionDiagnosticsConfig(int argc, char** argv)
{
    MotionDiagnosticsConfig config;
    for (int i = 1; i < argc; ++i)
    {
        const std::string arg = argv[i];
        if (arg == "--motion-diagnostics")
        {
            config.enabled = true;
            continue;
        }
        if (arg == "--no-motion-diagnostics")
        {
            config.enabled = false;
            continue;
        }
        if (arg == "--motion-overlay")
        {
            config.overlay = true;
            continue;
        }
        if (arg == "--no-motion-overlay")
        {
            config.overlay = false;
            continue;
        }
        if (arg == "--motion-log")
        {
            config.logConsole = true;
            continue;
        }
        parseIntOption(arg, "--motion-log-every-n=", config.logEveryFrames);
        parseIntOption(arg, "--motion-smooth-percent=", config.smoothPercent);
        parseIntOption(arg, "--motion-visual-downscale-width-px=", config.visualDownscaleWidthPixels);
        parseIntOption(arg, "--motion-visual-min-response-percent=", config.visualMinResponsePercent);
        parseIntOption(arg, "--motion-visual-move-millipx=", config.visualMoveMilliPixels);
        parseIntOption(arg, "--motion-visual-shake-millipx=", config.visualShakeMilliPixels);
        parseIntOption(arg, "--motion-gyro-move-milliradps=", config.gyroMoveMilliRadps);
        parseIntOption(arg, "--motion-gyro-shake-milliradps=", config.gyroShakeMilliRadps);
        parseIntOption(arg, "--motion-accel-delta-move-milli-mps2=", config.accelDeltaMoveMilliMps2);
        parseIntOption(arg, "--motion-accel-delta-shake-milli-mps2=", config.accelDeltaShakeMilliMps2);
    }

    config.logEveryFrames = std::clamp(config.logEveryFrames, 1, 100000);
    config.smoothPercent = std::clamp(config.smoothPercent, 1, 100);
    config.visualDownscaleWidthPixels = std::clamp(config.visualDownscaleWidthPixels, 64, 640);
    config.visualMinResponsePercent = std::clamp(config.visualMinResponsePercent, 0, 100);
    config.visualMoveMilliPixels = std::clamp(config.visualMoveMilliPixels, 0, 100000);
    config.visualShakeMilliPixels =
        std::max(config.visualMoveMilliPixels, std::clamp(config.visualShakeMilliPixels, 1, 200000));
    config.gyroMoveMilliRadps = std::clamp(config.gyroMoveMilliRadps, 0, 5000);
    config.gyroShakeMilliRadps =
        std::max(config.gyroMoveMilliRadps, std::clamp(config.gyroShakeMilliRadps, 1, 10000));
    config.accelDeltaMoveMilliMps2 = std::clamp(config.accelDeltaMoveMilliMps2, 0, 20000);
    config.accelDeltaShakeMilliMps2 =
        std::max(config.accelDeltaMoveMilliMps2, std::clamp(config.accelDeltaShakeMilliMps2, 1, 50000));
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

bool deviceSupportsStream(const rs2::device& device, rs2_stream streamType)
{
    try
    {
        for (const rs2::sensor& sensor : device.query_sensors())
        {
            for (const rs2::stream_profile& profile : sensor.get_stream_profiles())
            {
                if (profile.stream_type() == streamType)
                {
                    return true;
                }
            }
        }
    }
    catch (const rs2::error&)
    {
        return false;
    }

    return false;
}

bool deviceListSupportsStream(const rs2::device_list& devices, rs2_stream streamType)
{
    for (const rs2::device& device : devices)
    {
        if (deviceSupportsStream(device, streamType))
        {
            return true;
        }
    }

    return false;
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

constexpr double kPi = 3.14159265358979323846;

double radiansToDegrees(double radians)
{
    return radians * 180.0 / kPi;
}

double normalizeDegrees180(double degrees)
{
    while (degrees > 180.0)
    {
        degrees -= 360.0;
    }
    while (degrees < -180.0)
    {
        degrees += 360.0;
    }
    return degrees;
}

cv::Vec3d smoothVec3d(const cv::Vec3d& previous, const cv::Vec3d& sample, double alpha)
{
    return cv::Vec3d(
        previous[0] * (1.0 - alpha) + sample[0] * alpha,
        previous[1] * (1.0 - alpha) + sample[1] * alpha,
        previous[2] * (1.0 - alpha) + sample[2] * alpha);
}

std::string fixedNumber(double value, int precision)
{
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(precision) << value;
    return stream.str();
}

class PoseReader
{
public:
    PoseReader(const PoseReadConfig& config, bool accelStreamEnabled, bool gyroStreamEnabled)
        : alpha_(std::clamp(config.smoothPercent, 1, 100) / 100.0)
    {
        state_.enabled = config.enabled && (accelStreamEnabled || gyroStreamEnabled);
        state_.accelStreamEnabled = accelStreamEnabled;
        state_.gyroStreamEnabled = gyroStreamEnabled;
    }

    void update(const rs2::frameset& frames)
    {
        if (!state_.enabled)
        {
            return;
        }

        if (state_.accelStreamEnabled)
        {
            updateAccel(frames.first_or_default(RS2_STREAM_ACCEL));
        }
        if (state_.gyroStreamEnabled)
        {
            updateGyro(frames.first_or_default(RS2_STREAM_GYRO));
        }
    }

    const PoseState& state() const
    {
        return state_;
    }

private:
    void updateAccel(const rs2::frame& frame)
    {
        const rs2::motion_frame motion(frame);
        if (!motion)
        {
            return;
        }

        const rs2_vector data = motion.get_motion_data();
        const cv::Vec3d sample(data.x, data.y, data.z);
        state_.accelMps2 = state_.accelValid
            ? smoothVec3d(state_.accelMps2, sample, alpha_)
            : sample;
        state_.accelTimestampMs = motion.get_timestamp();
        state_.accelValid = true;
        ++state_.accelFrames;

        const double x = state_.accelMps2[0];
        const double y = state_.accelMps2[1];
        const double z = state_.accelMps2[2];
        const double yzNorm = std::sqrt(y * y + z * z);
        const double norm = std::sqrt(x * x + y * y + z * z);
        if (norm > 1e-6)
        {
            state_.rollDeg = radiansToDegrees(std::atan2(y, z));
            state_.pitchDeg = radiansToDegrees(std::atan2(-x, yzNorm));
            state_.orientationValid =
                std::isfinite(state_.rollDeg) &&
                std::isfinite(state_.pitchDeg);
        }
    }

    void updateGyro(const rs2::frame& frame)
    {
        const rs2::motion_frame motion(frame);
        if (!motion)
        {
            return;
        }

        const rs2_vector data = motion.get_motion_data();
        const cv::Vec3d sample(data.x, data.y, data.z);
        const double timestampMs = motion.get_timestamp();
        const bool hadGyro = state_.gyroValid;
        state_.gyroRadPerSec = hadGyro
            ? smoothVec3d(state_.gyroRadPerSec, sample, alpha_)
            : sample;

        if (hadGyro && timestampMs > lastGyroTimestampMs_)
        {
            const double dtSec = (timestampMs - lastGyroTimestampMs_) / 1000.0;
            if (dtSec > 0.0 && dtSec < 0.5)
            {
                state_.yawDeg = normalizeDegrees180(
                    state_.yawDeg + radiansToDegrees(state_.gyroRadPerSec[2] * dtSec));
            }
        }

        state_.gyroTimestampMs = timestampMs;
        state_.gyroValid = true;
        lastGyroTimestampMs_ = timestampMs;
        ++state_.gyroFrames;
    }

    double alpha_ = 0.20;
    double lastGyroTimestampMs_ = 0.0;
    PoseState state_;
};

std::string poseStateSummary(uint64_t frameId, const PoseState& state)
{
    std::ostringstream stream;
    stream << "pose frame=" << frameId
        << " accel=" << (state.accelValid ? "ok" : "missing")
        << " gyro=" << (state.gyroValid ? "ok" : "missing");
    if (state.orientationValid)
    {
        stream << " roll=" << fixedNumber(state.rollDeg, 2)
            << " pitch=" << fixedNumber(state.pitchDeg, 2)
            << " yaw_rel=" << fixedNumber(state.yawDeg, 2);
    }
    return stream.str();
}

void drawOutlinedText(
    cv::Mat& view,
    const std::string& text,
    const cv::Point& origin,
    double scale = 0.46,
    const cv::Scalar& foreground = cv::Scalar(255, 255, 255))
{
    cv::putText(
        view,
        text,
        origin,
        cv::FONT_HERSHEY_SIMPLEX,
        scale,
        cv::Scalar(0, 0, 0),
        3,
        cv::LINE_AA);
    cv::putText(
        view,
        text,
        origin,
        cv::FONT_HERSHEY_SIMPLEX,
        scale,
        foreground,
        1,
        cv::LINE_AA);
}

void drawPoseOverlay(cv::Mat& view, const PoseState& state)
{
    if (!state.enabled || view.empty())
    {
        return;
    }

    const std::string poseLine = state.orientationValid
        ? "pose roll=" + fixedNumber(state.rollDeg, 1) +
            " pitch=" + fixedNumber(state.pitchDeg, 1) +
            " yaw_rel=" + fixedNumber(state.yawDeg, 1)
        : "pose waiting for accel/gyro";
    const std::string streamLine =
        "imu accel=" + std::string(state.accelValid ? "ok" : "missing") +
        " gyro=" + std::string(state.gyroValid ? "ok" : "missing") +
        " frames=" + std::to_string(state.accelFrames) + "/" + std::to_string(state.gyroFrames);

    drawOutlinedText(view, poseLine, cv::Point(12, 24), 0.46, cv::Scalar(80, 255, 255));
    drawOutlinedText(view, streamLine, cv::Point(12, 46), 0.42, cv::Scalar(255, 255, 255));
}

std::string dominantSignedAxisLabel(const cv::Vec3d& vector)
{
    static constexpr std::array<char, 3> axisNames{'X', 'Y', 'Z'};
    int bestAxis = 0;
    for (int axis = 1; axis < 3; ++axis)
    {
        if (std::abs(vector[axis]) > std::abs(vector[bestAxis]))
        {
            bestAxis = axis;
        }
    }

    std::string label;
    label += vector[bestAxis] >= 0.0 ? '+' : '-';
    label += axisNames[bestAxis];
    return label;
}

void drawGravityDirectionOverlay(cv::Mat& view, const PoseState& state)
{
    if (!state.enabled || !state.accelValid || view.empty())
    {
        return;
    }

    const double x = state.accelMps2[0];
    const double y = state.accelMps2[1];
    const double z = state.accelMps2[2];
    const double norm = std::sqrt(x * x + y * y + z * z);
    if (norm < 1e-6)
    {
        return;
    }

    // Image Y grows downward. Negating IMU Y makes a -Y-dominant gravity estimate point down on screen.
    const double screenX = x;
    const double screenY = -y;
    const double planarNorm = std::sqrt(screenX * screenX + screenY * screenY);
    const cv::Point center(view.cols / 2, view.rows / 2);
    const int baseThickness = std::max(5, std::min(view.cols, view.rows) / 85);
    const int shadowThickness = baseThickness + 5;
    const double maxLength = std::min(view.cols, view.rows) * 0.36;
    const double visibleRatio = std::clamp(planarNorm / norm, 0.18, 1.0);
    const double arrowLength = maxLength * visibleRatio;

    cv::Point tip = center;
    if (planarNorm > 1e-6)
    {
        tip.x += static_cast<int>(std::round(screenX / planarNorm * arrowLength));
        tip.y += static_cast<int>(std::round(screenY / planarNorm * arrowLength));
    }

    cv::arrowedLine(view, center, tip, cv::Scalar(0, 0, 0), shadowThickness, cv::LINE_AA, 0, 0.22);
    cv::arrowedLine(view, center, tip, cv::Scalar(0, 255, 255), baseThickness, cv::LINE_AA, 0, 0.22);
    cv::circle(view, center, baseThickness + 3, cv::Scalar(0, 0, 0), cv::FILLED, cv::LINE_AA);
    cv::circle(view, center, baseThickness, cv::Scalar(255, 255, 255), cv::FILLED, cv::LINE_AA);

    const std::string label =
        "gravity " + dominantSignedAxisLabel(state.accelMps2) +
        " |g|=" + fixedNumber(norm, 2) +
        " z=" + fixedNumber(z, 2);
    cv::Point labelOrigin(tip.x + 10, tip.y - 10);
    const int margin = 8;
    const int maxX = std::max(margin, view.cols - 230);
    const int maxY = std::max(42, view.rows - margin);
    labelOrigin.x = std::clamp(labelOrigin.x, margin, maxX);
    labelOrigin.y = std::clamp(labelOrigin.y, 42, maxY);
    drawOutlinedText(view, label, labelOrigin, 0.54, cv::Scalar(0, 255, 255));
}

double vec3Magnitude(const cv::Vec3d& value)
{
    return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

double smoothScalar(double previous, double sample, double alpha, bool hadPrevious)
{
    return hadPrevious
        ? previous * (1.0 - alpha) + sample * alpha
        : sample;
}

const char* motionLevelLabel(int level)
{
    if (level >= 2)
    {
        return "shaking";
    }
    if (level == 1)
    {
        return "moving";
    }
    return "stable";
}

class MotionDiagnostics
{
public:
    explicit MotionDiagnostics(const MotionDiagnosticsConfig& config)
        : config_(config),
          alpha_(std::clamp(config.smoothPercent, 1, 100) / 100.0)
    {
        state_.enabled = config.enabled;
    }

    void update(const cv::Mat& colorBgr, const PoseState& poseState)
    {
        state_.enabled = config_.enabled;
        if (!config_.enabled)
        {
            return;
        }

        updateVisual(colorBgr);
        updateImu(poseState);
        updateLevel();
    }

    const MotionState& state() const
    {
        return state_;
    }

private:
    void updateVisual(const cv::Mat& colorBgr)
    {
        if (colorBgr.empty())
        {
            state_.visualValid = false;
            return;
        }

        cv::Mat gray8;
        if (colorBgr.channels() == 1)
        {
            gray8 = colorBgr;
        }
        else
        {
            cv::cvtColor(colorBgr, gray8, cv::COLOR_BGR2GRAY);
        }

        const int width = std::min(config_.visualDownscaleWidthPixels, gray8.cols);
        const int height = std::max(1, gray8.rows * width / std::max(1, gray8.cols));
        cv::Mat small8;
        cv::resize(gray8, small8, cv::Size(width, height), 0.0, 0.0, cv::INTER_AREA);

        cv::Mat current32;
        small8.convertTo(current32, CV_32F, 1.0 / 255.0);
        if (previousVisual32_.empty() || previousVisual32_.size() != current32.size())
        {
            previousVisual32_ = current32;
            hanningWindow_.release();
            state_.visualValid = false;
            return;
        }
        if (hanningWindow_.empty() || hanningWindow_.size() != current32.size())
        {
            cv::createHanningWindow(hanningWindow_, current32.size(), CV_32F);
        }

        double response = 0.0;
        const cv::Point2d shift = cv::phaseCorrelate(previousVisual32_, current32, hanningWindow_, &response);
        previousVisual32_ = current32;

        state_.visualResponse = std::isfinite(response) ? response : 0.0;
        const bool responseOk =
            state_.visualResponse * 100.0 >= static_cast<double>(config_.visualMinResponsePercent);
        if (!responseOk || !std::isfinite(shift.x) || !std::isfinite(shift.y))
        {
            state_.visualValid = false;
            return;
        }

        const double scaleX = static_cast<double>(gray8.cols) / static_cast<double>(current32.cols);
        const double scaleY = static_cast<double>(gray8.rows) / static_cast<double>(current32.rows);
        state_.visualShiftXPixels = shift.x * scaleX;
        state_.visualShiftYPixels = shift.y * scaleY;
        state_.visualShiftPixels = std::sqrt(
            state_.visualShiftXPixels * state_.visualShiftXPixels +
            state_.visualShiftYPixels * state_.visualShiftYPixels);
        state_.visualShiftSmoothPixels = smoothScalar(
            state_.visualShiftSmoothPixels,
            state_.visualShiftPixels,
            alpha_,
            state_.visualValid);
        state_.visualValid = true;
        ++state_.visualFrames;
    }

    void updateImu(const PoseState& poseState)
    {
        state_.imuGyroValid = poseState.enabled && poseState.gyroValid;
        if (state_.imuGyroValid)
        {
            state_.gyroRadPerSec = vec3Magnitude(poseState.gyroRadPerSec);
            state_.gyroSmoothRadPerSec = smoothScalar(
                state_.gyroSmoothRadPerSec,
                state_.gyroRadPerSec,
                alpha_,
                hadGyro_);
            hadGyro_ = true;
        }

        state_.imuAccelValid = false;
        if (poseState.enabled && poseState.accelValid)
        {
            if (hadAccel_)
            {
                const cv::Vec3d delta = poseState.accelMps2 - previousAccel_;
                state_.accelDeltaMps2 = vec3Magnitude(delta);
                state_.accelDeltaSmoothMps2 = smoothScalar(
                    state_.accelDeltaSmoothMps2,
                    state_.accelDeltaMps2,
                    alpha_,
                    hadAccelDelta_);
                hadAccelDelta_ = true;
                state_.imuAccelValid = true;
            }
            previousAccel_ = poseState.accelMps2;
            hadAccel_ = true;
        }
    }

    void updateLevel()
    {
        const double visualMove = config_.visualMoveMilliPixels / 1000.0;
        const double visualShake = config_.visualShakeMilliPixels / 1000.0;
        const double gyroMove = config_.gyroMoveMilliRadps / 1000.0;
        const double gyroShake = config_.gyroShakeMilliRadps / 1000.0;
        const double accelMove = config_.accelDeltaMoveMilliMps2 / 1000.0;
        const double accelShake = config_.accelDeltaShakeMilliMps2 / 1000.0;

        bool moving = false;
        bool shaking = false;
        double score = 0.0;
        auto addSignal = [&moving, &shaking, &score](bool valid, double value, double moveThreshold, double shakeThreshold)
        {
            if (!valid || shakeThreshold <= 0.0)
            {
                return;
            }
            score = std::max(score, value / shakeThreshold);
            if (value >= shakeThreshold)
            {
                shaking = true;
            }
            if (value >= moveThreshold)
            {
                moving = true;
            }
        };

        addSignal(state_.visualValid, state_.visualShiftSmoothPixels, visualMove, visualShake);
        addSignal(state_.imuGyroValid, state_.gyroSmoothRadPerSec, gyroMove, gyroShake);
        addSignal(state_.imuAccelValid, state_.accelDeltaSmoothMps2, accelMove, accelShake);

        state_.score = score;
        state_.level = shaking ? 2 : (moving ? 1 : 0);
    }

    MotionDiagnosticsConfig config_;
    double alpha_ = 0.35;
    MotionState state_;
    cv::Mat previousVisual32_;
    cv::Mat hanningWindow_;
    cv::Vec3d previousAccel_{0.0, 0.0, 0.0};
    bool hadAccel_ = false;
    bool hadAccelDelta_ = false;
    bool hadGyro_ = false;
};

std::string motionStateSummary(uint64_t frameId, const MotionState& state)
{
    std::ostringstream stream;
    const std::string visualSummary = state.visualValid
        ? fixedNumber(state.visualShiftSmoothPixels, 2) + "px"
        : "missing";
    stream << "motion frame=" << frameId
        << " level=" << motionLevelLabel(state.level)
        << " score=" << fixedNumber(state.score, 2)
        << " visual=" << visualSummary
        << " gyro=" << (state.imuGyroValid ? fixedNumber(state.gyroSmoothRadPerSec, 4) : "missing")
        << " accel_delta=" << (state.imuAccelValid ? fixedNumber(state.accelDeltaSmoothMps2, 3) : "missing");
    return stream.str();
}

void drawMotionOverlay(cv::Mat& view, const MotionState& state, int topY)
{
    if (!state.enabled || view.empty())
    {
        return;
    }

    const cv::Scalar color = state.level >= 2
        ? cv::Scalar(40, 80, 255)
        : (state.level == 1 ? cv::Scalar(60, 220, 255) : cv::Scalar(100, 255, 120));
    const std::string motionLine =
        "motion " + std::string(motionLevelLabel(state.level)) +
        " score=" + fixedNumber(state.score, 2) +
        " visual=" + (state.visualValid ? fixedNumber(state.visualShiftSmoothPixels, 1) + "px" : "wait");
    const std::string imuLine =
        "imu gyro=" + (state.imuGyroValid ? fixedNumber(state.gyroSmoothRadPerSec, 3) : "missing") +
        " accel_delta=" + (state.imuAccelValid ? fixedNumber(state.accelDeltaSmoothMps2, 2) : "missing");

    drawOutlinedText(view, motionLine, cv::Point(12, topY), 0.46, color);
    drawOutlinedText(view, imuLine, cv::Point(12, topY + 22), 0.42, cv::Scalar(255, 255, 255));
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

struct ReplayFramePaths
{
    uint64_t frameId = 0;
    std::filesystem::path colorPath;
    std::filesystem::path depthPath;
    std::filesystem::path irLeftPath;
    std::filesystem::path irRightPath;
};

bool parseFrameIdFromStem(const std::string& stem, uint64_t& frameId)
{
    const size_t suffixPos = stem.rfind("_color");
    const std::string idText = suffixPos == std::string::npos ? stem : stem.substr(0, suffixPos);
    if (idText.empty() || !std::all_of(idText.begin(), idText.end(), [](unsigned char ch) { return std::isdigit(ch); }))
    {
        return false;
    }
    frameId = static_cast<uint64_t>(std::stoull(idText));
    return true;
}

std::vector<ReplayFramePaths> listReplayFrames(const std::filesystem::path& replayDir)
{
    const std::filesystem::path framesDir = replayDir / "frames";
    if (!std::filesystem::exists(framesDir))
    {
        throw std::runtime_error("Replay frames directory does not exist: " + framesDir.string());
    }

    std::vector<ReplayFramePaths> frames;
    for (const std::filesystem::directory_entry& entry : std::filesystem::directory_iterator(framesDir))
    {
        if (!entry.is_regular_file())
        {
            continue;
        }
        const std::filesystem::path path = entry.path();
        if (path.extension() != ".png")
        {
            continue;
        }
        const std::string stem = path.stem().string();
        if (stem.size() < 6 || stem.substr(stem.size() - 6) != "_color")
        {
            continue;
        }
        uint64_t frameId = 0;
        if (!parseFrameIdFromStem(stem, frameId))
        {
            continue;
        }
        const std::string prefix = stem.substr(0, stem.size() - 6);
        ReplayFramePaths replayFrame;
        replayFrame.frameId = frameId;
        replayFrame.colorPath = path;
        replayFrame.depthPath = framesDir / (prefix + "_depth16.png");
        replayFrame.irLeftPath = framesDir / (prefix + "_ir_left.png");
        replayFrame.irRightPath = framesDir / (prefix + "_ir_right.png");
        if (std::filesystem::exists(replayFrame.depthPath))
        {
            frames.push_back(replayFrame);
        }
    }
    std::sort(frames.begin(), frames.end(), [](const ReplayFramePaths& lhs, const ReplayFramePaths& rhs) {
        return lhs.frameId < rhs.frameId;
    });
    return frames;
}

cv::Mat loadReplayDepth16(const std::filesystem::path& path)
{
    cv::Mat depth = cv::imread(path.string(), cv::IMREAD_UNCHANGED);
    if (depth.empty())
    {
        throw std::runtime_error("Failed to read replay depth frame: " + path.string());
    }
    if (depth.channels() > 1)
    {
        std::vector<cv::Mat> channels;
        cv::split(depth, channels);
        depth = channels.front();
    }
    if (depth.type() != CV_16UC1)
    {
        cv::Mat converted;
        depth.convertTo(converted, CV_16UC1);
        depth = converted;
    }
    return depth;
}

cv::Mat loadReplayGray8(const std::filesystem::path& path)
{
    if (!std::filesystem::exists(path))
    {
        return cv::Mat();
    }
    return cv::imread(path.string(), cv::IMREAD_GRAYSCALE);
}

rs2_intrinsics makeReplayIntrinsics(const cv::Size& size)
{
    rs2_intrinsics intrinsics{};
    intrinsics.width = size.width;
    intrinsics.height = size.height;
    intrinsics.ppx = static_cast<float>(size.width - 1) * 0.5f;
    intrinsics.ppy = static_cast<float>(size.height - 1) * 0.5f;
    intrinsics.fx = static_cast<float>(std::max(size.width, size.height)) * 0.6f;
    intrinsics.fy = intrinsics.fx;
    intrinsics.model = RS2_DISTORTION_NONE;
    for (float& coeff : intrinsics.coeffs)
    {
        coeff = 0.0f;
    }
    return intrinsics;
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

    int validDepthCount = 0;
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
            if (depthUnits == 0)
            {
                continue;
            }

            depthSumUnits += depthUnits;
            minDepthUnits = std::min(minDepthUnits, depthUnits);
            maxDepthUnits = std::max(maxDepthUnits, depthUnits);
            ++validDepthCount;

            if (!canDeproject)
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

    measurement.validDepth = validDepthCount > 0;
    if (!measurement.validDepth)
    {
        return measurement;
    }

    measurement.meanDepthMm =
        static_cast<int>((depthSumUnits / static_cast<double>(validDepthCount)) * depthScale * 1000.0f + 0.5f);
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
    cv::Mat contourSource = mask.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(contourSource, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    const int frameArea = mask.rows * mask.cols;
    const int maxAreaPixels = frameArea * config.maxAreaPercent / 100;
    const int maxRoiAreaPixels = frameArea * config.maxRoiAreaPercent / 100;
    const int maxBorderAreaPixels = frameArea * config.maxBorderAreaPercent / 100;
    const int foregroundMaxAreaPixels = frameArea * config.foregroundMaxAreaPercent / 100;
    const int foregroundMaxRoiAreaPixels = frameArea * config.foregroundMaxRoiAreaPercent / 100;
    const int maxMaterialAreaPixels = frameArea * config.maxMaterialAreaPercent / 100;
    const int maxMaterialRoiAreaPixels = frameArea * config.maxMaterialRoiAreaPercent / 100;

    for (const std::vector<cv::Point>& contour : contours)
    {
        if (contour.size() < 3)
        {
            continue;
        }

        const double contourArea = cv::contourArea(contour);
        if (contourArea < static_cast<double>(config.minAreaPixels))
        {
            continue;
        }

        const cv::Rect roi = cv::boundingRect(contour);
        if (roi.empty())
        {
            continue;
        }
        if (roi.area() > maxMaterialRoiAreaPixels)
        {
            continue;
        }

        cv::Mat localMask = mask(roi).clone();
        const int area = cv::countNonZero(localMask);
        if (area < config.minAreaPixels)
        {
            continue;
        }

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

        if (contourArea > static_cast<double>(maxMaterialAreaPixels) || roi.area() > maxMaterialRoiAreaPixels)
        {
            continue;
        }

        std::vector<cv::Point> preciseContour;
        const double epsilon = cv::arcLength(contour, true) * config.contourApproxRatio;
        if (epsilon >= 0.5)
        {
            cv::approxPolyDP(contour, preciseContour, epsilon, true);
        }
        if (preciseContour.size() < 3)
        {
            preciseContour = contour;
        }

        ObservationMaterial material;
        material.sourceFrameId = sourceFrameId;
        material.observationId = sourceFrameId * 1000ULL + nextObservationId++;
        material.roi = roi;
        const cv::Moments moments = cv::moments(contour);
        if (std::abs(moments.m00) > 1e-6)
        {
            material.center = cv::Point(
                static_cast<int>(moments.m10 / moments.m00 + 0.5),
                static_cast<int>(moments.m01 / moments.m00 + 0.5));
        }
        else
        {
            material.center = cv::Point(roi.x + roi.width / 2, roi.y + roi.height / 2);
        }
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

cv::Mat cleanFarDistanceMask(const cv::Mat& inputMask, const SegmentationConfig& config)
{
    cv::Mat mask = inputMask.clone();
    const int kernelSize = std::max(3, config.farMorphKernelSize | 1);
    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(kernelSize, kernelSize));

    cv::morphologyEx(mask, mask, cv::MORPH_OPEN, kernel);
    cv::morphologyEx(mask, mask, cv::MORPH_CLOSE, kernel);
    return mask;
}

cv::Mat makeFarDistanceBoundaryMask(
    const cv::Mat& edgeGrayOrBgr,
    const cv::Mat& farMask,
    const SegmentationConfig& config,
    bool edgeSourceIsInfrared)
{
    if (edgeGrayOrBgr.empty() || farMask.empty() || !config.boundarySplit)
    {
        return {};
    }

    cv::Mat gray;
    if (edgeGrayOrBgr.channels() == 1)
    {
        gray = edgeGrayOrBgr.clone();
    }
    else
    {
        cv::cvtColor(edgeGrayOrBgr, gray, cv::COLOR_BGR2GRAY);
    }
    if (gray.size() != farMask.size())
    {
        cv::resize(gray, gray, farMask.size(), 0.0, 0.0, cv::INTER_LINEAR);
    }

    cv::GaussianBlur(gray, gray, cv::Size(3, 3), 0.0);
    cv::Mat edges;
    const int edgeCannyLow = edgeSourceIsInfrared ? config.infraredCannyLow : config.colorCannyLow;
    const int edgeCannyHigh = edgeSourceIsInfrared ? config.infraredCannyHigh : config.colorCannyHigh;
    cv::Canny(gray, edges, edgeCannyLow, edgeCannyHigh, 3, true);
    cv::bitwise_and(edges, farMask, edges);

    const int kernelSize = std::max(1, config.splitBoundaryPixels | 1);
    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(kernelSize, kernelSize));
    cv::dilate(edges, edges, kernel);
    return edges;
}

int quantizeFarIntervalMinMm(int medianDepthMm, const SegmentationConfig& config)
{
    const int clampedDepth = std::clamp(medianDepthMm, config.maxDepthMm + 1, config.farMaxDepthMm);
    const int bucket = (clampedDepth - config.maxDepthMm - 1) / std::max(1, config.farIntervalMm);
    return config.maxDepthMm + bucket * config.farIntervalMm;
}

std::vector<FarDistanceMaterial> extractFarDistanceMaterials(
    const cv::Mat& depth16,
    const cv::Mat& splitBoundaryMask,
    const cv::Mat& edgeGrayOrBgr,
    bool edgeSourceIsInfrared,
    const SegmentationConfig& config,
    float depthScale)
{
    std::vector<FarDistanceMaterial> materials;
    if (!config.farDistanceIntervals || depth16.empty() || config.farMaxDepthMm <= config.maxDepthMm)
    {
        return materials;
    }

    cv::Mat farMask;
    cv::inRange(
        depth16,
        depthUnitsFromMm(config.maxDepthMm + 1, depthScale),
        depthUnitsFromMm(config.farMaxDepthMm, depthScale),
        farMask);

    farMask = cleanFarDistanceMask(farMask, config);
    if (!splitBoundaryMask.empty())
    {
        farMask.setTo(0, splitBoundaryMask);
    }
    const cv::Mat farBoundaryMask =
        makeFarDistanceBoundaryMask(edgeGrayOrBgr, farMask, config, edgeSourceIsInfrared);
    if (!farBoundaryMask.empty())
    {
        farMask.setTo(0, farBoundaryMask);
    }

    cv::Mat contourSource = farMask.clone();
    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(contourSource, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    for (const std::vector<cv::Point>& contour : contours)
    {
        if (contour.size() < 3)
        {
            continue;
        }

        const double contourArea = cv::contourArea(contour);
        if (contourArea < static_cast<double>(config.farMinAreaPixels))
        {
            continue;
        }

        const cv::Rect roi = cv::boundingRect(contour);
        if (roi.empty())
        {
            continue;
        }

        const cv::Mat localMask = farMask(roi);
        const int pixelCount = cv::countNonZero(localMask);
        if (pixelCount < config.farMinAreaPixels)
        {
            continue;
        }

        std::vector<int> depthSamplesMm;
        depthSamplesMm.reserve(static_cast<size_t>(pixelCount));
        int observedMinMm = std::numeric_limits<int>::max();
        int observedMaxMm = 0;
        for (int y = 0; y < roi.height; ++y)
        {
            const uint8_t* maskRow = localMask.ptr<uint8_t>(y);
            const uint16_t* depthRow = depth16.ptr<uint16_t>(roi.y + y) + roi.x;
            for (int x = 0; x < roi.width; ++x)
            {
                if (maskRow[x] == 0 || depthRow[x] == 0)
                {
                    continue;
                }

                const int depthMm = static_cast<int>(depthRow[x] * depthScale * 1000.0f + 0.5f);
                if (depthMm <= config.maxDepthMm || depthMm > config.farMaxDepthMm)
                {
                    continue;
                }

                depthSamplesMm.push_back(depthMm);
                observedMinMm = std::min(observedMinMm, depthMm);
                observedMaxMm = std::max(observedMaxMm, depthMm);
            }
        }

        if (depthSamplesMm.empty())
        {
            continue;
        }

        const size_t medianIndex = depthSamplesMm.size() / 2;
        std::nth_element(depthSamplesMm.begin(), depthSamplesMm.begin() + medianIndex, depthSamplesMm.end());
        const int medianDepthMm = depthSamplesMm[medianIndex];
        const int intervalMinMm = quantizeFarIntervalMinMm(medianDepthMm, config);
        const int intervalMaxMm = std::min(config.farMaxDepthMm, intervalMinMm + config.farIntervalMm);

        std::vector<cv::Point> preciseContour;
        const double epsilon = cv::arcLength(contour, true) * config.contourApproxRatio;
        if (epsilon >= 0.5)
        {
            cv::approxPolyDP(contour, preciseContour, epsilon, true);
        }
        if (preciseContour.size() < 3)
        {
            preciseContour = contour;
        }

        FarDistanceMaterial material;
        material.roi = roi;
        const cv::Moments moments = cv::moments(contour);
        if (std::abs(moments.m00) > 1e-6)
        {
            material.center = cv::Point(
                static_cast<int>(moments.m10 / moments.m00 + 0.5),
                static_cast<int>(moments.m01 / moments.m00 + 0.5));
        }
        else
        {
            material.center = cv::Point(roi.x + roi.width / 2, roi.y + roi.height / 2);
        }
        material.pixelCount = pixelCount;
        material.observedDepthMinMm = observedMinMm;
        material.observedDepthMaxMm = observedMaxMm;
        material.medianDepthMm = medianDepthMm;
        material.intervalMinMm = intervalMinMm;
        material.intervalMaxMm = intervalMaxMm;
        material.contourArea = contourArea;
        material.contour = std::move(preciseContour);
        materials.push_back(std::move(material));
    }

    std::sort(
        materials.begin(),
        materials.end(),
        [](const FarDistanceMaterial& lhs, const FarDistanceMaterial& rhs)
        {
            if (lhs.medianDepthMm != rhs.medianDepthMm)
            {
                return lhs.medianDepthMm < rhs.medianDepthMm;
            }
            if (lhs.observedDepthMinMm != rhs.observedDepthMinMm)
            {
                return lhs.observedDepthMinMm < rhs.observedDepthMinMm;
            }
            return lhs.contourArea > rhs.contourArea;
        });

    if (materials.size() > static_cast<size_t>(config.farMaxMaterials))
    {
        materials.resize(static_cast<size_t>(config.farMaxMaterials));
    }
    for (size_t index = 0; index < materials.size(); ++index)
    {
        materials[index].rankNearFirst = static_cast<int>(index + 1);
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

std::vector<ObservationMaterial> buildCurrentFrameDisplayBaseMaterials(
    const std::vector<ObservationMaterial>& stableMaterials,
    const std::vector<ObservationMaterial>& currentCandidates,
    const SegmentationConfig& config)
{
    if (stableMaterials.empty() || currentCandidates.empty() || !config.historyTracking)
    {
        return stableMaterials;
    }

    SegmentationConfig displayMatchConfig = config;
    displayMatchConfig.trackIouPercent = 0;
    displayMatchConfig.trackCenterGapPixels =
        std::max(config.trackCenterGapPixels * 4, config.trackCenterGapPixels + 96);
    displayMatchConfig.trackDepthGapMm = std::max(config.trackDepthGapMm * 2, config.trackDepthGapMm + 300);

    std::vector<CandidateTrackMatch> matches;
    matches.reserve(stableMaterials.size() * currentCandidates.size());
    for (size_t stableIndex = 0; stableIndex < stableMaterials.size(); ++stableIndex)
    {
        for (size_t candidateIndex = 0; candidateIndex < currentCandidates.size(); ++candidateIndex)
        {
            const double score = materialTrackMatchScore(
                stableMaterials[stableIndex],
                currentCandidates[candidateIndex],
                displayMatchConfig);
            if (score >= 0.0)
            {
                matches.push_back(CandidateTrackMatch{stableIndex, candidateIndex, score});
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

    std::vector<ObservationMaterial> displayMaterials = stableMaterials;
    std::vector<bool> stableUsed(stableMaterials.size(), false);
    std::vector<bool> candidateUsed(currentCandidates.size(), false);
    for (const CandidateTrackMatch& match : matches)
    {
        if (stableUsed[match.trackIndex] || candidateUsed[match.candidateIndex])
        {
            continue;
        }

        ObservationMaterial currentGeometry = currentCandidates[match.candidateIndex];
        currentGeometry.observationId = stableMaterials[match.trackIndex].observationId;
        currentGeometry.groupId = stableMaterials[match.trackIndex].groupId;
        displayMaterials[match.trackIndex] = std::move(currentGeometry);
        stableUsed[match.trackIndex] = true;
        candidateUsed[match.candidateIndex] = true;
    }

    return displayMaterials;
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

bool buildNearPlaneMaterialFromMask(
    const cv::Mat& componentMask,
    const cv::Mat& depth16,
    const SegmentationConfig& config,
    float depthScale,
    const rs2_intrinsics& intrinsics,
    uint64_t sourceFrameId,
    uint64_t& nextObservationId,
    ObservationMaterial& material)
{
    if (componentMask.empty())
    {
        return false;
    }

    const int pixelCount = cv::countNonZero(componentMask);
    const int minAreaPixels = std::max(1, depth16.rows * depth16.cols * config.nearPlaneMinAreaPercent / 100);
    if (pixelCount < minAreaPixels)
    {
        return false;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(componentMask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
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
    if (contourArea < static_cast<double>(minAreaPixels))
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

    const cv::Rect frameRect(0, 0, depth16.cols, depth16.rows);
    const cv::Rect roi = cv::boundingRect(preciseContour) & frameRect;
    if (roi.empty())
    {
        return false;
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

    const bool canDeproject = intrinsics.fx > 0.0f && intrinsics.fy > 0.0f;
    int pointCount = 0;
    for (int y = roi.y; y < roi.y + roi.height; ++y)
    {
        const uint8_t* maskRow = componentMask.ptr<uint8_t>(y);
        const uint16_t* depthRow = depth16.ptr<uint16_t>(y);
        for (int x = roi.x; x < roi.x + roi.width; ++x)
        {
            if (maskRow[x] == 0 || depthRow[x] == 0)
            {
                continue;
            }

            const uint16_t depthUnits = depthRow[x];
            minDepthUnits = std::min(minDepthUnits, depthUnits);
            maxDepthUnits = std::max(maxDepthUnits, depthUnits);
            depthSumUnits += depthUnits;
            ++validDepthCount;

            if (!canDeproject)
            {
                continue;
            }

            const cv::Point3f point = deprojectDepthPoint(x, y, depthUnits, depthScale, intrinsics);
            minPoint.x = std::min(minPoint.x, point.x);
            minPoint.y = std::min(minPoint.y, point.y);
            minPoint.z = std::min(minPoint.z, point.z);
            maxPoint.x = std::max(maxPoint.x, point.x);
            maxPoint.y = std::max(maxPoint.y, point.y);
            maxPoint.z = std::max(maxPoint.z, point.z);
            ++pointCount;
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
            static_cast<int>(moments.m10 / moments.m00 + 0.5),
            static_cast<int>(moments.m01 / moments.m00 + 0.5));
    }

    material = ObservationMaterial{};
    material.sourceFrameId = sourceFrameId;
    material.observationId = sourceFrameId * 1000ULL + 700ULL + nextObservationId++;
    material.roi = roi;
    material.center = center;
    material.pixelCount = pixelCount;
    material.depthMinMm = static_cast<int>(minDepthUnits * depthScale * 1000.0f + 0.5f);
    material.depthMaxMm = static_cast<int>(maxDepthUnits * depthScale * 1000.0f + 0.5f);
    material.observedDepthMinMm = material.depthMinMm;
    material.observedDepthMaxMm = material.depthMaxMm;
    material.meanDepthMm =
        static_cast<int>((depthSumUnits / static_cast<double>(validDepthCount)) * depthScale * 1000.0f + 0.5f);
    material.groupId = 700;
    material.contourArea = contourArea;
    material.hasPointCloudBounds = pointCount > 0;
    material.minPointMeters = minPoint;
    material.maxPointMeters = maxPoint;
    material.contour = std::move(preciseContour);
    return true;
}

std::vector<ObservationMaterial> extractNearPlaneDisplayMaterials(
    const cv::Mat& depth16,
    const cv::Mat& splitBoundaryMask,
    const SegmentationConfig& config,
    float depthScale,
    const rs2_intrinsics& intrinsics,
    uint64_t sourceFrameId)
{
    std::vector<ObservationMaterial> materials;
    if (!config.nearPlaneDisplay || depth16.empty() || intrinsics.fx <= 0.0f || intrinsics.fy <= 0.0f)
    {
        return materials;
    }

    cv::Mat validMask;
    cv::inRange(
        depth16,
        depthUnitsFromMm(config.minDepthMm, depthScale),
        depthUnitsFromMm(config.nearPlaneMaxDepthMm, depthScale),
        validMask);

    cv::Mat planeMask = cv::Mat::zeros(depth16.size(), CV_8UC1);
    const int sampleStep = config.nearPlaneSampleStepPixels;
    const int neighborStep = std::max(2, std::min(config.indoorPlaneNormalNeighborPixels, sampleStep * 2));
    const int minDepthUnits = depthUnitsFromMm(config.minDepthMm, depthScale);
    const int maxDepthUnits = depthUnitsFromMm(config.nearPlaneMaxDepthMm, depthScale);
    const double horizontalMin = config.nearPlaneNormalMinPercent / 100.0;
    const int minY = depth16.rows * config.nearPlaneMinCenterYPercent / 100;

    for (int y = std::max(neighborStep, minY); y + neighborStep < depth16.rows; y += sampleStep)
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
            const cv::Point3f normal = crossProduct(right - center, down - center);
            const double length = vectorLength(normal);
            if (length < 1e-6)
            {
                continue;
            }

            const double ny = std::abs(static_cast<double>(normal.y) / length);
            if (ny < horizontalMin)
            {
                continue;
            }

            const cv::Rect block(
                std::max(0, x - sampleStep / 2),
                std::max(0, y - sampleStep / 2),
                std::min(sampleStep, depth16.cols - std::max(0, x - sampleStep / 2)),
                std::min(sampleStep, depth16.rows - std::max(0, y - sampleStep / 2)));
            if (!block.empty())
            {
                planeMask(block).setTo(255);
            }
        }
    }

    const int kernelSize = config.nearPlaneMorphKernelSize | 1;
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
    cv::morphologyEx(planeMask, planeMask, cv::MORPH_CLOSE, kernel);
    cv::morphologyEx(planeMask, planeMask, cv::MORPH_OPEN, kernel);
    planeMask.setTo(0, ~validMask);
    if (!splitBoundaryMask.empty())
    {
        cv::Mat boundaryBarrier = dilateMask(splitBoundaryMask, std::max(3, config.splitBoundaryPixels * 2 + 1));
        planeMask.setTo(0, boundaryBarrier);
    }

    cv::Mat labels;
    cv::Mat stats;
    cv::Mat centroids;
    const int labelCount = cv::connectedComponentsWithStats(planeMask, labels, stats, centroids, 8, CV_32S);
    const int minAreaPixels = std::max(1, depth16.rows * depth16.cols * config.nearPlaneMinAreaPercent / 100);
    std::vector<int> labelsByArea;
    for (int label = 1; label < labelCount; ++label)
    {
        if (stats.at<int>(label, cv::CC_STAT_AREA) >= minAreaPixels)
        {
            labelsByArea.push_back(label);
        }
    }
    std::sort(
        labelsByArea.begin(),
        labelsByArea.end(),
        [&stats](int lhs, int rhs)
        {
            return stats.at<int>(lhs, cv::CC_STAT_AREA) > stats.at<int>(rhs, cv::CC_STAT_AREA);
        });

    uint64_t nextObservationId = 1;
    for (const int label : labelsByArea)
    {
        cv::Mat componentMask;
        cv::compare(labels, label, componentMask, cv::CMP_EQ);
        cv::bitwise_and(componentMask, validMask, componentMask);

        ObservationMaterial material;
        if (buildNearPlaneMaterialFromMask(
                componentMask,
                depth16,
                config,
                depthScale,
                intrinsics,
                sourceFrameId,
                nextObservationId,
                material))
        {
            materials.push_back(std::move(material));
            if (materials.size() >= static_cast<size_t>(config.nearPlaneMaxMaterials))
            {
                break;
            }
        }
    }

    return materials;
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
    bool allowColorPrimary,
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
    if (allowColorPrimary)
    {
        const int maxPrimaryArea =
            colorBgr.rows * colorBgr.cols * config.colorContourPrimaryMaxAreaPercent / 100;
        if (candidateArea > maxPrimaryArea)
        {
            return false;
        }
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
    const double originalOverlapPercent =
        100.0 * static_cast<double>(intersectionArea) / static_cast<double>(originalArea);
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

    const bool strictAccepted =
        iouPercent >= static_cast<double>(config.colorContourCompletionMinIouPercent) &&
        areaDeltaPercent <= static_cast<double>(config.colorContourCompletionMaxAreaDeltaPercent) &&
        centerShift <= static_cast<double>(config.colorContourCompletionMaxCenterShiftPixels);
    const bool primaryAccepted =
        allowColorPrimary &&
        originalOverlapPercent >= static_cast<double>(config.colorContourPrimaryMinOverlapPercent) &&
        areaDeltaPercent <= static_cast<double>(config.colorContourPrimaryMaxAreaDeltaPercent) &&
        centerShift <= static_cast<double>(config.colorContourPrimaryMaxCenterShiftPixels);
    if (!strictAccepted && !primaryAccepted)
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
    const cv::Mat& anchorMask,
    const cv::Mat& depth16,
    float depthScale,
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
        const StableContourSupport support =
            measureStableContourSupport(displayMaterials[index], anchorMask, depth16, depthScale);
        const bool allowColorPrimary =
            config.colorContourPrimary &&
            support.anchorCount >= config.colorContourPrimaryMinAnchors;
        ObservationMaterial completedMaterial;
        if (buildColorCompletedMaterial(
                colorBgr,
                colorEdges,
                stableUnionMask,
                displayMaterials[index],
                config,
                allowColorPrimary,
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

std::string formatDistanceIntervalMeters(int minMm, int maxMm);

ObservationMaterial farDistanceMaterialToObservationMaterial(
    const FarDistanceMaterial& farMaterial,
    uint64_t sourceFrameId)
{
    ObservationMaterial material;
    material.sourceFrameId = sourceFrameId;
    material.observationId = sourceFrameId * 1000ULL + 800ULL + static_cast<uint64_t>(farMaterial.rankNearFirst);
    material.roi = farMaterial.roi;
    material.center = farMaterial.center;
    material.pixelCount = farMaterial.pixelCount;
    material.depthMinMm = farMaterial.intervalMinMm;
    material.depthMaxMm = farMaterial.intervalMaxMm;
    material.observedDepthMinMm = farMaterial.observedDepthMinMm;
    material.observedDepthMaxMm = farMaterial.observedDepthMaxMm;
    material.meanDepthMm = farMaterial.medianDepthMm;
    material.groupId = 800 + farMaterial.rankNearFirst;
    material.contourArea = farMaterial.contourArea;
    material.contour = farMaterial.contour;
    return material;
}

std::vector<ObservationMaterial> buildMosaicDisplayMaterials(
    const std::vector<ObservationMaterial>& displayStableMaterials,
    const std::vector<ObservationMaterial>& nearPlaneMaterials,
    const std::vector<FarDistanceMaterial>& farDistanceMaterials,
    uint64_t sourceFrameId)
{
    std::vector<ObservationMaterial> materials = displayStableMaterials;
    materials.reserve(
        displayStableMaterials.size() +
        nearPlaneMaterials.size() +
        farDistanceMaterials.size());

    for (const ObservationMaterial& material : nearPlaneMaterials)
    {
        materials.push_back(material);
    }
    for (const FarDistanceMaterial& farMaterial : farDistanceMaterials)
    {
        materials.push_back(farDistanceMaterialToObservationMaterial(farMaterial, sourceFrameId));
    }

    return materials;
}

cv::Scalar clusterColorForId(int id);

cv::Mat contourToMask(const std::vector<cv::Point>& contour, const cv::Size& frameSize)
{
    cv::Mat mask = cv::Mat::zeros(frameSize, CV_8UC1);
    if (contour.size() >= 3)
    {
        const std::vector<std::vector<cv::Point>> contours{contour};
        cv::drawContours(mask, contours, -1, cv::Scalar(255), cv::FILLED, cv::LINE_8);
    }
    return mask;
}

bool overlapsExistingColorRegion(
    const std::vector<ColorContourRegion>& regions,
    const cv::Mat& candidateMask,
    int candidatePixels,
    const cv::Size& frameSize)
{
    if (candidateMask.empty() || candidatePixels <= 0)
    {
        return false;
    }

    for (const ColorContourRegion& region : regions)
    {
        const cv::Mat existingMask = contourToMask(region.contour, frameSize);
        cv::Mat overlap;
        cv::bitwise_and(candidateMask, existingMask, overlap);
        const int overlapPixels = cv::countNonZero(overlap);
        const int smallerArea = std::max(1, std::min(candidatePixels, region.pixelCount));
        if (100.0 * static_cast<double>(overlapPixels) / static_cast<double>(smallerArea) >= 72.0)
        {
            return true;
        }
    }
    return false;
}

bool appendColorContourRegionFromMask(
    const cv::Mat& sourceMask,
    const cv::Size& frameSize,
    const SegmentationConfig& config,
    std::vector<ColorContourRegion>& regions)
{
    if (sourceMask.empty() || cv::countNonZero(sourceMask) < config.colorSegmentationMinAreaPixels)
    {
        return false;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(sourceMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    if (contours.empty())
    {
        return false;
    }
    const auto contourIt = std::max_element(
        contours.begin(),
        contours.end(),
        [](const std::vector<cv::Point>& lhs, const std::vector<cv::Point>& rhs)
        {
            return std::abs(cv::contourArea(lhs)) < std::abs(cv::contourArea(rhs));
        });
    if (contourIt == contours.end() || contourIt->size() < 3)
    {
        return false;
    }

    std::vector<cv::Point> preciseContour;
    const double epsilon = cv::arcLength(*contourIt, true) * config.contourApproxRatio;
    if (epsilon >= 0.5)
    {
        cv::approxPolyDP(*contourIt, preciseContour, epsilon, true);
    }
    if (preciseContour.size() < 3)
    {
        preciseContour = *contourIt;
    }

    const cv::Rect frameRect(0, 0, frameSize.width, frameSize.height);
    const cv::Rect roi = cv::boundingRect(preciseContour) & frameRect;
    if (roi.empty())
    {
        return false;
    }

    const int frameArea = std::max(1, frameSize.width * frameSize.height);
    const int maxRoiArea = frameArea * config.colorSegmentationMaxRoiAreaPercent / 100;
    const int borderRejectArea = frameArea * config.colorSegmentationBorderRejectAreaPercent / 100;
    if (roi.area() > maxRoiArea)
    {
        return false;
    }

    cv::Mat regionMask = contourToMask(preciseContour, frameSize);
    const int pixelCount = cv::countNonZero(regionMask);
    if (pixelCount < config.colorSegmentationMinAreaPixels)
    {
        return false;
    }

    const bool touchesBorder =
        roi.x <= 1 ||
        roi.y <= 1 ||
        roi.x + roi.width >= frameSize.width - 1 ||
        roi.y + roi.height >= frameSize.height - 1;
    const bool spansMostFrame =
        roi.width >= frameSize.width * 85 / 100 ||
        roi.height >= frameSize.height * 85 / 100;
    if (touchesBorder && spansMostFrame && pixelCount > borderRejectArea)
    {
        return false;
    }
    if (overlapsExistingColorRegion(regions, regionMask, pixelCount, frameSize))
    {
        return false;
    }

    const cv::Moments moments = cv::moments(regionMask, true);
    ColorContourRegion region;
    region.roi = roi;
    region.center = std::abs(moments.m00) > 1e-6
        ? cv::Point(
            static_cast<int>(std::lround(moments.m10 / moments.m00)),
            static_cast<int>(std::lround(moments.m01 / moments.m00)))
        : cv::Point(roi.x + roi.width / 2, roi.y + roi.height / 2);
    region.pixelCount = pixelCount;
    region.contourArea = std::abs(cv::contourArea(preciseContour));
    region.contour = std::move(preciseContour);
    region.contourConfidence = std::clamp(
        region.contourArea / static_cast<double>(std::max(1, frameArea / 20)),
        0.10,
        0.95);
    regions.push_back(std::move(region));
    return true;
}

void appendColorConnectedComponentRegions(
    const cv::Mat& colorBgr,
    const SegmentationConfig& config,
    std::vector<ColorContourRegion>& regions)
{
    cv::Mat filtered = colorBgr;
    if (config.colorSegmentationMeanShiftSpatial > 0 && config.colorSegmentationMeanShiftColor > 0)
    {
        cv::pyrMeanShiftFiltering(
            colorBgr,
            filtered,
            static_cast<double>(config.colorSegmentationMeanShiftSpatial),
            static_cast<double>(config.colorSegmentationMeanShiftColor));
    }

    cv::Mat lab;
    cv::cvtColor(filtered, lab, cv::COLOR_BGR2Lab);
    cv::Mat quantized(lab.size(), CV_32S);
    std::map<int, int> labelCounts;
    const int bins = std::max(2, config.colorSegmentationColorBins);
    for (int y = 0; y < lab.rows; ++y)
    {
        const cv::Vec3b* labRow = lab.ptr<cv::Vec3b>(y);
        int* labelRow = quantized.ptr<int>(y);
        for (int x = 0; x < lab.cols; ++x)
        {
            const int lBin = std::min(bins - 1, labRow[x][0] * bins / 256);
            const int aBin = std::min(bins - 1, labRow[x][1] * bins / 256);
            const int bBin = std::min(bins - 1, labRow[x][2] * bins / 256);
            const int label = (lBin * bins + aBin) * bins + bBin;
            labelRow[x] = label;
            ++labelCounts[label];
        }
    }

    struct MaskCandidate
    {
        int area = 0;
        cv::Mat mask;
    };
    std::vector<MaskCandidate> candidates;
    const cv::Mat kernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(config.colorSegmentationMorphKernelSize, config.colorSegmentationMorphKernelSize));
    for (const auto& [label, count] : labelCounts)
    {
        if (count < config.colorSegmentationMinAreaPixels)
        {
            continue;
        }

        cv::Mat labelMask;
        cv::compare(quantized, cv::Scalar(label), labelMask, cv::CMP_EQ);
        cv::morphologyEx(labelMask, labelMask, cv::MORPH_OPEN, cv::Mat());
        cv::morphologyEx(labelMask, labelMask, cv::MORPH_CLOSE, kernel);

        cv::Mat componentLabels;
        cv::Mat stats;
        cv::Mat centroids;
        const int componentCount = cv::connectedComponentsWithStats(
            labelMask,
            componentLabels,
            stats,
            centroids,
            8,
            CV_32S);
        for (int component = 1; component < componentCount; ++component)
        {
            const int area = stats.at<int>(component, cv::CC_STAT_AREA);
            if (area < config.colorSegmentationMinAreaPixels)
            {
                continue;
            }

            cv::Mat componentMask;
            cv::compare(componentLabels, cv::Scalar(component), componentMask, cv::CMP_EQ);
            candidates.push_back(MaskCandidate{area, componentMask});
        }
    }

    std::sort(
        candidates.begin(),
        candidates.end(),
        [](const MaskCandidate& lhs, const MaskCandidate& rhs)
        {
            return lhs.area > rhs.area;
        });
    for (const MaskCandidate& candidate : candidates)
    {
        appendColorContourRegionFromMask(candidate.mask, colorBgr.size(), config, regions);
    }
}

std::vector<ColorContourRegion> extractColorContourRegions(
    const cv::Mat& colorBgr,
    const SegmentationConfig& config)
{
    std::vector<ColorContourRegion> regions;
    if (!config.qualitySegmentation || !config.colorSegmentation || colorBgr.empty())
    {
        return regions;
    }

    cv::Mat gray;
    cv::cvtColor(colorBgr, gray, cv::COLOR_BGR2GRAY);
    cv::Mat equalized;
    cv::equalizeHist(gray, equalized);
    cv::Mat blurred;
    cv::GaussianBlur(equalized, blurred, cv::Size(3, 3), 0.0);
    cv::Mat edges;
    cv::Canny(blurred, edges, config.colorCannyLow, config.colorCannyHigh);

    const int kernelSize = config.colorSegmentationMorphKernelSize | 1;
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
    cv::morphologyEx(edges, edges, cv::MORPH_CLOSE, kernel);
    cv::dilate(edges, edges, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::sort(
        contours.begin(),
        contours.end(),
        [](const std::vector<cv::Point>& lhs, const std::vector<cv::Point>& rhs)
        {
            return std::abs(cv::contourArea(lhs)) > std::abs(cv::contourArea(rhs));
        });

    for (const std::vector<cv::Point>& contour : contours)
    {
        if (contour.size() < 3)
        {
            continue;
        }

        const double contourArea = std::abs(cv::contourArea(contour));
        if (contourArea < static_cast<double>(config.colorSegmentationMinAreaPixels))
        {
            continue;
        }

        const cv::Mat regionMask = contourToMask(contour, colorBgr.size());
        appendColorContourRegionFromMask(regionMask, colorBgr.size(), config, regions);
    }

    appendColorConnectedComponentRegions(colorBgr, config, regions);
    std::sort(
        regions.begin(),
        regions.end(),
        [](const ColorContourRegion& lhs, const ColorContourRegion& rhs)
        {
            return lhs.pixelCount > rhs.pixelCount;
        });
    if (regions.size() > static_cast<size_t>(config.colorSegmentationMaxRegions))
    {
        regions.resize(static_cast<size_t>(config.colorSegmentationMaxRegions));
    }
    for (size_t index = 0; index < regions.size(); ++index)
    {
        regions[index].id = static_cast<int>(index + 1);
    }
    return regions;
}

std::vector<ColorContourRegion> extractColorContourRegionsInRoi(
    const cv::Mat& colorBgr,
    const SegmentationConfig& config,
    const cv::Rect& refreshRoi)
{
    const cv::Rect frameRect(0, 0, colorBgr.cols, colorBgr.rows);
    const cv::Rect roi = refreshRoi & frameRect;
    if (roi.empty())
    {
        return extractColorContourRegions(colorBgr, config);
    }

    std::vector<ColorContourRegion> regions = extractColorContourRegions(colorBgr(roi).clone(), config);
    for (ColorContourRegion& region : regions)
    {
        region.roi.x += roi.x;
        region.roi.y += roi.y;
        region.center.x += roi.x;
        region.center.y += roi.y;
        for (cv::Point& point : region.contour)
        {
            point.x += roi.x;
            point.y += roi.y;
        }
    }
    return regions;
}

cv::Rect colorContourMotionRefreshRoi(
    const cv::Mat& currentSignature,
    const cv::Mat& previousSignature,
    const cv::Size& frameSize,
    const SegmentationConfig& config,
    int* candidatePixels = nullptr,
    bool* rejectedEmpty = nullptr,
    bool* rejectedLarge = nullptr)
{
    if (candidatePixels)
    {
        *candidatePixels = 0;
    }
    if (rejectedEmpty)
    {
        *rejectedEmpty = false;
    }
    if (rejectedLarge)
    {
        *rejectedLarge = false;
    }
    if (!config.colorContourRefreshMotionRoi ||
        currentSignature.empty() ||
        previousSignature.empty() ||
        currentSignature.size() != previousSignature.size() ||
        frameSize.empty())
    {
        return cv::Rect();
    }

    cv::Mat diff;
    cv::absdiff(currentSignature, previousSignature, diff);
    cv::threshold(diff, diff, 24, 255, cv::THRESH_BINARY);
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(5, 5));
    cv::morphologyEx(diff, diff, cv::MORPH_CLOSE, kernel);
    cv::dilate(diff, diff, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(diff, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    cv::Rect smallRoi;
    for (const std::vector<cv::Point>& contour : contours)
    {
        const cv::Rect contourRoi = cv::boundingRect(contour);
        smallRoi = smallRoi.empty() ? contourRoi : (smallRoi | contourRoi);
    }
    if (smallRoi.empty())
    {
        if (rejectedEmpty)
        {
            *rejectedEmpty = true;
        }
        return cv::Rect();
    }

    const double scaleX = static_cast<double>(frameSize.width) / static_cast<double>(currentSignature.cols);
    const double scaleY = static_cast<double>(frameSize.height) / static_cast<double>(currentSignature.rows);
    cv::Rect roi(
        static_cast<int>(std::floor(smallRoi.x * scaleX)),
        static_cast<int>(std::floor(smallRoi.y * scaleY)),
        static_cast<int>(std::ceil((smallRoi.x + smallRoi.width) * scaleX)) -
            static_cast<int>(std::floor(smallRoi.x * scaleX)),
        static_cast<int>(std::ceil((smallRoi.y + smallRoi.height) * scaleY)) -
            static_cast<int>(std::floor(smallRoi.y * scaleY)));
    roi = expandedRect(roi, std::max(0, config.colorContourRefreshRoiPaddingPixels), frameSize);
    if (candidatePixels)
    {
        *candidatePixels = roi.area();
    }
    const int framePixels = std::max(1, frameSize.width * frameSize.height);
    const int maxRoiPixels = framePixels * std::clamp(config.colorContourRefreshMaxRoiAreaPercent, 1, 100) / 100;
    if (roi.empty())
    {
        if (rejectedEmpty)
        {
            *rejectedEmpty = true;
        }
        return cv::Rect();
    }
    if (roi.area() > maxRoiPixels)
    {
        if (rejectedLarge)
        {
            *rejectedLarge = true;
        }
        return cv::Rect();
    }
    return roi;
}

void sortAndLimitColorContourRegions(
    std::vector<ColorContourRegion>& regions,
    const SegmentationConfig& config)
{
    std::sort(
        regions.begin(),
        regions.end(),
        [](const ColorContourRegion& lhs, const ColorContourRegion& rhs)
        {
            return lhs.pixelCount > rhs.pixelCount;
        });
    if (regions.size() > static_cast<size_t>(config.colorSegmentationMaxRegions))
    {
        regions.resize(static_cast<size_t>(config.colorSegmentationMaxRegions));
    }
    for (size_t index = 0; index < regions.size(); ++index)
    {
        regions[index].id = static_cast<int>(index + 1);
    }
}

bool hasMatchingStereoRegion(
    const ColorContourRegion& target,
    const std::vector<ColorContourRegion>& regions)
{
    for (const ColorContourRegion& region : regions)
    {
        if (region.estimatedDistanceMm <= 0)
        {
            continue;
        }
        const double iou = rectIou(target.roi, region.roi);
        const double centerDistance = std::hypot(
            static_cast<double>(target.center.x - region.center.x),
            static_cast<double>(target.center.y - region.center.y));
        if (iou >= 0.05 || centerDistance <= 120.0)
        {
            return true;
        }
    }
    return false;
}

std::vector<ColorContourRegion> mergeColorContourRoiRefresh(
    const std::vector<ColorContourRegion>& cachedRegions,
    std::vector<ColorContourRegion> refreshedRegions,
    const cv::Rect& refreshRoi,
    const SegmentationConfig& config,
    int* preservedStereoCount = nullptr)
{
    if (preservedStereoCount)
    {
        *preservedStereoCount = 0;
    }
    if (refreshRoi.empty())
    {
        sortAndLimitColorContourRegions(refreshedRegions, config);
        return refreshedRegions;
    }

    std::vector<ColorContourRegion> merged;
    for (const ColorContourRegion& cached : cachedRegions)
    {
        const bool outsideRefreshRoi = (cached.roi & refreshRoi).empty();
        const bool keepOverlappingStereoEvidence =
            !outsideRefreshRoi &&
            cached.estimatedDistanceMm > 0 &&
            !hasMatchingStereoRegion(cached, refreshedRegions);
        if (outsideRefreshRoi || keepOverlappingStereoEvidence)
        {
            merged.push_back(cached);
            if (keepOverlappingStereoEvidence && preservedStereoCount)
            {
                ++(*preservedStereoCount);
            }
        }
    }
    for (ColorContourRegion& refreshed : refreshedRegions)
    {
        merged.push_back(std::move(refreshed));
    }
    sortAndLimitColorContourRegions(merged, config);
    return merged;
}

int countStereoDistanceValidRegions(const std::vector<ColorContourRegion>& regions)
{
    int count = 0;
    for (const ColorContourRegion& region : regions)
    {
        if (region.estimatedDistanceMm > 0)
        {
            ++count;
        }
    }
    return count;
}

void updateRoiStereoRefreshStats(
    ColorContourRefreshStats& stats,
    const std::vector<ColorContourRegion>& regions)
{
    stats.roiRefreshedRegionCount = static_cast<int>(regions.size());
    const int stereoValidCount = countStereoDistanceValidRegions(regions);
    stats.roiStereoReuseCount = stats.stereoReuseCount;
    stats.roiStereoBuiltCount = std::max(0, stereoValidCount - stats.roiStereoReuseCount);
    stats.roiStereoFailedCount = std::max(0, stats.roiRefreshedRegionCount - stereoValidCount);
}

int dropStereoFailedColorContourRegions(std::vector<ColorContourRegion>& regions)
{
    const int before = static_cast<int>(regions.size());
    regions.erase(
        std::remove_if(
            regions.begin(),
            regions.end(),
            [](const ColorContourRegion& region)
            {
                return region.estimatedDistanceMm <= 0;
            }),
        regions.end());
    return before - static_cast<int>(regions.size());
}

int reuseCachedStereoDistances(
    std::vector<ColorContourRegion>& regions,
    const std::vector<ColorContourRegion>& cachedRegions)
{
    int reused = 0;
    for (ColorContourRegion& region : regions)
    {
        const ColorContourRegion* best = nullptr;
        double bestScore = 0.0;
        for (const ColorContourRegion& cached : cachedRegions)
        {
            if (cached.estimatedDistanceMm <= 0)
            {
                continue;
            }
            const double iou = rectIou(region.roi, cached.roi);
            const double centerDistance = std::hypot(
                static_cast<double>(region.center.x - cached.center.x),
                static_cast<double>(region.center.y - cached.center.y));
            const double centerScore = std::max(0.0, 1.0 - centerDistance / 160.0);
            const double score = iou * 3.0 + centerScore;
            if ((iou >= 0.05 || centerDistance <= 120.0) && score > bestScore)
            {
                best = &cached;
                bestScore = score;
            }
        }
        if (best == nullptr)
        {
            continue;
        }
        region.estimatedDistanceMm = best->estimatedDistanceMm;
        region.distanceUncertaintyMm = best->distanceUncertaintyMm;
        region.estimatedWidthMm = best->estimatedWidthMm;
        region.estimatedHeightMm = best->estimatedHeightMm;
        region.medianDisparityPx = best->medianDisparityPx;
        region.matchedStereoPoints = best->matchedStereoPoints;
        region.distanceConfidence = best->distanceConfidence;
        ++reused;
    }
    return reused;
}

struct AsyncColorContourRefreshTask
{
    uint64_t frameId = 0;
    cv::Mat colorBgr;
    cv::Mat motionSignature;
    cv::Rect refreshRoi;
    SegmentationConfig config;
};

struct AsyncColorContourRefreshResult
{
    uint64_t frameId = 0;
    std::vector<ColorContourRegion> regions;
    cv::Mat motionSignature;
    cv::Rect refreshRoi;
    double workerMs = 0.0;
};

class AsyncColorContourRefreshWorker
{
public:
    explicit AsyncColorContourRefreshWorker(bool lowPriority)
        : lowPriority_(lowPriority)
        , thread_([this]() { run(); })
    {
    }

    ~AsyncColorContourRefreshWorker()
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            stop_ = true;
        }
        condition_.notify_one();
        if (thread_.joinable())
        {
            thread_.join();
        }
    }

    bool submitLatest(
        uint64_t frameId,
        const cv::Mat& colorBgr,
        const cv::Mat& motionSignature,
        const cv::Rect& refreshRoi,
        const SegmentationConfig& config)
    {
        AsyncColorContourRefreshTask task;
        task.frameId = frameId;
        task.colorBgr = colorBgr.clone();
        task.motionSignature = motionSignature.clone();
        task.refreshRoi = refreshRoi;
        task.config = config;

        bool droppedPending = false;
        {
            std::lock_guard<std::mutex> lock(mutex_);
            droppedPending = hasPendingTask_;
            pendingTask_ = std::move(task);
            hasPendingTask_ = true;
        }
        condition_.notify_one();
        return droppedPending;
    }

    bool takeResult(AsyncColorContourRefreshResult& result)
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (!hasResult_)
        {
            return false;
        }
        result = std::move(result_);
        result_ = AsyncColorContourRefreshResult{};
        hasResult_ = false;
        return true;
    }

    bool hasPendingWork() const
    {
        std::lock_guard<std::mutex> lock(mutex_);
        return hasPendingTask_ || busy_;
    }

private:
    void run()
    {
#ifdef _WIN32
        if (lowPriority_)
        {
            SetThreadPriority(GetCurrentThread(), THREAD_PRIORITY_BELOW_NORMAL);
        }
#endif
        while (true)
        {
            AsyncColorContourRefreshTask task;
            {
                std::unique_lock<std::mutex> lock(mutex_);
                condition_.wait(lock, [this]() { return stop_ || hasPendingTask_; });
                if (stop_ && !hasPendingTask_)
                {
                    return;
                }
                task = std::move(pendingTask_);
                pendingTask_ = AsyncColorContourRefreshTask{};
                hasPendingTask_ = false;
                busy_ = true;
            }

            const auto start = std::chrono::steady_clock::now();
            AsyncColorContourRefreshResult result;
            result.frameId = task.frameId;
            result.motionSignature = task.motionSignature;
            result.refreshRoi = task.refreshRoi;
            result.regions = extractColorContourRegionsInRoi(
                task.colorBgr,
                task.config,
                task.refreshRoi);
            result.workerMs = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - start).count();

            {
                std::lock_guard<std::mutex> lock(mutex_);
                result_ = std::move(result);
                hasResult_ = true;
                busy_ = false;
            }
        }
    }

    mutable std::mutex mutex_;
    std::condition_variable condition_;
    bool lowPriority_ = false;
    std::thread thread_;
    bool stop_ = false;
    bool busy_ = false;
    bool hasPendingTask_ = false;
    bool hasResult_ = false;
    AsyncColorContourRefreshTask pendingTask_;
    AsyncColorContourRefreshResult result_;
};

cv::Mat makeColorContourMotionSignature(const cv::Mat& colorBgr)
{
    if (colorBgr.empty())
    {
        return cv::Mat();
    }

    cv::Mat gray;
    if (colorBgr.channels() == 1)
    {
        gray = colorBgr;
    }
    else
    {
        cv::cvtColor(colorBgr, gray, cv::COLOR_BGR2GRAY);
    }
    const int targetWidth = 80;
    const int targetHeight = std::max(1, gray.rows * targetWidth / std::max(1, gray.cols));
    cv::Mat resized;
    cv::resize(gray, resized, cv::Size(targetWidth, targetHeight), 0.0, 0.0, cv::INTER_AREA);
    return resized;
}

double colorContourMotionDeltaPercent(const cv::Mat& currentSignature, const cv::Mat& previousSignature)
{
    if (currentSignature.empty() ||
        previousSignature.empty() ||
        currentSignature.size() != previousSignature.size())
    {
        return 0.0;
    }

    cv::Mat diff;
    cv::absdiff(currentSignature, previousSignature, diff);
    cv::threshold(diff, diff, 24, 255, cv::THRESH_BINARY);
    const int changed = cv::countNonZero(diff);
    const int total = std::max(1, diff.rows * diff.cols);
    return 100.0 * static_cast<double>(changed) / static_cast<double>(total);
}

double maskOverlapPercent(const cv::Mat& lhsMask, const cv::Mat& rhsMask, int denominatorPixels)
{
    if (lhsMask.empty() || rhsMask.empty() || lhsMask.size() != rhsMask.size() || denominatorPixels <= 0)
    {
        return 0.0;
    }

    cv::Mat intersection;
    cv::bitwise_and(lhsMask, rhsMask, intersection);
    return 100.0 * static_cast<double>(cv::countNonZero(intersection)) /
        static_cast<double>(denominatorPixels);
}

std::vector<ObservationMaterial> refineDepthMaterialsWithColorContours(
    const std::vector<ObservationMaterial>& depthMaterials,
    const std::vector<ColorContourRegion>& colorRegions,
    const cv::Mat& depth16,
    float depthScale,
    const rs2_intrinsics& intrinsics,
    const SegmentationConfig& config,
    uint64_t sourceFrameId)
{
    if (!config.qualitySegmentation || !config.colorRefineDepthMasks || colorRegions.empty())
    {
        return depthMaterials;
    }

    std::vector<ObservationMaterial> refined = depthMaterials;
    const cv::Size frameSize = depth16.size();
    const cv::Rect frameRect(0, 0, frameSize.width, frameSize.height);
    for (ObservationMaterial& material : refined)
    {
        if (material.contour.size() < 3)
        {
            continue;
        }

        const cv::Mat depthMask = contourToMask(material.contour, frameSize);
        const int depthArea = cv::countNonZero(depthMask);
        if (depthArea <= 0)
        {
            continue;
        }

        const ColorContourRegion* bestRegion = nullptr;
        double bestOverlapPercent = 0.0;
        for (const ColorContourRegion& region : colorRegions)
        {
            if ((material.roi & region.roi).empty())
            {
                continue;
            }
            const cv::Mat colorMask = contourToMask(region.contour, frameSize);
            const double overlapPercent = maskOverlapPercent(depthMask, colorMask, depthArea);
            if (overlapPercent > bestOverlapPercent)
            {
                bestOverlapPercent = overlapPercent;
                bestRegion = &region;
            }
        }

        if (bestRegion == nullptr ||
            bestOverlapPercent < static_cast<double>(config.colorRefineMinOverlapPercent))
        {
            continue;
        }

        const int colorArea = std::max(1, bestRegion->pixelCount);
        const double areaDeltaPercent =
            100.0 * std::abs(colorArea - depthArea) / static_cast<double>(std::max(1, depthArea));
        if (areaDeltaPercent > static_cast<double>(config.colorRefineMaxAreaDeltaPercent))
        {
            continue;
        }

        const cv::Mat colorMask = contourToMask(bestRegion->contour, frameSize);
        const cv::Rect roi = bestRegion->roi & frameRect;
        if (roi.empty())
        {
            continue;
        }

        const LocalComponentMeasurement measurement = measureLocalComponent(
            colorMask(roi),
            roi,
            depth16,
            depthScale,
            intrinsics,
            colorArea);
        if (!measurement.validDepth)
        {
            continue;
        }

        material.sourceFrameId = sourceFrameId;
        material.roi = roi;
        material.center = bestRegion->center;
        material.pixelCount = colorArea;
        material.depthMinMm = measurement.observedDepthMinMm;
        material.depthMaxMm = measurement.observedDepthMaxMm;
        material.meanDepthMm = measurement.meanDepthMm;
        material.observedDepthMinMm = measurement.observedDepthMinMm;
        material.observedDepthMaxMm = measurement.observedDepthMaxMm;
        material.contourArea = bestRegion->contourArea;
        material.contour = bestRegion->contour;
        material.hasPointCloudBounds = measurement.hasPointCloudBounds;
        material.minPointMeters = measurement.minPointMeters;
        material.maxPointMeters = measurement.maxPointMeters;
    }

    return refined;
}

void estimateStereoContourDistances(
    std::vector<ColorContourRegion>& regions,
    const cv::Mat& leftIr,
    const cv::Mat& rightIr,
    const rs2_intrinsics& intrinsics,
    const SegmentationConfig& config)
{
    if (!config.qualitySegmentation ||
        !config.stereoContourDistance ||
        leftIr.empty() ||
        rightIr.empty() ||
        intrinsics.fx <= 0.0f)
    {
        return;
    }

    cv::Mat leftGray = leftIr;
    cv::Mat rightGray = rightIr;
    if (leftGray.channels() != 1)
    {
        cv::cvtColor(leftGray, leftGray, cv::COLOR_BGR2GRAY);
    }
    if (rightGray.channels() != 1)
    {
        cv::cvtColor(rightGray, rightGray, cv::COLOR_BGR2GRAY);
    }
    if (leftGray.size() != rightGray.size())
    {
        cv::resize(rightGray, rightGray, leftGray.size(), 0.0, 0.0, cv::INTER_LINEAR);
    }

    cv::Mat leftEdges;
    cv::Mat rightEdges;
    cv::Canny(leftGray, leftEdges, config.infraredCannyLow, config.infraredCannyHigh);
    cv::Canny(rightGray, rightEdges, config.infraredCannyLow, config.infraredCannyHigh);
    const cv::Rect frameRect(0, 0, leftGray.cols, leftGray.rows);
    const double minDisparityPx = config.stereoContourMinDisparityTenthsPx / 10.0;
    const int maxStereoRoiArea = std::max(
        1,
        frameRect.area() * config.stereoContourMaxRoiAreaPercent / 100);

    int processedRegions = 0;
    for (ColorContourRegion& region : regions)
    {
        cv::Rect templateRoi = region.roi & frameRect;
        if (templateRoi.width < 8 || templateRoi.height < 8)
        {
            continue;
        }
        if (templateRoi.area() > maxStereoRoiArea)
        {
            continue;
        }

        const int margin = config.stereoContourSearchMarginPixels;
        cv::Rect searchRoi(
            std::max(0, templateRoi.x - margin),
            std::max(0, templateRoi.y - config.stereoContourMaxVerticalShiftPixels),
            std::min(frameRect.width, templateRoi.x + templateRoi.width + margin) - std::max(0, templateRoi.x - margin),
            std::min(frameRect.height, templateRoi.y + templateRoi.height + config.stereoContourMaxVerticalShiftPixels) -
                std::max(0, templateRoi.y - config.stereoContourMaxVerticalShiftPixels));
        searchRoi &= frameRect;
        if (searchRoi.width <= templateRoi.width || searchRoi.height <= templateRoi.height)
        {
            continue;
        }

        cv::Mat templ = leftEdges(templateRoi);
        cv::Mat search = rightEdges(searchRoi);
        if (cv::countNonZero(templ) < 12)
        {
            continue;
        }
        if (processedRegions >= config.stereoContourMaxRegionsPerFrame)
        {
            break;
        }
        ++processedRegions;

        cv::Mat response;
        cv::matchTemplate(search, templ, response, cv::TM_CCOEFF_NORMED);
        double maxValue = 0.0;
        cv::Point maxLocation;
        cv::minMaxLoc(response, nullptr, &maxValue, nullptr, &maxLocation);
        if (maxValue < 0.12)
        {
            continue;
        }

        const double matchedRightX = static_cast<double>(searchRoi.x + maxLocation.x);
        const double disparityPx = static_cast<double>(templateRoi.x) - matchedRightX;
        if (disparityPx < minDisparityPx)
        {
            continue;
        }

        const double distanceMm = static_cast<double>(intrinsics.fx) *
            static_cast<double>(config.stereoContourBaselineMm) /
            disparityPx;
        if (!std::isfinite(distanceMm) || distanceMm <= 0.0)
        {
            continue;
        }

        region.medianDisparityPx = disparityPx;
        region.estimatedDistanceMm = static_cast<int>(std::lround(distanceMm));
        region.distanceUncertaintyMm = static_cast<int>(std::lround(
            distanceMm * std::max(0.15, 1.0 - std::clamp(maxValue, 0.0, 0.95))));
        region.estimatedWidthMm = static_cast<int>(std::lround(
            distanceMm * static_cast<double>(region.roi.width) / std::max(1.0f, intrinsics.fx)));
        region.estimatedHeightMm = static_cast<int>(std::lround(
            distanceMm * static_cast<double>(region.roi.height) / std::max(1.0f, intrinsics.fy)));
        region.matchedStereoPoints = cv::countNonZero(templ);
        region.distanceConfidence = std::clamp(maxValue, 0.0, 0.95);
    }
}

FinalSegmentationFrame buildFinalSegmentationFrame(
    const cv::Mat& colorBgr,
    const std::vector<ObservationMaterial>& depthMaterials,
    const std::vector<ColorContourRegion>& colorRegions)
{
    FinalSegmentationFrame frame;
    if (colorBgr.empty())
    {
        return frame;
    }

    frame.idMap = cv::Mat::zeros(colorBgr.size(), CV_16U);
    frame.overlay = colorBgr.clone();
    frame.depthMaterials = depthMaterials;
    frame.colorRegions = colorRegions;
    int nextId = 1;
    for (const ObservationMaterial& material : depthMaterials)
    {
        const cv::Mat mask = contourToMask(material.contour, colorBgr.size());
        frame.idMap.setTo(nextId, mask);
        cv::Mat colorLayer(colorBgr.size(), CV_8UC3, clusterColorForId(nextId));
        colorLayer.copyTo(frame.overlay, mask);
        ++nextId;
    }
    for (const ColorContourRegion& region : colorRegions)
    {
        const cv::Mat mask = contourToMask(region.contour, colorBgr.size());
        cv::Mat unassigned;
        cv::compare(frame.idMap, 0, unassigned, cv::CMP_EQ);
        cv::Mat addMask;
        cv::bitwise_and(mask, unassigned, addMask);
        if (cv::countNonZero(addMask) == 0)
        {
            continue;
        }
        frame.idMap.setTo(nextId, addMask);
        cv::Mat colorLayer(colorBgr.size(), CV_8UC3, clusterColorForId(nextId));
        colorLayer.copyTo(frame.overlay, addMask);
        ++nextId;
    }
    cv::addWeighted(colorBgr, 0.45, frame.overlay, 0.55, 0.0, frame.overlay);
    return frame;
}

bool materialHasD455PrecisionDepth(const ObservationMaterial& material, const SegmentationConfig& config)
{
    return
        (material.meanDepthMm >= config.minDepthMm && material.meanDepthMm <= config.maxDepthMm) ||
        (material.observedDepthMinMm >= config.minDepthMm && material.observedDepthMinMm <= config.maxDepthMm);
}

cv::Mat buildOutOfD455PrecisionColorContourView(
    const cv::Size& frameSize,
    const cv::Mat& depth16,
    float depthScale,
    const std::vector<ColorContourRegion>& colorRegions,
    const SegmentationConfig& config)
{
    cv::Mat view(frameSize, CV_8UC3, cv::Scalar(0, 0, 0));
    if (frameSize.empty() || colorRegions.empty())
    {
        return view;
    }

    cv::putText(
        view,
        "far + no-depth color ownership >= " +
            std::to_string(config.nonPreciseColorOwnershipMinPercent) + "%",
        cv::Point(12, 28),
        cv::FONT_HERSHEY_SIMPLEX,
        0.65,
        cv::Scalar(255, 255, 255),
        2,
        cv::LINE_AA);
    cv::putText(
        view,
        "bright=far depth, pale=no depth 2D owner",
        cv::Point(12, 52),
        cv::FONT_HERSHEY_SIMPLEX,
        0.5,
        cv::Scalar(210, 210, 210),
        1,
        cv::LINE_AA);

    cv::Mat farDepthMask = cv::Mat::zeros(frameSize, CV_8UC1);
    cv::Mat missingDepthMask = cv::Mat::zeros(frameSize, CV_8UC1);
    if (!depth16.empty())
    {
        cv::Mat depthForMask = depth16;
        if (depthForMask.size() != frameSize)
        {
            cv::resize(depthForMask, depthForMask, frameSize, 0.0, 0.0, cv::INTER_NEAREST);
        }
        cv::inRange(
            depthForMask,
            depthUnitsFromMm(config.maxDepthMm + 1, depthScale),
            depthUnitsFromMm(config.farMaxDepthMm, depthScale),
            farDepthMask);
        cv::compare(depthForMask, 0, missingDepthMask, cv::CMP_EQ);
    }

    const int minDisplayPixels = std::max(16, config.colorSegmentationMinAreaPixels / 4);
    for (const ColorContourRegion& region : colorRegions)
    {
        const cv::Mat colorMask = contourToMask(region.contour, frameSize);
        if (colorMask.empty())
        {
            continue;
        }

        const int colorPixels = cv::countNonZero(colorMask);
        cv::Mat farColorMask;
        cv::bitwise_and(colorMask, farDepthMask, farColorMask);
        const int farPixels = cv::countNonZero(farColorMask);
        cv::Mat missingColorMask;
        cv::bitwise_and(colorMask, missingDepthMask, missingColorMask);
        const int missingPixels = cv::countNonZero(missingColorMask);
        cv::Mat nonPreciseColorMask;
        cv::bitwise_or(farColorMask, missingColorMask, nonPreciseColorMask);
        const int nonPrecisePixels = farPixels + missingPixels;
        const int nonPreciseOwnershipPercent = colorPixels <= 0
            ? 0
            : static_cast<int>(std::lround(100.0 * nonPrecisePixels / static_cast<double>(colorPixels)));
        if (nonPreciseOwnershipPercent < config.nonPreciseColorOwnershipMinPercent)
        {
            continue;
        }

        if (nonPrecisePixels < minDisplayPixels)
        {
            continue;
        }

        const cv::Scalar clusterColor = clusterColorForId(region.id);
        cv::Mat farLayer(frameSize, CV_8UC3, clusterColor);
        farLayer.copyTo(view, farColorMask);

        cv::Scalar missingColor(
            0.45 * clusterColor[0] + 80.0,
            0.45 * clusterColor[1] + 80.0,
            0.45 * clusterColor[2] + 80.0);
        cv::Mat missingLayer(frameSize, CV_8UC3, missingColor);
        missingLayer.copyTo(view, missingColorMask);

        std::vector<std::vector<cv::Point>> contours;
        cv::findContours(nonPreciseColorMask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
        if (!contours.empty())
        {
            cv::drawContours(view, contours, -1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        }
    }

    return view;
}

bool isLikelyBackgroundForMosaic(
    const ObservationMaterial& material,
    const cv::Size& frameSize,
    const SegmentationConfig& config)
{
    const int frameArea = std::max(1, frameSize.width * frameSize.height);
    const int materialArea = std::max(
        material.pixelCount,
        static_cast<int>(std::lround(std::abs(material.contourArea))));
    const double areaPercent = 100.0 * static_cast<double>(materialArea) / static_cast<double>(frameArea);
    const bool nearForeground =
        (material.observedDepthMinMm > 0 && material.observedDepthMinMm <= config.foregroundKeepDepthMm) ||
        (material.meanDepthMm > 0 && material.meanDepthMm <= config.foregroundKeepDepthMm);
    if (nearForeground && areaPercent <= static_cast<double>(config.foregroundMaxAreaPercent))
    {
        return false;
    }

    if (areaPercent > static_cast<double>(config.mosaicMaxMaterialAreaPercent))
    {
        return true;
    }

    const cv::Rect frameRect(0, 0, frameSize.width, frameSize.height);
    const cv::Rect roi = material.roi.empty()
        ? (material.contour.size() >= 3 ? cv::boundingRect(material.contour) & frameRect : cv::Rect())
        : (material.roi & frameRect);
    if (roi.empty())
    {
        return false;
    }

    const bool touchesLeft = roi.x <= 1;
    const bool touchesTop = roi.y <= 1;
    const bool touchesRight = roi.x + roi.width >= frameSize.width - 1;
    const bool touchesBottom = roi.y + roi.height >= frameSize.height - 1;
    const bool touchesBorder = touchesLeft || touchesTop || touchesRight || touchesBottom;
    if (touchesBorder && areaPercent > static_cast<double>(config.mosaicBorderRejectAreaPercent))
    {
        return true;
    }

    const bool spansWide = roi.width >= frameSize.width * 80 / 100;
    const bool spansTall = roi.height >= frameSize.height * 80 / 100;
    return (spansWide || spansTall) &&
        areaPercent > static_cast<double>(config.mosaicBorderRejectAreaPercent);
}

std::vector<ObservationMaterial> filterMosaicForegroundMaterials(
    const std::vector<ObservationMaterial>& materials,
    const cv::Size& frameSize,
    const SegmentationConfig& config)
{
    if (!config.mosaicForegroundGate)
    {
        return materials;
    }

    std::vector<ObservationMaterial> filtered;
    filtered.reserve(materials.size());
    for (const ObservationMaterial& material : materials)
    {
        if (!isLikelyBackgroundForMosaic(material, frameSize, config))
        {
            filtered.push_back(material);
        }
    }
    return filtered;
}

void drawNearPlaneOverlay(
    cv::Mat& view,
    const std::vector<ObservationMaterial>& nearPlaneMaterials)
{
    if (view.empty() || nearPlaneMaterials.empty())
    {
        return;
    }

    for (size_t index = 0; index < nearPlaneMaterials.size(); ++index)
    {
        const ObservationMaterial& material = nearPlaneMaterials[index];
        if (material.contour.size() < 3)
        {
            continue;
        }

        const cv::Scalar color(0, 180, 255);
        const std::vector<std::vector<cv::Point>> contours{material.contour};
        cv::drawContours(view, contours, -1, color, 2, cv::LINE_AA);
        const std::string label =
            "P" + std::to_string(index + 1) +
            " near plane " + formatDistanceIntervalMeters(material.depthMinMm, material.depthMaxMm);

        int baseline = 0;
        const double fontScale = 0.42;
        const int thickness = 1;
        const cv::Size textSize = cv::getTextSize(
            label,
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            thickness,
            &baseline);
        cv::Point labelOrigin(material.center.x + 8, material.center.y - 8);
        labelOrigin.x = std::clamp(labelOrigin.x, 2, std::max(2, view.cols - textSize.width - 2));
        labelOrigin.y = std::clamp(labelOrigin.y, textSize.height + 52, std::max(textSize.height + 52, view.rows - 4));
        drawOutlinedText(view, label, labelOrigin, fontScale, color);
    }
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
    const std::vector<ObservationMaterial>& stableMaterials,
    const cv::Mat& stableMask)
{
    cv::Mat mosaic = cv::Mat::zeros(colorBgr.size(), colorBgr.type());
    if (!stableMask.empty() && stableMask.size() == colorBgr.size())
    {
        colorBgr.copyTo(mosaic, stableMask);
    }

    const cv::Rect frameRect(0, 0, colorBgr.cols, colorBgr.rows);
    const cv::Mat edgeKernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
    for (size_t index = 0; index < stableMaterials.size(); ++index)
    {
        const ObservationMaterial& material = stableMaterials[index];
        if (material.contour.size() < 3)
        {
            continue;
        }

        const cv::Rect roi = cv::boundingRect(material.contour) & frameRect;
        if (roi.empty())
        {
            continue;
        }

        std::vector<std::vector<cv::Point>> localContours(1);
        localContours[0].reserve(material.contour.size());
        for (const cv::Point& point : material.contour)
        {
            localContours[0].emplace_back(point.x - roi.x, point.y - roi.y);
        }

        cv::Mat materialMask = cv::Mat::zeros(roi.size(), CV_8UC1);
        cv::drawContours(materialMask, localContours, -1, cv::Scalar(255), cv::FILLED, cv::LINE_8);

        cv::Mat erodedMask;
        cv::erode(materialMask, erodedMask, edgeKernel, cv::Point(-1, -1), 1, cv::BORDER_CONSTANT, 0);

        cv::Mat materialEdgeMask;
        cv::subtract(materialMask, erodedMask, materialEdgeMask);
        mosaic(roi).setTo(palette(index), materialEdgeMask);
    }
    return mosaic;
}

cv::Mat buildOutsideStableContourColorImage(
    const cv::Mat& colorBgr,
    const cv::Mat& stableMask)
{
    cv::Mat outsideColor = cv::Mat::zeros(colorBgr.size(), colorBgr.type());
    if (stableMask.empty() || stableMask.size() != colorBgr.size())
    {
        return colorBgr.clone();
    }

    cv::Mat outsideMask;
    cv::compare(stableMask, 0, outsideMask, cv::CMP_EQ);
    colorBgr.copyTo(outsideColor, outsideMask);
    return outsideColor;
}

std::string defaultClusterMapExportBasePath();

std::string defaultFinalSegmentationExportBasePath();

std::filesystem::path resolveProjectOutputPath(
    const std::filesystem::path& requestedPath,
    const std::string& defaultExtension);

const char* clusterSpatialModeName(ClusterSpatialMode mode)
{
    switch (mode)
    {
    case ClusterSpatialMode::PreciseDepth3D:
        return "PreciseDepth3D";
    case ClusterSpatialMode::ApproxStereoContour:
        return "ApproxStereoContour";
    case ClusterSpatialMode::ImageOnlyContour:
        return "ImageOnlyContour";
    case ClusterSpatialMode::BackgroundPlane:
        return "BackgroundPlane";
    case ClusterSpatialMode::FarBackground:
        return "FarBackground";
    case ClusterSpatialMode::DepthHoleCandidate:
        return "DepthHoleCandidate";
    case ClusterSpatialMode::Unknown:
    default:
        return "Unknown";
    }
}

cv::Scalar clusterColorForId(int id)
{
    if (id <= 0)
    {
        return cv::Scalar(0, 0, 0);
    }

    const int r = (37 * id + 67) % 206 + 50;
    const int g = (83 * id + 29) % 206 + 50;
    const int b = (131 * id + 113) % 206 + 50;
    return cv::Scalar(b, g, r);
}

cv::Point2f centerFromMask(const cv::Mat& mask, const cv::Rect& bbox)
{
    const cv::Moments moments = cv::moments(mask, true);
    if (std::abs(moments.m00) > 1e-6)
    {
        return cv::Point2f(
            static_cast<float>(moments.m10 / moments.m00),
            static_cast<float>(moments.m01 / moments.m00));
    }

    return cv::Point2f(
        static_cast<float>(bbox.x + bbox.width * 0.5),
        static_cast<float>(bbox.y + bbox.height * 0.5));
}

bool appendClusterFromMask(
    ClusterMapFrame& frame,
    const cv::Mat& inputMask,
    ClusterSpatialMode mode,
    const std::string& source,
    int depthMinMm,
    int depthMeanMm,
    int depthMaxMm,
    double confidence,
    int& nextClusterId)
{
    if (inputMask.empty() || frame.clusterIdMap.empty() || inputMask.size() != frame.clusterIdMap.size())
    {
        return false;
    }

    cv::Mat mask;
    if (inputMask.type() == CV_8UC1)
    {
        mask = inputMask.clone();
    }
    else
    {
        cv::compare(inputMask, 0, mask, cv::CMP_NE);
    }

    cv::Mat unassigned;
    cv::compare(frame.clusterIdMap, 0, unassigned, cv::CMP_EQ);
    cv::bitwise_and(mask, unassigned, mask);
    const int pixelCount = cv::countNonZero(mask);
    if (pixelCount <= 0)
    {
        return false;
    }

    const int id = nextClusterId++;
    frame.clusterIdMap.setTo(id, mask);

    std::vector<cv::Point> nonZeroPoints;
    cv::findNonZero(mask, nonZeroPoints);
    const cv::Rect bbox = nonZeroPoints.empty()
        ? cv::Rect()
        : cv::boundingRect(nonZeroPoints);

    ClusterInfo info;
    info.id = id;
    info.mode = mode;
    info.bbox = bbox;
    info.center = centerFromMask(mask, bbox);
    info.pixelCount = pixelCount;
    info.depthMinMm = depthMinMm;
    info.depthMeanMm = depthMeanMm;
    info.depthMaxMm = depthMaxMm;
    info.confidence = confidence;
    info.source = source;
    frame.clusters.push_back(info);
    return true;
}

bool appendClusterFromContour(
    ClusterMapFrame& frame,
    const std::vector<cv::Point>& contour,
    ClusterSpatialMode mode,
    const std::string& source,
    int depthMinMm,
    int depthMeanMm,
    int depthMaxMm,
    double confidence,
    int& nextClusterId)
{
    if (contour.empty() || frame.clusterIdMap.empty())
    {
        return false;
    }

    cv::Mat mask = cv::Mat::zeros(frame.clusterIdMap.size(), CV_8UC1);
    const std::vector<std::vector<cv::Point>> contours{contour};
    cv::drawContours(mask, contours, -1, cv::Scalar(255), cv::FILLED, cv::LINE_8);
    return appendClusterFromMask(
        frame,
        mask,
        mode,
        source,
        depthMinMm,
        depthMeanMm,
        depthMaxMm,
        confidence,
        nextClusterId);
}

void appendObservationMaterialClusters(
    ClusterMapFrame& frame,
    const std::vector<ObservationMaterial>& materials,
    ClusterSpatialMode mode,
    const std::string& source,
    double confidence,
    int& nextClusterId)
{
    for (const ObservationMaterial& material : materials)
    {
        appendClusterFromContour(
            frame,
            material.contour,
            mode,
            source,
            material.depthMinMm,
            material.meanDepthMm,
            material.depthMaxMm,
            confidence,
            nextClusterId);
    }
}

void appendFarDistanceClusters(
    ClusterMapFrame& frame,
    const std::vector<FarDistanceMaterial>& materials,
    int& nextClusterId)
{
    for (const FarDistanceMaterial& material : materials)
    {
        appendClusterFromContour(
            frame,
            material.contour,
            ClusterSpatialMode::ApproxStereoContour,
            "far_depth_interval",
            material.observedDepthMinMm,
            material.medianDepthMm,
            material.observedDepthMaxMm,
            0.45,
            nextClusterId);
    }
}

void appendVisualContourClusters(
    ClusterMapFrame& frame,
    const cv::Mat& visualGray,
    const SegmentationConfig& config,
    int& nextClusterId)
{
    if (visualGray.empty() || frame.clusterIdMap.empty())
    {
        return;
    }

    cv::Mat gray;
    if (visualGray.channels() == 1)
    {
        gray = visualGray;
    }
    else
    {
        cv::cvtColor(visualGray, gray, cv::COLOR_BGR2GRAY);
    }

    cv::Mat blurred;
    cv::GaussianBlur(gray, blurred, cv::Size(3, 3), 0.0);
    cv::Mat edges;
    cv::Canny(blurred, edges, 60, 150);
    const int kernelSize = config.clusterMapVisualMorphKernelSize | 1;
    const cv::Mat kernel = cv::getStructuringElement(cv::MORPH_ELLIPSE, cv::Size(kernelSize, kernelSize));
    cv::morphologyEx(edges, edges, cv::MORPH_CLOSE, kernel);
    cv::dilate(edges, edges, kernel);

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(edges, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);
    std::sort(
        contours.begin(),
        contours.end(),
        [](const std::vector<cv::Point>& lhs, const std::vector<cv::Point>& rhs)
        {
            return std::abs(cv::contourArea(lhs)) > std::abs(cv::contourArea(rhs));
        });

    int accepted = 0;
    for (const std::vector<cv::Point>& contour : contours)
    {
        if (accepted >= config.clusterMapMaxVisualRegions)
        {
            break;
        }
        if (std::abs(cv::contourArea(contour)) < config.clusterMapVisualMinAreaPixels)
        {
            continue;
        }

        if (appendClusterFromContour(
                frame,
                contour,
                ClusterSpatialMode::ImageOnlyContour,
                "visual_contour_no_depth_requirement",
                0,
                0,
                0,
                0.35,
                nextClusterId))
        {
            ++accepted;
        }
    }
}

void appendIndoorPlaneClusters(
    ClusterMapFrame& frame,
    const IndoorPlaneAnalysis& indoorPlaneAnalysis,
    int& nextClusterId)
{
    appendClusterFromMask(
        frame,
        indoorPlaneAnalysis.supportMask,
        ClusterSpatialMode::BackgroundPlane,
        "indoor_support_plane",
        0,
        0,
        0,
        0.55,
        nextClusterId);
    appendClusterFromMask(
        frame,
        indoorPlaneAnalysis.wallMask,
        ClusterSpatialMode::BackgroundPlane,
        "indoor_wall_plane",
        0,
        0,
        0,
        0.50,
        nextClusterId);
    appendClusterFromMask(
        frame,
        indoorPlaneAnalysis.ceilingMask,
        ClusterSpatialMode::BackgroundPlane,
        "indoor_ceiling_plane",
        0,
        0,
        0,
        0.50,
        nextClusterId);
}

ClusterMapFrame buildFullFrameClusterMap(
    const cv::Size& frameSize,
    const std::vector<ObservationMaterial>& stableMaterials,
    const std::vector<FarDistanceMaterial>& farDistanceMaterials,
    const std::vector<ObservationMaterial>& nearPlaneMaterials,
    const IndoorPlaneAnalysis& indoorPlaneAnalysis,
    const cv::Mat& visualGray,
    const SegmentationConfig& config)
{
    ClusterMapFrame frame;
    frame.clusterIdMap = cv::Mat::zeros(frameSize, CV_32S);
    int nextClusterId = 1;

    appendObservationMaterialClusters(
        frame,
        stableMaterials,
        ClusterSpatialMode::PreciseDepth3D,
        "stable_depth_tracker",
        0.90,
        nextClusterId);
    appendFarDistanceClusters(frame, farDistanceMaterials, nextClusterId);
    appendVisualContourClusters(frame, visualGray, config, nextClusterId);
    appendObservationMaterialClusters(
        frame,
        nearPlaneMaterials,
        ClusterSpatialMode::BackgroundPlane,
        "near_horizontal_plane_display",
        0.50,
        nextClusterId);
    appendIndoorPlaneClusters(frame, indoorPlaneAnalysis, nextClusterId);

    cv::Mat unassigned;
    cv::compare(frame.clusterIdMap, 0, unassigned, cv::CMP_EQ);
    appendClusterFromMask(
        frame,
        unassigned,
        ClusterSpatialMode::FarBackground,
        "unassigned_far_background_remainder",
        0,
        0,
        0,
        0.20,
        nextClusterId);

    const int framePixels = std::max(1, frameSize.width * frameSize.height);
    int assignedPixels = 0;
    int unknownPixels = 0;
    for (const ClusterInfo& cluster : frame.clusters)
    {
        assignedPixels += cluster.pixelCount;
        if (cluster.mode == ClusterSpatialMode::Unknown)
        {
            unknownPixels += cluster.pixelCount;
        }
    }
    const int clusteredPixels = std::max(0, assignedPixels - unknownPixels);
    frame.assignmentCoveragePercent =
        100.0 * static_cast<double>(assignedPixels) / static_cast<double>(framePixels);
    frame.clusterCoveragePercent =
        100.0 * static_cast<double>(clusteredPixels) / static_cast<double>(framePixels);
    frame.unknownPercent =
        100.0 * static_cast<double>(unknownPixels) / static_cast<double>(framePixels);
    return frame;
}

cv::Mat colorizeClusterIdMap(const cv::Mat& clusterIdMap)
{
    cv::Mat color(clusterIdMap.size(), CV_8UC3, cv::Scalar(0, 0, 0));
    for (int y = 0; y < clusterIdMap.rows; ++y)
    {
        const int* idRow = clusterIdMap.ptr<int>(y);
        cv::Vec3b* colorRow = color.ptr<cv::Vec3b>(y);
        for (int x = 0; x < clusterIdMap.cols; ++x)
        {
            const cv::Scalar scalar = clusterColorForId(idRow[x]);
            colorRow[x] = cv::Vec3b(
                static_cast<uchar>(scalar[0]),
                static_cast<uchar>(scalar[1]),
                static_cast<uchar>(scalar[2]));
        }
    }
    return color;
}

std::filesystem::path clusterMapExportBasePath(const SegmentationConfig& config)
{
    std::filesystem::path requested = config.clusterMapExportPath.empty()
        ? std::filesystem::path(defaultClusterMapExportBasePath())
        : std::filesystem::path(config.clusterMapExportPath);
    std::filesystem::path output = resolveProjectOutputPath(requested, "");
    const std::string extension = output.extension().string();
    if (extension == ".json" || extension == ".png" || extension == ".bin" || extension == ".csv")
    {
        output.replace_extension();
    }
    return output;
}

std::string frameExportSuffix(uint64_t frameId)
{
    std::ostringstream stream;
    stream << "_frame_" << std::setw(6) << std::setfill('0') << frameId;
    return stream.str();
}

std::filesystem::path appendFrameExportSuffix(const std::filesystem::path& base, uint64_t frameId)
{
    return base.parent_path() / (base.filename().string() + frameExportSuffix(frameId));
}

void writeClusterMapExport(
    const ClusterMapFrame& frame,
    const cv::Mat& colorBgr,
    const SegmentationConfig& config,
    int64_t exportFrameId = -1)
{
    if (frame.clusterIdMap.empty())
    {
        return;
    }

    std::filesystem::path base = clusterMapExportBasePath(config);
    if (exportFrameId >= 0)
    {
        base = appendFrameExportSuffix(base, static_cast<uint64_t>(exportFrameId));
    }
    if (base.has_parent_path())
    {
        std::filesystem::create_directories(base.parent_path());
    }

    const std::filesystem::path idPath =
        base.parent_path() / (base.filename().string() + "_ids.png");
    const std::filesystem::path overlayPath =
        base.parent_path() / (base.filename().string() + "_overlay.png");
    const std::filesystem::path metadataPath =
        base.parent_path() / (base.filename().string() + "_metadata.json");

    cv::Mat ids16;
    frame.clusterIdMap.convertTo(ids16, CV_16U);
    cv::imwrite(idPath.string(), ids16);

    if (!colorBgr.empty() && colorBgr.size() == frame.clusterIdMap.size())
    {
        cv::Mat overlay;
        cv::addWeighted(colorBgr, 0.55, colorizeClusterIdMap(frame.clusterIdMap), 0.45, 0.0, overlay);
        cv::imwrite(overlayPath.string(), overlay);
    }

    std::ofstream metadata(metadataPath, std::ios::out | std::ios::trunc);
    if (!metadata.is_open())
    {
        throw std::runtime_error("Failed to open cluster map metadata file: " + metadataPath.string());
    }

    metadata << "{\n";
    metadata << "  \"image_size\": [" << frame.clusterIdMap.cols << ", " << frame.clusterIdMap.rows << "],\n";
    if (exportFrameId >= 0)
    {
        metadata << "  \"frame_id\": " << exportFrameId << ",\n";
    }
    metadata << "  \"assignment_coverage_percent\": "
        << std::fixed << std::setprecision(3) << frame.assignmentCoveragePercent << ",\n";
    metadata << "  \"cluster_coverage_percent\": "
        << std::fixed << std::setprecision(3) << frame.clusterCoveragePercent << ",\n";
    metadata << "  \"unknown_percent\": "
        << std::fixed << std::setprecision(3) << frame.unknownPercent << ",\n";
    metadata << "  \"clusters\": [\n";
    for (size_t index = 0; index < frame.clusters.size(); ++index)
    {
        const ClusterInfo& cluster = frame.clusters[index];
        metadata << "    {\n";
        metadata << "      \"id\": " << cluster.id << ",\n";
        metadata << "      \"mode\": \"" << clusterSpatialModeName(cluster.mode) << "\",\n";
        metadata << "      \"source\": \"" << cluster.source << "\",\n";
        metadata << "      \"pixel_count\": " << cluster.pixelCount << ",\n";
        metadata << "      \"bbox_2d\": ["
            << cluster.bbox.x << ", " << cluster.bbox.y << ", "
            << cluster.bbox.width << ", " << cluster.bbox.height << "],\n";
        metadata << "      \"center_2d\": ["
            << std::setprecision(3) << cluster.center.x << ", "
            << std::setprecision(3) << cluster.center.y << "],\n";
        metadata << "      \"depth_min_mm\": " << cluster.depthMinMm << ",\n";
        metadata << "      \"depth_mean_mm\": " << cluster.depthMeanMm << ",\n";
        metadata << "      \"depth_max_mm\": " << cluster.depthMaxMm << ",\n";
        metadata << "      \"confidence\": " << std::setprecision(3) << cluster.confidence << "\n";
        metadata << "    }" << (index + 1 == frame.clusters.size() ? "\n" : ",\n");
    }
    metadata << "  ]\n";
    metadata << "}\n";
    metadata.close();

    std::cout << "Cluster map export saved: "
        << idPath.string() << " ; "
        << metadataPath.string() << '\n';
}

std::filesystem::path finalSegmentationExportBasePath(const SegmentationConfig& config)
{
    std::filesystem::path requested = config.finalSegmentationExportPath.empty()
        ? std::filesystem::path(defaultFinalSegmentationExportBasePath())
        : std::filesystem::path(config.finalSegmentationExportPath);
    std::filesystem::path output = resolveProjectOutputPath(requested, "");
    const std::string extension = output.extension().string();
    if (extension == ".json" || extension == ".png" || extension == ".bin" || extension == ".csv")
    {
        output.replace_extension();
    }
    return output;
}

void writeFinalSegmentationExport(
    const FinalSegmentationFrame& frame,
    const SegmentationConfig& config,
    int64_t exportFrameId = -1)
{
    if (frame.idMap.empty())
    {
        return;
    }

    std::filesystem::path base = finalSegmentationExportBasePath(config);
    if (exportFrameId >= 0)
    {
        base = appendFrameExportSuffix(base, static_cast<uint64_t>(exportFrameId));
    }
    if (base.has_parent_path())
    {
        std::filesystem::create_directories(base.parent_path());
    }

    const std::filesystem::path idPath =
        base.parent_path() / (base.filename().string() + "_ids.png");
    const std::filesystem::path overlayPath =
        base.parent_path() / (base.filename().string() + "_overlay.png");
    const std::filesystem::path metadataPath =
        base.parent_path() / (base.filename().string() + "_metadata.json");

    cv::imwrite(idPath.string(), frame.idMap);
    if (!frame.overlay.empty())
    {
        cv::imwrite(overlayPath.string(), frame.overlay);
    }

    std::ofstream metadata(metadataPath, std::ios::out | std::ios::trunc);
    if (!metadata.is_open())
    {
        throw std::runtime_error("Failed to open final segmentation metadata file: " + metadataPath.string());
    }

    cv::Mat assignedMask;
    cv::compare(frame.idMap, 0, assignedMask, cv::CMP_GT);
    const int assignedPixelCount = cv::countNonZero(assignedMask);
    int stereoDistanceValidCount = 0;
    for (const ColorContourRegion& region : frame.colorRegions)
    {
        if (region.estimatedDistanceMm > 0)
        {
            ++stereoDistanceValidCount;
        }
    }

    metadata << "{\n";
    metadata << "  \"image_size\": [" << frame.idMap.cols << ", " << frame.idMap.rows << "],\n";
    if (exportFrameId >= 0)
    {
        metadata << "  \"frame_id\": " << exportFrameId << ",\n";
    }
    metadata << "  \"assigned_pixel_count\": " << assignedPixelCount << ",\n";
    metadata << "  \"depth_material_count\": " << frame.depthMaterials.size() << ",\n";
    metadata << "  \"color_region_count\": " << frame.colorRegions.size() << ",\n";
    metadata << "  \"stereo_distance_valid_count\": " << stereoDistanceValidCount << ",\n";
    metadata << "  \"depth_materials\": [\n";
    for (size_t index = 0; index < frame.depthMaterials.size(); ++index)
    {
        const ObservationMaterial& material = frame.depthMaterials[index];
        const int sizeXmm = material.hasPointCloudBounds
            ? static_cast<int>(std::lround((material.maxPointMeters.x - material.minPointMeters.x) * 1000.0f))
            : 0;
        const int sizeYmm = material.hasPointCloudBounds
            ? static_cast<int>(std::lround((material.maxPointMeters.y - material.minPointMeters.y) * 1000.0f))
            : 0;
        const int sizeZmm = material.hasPointCloudBounds
            ? static_cast<int>(std::lround((material.maxPointMeters.z - material.minPointMeters.z) * 1000.0f))
            : 0;
        metadata << "    {\"id\": " << (index + 1)
            << ", \"pixel_count\": " << material.pixelCount
            << ", \"bbox_2d\": [" << material.roi.x << ", " << material.roi.y << ", "
            << material.roi.width << ", " << material.roi.height << "]"
            << ", \"center_2d\": [" << material.center.x << ", " << material.center.y << "]"
            << ", \"depth_mean_mm\": " << material.meanDepthMm
            << ", \"depth_min_mm\": " << material.observedDepthMinMm
            << ", \"depth_max_mm\": " << material.observedDepthMaxMm
            << ", \"size_3d_mm\": [" << sizeXmm << ", " << sizeYmm << ", " << sizeZmm << "]"
            << "}" << (index + 1 == frame.depthMaterials.size() ? "\n" : ",\n");
    }
    metadata << "  ],\n";
    metadata << "  \"color_regions\": [\n";
    for (size_t index = 0; index < frame.colorRegions.size(); ++index)
    {
        const ColorContourRegion& region = frame.colorRegions[index];
        metadata << "    {\"id\": " << region.id
            << ", \"pixel_count\": " << region.pixelCount
            << ", \"bbox_2d\": [" << region.roi.x << ", " << region.roi.y << ", "
            << region.roi.width << ", " << region.roi.height << "]"
            << ", \"center_2d\": [" << region.center.x << ", " << region.center.y << "]"
            << ", \"estimated_distance_mm\": " << region.estimatedDistanceMm
            << ", \"distance_uncertainty_mm\": " << region.distanceUncertaintyMm
            << ", \"estimated_size_mm\": [" << region.estimatedWidthMm << ", " << region.estimatedHeightMm << "]"
            << ", \"median_disparity_px\": " << std::fixed << std::setprecision(3) << region.medianDisparityPx
            << ", \"matched_stereo_points\": " << region.matchedStereoPoints
            << ", \"contour_confidence\": " << std::setprecision(3) << region.contourConfidence
            << ", \"distance_confidence\": " << std::setprecision(3) << region.distanceConfidence
            << "}" << (index + 1 == frame.colorRegions.size() ? "\n" : ",\n");
    }
    metadata << "  ]\n";
    metadata << "}\n";
    metadata.close();

    std::cout << "Final segmentation export saved: "
        << idPath.string() << " ; "
        << metadataPath.string() << '\n';
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

std::string defaultImuGravityCheckPath()
{
    const std::filesystem::path path =
        projectRootForOutput() /
        "recordings" /
        ("imu_gravity_check_" + timestampForFilename() + ".csv");
    return path.string();
}

std::string defaultProfileCsvPath()
{
    const std::filesystem::path path =
        projectRootForOutput() /
        "recordings" /
        ("profile_" + timestampForFilename() + ".csv");
    return path.string();
}

std::string defaultClusterMapExportBasePath()
{
    const std::filesystem::path path =
        projectRootForOutput() /
        "recordings" /
        ("cluster_map_" + timestampForFilename());
    return path.string();
}

std::string defaultFinalSegmentationExportBasePath()
{
    const std::filesystem::path path =
        projectRootForOutput() /
        "recordings" /
        ("final_segmentation_" + timestampForFilename());
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

struct Vec3Stats
{
    int count = 0;
    cv::Vec3d sum{0.0, 0.0, 0.0};
    cv::Vec3d sumSquares{0.0, 0.0, 0.0};

    void add(const cv::Vec3d& sample)
    {
        ++count;
        for (int axis = 0; axis < 3; ++axis)
        {
            sum[axis] += sample[axis];
            sumSquares[axis] += sample[axis] * sample[axis];
        }
    }

    cv::Vec3d mean() const
    {
        if (count == 0)
        {
            return {};
        }

        return cv::Vec3d(
            sum[0] / static_cast<double>(count),
            sum[1] / static_cast<double>(count),
            sum[2] / static_cast<double>(count));
    }

    cv::Vec3d stddev() const
    {
        if (count <= 1)
        {
            return {};
        }

        const cv::Vec3d average = mean();
        cv::Vec3d result;
        for (int axis = 0; axis < 3; ++axis)
        {
            const double variance =
                sumSquares[axis] / static_cast<double>(count) -
                average[axis] * average[axis];
            result[axis] = std::sqrt(std::max(0.0, variance));
        }
        return result;
    }

    double rmsMagnitude() const
    {
        if (count == 0)
        {
            return 0.0;
        }

        return std::sqrt(
            (sumSquares[0] + sumSquares[1] + sumSquares[2]) /
            static_cast<double>(count));
    }
};

double vec3Norm(const cv::Vec3d& value)
{
    return std::sqrt(value[0] * value[0] + value[1] * value[1] + value[2] * value[2]);
}

double maxVec3Component(const cv::Vec3d& value)
{
    return std::max(value[0], std::max(value[1], value[2]));
}

std::string signedAxisName(int axisIndex, double value)
{
    static constexpr std::array<char, 3> axisNames{'X', 'Y', 'Z'};
    if (axisIndex < 0 || axisIndex >= static_cast<int>(axisNames.size()))
    {
        return "?";
    }

    std::string result;
    result += value >= 0.0 ? '+' : '-';
    result += axisNames[axisIndex];
    return result;
}

void writeImuGravityCheckCsv(
    const std::filesystem::path& output,
    const ImuGravityCheckConfig& config,
    const Vec3Stats& accelStats,
    const Vec3Stats& gyroStats,
    const std::string& bestAxis,
    double accelNorm,
    double gravityNormErrorPercent,
    double zAxisAngleDeg,
    double zDominancePercent,
    double accelStdMax,
    double gyroRms,
    bool normOk,
    bool zAngleOk,
    bool zDominanceOk,
    bool accelStableOk,
    bool gyroStableOk,
    bool zAxisCanBeGravity)
{
    if (output.has_parent_path())
    {
        std::filesystem::create_directories(output.parent_path());
    }

    std::ofstream stream(output, std::ios::out | std::ios::trunc);
    if (!stream.is_open())
    {
        throw std::runtime_error("Failed to open IMU gravity check CSV: " + output.string());
    }

    const cv::Vec3d accelMean = accelStats.mean();
    const cv::Vec3d accelStd = accelStats.stddev();
    const cv::Vec3d gyroMean = gyroStats.mean();
    const cv::Vec3d gyroStd = gyroStats.stddev();

    stream
        << "sample_frames,warmup_frames,accel_samples,gyro_samples,"
        << "accel_mean_x_mps2,accel_mean_y_mps2,accel_mean_z_mps2,"
        << "accel_std_x_mps2,accel_std_y_mps2,accel_std_z_mps2,"
        << "accel_norm_mps2,gravity_norm_error_percent,best_gravity_axis,"
        << "z_axis_angle_deg,z_axis_dominance_percent,"
        << "gyro_mean_x_radps,gyro_mean_y_radps,gyro_mean_z_radps,"
        << "gyro_std_x_radps,gyro_std_y_radps,gyro_std_z_radps,gyro_rms_radps,"
        << "norm_ok,z_angle_ok,z_dominance_ok,accel_stable_ok,gyro_stable_ok,"
        << "z_axis_can_be_gravity\n";
    stream
        << config.sampleFrames << ','
        << config.warmupFrames << ','
        << accelStats.count << ','
        << gyroStats.count << ','
        << std::fixed << std::setprecision(6)
        << accelMean[0] << ','
        << accelMean[1] << ','
        << accelMean[2] << ','
        << accelStd[0] << ','
        << accelStd[1] << ','
        << accelStd[2] << ','
        << accelNorm << ','
        << gravityNormErrorPercent << ','
        << bestAxis << ','
        << zAxisAngleDeg << ','
        << zDominancePercent << ','
        << gyroMean[0] << ','
        << gyroMean[1] << ','
        << gyroMean[2] << ','
        << gyroStd[0] << ','
        << gyroStd[1] << ','
        << gyroStd[2] << ','
        << gyroRms << ','
        << (normOk ? 1 : 0) << ','
        << (zAngleOk ? 1 : 0) << ','
        << (zDominanceOk ? 1 : 0) << ','
        << (accelStableOk ? 1 : 0) << ','
        << (gyroStableOk ? 1 : 0) << ','
        << (zAxisCanBeGravity ? 1 : 0)
        << '\n';
}

int runImuGravityCheck(
    const rs2::device_list& devices,
    const ImuGravityCheckConfig& config)
{
    if (!deviceListSupportsStream(devices, RS2_STREAM_ACCEL))
    {
        std::cerr << "IMU gravity check failed: no accel motion stream found on connected device.\n";
        return 2;
    }

    const bool gyroAvailable = deviceListSupportsStream(devices, RS2_STREAM_GYRO);
    std::cout << "IMU gravity check: keep the camera still during sampling."
        << " samples=" << config.sampleFrames
        << " warmup=" << config.warmupFrames
        << " gyro=" << (gyroAvailable ? 1 : 0)
        << '\n';

    rs2::pipeline pipeline;
    rs2::config rsConfig;
    rsConfig.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    if (gyroAvailable)
    {
        rsConfig.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
    }

    (void)pipeline.start(rsConfig);
    Vec3Stats accelStats;
    Vec3Stats gyroStats;
    int accelFramesSeen = 0;
    int attempts = 0;
    const int maxAttempts =
        std::max(1000, (config.warmupFrames + config.sampleFrames) * 40);

    while (accelStats.count < config.sampleFrames && attempts < maxAttempts)
    {
        const rs2::frameset frames = pipeline.wait_for_frames();
        ++attempts;

        const rs2::motion_frame accelFrame(frames.first_or_default(RS2_STREAM_ACCEL));
        if (accelFrame)
        {
            ++accelFramesSeen;
            if (accelFramesSeen > config.warmupFrames)
            {
                const rs2_vector data = accelFrame.get_motion_data();
                accelStats.add(cv::Vec3d(data.x, data.y, data.z));
            }
        }

        const rs2::motion_frame gyroFrame(frames.first_or_default(RS2_STREAM_GYRO));
        if (gyroFrame && accelFramesSeen > config.warmupFrames)
        {
            const rs2_vector data = gyroFrame.get_motion_data();
            gyroStats.add(cv::Vec3d(data.x, data.y, data.z));
        }
    }

    pipeline.stop();

    if (accelStats.count < std::max(10, config.sampleFrames / 2))
    {
        std::cerr << "IMU gravity check collected too few accel samples: "
            << accelStats.count << "/" << config.sampleFrames << '\n';
        return 3;
    }
    if (accelStats.count < config.sampleFrames)
    {
        std::cerr << "IMU gravity check warning: collected only "
            << accelStats.count << "/" << config.sampleFrames
            << " accel samples after " << attempts << " waits.\n";
    }

    const cv::Vec3d accelMean = accelStats.mean();
    const cv::Vec3d accelStd = accelStats.stddev();
    const double accelNorm = vec3Norm(accelMean);
    const double gravityNorm = 9.80665;
    const double gravityNormErrorPercent =
        accelNorm <= 1e-9
            ? 100.0
            : std::abs(accelNorm - gravityNorm) * 100.0 / gravityNorm;
    const double zRatio =
        accelNorm <= 1e-9
            ? 0.0
            : std::clamp(std::abs(accelMean[2]) / accelNorm, 0.0, 1.0);
    const double zAxisAngleDeg = radiansToDegrees(std::acos(zRatio));
    const double zDominancePercent = zRatio * 100.0;
    const double accelStdMax = maxVec3Component(accelStd);
    const double gyroRms = gyroStats.rmsMagnitude();

    int bestAxisIndex = 0;
    for (int axis = 1; axis < 3; ++axis)
    {
        if (std::abs(accelMean[axis]) > std::abs(accelMean[bestAxisIndex]))
        {
            bestAxisIndex = axis;
        }
    }
    const std::string bestAxis = signedAxisName(bestAxisIndex, accelMean[bestAxisIndex]);

    const double maxAccelStd = config.maxAccelStdMilliMps2 / 1000.0;
    const double maxGyroRms = config.maxGyroMilliRadps / 1000.0;
    const bool normOk = gravityNormErrorPercent <= config.gravityNormTolerancePercent;
    const bool zAngleOk = zAxisAngleDeg <= config.zMaxAngleDeg;
    const bool zDominanceOk = zDominancePercent >= config.zMinDominancePercent;
    const bool accelStableOk = accelStdMax <= maxAccelStd;
    const bool gyroStableOk = gyroStats.count == 0 || gyroRms <= maxGyroRms;
    const bool zAxisCanBeGravity =
        normOk && zAngleOk && zDominanceOk && accelStableOk && gyroStableOk;

    std::cout << "\n[IMU gravity axis check]\n";
    std::cout << "accel_samples=" << accelStats.count
        << " gyro_samples=" << gyroStats.count
        << " attempts=" << attempts << '\n';
    std::cout << "accel_mean_mps2=("
        << fixedNumber(accelMean[0], 4) << ", "
        << fixedNumber(accelMean[1], 4) << ", "
        << fixedNumber(accelMean[2], 4) << ")"
        << " norm=" << fixedNumber(accelNorm, 4)
        << " gravity_norm_error=" << fixedNumber(gravityNormErrorPercent, 2) << "%\n";
    std::cout << "accel_std_mps2=("
        << fixedNumber(accelStd[0], 4) << ", "
        << fixedNumber(accelStd[1], 4) << ", "
        << fixedNumber(accelStd[2], 4) << ")"
        << " max=" << fixedNumber(accelStdMax, 4)
        << " limit=" << fixedNumber(maxAccelStd, 4) << '\n';
    std::cout << "best_gravity_axis=" << bestAxis
        << " z_axis_angle=" << fixedNumber(zAxisAngleDeg, 2) << "deg"
        << " z_axis_dominance=" << fixedNumber(zDominancePercent, 2) << "%\n";
    if (gyroStats.count > 0)
    {
        const cv::Vec3d gyroMean = gyroStats.mean();
        std::cout << "gyro_mean_radps=("
            << fixedNumber(gyroMean[0], 5) << ", "
            << fixedNumber(gyroMean[1], 5) << ", "
            << fixedNumber(gyroMean[2], 5) << ")"
            << " gyro_rms=" << fixedNumber(gyroRms, 5)
            << " limit=" << fixedNumber(maxGyroRms, 5) << '\n';
    }
    else
    {
        std::cout << "gyro_samples=0; static verdict uses accel stability only.\n";
    }

    std::cout << "checks:"
        << " norm=" << (normOk ? "ok" : "fail")
        << " z_angle=" << (zAngleOk ? "ok" : "fail")
        << " z_dominance=" << (zDominanceOk ? "ok" : "fail")
        << " accel_stable=" << (accelStableOk ? "ok" : "fail")
        << " gyro_stable=" << (gyroStableOk ? "ok" : "fail")
        << '\n';
    std::cout << "verdict="
        << (zAxisCanBeGravity
            ? "PASS: current IMU Z axis can be treated as the gravity axis for this fixed pose."
            : "FAIL: do not treat IMU Z as the gravity axis in this pose.")
        << '\n';

    const std::filesystem::path csvOutput = resolveProjectOutputPath(
        config.csvPath.empty()
            ? std::filesystem::path(defaultImuGravityCheckPath())
            : std::filesystem::path(config.csvPath),
        ".csv");
    writeImuGravityCheckCsv(
        csvOutput,
        config,
        accelStats,
        gyroStats,
        bestAxis,
        accelNorm,
        gravityNormErrorPercent,
        zAxisAngleDeg,
        zDominancePercent,
        accelStdMax,
        gyroRms,
        normOk,
        zAngleOk,
        zDominanceOk,
        accelStableOk,
        gyroStableOk,
        zAxisCanBeGravity);
    std::cout << "IMU gravity check CSV saved: " << csvOutput.string() << '\n';

    return zAxisCanBeGravity ? 0 : 6;
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

cv::Mat buildTiledFrame(const std::vector<cv::Mat>& views, int columns = 2)
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
    const int columnCount = std::max(1, std::min(columns, panelCount));
    const int rowCount = (panelCount + columnCount - 1) / columnCount;
    cv::Mat frame(
        panelSize.height * rowCount,
        panelSize.width * columnCount,
        CV_8UC3,
        cv::Scalar(0, 0, 0));
    for (int index = 0; index < panelCount; ++index)
    {
        const int row = index / columnCount;
        const int column = index % columnCount;
        panels[static_cast<size_t>(index)].copyTo(
            frame(cv::Rect(
                panelSize.width * column,
                panelSize.height * row,
                panelSize.width,
                panelSize.height)));
    }
    return frame;
}

int dashboardColumnCount(int panelCount)
{
    return panelCount > 4 ? 3 : 2;
}

cv::Rect computeStereoOverlapCrop(const cv::Mat& depth16, const SegmentationConfig& config, float depthScale)
{
    if (!config.overlapTrim || depth16.empty())
    {
        return cv::Rect(0, 0, depth16.cols, depth16.rows);
    }

    const int maxDepthUnits = depthUnitsFromMm(config.farMaxDepthMm, depthScale);
    std::vector<int> columnCounts(static_cast<size_t>(depth16.cols), 0);
    std::vector<int> rowCounts(static_cast<size_t>(depth16.rows), 0);
    for (int y = 0; y < depth16.rows; ++y)
    {
        const uint16_t* depthRow = depth16.ptr<uint16_t>(y);
        for (int x = 0; x < depth16.cols; ++x)
        {
            if (depthRow[x] == 0 || depthRow[x] > maxDepthUnits)
            {
                continue;
            }

            ++columnCounts[static_cast<size_t>(x)];
            ++rowCounts[static_cast<size_t>(y)];
        }
    }

    const int columnThreshold = std::max(1, depth16.rows * config.overlapTrimMinSupportPercent / 100);
    const int rowThreshold = std::max(1, depth16.cols * config.overlapTrimMinSupportPercent / 100);

    int left = 0;
    while (left < depth16.cols && columnCounts[static_cast<size_t>(left)] < columnThreshold)
    {
        ++left;
    }
    int right = depth16.cols - 1;
    while (right > left && columnCounts[static_cast<size_t>(right)] < columnThreshold)
    {
        --right;
    }
    int top = 0;
    while (top < depth16.rows && rowCounts[static_cast<size_t>(top)] < rowThreshold)
    {
        ++top;
    }
    int bottom = depth16.rows - 1;
    while (bottom > top && rowCounts[static_cast<size_t>(bottom)] < rowThreshold)
    {
        --bottom;
    }

    if (left >= right || top >= bottom)
    {
        return cv::Rect(0, 0, depth16.cols, depth16.rows);
    }

    const int padding = config.overlapTrimPaddingPixels;
    left = std::max(0, left - padding);
    right = std::min(depth16.cols - 1, right + padding);
    top = std::max(0, top - padding);
    bottom = std::min(depth16.rows - 1, bottom + padding);

    const cv::Rect crop(left, top, right - left + 1, bottom - top + 1);
    if (crop.width < std::max(1, depth16.cols / 2) || crop.height < std::max(1, depth16.rows / 2))
    {
        return cv::Rect(0, 0, depth16.cols, depth16.rows);
    }

    return crop;
}

cv::Rect insetCropRect(const cv::Rect& crop, int insetPixels, const cv::Size& frameSize)
{
    cv::Rect safeCrop = crop & cv::Rect(0, 0, frameSize.width, frameSize.height);
    if (safeCrop.empty() || insetPixels <= 0)
    {
        return safeCrop.empty() ? cv::Rect(0, 0, frameSize.width, frameSize.height) : safeCrop;
    }

    const int inset = std::min(
        insetPixels,
        std::max(0, std::min(safeCrop.width / 2 - 1, safeCrop.height / 2 - 1)));
    if (inset <= 0)
    {
        return safeCrop;
    }

    return cv::Rect(
        safeCrop.x + inset,
        safeCrop.y + inset,
        safeCrop.width - inset * 2,
        safeCrop.height - inset * 2);
}

cv::Size scaledPanelSize(const cv::Size& sourceSize, const SegmentationConfig& config)
{
    const double scale = config.processedViewScalePercent / 100.0;
    return cv::Size(
        std::max(1, static_cast<int>(std::round(sourceSize.width * scale))),
        std::max(1, static_cast<int>(std::round(sourceSize.height * scale))));
}

cv::Mat postProcessViewForDashboard(
    const cv::Mat& view,
    const cv::Rect& crop,
    const cv::Size& targetSize,
    const SegmentationConfig& config)
{
    cv::Mat panel = ensureBgrFrame(view);
    if (panel.empty())
    {
        return {};
    }

    cv::Rect safeCrop = crop & cv::Rect(0, 0, panel.cols, panel.rows);
    if (safeCrop.empty())
    {
        safeCrop = cv::Rect(0, 0, panel.cols, panel.rows);
    }

    cv::Mat output = panel(safeCrop).clone();
    cv::Size finalSize = targetSize;
    if (finalSize.empty())
    {
        finalSize = scaledPanelSize(panel.size(), config);
    }
    if (output.size() != finalSize)
    {
        cv::resize(output, output, finalSize, 0.0, 0.0, cv::INTER_AREA);
    }

    return output;
}

std::vector<cv::Mat> postProcessViewsForDashboard(
    const std::vector<cv::Mat>& views,
    const cv::Rect& crop,
    const cv::Size& targetSize,
    const SegmentationConfig& config)
{
    std::vector<cv::Mat> output;
    output.reserve(views.size());
    for (const cv::Mat& view : views)
    {
        output.push_back(postProcessViewForDashboard(view, crop, targetSize, config));
    }
    return output;
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

    void start()
    {
        if (config_.enabled)
        {
            std::cout << "Recording already armed";
            if (writer_.isOpened())
            {
                std::cout << ": " << outputPath_;
            }
            std::cout << '\n';
            return;
        }
        config_.enabled = true;
        submittedFrames_ = 0;
        std::cout << "Recording command: start. File will open on next rendered frame.\n";
    }

    void stop()
    {
        if (!config_.enabled && !writer_.isOpened())
        {
            std::cout << "Recording already stopped.\n";
            return;
        }
        close();
        config_.enabled = false;
        submittedFrames_ = 0;
        std::cout << "Recording command: stop.\n";
    }

    bool isEnabled() const
    {
        return config_.enabled;
    }

    bool isOpened() const
    {
        return writer_.isOpened();
    }

    void close()
    {
        if (writer_.isOpened())
        {
            writer_.release();
            std::cout << "Recording saved: " << outputPath_
                << " frames=" << writtenFrames_ << '\n';
            writtenFrames_ = 0;
        }
    }

private:
    std::filesystem::path nextOutputPath()
    {
        std::filesystem::path requested = config_.outputPath.empty()
            ? std::filesystem::path(defaultRecordingPath())
            : std::filesystem::path(config_.outputPath);
        if (!config_.outputPath.empty() && openCount_ > 0)
        {
            const std::string extension = requested.has_extension()
                ? requested.extension().string()
                : std::string(".avi");
            requested.replace_extension();
            requested = requested.parent_path() /
                (requested.filename().string() + "_" + timestampForFilename() + extension);
        }
        return resolveProjectOutputPath(requested, ".avi");
    }

    void open(const cv::Size& frameSize)
    {
        const std::filesystem::path output = nextOutputPath();
        if (output.has_parent_path())
        {
            std::filesystem::create_directories(output.parent_path());
        }

        frameSize_ = frameSize;
        outputPath_ = output.string();
        writtenFrames_ = 0;
        std::string backendName;
        bool opened = writer_.open(
            outputPath_,
            cv::CAP_FFMPEG,
            cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
            static_cast<double>(config_.fps),
            frameSize_,
            true);
        if (opened)
        {
            backendName = "ffmpeg";
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
            if (opened)
            {
                backendName = "msmf";
            }
        }
        if (!opened)
        {
            opened = writer_.open(
                outputPath_,
                cv::CAP_OPENCV_MJPEG,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                static_cast<double>(config_.fps),
                frameSize_,
                true);
            if (opened)
            {
                backendName = "opencv-mjpeg";
            }
        }
        if (!opened)
        {
            opened = writer_.open(
                outputPath_,
                cv::VideoWriter::fourcc('M', 'J', 'P', 'G'),
                static_cast<double>(config_.fps),
                frameSize_,
                true);
            if (opened)
            {
                backendName = "default";
            }
        }
        if (!opened || !writer_.isOpened())
        {
            throw std::runtime_error("Failed to open video recording file: " + outputPath_);
        }

        std::cout << "Recording video: " << outputPath_
            << " backend=" << backendName
            << " fps=" << config_.fps
            << " every=" << config_.everyN
            << " scale=" << config_.scalePercent << "%"
            << " size=" << frameSize_.width << "x" << frameSize_.height << '\n';
        ++openCount_;
    }

    VideoRecordingConfig config_;
    cv::VideoWriter writer_;
    cv::Size frameSize_;
    std::string outputPath_;
    int64_t submittedFrames_ = 0;
    int64_t writtenFrames_ = 0;
    int openCount_ = 0;
};

enum class RuntimeCommand
{
    None,
    StartRecording,
    StopRecording,
    Quit
};

RuntimeCommand runtimeCommandFromKey(int key)
{
    if (key < 0)
    {
        return RuntimeCommand::None;
    }
    const int normalizedKey = key & 0xff;
    if (normalizedKey == 27 || normalizedKey == 'q' || normalizedKey == 'Q')
    {
        return RuntimeCommand::Quit;
    }
    if (normalizedKey == 'r' || normalizedKey == 'R')
    {
        return RuntimeCommand::StartRecording;
    }
    if (normalizedKey == 's' || normalizedKey == 'S')
    {
        return RuntimeCommand::StopRecording;
    }
    return RuntimeCommand::None;
}

RuntimeCommand pollConsoleRuntimeCommand()
{
#ifdef _WIN32
    while (_kbhit())
    {
        const int key = _getch();
        RuntimeCommand command = runtimeCommandFromKey(key);
        if (command != RuntimeCommand::None)
        {
            return command;
        }
    }
#endif
    return RuntimeCommand::None;
}

void applyRuntimeCommand(RuntimeCommand command, VideoRecorder& videoRecorder)
{
    switch (command)
    {
    case RuntimeCommand::StartRecording:
        videoRecorder.start();
        break;
    case RuntimeCommand::StopRecording:
        videoRecorder.stop();
        break;
    case RuntimeCommand::None:
    case RuntimeCommand::Quit:
        break;
    }
}

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
    AcceptanceMetricsWriter(
        const AcceptanceMetricsConfig& config,
        bool includePoseColumns,
        bool includeMotionColumns)
        : config_(config),
          includePoseColumns_(includePoseColumns),
          includeMotionColumns_(includeMotionColumns)
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
        const std::vector<FarDistanceMaterial>& farMaterials,
        const cv::Mat& stableMask,
        const BoundaryAnalysis& boundaryAnalysis,
        const CueSelectionSummary& cueSummary,
        const IndoorPlaneAnalysis& indoorPlaneAnalysis,
        int anchorCount,
        int anchorPointsOnCandidates,
        const ColorContourCompletionStats& completionStats,
        const FrameTimingStats& timingStats,
        const PoseState& poseState,
        const MotionState& motionState)
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
        const int farCandidateCount = static_cast<int>(farMaterials.size());
        int farNearestIntervalMinMm = 0;
        int farNearestIntervalMaxMm = 0;
        int farNearestMedianMm = 0;
        int farFarthestIntervalMinMm = 0;
        int farFarthestIntervalMaxMm = 0;
        int farFarthestMedianMm = 0;
        if (!farMaterials.empty())
        {
            const FarDistanceMaterial& nearest = farMaterials.front();
            const FarDistanceMaterial& farthest = farMaterials.back();
            farNearestIntervalMinMm = nearest.intervalMinMm;
            farNearestIntervalMaxMm = nearest.intervalMaxMm;
            farNearestMedianMm = nearest.medianDepthMm;
            farFarthestIntervalMinMm = farthest.intervalMinMm;
            farFarthestIntervalMaxMm = farthest.intervalMaxMm;
            farFarthestMedianMm = farthest.medianDepthMm;
        }

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
            << farCandidateCount << ','
            << farNearestIntervalMinMm << ','
            << farNearestIntervalMaxMm << ','
            << farNearestMedianMm << ','
            << farFarthestIntervalMinMm << ','
            << farFarthestIntervalMaxMm << ','
            << farFarthestMedianMm << ','
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
            << timingStats.captureWaitAlignMs << ','
            << timingStats.depthPostMs << ','
            << timingStats.frameConvertMs << ','
            << timingStats.motionMs << ','
            << timingStats.grayPrepareMs << ','
            << timingStats.anchorMs << ','
            << timingStats.boundaryMs << ','
            << timingStats.extractMs << ','
            << timingStats.farExtractMs << ','
            << timingStats.pclMs << ','
            << timingStats.calibrateMs << ','
            << timingStats.cueMs << ','
            << timingStats.trackerMs << ','
            << timingStats.supportMs << ','
            << timingStats.renderMs << ','
            << timingStats.completionMs << ','
            << timingStats.diagnosticsMs << ','
            << timingStats.displayMs << ','
            << timingStats.recordMs;

        if (includePoseColumns_)
        {
            stream_
                << ',' << (poseState.enabled ? 1 : 0)
                << ',' << (poseState.accelStreamEnabled ? 1 : 0)
                << ',' << (poseState.gyroStreamEnabled ? 1 : 0)
                << ',' << (poseState.accelValid ? 1 : 0)
                << ',' << (poseState.gyroValid ? 1 : 0)
                << ',' << (poseState.orientationValid ? 1 : 0)
                << ',' << poseState.accelFrames
                << ',' << poseState.gyroFrames
                << ',' << std::setprecision(3) << poseState.accelTimestampMs
                << ',' << poseState.gyroTimestampMs
                << ',' << poseState.rollDeg
                << ',' << poseState.pitchDeg
                << ',' << poseState.yawDeg
                << ',' << poseState.accelMps2[0]
                << ',' << poseState.accelMps2[1]
                << ',' << poseState.accelMps2[2]
                << ',' << poseState.gyroRadPerSec[0]
                << ',' << poseState.gyroRadPerSec[1]
                << ',' << poseState.gyroRadPerSec[2];
        }

        if (includeMotionColumns_)
        {
            stream_
                << ',' << (motionState.enabled ? 1 : 0)
                << ',' << (motionState.visualValid ? 1 : 0)
                << ',' << (motionState.imuAccelValid ? 1 : 0)
                << ',' << (motionState.imuGyroValid ? 1 : 0)
                << ',' << motionState.visualFrames
                << ',' << motionState.level
                << ',' << std::setprecision(3) << motionState.score
                << ',' << motionState.visualShiftXPixels
                << ',' << motionState.visualShiftYPixels
                << ',' << motionState.visualShiftPixels
                << ',' << motionState.visualResponse
                << ',' << motionState.visualShiftSmoothPixels
                << ',' << motionState.gyroRadPerSec
                << ',' << motionState.gyroSmoothRadPerSec
                << ',' << motionState.accelDeltaMps2
                << ',' << motionState.accelDeltaSmoothMps2;
        }

        stream_ << '\n';

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
            << "far_candidate_count,far_nearest_interval_min_mm,far_nearest_interval_max_mm,"
            << "far_nearest_median_mm,far_farthest_interval_min_mm,far_farthest_interval_max_mm,"
            << "far_farthest_median_mm,"
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
            << "capture_wait_align_ms,depth_post_ms,frame_convert_ms,motion_ms,gray_prepare_ms,anchor_ms,boundary_ms,"
            << "extract_ms,far_extract_ms,pcl_ms,calibrate_ms,cue_ms,tracker_ms,support_ms,"
            << "render_ms,completion_ms,diagnostics_ms,display_ms,record_ms";
        if (includePoseColumns_)
        {
            stream_
                << ",pose_enabled,pose_accel_stream_enabled,pose_gyro_stream_enabled,"
                << "pose_accel_valid,pose_gyro_valid,pose_orientation_valid,"
                << "pose_accel_frames,pose_gyro_frames,pose_accel_timestamp_ms,pose_gyro_timestamp_ms,"
                << "pose_roll_deg,pose_pitch_deg,pose_yaw_relative_deg,"
                << "pose_accel_x_mps2,pose_accel_y_mps2,pose_accel_z_mps2,"
                << "pose_gyro_x_radps,pose_gyro_y_radps,pose_gyro_z_radps";
        }
        if (includeMotionColumns_)
        {
            stream_
                << ",motion_enabled,motion_visual_valid,motion_imu_accel_valid,motion_imu_gyro_valid,"
                << "motion_visual_frames,motion_level,motion_score,"
                << "motion_visual_shift_x_px,motion_visual_shift_y_px,motion_visual_shift_px,"
                << "motion_visual_response,motion_visual_shift_smooth_px,"
                << "motion_gyro_radps,motion_gyro_smooth_radps,"
                << "motion_accel_delta_mps2,motion_accel_delta_smooth_mps2";
        }
        stream_ << "\n";
        std::cout << "Acceptance metrics: " << outputPath_ << '\n';
    }

    AcceptanceMetricsConfig config_;
    bool includePoseColumns_ = false;
    bool includeMotionColumns_ = false;
    std::ofstream stream_;
    std::string outputPath_;
    std::map<uint64_t, ObservationMaterial> previousStableById_;
};

class ProfileCsvWriter
{
public:
    explicit ProfileCsvWriter(const ProfileCsvConfig& config)
        : config_(config)
    {
    }

    ~ProfileCsvWriter()
    {
        close();
    }

    void write(
        uint64_t frameId,
        double processingMs,
        int candidateCount,
        int anchorSupportedCount,
        int stableCount,
        int farCandidateCount,
        int anchorCount,
        int anchorPointsOnCandidates,
        const FrameTimingStats& timingStats,
        const ColorContourRefreshStats& colorContourRefreshStats)
    {
        if (!config_.enabled)
        {
            return;
        }
        if (!stream_.is_open())
        {
            open();
        }

        const double totalMs = timingStats.captureWaitAlignMs + processingMs;
        stream_
            << frameId << ','
            << std::fixed << std::setprecision(3)
            << totalMs << ','
            << processingMs << ','
            << timingStats.captureWaitAlignMs << ','
            << timingStats.depthPostMs << ','
            << timingStats.frameConvertMs << ','
            << timingStats.motionMs << ','
            << timingStats.grayPrepareMs << ','
            << timingStats.anchorMs << ','
            << timingStats.boundaryMs << ','
            << timingStats.extractMs << ','
            << timingStats.farExtractMs << ','
            << timingStats.pclMs << ','
            << timingStats.calibrateMs << ','
            << timingStats.cueMs << ','
            << timingStats.trackerMs << ','
            << timingStats.supportMs << ','
            << timingStats.renderMs << ','
            << timingStats.completionMs << ','
            << timingStats.diagnosticsMs << ','
            << timingStats.displayMs << ','
            << timingStats.recordMs << ','
            << candidateCount << ','
            << anchorSupportedCount << ','
            << stableCount << ','
            << farCandidateCount << ','
            << anchorCount << ','
            << anchorPointsOnCandidates << ','
            << (colorContourRefreshStats.refreshed ? 1 : 0) << ','
            << (colorContourRefreshStats.cacheReused ? 1 : 0) << ','
            << (colorContourRefreshStats.reasonStartup ? 1 : 0) << ','
            << (colorContourRefreshStats.reasonInterval ? 1 : 0) << ','
            << (colorContourRefreshStats.reasonMotion ? 1 : 0) << ','
            << (colorContourRefreshStats.reasonUnknownSpike ? 1 : 0) << ','
            << (colorContourRefreshStats.reasonFarLoss ? 1 : 0) << ','
            << std::setprecision(3) << colorContourRefreshStats.motionDeltaPercent << ','
            << colorContourRefreshStats.regionCount << ','
            << colorContourRefreshStats.stereoValidCount << ','
            << colorContourRefreshStats.stereoReuseCount << ','
            << (colorContourRefreshStats.asyncSubmitted ? 1 : 0) << ','
            << (colorContourRefreshStats.asyncApplied ? 1 : 0) << ','
            << (colorContourRefreshStats.asyncDropped ? 1 : 0) << ','
            << (colorContourRefreshStats.asyncPending ? 1 : 0) << ','
            << (colorContourRefreshStats.cooldownSkipped ? 1 : 0) << ','
            << (colorContourRefreshStats.roiRefresh ? 1 : 0) << ','
            << colorContourRefreshStats.roiPixels << ','
            << colorContourRefreshStats.roiCandidatePixels << ','
            << (colorContourRefreshStats.roiRejectedEmpty ? 1 : 0) << ','
            << (colorContourRefreshStats.roiRejectedLarge ? 1 : 0) << ','
            << colorContourRefreshStats.roiRefreshedRegionCount << ','
            << colorContourRefreshStats.roiStereoReuseCount << ','
            << colorContourRefreshStats.roiStereoBuiltCount << ','
            << colorContourRefreshStats.roiStereoFailedCount << ','
            << colorContourRefreshStats.roiPreservedStereoCount << ','
            << colorContourRefreshStats.cacheAgeFrames << ','
            << std::setprecision(3) << colorContourRefreshStats.asyncWorkerMs
            << '\n';
    }

    void close()
    {
        if (stream_.is_open())
        {
            stream_.close();
            std::cout << "Profile CSV saved: " << outputPath_ << '\n';
        }
    }

private:
    void open()
    {
        const std::filesystem::path output = resolveProjectOutputPath(
            config_.csvPath.empty()
                ? std::filesystem::path(defaultProfileCsvPath())
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
            throw std::runtime_error("Failed to open profile CSV file: " + outputPath_);
        }

        stream_
            << "frame_index,total_ms,processing_ms,capture_wait_align_ms,"
            << "depth_post_ms,frame_convert_ms,motion_ms,gray_prepare_ms,anchor_ms,boundary_ms,"
            << "extract_ms,far_extract_ms,pcl_ms,calibrate_ms,cue_ms,tracker_ms,support_ms,"
            << "render_ms,completion_ms,diagnostics_ms,display_ms,record_ms,"
            << "candidate_count,anchor_supported_count,stable_count,far_candidate_count,"
            << "anchor_count,anchor_points_on_candidates,"
            << "color_contour_refreshed,color_contour_cache_reused,"
            << "color_contour_refresh_startup,color_contour_refresh_interval,"
            << "color_contour_refresh_motion,color_contour_refresh_unknown_spike,"
            << "color_contour_refresh_far_loss,color_contour_motion_delta_percent,"
            << "color_contour_region_count,color_contour_stereo_valid_count,"
            << "color_contour_stereo_reuse_count,"
            << "color_contour_async_submitted,color_contour_async_applied,"
            << "color_contour_async_dropped,color_contour_async_pending,"
            << "color_contour_refresh_cooldown_skipped,"
            << "color_contour_refresh_roi,color_contour_refresh_roi_pixels,"
            << "color_contour_refresh_roi_candidate_pixels,"
            << "color_contour_refresh_roi_rejected_empty,"
            << "color_contour_refresh_roi_rejected_large,"
            << "color_contour_refresh_roi_region_count,"
            << "color_contour_refresh_roi_stereo_reuse_count,"
            << "color_contour_refresh_roi_stereo_built_count,"
            << "color_contour_refresh_roi_stereo_failed_count,"
            << "color_contour_refresh_roi_preserved_stereo_count,"
            << "color_contour_cache_age_frames,color_contour_async_worker_ms\n";
        std::cout << "Profile CSV: " << outputPath_ << '\n';
    }

    ProfileCsvConfig config_;
    std::ofstream stream_;
    std::string outputPath_;
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

struct StablePlaneAnchorAssignment
{
    int anchorLabel = 0;
    int anchorCount = 0;
};

int buildStablePlaneAnchorLabels(
    const cv::Mat& anchorMask,
    const cv::Mat& splitBoundaryMask,
    const cv::Mat& depth16,
    const SegmentationConfig& config,
    float depthScale,
    cv::Mat& anchorLabels)
{
    anchorLabels.release();
    if (!config.stablePlaneMerge || anchorMask.empty() || depth16.empty())
    {
        return 0;
    }

    cv::Mat clusterMask = dilateMask(anchorMask, config.stablePlaneMergeAnchorGapPixels * 2 + 1);
    const cv::Mat validMask = makeRawDepthRangeMask(depth16, config, depthScale);
    clusterMask.setTo(0, ~validMask);

    if (!splitBoundaryMask.empty())
    {
        cv::Mat boundaryBarrier = dilateMask(
            splitBoundaryMask,
            std::max(3, config.splitBoundaryPixels * 2 + 1));
        clusterMask.setTo(0, boundaryBarrier);
    }

    if (cv::countNonZero(clusterMask) == 0)
    {
        return 0;
    }

    cv::Mat stats;
    cv::Mat centroids;
    return cv::connectedComponentsWithStats(clusterMask, anchorLabels, stats, centroids, 8, CV_32S);
}

StablePlaneAnchorAssignment assignMaterialToStablePlaneAnchorCluster(
    const ObservationMaterial& material,
    const cv::Mat& anchorMask,
    const cv::Mat& anchorLabels)
{
    StablePlaneAnchorAssignment assignment;
    if (material.contour.size() < 3 || anchorMask.empty() || anchorLabels.empty())
    {
        return assignment;
    }

    const cv::Rect imageBounds(0, 0, anchorMask.cols, anchorMask.rows);
    const cv::Rect roi = material.roi & imageBounds;
    if (roi.empty())
    {
        return assignment;
    }

    const cv::Mat localMask = contourMaskInRoi(material, roi);
    std::map<int, int> labelVotes;
    for (int localY = 0; localY < roi.height; ++localY)
    {
        const uint8_t* maskRow = localMask.ptr<uint8_t>(localY);
        const uint8_t* anchorRow = anchorMask.ptr<uint8_t>(roi.y + localY);
        const int* labelRow = anchorLabels.ptr<int>(roi.y + localY);
        for (int localX = 0; localX < roi.width; ++localX)
        {
            const int x = roi.x + localX;
            if (maskRow[localX] == 0 || anchorRow[x] == 0)
            {
                continue;
            }

            ++assignment.anchorCount;
            const int anchorLabel = labelRow[x];
            if (anchorLabel > 0)
            {
                ++labelVotes[anchorLabel];
            }
        }
    }

    int bestVotes = 0;
    for (const auto& [label, votes] : labelVotes)
    {
        if (votes > bestVotes)
        {
            bestVotes = votes;
            assignment.anchorLabel = label;
        }
    }

    return assignment;
}

int countMaterialOverlapInMask(
    const ObservationMaterial& material,
    const cv::Mat& globalMask)
{
    if (material.contour.size() < 3 || globalMask.empty())
    {
        return 0;
    }

    const cv::Rect imageBounds(0, 0, globalMask.cols, globalMask.rows);
    const cv::Rect roi = material.roi & imageBounds;
    if (roi.empty())
    {
        return 0;
    }

    const cv::Mat localMask = contourMaskInRoi(material, roi);
    cv::Mat overlap;
    cv::bitwise_and(globalMask(roi), localMask, overlap);
    return cv::countNonZero(overlap);
}

bool buildMergedStablePlaneMaterial(
    const cv::Mat& componentMask,
    const std::vector<ObservationMaterial>& sourceMaterials,
    const std::vector<size_t>& sourceIndexes,
    const cv::Mat& depth16,
    const cv::Mat& anchorMask,
    const SegmentationConfig& config,
    float depthScale,
    const rs2_intrinsics& intrinsics,
    uint64_t sourceFrameId,
    ObservationMaterial& mergedMaterial)
{
    if (componentMask.empty() || sourceIndexes.size() < 2)
    {
        return false;
    }

    const int pixelCount = cv::countNonZero(componentMask);
    if (pixelCount < config.minAreaPixels)
    {
        return false;
    }

    const int frameArea = depth16.rows * depth16.cols;
    const int maxAreaPixels = frameArea * config.stablePlaneMergeMaxAreaPercent / 100;
    if (pixelCount > maxAreaPixels)
    {
        return false;
    }

    std::vector<std::vector<cv::Point>> contours;
    cv::findContours(componentMask.clone(), contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_NONE);
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

    const cv::Rect frameRect(0, 0, depth16.cols, depth16.rows);
    const cv::Rect roi = cv::boundingRect(preciseContour) & frameRect;
    const int maxRoiAreaPixels = frameArea * config.stablePlaneMergeMaxRoiAreaPercent / 100;
    if (roi.empty() || roi.area() > maxRoiAreaPixels)
    {
        return false;
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
    const bool canDeproject = intrinsics.fx > 0.0f && intrinsics.fy > 0.0f;
    int pointCount = 0;

    for (int localY = 0; localY < roi.height; ++localY)
    {
        const uint8_t* maskRow = componentMask.ptr<uint8_t>(roi.y + localY);
        const uint16_t* depthRow = depth16.ptr<uint16_t>(roi.y + localY);
        for (int localX = 0; localX < roi.width; ++localX)
        {
            const int x = roi.x + localX;
            if (maskRow[x] == 0)
            {
                continue;
            }

            const uint16_t depthUnits = depthRow[x];
            if (depthUnits == 0)
            {
                continue;
            }

            depthSumUnits += depthUnits;
            minDepthUnits = std::min(minDepthUnits, depthUnits);
            maxDepthUnits = std::max(maxDepthUnits, depthUnits);
            ++validDepthCount;

            if (!canDeproject)
            {
                continue;
            }

            const cv::Point3f point = deprojectPixelMeters(x, roi.y + localY, depthUnits, depthScale, intrinsics);
            minPoint.x = std::min(minPoint.x, point.x);
            minPoint.y = std::min(minPoint.y, point.y);
            minPoint.z = std::min(minPoint.z, point.z);
            maxPoint.x = std::max(maxPoint.x, point.x);
            maxPoint.y = std::max(maxPoint.y, point.y);
            maxPoint.z = std::max(maxPoint.z, point.z);
            ++pointCount;
        }
    }

    if (validDepthCount == 0)
    {
        return false;
    }

    uint64_t observationId = sourceMaterials[sourceIndexes.front()].observationId;
    int pclClusterId = sourceMaterials[sourceIndexes.front()].pclClusterId;
    for (const size_t sourceIndex : sourceIndexes)
    {
        observationId = std::min(observationId, sourceMaterials[sourceIndex].observationId);
        if (sourceMaterials[sourceIndex].pclClusterId != pclClusterId)
        {
            pclClusterId = 0;
        }
    }

    mergedMaterial = ObservationMaterial{};
    mergedMaterial.sourceFrameId = sourceFrameId;
    mergedMaterial.observationId = observationId;
    mergedMaterial.roi = roi;
    mergedMaterial.center = cv::Point(roi.x + roi.width / 2, roi.y + roi.height / 2);
    mergedMaterial.pixelCount = pixelCount;
    mergedMaterial.depthMinMm = static_cast<int>(minDepthUnits * depthScale * 1000.0f + 0.5f);
    mergedMaterial.depthMaxMm = static_cast<int>(maxDepthUnits * depthScale * 1000.0f + 0.5f);
    mergedMaterial.observedDepthMinMm = mergedMaterial.depthMinMm;
    mergedMaterial.observedDepthMaxMm = mergedMaterial.depthMaxMm;
    mergedMaterial.meanDepthMm =
        static_cast<int>((depthSumUnits / static_cast<double>(validDepthCount)) * depthScale * 1000.0f + 0.5f);
    mergedMaterial.pclClusterId = pclClusterId;
    mergedMaterial.contourArea = contourArea;
    mergedMaterial.hasPointCloudBounds = pointCount > 0;
    mergedMaterial.minPointMeters = minPoint;
    mergedMaterial.maxPointMeters = maxPoint;
    mergedMaterial.contour = std::move(preciseContour);

    const StableContourSupport support =
        measureStableContourSupport(mergedMaterial, anchorMask, depth16, depthScale);
    if (support.anchorCount < config.stablePlaneMergeMinAnchors)
    {
        return false;
    }

    mergedMaterial.center = support.anchorCenter;
    mergedMaterial.meanDepthMm = support.meanAnchorDepthMm;
    mergedMaterial.groupId = support.anchorCount;
    return true;
}

int countStableAnchorPointsOnMaterials(
    const std::vector<ObservationMaterial>& materials,
    const cv::Mat& anchorMask,
    const cv::Mat& depth16,
    float depthScale)
{
    int totalAnchorPoints = 0;
    for (const ObservationMaterial& material : materials)
    {
        totalAnchorPoints += measureStableContourSupport(material, anchorMask, depth16, depthScale).anchorCount;
    }
    return totalAnchorPoints;
}

std::vector<ObservationMaterial> mergeStablePlaneFragmentsByAnchors(
    const std::vector<ObservationMaterial>& materials,
    const cv::Mat& anchorMask,
    const cv::Mat& splitBoundaryMask,
    const cv::Mat& depth16,
    const SegmentationConfig& config,
    float depthScale,
    const rs2_intrinsics& intrinsics,
    uint64_t sourceFrameId)
{
    if (!config.stablePlaneMerge || materials.size() < 2 || anchorMask.empty() || depth16.empty())
    {
        return materials;
    }

    cv::Mat anchorLabels;
    const int anchorLabelCount =
        buildStablePlaneAnchorLabels(anchorMask, splitBoundaryMask, depth16, config, depthScale, anchorLabels);
    if (anchorLabelCount <= 1 || anchorLabels.empty())
    {
        return materials;
    }

    std::vector<StablePlaneAnchorAssignment> assignments(materials.size());
    std::map<int, std::vector<size_t>> materialsByAnchorLabel;
    for (size_t index = 0; index < materials.size(); ++index)
    {
        assignments[index] =
            assignMaterialToStablePlaneAnchorCluster(materials[index], anchorMask, anchorLabels);
        if (assignments[index].anchorLabel > 0)
        {
            materialsByAnchorLabel[assignments[index].anchorLabel].push_back(index);
        }
    }

    std::vector<bool> consumed(materials.size(), false);
    std::vector<ObservationMaterial> mergedMaterials;
    const cv::Size frameSize(depth16.cols, depth16.rows);
    const cv::Mat mergeKernel = cv::getStructuringElement(
        cv::MORPH_ELLIPSE,
        cv::Size(config.stablePlaneMergeMaskGapPixels | 1, config.stablePlaneMergeMaskGapPixels | 1));

    for (const auto& [anchorLabel, groupIndexes] : materialsByAnchorLabel)
    {
        (void)anchorLabel;
        if (groupIndexes.size() < 2)
        {
            continue;
        }

        int groupAnchorCount = 0;
        cv::Mat groupMask = cv::Mat::zeros(frameSize, CV_8UC1);
        for (const size_t materialIndex : groupIndexes)
        {
            groupAnchorCount += assignments[materialIndex].anchorCount;
            if (materials[materialIndex].contour.size() < 3)
            {
                continue;
            }

            const std::vector<std::vector<cv::Point>> contours{materials[materialIndex].contour};
            cv::drawContours(groupMask, contours, -1, cv::Scalar(255), cv::FILLED, cv::LINE_8);
        }
        if (groupAnchorCount < config.stablePlaneMergeMinAnchors)
        {
            continue;
        }

        cv::morphologyEx(groupMask, groupMask, cv::MORPH_CLOSE, mergeKernel);
        if (!splitBoundaryMask.empty())
        {
            groupMask.setTo(0, splitBoundaryMask);
        }

        cv::Mat componentLabels;
        cv::Mat componentStats;
        cv::Mat componentCentroids;
        const int componentLabelCount = cv::connectedComponentsWithStats(
            groupMask,
            componentLabels,
            componentStats,
            componentCentroids,
            8,
            CV_32S);

        for (int label = 1; label < componentLabelCount; ++label)
        {
            if (componentStats.at<int>(label, cv::CC_STAT_AREA) < config.minAreaPixels)
            {
                continue;
            }

            cv::Mat componentMask;
            cv::compare(componentLabels, label, componentMask, cv::CMP_EQ);
            std::vector<size_t> componentMaterialIndexes;
            int componentAnchorCount = 0;
            for (const size_t materialIndex : groupIndexes)
            {
                if (consumed[materialIndex])
                {
                    continue;
                }
                if (countMaterialOverlapInMask(materials[materialIndex], componentMask) == 0)
                {
                    continue;
                }

                componentMaterialIndexes.push_back(materialIndex);
                componentAnchorCount += assignments[materialIndex].anchorCount;
            }

            if (componentMaterialIndexes.size() < 2 ||
                componentAnchorCount < config.stablePlaneMergeMinAnchors)
            {
                continue;
            }

            ObservationMaterial mergedMaterial;
            if (!buildMergedStablePlaneMaterial(
                    componentMask,
                    materials,
                    componentMaterialIndexes,
                    depth16,
                    anchorMask,
                    config,
                    depthScale,
                    intrinsics,
                    sourceFrameId,
                    mergedMaterial))
            {
                continue;
            }

            for (const size_t materialIndex : componentMaterialIndexes)
            {
                consumed[materialIndex] = true;
            }
            mergedMaterials.push_back(std::move(mergedMaterial));
        }
    }

    if (mergedMaterials.empty())
    {
        return materials;
    }

    std::vector<ObservationMaterial> output;
    output.reserve(materials.size());
    for (const ObservationMaterial& material : mergedMaterials)
    {
        output.push_back(material);
    }
    for (size_t index = 0; index < materials.size(); ++index)
    {
        if (!consumed[index])
        {
            output.push_back(materials[index]);
        }
    }

    std::sort(
        output.begin(),
        output.end(),
        [](const ObservationMaterial& lhs, const ObservationMaterial& rhs)
        {
            return lhs.contourArea > rhs.contourArea;
        });
    if (output.size() > static_cast<size_t>(config.maxMaterials))
    {
        output.resize(static_cast<size_t>(config.maxMaterials));
    }
    return output;
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

std::vector<ObservationMaterial> filterMosaicTrackQuality(
    const std::vector<ObservationMaterial>& materials,
    const std::map<uint64_t, StableContourTrackAggregate>& trackAggregates,
    int frameIndex,
    const SegmentationConfig& config)
{
    if (!config.mosaicForegroundGate)
    {
        return materials;
    }

    std::vector<ObservationMaterial> filtered;
    filtered.reserve(materials.size());
    for (const ObservationMaterial& material : materials)
    {
        const auto aggregateIt = trackAggregates.find(material.observationId);
        if (aggregateIt == trackAggregates.end())
        {
            continue;
        }

        const StableContourTrackAggregate& aggregate = aggregateIt->second;
        const bool nearForeground =
            (material.observedDepthMinMm > 0 && material.observedDepthMinMm <= config.foregroundKeepDepthMm) ||
            (material.meanDepthMm > 0 && material.meanDepthMm <= config.foregroundKeepDepthMm);
        const int minObservations = nearForeground
            ? config.mosaicNearMinTrackObservations
            : config.mosaicMinTrackObservations;
        const int minScore = nearForeground
            ? config.mosaicNearMinTrackScore
            : config.mosaicMinTrackScore;

        if (aggregate.observations < minObservations)
        {
            continue;
        }
        if (frameIndex - aggregate.lastFrame > config.mosaicMaxTrackStaleFrames)
        {
            continue;
        }

        const double score = contourStabilityScore(aggregate, std::max(1, frameIndex));
        if (score < static_cast<double>(minScore))
        {
            continue;
        }

        filtered.push_back(material);
    }
    return filtered;
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

cv::Mat buildRealtimeStableContourVisualization(
    const cv::Mat& colorBgr,
    const std::vector<ObservationMaterial>& anchorSupportedCandidates,
    const std::vector<ObservationMaterial>& stableMaterials,
    int frameIndex)
{
    cv::Mat view;
    colorBgr.convertTo(view, -1, 0.70, 0.0);

    for (const ObservationMaterial& material : anchorSupportedCandidates)
    {
        if (material.contour.size() < 3)
        {
            continue;
        }
        const std::vector<std::vector<cv::Point>> contours{material.contour};
        cv::drawContours(view, contours, -1, cv::Scalar(0, 210, 255), 1, cv::LINE_8);
    }

    for (size_t index = 0; index < stableMaterials.size(); ++index)
    {
        const ObservationMaterial& material = stableMaterials[index];
        if (material.contour.size() < 3)
        {
            continue;
        }

        const std::vector<std::vector<cv::Point>> contours{material.contour};
        cv::drawContours(view, contours, -1, palette(index), 2, cv::LINE_8);
    }

    drawOutlinedText(
        view,
        "F" + std::to_string(frameIndex) +
            " realtime: yellow=candidates color=stable",
        cv::Point(12, 24),
        0.48);
    return view;
}

cv::Scalar farDistanceColor(size_t index)
{
    static const std::array<cv::Scalar, 6> colors{
        cv::Scalar(255, 255, 0),
        cv::Scalar(255, 170, 0),
        cv::Scalar(255, 110, 90),
        cv::Scalar(220, 90, 255),
        cv::Scalar(120, 170, 255),
        cv::Scalar(80, 230, 180)};
    return colors[index % colors.size()];
}

std::string formatDistanceIntervalMeters(int minMm, int maxMm)
{
    return fixedNumber(minMm / 1000.0, 1) + "-" + fixedNumber(maxMm / 1000.0, 1) + "m";
}

std::string farRelativeWord(int rankNearFirst, int total)
{
    if (total <= 1)
    {
        return "far";
    }
    if (rankNearFirst <= 1)
    {
        return "nearest";
    }
    if (rankNearFirst >= total)
    {
        return "farthest";
    }
    return "farther";
}

void drawFarDistanceOverlay(
    cv::Mat& view,
    const std::vector<FarDistanceMaterial>& farMaterials,
    const SegmentationConfig& config)
{
    if (view.empty() || !config.farDistanceIntervals)
    {
        return;
    }

    std::ostringstream orderLine;
    if (farMaterials.empty())
    {
        orderLine << "far intervals >" << fixedNumber(config.maxDepthMm / 1000.0, 1) << "m: 0";
    }
    else
    {
        orderLine << "far intervals near->far: ";
        for (size_t index = 0; index < farMaterials.size(); ++index)
        {
            if (index > 0)
            {
                orderLine << "<";
            }
            orderLine << "F" << farMaterials[index].rankNearFirst;
        }
    }
    drawOutlinedText(view, orderLine.str(), cv::Point(12, 48), 0.42, cv::Scalar(255, 255, 0));

    const int total = static_cast<int>(farMaterials.size());
    for (size_t index = 0; index < farMaterials.size(); ++index)
    {
        const FarDistanceMaterial& material = farMaterials[index];
        if (material.contour.size() < 3)
        {
            continue;
        }

        const cv::Scalar color = farDistanceColor(index);
        const std::vector<std::vector<cv::Point>> contours{material.contour};
        cv::drawContours(view, contours, -1, color, 2, cv::LINE_AA);
        cv::circle(view, material.center, 4, color, cv::FILLED, cv::LINE_AA);

        const std::string label =
            "F" + std::to_string(material.rankNearFirst) + " " +
            farRelativeWord(material.rankNearFirst, total) + " " +
            formatDistanceIntervalMeters(material.intervalMinMm, material.intervalMaxMm);

        int baseline = 0;
        const double fontScale = 0.42;
        const int thickness = 1;
        const cv::Size textSize = cv::getTextSize(
            label,
            cv::FONT_HERSHEY_SIMPLEX,
            fontScale,
            thickness,
            &baseline);
        cv::Point labelOrigin(material.center.x + 8, material.center.y - 8);
        labelOrigin.x = std::clamp(labelOrigin.x, 2, std::max(2, view.cols - textSize.width - 2));
        labelOrigin.y = std::clamp(labelOrigin.y, textSize.height + 52, std::max(textSize.height + 52, view.rows - 4));
        drawOutlinedText(view, label, labelOrigin, fontScale, color);
    }
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
        anchorSupportedCandidates = mergeStablePlaneFragmentsByAnchors(
            anchorSupportedCandidates,
            anchorMask,
            splitBoundaryMask,
            depth16,
            config,
            depthScale,
            colorIntrinsics,
            frameId);
        anchorPointsOnCandidates =
            countStableAnchorPointsOnMaterials(anchorSupportedCandidates, anchorMask, depth16, depthScale);

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
        RuntimeCommand runtimeCommand = RuntimeCommand::None;
        if (showWindow)
        {
            cv::imshow(kStableContourWindow, stableContourView);
            const int key = cv::waitKey(1);
            runtimeCommand = runtimeCommandFromKey(key);
        }
        if (recordingConfig.commandControl)
        {
            const RuntimeCommand consoleCommand = pollConsoleRuntimeCommand();
            if (consoleCommand != RuntimeCommand::None)
            {
                runtimeCommand = consoleCommand;
            }
            applyRuntimeCommand(runtimeCommand, videoRecorder);
        }
        if (runtimeCommand == RuntimeCommand::Quit)
        {
            exitRequested = true;
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
        << "D455 five-panel observation-material dashboard demo\n"
        << "Keys: r starts recording, s stops recording, q or Esc exits.\n"
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
        << "  --record-command-control\n"
        << "  --no-record-command-control\n"
        << "  --record-fps=30\n"
        << "  --record-every-n=1\n"
        << "  --record-scale-percent=100\n"
        << "  --acceptance-baseline\n"
        << "  --acceptance-no-record\n"
        << "  --acceptance-label=acceptance_baseline\n"
        << "  --acceptance-csv=recordings\\acceptance_baseline.csv\n"
        << "  --profile-csv\n"
        << "  --profile-csv=recordings\\profile.csv\n"
        << "  --pose-read\n"
        << "  --no-pose-read\n"
        << "  --pose-overlay\n"
        << "  --no-pose-overlay\n"
        << "  --gravity-line\n"
        << "  --no-gravity-line\n"
        << "  --pose-log\n"
        << "  --pose-log-every-n=30\n"
        << "  --pose-smooth-percent=20\n"
        << "  --motion-diagnostics\n"
        << "  --no-motion-diagnostics\n"
        << "  --motion-overlay\n"
        << "  --no-motion-overlay\n"
        << "  --motion-log\n"
        << "  --motion-log-every-n=30\n"
        << "  --motion-smooth-percent=35\n"
        << "  --motion-visual-downscale-width-px=160\n"
        << "  --motion-visual-min-response-percent=12\n"
        << "  --motion-visual-move-millipx=2000\n"
        << "  --motion-visual-shake-millipx=8000\n"
        << "  --motion-gyro-move-milliradps=80\n"
        << "  --motion-gyro-shake-milliradps=250\n"
        << "  --motion-accel-delta-move-milli-mps2=450\n"
        << "  --motion-accel-delta-shake-milli-mps2=1400\n"
        << "  --imu-gravity-check\n"
        << "  --imu-gravity-frames=120\n"
        << "  --imu-gravity-warmup=30\n"
        << "  --imu-gravity-z-max-angle-deg=10\n"
        << "  --imu-gravity-z-min-dominance-percent=90\n"
        << "  --imu-gravity-norm-tolerance-percent=15\n"
        << "  --imu-gravity-max-accel-std-milli-mps2=250\n"
        << "  --imu-gravity-max-gyro-milliradps=50\n"
        << "  --imu-gravity-csv=recordings\\imu_gravity_check.csv\n"
        << "  --capture-replay-dir=datasets\\near_single_object\n"
        << "  --capture-replay-frames=120\n"
        << "  --capture-replay-warmup=30\n"
        << "  --replay-dir=datasets\\near_single_object\n"
        << "  --no-display\n"
        << "  --max-frames=0\n"
        << "  --min-depth-mm=250\n"
        << "  --max-depth-mm=3500\n"
        << "  --far-distance-intervals\n"
        << "  --no-far-distance-intervals\n"
        << "  --far-max-depth-mm=12000\n"
        << "  --far-interval-mm=1500\n"
        << "  --far-min-area-px=600\n"
        << "  --far-max-materials=8\n"
        << "  --far-morph-kernel-px=7\n"
        << "  --near-plane-display\n"
        << "  --no-near-plane-display\n"
        << "  --near-plane-max-depth-mm=2200\n"
        << "  --near-plane-min-area-percent=3\n"
        << "  --near-plane-max-materials=2\n"
        << "  --near-plane-normal-min-percent=60\n"
        << "  --near-plane-min-center-y-percent=30\n"
        << "  --near-plane-sample-step-px=16\n"
        << "  --near-plane-morph-kernel-px=9\n"
        << "  --near-plane-frame-interval=10\n"
        << "  --cluster-map\n"
        << "  --cluster-map-export=recordings\\cluster_map_sample\n"
        << "  --analysis-export-every-n=30\n"
        << "  --cluster-map-visual-min-area-px=700\n"
        << "  --cluster-map-visual-morph-kernel-px=7\n"
        << "  --cluster-map-max-visual-regions=24\n"
        << "  --quality-segmentation\n"
        << "  --no-quality-segmentation\n"
        << "  --color-segmentation\n"
        << "  --no-color-segmentation\n"
        << "  --color-refine-depth-masks\n"
        << "  --no-color-refine-depth-masks\n"
        << "  --stereo-contour-distance\n"
        << "  --no-stereo-contour-distance\n"
        << "  --stereo-contour-reuse-cached-on-refresh\n"
        << "  --no-stereo-contour-reuse-cached-on-refresh\n"
        << "  --async-color-contour-refresh\n"
        << "  --no-async-color-contour-refresh\n"
        << "  --async-color-contour-low-priority\n"
        << "  --no-async-color-contour-low-priority\n"
        << "  --final-segmentation-export=recordings\\final_segmentation_sample\n"
        << "  --color-segmentation-min-area-px=700\n"
        << "  --color-segmentation-max-roi-area-percent=55\n"
        << "  --color-segmentation-border-reject-area-percent=28\n"
        << "  --color-segmentation-color-bins=6\n"
        << "  --color-segmentation-mean-shift-spatial=9\n"
        << "  --color-segmentation-mean-shift-color=18\n"
        << "  --color-refine-min-overlap-percent=18\n"
        << "  --color-refine-max-area-delta-percent=280\n"
        << "  --non-precise-color-ownership-min-percent=10\n"
        << "  --d455-precision-min-far-depth-support-percent=10 (compat alias)\n"
        << "  --d455-precision-min-color-depth-support-percent=10 (compat alias)\n"
        << "  --color-contour-frame-interval=1\n"
        << "  --color-contour-refresh-min-gap-frames=0\n"
        << "  --color-contour-refresh-motion-roi\n"
        << "  --no-color-contour-refresh-motion-roi\n"
        << "  --color-contour-refresh-roi-drop-stereo-failed\n"
        << "  --no-color-contour-refresh-roi-drop-stereo-failed\n"
        << "  --color-contour-refresh-roi-padding-px=48\n"
        << "  --color-contour-refresh-max-roi-area-percent=35\n"
        << "  --color-contour-refresh-on-motion\n"
        << "  --color-contour-refresh-motion-delta-percent=25\n"
        << "  --color-contour-refresh-on-unknown-spike\n"
        << "  --color-contour-refresh-unknown-percent=5\n"
        << "  --color-contour-refresh-on-far-loss\n"
        << "  --stereo-contour-min-disparity-tenths-px=5\n"
        << "  --stereo-contour-max-regions-per-frame=32\n"
        << "  --stereo-contour-max-roi-area-percent=100\n"
        << "  --stereo-contour-baseline-mm=95\n"
        << "  --mosaic-foreground-gate\n"
        << "  --no-mosaic-foreground-gate\n"
        << "  --mosaic-max-material-area-percent=32\n"
        << "  --mosaic-border-reject-area-percent=5\n"
        << "  --mosaic-min-track-observations=45\n"
        << "  --mosaic-min-track-score=45\n"
        << "  --mosaic-near-min-track-observations=3\n"
        << "  --mosaic-near-min-track-score=0\n"
        << "  --mosaic-max-track-stale-frames=2\n"
        << "  --extra-candidates-in-mosaic\n"
        << "  --no-extra-candidates-in-mosaic\n"
        << "  --processed-view-scale-percent=75\n"
        << "  --overlap-trim\n"
        << "  --no-overlap-trim\n"
        << "  --overlap-trim-min-support-percent=3\n"
        << "  --overlap-trim-padding-px=6\n"
        << "  --overlap-trim-extra-crop-px=2\n"
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
        << "  --stable-plane-merge-anchor-gap-px=14\n"
        << "  --stable-plane-merge-mask-gap-px=21\n"
        << "  --stable-plane-merge-min-anchors=24\n"
        << "  --stable-plane-merge-max-area-percent=85\n"
        << "  --stable-plane-merge-max-roi-area-percent=98\n"
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
        << "  --realtime-30\n"
        << "  --no-realtime-30\n"
        << "  --realtime-skip-depth-post\n"
        << "  --no-realtime-skip-depth-post\n"
        << "  --realtime-fast-depth-post\n"
        << "  --no-realtime-fast-depth-post\n"
        << "  --realtime-lite-visualization\n"
        << "  --no-realtime-lite-visualization\n"
        << "  --realtime-disable-pcl\n"
        << "  --no-realtime-disable-pcl\n"
        << "  --realtime-disable-depth-confirm-split\n"
        << "  --no-realtime-disable-depth-confirm-split\n"
        << "  --realtime-depth-only-boundary\n"
        << "  --no-realtime-depth-only-boundary\n"
        << "  --realtime-pcl-frame-interval=6\n"
        << "  --realtime-anchor-step-px=6\n"
        << "  --realtime-stable-contour-min-anchors=10\n"
        << "  --realtime-depth-slice-mm=450\n"
        << "  --color-contour-completion\n"
        << "  --no-color-contour-completion\n"
        << "  --color-contour-primary\n"
        << "  --no-color-contour-primary\n"
        << "  --color-contour-completion-padding-px=12\n"
        << "  --color-contour-completion-min-iou-percent=72\n"
        << "  --color-contour-completion-max-area-delta-percent=24\n"
        << "  --color-contour-completion-max-center-shift-px=16\n"
        << "  --color-contour-completion-close-px=3\n"
        << "  --color-contour-completion-other-guard-px=7\n"
        << "  --color-contour-completion-max-other-overlap-px=4\n"
        << "  --color-contour-primary-min-anchors=24\n"
        << "  --color-contour-primary-min-overlap-percent=45\n"
        << "  --color-contour-primary-max-area-delta-percent=220\n"
        << "  --color-contour-primary-max-center-shift-px=96\n"
        << "  --color-contour-primary-max-area-percent=85\n"
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
        << "  --pcl-clustering\n"
        << "  --no-pcl-clustering\n"
        << "  --no-spatial-cluster-check\n"
        << "  --stable-plane-merge\n"
        << "  --no-stable-plane-merge\n"
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
int runReplayDirectory(
    const std::string& replayDirText,
    int argc,
    char** argv,
    SegmentationConfig config,
    int maxFrames)
{
    const std::filesystem::path replayDir = resolveProjectOutputPath(std::filesystem::path(replayDirText), "");
    std::vector<ReplayFramePaths> replayFrames = listReplayFrames(replayDir);
    if (replayFrames.empty())
    {
        std::cerr << "No replay frames found under " << (replayDir / "frames").string() << '\n';
        return 1;
    }
    if (maxFrames > 0 && static_cast<size_t>(maxFrames) < replayFrames.size())
    {
        replayFrames.resize(static_cast<size_t>(maxFrames));
    }

    RgbDepthAccuracyConfig stableDisplayConfig = parseRgbDepthAccuracyConfig(argc, argv);
    stableDisplayConfig.stableContourVideo = true;
    if (config.realtime30)
    {
        const bool explicitAnchorStep =
            hasOptionPrefix(argc, argv, "--rgb-depth-anchor-step-px=");
        const bool explicitStableMinAnchors =
            hasOptionPrefix(argc, argv, "--stable-contour-min-anchors=");
        const bool explicitDepthSlice =
            hasOptionPrefix(argc, argv, "--depth-slice-mm=");
        if (!explicitAnchorStep)
        {
            stableDisplayConfig.anchorStepPixels =
                std::max(stableDisplayConfig.anchorStepPixels, config.realtimeAnchorStepPixels);
        }
        if (!explicitStableMinAnchors)
        {
            stableDisplayConfig.stableContourMinAnchors =
                std::min(stableDisplayConfig.stableContourMinAnchors, config.realtimeStableContourMinAnchors);
        }
        if (!explicitDepthSlice)
        {
            config.depthSliceMm = std::max(config.depthSliceMm, config.realtimeDepthSliceMm);
        }
        config.pclFrameInterval = std::max(config.pclFrameInterval, config.realtimePclFrameInterval);
        if (config.realtimeDisablePcl)
        {
            config.pclClustering = false;
        }
        if (config.realtimeDepthOnlyBoundary)
        {
            config.colorSplit = false;
            config.contourDepthConfirmSplit = false;
        }
        if (config.realtimeDisableDepthConfirmSplit)
        {
            config.contourDepthConfirmSplit = false;
        }
    }

    const float depthScale = 0.001f;
    std::cout << "Replay directory enabled: " << replayDir.string()
        << " frames=" << replayFrames.size()
        << " depth_scale=" << depthScale
        << " analysis_export_every_n=" << config.analysisExportEveryN << '\n';

    ProfileCsvWriter profileCsv(parseProfileCsvConfig(argc, argv));
    SegmentationTracker tracker;
    std::vector<ObservationMaterial> pclCandidateCache;
    std::map<uint64_t, StableContourTrackAggregate> stableTrackAggregates;
    IndoorPlaneAnalysis cachedIndoorPlaneAnalysis;
    std::vector<ObservationMaterial> cachedNearPlaneMaterials;
    bool hasIndoorPlaneCache = false;
    bool hasNearPlaneCache = false;
    ClusterMapFrame lastClusterMapFrame;
    cv::Mat lastClusterMapColor;
    bool hasLastClusterMapFrame = false;
    FinalSegmentationFrame lastFinalSegmentationFrame;
    bool hasLastFinalSegmentationFrame = false;
    std::vector<ColorContourRegion> cachedColorContourRegions;
    bool hasColorContourRegionCache = false;
    uint64_t cachedColorContourSourceFrameId = 0;
    uint64_t lastColorContourRefreshRequestFrameId = 0;
    bool hasLastColorContourRefreshRequestFrameId = false;
    cv::Mat cachedColorContourMotionSignature;
    bool refreshColorContourFromUnknownSpikeNextFrame = false;
    bool refreshColorContourFromFarLossNextFrame = false;
    rs2_intrinsics colorIntrinsics{};
    std::unique_ptr<AsyncColorContourRefreshWorker> asyncColorContourWorker;
    if (config.asyncColorContourRefresh)
    {
        asyncColorContourWorker = std::make_unique<AsyncColorContourRefreshWorker>(
            config.asyncColorContourLowPriority);
    }

    for (size_t replayIndex = 0; replayIndex < replayFrames.size(); ++replayIndex)
    {
        const ReplayFramePaths& replayFrame = replayFrames[replayIndex];
        const uint64_t frameId = replayFrame.frameId;
        const auto frameStart = std::chrono::steady_clock::now();
        auto sectionStart = frameStart;
        FrameTimingStats timingStats;
        auto takeSectionMs = [&sectionStart]() {
            const auto sectionEnd = std::chrono::steady_clock::now();
            const double elapsedMs =
                std::chrono::duration<double, std::milli>(sectionEnd - sectionStart).count();
            sectionStart = sectionEnd;
            return elapsedMs;
        };

        cv::Mat colorBgr = cv::imread(replayFrame.colorPath.string(), cv::IMREAD_COLOR);
        if (colorBgr.empty())
        {
            std::cerr << "Failed to read replay color frame: " << replayFrame.colorPath.string() << '\n';
            return 2;
        }
        cv::Mat rawDepth16 = loadReplayDepth16(replayFrame.depthPath);
        cv::Mat depth16 = rawDepth16;
        if (rawDepth16.size() != colorBgr.size())
        {
            cv::resize(rawDepth16, rawDepth16, colorBgr.size(), 0.0, 0.0, cv::INTER_NEAREST);
            depth16 = rawDepth16;
        }
        if (replayIndex == 0)
        {
            colorIntrinsics = makeReplayIntrinsics(colorBgr.size());
        }
        timingStats.frameConvertMs = takeSectionMs();

        cv::Mat segmentationGray = loadReplayGray8(replayFrame.irLeftPath);
        bool edgeSourceIsInfrared = !segmentationGray.empty();
        if (segmentationGray.empty())
        {
            cv::cvtColor(colorBgr, segmentationGray, cv::COLOR_BGR2GRAY);
        }
        if (segmentationGray.size() != colorBgr.size())
        {
            cv::resize(segmentationGray, segmentationGray, colorBgr.size(), 0.0, 0.0, cv::INTER_LINEAR);
        }

        std::vector<ColorContourRegion> colorContourRegions;
        ColorContourRefreshStats colorContourRefreshStats;
        if (asyncColorContourWorker)
        {
            AsyncColorContourRefreshResult asyncResult;
            if (asyncColorContourWorker->takeResult(asyncResult))
            {
                colorContourRefreshStats.refreshed = true;
                colorContourRefreshStats.asyncApplied = true;
                colorContourRefreshStats.asyncWorkerMs = asyncResult.workerMs;
                colorContourRefreshStats.stereoReuseCount =
                    reuseCachedStereoDistances(asyncResult.regions, cachedColorContourRegions);
                if (!asyncResult.refreshRoi.empty())
                {
                    updateRoiStereoRefreshStats(colorContourRefreshStats, asyncResult.regions);
                }
                cachedColorContourRegions = mergeColorContourRoiRefresh(
                    cachedColorContourRegions,
                    std::move(asyncResult.regions),
                    asyncResult.refreshRoi,
                    config,
                    &colorContourRefreshStats.roiPreservedStereoCount);
                cachedColorContourMotionSignature = asyncResult.motionSignature;
                cachedColorContourSourceFrameId = asyncResult.frameId;
                hasColorContourRegionCache = true;
            }
        }
        cv::Mat currentColorContourMotionSignature;
        bool refreshBecauseOfMotion = false;
        if (config.colorContourRefreshOnMotion)
        {
            currentColorContourMotionSignature = makeColorContourMotionSignature(segmentationGray);
            colorContourRefreshStats.motionDeltaPercent = colorContourMotionDeltaPercent(
                currentColorContourMotionSignature,
                cachedColorContourMotionSignature);
            refreshBecauseOfMotion =
                hasColorContourRegionCache &&
                colorContourRefreshStats.motionDeltaPercent >=
                    static_cast<double>(config.colorContourRefreshMotionDeltaPercent);
        }
        const bool refreshBecauseOfStartup = !hasColorContourRegionCache;
        const bool refreshBecauseOfInterval =
            config.colorContourFrameInterval <= 1 ||
            (frameId % static_cast<uint64_t>(config.colorContourFrameInterval) == 0);
        bool refreshColorContourRegions =
            refreshBecauseOfStartup ||
            refreshColorContourFromUnknownSpikeNextFrame ||
            refreshColorContourFromFarLossNextFrame ||
            refreshBecauseOfMotion ||
            refreshBecauseOfInterval;
        if (refreshColorContourRegions && !refreshBecauseOfStartup && config.colorContourRefreshMinGapFrames > 0)
        {
            const uint64_t minGap = static_cast<uint64_t>(config.colorContourRefreshMinGapFrames);
            if (hasLastColorContourRefreshRequestFrameId &&
                frameId >= lastColorContourRefreshRequestFrameId &&
                frameId - lastColorContourRefreshRequestFrameId < minGap)
            {
                refreshColorContourRegions = false;
                colorContourRefreshStats.cooldownSkipped = true;
            }
        }
        if (refreshColorContourRegions)
        {
            lastColorContourRefreshRequestFrameId = frameId;
            hasLastColorContourRefreshRequestFrameId = true;
            colorContourRefreshStats.reasonStartup = refreshBecauseOfStartup;
            colorContourRefreshStats.reasonInterval = refreshBecauseOfInterval && !refreshBecauseOfStartup;
            colorContourRefreshStats.reasonMotion = refreshBecauseOfMotion;
            colorContourRefreshStats.reasonUnknownSpike = refreshColorContourFromUnknownSpikeNextFrame;
            colorContourRefreshStats.reasonFarLoss = refreshColorContourFromFarLossNextFrame;
            const cv::Rect refreshRoi =
                refreshBecauseOfMotion && !refreshBecauseOfStartup
                    ? colorContourMotionRefreshRoi(
                        currentColorContourMotionSignature,
                        cachedColorContourMotionSignature,
                        colorBgr.size(),
                        config,
                        &colorContourRefreshStats.roiCandidatePixels,
                        &colorContourRefreshStats.roiRejectedEmpty,
                        &colorContourRefreshStats.roiRejectedLarge)
                    : cv::Rect();
            colorContourRefreshStats.roiRefresh = !refreshRoi.empty();
            colorContourRefreshStats.roiPixels = refreshRoi.area();
            if (asyncColorContourWorker && hasColorContourRegionCache && !refreshBecauseOfStartup)
            {
                if (!asyncColorContourWorker->hasPendingWork())
                {
                    cv::Mat submitSignature = currentColorContourMotionSignature.empty()
                        ? makeColorContourMotionSignature(segmentationGray)
                        : currentColorContourMotionSignature;
                    colorContourRefreshStats.asyncSubmitted = true;
                    colorContourRefreshStats.asyncDropped = asyncColorContourWorker->submitLatest(
                        frameId,
                        colorBgr,
                        submitSignature,
                        refreshRoi,
                        config);
                }
                colorContourRegions = cachedColorContourRegions;
                colorContourRefreshStats.cacheReused = true;
                colorContourRefreshStats.stereoReuseCount =
                    countStereoDistanceValidRegions(colorContourRegions);
                refreshColorContourFromUnknownSpikeNextFrame = false;
                refreshColorContourFromFarLossNextFrame = false;
            }
            else
            {
                colorContourRefreshStats.refreshed = true;
                colorContourRegions = extractColorContourRegionsInRoi(colorBgr, config, refreshRoi);
                const bool reuseCachedStereoThisRefresh =
                    config.stereoContourReuseCachedOnRefresh && hasColorContourRegionCache;
                if (reuseCachedStereoThisRefresh)
                {
                    colorContourRefreshStats.stereoReuseCount =
                        reuseCachedStereoDistances(colorContourRegions, cachedColorContourRegions);
                }
                if (!reuseCachedStereoThisRefresh &&
                    !colorContourRegions.empty() &&
                    config.stereoContourDistance)
                {
                    cv::Mat leftIrForStereo = loadReplayGray8(replayFrame.irLeftPath);
                    cv::Mat rightIrForStereo = loadReplayGray8(replayFrame.irRightPath);
                    if (!leftIrForStereo.empty() && leftIrForStereo.size() != colorBgr.size())
                    {
                        cv::resize(leftIrForStereo, leftIrForStereo, colorBgr.size(), 0.0, 0.0, cv::INTER_LINEAR);
                    }
                    if (!rightIrForStereo.empty() && rightIrForStereo.size() != colorBgr.size())
                    {
                        cv::resize(rightIrForStereo, rightIrForStereo, colorBgr.size(), 0.0, 0.0, cv::INTER_LINEAR);
                    }
                    estimateStereoContourDistances(
                        colorContourRegions,
                        leftIrForStereo,
                        rightIrForStereo,
                        colorIntrinsics,
                        config);
                }
                if (!refreshRoi.empty() && config.colorContourRefreshRoiDropStereoFailed)
                {
                    dropStereoFailedColorContourRegions(colorContourRegions);
                }
                if (!refreshRoi.empty())
                {
                    updateRoiStereoRefreshStats(colorContourRefreshStats, colorContourRegions);
                }
                cachedColorContourRegions = mergeColorContourRoiRefresh(
                    cachedColorContourRegions,
                    colorContourRegions,
                    refreshRoi,
                    config,
                    &colorContourRefreshStats.roiPreservedStereoCount);
                colorContourRegions = cachedColorContourRegions;
                cachedColorContourSourceFrameId = frameId;
                if (config.colorContourRefreshOnMotion)
                {
                    cachedColorContourMotionSignature = currentColorContourMotionSignature.empty()
                        ? makeColorContourMotionSignature(segmentationGray)
                        : currentColorContourMotionSignature;
                }
                hasColorContourRegionCache = true;
                refreshColorContourFromUnknownSpikeNextFrame = false;
                refreshColorContourFromFarLossNextFrame = false;
            }
        }
        else
        {
            colorContourRefreshStats.cacheReused = true;
            colorContourRegions = cachedColorContourRegions;
            colorContourRefreshStats.stereoReuseCount = countStereoDistanceValidRegions(colorContourRegions);
        }
        colorContourRefreshStats.regionCount = static_cast<int>(colorContourRegions.size());
        colorContourRefreshStats.stereoValidCount = countStereoDistanceValidRegions(colorContourRegions);
        if (asyncColorContourWorker)
        {
            colorContourRefreshStats.asyncPending = asyncColorContourWorker->hasPendingWork();
        }
        if (hasColorContourRegionCache && frameId >= cachedColorContourSourceFrameId)
        {
            colorContourRefreshStats.cacheAgeFrames =
                static_cast<int>(frameId - cachedColorContourSourceFrameId);
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
        candidateMaterials = refineDepthMaterialsWithColorContours(
            candidateMaterials,
            colorContourRegions,
            depth16,
            depthScale,
            colorIntrinsics,
            config,
            frameId);
        timingStats.extractMs = takeSectionMs();

        std::vector<FarDistanceMaterial> farDistanceMaterials;
        if (config.farDistanceIntervals && config.clusterMap)
        {
            farDistanceMaterials =
                extractFarDistanceMaterials(
                    depth16,
                    splitBoundaryMask,
                    segmentationGray,
                    edgeSourceIsInfrared,
                    config,
                    depthScale);
            timingStats.farExtractMs = takeSectionMs();
        }

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

        if (config.qualitySegmentation)
        {
            lastFinalSegmentationFrame =
                buildFinalSegmentationFrame(colorBgr, candidateMaterials, colorContourRegions);
            hasLastFinalSegmentationFrame = !lastFinalSegmentationFrame.idMap.empty();
        }

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
        anchorSupportedCandidates = mergeStablePlaneFragmentsByAnchors(
            anchorSupportedCandidates,
            anchorMask,
            splitBoundaryMask,
            depth16,
            config,
            depthScale,
            colorIntrinsics,
            frameId);
        anchorPointsOnCandidates =
            countStableAnchorPointsOnMaterials(anchorSupportedCandidates, anchorMask, depth16, depthScale);
        timingStats.calibrateMs += takeSectionMs();

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

        std::vector<ObservationMaterial> nearPlaneMaterials;
        if (config.clusterMap && config.nearPlaneDisplay)
        {
            const bool refreshNearPlanes =
                !hasNearPlaneCache ||
                frameId % static_cast<uint64_t>(config.nearPlaneFrameInterval) == 0;
            if (refreshNearPlanes)
            {
                cachedNearPlaneMaterials =
                    extractNearPlaneDisplayMaterials(
                        depth16,
                        splitBoundaryMask,
                        config,
                        depthScale,
                        colorIntrinsics,
                        frameId);
                hasNearPlaneCache = true;
            }
            nearPlaneMaterials = cachedNearPlaneMaterials;
            timingStats.diagnosticsMs += takeSectionMs();
        }

        cv::Mat stableMaskForFrame;
        if (config.clusterMap)
        {
            stableMaskForFrame = buildStableContourMask(colorBgr.size(), stableMaterials);
        }
        IndoorPlaneAnalysis indoorPlaneAnalysis;
        if (config.clusterMap)
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
        timingStats.diagnosticsMs += takeSectionMs();

        if (config.clusterMap)
        {
            lastClusterMapFrame = buildFullFrameClusterMap(
                colorBgr.size(),
                stableMaterials,
                farDistanceMaterials,
                nearPlaneMaterials,
                indoorPlaneAnalysis,
                segmentationGray,
                config);
            lastClusterMapColor = colorBgr.clone();
            hasLastClusterMapFrame = true;
            timingStats.diagnosticsMs += takeSectionMs();
        }

        if (config.colorContourRefreshOnUnknownSpike &&
            hasLastClusterMapFrame &&
            lastClusterMapFrame.unknownPercent >= static_cast<double>(config.colorContourRefreshUnknownPercent))
        {
            refreshColorContourFromUnknownSpikeNextFrame = true;
        }
        if (config.colorContourRefreshOnFarLoss &&
            !colorContourRegions.empty() &&
            countStereoDistanceValidRegions(colorContourRegions) <= 0)
        {
            refreshColorContourFromFarLossNextFrame = true;
        }

        if (config.analysisExportEveryN > 0 &&
            frameId % static_cast<uint64_t>(config.analysisExportEveryN) == 0)
        {
            if (config.clusterMap && hasLastClusterMapFrame)
            {
                writeClusterMapExport(
                    lastClusterMapFrame,
                    lastClusterMapColor,
                    config,
                    static_cast<int64_t>(frameId));
            }
            if (!config.finalSegmentationExportPath.empty() && hasLastFinalSegmentationFrame)
            {
                writeFinalSegmentationExport(
                    lastFinalSegmentationFrame,
                    config,
                    static_cast<int64_t>(frameId));
            }
        }

        const auto frameEnd = std::chrono::steady_clock::now();
        const double frameMs =
            std::chrono::duration<double, std::milli>(frameEnd - frameStart).count();
        profileCsv.write(
            frameId,
            frameMs,
            static_cast<int>(candidateMaterials.size()),
            static_cast<int>(anchorSupportedCandidates.size()),
            static_cast<int>(stableMaterials.size()),
            static_cast<int>(farDistanceMaterials.size()),
            cv::countNonZero(anchorMask),
            anchorPointsOnCandidates,
            timingStats,
            colorContourRefreshStats);
    }

    profileCsv.close();
    if (config.clusterMap && hasLastClusterMapFrame)
    {
        writeClusterMapExport(lastClusterMapFrame, lastClusterMapColor, config);
    }
    if (!config.finalSegmentationExportPath.empty() && hasLastFinalSegmentationFrame)
    {
        writeFinalSegmentationExport(lastFinalSegmentationFrame, config);
    }
    std::cout << "Replay directory complete: processed " << replayFrames.size() << " frames\n";
    return 0;
}

cv::Mat depthUnitsToMillimeters(const cv::Mat& depthUnits16, float depthScale)
{
    cv::Mat depthMm32f;
    depthUnits16.convertTo(depthMm32f, CV_32F, depthScale * 1000.0f);
    cv::Mat depthMm16;
    depthMm32f.convertTo(depthMm16, CV_16UC1);
    return depthMm16;
}

int runCaptureReplayDirectory(
    const std::string& captureDirText,
    int argc,
    char** argv,
    const SegmentationConfig& config,
    int maxFrames)
{
    const std::filesystem::path captureDir = resolveProjectOutputPath(std::filesystem::path(captureDirText), "");
    const std::filesystem::path framesDir = captureDir / "frames";
    std::filesystem::create_directories(framesDir);

    const int captureFrames = std::max(
        1,
        parseIntOptionOrDefault(
            argc,
            argv,
            "--capture-replay-frames=",
            maxFrames > 0 ? maxFrames : 120));
    const int warmupFrames = std::max(
        0,
        parseIntOptionOrDefault(argc, argv, "--capture-replay-warmup=", 30));

    rs2::pipeline pipeline;
    rs2::config rsConfig;
    rsConfig.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    rsConfig.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
    rsConfig.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
    const bool captureRightIr = config.qualitySegmentation && config.stereoContourDistance;
    if (captureRightIr)
    {
        rsConfig.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
    }

    const rs2::pipeline_profile profile = pipeline.start(rsConfig);
    const float depthScale = findDepthScale(profile);
    rs2::align alignToColor(RS2_STREAM_COLOR);
    std::cout << "Capture replay directory: " << captureDir.string()
        << " frames=" << captureFrames
        << " warmup=" << warmupFrames
        << " depth_scale=" << depthScale << '\n';

    for (int i = 0; i < warmupFrames; ++i)
    {
        (void)waitForLatestRgbdFrames(pipeline);
    }

    int captured = 0;
    while (captured < captureFrames)
    {
        rs2::frameset rawFrames = waitForLatestRgbdFrames(pipeline);
        rs2::frameset alignedFrames = alignToColor.process(rawFrames);
        const rs2::video_frame colorFrame = alignedFrames.get_color_frame();
        const rs2::depth_frame depthFrame = alignedFrames.get_depth_frame();
        const rs2::video_frame leftIrFrame = rawFrames.get_infrared_frame(1);
        const rs2::video_frame rightIrFrame = rawFrames.get_infrared_frame(2);
        if (!colorFrame || !depthFrame)
        {
            continue;
        }

        cv::Mat colorBgr = colorFrameToBgr(colorFrame);
        cv::Mat depthMm16 = depthUnitsToMillimeters(depthFrameToMat(depthFrame), depthScale);
        cv::Mat leftIr = leftIrFrame ? videoFrameToGray8(leftIrFrame) : cv::Mat();
        cv::Mat rightIr = rightIrFrame ? videoFrameToGray8(rightIrFrame) : cv::Mat();
        if (!leftIr.empty() && leftIr.size() != colorBgr.size())
        {
            cv::resize(leftIr, leftIr, colorBgr.size(), 0.0, 0.0, cv::INTER_LINEAR);
        }
        if (!rightIr.empty() && rightIr.size() != colorBgr.size())
        {
            cv::resize(rightIr, rightIr, colorBgr.size(), 0.0, 0.0, cv::INTER_LINEAR);
        }

        std::ostringstream prefix;
        prefix << std::setw(6) << std::setfill('0') << captured;
        const std::string stem = prefix.str();
        cv::imwrite((framesDir / (stem + "_color.png")).string(), colorBgr);
        cv::imwrite((framesDir / (stem + "_depth16.png")).string(), depthMm16);
        if (!leftIr.empty())
        {
            cv::imwrite((framesDir / (stem + "_ir_left.png")).string(), leftIr);
        }
        if (!rightIr.empty())
        {
            cv::imwrite((framesDir / (stem + "_ir_right.png")).string(), rightIr);
        }
        ++captured;
    }

    std::ofstream manifest(captureDir / "case_manifest.json", std::ios::out | std::ios::trunc);
    manifest << "{\n";
    manifest << "  \"case_id\": \"" << captureDir.filename().string() << "\",\n";
    manifest << "  \"frame_count\": " << captured << ",\n";
    manifest << "  \"format\": \"d455_directory_replay_v1\",\n";
    manifest << "  \"depth_unit\": \"millimeter_uint16\",\n";
    manifest << "  \"color_resolution\": [640, 480],\n";
    manifest << "  \"depth_resolution\": [640, 480],\n";
    manifest << "  \"ir_left_present\": true,\n";
    manifest << "  \"ir_right_present\": " << (captureRightIr ? "true" : "false") << ",\n";
    manifest << "  \"notes\": \"Fill expected scene notes before using this case for regression.\"\n";
    manifest << "}\n";
    manifest.close();

    pipeline.stop();
    std::cout << "Capture replay complete: wrote " << captured << " frames to " << captureDir.string() << '\n';
    return 0;
}
}

int main(int argc, char** argv)
{
    SegmentationConfig config = parseConfig(argc, argv);
    PoseReadConfig poseConfig = parsePoseReadConfig(argc, argv);
    ImuGravityCheckConfig imuGravityConfig = parseImuGravityCheckConfig(argc, argv);
    MotionDiagnosticsConfig motionConfig = parseMotionDiagnosticsConfig(argc, argv);
    const bool explicitPoseRead = hasFlag(argc, argv, "--pose-read");
    const bool explicitNoPoseRead = hasFlag(argc, argv, "--no-pose-read");
    if (motionConfig.enabled && !poseConfig.enabled && !explicitNoPoseRead)
    {
        poseConfig.enabled = true;
        if (!explicitPoseRead && !hasFlag(argc, argv, "--pose-overlay"))
        {
            poseConfig.overlay = false;
        }
    }
    const int maxFrames = std::max(0, parseIntOptionOrDefault(argc, argv, "--max-frames=", 0));
    const bool displayEnabled = !hasFlag(argc, argv, "--no-display");
    cv::utils::logging::setLogLevel(cv::utils::logging::LOG_LEVEL_WARNING);
    printUsage();
    if (hasFlag(argc, argv, "--help") || hasFlag(argc, argv, "-h"))
    {
        return 0;
    }
    std::string replayDirPath;
    std::string captureReplayDirPath;
    for (int i = 1; i < argc; ++i)
    {
        parseStringOption(argv[i], "--replay-dir=", replayDirPath);
        parseStringOption(argv[i], "--capture-replay-dir=", captureReplayDirPath);
    }
    if (!replayDirPath.empty())
    {
        return runReplayDirectory(replayDirPath, argc, argv, config, maxFrames);
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
        if (!captureReplayDirPath.empty())
        {
            return runCaptureReplayDirectory(captureReplayDirPath, argc, argv, config, maxFrames);
        }

        if (imuGravityConfig.enabled)
        {
            return runImuGravityCheck(devices, imuGravityConfig);
        }

        bool poseAccelStreamEnabled = false;
        bool poseGyroStreamEnabled = false;
        if (poseConfig.enabled)
        {
            poseAccelStreamEnabled = deviceListSupportsStream(devices, RS2_STREAM_ACCEL);
            poseGyroStreamEnabled = deviceListSupportsStream(devices, RS2_STREAM_GYRO);
            if (!poseAccelStreamEnabled && !poseGyroStreamEnabled)
            {
                std::cerr << "Pose read disabled: no accel/gyro motion stream found on connected device.\n";
                poseConfig.enabled = false;
            }
        }

        rs2::pipeline pipeline;
        rs2::config rsConfig;
        rsConfig.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        rsConfig.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        rsConfig.enable_stream(RS2_STREAM_INFRARED, 1, 640, 480, RS2_FORMAT_Y8, 30);
        if (config.qualitySegmentation && config.stereoContourDistance)
        {
            rsConfig.enable_stream(RS2_STREAM_INFRARED, 2, 640, 480, RS2_FORMAT_Y8, 30);
        }
        if (poseConfig.enabled && poseAccelStreamEnabled)
        {
            rsConfig.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        }
        if (poseConfig.enabled && poseGyroStreamEnabled)
        {
            rsConfig.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        }
        if (poseConfig.enabled)
        {
            std::cout << "Pose read enabled: accel=" << (poseAccelStreamEnabled ? 1 : 0)
                << " gyro=" << (poseGyroStreamEnabled ? 1 : 0)
                << " smooth_percent=" << poseConfig.smoothPercent
                << " overlay=" << (poseConfig.overlay ? 1 : 0)
                << " gravity_line=" << (poseConfig.gravityLine ? 1 : 0) << '\n';
        }
        if (motionConfig.enabled)
        {
            std::cout << "Motion diagnostics enabled: overlay=" << (motionConfig.overlay ? 1 : 0)
                << " visual_downscale_width=" << motionConfig.visualDownscaleWidthPixels
                << " visual_move_px=" << fixedNumber(motionConfig.visualMoveMilliPixels / 1000.0, 2)
                << " visual_shake_px=" << fixedNumber(motionConfig.visualShakeMilliPixels / 1000.0, 2)
                << " gyro_move_radps=" << fixedNumber(motionConfig.gyroMoveMilliRadps / 1000.0, 3)
                << " gyro_shake_radps=" << fixedNumber(motionConfig.gyroShakeMilliRadps / 1000.0, 3)
                << '\n';
        }

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
        RgbDepthAccuracyConfig stableDisplayConfig = parseRgbDepthAccuracyConfig(argc, argv);
        stableDisplayConfig.stableContourVideo = true;
        if (config.realtime30)
        {
            const bool explicitAnchorStep =
                hasOptionPrefix(argc, argv, "--rgb-depth-anchor-step-px=");
            const bool explicitStableMinAnchors =
                hasOptionPrefix(argc, argv, "--stable-contour-min-anchors=");
            const bool explicitDepthSlice =
                hasOptionPrefix(argc, argv, "--depth-slice-mm=");
            if (!explicitAnchorStep)
            {
                stableDisplayConfig.anchorStepPixels =
                    std::max(stableDisplayConfig.anchorStepPixels, config.realtimeAnchorStepPixels);
            }
            if (!explicitStableMinAnchors)
            {
                stableDisplayConfig.stableContourMinAnchors =
                    std::min(stableDisplayConfig.stableContourMinAnchors, config.realtimeStableContourMinAnchors);
            }
            if (!explicitDepthSlice)
            {
                config.depthSliceMm = std::max(config.depthSliceMm, config.realtimeDepthSliceMm);
            }
            config.pclFrameInterval = std::max(config.pclFrameInterval, config.realtimePclFrameInterval);
            if (config.realtimeDisablePcl)
            {
                config.pclClustering = false;
            }
            if (config.realtimeDepthOnlyBoundary)
            {
                config.colorSplit = false;
                config.contourDepthConfirmSplit = false;
            }
            if (config.realtimeDisableDepthConfirmSplit)
            {
                config.contourDepthConfirmSplit = false;
            }
            std::cout << "Realtime 30ms mode: fast_depth_post="
                << ((config.realtimeFastDepthPost && config.realtime30 && !config.realtimeSkipDepthPost) ? 1 : 0)
                << " skip_depth_post=" << (config.realtimeSkipDepthPost ? 1 : 0)
                << " lite_visualization=" << (config.realtimeLiteVisualization ? 1 : 0)
                << " pcl=" << (config.pclClustering ? 1 : 0)
                << " color_split=" << (config.colorSplit ? 1 : 0)
                << " depth_confirm_split=" << (config.contourDepthConfirmSplit ? 1 : 0)
                << " depth_slice_mm=" << config.depthSliceMm
                << " anchor_step=" << stableDisplayConfig.anchorStepPixels
                << " min_anchors=" << stableDisplayConfig.stableContourMinAnchors << '\n';
        }
        const bool skipDepthPost = config.realtime30 && config.realtimeSkipDepthPost;
        DepthPostProcessor depthPostProcessor(
            config.realtime30 && config.realtimeFastDepthPost && !skipDepthPost);
        AcceptanceMetricsConfig acceptanceConfig = parseAcceptanceMetricsConfig(argc, argv);
        ProfileCsvConfig profileCsvConfig = parseProfileCsvConfig(argc, argv);
        const bool explicitRecordingOption =
            hasFlag(argc, argv, "--record-video") ||
            hasOptionPrefix(argc, argv, "--record-video=");
        VideoRecordingConfig recordingConfig =
            parseVideoRecordingConfig(argc, argv, acceptanceConfig.enabled || explicitRecordingOption);
        if (!displayEnabled && !recordingConfig.commandControlExplicit)
        {
            recordingConfig.commandControl = false;
        }
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

        int nonPreciseOwnershipTrackbar =
            std::clamp(config.nonPreciseColorOwnershipMinPercent, 1, 100);
        if (displayEnabled)
        {
            cv::namedWindow(kObservationDashboardWindow, cv::WINDOW_AUTOSIZE);
            cv::createTrackbar(
                "Color owner %",
                kObservationDashboardWindow,
                nullptr,
                100);
            cv::setTrackbarPos(
                "Color owner %",
                kObservationDashboardWindow,
                nonPreciseOwnershipTrackbar);
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
        std::vector<ObservationMaterial> cachedNearPlaneMaterials;
        bool hasIndoorPlaneCache = false;
        bool hasNearPlaneCache = false;
        bool hasLockedOutputCrop = false;
        cv::Rect lockedOutputCrop;
        cv::Size lockedOutputPanelSize;
        const bool needsViews = displayEnabled || recordingConfig.enabled || recordingConfig.commandControl;

        VideoRecorder videoRecorder(recordingConfig);
        if (recordingConfig.commandControl)
        {
            std::cout << "Runtime recording commands: press r to start, s to stop, q/Esc to exit."
                << " fps=" << recordingConfig.fps
                << " every=" << recordingConfig.everyN
                << " scale=" << recordingConfig.scalePercent << "%\n";
        }
        AcceptanceMetricsWriter acceptanceMetrics(acceptanceConfig, poseConfig.enabled, motionConfig.enabled);
        ProfileCsvWriter profileCsv(profileCsvConfig);
        PoseReader poseReader(poseConfig, poseAccelStreamEnabled, poseGyroStreamEnabled);
        MotionDiagnostics motionDiagnostics(motionConfig);
        ClusterMapFrame lastClusterMapFrame;
        cv::Mat lastClusterMapColor;
        bool hasLastClusterMapFrame = false;
        FinalSegmentationFrame lastFinalSegmentationFrame;
        bool hasLastFinalSegmentationFrame = false;
        std::vector<ColorContourRegion> cachedColorContourRegions;
        bool hasColorContourRegionCache = false;
        uint64_t cachedColorContourSourceFrameId = 0;
        uint64_t lastColorContourRefreshRequestFrameId = 0;
        bool hasLastColorContourRefreshRequestFrameId = false;
        cv::Mat cachedColorContourMotionSignature;
        bool refreshColorContourFromUnknownSpikeNextFrame = false;
        bool refreshColorContourFromFarLossNextFrame = false;
        std::unique_ptr<AsyncColorContourRefreshWorker> asyncColorContourWorker;
        if (config.asyncColorContourRefresh)
        {
            asyncColorContourWorker = std::make_unique<AsyncColorContourRefreshWorker>(
                config.asyncColorContourLowPriority);
        }
        uint64_t frameId = 0;
        while (true)
        {
            const auto captureWaitStart = std::chrono::steady_clock::now();
            rs2::frameset rawFrames = waitForLatestRgbdFrames(pipeline);
            poseReader.update(rawFrames);
            if (!rawFrames.get_color_frame() || !rawFrames.get_depth_frame())
            {
                continue;
            }

            rs2::frameset frames = alignToColor.process(rawFrames);

            const rs2::video_frame colorFrame = frames.get_color_frame();
            const rs2::depth_frame depthFrame = frames.get_depth_frame();
            if (!colorFrame || !depthFrame)
            {
                continue;
            }

            const auto frameStart = std::chrono::steady_clock::now();
            FrameTimingStats timingStats;
            timingStats.captureWaitAlignMs =
                std::chrono::duration<double, std::milli>(frameStart - captureWaitStart).count();
            auto sectionStart = frameStart;
            auto takeSectionMs = [&sectionStart]()
            {
                const auto sectionEnd = std::chrono::steady_clock::now();
                const double elapsedMs =
                    std::chrono::duration<double, std::milli>(sectionEnd - sectionStart).count();
                sectionStart = sectionEnd;
                return elapsedMs;
            };
            const rs2::depth_frame filteredDepth = skipDepthPost
                ? depthFrame
                : depthPostProcessor.process(depthFrame);
            timingStats.depthPostMs = takeSectionMs();
            cv::Mat colorBgr = colorFrameToBgr(colorFrame);
            cv::Mat rawDepth16 = depthFrameToMat(depthFrame);
            cv::Mat depth16 = skipDepthPost ? rawDepth16 : depthFrameToMat(filteredDepth);

            if (rawDepth16.size() != colorBgr.size())
            {
                cv::resize(rawDepth16, rawDepth16, colorBgr.size(), 0.0, 0.0, cv::INTER_NEAREST);
            }
            if (depth16.size() != colorBgr.size())
            {
                cv::resize(depth16, depth16, colorBgr.size(), 0.0, 0.0, cv::INTER_NEAREST);
            }
            if (displayEnabled)
            {
                nonPreciseOwnershipTrackbar = cv::getTrackbarPos(
                    "Color owner %",
                    kObservationDashboardWindow);
                config.nonPreciseColorOwnershipMinPercent =
                    std::clamp(nonPreciseOwnershipTrackbar, 1, 100);
            }
            timingStats.frameConvertMs = takeSectionMs();

            motionDiagnostics.update(colorBgr, poseReader.state());
            timingStats.motionMs = takeSectionMs();

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

            std::vector<ColorContourRegion> colorContourRegions;
            ColorContourRefreshStats colorContourRefreshStats;
            if (asyncColorContourWorker)
            {
                AsyncColorContourRefreshResult asyncResult;
                if (asyncColorContourWorker->takeResult(asyncResult))
                {
                    colorContourRefreshStats.refreshed = true;
                    colorContourRefreshStats.asyncApplied = true;
                    colorContourRefreshStats.asyncWorkerMs = asyncResult.workerMs;
                    colorContourRefreshStats.stereoReuseCount =
                        reuseCachedStereoDistances(asyncResult.regions, cachedColorContourRegions);
                    if (!asyncResult.refreshRoi.empty())
                    {
                        updateRoiStereoRefreshStats(colorContourRefreshStats, asyncResult.regions);
                    }
                    cachedColorContourRegions = mergeColorContourRoiRefresh(
                        cachedColorContourRegions,
                        std::move(asyncResult.regions),
                        asyncResult.refreshRoi,
                        config,
                        &colorContourRefreshStats.roiPreservedStereoCount);
                    cachedColorContourMotionSignature = asyncResult.motionSignature;
                    cachedColorContourSourceFrameId = asyncResult.frameId;
                    hasColorContourRegionCache = true;
                }
            }
            cv::Mat currentColorContourMotionSignature;
            bool refreshBecauseOfMotion = false;
            if (config.colorContourRefreshOnMotion)
            {
                currentColorContourMotionSignature = makeColorContourMotionSignature(segmentationGray);
                colorContourRefreshStats.motionDeltaPercent = colorContourMotionDeltaPercent(
                    currentColorContourMotionSignature,
                    cachedColorContourMotionSignature);
                refreshBecauseOfMotion =
                    hasColorContourRegionCache &&
                    colorContourRefreshStats.motionDeltaPercent >=
                        static_cast<double>(config.colorContourRefreshMotionDeltaPercent);
            }
            const bool refreshBecauseOfStartup = !hasColorContourRegionCache;
            const bool refreshBecauseOfInterval =
                config.colorContourFrameInterval <= 1 ||
                (frameId % static_cast<uint64_t>(config.colorContourFrameInterval) == 0);
            bool refreshColorContourRegions =
                refreshBecauseOfStartup ||
                refreshColorContourFromUnknownSpikeNextFrame ||
                refreshColorContourFromFarLossNextFrame ||
                refreshBecauseOfMotion ||
                refreshBecauseOfInterval;
            if (refreshColorContourRegions && !refreshBecauseOfStartup && config.colorContourRefreshMinGapFrames > 0)
            {
                const uint64_t minGap = static_cast<uint64_t>(config.colorContourRefreshMinGapFrames);
                if (hasLastColorContourRefreshRequestFrameId &&
                    frameId >= lastColorContourRefreshRequestFrameId &&
                    frameId - lastColorContourRefreshRequestFrameId < minGap)
                {
                    refreshColorContourRegions = false;
                    colorContourRefreshStats.cooldownSkipped = true;
                }
            }
            if (refreshColorContourRegions)
            {
                lastColorContourRefreshRequestFrameId = frameId;
                hasLastColorContourRefreshRequestFrameId = true;
                colorContourRefreshStats.reasonStartup = refreshBecauseOfStartup;
                colorContourRefreshStats.reasonInterval = refreshBecauseOfInterval && !refreshBecauseOfStartup;
                colorContourRefreshStats.reasonMotion = refreshBecauseOfMotion;
                colorContourRefreshStats.reasonUnknownSpike = refreshColorContourFromUnknownSpikeNextFrame;
                colorContourRefreshStats.reasonFarLoss = refreshColorContourFromFarLossNextFrame;
                const cv::Rect refreshRoi =
                    refreshBecauseOfMotion && !refreshBecauseOfStartup
                        ? colorContourMotionRefreshRoi(
                            currentColorContourMotionSignature,
                            cachedColorContourMotionSignature,
                            colorBgr.size(),
                            config,
                            &colorContourRefreshStats.roiCandidatePixels,
                            &colorContourRefreshStats.roiRejectedEmpty,
                            &colorContourRefreshStats.roiRejectedLarge)
                        : cv::Rect();
                colorContourRefreshStats.roiRefresh = !refreshRoi.empty();
                colorContourRefreshStats.roiPixels = refreshRoi.area();
                if (asyncColorContourWorker && hasColorContourRegionCache && !refreshBecauseOfStartup)
                {
                    if (!asyncColorContourWorker->hasPendingWork())
                    {
                        cv::Mat submitSignature = currentColorContourMotionSignature.empty()
                            ? makeColorContourMotionSignature(segmentationGray)
                            : currentColorContourMotionSignature;
                        colorContourRefreshStats.asyncSubmitted = true;
                        colorContourRefreshStats.asyncDropped = asyncColorContourWorker->submitLatest(
                            frameId,
                            colorBgr,
                            submitSignature,
                            refreshRoi,
                            config);
                    }
                    colorContourRegions = cachedColorContourRegions;
                    colorContourRefreshStats.cacheReused = true;
                    colorContourRefreshStats.stereoReuseCount =
                        countStereoDistanceValidRegions(colorContourRegions);
                    refreshColorContourFromUnknownSpikeNextFrame = false;
                    refreshColorContourFromFarLossNextFrame = false;
                }
                else
                {
                    colorContourRefreshStats.refreshed = true;
                    colorContourRegions = extractColorContourRegionsInRoi(colorBgr, config, refreshRoi);
                    const bool reuseCachedStereoThisRefresh =
                        config.stereoContourReuseCachedOnRefresh && hasColorContourRegionCache;
                    if (reuseCachedStereoThisRefresh)
                    {
                        colorContourRefreshStats.stereoReuseCount =
                            reuseCachedStereoDistances(colorContourRegions, cachedColorContourRegions);
                    }
                    if (!reuseCachedStereoThisRefresh &&
                        !colorContourRegions.empty() &&
                        config.stereoContourDistance)
                    {
                        cv::Mat leftIrForStereo;
                        cv::Mat rightIrForStereo;
                        const rs2::video_frame rawLeftIr = rawFrames.get_infrared_frame(1);
                        const rs2::video_frame rawRightIr = rawFrames.get_infrared_frame(2);
                        if (rawLeftIr)
                        {
                            leftIrForStereo = videoFrameToGray8(rawLeftIr);
                        }
                        if (rawRightIr)
                        {
                            rightIrForStereo = videoFrameToGray8(rawRightIr);
                        }
                        if (!leftIrForStereo.empty() && leftIrForStereo.size() != colorBgr.size())
                        {
                            cv::resize(leftIrForStereo, leftIrForStereo, colorBgr.size(), 0.0, 0.0, cv::INTER_LINEAR);
                        }
                        if (!rightIrForStereo.empty() && rightIrForStereo.size() != colorBgr.size())
                        {
                            cv::resize(rightIrForStereo, rightIrForStereo, colorBgr.size(), 0.0, 0.0, cv::INTER_LINEAR);
                        }
                        estimateStereoContourDistances(
                            colorContourRegions,
                            leftIrForStereo,
                            rightIrForStereo,
                            colorIntrinsics,
                            config);
                    }
                    if (!refreshRoi.empty() && config.colorContourRefreshRoiDropStereoFailed)
                    {
                        dropStereoFailedColorContourRegions(colorContourRegions);
                    }
                    if (!refreshRoi.empty())
                    {
                        updateRoiStereoRefreshStats(colorContourRefreshStats, colorContourRegions);
                    }
                    cachedColorContourRegions = mergeColorContourRoiRefresh(
                        cachedColorContourRegions,
                        colorContourRegions,
                        refreshRoi,
                        config,
                        &colorContourRefreshStats.roiPreservedStereoCount);
                    colorContourRegions = cachedColorContourRegions;
                    cachedColorContourSourceFrameId = frameId;
                    if (config.colorContourRefreshOnMotion)
                    {
                        cachedColorContourMotionSignature = currentColorContourMotionSignature.empty()
                            ? makeColorContourMotionSignature(segmentationGray)
                            : currentColorContourMotionSignature;
                    }
                    hasColorContourRegionCache = true;
                    refreshColorContourFromUnknownSpikeNextFrame = false;
                    refreshColorContourFromFarLossNextFrame = false;
                }
            }
            else
            {
                colorContourRefreshStats.cacheReused = true;
                colorContourRegions = cachedColorContourRegions;
                colorContourRefreshStats.stereoReuseCount = countStereoDistanceValidRegions(colorContourRegions);
            }
            colorContourRefreshStats.regionCount = static_cast<int>(colorContourRegions.size());
            colorContourRefreshStats.stereoValidCount = countStereoDistanceValidRegions(colorContourRegions);
            if (asyncColorContourWorker)
            {
                colorContourRefreshStats.asyncPending = asyncColorContourWorker->hasPendingWork();
            }
            if (hasColorContourRegionCache && frameId >= cachedColorContourSourceFrameId)
            {
                colorContourRefreshStats.cacheAgeFrames =
                    static_cast<int>(frameId - cachedColorContourSourceFrameId);
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
            candidateMaterials = refineDepthMaterialsWithColorContours(
                candidateMaterials,
                colorContourRegions,
                depth16,
                depthScale,
                colorIntrinsics,
                config,
                frameId);
            timingStats.extractMs = takeSectionMs();
            std::vector<FarDistanceMaterial> farDistanceMaterials;
            if (config.farDistanceIntervals && (needsViews || acceptanceConfig.enabled || config.clusterMap))
            {
                farDistanceMaterials =
                    extractFarDistanceMaterials(
                        depth16,
                        splitBoundaryMask,
                        segmentationGray,
                        edgeSourceIsInfrared,
                        config,
                        depthScale);
                timingStats.farExtractMs = takeSectionMs();
            }
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

            if (config.qualitySegmentation)
            {
                lastFinalSegmentationFrame =
                    buildFinalSegmentationFrame(colorBgr, candidateMaterials, colorContourRegions);
                hasLastFinalSegmentationFrame = !lastFinalSegmentationFrame.idMap.empty();
            }

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
            anchorSupportedCandidates = mergeStablePlaneFragmentsByAnchors(
                anchorSupportedCandidates,
                anchorMask,
                splitBoundaryMask,
                depth16,
                config,
                depthScale,
                colorIntrinsics,
                frameId);
            anchorPointsOnCandidates =
                countStableAnchorPointsOnMaterials(anchorSupportedCandidates, anchorMask, depth16, depthScale);
            timingStats.calibrateMs += takeSectionMs();

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

            std::vector<ObservationMaterial> nearPlaneMaterials;
            if ((needsViews || config.clusterMap) && config.nearPlaneDisplay)
            {
                const bool refreshNearPlanes =
                    !hasNearPlaneCache ||
                    frameId % static_cast<uint64_t>(config.nearPlaneFrameInterval) == 0;
                if (refreshNearPlanes)
                {
                    cachedNearPlaneMaterials =
                        extractNearPlaneDisplayMaterials(
                            depth16,
                            splitBoundaryMask,
                            config,
                            depthScale,
                            colorIntrinsics,
                            frameId);
                    hasNearPlaneCache = true;
                }
                nearPlaneMaterials = cachedNearPlaneMaterials;
                timingStats.diagnosticsMs += takeSectionMs();
            }

            cv::Mat stableMaskForFrame;
            if (config.indoorPlaneDiagnostics || acceptanceConfig.enabled || config.clusterMap)
            {
                stableMaskForFrame = buildStableContourMask(colorBgr.size(), stableMaterials);
            }
            IndoorPlaneAnalysis indoorPlaneAnalysis;
            if (config.indoorPlaneDiagnostics || config.clusterMap)
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
            timingStats.diagnosticsMs += takeSectionMs();

            if (config.clusterMap)
            {
                lastClusterMapFrame = buildFullFrameClusterMap(
                    colorBgr.size(),
                    stableMaterials,
                    farDistanceMaterials,
                    nearPlaneMaterials,
                    indoorPlaneAnalysis,
                    segmentationGray,
                    config);
                lastClusterMapColor = colorBgr.clone();
                hasLastClusterMapFrame = true;
                timingStats.diagnosticsMs += takeSectionMs();
            }
            if (config.colorContourRefreshOnUnknownSpike &&
                hasLastClusterMapFrame &&
                lastClusterMapFrame.unknownPercent >= static_cast<double>(config.colorContourRefreshUnknownPercent))
            {
                refreshColorContourFromUnknownSpikeNextFrame = true;
            }
            if (config.colorContourRefreshOnFarLoss &&
                !colorContourRegions.empty() &&
                countStereoDistanceValidRegions(colorContourRegions) <= 0)
            {
                refreshColorContourFromFarLossNextFrame = true;
            }
            if (config.analysisExportEveryN > 0 &&
                frameId % static_cast<uint64_t>(config.analysisExportEveryN) == 0)
            {
                if (config.clusterMap && hasLastClusterMapFrame)
                {
                    writeClusterMapExport(
                        lastClusterMapFrame,
                        lastClusterMapColor,
                        config,
                        static_cast<int64_t>(frameId));
                }
                if (!config.finalSegmentationExportPath.empty() && hasLastFinalSegmentationFrame)
                {
                    writeFinalSegmentationExport(
                        lastFinalSegmentationFrame,
                        config,
                        static_cast<int64_t>(frameId));
                }
            }
            ColorContourCompletionStats completionStats;
            cv::Mat segmentedView;
            cv::Mat mosaicView;
            cv::Mat outsideColorView;
            cv::Mat contourSegmentationView;
            cv::Mat boundaryDiagnosticsView;
            cv::Mat indoorPlaneDiagnosticsView;
            if (needsViews)
            {
                cv::Mat segmentationBgr;
                cv::cvtColor(segmentationGray, segmentationBgr, cv::COLOR_GRAY2BGR);
                if (config.realtime30 && config.realtimeLiteVisualization)
                {
                    segmentedView = buildRealtimeStableContourVisualization(
                        segmentationBgr,
                        cueSelectedCandidates,
                        stableMaterials,
                        static_cast<int>(frameId + 1));
                }
                else
                {
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
                }
                if (config.cueSelection)
                {
                    drawCueSelectionOverlay(segmentedView, cueSummary);
                }
                drawFarDistanceOverlay(segmentedView, farDistanceMaterials, config);
                drawNearPlaneOverlay(segmentedView, nearPlaneMaterials);
                timingStats.renderMs = takeSectionMs();

                const std::vector<ObservationMaterial> displayBaseMaterials =
                    buildCurrentFrameDisplayBaseMaterials(stableMaterials, cueSelectedCandidates, config);
                const std::vector<ObservationMaterial> displayStableMaterials =
                    buildDisplayStableMaterials(
                        colorBgr,
                        anchorMask,
                        depth16,
                        depthScale,
                        displayBaseMaterials,
                        config,
                        completionStats);
                timingStats.completionMs = takeSectionMs();

                const std::vector<ObservationMaterial> mosaicDisplayMaterials =
                    config.extraCandidatesInMosaic
                        ? buildMosaicDisplayMaterials(
                            displayStableMaterials,
                            nearPlaneMaterials,
                            farDistanceMaterials,
                            frameId)
                        : displayStableMaterials;
                const cv::Mat displayStableMask =
                    buildStableContourMask(colorBgr.size(), mosaicDisplayMaterials);
                mosaicView = buildStableContourColorMosaic(colorBgr, mosaicDisplayMaterials, displayStableMask);
                drawColorContourCompletionOverlay(mosaicView, completionStats, config.colorContourCompletion);
                if (config.extraCandidatesInMosaic)
                {
                    drawFarDistanceOverlay(mosaicView, farDistanceMaterials, config);
                    drawNearPlaneOverlay(mosaicView, nearPlaneMaterials);
                }
                outsideColorView = buildOutsideStableContourColorImage(colorBgr, displayStableMask);
                contourSegmentationView =
                    buildOutOfD455PrecisionColorContourView(
                        colorBgr.size(),
                        depth16,
                        depthScale,
                        colorContourRegions,
                        config);
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

            if (poseConfig.enabled &&
                poseConfig.logConsole &&
                frameId % static_cast<uint64_t>(poseConfig.logEveryFrames) == 0)
            {
                std::cout << poseStateSummary(frameId, poseReader.state()) << '\n';
            }
            if (motionConfig.enabled &&
                motionConfig.logConsole &&
                frameId % static_cast<uint64_t>(motionConfig.logEveryFrames) == 0)
            {
                std::cout << motionStateSummary(frameId, motionDiagnostics.state()) << '\n';
            }

            cv::Mat rawView = colorBgr;
            const bool drawPose = poseConfig.enabled && poseConfig.overlay;
            const bool drawGravity = poseConfig.enabled && poseConfig.gravityLine;
            const bool drawMotion = motionConfig.enabled && motionConfig.overlay;
            if (needsViews && (drawPose || drawGravity || drawMotion))
            {
                rawView = colorBgr.clone();
                if (drawGravity)
                {
                    drawGravityDirectionOverlay(rawView, poseReader.state());
                }
                if (drawPose)
                {
                    drawPoseOverlay(rawView, poseReader.state());
                }
                if (drawMotion)
                {
                    drawMotionOverlay(rawView, motionDiagnostics.state(), drawPose ? 70 : 24);
                }
            }

            int key = -1;
            cv::Mat dashboardView;
            if (needsViews && !hasLockedOutputCrop)
            {
                lockedOutputCrop = insetCropRect(
                    computeStereoOverlapCrop(depth16, config, depthScale),
                    config.overlapTrimExtraCropPixels,
                    colorBgr.size());
                lockedOutputPanelSize = scaledPanelSize(colorBgr.size(), config);
                hasLockedOutputCrop = true;
                std::cout << "Locked dashboard crop: x=" << lockedOutputCrop.x
                    << " y=" << lockedOutputCrop.y
                    << " w=" << lockedOutputCrop.width
                    << " h=" << lockedOutputCrop.height
                    << " panel=" << lockedOutputPanelSize.width
                    << "x" << lockedOutputPanelSize.height << '\n';
            }
            const cv::Rect outputCrop = hasLockedOutputCrop
                ? lockedOutputCrop
                : cv::Rect(0, 0, colorBgr.cols, colorBgr.rows);
            const cv::Size outputPanelSize = hasLockedOutputCrop
                ? lockedOutputPanelSize
                : scaledPanelSize(colorBgr.size(), config);
            std::vector<cv::Mat> dashboardViews =
                postProcessViewsForDashboard(
                    std::vector<cv::Mat>{rawView, segmentedView, mosaicView, outsideColorView, contourSegmentationView},
                    outputCrop,
                    outputPanelSize,
                    config);
            if (config.boundaryDiagnostics)
            {
                boundaryDiagnosticsView =
                    postProcessViewForDashboard(boundaryDiagnosticsView, outputCrop, outputPanelSize, config);
            }
            if (config.indoorPlaneDiagnostics)
            {
                indoorPlaneDiagnosticsView =
                    postProcessViewForDashboard(indoorPlaneDiagnosticsView, outputCrop, outputPanelSize, config);
            }
            if (displayEnabled)
            {
                dashboardView = buildTiledFrame(dashboardViews, dashboardColumnCount(static_cast<int>(dashboardViews.size())));
                cv::imshow(kObservationDashboardWindow, dashboardView);
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
            RuntimeCommand runtimeCommand = runtimeCommandFromKey(key);
            if (!recordingConfig.commandControl && runtimeCommand != RuntimeCommand::Quit)
            {
                runtimeCommand = RuntimeCommand::None;
            }
            if (recordingConfig.commandControl)
            {
                const RuntimeCommand consoleCommand = pollConsoleRuntimeCommand();
                if (consoleCommand != RuntimeCommand::None)
                {
                    runtimeCommand = consoleCommand;
                }
                applyRuntimeCommand(runtimeCommand, videoRecorder);
            }
            timingStats.displayMs = takeSectionMs();

            if (videoRecorder.isEnabled())
            {
                std::vector<cv::Mat> recordingViews = dashboardViews;
                if (config.boundaryDiagnostics)
                {
                    recordingViews.push_back(boundaryDiagnosticsView);
                }
                if (config.indoorPlaneDiagnostics)
                {
                    recordingViews.push_back(indoorPlaneDiagnosticsView);
                }
                videoRecorder.write(
                    buildTiledFrame(recordingViews, dashboardColumnCount(static_cast<int>(recordingViews.size()))));
            }
            timingStats.recordMs = takeSectionMs();

            const auto frameEnd = std::chrono::steady_clock::now();
            const double frameMs =
                std::chrono::duration<double, std::milli>(frameEnd - frameStart).count();
            profileCsv.write(
                frameId,
                frameMs,
                static_cast<int>(candidateMaterials.size()),
                static_cast<int>(anchorSupportedCandidates.size()),
                static_cast<int>(stableMaterials.size()),
                static_cast<int>(farDistanceMaterials.size()),
                cv::countNonZero(anchorMask),
                anchorPointsOnCandidates,
                timingStats,
                colorContourRefreshStats);
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
                    farDistanceMaterials,
                    stableMask,
                    boundaryAnalysis,
                    cueSummary,
                    indoorPlaneAnalysis,
                    cv::countNonZero(anchorMask),
                    anchorPointsOnCandidates,
                    completionStats,
                    timingStats,
                    poseReader.state(),
                    motionDiagnostics.state());
            }

            if (runtimeCommand == RuntimeCommand::Quit)
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
        profileCsv.close();
        if (config.clusterMap && hasLastClusterMapFrame)
        {
            writeClusterMapExport(lastClusterMapFrame, lastClusterMapColor, config);
        }
        if (!config.finalSegmentationExportPath.empty() && hasLastFinalSegmentationFrame)
        {
            writeFinalSegmentationExport(lastFinalSegmentationFrame, config);
        }
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
