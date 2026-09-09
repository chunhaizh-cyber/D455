#include "Sensor.h"
#include "SourceTiming.h"
#include <windows.h>
#include <bcrypt.h>
#include <opencv2/imgcodecs.hpp>
#include <array>
#include <chrono>
#include <cmath>
#include <fstream>
#include <iomanip>
#include <set>
#include <sstream>

namespace pcs {
int64_t nowMs() {
    return std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count();
}
std::string utf8(const fs::path& path) {
    const auto s = path.u8string();
    return std::string(s.begin(), s.end());
}
void onlyKeys(const Json& value, std::initializer_list<const char*> allowed) {
    require(value.is_object(), "invalid_argument", "参数必须是对象");
    for (const auto& item : value.items()) {
        bool found = false;
        for (const char* key : allowed) found |= item.key() == key;
        require(found, "unknown_field", "未知字段: " + item.key());
    }
}
int boundedInt(const Json& value, int minimum, int maximum, const std::string& code) {
    require(value.is_number_integer(), code, "参数必须是整数");
    if (value.is_number_unsigned()) {
        const auto n = value.get<uint64_t>();
        require(n <= static_cast<uint64_t>(maximum) && (minimum <= 0 || n >= static_cast<uint64_t>(minimum)), code, "整数参数越界");
        return static_cast<int>(n);
    }
    const auto n = value.get<int64_t>();
    require(n >= minimum && n <= maximum, code, "整数参数越界");
    return static_cast<int>(n);
}
Json parseJson(const std::string& text) {
    std::vector<std::set<std::string>> keys;
    return Json::parse(text, [&keys](int depth, Json::parse_event_t event, Json& value) {
        require(depth <= 32, "invalid_json", "JSON嵌套过深");
        if (event == Json::parse_event_t::object_start) keys.emplace_back();
        if (event == Json::parse_event_t::key)
            require(keys.back().insert(value.get<std::string>()).second, "invalid_json", "重复JSON字段");
        if (event == Json::parse_event_t::object_end) keys.pop_back();
        return true;
    });
}
std::vector<uint8_t> readBytes(const fs::path& path, size_t limit) {
    const auto size = fs::file_size(path);
    require(size <= limit, "resource_limit", "材料过大: " + utf8(path));
    std::vector<uint8_t> bytes(static_cast<size_t>(size));
    std::ifstream file(path, std::ios::binary);
    require(file.is_open(), "io_error", "无法打开材料: " + utf8(path));
    file.read(reinterpret_cast<char*>(bytes.data()), static_cast<std::streamsize>(bytes.size()));
    require(file.good() || (file.eof() && file.gcount() == static_cast<std::streamsize>(size)),
            "io_error", "材料读取不完整");
    return bytes;
}
Json readJson(const fs::path& path, size_t limit) {
    const auto bytes = readBytes(path, limit);
    return parseJson(std::string(bytes.begin(), bytes.end()));
}
void writeJson(const fs::path& path, const Json& value) {
    const std::string text = value.dump(2) + "\n";
    std::ofstream file(path, std::ios::binary | std::ios::trunc);
    require(file.is_open(), "io_error", "无法创建JSON材料");
    file.write(text.data(), static_cast<std::streamsize>(text.size()));
    file.close();
    require(!file.fail(), "io_error", "JSON写入失败");
}
std::string sha256(const std::vector<uint8_t>& bytes) {
    std::array<uint8_t, 32> digest{};
    const NTSTATUS status = BCryptHash(BCRYPT_SHA256_ALG_HANDLE, nullptr, 0,
        const_cast<PUCHAR>(bytes.data()), static_cast<ULONG>(bytes.size()),
        digest.data(), static_cast<ULONG>(digest.size()));
    require(status >= 0, "hash_error", "SHA256失败");
    std::ostringstream out;
    for (const auto b : digest) out << std::hex << std::setw(2) << std::setfill('0') << static_cast<int>(b);
    return out.str();
}
Json intrinsicsJson(const rs2_intrinsics& v) {
    return {{"宽", v.width}, {"高", v.height}, {"焦距X", v.fx}, {"焦距Y", v.fy},
            {"主点X", v.ppx}, {"主点Y", v.ppy}, {"畸变模型", static_cast<int>(v.model)},
            {"畸变系数", std::vector<float>(v.coeffs, v.coeffs + 5)}};
}
rs2_intrinsics parseIntrinsics(const Json& j) {
    onlyKeys(j, {"宽", "高", "焦距X", "焦距Y", "主点X", "主点Y", "畸变模型", "畸变系数"});
    require(j.at("宽").is_number_integer() && j.at("高").is_number_integer(), "invalid_calibration", "尺寸必须是整数");
    rs2_intrinsics v{};
    v.width = boundedInt(j.at("宽"), 1, MaxPixels, "invalid_calibration");
    v.height = boundedInt(j.at("高"), 1, MaxPixels, "invalid_calibration");
    require(v.width > 0 && v.height > 0 && int64_t(v.width) * v.height <= MaxPixels,
            "invalid_calibration", "标定尺寸超限");
    v.fx = j.at("焦距X").get<float>(); v.fy = j.at("焦距Y").get<float>();
    v.ppx = j.at("主点X").get<float>(); v.ppy = j.at("主点Y").get<float>();
    require(j.at("畸变模型").is_number_integer(), "invalid_calibration", "畸变模型必须是整数");
    v.model = static_cast<rs2_distortion>(boundedInt(j.at("畸变模型"), 0, 100, "unsupported_calibration"));
    require(v.model == RS2_DISTORTION_NONE || v.model == RS2_DISTORTION_BROWN_CONRADY || v.model == RS2_DISTORTION_INVERSE_BROWN_CONRADY,
            "unsupported_calibration", "首版仅支持无畸变、Brown-Conrady及Inverse-Brown-Conrady");
    require(std::isfinite(v.fx) && std::isfinite(v.fy) && v.fx > 0 && v.fy > 0 &&
            std::isfinite(v.ppx) && std::isfinite(v.ppy), "invalid_calibration", "内参非法");
    require(j.at("畸变系数").size() == 5, "invalid_calibration", "畸变系数数量错误");
    for (int i = 0; i < 5; ++i) {
        v.coeffs[i] = j.at("畸变系数").at(i).get<float>();
        require(std::isfinite(v.coeffs[i]), "invalid_calibration", "畸变系数非法");
    }
    return v;
}
Json extrinsicsJson(const rs2_extrinsics& v) {
    return {{"旋转列优先", std::vector<float>(v.rotation, v.rotation + 9)},
            {"平移米", std::vector<float>(v.translation, v.translation + 3)}};
}
rs2_extrinsics parseExtrinsics(const Json& j) {
    onlyKeys(j, {"旋转列优先", "平移米"});
    require(j.at("旋转列优先").size() == 9 && j.at("平移米").size() == 3,
            "invalid_calibration", "外参数量错误");
    rs2_extrinsics v{};
    cv::Matx33d matrix;
    for (int i = 0; i < 9; ++i) {
        v.rotation[i] = j.at("旋转列优先").at(i).get<float>();
        require(std::isfinite(v.rotation[i]), "invalid_calibration", "旋转值非法");
        matrix(i % 3, i / 3) = v.rotation[i];
    }
    require(cv::norm(cv::Mat(matrix.t() * matrix - cv::Matx33d::eye())) < 0.002 &&
            std::abs(cv::determinant(cv::Mat(matrix)) - 1.0) < 0.002,
            "invalid_calibration", "外参旋转不是刚体旋转");
    for (int i = 0; i < 3; ++i) {
        v.translation[i] = j.at("平移米").at(i).get<float>();
        require(std::isfinite(v.translation[i]), "invalid_calibration", "平移值非法");
    }
    return v;
}

namespace {
fs::path containedPath(const fs::path& root, const std::string& relative) {
    require(relative.size() <= 1024 && relative.find('\0') == std::string::npos, "invalid_path", "材料路径非法");
    const auto requested = fs::path(std::u8string(relative.begin(), relative.end()));
    require(!requested.empty() && !requested.is_absolute() && !requested.has_root_name(),
            "invalid_path", "材料路径必须相对清单目录");
    const auto base = fs::canonical(root);
    const auto result = fs::canonical(base / requested);
    auto b = base.begin(), r = result.begin();
    for (; b != base.end(); ++b, ++r)
        require(r != result.end() && *b == *r, "invalid_path", "材料路径越出清单目录");
    require(fs::is_regular_file(result), "invalid_path", "材料不是普通文件");
    return result;
}
cv::Mat loadPng(const fs::path& path, int width, int height, int type) {
    const auto bytes = readBytes(path, 16 * 1024 * 1024);
    const std::array<uint8_t, 8> signature{137, 80, 78, 71, 13, 10, 26, 10};
    require(bytes.size() >= 33 && std::equal(signature.begin(), signature.end(), bytes.begin()),
            "invalid_image", "输入必须是PNG");
    auto be32 = [&](size_t offset) {
        return (uint32_t(bytes[offset]) << 24) | (uint32_t(bytes[offset + 1]) << 16) |
               (uint32_t(bytes[offset + 2]) << 8) | bytes[offset + 3];
    };
    require(be32(16) == uint32_t(width) && be32(20) == uint32_t(height),
            "calibration_mismatch", "图像尺寸与标定不同，禁止直接缩放配准");
    const auto image = cv::imdecode(bytes, cv::IMREAD_UNCHANGED);
    require(!image.empty() && image.type() == type, "invalid_image", "图像格式错误");
    return image;
}

class ReplaySource final : public Source {
    fs::path root_;
    Json manifest_;
    RawFrame calibration_;
    size_t index_ = 0;
    uint64_t lastFrame_ = 0;
public:
    explicit ReplaySource(const fs::path& path) : root_(path.parent_path()), manifest_(readJson(path)) {
        onlyKeys(manifest_, {"格式", "材料来源", "设备标识", "深度单位米", "彩图内参", "深度内参", "深度到彩图外参", "帧列表"});
        require(manifest_.at("格式") == "PCS.RawSequence/1", "unsupported_format", "回放格式不支持");
        const auto origin = manifest_.at("材料来源").get<std::string>();
        require(origin == "合成夹具" || origin == "历史回放", "invalid_source", "回放不能标为当前实时观测");
        require(manifest_.at("设备标识").is_string() && manifest_.at("设备标识").get<std::string>().size() <= 128,
                "invalid_source", "设备标识必须是有界字符串");
        calibration_.colorIntrinsics = parseIntrinsics(manifest_.at("彩图内参"));
        calibration_.depthIntrinsics = parseIntrinsics(manifest_.at("深度内参"));
        calibration_.depthToColor = parseExtrinsics(manifest_.at("深度到彩图外参"));
        calibration_.depthScale = manifest_.at("深度单位米").get<float>();
        require(std::isfinite(calibration_.depthScale) && calibration_.depthScale > 0 && calibration_.depthScale <= 1,
                "invalid_calibration", "深度单位非法");
        const auto& frames = manifest_.at("帧列表");
        require(frames.is_array() && !frames.empty() && frames.size() <= 2048, "resource_limit", "帧清单为空或超限");
        uint64_t previous = 0;
        bool first = true;
        for (const auto& frame : frames) {
            onlyKeys(frame, {"彩图", "深度", "源帧号", "彩图时间戳毫秒", "深度时间戳毫秒", "时间域"});
            const auto idText = frame.at("源帧号").get<std::string>();
            require(!idText.empty() && idText.size() <= 20 && idText.find_first_not_of("0123456789") == std::string::npos,
                    "invalid_source", "源帧号必须是十进制字符串");
            const auto id = std::stoull(idText);
            require(first || id > previous, "invalid_source", "源帧号必须严格递增");
            previous = id; first = false;
            for (const auto key : {"彩图时间戳毫秒", "深度时间戳毫秒"})
                require(frame.at(key).is_number() && std::isfinite(frame.at(key).get<double>()),
                        "invalid_source", "时间戳非法");
            require(frame.at("时间域").is_string() && !frame.at("时间域").get<std::string>().empty() &&
                    frame.at("时间域").get<std::string>().size() <= 64, "invalid_source", "时间域缺失或过长");
        }
    }
    RawFrame next() override {
        require(index_ < manifest_.at("帧列表").size(), "end_of_source", "回放结束");
        const auto& item = manifest_.at("帧列表").at(index_);
        RawFrame frame = calibration_;
        frame.colorBgr = loadPng(containedPath(root_, item.at("彩图").get<std::string>()),
            frame.colorIntrinsics.width, frame.colorIntrinsics.height, CV_8UC3);
        frame.depth16 = loadPng(containedPath(root_, item.at("深度").get<std::string>()),
            frame.depthIntrinsics.width, frame.depthIntrinsics.height, CV_16UC1);
        const auto frameNumber = std::stoull(item.at("源帧号").get<std::string>());
        const auto gap = index_ ? frameNumber - lastFrame_ - 1 : 0;
        frame.source = {{"类型", manifest_.at("材料来源")}, {"设备标识", manifest_.at("设备标识")},
            {"源帧号", item.at("源帧号")}, {"彩图源帧号", item.at("源帧号")}, {"深度源帧号", item.at("源帧号")},
            {"彩图时间戳毫秒", item.at("彩图时间戳毫秒")}, {"深度时间戳毫秒", item.at("深度时间戳毫秒")},
            {"彩图时间域", item.at("时间域")}, {"深度时间域", item.at("时间域")},
            {"彩图源帧缺口", gap}, {"深度源帧缺口", gap},
            {"接收Unix毫秒", nowMs()}, {"绝对采集时间已校准", false}};
        lastFrame_ = frameNumber;
        ++index_;
        return frame;
    }
    Json description() const override {
        return {{"来源", manifest_.at("材料来源")}, {"设备标识", manifest_.at("设备标识")},
            {"总帧数", manifest_.at("帧列表").size()}, {"已读帧数", index_},
            {"彩图内参", intrinsicsJson(calibration_.colorIntrinsics)},
            {"深度内参", intrinsicsJson(calibration_.depthIntrinsics)},
            {"深度到彩图外参", extrinsicsJson(calibration_.depthToColor)}, {"深度单位米", calibration_.depthScale}};
    }
};

class CameraSource final : public Source {
    rs2::context context_;
    rs2::pipeline pipeline_{context_};
    bool started_ = false;
    Json info_;
    uint64_t lastColor_ = 0;
    uint64_t lastDepth_ = 0;
    uint64_t startupRejected_ = 0;
    Json lastStartupRejected_ = nullptr;
public:
    explicit CameraSource(const std::string& serial) {
        auto devices = context_.query_devices();
        require(devices.size() > 0, "device_unavailable", "未连接RealSense设备");
        require(!serial.empty() || devices.size() == 1, "ambiguous_device", "存在多个设备，请指定序列号");
        rs2::config config;
        config.enable_device(serial.empty() ? devices[0].get_info(RS2_CAMERA_INFO_SERIAL_NUMBER) : serial);
        config.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
        config.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
        const auto profile = pipeline_.start(config);
        started_ = true;
        try {
            const auto ci = profile.get_stream(RS2_STREAM_COLOR).as<rs2::video_stream_profile>().get_intrinsics();
            const auto di = profile.get_stream(RS2_STREAM_DEPTH).as<rs2::video_stream_profile>().get_intrinsics();
            parseIntrinsics(intrinsicsJson(ci)); parseIntrinsics(intrinsicsJson(di));
            const auto ex = profile.get_stream(RS2_STREAM_DEPTH).get_extrinsics_to(profile.get_stream(RS2_STREAM_COLOR));
            parseExtrinsics(extrinsicsJson(ex));
            info_ = {{"来源", "实时相机"}, {"设备标识", profile.get_device().get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)},
                {"固件", profile.get_device().get_info(RS2_CAMERA_INFO_FIRMWARE_VERSION)},
                {"彩图内参", intrinsicsJson(ci)}, {"深度内参", intrinsicsJson(di)},
                {"深度到彩图外参", extrinsicsJson(ex)},
                {"深度单位米", profile.get_device().first<rs2::depth_sensor>().get_depth_scale()},
                {"宽", 640}, {"高", 480}, {"帧率", 30}, {"预热状态", "未证明稳定"},
                {"启动筛帧", "首帧配对通过前，在单次1000ms采集预算内跳过不相容帧对；不改变50ms门槛"}};
        } catch (...) { pipeline_.stop(); started_ = false; throw; }
    }
    ~CameraSource() override {
        if (started_) { try { pipeline_.stop(); } catch (...) {} }
    }
    RawFrame next() override {
        const auto deadline = std::chrono::steady_clock::now() + std::chrono::milliseconds(1000);
        auto remainingMs = [&]() {
            return static_cast<int>(std::chrono::duration_cast<std::chrono::milliseconds>(
                deadline - std::chrono::steady_clock::now()).count());
        };
        rs2::frameset frames;
        // Only startup may discard incompatible pairs. Established sessions fail explicitly.
        for (;;) {
            const int remaining = remainingMs();
            require(remaining > 0 && pipeline_.try_wait_for_frames(&frames, static_cast<unsigned>(remaining)),
                    "capture_timeout", "采集超时；最近启动拒绝: " + lastStartupRejected_.dump());
            const auto color = frames.get_color_frame();
            const auto depth = frames.get_depth_frame();
            const PairTiming pair{bool(color), bool(depth), color ? color.get_frame_number() : 0,
                depth ? depth.get_frame_number() : 0, color ? color.get_timestamp() : 0,
                depth ? depth.get_timestamp() : 0, color ? int(color.get_frame_timestamp_domain()) : -1,
                depth ? int(depth.get_frame_timestamp_domain()) : -1};
            const auto issue = checkPair(pair, lastColor_, lastDepth_);
            if (issue == PairIssue::None) break;
            const std::string code = issue == PairIssue::MissingStream ? "missing_stream" :
                issue == PairIssue::StaleFrame ? "stale_frame" : "unsynchronized_frame";
            const Json detail = {{"代码", code}, {"彩图源帧号", std::to_string(pair.colorNumber)},
                {"深度源帧号", std::to_string(pair.depthNumber)}, {"彩图时间戳毫秒", pair.colorMs},
                {"深度时间戳毫秒", pair.depthMs},
                {"彩图时间域", color ? rs2_timestamp_domain_to_string(color.get_frame_timestamp_domain()) : "缺流"},
                {"深度时间域", depth ? rs2_timestamp_domain_to_string(depth.get_frame_timestamp_domain()) : "缺流"}};
            if (!lastColor_) { ++startupRejected_; lastStartupRejected_ = detail; }
            require(retryStartupPair(issue, lastColor_ != 0, remainingMs()), code,
                    "帧对被拒绝，未生成观察包: " + detail.dump());
        }
        const auto color = frames.get_color_frame();
        const auto depth = frames.get_depth_frame();
        RawFrame result;
        result.colorBgr = cv::Mat(color.get_height(), color.get_width(), CV_8UC3,
            const_cast<void*>(color.get_data()), color.get_stride_in_bytes()).clone();
        result.depth16 = cv::Mat(depth.get_height(), depth.get_width(), CV_16UC1,
            const_cast<void*>(depth.get_data()), depth.get_stride_in_bytes()).clone();
        result.colorIntrinsics = color.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
        result.depthIntrinsics = depth.get_profile().as<rs2::video_stream_profile>().get_intrinsics();
        result.depthToColor = depth.get_profile().get_extrinsics_to(color.get_profile());
        result.depthScale = info_.at("深度单位米").get<float>();
        const auto colorDomain = color.get_frame_timestamp_domain();
        const auto depthDomain = depth.get_frame_timestamp_domain();
        result.source = {{"类型", "实时相机"}, {"设备标识", info_.at("设备标识")},
            {"源帧号", std::to_string(color.get_frame_number())},
            {"彩图源帧号", std::to_string(color.get_frame_number())}, {"深度源帧号", std::to_string(depth.get_frame_number())},
            {"彩图时间戳毫秒", color.get_timestamp()}, {"深度时间戳毫秒", depth.get_timestamp()},
            {"彩图时间域", rs2_timestamp_domain_to_string(colorDomain)}, {"深度时间域", rs2_timestamp_domain_to_string(depthDomain)},
            {"彩图源帧缺口", lastColor_ ? color.get_frame_number() - lastColor_ - 1 : 0},
            {"深度源帧缺口", lastDepth_ ? depth.get_frame_number() - lastDepth_ - 1 : 0},
            {"接收Unix毫秒", nowMs()}, {"绝对采集时间已校准", false},
            {"启动拒绝帧对累计数", startupRejected_}, {"最近启动拒绝帧对", lastStartupRejected_},
            {"预热状态", "未证明稳定"},
            {"新帧语义", "本会话未交付过的源帧，不保证曝光晚于请求时刻"}};
        lastColor_ = color.get_frame_number(); lastDepth_ = depth.get_frame_number();
        return result;
    }
    Json description() const override {
        auto info = info_;
        info["首帧配对已通过"] = lastColor_ != 0;
        info["启动拒绝帧对累计数"] = startupRejected_;
        info["最近启动拒绝帧对"] = lastStartupRejected_;
        return info;
    }
};
}

Json deviceCapabilities() {
    rs2::context context;
    Json devices = Json::array();
    for (const auto& device : context.query_devices()) {
        devices.push_back({{"名称", device.get_info(RS2_CAMERA_INFO_NAME)},
                           {"序列号", device.get_info(RS2_CAMERA_INFO_SERIAL_NUMBER)}});
    }
    return devices;
}
std::unique_ptr<Source> openSource(const Json& parameters) {
    onlyKeys(parameters, {"来源", "清单", "设备序列号", "宽", "高", "帧率"});
    const auto kind = parameters.at("来源").get<std::string>();
    if (kind == "目录回放") {
        require(parameters.size() == 2 && parameters.contains("清单"), "invalid_argument", "回放只接受来源和清单");
        const auto text = parameters.at("清单").get<std::string>();
        require(!text.empty() && text.size() <= 4096 && text.find('\0') == std::string::npos, "invalid_path", "清单路径非法");
        return std::make_unique<ReplaySource>(fs::absolute(fs::path(std::u8string(text.begin(), text.end()))));
    }
    require(kind == "实时相机", "unsupported_source", "不支持该输入来源");
    require(!parameters.contains("清单"), "invalid_argument", "实时相机不能指定回放清单");
    for (const auto* key : {"宽", "高", "帧率"})
        if (parameters.contains(key)) require(parameters.at(key).is_number_integer(), "invalid_argument", "流配置必须是整数");
    require(boundedInt(parameters.value("宽", Json(640)), 1, MaxPixels, "unsupported_profile") == 640 &&
            boundedInt(parameters.value("高", Json(480)), 1, MaxPixels, "unsupported_profile") == 480 &&
            boundedInt(parameters.value("帧率", Json(30)), 1, 120, "unsupported_profile") == 30,
            "unsupported_profile", "首版相机仅实现640x480@30 RGBD，禁止静默修改请求");
    return std::make_unique<CameraSource>(parameters.value("设备序列号", std::string{}));
}
}
