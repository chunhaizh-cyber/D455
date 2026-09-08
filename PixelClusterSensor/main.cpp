#include "Sensor.h"
#include <windows.h>
#include <bcrypt.h>
#include <algorithm>
#include <array>
#include <chrono>
#include <deque>
#include <iostream>
#include <map>
#include <thread>

namespace pcs {
namespace {
std::string newSession() {
    std::array<uint8_t, 16> bytes{};
    require(BCryptGenRandom(nullptr, bytes.data(), static_cast<ULONG>(bytes.size()), BCRYPT_USE_SYSTEM_PREFERRED_RNG) >= 0,
            "session_error", "无法生成会话标识");
    const char* hex = "0123456789abcdef";
    std::string result;
    for (const auto b : bytes) { result += hex[b >> 4]; result += hex[b & 15]; }
    return result;
}
Json failure(const Json& id, const std::string& code, const std::string& message) {
    return {{"协议", "PCS.Control/1"}, {"请求编号", id}, {"状态", "失败"},
            {"错误", {{"代码", code}, {"说明", message.substr(0, 2048)}}}};
}
Json capabilities() {
    return {{"设备", deviceCapabilities()}, {"输出格式", "PCS.Observation/1"},
        {"支持指令", {"查询设备能力", "查询运行状态", "打开设备", "关闭设备", "读取配置与标定", "设置处理配置",
            "获取单帧观察", "开始连续观察", "停止连续观察", "读取观察结果", "取消请求", "退出"}},
        {"未实现指令", {"设置成像参数", "设置采集配置", "设置关注区域", "取消关注区域", "请求局部复核", "订阅运动信息",
            "取消订阅", "设置输出订阅", "释放观测材料", "开始录制", "停止录制", "重置临时缓存", "恢复设备连接"}},
        {"相机采集规格", {{"宽", 640}, {"高", 480}, {"帧率", 30}, {"流", {"颜色", "深度"}}, {"通过打开设备请求", true}}},
        {"回放输入", "PCS.RawSequence/1，必须包含真实适用标定或明确合成标定"},
        {"数据交付", "单帧同步返回引用；连续观察经读取观察结果拉取，队列上限32"},
        {"取消粒度", "连续任务帧边界；当前单帧计算不抢占"},
        {"控制接收", "仅本机UTF-8 JSONL管道或输入文件；无网络监听"},
        {"资源上限", {{"单请求字节", 65536}, {"幂等记录数", 4096}, {"单图像像素", MaxPixels}, {"单包字节", MaxPacketBytes}}},
        {"体素", false}, {"存在身份裁决", false}, {"历史深度补全", false}};
}

class Service {
    struct Cached { std::string request; Json response; };
    std::map<std::string, Cached> receipts_;
    size_t receiptBytes_ = 0;
    fs::path root_;
    size_t byteLimit_;
    uint64_t packetLimit_;
    std::unique_ptr<Source> source_;
    ProcessingConfig config_;
    uint64_t revision_ = 1, sequence_ = 0, published_ = 0;
    size_t publishedBytes_ = 0;
    std::string session_;
    bool exit_ = false;
    Json lastError_ = nullptr;
    Json lastFrame_ = nullptr;
    struct Task {
        std::string id;
        std::string state = "无任务";
        bool active = false;
        bool latest = false;
        int requested = 0, completed = 0, dropped = 0, intervalMs = 0;
        Json lastDropped = nullptr;
        Json error = nullptr;
        std::chrono::steady_clock::time_point deadline, next;
    } task_;
    std::deque<Json> ready_;

    Json taskStatus() const {
        return {{"任务编号", task_.id}, {"状态", task_.state}, {"请求帧数", task_.requested},
            {"完成帧数", task_.completed}, {"待取结果数", ready_.size()}, {"丢弃未消费引用数", task_.dropped},
            {"最后丢弃输出序号", task_.lastDropped}, {"错误", task_.error}};
    }
    void sessionCheck(const Json& request) const {
        require(source_ != nullptr, "device_closed", "设备或回放未打开");
        require(request.value("会话标识", std::string{}) == session_, "stale_session", "设备会话不匹配");
    }
    Json observe(const std::string& requestId) {
        require(published_ < packetLimit_ && publishedBytes_ < byteLimit_, "storage_quota", "进程观察包存储预算已用尽");
        const RawFrame raw = source_->next();
        if (task_.active && !task_.latest && task_.completed > 0) {
            require(raw.source.value("彩图源帧缺口", uint64_t(0)) == 0 && raw.source.value("深度源帧缺口", uint64_t(0)) == 0,
                    "source_gap", "完整处理任务发现源帧缺口，不能宣称连续观察");
        }
        const auto observation = process(raw, config_);
        const Json control = {{"请求编号", requestId}, {"连续任务", task_.active}};
        const auto result = publish(root_, session_, ++sequence_, raw, observation, config_, revision_, control, byteLimit_ - publishedBytes_);
        publishedBytes_ += result.at("字节数").get<size_t>();
        ++published_;
        lastFrame_ = result;
        return result;
    }
    Json execute(const Json& request) {
        const auto command = request.at("指令").get<std::string>();
        const Json parameters = request.value("参数", Json::object());
        require(parameters.is_object(), "invalid_argument", "参数必须是对象");
        if (command == "查询设备能力") { onlyKeys(parameters, {}); return capabilities(); }
        if (command == "查询运行状态") {
            onlyKeys(parameters, {});
            return {{"设备打开", source_ != nullptr}, {"会话标识", session_}, {"配置版本", std::to_string(revision_)},
                {"任务", taskStatus()}, {"已发布包数", published_}, {"已发布字节", publishedBytes_},
                {"包预算", packetLimit_}, {"字节预算", byteLimit_}, {"最近帧", lastFrame_}, {"最后错误", lastError_}};
        }
        if (command == "打开设备") {
            require(!source_, "device_busy", "先关闭已有设备会话");
            auto source = openSource(parameters);
            auto session = newSession();
            const auto description = source->description();
            source_ = std::move(source); session_ = std::move(session);
            ready_.clear(); task_ = Task{}; lastFrame_ = nullptr;
            return {{"会话标识", session_}, {"配置版本", std::to_string(revision_)}, {"实际配置与标定", description},
                    {"处理配置", config_.json()}};
        }
        if (command == "退出") {
            onlyKeys(parameters, {});
            if (source_) sessionCheck(request);
            stop("取消"); source_.reset(); exit_ = true;
            return {{"已退出", true}, {"已发布材料保留", true}};
        }
        sessionCheck(request);
        if (command == "关闭设备") {
            onlyKeys(parameters, {});
            require(ready_.empty(), "unread_results", "请先读取连续观察结果再关闭设备");
            stop("取消"); source_.reset();
            return {{"已关闭会话", session_}, {"已发布材料保留", true}};
        }
        if (command == "读取配置与标定") {
            onlyKeys(parameters, {});
            return {{"实际配置与标定", source_->description()}, {"处理配置", config_.json()}, {"配置版本", std::to_string(revision_)}};
        }
        if (command == "设置处理配置") {
            require(!task_.active, "task_busy", "先停止连续观察再切换处理配置");
            require(request.value("预期配置版本", std::string{}) == std::to_string(revision_),
                    "stale_config", "必须提供匹配的预期配置版本");
            Json merged = config_.json();
            merged.update(parameters);
            const auto proposed = ProcessingConfig::parse(merged);
            config_ = proposed; ++revision_;
            return {{"处理配置", config_.json()}, {"配置版本", std::to_string(revision_)},
                    {"生效边界", "此命令后开始处理的新观察；不改写已有材料"}};
        }
        if (command == "获取单帧观察") {
            onlyKeys(parameters, {});
            require(!task_.active, "task_busy", "连续观察中请读取观察结果，不另开采集器");
            return observe(request.at("请求编号").get<std::string>());
        }
        if (command == "开始连续观察") {
            onlyKeys(parameters, {"帧数", "模式", "最长秒数", "间隔毫秒"});
            require(!task_.active && ready_.empty(), "task_busy", "旧任务仍在运行或结果尚未取完");
            require(parameters.at("帧数").is_number_integer(), "invalid_argument", "帧数必须是整数");
            for (const auto* key : {"最长秒数", "间隔毫秒"})
                if (parameters.contains(key)) require(parameters.at(key).is_number_integer(), "invalid_argument", "计数参数必须是整数");
            const int frames = boundedInt(parameters.at("帧数"), 1, 10000, "invalid_argument");
            const int seconds = boundedInt(parameters.value("最长秒数", Json(60)), 1, 3600, "invalid_argument");
            const int interval = boundedInt(parameters.value("间隔毫秒", Json(0)), 0, 10000, "invalid_argument");
            const auto mode = parameters.value("模式", std::string("完整处理"));
            require(frames >= 1 && frames <= 10000 && seconds >= 1 && seconds <= 3600 && interval >= 0 && interval <= 10000,
                    "invalid_argument", "连续观察预算越界");
            require(mode == "完整处理" || mode == "实时优先", "invalid_argument", "未知连续观察模式");
            require(published_ + static_cast<uint64_t>(frames) <= packetLimit_, "storage_quota", "请求超过剩余包数量预算");
            task_ = Task{}; task_.active = true; task_.state = "运行中";
            task_.id = request.at("请求编号").get<std::string>(); task_.requested = frames;
            task_.latest = mode == "实时优先"; task_.intervalMs = interval;
            task_.next = std::chrono::steady_clock::now();
            task_.deadline = task_.next + std::chrono::seconds(seconds);
            return {{"任务编号", task_.id}, {"任务状态", "运行中"}, {"模式", mode},
                {"注意", "命令已生效，不代表所请求帧已完成；实时相机源帧缺口另行报告"}};
        }
        if (command == "停止连续观察" || command == "取消请求") {
            onlyKeys(parameters, {"任务编号"});
            require(!task_.id.empty() && parameters.at("任务编号") == task_.id, "unknown_task", "任务编号不匹配");
            stop("取消"); return taskStatus();
        }
        if (command == "读取观察结果") {
            onlyKeys(parameters, {"任务编号", "最多帧数"});
            require(!task_.id.empty() && parameters.at("任务编号") == task_.id, "unknown_task", "任务编号不匹配");
            if (parameters.contains("最多帧数")) require(parameters.at("最多帧数").is_number_integer(), "invalid_argument", "最多帧数必须是整数");
            const int maximum = boundedInt(parameters.value("最多帧数", Json(8)), 1, 8, "invalid_argument");
            require(maximum >= 1 && maximum <= 8, "invalid_argument", "每次最多读取1至8帧");
            Json results = Json::array();
            while (!ready_.empty() && results.size() < static_cast<size_t>(maximum)) {
                results.push_back(ready_.front()); ready_.pop_front();
            }
            return {{"观察结果", results}, {"任务", taskStatus()}};
        }
        throw Error("unsupported_command", "未实现指令，不产生副作用: " + command);
    }
public:
    Service(fs::path root, uint64_t packetLimit, size_t byteLimit)
        : root_(std::move(root)), byteLimit_(byteLimit), packetLimit_(packetLimit) {}
    bool exiting() const { return exit_; }
    void stop(const std::string& state) {
        if (task_.active) { task_.active = false; task_.state = state; }
    }
    Json handle(const std::string& line) {
        Json id = nullptr;
        std::string key, canonical;
        bool reservable = false;
        Json response;
        try {
            const auto request = parseJson(line);
            onlyKeys(request, {"协议", "请求编号", "指令", "参数", "会话标识", "截止Unix毫秒", "预期配置版本"});
            require(request.at("协议") == "PCS.Control/1", "unsupported_protocol", "协议版本不支持");
            key = request.at("请求编号").get<std::string>();
            require(!key.empty() && key.size() <= 128, "invalid_request", "请求编号长度必须为1至128字节");
            id = key; canonical = request.dump();
            const auto cached = receipts_.find(key);
            if (cached != receipts_.end()) {
                require(cached->second.request == canonical, "request_conflict", "同一请求编号不能绑定不同内容");
                return cached->second.response;
            }
            require(receipts_.size() < 4096 && receiptBytes_ + canonical.size() + 65536 < 16 * 1024 * 1024,
                    "request_capacity", "幂等账本预算用尽，请关闭输入结束会话，不自动淘汰重执行");
            reservable = true;
            require(request.at("截止Unix毫秒").is_number_integer(), "invalid_request", "必须提供Unix毫秒截止时间");
            require(request.at("截止Unix毫秒").get<int64_t>() > nowMs(), "expired_request", "请求已过期");
            const auto result = execute(request);
            response = {{"协议", "PCS.Control/1"}, {"请求编号", id}, {"状态", "完成"}, {"结果", result}};
        } catch (const Error& error) {
            response = failure(id, error.code, error.what());
        } catch (const rs2::error& error) {
            response = failure(id, "device_error", error.what());
        } catch (const Json::exception& error) {
            response = failure(id, "invalid_argument", error.what());
        } catch (const std::exception& error) {
            response = failure(id, "execution_error", error.what());
        }
        if (response.at("状态") == "失败") lastError_ = response.at("错误");
        if (reservable) {
            receiptBytes_ += canonical.size() + response.dump().size();
            receipts_.emplace(key, Cached{canonical, response});
        }
        return response;
    }
    void step() {
        if (!task_.active) return;
        const auto now = std::chrono::steady_clock::now();
        if (now >= task_.deadline) {
            task_.error = {{"代码", "task_deadline"}, {"说明", "任务期限已到"}};
            stop("未完成"); return;
        }
        if (now < task_.next || (ready_.size() >= 32 && !task_.latest)) return;
        try {
            auto frame = observe(task_.id);
            if (ready_.size() >= 32) {
                task_.lastDropped = ready_.front().at("输出序号"); ready_.pop_front(); ++task_.dropped;
            }
            ready_.push_back(std::move(frame)); ++task_.completed;
            task_.next = std::chrono::steady_clock::now() + std::chrono::milliseconds(task_.intervalMs);
            if (std::chrono::steady_clock::now() >= task_.deadline) {
                task_.error = {{"代码", "task_deadline"}, {"说明", "帧处理结束时已超期；完成材料保留"}};
                stop("未完成");
            } else if (task_.completed == task_.requested) stop("完成");
        } catch (const Error& error) {
            task_.error = {{"代码", error.code}, {"说明", error.what()}}; stop("失败"); lastError_ = task_.error;
        } catch (const std::exception& error) {
            task_.error = {{"代码", "execution_error"}, {"说明", error.what()}}; stop("失败"); lastError_ = task_.error;
        }
    }
};

int serve(const fs::path& root, uint64_t packetLimit, size_t byteLimit) {
    const HANDLE input = GetStdHandle(STD_INPUT_HANDLE);
    const DWORD kind = GetFileType(input);
    require(kind == FILE_TYPE_PIPE || kind == FILE_TYPE_DISK, "unsupported_transport", "请使用JSONL管道或输入文件，不直接读取控制台按键");
    Service service(root, packetLimit, byteLimit);
    std::string line;
    bool discarding = false, closed = false;
    while (!closed && !service.exiting()) {
        DWORD available = 4096;
        if (kind == FILE_TYPE_PIPE && !PeekNamedPipe(input, nullptr, 0, nullptr, &available, nullptr)) {
            closed = true; break;
        }
        if (available) {
            std::array<char, 4096> buffer{};
            DWORD count = 0;
            if (!ReadFile(input, buffer.data(), std::min(available, DWORD(buffer.size())), &count, nullptr) || !count) {
                closed = true; break;
            }
            for (DWORD i = 0; i < count && !service.exiting(); ++i) {
                const char ch = buffer[i];
                if (ch == '\n') {
                    if (discarding) {
                        std::cout << failure(nullptr, "request_too_large", "请求超过65536字节").dump() << std::endl;
                    } else {
                        if (!line.empty() && line.back() == '\r') line.pop_back();
                        if (!line.empty()) std::cout << service.handle(line).dump() << std::endl;
                    }
                    line.clear(); discarding = false;
                } else if (!discarding) {
                    if (line.size() >= 65536) { discarding = true; line.clear(); }
                    else line += ch;
                }
            }
        }
        if (!service.exiting()) service.step();
        if (!available) std::this_thread::sleep_for(std::chrono::milliseconds(2));
    }
    service.stop("取消");
    if (!line.empty() || discarding)
        std::cout << failure(nullptr, "truncated_request", "输入结束前缺少完整换行请求").dump() << std::endl;
    return 0;
}
}
}

int wmain(int argc, wchar_t** argv) {
    try {
        if (argc == 2 && std::wstring(argv[1]) == L"--list-devices") {
            std::cout << pcs::Json({{"设备", pcs::deviceCapabilities()}}).dump() << std::endl;
            return 0;
        }
        if (argc < 2 || std::wstring(argv[1]) == L"--help") {
            std::cout << "PixelClusterSensor --stdio [--output-root=PATH] [--max-packets=128] [--max-bytes=268435456]\n"
                         "PixelClusterSensor --list-devices\n";
            return 0;
        }
        pcs::require(std::wstring(argv[1]) == L"--stdio", "invalid_argument", "未知启动参数");
        pcs::fs::path root = pcs::fs::current_path() / ".codex_tmp" / "PixelClusterSensor";
        uint64_t packets = 128;
        size_t bytes = 256 * 1024 * 1024;
        for (int i = 2; i < argc; ++i) {
            const std::wstring arg = argv[i];
            if (arg.starts_with(L"--output-root=")) root = pcs::fs::path(arg.substr(14));
            else if (arg.starts_with(L"--max-packets=")) packets = std::stoull(arg.substr(14));
            else if (arg.starts_with(L"--max-bytes=")) bytes = std::stoull(arg.substr(12));
            else throw pcs::Error("invalid_argument", "未知启动参数");
        }
        pcs::require(!root.empty() && packets >= 1 && packets <= 10000 && bytes >= 1024 * 1024 && bytes <= size_t(16) * 1024 * 1024 * 1024,
                     "invalid_argument", "输出预算越界");
        return pcs::serve(pcs::fs::absolute(root), packets, bytes);
    } catch (const std::exception& error) {
        std::cerr << error.what() << '\n'; return 2;
    }
}
