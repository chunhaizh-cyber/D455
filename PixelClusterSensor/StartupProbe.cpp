#include "Sensor.h"
#include <opencv2/imgcodecs.hpp>
#include <chrono>
#include <fstream>
#include <iostream>

namespace pcs {
int probeStartup(const fs::path& root, int frames, int sessions, const Json& parameters) {
    require(!fs::exists(root), "output_exists", "诊断不覆盖既有输出目录");
    fs::create_directories(root);
    Json report = {{"格式", "PCS.RawStartupProbe/1"}, {"状态", "采集中"}, {"请求帧数", frames},
        {"请求会话数", sessions}, {"输入参数", parameters}, {"RealSenseSDK版本", RS2_API_VERSION_STR},
        {"OpenCV版本", CV_VERSION}, {"处理链", "共享Source原始RGBD；不分割、不配准、不插值"},
        {"采集期写图", false}, {"会话", Json::array()}};
    writeJson(root / "probe.json", report);
    bool complete = true;
    try {
        for (int session = 0; session < sessions; ++session) {
            std::vector<RawFrame> buffer;
            buffer.reserve(frames);
            std::vector<double> receiveMs;
            size_t bufferedBytes = 0;
            Json events = Json::array();
            auto source = openSource(parameters);
            const auto opened = std::chrono::steady_clock::now();
            const auto deadline = opened + std::chrono::seconds(20);
            for (int attempt = 0; buffer.size() < static_cast<size_t>(frames) && attempt < frames * 3 &&
                    std::chrono::steady_clock::now() < deadline; ++attempt) {
                try {
                    auto frame = source->next();
                    const auto bytes = frame.colorBgr.total() * frame.colorBgr.elemSize() + frame.depth16.total() * frame.depth16.elemSize();
                    require(bufferedBytes + bytes <= size_t(512) * 1024 * 1024, "resource_limit", "原始诊断缓冲超过512MiB");
                    bufferedBytes += bytes;
                    buffer.push_back(std::move(frame));
                    receiveMs.push_back(std::chrono::duration<double, std::milli>(
                        std::chrono::steady_clock::now() - opened).count());
                } catch (const Error& error) {
                    events.push_back({{"已接受帧数", buffer.size()}, {"代码", error.code}, {"说明", error.what()}});
                    // Diagnostic retries are recorded; they are never advertised as a complete stream.
                    if (error.code != "stale_frame" && error.code != "unsynchronized_frame" && error.code != "missing_stream") break;
                } catch (const std::exception& error) {
                    events.push_back({{"已接受帧数", buffer.size()}, {"代码", "capture_error"}, {"说明", error.what()}});
                    break;
                }
            }
            const Json calibration = source->description();
            source.reset();
            const auto directory = root / ("session_" + std::to_string(session + 1));
            fs::create_directory(directory);
            Json rows = Json::array();
            for (size_t i = 0; i < buffer.size(); ++i) {
                const auto& frame = buffer[i];
                auto save = [&](const cv::Mat& image, const std::string& kind) {
                    std::vector<uint8_t> encoded;
                    require(cv::imencode(".png", image, encoded), "io_error", "原始图像编码失败");
                    const auto name = std::to_string(i) + "_" + kind + ".png";
                    std::ofstream out(directory / name, std::ios::binary);
                    out.write(reinterpret_cast<const char*>(encoded.data()), static_cast<std::streamsize>(encoded.size()));
                    out.close(); require(!out.fail(), "io_error", "原始图像写入失败");
                    return Json{{"文件", "session_" + std::to_string(session + 1) + "/" + name},
                                {"字节数", encoded.size()}, {"SHA256", sha256(encoded)}};
                };
                rows.push_back({{"序号", i}, {"打开后接收毫秒", receiveMs[i]}, {"源信息", frame.source},
                    {"彩图", save(frame.colorBgr, "color")}, {"深度", save(frame.depth16, "depth")}});
            }
            const bool captured = buffer.size() == static_cast<size_t>(frames);
            complete &= captured;
            report["会话"].push_back({{"编号", session + 1}, {"请求数量已达到", captured},
                {"标定与源状态", calibration}, {"异常事件", events}, {"帧列表", rows}});
            writeJson(root / "probe.json", report);
            std::cout << "session " << session + 1 << ": " << buffer.size() << " frames, " << events.size() << " events\n" << std::flush;
            if (!captured) break;
        }
        report["状态"] = complete ? "数量完成，连续性另行核查" : "未完成";
    } catch (const std::exception& error) {
        report["状态"] = "失败"; report["错误"] = error.what();
        writeJson(root / "probe.json", report);
        throw;
    }
    writeJson(root / "probe.json", report);
    return complete ? 0 : 2;
}
}
