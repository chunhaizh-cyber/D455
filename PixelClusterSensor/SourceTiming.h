#pragma once
#include <cmath>
#include <cstdint>

namespace pcs {
enum class PairIssue { None, MissingStream, StaleFrame, Unsynchronized };

struct PairTiming {
    bool hasColor, hasDepth;
    uint64_t colorNumber, depthNumber;
    double colorMs, depthMs;
    int colorDomain, depthDomain;
};

inline PairIssue checkPair(const PairTiming& pair, uint64_t lastColor, uint64_t lastDepth) {
    if (!pair.hasColor || !pair.hasDepth) return PairIssue::MissingStream;
    if (pair.colorNumber <= lastColor || pair.depthNumber <= lastDepth) return PairIssue::StaleFrame;
    if (pair.colorDomain != pair.depthDomain || !std::isfinite(pair.colorMs) || !std::isfinite(pair.depthMs) ||
        std::abs(pair.colorMs - pair.depthMs) > 50) return PairIssue::Unsynchronized;
    return PairIssue::None;
}

inline bool retryStartupPair(PairIssue issue, bool delivered, int remainingMs) {
    return issue != PairIssue::None && !delivered && remainingMs > 0;
}
}
