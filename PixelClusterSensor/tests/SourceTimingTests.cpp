#include "../SourceTiming.h"
#include <iostream>
#include <limits>
#include <stdexcept>

void check(bool ok) { if (!ok) throw std::runtime_error("Source timing regression"); }
int main() {
    using namespace pcs;
    const PairTiming valid{true, true, 10, 9, 1000, 990, 1, 1};
    check(checkPair(valid, 9, 8) == PairIssue::None);
    auto p = valid; p.colorMs = 1040;
    check(checkPair(p, 9, 8) == PairIssue::None);
    p.colorMs = 1040.01;
    check(checkPair(p, 9, 8) == PairIssue::Unsynchronized);
    p = valid; p.colorMs = 1788916090113.35; p.depthMs = 1788916089881.988;
    check(checkPair(p, 0, 0) == PairIssue::Unsynchronized);
    p = valid; p.colorDomain = 2;
    check(checkPair(p, 0, 0) == PairIssue::Unsynchronized);
    p = valid; p.colorMs = std::numeric_limits<double>::quiet_NaN();
    check(checkPair(p, 0, 0) == PairIssue::Unsynchronized);
    p = valid; p.depthMs = std::numeric_limits<double>::infinity();
    check(checkPair(p, 0, 0) == PairIssue::Unsynchronized);
    check(checkPair(valid, 10, 8) == PairIssue::StaleFrame);
    check(checkPair(valid, 9, 10) == PairIssue::StaleFrame);
    p = valid; p.hasDepth = false;
    check(checkPair(p, 0, 0) == PairIssue::MissingStream);
    for (const auto issue : {PairIssue::MissingStream, PairIssue::StaleFrame, PairIssue::Unsynchronized}) {
        check(retryStartupPair(issue, false, 1));
        check(!retryStartupPair(issue, false, 0));
        check(!retryStartupPair(issue, false, -1));
        check(!retryStartupPair(issue, true, 1000));
    }
    check(!retryStartupPair(PairIssue::None, false, 1000));
    std::cout << "PASS 23 source timing assertions\n";
}
