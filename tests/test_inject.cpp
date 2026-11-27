// Fault injection checks (vio/inject.h):
//   - parsing: the five kinds, and readable errors for wrong specs;
//   - timing: which faults are active when, td_step lasting;
//   - imu_gap: the samples are removed from the session, and the gap reaches the quality monitor
//     through the same IMU vector the replay hands it (summarize_imu, FAIL_IMU_GAP);
//   - td_step: camera times shifted from the start on, still in order;
//   - outliers: the share moved, each by exactly the given distance; with two windows each applies
//     only inside its own.

#include "vio/inject.h"
#include "vio/quality.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <random>
#include <stdexcept>

namespace {

int failures = 0;

void check(bool ok, const char* what, double value = 0) {
    std::printf("%s %-70s %.3g\n", ok ? "ok  " : "FAIL", what, value);
    if (!ok) ++failures;
}

bool throws(const char* spec) {
    try {
        vio::parse_fault(spec);
    } catch (const std::runtime_error&) {
        return true;
    }
    return false;
}

constexpr vio::Timestamp kRef = 1'000'000'000;  // first IMU sample
vio::Timestamp at(double s) { return kRef + static_cast<vio::Timestamp>(std::llround(s * 1e9)); }

vio::Session make_session() {
    vio::Session s;
    for (int i = 0; i <= 2000; ++i) {  // 10 s at 200 Hz
        vio::ImuSample m;
        m.t = kRef + static_cast<vio::Timestamp>(i) * 5'000'000;
        m.accel = Eigen::Vector3d(0, 0, -9.81);
        s.imu.push_back(m);
    }
    for (int i = 0; i < 300; ++i) {  // 30 Hz
        vio::CameraFrame f;
        f.t = kRef + static_cast<vio::Timestamp>(i) * 33'333'333;
        s.frames.push_back(f);
    }
    return s;
}

void test_parse() {
    const vio::Fault o = vio::parse_fault("outliers:12.5:3:0.3:8");
    check(o.kind == vio::Fault::OUTLIERS && o.t_s == 12.5 && o.dur_s == 3 && o.share == 0.3 && o.px == 8,
          "outliers:12.5:3:0.3:8 parsed");
    const vio::Fault t = vio::parse_fault("td_step:20:-15");
    check(t.kind == vio::Fault::TD_STEP && t.td_ms == -15, "td_step:20:-15 parsed");
    check(vio::parse_fault("blackout:1:2").kind == vio::Fault::BLACKOUT &&
              vio::parse_fault("frozen:1:2").kind == vio::Fault::FROZEN &&
              vio::parse_fault("imu_gap:1:0.1").kind == vio::Fault::IMU_GAP,
          "blackout, frozen, imu_gap parsed");
    check(throws("smoke:1:2") && throws("blackout:1") && throws("blackout:1:x") && throws("blackout:1:0") &&
              throws("outliers:1:2:1.5:3"),
          "unknown kind, missing value, not a number, zero duration, share above 1: errors");
}

void test_timing() {
    const std::vector<vio::Fault> f = {vio::parse_fault("blackout:2:1"), vio::parse_fault("td_step:5:10")};
    check(vio::active_faults(f, at(1.99), kRef) == 0 && vio::active_faults(f, at(2.0), kRef) == 1 &&
              vio::active_faults(f, at(2.99), kRef) == 1 && vio::active_faults(f, at(3.0), kRef) == 0,
          "blackout active in [2, 3) s");
    check(vio::active_faults(f, at(100), kRef) == 16, "td_step active from its start on");
}

void test_imu_gap() {
    vio::Session s = make_session();
    vio::apply_session_faults(s, {vio::parse_fault("imu_gap:3:0.5")}, kRef);
    int inside = 0;
    for (const auto& m : s.imu) inside += m.t >= at(3) && m.t < at(3.5);
    check(inside == 0 && s.imu.size() == 2001 - 100, "imu_gap:3:0.5 removes the 100 samples in [3, 3.5) s",
          s.imu.size());
    // The replay gives the monitor the IMU between two update frames from session.imu.
    const vio::ImuWindow w = vio::summarize_imu(s.imu, at(2.9), at(3.6));
    check(std::abs(w.max_gap_ms - 505) < 1e-6, "the monitor's IMU window across it: largest gap 505 ms",
          w.max_gap_ms);
    const vio::ImuWindow inside_gap = vio::summarize_imu(s.imu, at(3.1), at(3.3));
    check(inside_gap.n == 0 && inside_gap.max_gap_ms > 200, "a window inside the gap: no samples, gap counted",
          inside_gap.max_gap_ms);
    vio::QualityConfig cfg;
    vio::QualityMonitor mon(cfg);
    vio::FrameSignals sig;
    sig.t = at(3.6);
    sig.features = 100;
    sig.cells = 40;
    sig.imu_rate_hz = w.rate_hz;
    sig.imu_gap_ms = w.max_gap_ms;
    sig.C_vel = Eigen::Matrix3d::Identity() * 0.01;
    const unsigned fails = mon.update(sig).fails;
    check(fails == (vio::FAIL_IMU_GAP | vio::FAIL_IMU_RATE), "the monitor fails the frame for IMU gap and rate",
          fails);
}

void test_td_step() {
    vio::Session s = make_session();
    vio::apply_session_faults(s, {vio::parse_fault("td_step:5:20")}, kRef);
    bool shifted = true, ordered = true;
    for (size_t i = 0; i < s.frames.size(); ++i) {
        const vio::Timestamp orig = kRef + static_cast<vio::Timestamp>(i) * 33'333'333;
        shifted &= s.frames[i].t == (orig >= at(5) ? orig + 20'000'000 : orig);
        if (i) ordered &= s.frames[i].t > s.frames[i - 1].t;
    }
    check(shifted && ordered && s.frames.size() == 300, "td_step +20 ms: frames from 5 s shifted, all kept, in order");
    vio::Session b = make_session();
    vio::apply_session_faults(b, {vio::parse_fault("td_step:5:-80")}, kRef);
    ordered = true;
    for (size_t i = 1; i < b.frames.size(); ++i) ordered &= b.frames[i].t > b.frames[i - 1].t;
    check(ordered && b.frames.size() == 298, "td_step -80 ms: the 2 frames that would go back are dropped",
          b.frames.size());
}

void test_outliers() {
    std::vector<vio::FeatureObs> f(10000);
    for (size_t i = 0; i < f.size(); ++i) f[i].uv = Eigen::Vector2d(100, 200);
    std::mt19937 rng(1);
    vio::move_outliers(f, 0.3, 8, rng);
    int moved = 0;
    bool exact = true;
    for (const auto& o : f) {
        const double d = (o.uv - Eigen::Vector2d(100, 200)).norm();
        if (d > 1e-9) {
            ++moved;
            exact &= std::abs(d - 8) < 1e-9;
        }
    }
    check(std::abs(moved / 10000.0 - 0.3) < 0.02 && exact, "outliers 0.3 / 8 px: about 30 % moved, each by 8 px",
          moved / 10000.0);
    // Two windows: every observation moved, by 5 px in [1, 2) s and by 20 px in [3, 4) s, none between.
    const std::vector<vio::Fault> two = {vio::parse_fault("outliers:1:1:1:5"), vio::parse_fault("outliers:3:1:1:20")};
    auto moved_by = [&](double t_s) {
        std::vector<vio::FeatureObs> g(50);
        vio::apply_outliers(two, at(t_s), kRef, g, rng);
        double lo = 1e9, hi = 0;
        for (const auto& o : g) {
            lo = std::min(lo, o.uv.norm());
            hi = std::max(hi, o.uv.norm());
        }
        return std::make_pair(lo, hi);
    };
    const auto a = moved_by(1.5), b = moved_by(2.5), c = moved_by(3.5);
    check(std::abs(a.first - 5) < 1e-9 && std::abs(a.second - 5) < 1e-9 && b.second == 0 &&
              std::abs(c.first - 20) < 1e-9 && std::abs(c.second - 20) < 1e-9,
          "two outlier windows: 5 px only in the first, 20 px only in the second", c.second);
}

}  // namespace

int main() {
    test_parse();
    test_timing();
    test_imu_gap();
    test_td_step();
    test_outliers();
    std::printf("%s\n", failures ? "FAILED" : "all passed");
    return failures ? 1 : 0;
}
