#include "vio/inject.h"

#include <algorithm>
#include <cmath>
#include <sstream>
#include <stdexcept>

namespace vio {
namespace {

Timestamp at(Timestamp t_ref, double s) { return t_ref + static_cast<Timestamp>(std::llround(s * 1e9)); }

}  // namespace

Fault parse_fault(const std::string& spec) {
    std::vector<std::string> p;
    std::stringstream ss(spec);
    for (std::string x; std::getline(ss, x, ':');) p.push_back(x);
    auto bad = [&](const char* why) {
        return std::runtime_error("--inject " + spec + ": " + why +
                                  " (blackout:T:D, frozen:T:D, outliers:T:D:SHARE:PX, imu_gap:T:D, td_step:T:MS)");
    };
    if (p.empty()) throw bad("empty");
    Fault f;
    std::vector<double> v;
    try {
        for (size_t i = 1; i < p.size(); ++i) v.push_back(std::stod(p[i]));
    } catch (const std::exception&) {
        throw bad("not a number");
    }
    const std::string& k = p[0];
    size_t need = 0;
    if (k == "blackout") {
        f.kind = Fault::BLACKOUT;
        need = 2;
    } else if (k == "frozen") {
        f.kind = Fault::FROZEN;
        need = 2;
    } else if (k == "outliers") {
        f.kind = Fault::OUTLIERS;
        need = 4;
    } else if (k == "imu_gap") {
        f.kind = Fault::IMU_GAP;
        need = 2;
    } else if (k == "td_step") {
        f.kind = Fault::TD_STEP;
        need = 2;
    } else {
        throw bad("unknown fault");
    }
    if (v.size() != need) throw bad("wrong number of values");
    f.t_s = v[0];
    if (f.kind == Fault::TD_STEP) {
        f.td_ms = v[1];
    } else {
        f.dur_s = v[1];
        if (f.dur_s <= 0) throw bad("duration must be above 0");
    }
    if (f.kind == Fault::OUTLIERS) {
        f.share = v[2];
        f.px = v[3];
        if (f.share < 0 || f.share > 1) throw bad("share must be 0-1");
    }
    return f;
}

bool fault_active(const Fault& f, Timestamp t, Timestamp t_ref) {
    const bool started = t >= at(t_ref, f.t_s);
    return f.kind == Fault::TD_STEP ? started : started && t < at(t_ref, f.t_s + f.dur_s);
}

unsigned active_faults(const std::vector<Fault>& faults, Timestamp t, Timestamp t_ref) {
    unsigned bits = 0;
    for (const auto& f : faults)
        if (fault_active(f, t, t_ref)) bits |= 1u << f.kind;
    return bits;
}

void apply_outliers(const std::vector<Fault>& faults, Timestamp t, Timestamp t_ref,
                    std::vector<FeatureObs>& features, std::mt19937& rng) {
    for (const auto& f : faults)
        if (f.kind == Fault::OUTLIERS && fault_active(f, t, t_ref)) move_outliers(features, f.share, f.px, rng);
}

void apply_session_faults(Session& s, const std::vector<Fault>& faults, Timestamp t_ref) {
    for (const auto& f : faults) {
        const Timestamp t0 = at(t_ref, f.t_s);
        if (f.kind == Fault::IMU_GAP) {
            const Timestamp t1 = at(t_ref, f.t_s + f.dur_s);
            s.imu.erase(std::remove_if(s.imu.begin(), s.imu.end(),
                                       [&](const ImuSample& m) { return m.t >= t0 && m.t < t1; }),
                        s.imu.end());
        } else if (f.kind == Fault::TD_STEP) {
            const auto shift = static_cast<Timestamp>(std::llround(f.td_ms * 1e6));
            for (auto& fr : s.frames)
                if (fr.t >= t0) fr.t += shift;
            std::vector<CameraFrame> kept;
            for (auto& fr : s.frames)
                if (kept.empty() || fr.t > kept.back().t) kept.push_back(std::move(fr));
            s.frames = std::move(kept);
        }
    }
}

void move_outliers(std::vector<FeatureObs>& features, double share, double px, std::mt19937& rng) {
    std::uniform_real_distribution<double> u(0, 1);
    for (auto& f : features) {
        if (u(rng) >= share) continue;
        const double a = 2 * M_PI * u(rng);
        f.uv += px * Eigen::Vector2d(std::cos(a), std::sin(a));
    }
}

}  // namespace vio
