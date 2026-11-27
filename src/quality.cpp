#include "vio/quality.h"

#include <Eigen/Cholesky>
#include <Eigen/Eigenvalues>

#include <algorithm>
#include <cmath>
#include <limits>
#include <set>

namespace vio {
namespace {

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();

double clamp01(double x) { return std::isfinite(x) ? std::clamp(x, 0.0, 1.0) : 0.0; }

}  // namespace

double largest_sigma(const Eigen::Matrix3d& C) {
    if (!C.allFinite()) return kNaN;
    const Eigen::SelfAdjointEigenSolver<Eigen::Matrix3d> es(C, Eigen::EigenvaluesOnly);
    return std::sqrt(std::max(0.0, es.eigenvalues().maxCoeff()));
}

double poserr_463(const Eigen::Matrix3d& C_pos) {
    return std::cbrt(C_pos(0, 0) * C_pos(0, 0) + C_pos(1, 1) * C_pos(1, 1) + C_pos(2, 2) * C_pos(2, 2));
}

QualityResult QualityMonitor::update(const FrameSignals& s) {
    QualityResult r;
    r.sigma_v = largest_sigma(s.C_vel);
    int n_checks = 0, n_pass = 0;
    auto check = [&](bool applies, bool ok, unsigned bit) {
        if (!applies) return;
        ++n_checks;
        if (ok) ++n_pass;
        else r.fails |= bit;
    };
    check(true, s.features >= cfg_.min_features, FAIL_FEATURES);
    check(true, s.cells >= cfg_.min_grid_cells, FAIL_CELLS);
    check(s.tracked_ratio >= 0, s.tracked_ratio >= cfg_.min_tracked_ratio, FAIL_TRACKED);
    const int n_tracks = s.msckf_used + s.msckf_rejected;
    check(n_tracks >= cfg_.min_tracks_for_ratio, s.msckf_used >= cfg_.min_inlier_ratio * n_tracks,
          FAIL_INLIERS);
    check(true, !s.joint_rejected, FAIL_JOINT_GATE);
    check(s.imu_rate_hz >= 0, s.imu_rate_hz >= cfg_.imu_rate_min_hz, FAIL_IMU_RATE);
    check(s.imu_gap_ms >= 0, s.imu_gap_ms <= cfg_.imu_gap_max_ms, FAIL_IMU_GAP);
    check(true, std::abs(s.td_change_ms) <= cfg_.td_change_max_ms, FAIL_TD_CHANGE);
    check(true, s.speed <= cfg_.max_speed, FAIL_SPEED);
    const bool good = r.fails == 0;

    if (!started_) {
        started_ = true;
        last_good_ = s.t;
    }
    // "No good frame for lost_after_bad_s" is judged before this frame counts: after a long gap
    // without frames even a good frame starts from LOST.
    const bool timed_out = to_sec(s.t - last_good_) > cfg_.lost_after_bad_s;
    if (good) last_good_ = s.t;
    // Comparisons written so that a NaN sigma counts as too large.
    const bool good_v = good && r.sigma_v <= cfg_.vel_sigma_degraded;
    const bool v_lost = !(r.sigma_v <= cfg_.vel_sigma_lost);
    good_in_row_ = good_v ? good_in_row_ + 1 : 0;
    if (timed_out || v_lost) {
        state_ = QualityState::LOST;
        good_in_row_ = good_v ? 1 : 0;
    } else if (good_in_row_ >= cfg_.good_frames_to_recover) {
        state_ = QualityState::GOOD;
    } else if (state_ == QualityState::GOOD) {
        state_ = QualityState::DEGRADED;  // this frame is not good (or its velocity sigma too large)
    }
    r.state = state_;

    const double m_v = clamp01(1 - r.sigma_v / cfg_.vel_sigma_degraded);
    const double m_l = clamp01(1 - r.sigma_v / cfg_.vel_sigma_lost);
    const double c = n_checks ? static_cast<double>(n_pass) / n_checks : 0.0;
    switch (state_) {
        case QualityState::GOOD: r.score = 50 + static_cast<int>(std::lround(50 * m_v)); break;
        case QualityState::DEGRADED: {
            const int f = std::clamp(cfg_.score_floor, 1, 49);
            r.score = f + static_cast<int>(std::lround((49 - f) * c * m_l));
            break;
        }
        case QualityState::LOST: r.score = -1; break;
    }
    return r;
}

OutputDecision decide_output(QualityResult& q, QualityMonitor& monitor, const Eigen::Matrix3d& C_pos,
                             const QualityConfig& cfg) {
    OutputDecision d;
    const bool usable = C_pos.allFinite() && C_pos.llt().info() == Eigen::Success;
    if (!usable && q.state != QualityState::LOST) {
        monitor.force_lost();
        q.state = QualityState::LOST;
        q.score = -1;
    }
    if (q.state == QualityState::LOST) return d;
    d.position_valid = true;
    d.velocity_valid = q.sigma_v <= cfg.vel_sigma_degraded;  // false for NaN
    d.quality = q.score;
    return d;
}

int occupied_cells(const std::vector<FeatureObs>& features, int width, int height, int cols, int rows) {
    if (width <= 0 || height <= 0 || cols <= 0 || rows <= 0) return 0;
    std::set<int> cells;
    for (const auto& f : features) {
        const int cx = std::clamp(static_cast<int>(f.uv.x() * cols / width), 0, cols - 1);
        const int cy = std::clamp(static_cast<int>(f.uv.y() * rows / height), 0, rows - 1);
        cells.insert(cy * cols + cx);
    }
    return static_cast<int>(cells.size());
}

ImuWindow summarize_imu(const std::vector<ImuSample>& imu, Timestamp t0, Timestamp t1) {
    ImuWindow w;
    if (t1 <= t0) return w;
    auto later = [](Timestamp t, const ImuSample& m) { return t < m.t; };
    const size_t i0 = std::upper_bound(imu.begin(), imu.end(), t0, later) - imu.begin();  // first > t0
    const size_t i1 = std::upper_bound(imu.begin(), imu.end(), t1, later) - imu.begin();  // first > t1
    w.n = static_cast<int>(i1 - i0);
    w.rate_hz = w.n / to_sec(t1 - t0);
    // Gaps between consecutive samples, from the last one at or before t0, and from the last one at
    // or before t1 up to t1 (an IMU that stopped shows here).
    Timestamp prev = i0 > 0 ? imu[i0 - 1].t : t0;
    Timestamp gap = 0;
    for (size_t i = i0; i < i1; ++i) {
        gap = std::max(gap, imu[i].t - prev);
        prev = imu[i].t;
        w.mean_accel += imu[i].accel;
    }
    gap = std::max(gap, t1 - prev);
    w.max_gap_ms = gap * 1e-6;
    if (w.n > 0) w.mean_accel /= w.n;
    return w;
}

ExcitationMonitor::Result ExcitationMonitor::add(Timestamp t, const Eigen::Vector3d& a_world,
                                                 const Eigen::Vector3d& v_world,
                                                 const Eigen::Vector3d& down) {
    Result r;
    const Eigen::Vector3d vh = v_world - v_world.dot(down) * down;
    if (!(vh.norm() >= cfg_.excitation_min_speed) || !a_world.allFinite()) {
        buf_.clear();
        since_ = -1;
        r.a_lat = r.range = kNaN;
        return r;
    }
    r.a_lat = a_world.dot(down.cross(vh).normalized());
    if (since_ < 0) since_ = t;
    buf_.emplace_back(t, r.a_lat);
    const auto window = static_cast<Timestamp>(cfg_.excitation_window_s * 1e9);
    while (buf_.front().first < t - window) buf_.pop_front();
    if (t - since_ < window) {
        r.range = kNaN;
        return r;
    }
    double lo = r.a_lat, hi = r.a_lat;
    for (const auto& [ti, a] : buf_) {
        lo = std::min(lo, a);
        hi = std::max(hi, a);
    }
    r.range = hi - lo;
    r.flag = r.range < cfg_.excitation_min_range;
    return r;
}

}  // namespace vio
