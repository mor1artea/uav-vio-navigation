// Quality monitor checks on scripted signals (vio/quality.h):
//   - state machine: starts DEGRADED, GOOD after 5 good frames, DEGRADED on one bad frame, LOST after
//     2 s without a good frame or after a 3 s gap without frames, back to GOOD only after 5 good frames;
//   - velocity sigma: DEGRADED above 0.9 m/s, LOST above 2 m/s or NaN;
//   - the inlier ratio is checked only with enough tracks;
//   - output: position valid unless LOST, velocity NaN above 0.9 m/s, a huge position covariance
//     passed with a valid position and no change of state, a NaN or non-positive one forces LOST;
//   - low excitation flag (E1): set on a constant circle and a straight leg, not on S-turns, not below
//     the speed limit;
//   - IMU window: rate and largest gap.

#include "vio/quality.h"

#include <cmath>
#include <cstdio>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const char* what, double value = 0) {
    std::printf("%s %-70s %.3g\n", ok ? "ok  " : "FAIL", what, value);
    if (!ok) ++failures;
}

using vio::QualityState;

constexpr vio::Timestamp kFrame = 300'000'000;  // update frames every 0.3 s

vio::FrameSignals good_frame(vio::Timestamp t, double sigma_v = 0.2) {
    vio::FrameSignals s;
    s.t = t;
    s.features = 100;
    s.cells = 40;
    s.tracked_ratio = 0.9;
    s.msckf_used = 30;
    s.msckf_rejected = 2;
    s.imu_rate_hz = 200;
    s.imu_gap_ms = 5;
    s.speed = 10;
    s.C_vel = Eigen::Matrix3d::Identity() * sigma_v * sigma_v;
    return s;
}

vio::FrameSignals blind_frame(vio::Timestamp t) {
    vio::FrameSignals s = good_frame(t);
    s.features = 0;
    s.cells = 0;
    s.msckf_used = 0;
    s.msckf_rejected = 0;
    return s;
}

void test_state_machine() {
    vio::QualityConfig cfg;
    vio::QualityMonitor m(cfg);
    vio::Timestamp t = 0;
    QualityState st{};
    for (int i = 0; i < 4; ++i) st = m.update(good_frame(t += kFrame)).state;
    check(st == QualityState::DEGRADED, "4 good frames after the start: still DEGRADED");
    const vio::QualityResult r5 = m.update(good_frame(t += kFrame));
    check(r5.state == QualityState::GOOD && r5.score >= 50 && r5.score <= 100, "5th good frame: GOOD, score 50-100",
          r5.score);
    const vio::QualityResult bad = m.update(blind_frame(t += kFrame));
    check(bad.state == QualityState::DEGRADED && bad.score >= 10 && bad.score <= 49 &&
              bad.fails == (vio::FAIL_FEATURES | vio::FAIL_CELLS),
          "one frame without features: DEGRADED, score 10-49, fails features and cells", bad.score);
    // 2 s without a good frame: the last good one was 0.3 s before the blind frame above.
    int n_bad = 1;
    while (m.update(blind_frame(t += kFrame)).state != QualityState::LOST) ++n_bad;
    const double lost_after = (n_bad + 1) * 0.3;
    check(lost_after > 2.0 && lost_after < 2.0 + 0.31, "LOST once no good frame for more than 2 s (s)",
          lost_after);
    for (int i = 0; i < 4; ++i) st = m.update(good_frame(t += kFrame)).state;
    check(st == QualityState::LOST, "4 good frames after LOST: still LOST");
    const vio::QualityResult back = m.update(good_frame(t += kFrame));
    check(back.state == QualityState::GOOD, "5th good frame: GOOD again");
    const vio::QualityResult gap = m.update(good_frame(t += 3'000'000'000));
    check(gap.state == QualityState::LOST, "a good frame after a 3 s gap without frames: LOST");
}

void test_velocity_sigma() {
    vio::QualityConfig cfg;
    vio::QualityMonitor m(cfg);
    vio::Timestamp t = 0;
    for (int i = 0; i < 6; ++i) m.update(good_frame(t += kFrame));
    const vio::QualityResult d = m.update(good_frame(t += kFrame, 0.95));
    check(d.state == QualityState::DEGRADED && std::abs(d.sigma_v - 0.95) < 1e-9,
          "velocity sigma 0.95 m/s on good frames: DEGRADED", d.sigma_v);
    // The worst direction counts, not only the axes.
    vio::FrameSignals diag = good_frame(t += kFrame);
    diag.C_vel << 0.5, 0.45, 0, 0.45, 0.5, 0, 0, 0, 0.01;  // axes 0.71 m/s, along x+y 0.97 m/s
    const vio::QualityResult dd = m.update(diag);
    check(dd.state == QualityState::DEGRADED && dd.sigma_v > 0.96, "largest direction counts (axes 0.71, diagonal 0.97)",
          dd.sigma_v);
    const vio::QualityResult l = m.update(good_frame(t += kFrame, 2.1));
    check(l.state == QualityState::LOST && l.score == -1, "velocity sigma 2.1 m/s: LOST");
    int n = 0;
    while (m.update(good_frame(t += kFrame)).state != QualityState::GOOD) ++n;
    check(n == 4, "back to GOOD on the 5th good frame with sigma 0.2 m/s", n + 1);
    // DEGRADED at the edge of LOST, every check failing: the score stays at the floor.
    vio::QualityConfig cf;
    cf.score_floor = 20;
    vio::QualityMonitor mf(cf);
    vio::FrameSignals worst = blind_frame(kFrame);
    worst.joint_rejected = true;
    worst.speed = 30;
    worst.C_vel = Eigen::Matrix3d::Identity() * 1.99 * 1.99;
    const vio::QualityResult w = mf.update(worst);
    check(w.state == QualityState::DEGRADED && w.score == 20, "DEGRADED score never below the floor (20 here)",
          w.score);
    vio::FrameSignals nan_v = good_frame(t += kFrame);
    nan_v.C_vel(0, 0) = NAN;
    check(m.update(nan_v).state == QualityState::LOST, "NaN velocity covariance: LOST");
}

void test_inlier_ratio() {
    vio::QualityConfig cfg;
    vio::QualityMonitor m(cfg);
    vio::FrameSignals few = good_frame(kFrame);
    few.msckf_used = 2;
    few.msckf_rejected = 6;  // 25 % inliers, but only 8 tracks
    check(m.update(few).fails == 0, "8 tracks with 25 % inliers: ratio not checked");
    vio::FrameSignals many = good_frame(2 * kFrame);
    many.msckf_used = 10;
    many.msckf_rejected = 10;
    check(m.update(many).fails == vio::FAIL_INLIERS, "20 tracks with 50 % inliers: fails");
}

void test_output() {
    vio::QualityConfig cfg;
    vio::QualityMonitor m(cfg);
    vio::Timestamp t = 0;
    vio::QualityResult q;
    for (int i = 0; i < 5; ++i) q = m.update(good_frame(t += kFrame));
    const Eigen::Matrix3d huge = Eigen::Matrix3d::Identity() * 1e6;  // 1 km sigma
    vio::OutputDecision o = vio::decide_output(q, m, huge, cfg);
    check(q.state == QualityState::GOOD && o.position_valid && o.velocity_valid && o.quality == q.score,
          "GOOD with a 1 km position sigma: position and velocity sent, state unchanged");
    q = m.update(good_frame(t += kFrame, 1.0));
    o = vio::decide_output(q, m, Eigen::Matrix3d::Identity(), cfg);
    check(q.state == QualityState::DEGRADED && o.position_valid && !o.velocity_valid,
          "velocity sigma 1.0 m/s: position sent, velocity NaN");
    q = m.update(good_frame(t += kFrame));
    Eigen::Matrix3d bad = Eigen::Matrix3d::Identity();
    bad(1, 1) = NAN;
    o = vio::decide_output(q, m, bad, cfg);
    check(q.state == QualityState::LOST && !o.position_valid && !o.velocity_valid && o.quality == -1,
          "NaN in the position covariance: LOST, nothing valid, quality -1");
    q = m.update(good_frame(t += kFrame));
    check(q.state == QualityState::LOST, "the forced LOST is kept until 5 good frames");
    Eigen::Matrix3d neg = Eigen::Matrix3d::Identity();
    neg(2, 2) = -1;
    vio::QualityMonitor m2(cfg);
    q = m2.update(good_frame(kFrame));
    o = vio::decide_output(q, m2, neg, cfg);
    check(!o.position_valid && q.state == QualityState::LOST, "negative position variance: LOST");
    check(std::abs(vio::poserr_463(Eigen::Matrix3d::Identity() * 4.3 * 4.3) - 10.0) < 0.15,
          "posErr 4.6.3 reaches 10 m at sigma 4.3 m on all axes", vio::poserr_463(Eigen::Matrix3d::Identity() * 18.49));
}

// Runs 15 s of flight at 10 m/s with the given world acceleration and velocity direction.
bool excitation_flag(double speed, double a_lat_amplitude, double period_s, bool constant_turn) {
    vio::QualityConfig cfg;
    vio::ExcitationMonitor e(cfg);
    const Eigen::Vector3d down(0, 0, 1);
    vio::ExcitationMonitor::Result r;
    for (int i = 0; i <= 50; ++i) {
        const double t = i * 0.3;
        const double heading = constant_turn ? t * 0.2 : 0;  // rad
        const Eigen::Vector3d dir(std::cos(heading), std::sin(heading), 0);
        const Eigen::Vector3d lat = down.cross(dir);
        const double a = constant_turn ? a_lat_amplitude : a_lat_amplitude * std::sin(2 * M_PI * t / period_s);
        r = e.add(static_cast<vio::Timestamp>(t * 1e9), a * lat + Eigen::Vector3d(0.05, -0.05, 0.1),
                  speed * dir, down);
    }
    return r.flag;
}

void test_excitation() {
    check(excitation_flag(10, 0.0, 1, false), "straight leg at constant speed: low excitation flag");
    check(excitation_flag(10, 2.0, 1, true), "constant circle, 2 m/s^2 sideways: flag");
    check(!excitation_flag(10, 2.0, 8, false), "S-turns, +-2 m/s^2 sideways every 8 s: no flag");
    check(!excitation_flag(2, 0.0, 1, false), "2 m/s, below the speed limit: no flag");
}

void test_imu_window() {
    std::vector<vio::ImuSample> imu;
    for (int i = 0; i <= 200; ++i) {
        if (i > 100 && i < 110) continue;  // 50 ms gap
        vio::ImuSample s;
        s.t = static_cast<vio::Timestamp>(i) * 5'000'000;
        s.accel = Eigen::Vector3d(0, 0, -9.81);
        imu.push_back(s);
    }
    const vio::ImuWindow w = vio::summarize_imu(imu, 0, 1'000'000'000);
    check(std::abs(w.max_gap_ms - 50) < 1e-6 && std::abs(w.rate_hz - 191) < 1e-6 &&
              std::abs(w.mean_accel.z() + 9.81) < 1e-9,
          "IMU window: 191 samples in 1 s, largest gap 50 ms", w.max_gap_ms);
    const vio::ImuWindow end = vio::summarize_imu(imu, 900'000'000, 1'100'000'000);
    check(std::abs(end.max_gap_ms - 100) < 1e-6, "IMU stopped 100 ms before the window end: gap 100 ms",
          end.max_gap_ms);
}

}  // namespace

int main() {
    test_state_machine();
    test_velocity_sigma();
    test_inlier_ratio();
    test_output();
    test_excitation();
    test_imu_window();
    std::printf("%s\n", failures ? "FAILED" : "all passed");
    return failures ? 1 : 0;
}
