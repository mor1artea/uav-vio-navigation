// IMU propagation against motions with a known exact solution. Checks the frame and gravity
// conventions, which everything later depends on. Also the accel delay compensation (shift_accel):
// an FC stream whose accel lags the gyro by 6 ms, with jittered stamps, lines up after the shift.

#include "vio/imu_propagation.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <functional>
#include <random>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const char* what, double value) {
    std::printf("%s %-55s %.3g\n", ok ? "ok  " : "FAIL", what, value);
    if (!ok) ++failures;
}

// Generates IMU samples for a motion given as functions of time, integrates them and returns the
// final position and rotation error against the true motion.
struct Motion {
    std::function<Eigen::Vector3d(double)> p, v, a;      // world frame
    std::function<Eigen::Quaterniond(double)> q;         // q_wb
    std::function<Eigen::Vector3d(double)> w_body;       // angular rate in the body frame
};

void run(const Motion& m, const Eigen::Vector3d& g, double duration, double rate, double& pos_err,
         double& rot_err) {
    vio::ImuPropagator prop(g);
    vio::State s0;
    s0.t = 0;
    s0.p = m.p(0);
    s0.v = m.v(0);
    s0.q = m.q(0);
    prop.reset(s0);
    const int n = static_cast<int>(duration * rate);
    for (int i = 0; i <= n; ++i) {
        const double t = i / rate;
        vio::ImuSample smp;
        smp.t = static_cast<vio::Timestamp>(std::llround(t * 1e9));
        smp.gyro = m.w_body(t);
        smp.accel = m.q(t).conjugate() * (m.a(t) - g);  // specific force in the body frame
        prop.add(smp);
    }
    const vio::State& s = prop.state();
    pos_err = (s.p - m.p(duration)).norm();
    rot_err = Eigen::AngleAxisd(s.q.conjugate() * m.q(duration)).angle();
}

}  // namespace

// The same 2 Hz signal on gyro x and accel x, seen 7 ms (gyro) and 13 ms (accel) late, at 200 Hz
// with stamps jittered by up to 2.5 ms. After shift_accel(6 ms) the accel must match the gyro.
void test_shift_accel() {
    std::vector<vio::ImuSample> imu;
    std::mt19937 rng(3);
    std::uniform_real_distribution<double> jitter(0, 2.5e-3);
    const double w = 2 * M_PI * 2;
    for (int i = 0; i < 400; ++i) {
        const double t = i * 0.005 + jitter(rng);
        vio::ImuSample m;
        m.t = static_cast<vio::Timestamp>(std::llround(t * 1e9));
        m.gyro.x() = std::sin(w * (t - 0.007));
        m.accel.x() = std::sin(w * (t - 0.013));
        imu.push_back(m);
    }
    double before = 0, after = 0;
    for (const auto& m : imu) before = std::max(before, std::abs(m.accel.x() - m.gyro.x()));
    const size_t dropped = vio::shift_accel(imu, 0.006);
    for (const auto& m : imu) after = std::max(after, std::abs(m.accel.x() - m.gyro.x()));
    check(after < 2e-3 && before > 0.07 && dropped >= 1 && dropped <= 2,
          "accel shifted by 6 ms: largest accel - gyro difference", after);
}

int main() {
    const Eigen::Vector3d g_ned(0, 0, 9.81);
    double pos_err = 0, rot_err = 0;

    // 1. Constant yaw rate and constant world acceleration: midpoint integration is exact here,
    //    so any error means a sign or frame mistake.
    {
        const Eigen::Vector3d acc(1.0, -0.5, 0.2);
        const double wz = 0.3;
        Motion m;
        m.p = [=](double t) { return Eigen::Vector3d(0.5 * acc * t * t); };
        m.v = [=](double t) { return Eigen::Vector3d(acc * t); };
        m.a = [=](double) { return acc; };
        m.q = [=](double t) {
            return Eigen::Quaterniond(Eigen::AngleAxisd(wz * t, Eigen::Vector3d::UnitZ()));
        };
        m.w_body = [=](double) { return Eigen::Vector3d(0, 0, wz); };
        run(m, g_ned, 10.0, 200.0, pos_err, rot_err);
        check(pos_err < 1e-6, "yaw rate + constant acceleration: position error [m]", pos_err);
        check(rot_err < 1e-9, "yaw rate + constant acceleration: rotation error [rad]", rot_err);
    }

    // 2. Circle at constant speed with a banked attitude (coning-like rotation about a tilted
    //    axis). Not exact for midpoint integration, but the error must stay small at 200 Hz.
    {
        const double r = 20.0, w = 0.5;  // 10 m/s on a 20 m circle
        Motion m;
        m.p = [=](double t) { return Eigen::Vector3d(r * std::cos(w * t), r * std::sin(w * t), -30); };
        m.v = [=](double t) {
            return Eigen::Vector3d(-r * w * std::sin(w * t), r * w * std::cos(w * t), 0);
        };
        m.a = [=](double t) {
            return Eigen::Vector3d(-r * w * w * std::cos(w * t), -r * w * w * std::sin(w * t), 0);
        };
        const Eigen::Quaterniond bank(Eigen::AngleAxisd(0.4, Eigen::Vector3d::UnitX()));
        m.q = [=](double t) {
            return Eigen::Quaterniond(Eigen::AngleAxisd(w * t, Eigen::Vector3d::UnitZ())) * bank;
        };
        m.w_body = [=](double) { return Eigen::Vector3d(bank.conjugate() * Eigen::Vector3d(0, 0, w)); };
        run(m, g_ned, 10.0, 200.0, pos_err, rot_err);
        check(pos_err < 0.01, "banked circle 10 m/s, 10 s: position error [m]", pos_err);
        check(rot_err < 1e-6, "banked circle 10 m/s, 10 s: rotation error [rad]", rot_err);
    }
    test_shift_accel();

    std::printf("%s\n", failures == 0 ? "all ok" : "FAILED");
    return failures == 0 ? 0 : 1;
}
