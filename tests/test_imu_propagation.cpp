// IMU propagation against motions with a known exact solution. Checks the frame and gravity
// conventions, which everything later depends on.

#include "vio/imu_propagation.h"

#include <cmath>
#include <cstdio>
#include <functional>

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

    std::printf("%s\n", failures == 0 ? "all ok" : "FAILED");
    return failures == 0 ? 0 : 1;
}
