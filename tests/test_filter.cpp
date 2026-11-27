// Filter checks with exactly known answers:
//   - the error-state transition matrix against a numeric Jacobian of the mean propagation, over 1 s
//     of rotating and accelerating motion;
//   - the pixel Jacobians of a feature (pose and point) against numeric ones;
//   - the td Jacobian of a clone against a numeric derivative of the pose along the motion;
//   - a frame time between two IMU samples: interpolating there must not change the propagation;
//   - frames delivered at their IMU time for both signs of td (replay_in_imu_time);
//   - moving the origin: world positions and the ordinary covariance unchanged by the shift;
//   - track drift noise: a random walk in the accumulated view change (as the generator makes it),
//     whitened with track_noise(), must have unit covariance, also when the view comes back;
//   - landed zero-velocity updates with the motors running: applied on the ground in spite of the
//     vibration, and not when the flag says "in the air", the image moves or the drone accelerates;
//   - baro update: keeps the height of a drifting IMU, finds the baro offset, and gives no horizontal
//     information (it is never a source of scale);
//   - IMU gap noise (filter.imu_gap_noise): across a 0.5 s gap the velocity variance grows by about
//     (accel sigma x gap)^2 with the switch on; without a gap the switch changes nothing;
//   - static start and zero-velocity updates on a still, tilted IMU with a gyro bias: roll, pitch,
//     yaw, gyro bias, position and velocity must come out right, and the covariance must cover the
//     remaining errors.

#include "vio/filter.h"
#include "vio/imu_propagation.h"
#include "vio/msckf.h"
#include "vio/session.h"

#include <cmath>
#include <cstdio>
#include <random>
#include <string>
#include <tuple>
#include <vector>

namespace {

int failures = 0;

void check(bool ok, const char* what, double value) {
    std::printf("%s %-62s %.3g\n", ok ? "ok  " : "FAIL", what, value);
    if (!ok) ++failures;
}

const Eigen::Vector3d g_ned(0, 0, 9.81);

Eigen::Matrix3d skew_test(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
}

// A smooth motion with rotation and acceleration (body rate and specific force at time t).
Eigen::Vector3d motion_gyro(double t) {
    return Eigen::Vector3d(0.6 * std::sin(1.3 * t), -0.4 + 0.3 * t, 0.8 * std::cos(0.7 * t));
}

// IMU samples of that motion (no noise).
std::vector<vio::ImuSample> motion_samples(const vio::State& s0, double duration, double rate = 200) {
    std::vector<vio::ImuSample> out;
    for (int i = 0; i <= static_cast<int>(duration * rate); ++i) {
        const double t = i / rate;
        vio::ImuSample m;
        m.t = s0.t + static_cast<vio::Timestamp>(std::llround(t * 1e9));
        m.gyro = motion_gyro(t);
        m.accel = Eigen::Vector3d(1.5 * std::cos(t), 0.8, -9.81 + 2.0 * std::sin(2 * t));
        out.push_back(m);
    }
    return out;
}

// x (+) dx in the filter's error convention (invariant: p_true = Exp(dtheta) p + dp).
vio::State plus(vio::State s, int i, double eps) {
    Eigen::Matrix<double, 15, 1> dx = Eigen::Matrix<double, 15, 1>::Zero();
    dx(i) = eps;
    const Eigen::Quaterniond dq = vio::quat_exp(dx.segment<3>(vio::Filter::TH));
    s.q = (dq * s.q).normalized();
    s.p = dq * s.p + dx.segment<3>(vio::Filter::P);
    s.v = dq * s.v + dx.segment<3>(vio::Filter::V);
    s.bg += dx.segment<3>(vio::Filter::BG);
    s.ba += dx.segment<3>(vio::Filter::BA);
    return s;
}

// a (-) b in the filter's error convention.
Eigen::Matrix<double, 15, 1> minus(const vio::State& a, const vio::State& b) {
    Eigen::Matrix<double, 15, 1> d;
    const Eigen::Quaterniond dq = a.q * b.q.conjugate();
    const Eigen::AngleAxisd aa(dq);
    d.segment<3>(vio::Filter::TH) = aa.angle() * aa.axis();
    d.segment<3>(vio::Filter::P) = a.p - dq * b.p;
    d.segment<3>(vio::Filter::V) = a.v - dq * b.v;
    d.segment<3>(vio::Filter::BG) = a.bg - b.bg;
    d.segment<3>(vio::Filter::BA) = a.ba - b.ba;
    return d;
}

vio::State run_mean(const vio::State& s0, const std::vector<vio::ImuSample>& imu) {
    vio::ImuPropagator prop(g_ned);
    prop.reset(s0);
    for (const auto& m : imu) prop.add(m);
    return prop.state();
}

void test_transition() {
    vio::State s0;
    s0.t = 1'000'000'000;
    s0.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d(1, 2, 3).normalized()));
    s0.v = Eigen::Vector3d(3, -1, 0.5);
    const auto imu = motion_samples(s0, 1.0);

    vio::FilterConfig cfg;
    cfg.imu.rate_hz = 200;
    vio::Filter filter(cfg, g_ned);
    filter.init_from_state(s0);  // zero biases, as s0
    for (const auto& m : imu) filter.add_imu(m);
    const Eigen::Matrix<double, 15, 15> Phi = filter.pending_transition();

    const vio::State end = run_mean(s0, imu);
    Eigen::Matrix<double, 15, 15> Phi_num;
    for (int i = 0; i < 15; ++i) {
        const double eps = 1e-6;
        Phi_num.col(i) = (minus(run_mean(plus(s0, i, eps), imu), end) -
                          minus(run_mean(plus(s0, i, -eps), imu), end)) / (2 * eps);
    }
    const double rel = (Phi - Phi_num).norm() / Phi_num.norm();
    check(rel < 1e-4, "transition matrix vs numeric Jacobian over 1 s (relative)", rel);
}

void test_feature_jacobian() {
    vio::CameraInfo ci;
    ci.model = "equidistant";
    ci.fx = ci.fy = 340, ci.cx = 320, ci.cy = 240;
    ci.distortion = {0.02, -0.01, 0.003, -0.0005};
    ci.width = 640, ci.height = 480;
    const vio::Camera cam(ci);
    Eigen::Matrix3d R_bc;
    R_bc << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    const Eigen::Vector3d t_bc(-0.03, 0, 0.05);
    const Eigen::Matrix3d R_wb =
        Eigen::AngleAxisd(0.3, Eigen::Vector3d(1, 2, 3).normalized()).toRotationMatrix();
    const Eigen::Vector3d p_wb(40, -25, -30);  // far from the origin: the invariant terms matter
    const Eigen::Vector3d p_f = p_wb + R_wb * (t_bc + R_bc * Eigen::Vector3d(3, -2, 25));

    Eigen::Vector2d uv;
    Eigen::Matrix<double, 2, 6> Hp, Hp_num;
    Eigen::Matrix<double, 2, 3> Hf, Hf_num;
    vio::project_point(cam, R_bc, t_bc, R_wb, p_wb, p_f, uv, &Hp, &Hf);
    auto pixel = [&](const Eigen::Matrix<double, 9, 1>& d) {  // [dtheta, dp, dp_f]
        const Eigen::Matrix3d dR = vio::quat_exp(d.head<3>()).toRotationMatrix();
        Eigen::Vector2d u;
        vio::project_point(cam, R_bc, t_bc, dR * R_wb, dR * p_wb + d.segment<3>(3),
                           p_f + d.tail<3>(), u, nullptr, nullptr);
        return u;
    };
    for (int i = 0; i < 9; ++i) {
        Eigen::Matrix<double, 9, 1> d = Eigen::Matrix<double, 9, 1>::Zero();
        d(i) = 1e-6;
        const Eigen::Vector2d col = (pixel(d) - pixel(-d)) / 2e-6;
        if (i < 6) Hp_num.col(i) = col;
        else Hf_num.col(i - 6) = col;
    }
    const double rel = std::max((Hp - Hp_num).norm() / Hp_num.norm(), (Hf - Hf_num).norm() / Hf_num.norm());
    check(rel < 1e-6, "feature pixel Jacobians vs numeric (relative)", rel);
}

void test_td_jacobian() {
    // Pose along the motion at t - h, t, t + h (1 kHz samples, h = 1 ms), as invariant errors
    // relative to the pose at t; the derivative must match clone_td_jacobian.
    vio::State s0;
    s0.t = 1'000'000'000;
    s0.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d(1, 2, 3).normalized()));
    s0.p = Eigen::Vector3d(40, -25, -30);  // away from the origin: the [p]x term matters
    s0.v = Eigen::Vector3d(3, -1, 0.5);
    const auto imu = motion_samples(s0, 0.6, 1000);
    vio::ImuPropagator prop(g_ned);
    prop.reset(s0);
    std::vector<vio::State> st;
    for (const auto& m : imu) {
        prop.add(m);
        st.push_back(prop.state());
    }
    const size_t k = 500;  // t = 0.5 s
    auto rel = [&](const vio::State& a) {  // a relative to st[k], invariant error convention
        const Eigen::AngleAxisd aa(a.q * st[k].q.conjugate());
        Eigen::Matrix<double, 6, 1> d;
        d << aa.angle() * aa.axis(), a.p - (a.q * st[k].q.conjugate()) * st[k].p;
        return d;
    };
    const Eigen::Matrix<double, 6, 1> J_num = (rel(st[k + 1]) - rel(st[k - 1])) / 2e-3;
    const Eigen::Matrix<double, 6, 1> J = vio::clone_td_jacobian(st[k], motion_gyro(0.5));
    const double err = (J - J_num).norm() / J_num.norm();
    check(err < 1e-3, "clone td Jacobian vs numeric derivative along the motion (relative)", err);
}

void test_frame_between_samples() {
    // The same 1 s of IMU, once alone and once with a frame time 3.1 ms after every 10th sample
    // (between two samples). Interpolating the IMU there must leave the result unchanged; holding
    // the last sample (the old way) changes it.
    vio::State s0;
    s0.t = 1'000'000'000;
    s0.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d(1, 2, 3).normalized()));
    s0.v = Eigen::Vector3d(3, -1, 0.5);
    const auto imu = motion_samples(s0, 1.0);
    vio::FilterConfig cfg;
    cfg.imu.rate_hz = 200;
    auto run = [&](int frames) {  // 0: no frames, 1: interpolate, 2: hold
        vio::Filter f(cfg, g_ned);
        f.init_from_state(s0);
        for (size_t i = 0; i < imu.size(); ++i) {
            f.add_imu(imu[i]);
            if (frames && i % 10 == 3 && i + 1 < imu.size())
                f.propagate_to(imu[i].t + 3'100'000, frames == 1 ? &imu[i + 1] : nullptr);
        }
        return f.state();
    };
    const vio::State plain = run(0), interp = run(1), hold = run(2);
    const double e_interp = (interp.p - plain.p).norm() + (interp.v - plain.v).norm();
    const double e_hold = (hold.p - plain.p).norm() + (hold.v - plain.v).norm();
    std::printf("     holding the last sample instead: %.2g (position + velocity error)\n", e_hold);
    // Splitting a midpoint step changes it slightly (second order): 1e-5 is that, not a bias.
    check(e_interp < 1e-5 && e_interp < e_hold / 100,
          "frame between two IMU samples: interpolation changes nothing", e_interp);
}

void test_frame_order() {
    // IMU every 5 ms, frames at 12 and 37 ms (camera clock). With td = -4 ms the first frame is at
    // 8 ms in the IMU clock: after the sample at 5, before the one at 10, which it gets as "next".
    vio::Session s;
    for (int i = 0; i <= 10; ++i) s.imu.push_back({i * 5'000'000LL, Eigen::Vector3d::Zero(), Eigen::Vector3d::Zero()});
    s.frames.resize(2);
    s.frames[0].t = 12'000'000;
    s.frames[1].t = 37'000'000;
    auto order = [&](vio::Timestamp td) {
        std::string log;
        vio::replay_in_imu_time(
            s, [&] { return td; }, [&](const vio::ImuSample& m) { log += std::to_string(m.t / 1'000'000) + " "; },
            [&](const vio::CameraFrame& f, const vio::ImuSample* next) {
                log += "F" + std::to_string(f.t / 1'000'000) + ":" +
                       (next ? std::to_string(next->t / 1'000'000) : "-") + " ";
            });
        return log;
    };
    const std::string neg = order(-4'000'000), pos = order(4'000'000), on = order(-2'000'000);
    const bool ok = neg == "0 5 F12:10 10 15 20 25 30 F37:35 35 40 45 50 " &&
                    pos == "0 5 10 15 F12:20 20 25 30 35 40 F37:45 45 50 " &&
                    on == "0 5 10 F12:15 15 20 25 30 35 F37:40 40 45 50 ";
    if (!ok) std::printf("     td -4: %s\n     td +4: %s\n     td -2: %s\n", neg.c_str(), pos.c_str(), on.c_str());
    check(ok, "frames at their IMU time for td -4, +4 and -2 ms (on a sample)", 0);
}

void test_origin_shift() {
    // Far from the world origin, with clones and a landmark in the state: positions in the world
    // and the covariance of the ordinary errors (p_true = p + dp) must be the same just before and
    // just after the origin moves.
    vio::State s0;
    s0.t = 1'000'000'000;
    s0.q = Eigen::Quaterniond(Eigen::AngleAxisd(0.4, Eigen::Vector3d(1, 2, 3).normalized()));
    s0.p = Eigen::Vector3d(3000, -2000, -40);
    s0.v = Eigen::Vector3d(8, -3, 0.5);
    const auto imu = motion_samples(s0, 1.0);
    vio::FilterConfig cfg;
    cfg.imu.rate_hz = 200;
    cfg.imu.gyro_noise_density = 0.004;
    cfg.imu.accel_noise_density = 0.04;
    cfg.origin_shift_distance_m = 0;  // shifted by hand below
    vio::Filter f(cfg, g_ned);
    f.init_from_state(s0);
    for (size_t i = 0; i < imu.size(); ++i) {
        f.add_imu(imu[i]);
        if (i % 40 == 20) f.augment_clone(imu[i].t, imu[i].gyro);
    }
    // A landmark correlated with the IMU pose and the newest clone.
    const int n = static_cast<int>(f.covariance().rows());
    Eigen::MatrixXd A = Eigen::MatrixXd::Zero(3, n);
    A.block<3, 3>(0, vio::Filter::P) = -Eigen::Matrix3d::Identity();
    A.block<3, 3>(0, f.clone_index(f.clones().size() - 1)) = 0.3 * Eigen::Matrix3d::Identity();
    f.add_landmark({7, f.state().p + Eigen::Vector3d(5, -3, 40), s0.t, f.state().p}, A,
                   0.25 * Eigen::Matrix3d::Identity());

    // World positions and the ordinary covariance of [IMU p, theta, v, every clone p, landmark p].
    auto snapshot = [&](Eigen::VectorXd& pos, Eigen::MatrixXd& C) {
        const Eigen::MatrixXd& P = f.covariance();
        const size_t nc = f.clones().size(), m = 9 + 3 * (nc + 1);
        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(m, P.cols());
        pos.resize(3 * (nc + 2));
        const Eigen::Vector3d p = f.state().p;
        M.block<3, 3>(0, vio::Filter::P).setIdentity();
        M.block<3, 3>(0, vio::Filter::TH) = -skew_test(p);
        M.block<3, 3>(3, vio::Filter::TH).setIdentity();
        M.block<3, 3>(6, vio::Filter::V).setIdentity();
        M.block<3, 3>(6, vio::Filter::TH) = -skew_test(f.state().v);
        pos.head<3>() = f.world_state().p;
        for (size_t k = 0; k < nc; ++k) {
            const int i = f.clone_index(k);
            M.block<3, 3>(9 + 3 * k, i + 3).setIdentity();
            M.block<3, 3>(9 + 3 * k, i) = -skew_test(f.clones()[k].p);
            pos.segment<3>(3 + 3 * k) = f.clones()[k].p + f.origin();
        }
        const vio::Landmark& l = f.landmarks()[0];
        M.block<3, 3>(9 + 3 * nc, vio::Filter::landmark_index(0)).setIdentity();
        M.block<3, 3>(9 + 3 * nc, vio::Filter::TH) = -skew_test(l.p);
        pos.tail<3>() = l.p + f.origin();
        C = M * P * M.transpose();
    };
    Eigen::VectorXd pos0, pos1, pos2;
    Eigen::MatrixXd C0, C1, C2;
    snapshot(pos0, C0);
    f.shift_origin(Eigen::Vector3d(1234.5, -567.8, 30.2));
    snapshot(pos1, C1);
    // And the way the replay does it: to the current position (which is passed in as the shift).
    const Eigen::Vector3d world = f.world_state().p;
    f.shift_origin(f.state().p);
    snapshot(pos2, C2);
    const double e_pos = std::max((pos1 - pos0).cwiseAbs().maxCoeff(), (pos2 - pos0).cwiseAbs().maxCoeff());
    const double e_cov = std::max((C1 - C0).cwiseAbs().maxCoeff(), (C2 - C0).cwiseAbs().maxCoeff()) /
                         C0.cwiseAbs().maxCoeff();
    const bool at_drone = (f.origin() - world).norm() < 1e-9 && f.state().p.norm() < 1e-9;
    check(e_pos < 1e-9 && e_cov < 1e-9 && at_drone,
          "origin shift: world positions [m] and ordinary covariance unchanged", std::max(e_pos, e_cov));
}

void test_track_noise() {
    // Cameras fly 30 m past a ground point 25 m below and come back half way: the view since the
    // first observation goes back towards 0, the accumulated view change keeps growing.
    const Eigen::Vector3d p_f(0, 0, 0);
    std::vector<Eigen::Vector3d> cams;
    for (double x : {-15.0, -10.0, -5.0, 0.0, 5.0, 10.0, 15.0, 5.0, -5.0, -15.0})
        cams.emplace_back(x, 2.0, -25.0);
    const std::vector<double> s = vio::accumulated_view_change(cams, p_f);
    const double back = std::acos((p_f - cams[0]).normalized().dot((p_f - cams.back()).normalized()));
    check(back < 1e-9 && s.back() > 100, "view back at the start, accumulated change grows (deg)", s.back());

    // Drift as gen_synthetic.py makes it: per axis a random walk with variance q per degree of each
    // step's view change, white pixel noise on top. Whitened, it must have unit covariance.
    const double sigma_w = 0.5, q = 0.1;
    const int m = static_cast<int>(s.size()), n = 20000;
    const Eigen::LLT<Eigen::MatrixXd> llt(vio::track_noise(s, sigma_w, q));
    std::mt19937 rng(7);
    std::normal_distribution<double> N01(0, 1);
    Eigen::MatrixXd C = Eigen::MatrixXd::Zero(2 * m, 2 * m);
    for (int k = 0; k < n; ++k) {
        Eigen::VectorXd e(2 * m);
        Eigen::Vector2d drift = Eigen::Vector2d::Zero();
        for (int i = 0; i < m; ++i) {
            if (i > 0) drift += std::sqrt(q * (s[i] - s[i - 1])) * Eigen::Vector2d(N01(rng), N01(rng));
            e.segment<2>(2 * i) = drift + sigma_w * Eigen::Vector2d(N01(rng), N01(rng));
        }
        llt.matrixL().solveInPlace(e);
        C += e * e.transpose() / n;
    }
    const double dev = (C - Eigen::MatrixXd::Identity(2 * m, 2 * m)).cwiseAbs().maxCoeff();
    check(dev < 0.05, "whitened drifting track: covariance minus identity, max", dev);
}

void test_landed_zupt() {
    // Level drone standing on the ground with the motors running: accel and gyro shake far more
    // than the "still" thresholds, the mean specific force is still gravity. 5 s, one call per
    // 0.2 s (update frames). Returns the kinds of update applied and the final speed.
    auto run = [](bool landed, bool image_moving, double climb_accel) {
        vio::FilterConfig cfg;
        cfg.imu.rate_hz = 200;
        cfg.zupt_landed = true;  // off by default; the test checks the update when switched on
        vio::Filter filter(cfg, g_ned);
        vio::State s0;
        s0.t = 1'000'000'000LL;
        filter.init_from_state(s0);
        std::mt19937 rng(5);
        std::normal_distribution<double> vib_a(0, 1.0), vib_g(0, 0.03);
        int still = 0, on_ground = 0;
        for (int i = 1; i <= 200 * 5; ++i) {
            vio::ImuSample m;
            m.t = s0.t + i * 5'000'000LL;
            m.gyro = Eigen::Vector3d(vib_g(rng), vib_g(rng), vib_g(rng));
            m.accel = Eigen::Vector3d(0, 0, -9.81 - climb_accel) + Eigen::Vector3d(vib_a(rng), vib_a(rng), vib_a(rng));
            filter.add_imu(m);
            if (i % 40 == 0) {
                const int k = filter.zero_velocity_update(false, image_moving, landed);
                still += k == 1;
                on_ground += k == 2;
            }
        }
        return std::make_tuple(still, on_ground, filter.state().v.norm());
    };
    const auto [still, ground, speed] = run(true, false, 0.0);
    const auto [s1, flag_off, speed_off] = run(false, false, 0.0);
    std::printf("     (landed updates %d of 25, still %d, speed %.3f m/s, without the flag %.3f m/s)\n", ground,
                still, speed, speed_off);
    check(still == 0 && ground >= 20 && speed < 0.05 && speed_off > 5 * speed,
          "landed, motors running: updates applied, speed [m/s]", speed);
    const auto [s2, moving, sp2] = run(true, true, 0.0);
    const auto [s3, climbing, sp3] = run(true, false, 1.0);
    check(flag_off + moving + climbing == 0, "no landed update: flag off, image moving, climbing 1 m/s^2",
          flag_off + moving + climbing);
}

void test_baro() {
    // 60 s of rest in the air with an accel bias the filter does not know (0.05 m/s^2 up): the IMU
    // alone drifts by about 90 m in height. Baro at 5 Hz reads true height + 300 m + 0.3 m noise.
    auto run = [](bool with_baro) {
        vio::FilterConfig cfg;
        cfg.imu.rate_hz = 200;
        vio::Filter filter(cfg, g_ned);
        vio::State s0;
        s0.t = 1'000'000'000LL;
        s0.p = Eigen::Vector3d(10, -20, -40);  // NED: 40 m up
        filter.init_from_state(s0);
        std::mt19937 rng(9);
        std::normal_distribution<double> n_baro(0, 0.3);
        for (int i = 1; i <= 200 * 60; ++i) {
            vio::ImuSample m;
            m.t = s0.t + i * 5'000'000LL;
            m.accel = Eigen::Vector3d(0, 0, -9.81 - 0.05);
            filter.add_imu(m);
            if (with_baro && i % 40 == 0) filter.baro_update(40.0 + 300.0 + n_baro(rng));
        }
        const Eigen::Matrix<double, 9, 9> C = filter.ordinary_covariance();
        return std::make_tuple(std::abs(filter.world_state().p.z() + 40), filter.baro_bias(),
                               std::sqrt(C(0, 0) + C(1, 1)));
    };
    const auto [dz_baro, bias, sigma_xy_baro] = run(true);
    const auto [dz_imu, b0, sigma_xy_imu] = run(false);
    check(dz_baro < 3 && dz_imu > 30 && std::abs(bias - 300) < 3,
          "baro: height error with / without [m] (bias found)", dz_baro);
    check(std::abs(sigma_xy_baro / sigma_xy_imu - 1) < 0.01, "baro: horizontal sigma unchanged (ratio - 1)",
          sigma_xy_baro / sigma_xy_imu - 1);
}

void test_static_start() {
    // Still drone: tilted, yawed, with a gyro bias. 200 Hz IMU with white noise.
    const Eigen::Matrix3d R_true =
        (Eigen::AngleAxisd(0.7, Eigen::Vector3d::UnitZ()) * Eigen::AngleAxisd(-0.05, Eigen::Vector3d::UnitY()) *
         Eigen::AngleAxisd(0.09, Eigen::Vector3d::UnitX())).toRotationMatrix();
    const Eigen::Vector3d bg_true(0.01, -0.02, 0.005);
    vio::FilterConfig cfg;
    cfg.imu.rate_hz = 200;
    cfg.imu.gyro_noise_density = 0.0003;  // still and on the ground: sample spread 0.004 rad/s
    cfg.imu.accel_noise_density = 0.002;  // and 0.03 m/s^2, under the "still" thresholds
    vio::Filter filter(cfg, g_ned);
    std::mt19937 rng(4);
    std::normal_distribution<double> ng(0, cfg.imu.gyro_noise_density * std::sqrt(200.0)),
        na(0, cfg.imu.accel_noise_density * std::sqrt(200.0));

    int zupts = 0;
    for (int i = 0; i < 200 * 6; ++i) {
        vio::ImuSample m;
        m.t = 1'000'000'000LL + i * 5'000'000LL;
        m.gyro = bg_true + Eigen::Vector3d(ng(rng), ng(rng), ng(rng));
        m.accel = R_true.transpose() * (-g_ned) + Eigen::Vector3d(na(rng), na(rng), na(rng));
        filter.add_imu(m);
        if (!filter.initialized()) {
            filter.try_init_static(R_true, Eigen::Vector3d::Zero());
        } else if (i % 7 == 0) {  // about 30 Hz frames
            filter.propagate_to(m.t, nullptr);
            zupts += filter.zupt_if_static(true);
        }
    }
    check(filter.initialized() && zupts > 100, "static start and zero-velocity updates applied", zupts);
    if (!filter.initialized()) return;
    const vio::State& s = filter.state();
    const Eigen::AngleAxisd err(R_true * s.q.toRotationMatrix().transpose());
    const Eigen::MatrixXd& P = filter.covariance();
    const double att_sigma = std::sqrt(P.block<3, 3>(vio::Filter::TH, vio::Filter::TH).trace());

    // Tilt is observable (gravity), yaw only drifts from its reference (gyro noise, bias error).
    const Eigen::Vector3d e = err.angle() * err.axis();  // world frame, NED: z is yaw
    check(e.head<2>().norm() < 0.05 * M_PI / 180, "tilt error after the static start [deg]",
          e.head<2>().norm() * 180 / M_PI);
    check(err.angle() < 3 * att_sigma, "attitude error inside 3 sigma (error / sigma)",
          err.angle() / att_sigma);
    check((s.bg - bg_true).norm() < 3e-4, "gyro bias error [rad/s]", (s.bg - bg_true).norm());
    check(s.v.norm() < 0.01 && s.p.norm() < 0.02, "velocity and position stay at zero [m]", s.p.norm());
    check(Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd>(P).eigenvalues().minCoeff() > 0,
          "covariance positive definite", 0);
}

}  // namespace

// Velocity (index V) or attitude (TH) variance after 2 s of hovering IMU data at 200 Hz, with or
// without a 0.5 s gap; only one of the two gap noise terms on, so each can be checked alone.
double variance_after(int index, bool gap_noise, bool gap, double accel_sigma, double rate_sigma) {
    vio::FilterConfig cfg;
    cfg.gap_noise = gap_noise;
    cfg.gap_accel_sigma = accel_sigma;
    cfg.gap_rate_sigma = rate_sigma;
    vio::Filter f(cfg, g_ned);
    vio::State s0;
    s0.t = 1'000'000'000;
    f.init_from_state(s0);
    for (int i = 0; i <= 400; ++i) {
        vio::ImuSample m;
        m.t = s0.t + static_cast<vio::Timestamp>(i) * 5'000'000;
        if (gap && m.t > s0.t + 1'000'000'000 && m.t < s0.t + 1'500'000'000) continue;
        m.accel = -g_ned;
        f.add_imu(m);
    }
    return f.covariance()(index, index);
}

void test_gap_noise() {
    const double gap = 0.495;  // between the samples on either side
    const int V = vio::Filter::V, TH = vio::Filter::TH;
    const double dv = variance_after(V, true, true, 0.4, 0) - variance_after(V, false, true, 0.4, 0);
    const double dth = variance_after(TH, true, true, 0, 0.04) - variance_after(TH, false, true, 0, 0.04);
    check(std::abs(dv / std::pow(0.4 * gap, 2) - 1) < 0.1 && std::abs(dth / std::pow(0.04 * gap, 2) - 1) < 0.1,
          "IMU gap noise: extra velocity / attitude variance against (sigma x gap)^2", dv / std::pow(0.4 * gap, 2));
    check(variance_after(V, true, false, 0.4, 0.04) == variance_after(V, false, false, 0.4, 0.04),
          "IMU gap noise on, no gap: covariance unchanged", 0);
}

int main() {
    test_transition();
    test_feature_jacobian();
    test_td_jacobian();
    test_frame_between_samples();
    test_frame_order();
    test_origin_shift();
    test_track_noise();
    test_landed_zupt();
    test_baro();
    test_static_start();
    test_gap_noise();
    return failures == 0 ? 0 : 1;
}
