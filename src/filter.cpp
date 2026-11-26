#include "vio/filter.h"

#include <chrono>
#include <cmath>

namespace vio {
namespace {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
}

double ms_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

}  // namespace

double chi2_quantile(double probability, int dof) {
    // Normal quantile (Abramowitz and Stegun 26.2.23, error under 5e-4), then Wilson-Hilferty.
    const double p = 1 - probability;
    const double t = std::sqrt(-2 * std::log(p));
    const double z = t - (2.515517 + 0.802853 * t + 0.010328 * t * t) /
                             (1 + 1.432788 * t + 0.189269 * t * t + 0.001308 * t * t * t);
    const double k = dof, a = 2.0 / (9 * k);
    return k * std::pow(1 - a + z * std::sqrt(a), 3);
}

// ---------------------------------------------------------------- static detector

void StaticDetector::add(const ImuSample& s) {
    buf_.push_back(s);
    while (buf_.front().t < s.t - window_) buf_.pop_front();
}

bool StaticDetector::is_static() const {
    if (buf_.size() < 3 || buf_.back().t - buf_.front().t < window_ * 9 / 10) return false;
    const Eigen::Vector3d ma = mean_accel(), mg = mean_gyro();
    Eigen::Vector3d va = Eigen::Vector3d::Zero(), vg = Eigen::Vector3d::Zero();
    for (const auto& s : buf_) {
        va += (s.accel - ma).cwiseAbs2();
        vg += (s.gyro - mg).cwiseAbs2();
    }
    const double n = static_cast<double>(buf_.size());
    return std::sqrt(va.maxCoeff() / n) < accel_max_ && std::sqrt(vg.maxCoeff() / n) < gyro_max_;
}

Eigen::Vector3d StaticDetector::mean_accel() const {
    Eigen::Vector3d m = Eigen::Vector3d::Zero();
    for (const auto& s : buf_) m += s.accel;
    return m / static_cast<double>(buf_.size());
}

Eigen::Vector3d StaticDetector::mean_gyro() const {
    Eigen::Vector3d m = Eigen::Vector3d::Zero();
    for (const auto& s : buf_) m += s.gyro;
    return m / static_cast<double>(buf_.size());
}

// ---------------------------------------------------------------- filter

Filter::Filter(const FilterConfig& cfg, const Eigen::Vector3d& gravity_world)
    : cfg_(cfg), g_(gravity_world), prop_(gravity_world),
      detector_(cfg.static_window_s, cfg.zupt_accel_std, cfg.zupt_gyro_std) {
    Phi_acc_.setIdentity();
    Q_acc_.setZero();
}

void Filter::add_imu(const ImuSample& s) {
    detector_.add(s);
    if (!initialized_) {
        last_ = s;
        return;
    }
    zupt_gyro_sum_ += s.gyro;
    zupt_accel_sum_ += s.accel;
    ++zupt_gyro_n_;
    propagate_one(s);
}

void Filter::propagate_to(Timestamp t, const ImuSample* next) {
    if (!initialized_ || !have_last_ || t <= prop_.state().t) return;
    // The IMU at time t: interpolated between the last sample and the next one. A camera frame
    // almost never lands on an IMU sample. Holding the last sample instead integrates the wrong
    // rate and acceleration over the gap, and the next sample then starts from the wrong value.
    ImuSample s = last_;
    s.t = t;
    if (next && next->t >= t && next->t > last_.t) {
        const double w = static_cast<double>(t - last_.t) / static_cast<double>(next->t - last_.t);
        s.gyro = (1 - w) * last_.gyro + w * next->gyro;
        s.accel = (1 - w) * last_.accel + w * next->accel;
    }
    propagate_one(s);  // not given to the still detector or the zero-velocity sums
}

Eigen::Matrix<double, 6, 1> clone_td_jacobian(const State& s, const Eigen::Vector3d& rate_body) {
    // A clone taken at t + dtd instead of t: rotated by R w dtd and moved by v dtd. As invariant
    // errors (p_true = Exp(dtheta) p + dp): dtheta = R w dtd, dp = (v + [p]x R w) dtd.
    const Eigen::Vector3d w = s.q * rate_body;  // world frame
    Eigen::Matrix<double, 6, 1> J;
    J << w, s.v + skew(s.p) * w;
    return J;
}

void Filter::propagate_one(const ImuSample& s) {
    // Propagation of the mean and of the covariance, one IMU interval.
    const auto t0 = std::chrono::steady_clock::now();
    const State s0 = prop_.state();
    prop_.add(s);
    const State& s1 = prop_.state();
    const double dt = to_sec(s1.t - s0.t);
    if (dt > 0) {
        // Right-invariant error dynamics, evaluated at the interval midpoint:
        //   dtheta' = -R dbg
        //   dv'     = [g]x dtheta - [v]x R dbg - R dba
        //   dp'     = dv - [p]x R dbg
        // The yaw and position directions the filter cannot observe are constant in these
        // coordinates, so the filter cannot gain fake information about them.
        const Eigen::Matrix3d R = s0.q.slerp(0.5, s1.q).toRotationMatrix();
        const Eigen::Vector3d v = 0.5 * (s0.v + s1.v), p = 0.5 * (s0.p + s1.p);
        const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
        Eigen::Matrix<double, 15, 15> A = Eigen::Matrix<double, 15, 15>::Zero();
        A.block<3, 3>(TH, BG) = -R;
        A.block<3, 3>(P, V) = I;
        A.block<3, 3>(P, BG) = -skew(p) * R;
        A.block<3, 3>(V, TH) = skew(g_);
        A.block<3, 3>(V, BG) = -skew(v) * R;
        A.block<3, 3>(V, BA) = -R;
        const Eigen::Matrix<double, 15, 15> Adt = A * dt;
        const Eigen::Matrix<double, 15, 15> F = Eigen::Matrix<double, 15, 15>::Identity() + Adt +
                                                0.5 * Adt * Adt + Adt * Adt * Adt / 6;

        // Noise: white gyro and accel noise (n_g, n_a) and the bias random walks.
        Eigen::Matrix<double, 15, 12> G = Eigen::Matrix<double, 15, 12>::Zero();
        G.block<3, 3>(TH, 0) = -R;
        G.block<3, 3>(P, 0) = -skew(p) * R;
        G.block<3, 3>(V, 0) = -skew(v) * R;
        G.block<3, 3>(V, 3) = -R;
        G.block<3, 3>(BG, 6) = I;
        G.block<3, 3>(BA, 9) = I;
        Eigen::Matrix<double, 12, 1> qc;
        qc << Eigen::Vector3d::Constant(std::pow(cfg_.imu.gyro_noise_density, 2)),
            Eigen::Vector3d::Constant(std::pow(cfg_.imu.accel_noise_density, 2)),
            Eigen::Vector3d::Constant(std::pow(cfg_.imu.gyro_random_walk, 2)),
            Eigen::Vector3d::Constant(std::pow(cfg_.imu.accel_random_walk, 2));
        const Eigen::Matrix<double, 15, 15> Qd = G * qc.asDiagonal() * G.transpose() * dt;
        const Eigen::Matrix<double, 15, 15> Q = 0.5 * (F * Qd * F.transpose() + Qd);  // trapezoid

        Phi_acc_ = F * Phi_acc_;
        Q_acc_ = F * Q_acc_ * F.transpose() + Q;
        dt_acc_ += dt;
    }
    last_ = s;
    have_last_ = true;
    ms_propagate_ += ms_since(t0);
}

void Filter::apply_propagation() {
    if (dt_acc_ <= 0) return;
    const int n = static_cast<int>(P_.rows());
    if (landmarks_.empty()) {
        P_.topLeftCorner<15, 15>() =
            Phi_acc_ * P_.topLeftCorner<15, 15>() * Phi_acc_.transpose() + Q_acc_;
        if (n > 15) {
            P_.topRightCorner(15, n - 15) = Phi_acc_ * P_.topRightCorner(15, n - 15);
            P_.bottomLeftCorner(n - 15, 15) = P_.topRightCorner(15, n - 15).transpose();
        }
    } else {
        // Landmarks follow the change of the IMU attitude error (see filter.h):
        //   dp_f_new = dp_f + [p_f]x (dtheta_new - dtheta),  dtheta_new = Phi_theta * x + w_theta
        Eigen::MatrixXd T = Eigen::MatrixXd::Identity(n, n);
        T.topLeftCorner<15, 15>() = Phi_acc_;
        Eigen::MatrixXd Gw = Eigen::MatrixXd::Zero(n, 15);  // how the IMU noise enters
        Gw.topRows<15>().setIdentity();
        Eigen::Matrix<double, 3, 15> d_theta = Phi_acc_.topRows<3>();
        d_theta.leftCols<3>() -= Eigen::Matrix3d::Identity();
        for (size_t j = 0; j < landmarks_.size(); ++j) {
            const Eigen::Matrix3d S = skew(landmarks_[j].p);
            const int i = landmark_index(j);
            T.block(i, 0, 3, 15) = S * d_theta;
            Gw.block(i, TH, 3, 3) = S;
        }
        P_ = T * P_ * T.transpose() + Gw * Q_acc_ * Gw.transpose();
    }
    Phi_acc_.setIdentity();
    Q_acc_.setZero();
    dt_acc_ = 0;
}

void Filter::augment_clone(Timestamp key, const Eigen::Vector3d& gyro) {
    apply_propagation();
    const State& s = prop_.state();
    clones_.push_back({key, s.q, s.p});

    // New clone error = J * old error: same attitude and position errors, plus the td term.
    const int n = static_cast<int>(P_.rows());
    Eigen::MatrixXd J = Eigen::MatrixXd::Zero(6, n);
    J.block<3, 3>(0, TH).setIdentity();
    J.block<3, 3>(3, P).setIdentity();
    J.block<6, 1>(0, TD) = clone_td_jacobian(s, gyro - s.bg);
    const Eigen::MatrixXd JP = J * P_;
    P_.conservativeResize(n + 6, n + 6);
    P_.block(n, 0, 6, n) = JP;
    P_.block(0, n, n, 6) = JP.transpose();
    P_.block(n, n, 6, 6) = JP * J.transpose();
}

void Filter::remove_block(int i, int size) {
    // Marginalizing a Gaussian = dropping its rows and columns.
    const int n = static_cast<int>(P_.rows()), tail = n - i - size;
    P_.block(i, 0, tail, n) = P_.block(i + size, 0, tail, n).eval();
    P_.block(0, i, n, tail) = P_.block(0, i + size, n, tail).eval();
    P_.conservativeResize(n - size, n - size);
}

void Filter::remove_clone(size_t k) {
    apply_propagation();
    remove_block(clone_index(k), 6);
    clones_.erase(clones_.begin() + static_cast<long>(k));
}

void Filter::add_landmark(const Landmark& l, const Eigen::MatrixXd& A, const Eigen::Matrix3d& noise) {
    apply_propagation();
    const int n = static_cast<int>(P_.rows());
    const int at = CORE + 3 * static_cast<int>(landmarks_.size());  // after the landmarks, before clones
    const Eigen::MatrixXd AP = A * P_;
    Eigen::MatrixXd big(n + 3, n + 3);
    big.topLeftCorner(n, n) = P_;
    big.block(n, 0, 3, n) = AP;
    big.block(0, n, n, 3) = AP.transpose();
    big.block(n, n, 3, 3) = AP * A.transpose() + noise;
    std::vector<int> order;  // move the new block from the end to position "at"
    for (int i = 0; i < at; ++i) order.push_back(i);
    for (int i = n; i < n + 3; ++i) order.push_back(i);
    for (int i = at; i < n; ++i) order.push_back(i);
    P_ = big(order, order);
    landmarks_.push_back(l);
}

bool Filter::baro_update(double altitude) {
    if (!cfg_.baro || !initialized_) return false;
    apply_propagation();
    const State& s = prop_.state();
    const Eigen::Vector3d up = -g_.normalized();
    const double height = up.dot(s.p + origin_);
    if (!baro_seen_) {  // the first reading only sets the bias
        baro_bias_ = altitude - height;
        baro_seen_ = true;
        last_baro_t_ = s.t;
        return false;
    }
    P_(BB, BB) += cfg_.baro_bias_random_walk * cfg_.baro_bias_random_walk * to_sec(s.t - last_baro_t_);
    last_baro_t_ = s.t;
    double sigma = cfg_.baro_noise;
    if (have_ground_ && height - ground_height_ < cfg_.baro_inflate_below_agl) sigma *= cfg_.baro_inflate_factor;
    if (std::abs(up.dot(s.v)) > cfg_.baro_inflate_above_climb) sigma *= cfg_.baro_inflate_factor;
    // h(p_true) = up . (Exp(dtheta) p + dp + origin): d/d dtheta = -up^T [p]x (p relative to the
    // origin, so this coupling stays small), d/d dp = up^T, d/d bias = 1.
    Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, P_.rows());
    H.block<1, 3>(0, TH) = -up.transpose() * skew(s.p);
    H.block<1, 3>(0, P) = up.transpose();
    H(0, BB) = 1;
    Eigen::VectorXd r(1);
    r(0) = altitude - (height + baro_bias_);
    return update(H, r, Eigen::MatrixXd::Constant(1, 1, sigma * sigma), true);
}

void Filter::add_landmark_noise(size_t j, const Eigen::Matrix3d& Q) {
    apply_propagation();
    const int i = landmark_index(j);
    P_.block<3, 3>(i, i) += Q;
}

void Filter::remove_landmark(size_t j) {
    apply_propagation();
    remove_block(landmark_index(j), 3);
    landmarks_.erase(landmarks_.begin() + static_cast<long>(j));
}

void Filter::priors_to_invariant() {
    // The priors are written for ordinary errors (p_true = p + dp). Invariant errors:
    // dp_inv = dp + [p]x dtheta, dv_inv = dv + [v]x dtheta.
    const State& s = prop_.state();
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(P_.rows(), P_.cols());
    T.block<3, 3>(P, TH) = skew(s.p);
    T.block<3, 3>(V, TH) = skew(s.v);
    P_ = T * P_ * T.transpose();
}

State Filter::world_state() const {
    State s = prop_.state();
    s.p += origin_;
    return s;
}

void Filter::shift_origin(Eigen::Vector3d d) {
    apply_propagation();  // the accumulated transition was built with the old positions
    // p_true - d = Exp(dtheta) (p - d) + dp'  gives  dp' = dp + (Exp(dtheta) - I) d ~ dp - [d]x dtheta.
    const Eigen::Matrix3d D = skew(d);
    Eigen::MatrixXd T = Eigen::MatrixXd::Identity(P_.rows(), P_.cols());
    T.block<3, 3>(P, TH) = -D;
    for (size_t j = 0; j < landmarks_.size(); ++j) {  // landmarks use the IMU attitude error
        T.block<3, 3>(landmark_index(j), TH) = -D;
        landmarks_[j].p -= d;
        landmarks_[j].camera_at_birth -= d;
    }
    for (size_t k = 0; k < clones_.size(); ++k) {  // each clone has its own attitude error
        const int i = clone_index(k);
        T.block<3, 3>(i + 3, i) = -D;
        clones_[k].p -= d;
    }
    P_ = T * P_ * T.transpose();
    State s = prop_.state();
    s.p -= d;
    prop_.set_state(s);
    origin_ += d;
}

bool Filter::shift_origin_if_far() {
    if (cfg_.origin_shift_distance_m <= 0 || state().p.norm() <= cfg_.origin_shift_distance_m)
        return false;
    shift_origin(state().p);
    return true;
}

Eigen::Matrix<double, 9, 9> Filter::ordinary_covariance() {
    // Back to ordinary errors, order [p, theta, v]: dp = dp_inv - [p]x dtheta, dv likewise.
    apply_propagation();
    const State& s = prop_.state();
    Eigen::MatrixXd M = Eigen::MatrixXd::Zero(9, P_.cols());
    M.block<3, 3>(0, P).setIdentity();
    M.block<3, 3>(0, TH) = -skew(s.p);
    M.block<3, 3>(3, TH).setIdentity();
    M.block<3, 3>(6, V).setIdentity();
    M.block<3, 3>(6, TH) = -skew(s.v);
    return M * P_ * M.transpose();
}

const Eigen::MatrixXd& Filter::covariance() {
    apply_propagation();
    return P_;
}

bool Filter::try_init_static(const Eigen::Matrix3d& R_ref, const Eigen::Vector3d& p0) {
    if (initialized_) return true;
    if (!detector_.is_static()) return false;

    // Roll and pitch: the mean specific force points up in the world.
    const Eigen::Vector3d up = -g_.normalized();
    const Eigen::Quaterniond q_tilt =
        Eigen::Quaterniond::FromTwoVectors(detector_.mean_accel().normalized(), up);
    // Yaw: the turn about the vertical that best matches all three body axes to R_ref (least
    // squares). Using one axis alone fails when that axis points nearly up, as x does in EuRoC.
    const Eigen::Matrix3d R_tilt = q_tilt.toRotationMatrix();
    double sin_sum = 0, cos_sum = 0;
    for (int i = 0; i < 3; ++i) {
        const Eigen::Vector3d a = R_tilt.col(i) - up * up.dot(R_tilt.col(i));
        const Eigen::Vector3d b = R_ref.col(i) - up * up.dot(R_ref.col(i));
        sin_sum += up.dot(a.cross(b));
        cos_sum += a.dot(b);
    }
    const double yaw = std::atan2(sin_sum, cos_sum);

    State s;
    s.t = last_.t;
    s.q = (Eigen::Quaterniond(Eigen::AngleAxisd(yaw, up)) * q_tilt).normalized();
    origin_ = p0;  // positions inside the filter are relative to the start
    ground_height_ = -g_.normalized().dot(p0);
    have_ground_ = true;
    s.bg = detector_.mean_gyro();
    prop_.reset(s);

    // Covariance. The tilt comes from the accelerometer, so a horizontal accel bias shows up as a
    // tilt error: dtheta = -(1/|g|) [g_dir]x R dba. Keeping this correlation makes the start
    // consistent; without it the filter would believe tilt and bias are independently known.
    const Eigen::Matrix3d R = s.q.toRotationMatrix();
    const Eigen::Matrix3d A = -skew(g_.normalized()) * R / g_.norm();
    const Eigen::Matrix3d S_ba = cfg_.sigma_ba * cfg_.sigma_ba * Eigen::Matrix3d::Identity();
    const double T = cfg_.static_window_s;
    const double tilt_noise = cfg_.imu.accel_noise_density / std::sqrt(T) / g_.norm() + 1e-4;
    const double yaw_sigma = cfg_.sigma_yaw_deg * M_PI / 180;
    const Eigen::Matrix3d vert = up * up.transpose();

    P_ = Eigen::MatrixXd::Zero(CORE, CORE);
    P_.block<3, 3>(TH, TH) = A * S_ba * A.transpose() +
                             tilt_noise * tilt_noise * (Eigen::Matrix3d::Identity() - vert) +
                             yaw_sigma * yaw_sigma * vert;
    P_.block<3, 3>(TH, BA) = A * S_ba;
    P_.block<3, 3>(BA, TH) = (A * S_ba).transpose();
    P_.block<3, 3>(BA, BA) = S_ba;
    P_.block<3, 3>(P, P) = cfg_.sigma_pos * cfg_.sigma_pos * Eigen::Matrix3d::Identity();
    P_.block<3, 3>(V, V) =
        cfg_.zupt_velocity_noise * cfg_.zupt_velocity_noise * Eigen::Matrix3d::Identity();
    const double sbg = cfg_.imu.gyro_noise_density / std::sqrt(T) + 1e-5;
    P_.block<3, 3>(BG, BG) = sbg * sbg * Eigen::Matrix3d::Identity();
    P_(TD, TD) = cfg_.sigma_td_s * cfg_.sigma_td_s;
    P_(BB, BB) = cfg_.sigma_baro_bias * cfg_.sigma_baro_bias;
    td_ = cfg_.td_initial_s;
    priors_to_invariant();

    initialized_ = true;
    have_last_ = false;
    zupt_gyro_sum_.setZero();
    zupt_accel_sum_.setZero();
    zupt_gyro_n_ = 0;
    return true;
}

void Filter::init_from_state(const State& s0) {
    State s = s0;
    s.bg.setZero();  // biases unknown at the start, covered by their priors
    s.ba.setZero();
    origin_ = s0.p;  // positions inside the filter are relative to the start
    ground_height_ = -g_.normalized().dot(s0.p);
    have_ground_ = s0.v.norm() < 0.01;  // started at rest: on the ground
    s.p.setZero();
    prop_.reset(s);
    const Eigen::Matrix3d I = Eigen::Matrix3d::Identity();
    const double att = cfg_.sigma_att_deg * M_PI / 180;
    P_ = Eigen::MatrixXd::Zero(CORE, CORE);
    P_.block<3, 3>(TH, TH) = att * att * I;
    P_.block<3, 3>(P, P) = cfg_.sigma_pos * cfg_.sigma_pos * I;
    P_.block<3, 3>(V, V) = cfg_.sigma_vel * cfg_.sigma_vel * I;
    P_.block<3, 3>(BG, BG) = cfg_.sigma_bg * cfg_.sigma_bg * I;
    P_.block<3, 3>(BA, BA) = cfg_.sigma_ba * cfg_.sigma_ba * I;
    P_(TD, TD) = cfg_.sigma_td_s * cfg_.sigma_td_s;
    P_(BB, BB) = cfg_.sigma_baro_bias * cfg_.sigma_baro_bias;
    td_ = cfg_.td_initial_s;
    priors_to_invariant();
    initialized_ = true;
    have_last_ = false;
}

int Filter::zero_velocity_update(bool image_still, bool image_moving, bool landed) {
    int applied = 0;
    if (cfg_.zupt && initialized_ && image_still && detector_.is_static() && zupt_gyro_n_ > 0) {
        const int n = static_cast<int>(P_.rows());
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(6, n);
        H.block<3, 3>(0, V).setIdentity();
        H.block<3, 3>(0, TH) = -skew(prop_.state().v);  // v = Exp(dtheta) v_est + dv
        H.block<3, 3>(3, BG).setIdentity();
        Eigen::VectorXd r(6);
        const State& s = prop_.state();
        r.head<3>() = -s.v;
        r.tail<3>() = zupt_gyro_sum_ / zupt_gyro_n_ - s.bg;
        // Mean of n gyro samples: white noise variance density^2 * rate / n.
        const double sg2 = std::pow(cfg_.imu.gyro_noise_density, 2) * cfg_.imu.rate_hz / zupt_gyro_n_;
        Eigen::MatrixXd R = Eigen::MatrixXd::Zero(6, 6);
        R.diagonal() << Eigen::Vector3d::Constant(std::pow(cfg_.zupt_velocity_noise, 2)),
            Eigen::Vector3d::Constant(sg2);
        applied = update(H, r, R, true) ? 1 : 0;
    } else if (cfg_.zupt_landed && initialized_ && landed && !image_moving && zupt_gyro_n_ > 0 &&
               std::abs((zupt_accel_sum_ / zupt_gyro_n_).norm() - g_.norm()) <= cfg_.zupt_landed_accel_max) {
        const int n = static_cast<int>(P_.rows());
        Eigen::MatrixXd H = Eigen::MatrixXd::Zero(3, n);
        H.block<3, 3>(0, V).setIdentity();
        H.block<3, 3>(0, TH) = -skew(prop_.state().v);
        const Eigen::Vector3d r = -prop_.state().v;
        const Eigen::Matrix3d R = std::pow(cfg_.zupt_velocity_noise, 2) * Eigen::Matrix3d::Identity();
        // Not gated: the vibration is not in the noise model, so the predicted velocity is too
        // confident on the ground and a gate would reject the true v = 0. The flag, the image and
        // the accelerometer decide instead.
        applied = update(H, r, R, false) ? 2 : 0;
    }
    zupt_gyro_sum_.setZero();
    zupt_accel_sum_.setZero();
    zupt_gyro_n_ = 0;
    return applied;
}

bool Filter::update(const Eigen::MatrixXd& H, const Eigen::VectorXd& r, const Eigen::MatrixXd& R,
                    bool gate) {
    apply_propagation();
    const auto t0 = std::chrono::steady_clock::now();
    const Eigen::MatrixXd HP = H * P_;
    const Eigen::MatrixXd S = HP * H.transpose() + R;
    const Eigen::LDLT<Eigen::MatrixXd> S_ldlt(S);
    if (gate) {
        const double chi2 = r.dot(S_ldlt.solve(r));
        if (chi2 > chi2_quantile(cfg_.chi2_probability, static_cast<int>(r.size()))) {
            ms_update_ += ms_since(t0);
            return false;
        }
    }
    const Eigen::MatrixXd K = S_ldlt.solve(HP).transpose();  // P H^T S^-1, P symmetric
    correct(K * r);
    P_ -= K * HP;
    P_ = 0.5 * (P_ + P_.transpose()).eval();
    ms_update_ += ms_since(t0);
    return true;
}

void Filter::correct(const Eigen::VectorXd& dx) {
    State s = prop_.state();
    const Eigen::Quaterniond dq = quat_exp(dx.segment<3>(TH));
    s.q = (dq * s.q).normalized();
    s.p = dq * s.p + dx.segment<3>(P);
    s.v = dq * s.v + dx.segment<3>(V);
    s.bg += dx.segment<3>(BG);
    s.ba += dx.segment<3>(BA);
    prop_.set_state(s);
    td_ += dx(TD);
    baro_bias_ += dx(BB);
    for (size_t j = 0; j < landmarks_.size(); ++j)
        landmarks_[j].p = dq * landmarks_[j].p + dx.segment<3>(landmark_index(j));
    for (size_t k = 0; k < clones_.size(); ++k) {
        const int i = clone_index(k);
        const Eigen::Quaterniond dqk = quat_exp(dx.segment<3>(i));
        clones_[k].q = (dqk * clones_[k].q).normalized();
        clones_[k].p = dqk * clones_[k].p + dx.segment<3>(i + 3);
    }
}

void Filter::restore(const Filter& saved) {
    const double mp = ms_propagate_, mu = ms_update_;
    *this = saved;
    ms_propagate_ = mp;
    ms_update_ = mu;
}

double Filter::take_ms_propagate() {
    const double v = ms_propagate_;
    ms_propagate_ = 0;
    return v;
}

double Filter::take_ms_update() {
    const double v = ms_update_;
    ms_update_ = 0;
    return v;
}

}  // namespace vio
