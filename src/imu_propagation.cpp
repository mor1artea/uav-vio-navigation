#include "vio/imu_propagation.h"

#include <cmath>

namespace vio {

Eigen::Quaterniond quat_exp(const Eigen::Vector3d& rotvec) {
    const double angle = rotvec.norm();
    if (angle < 1e-12) {
        // First order is exact enough here, and avoids dividing by zero.
        return Eigen::Quaterniond(1.0, 0.5 * rotvec.x(), 0.5 * rotvec.y(), 0.5 * rotvec.z())
            .normalized();
    }
    return Eigen::Quaterniond(Eigen::AngleAxisd(angle, rotvec / angle));
}

void ImuPropagator::reset(const State& state) {
    state_ = state;
    have_last_ = false;
}

void ImuPropagator::add(const ImuSample& sample) {
    if (!have_last_) {
        // The first sample only sets the "previous" measurement. If it is later than the state,
        // integrate the gap with this sample held constant.
        last_ = sample;
        last_.t = state_.t;
        have_last_ = true;
    }
    if (sample.t > state_.t) integrate(state_, last_, sample);
    last_ = sample;
}

State ImuPropagator::predict(Timestamp t) const {
    State s = state_;
    if (!have_last_ || t <= s.t) return s;
    ImuSample end = last_;
    end.t = t;
    ImuSample start = last_;
    start.t = s.t;
    integrate(s, start, end);
    return s;
}

void ImuPropagator::integrate(State& s, const ImuSample& a, const ImuSample& b) const {
    const double dt = to_sec(b.t - s.t);
    if (dt <= 0) return;

    const Eigen::Vector3d w = 0.5 * (a.gyro + b.gyro) - s.bg;
    const Eigen::Quaterniond q_new = (s.q * quat_exp(w * dt)).normalized();

    const Eigen::Vector3d acc_a = s.q * (a.accel - s.ba) + g_;
    const Eigen::Vector3d acc_b = q_new * (b.accel - s.ba) + g_;
    const Eigen::Vector3d acc = 0.5 * (acc_a + acc_b);

    s.p += s.v * dt + 0.5 * acc * dt * dt;
    s.v += acc * dt;
    s.q = q_new;
    s.t = b.t;
}

}  // namespace vio
