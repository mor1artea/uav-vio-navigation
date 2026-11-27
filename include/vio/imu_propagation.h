// IMU strapdown propagation of the mean state (no covariance yet).
//
// Midpoint integration between two IMU samples: average angular rate for the rotation, and the
// average of the world-frame accelerations at both ends for velocity and position. The filter
// will reuse this for its mean propagation.
#pragma once

#include "vio/types.h"

#include <vector>

namespace vio {

// Accel delay compensation (step 4 piece 3a; ARDUPILOT_SETUP.md section 2). The FC filters the accel
// more than the gyro (20 Hz against 80 Hz low-pass, notches on the gyro only), so in its stream the
// accel lags the gyro by shift_s = accel delay - gyro delay (about 6 ms). Each sample gets the accel
// of the stream interpolated at t + shift_s, so both describe the same moment; the last samples,
// which have nothing that late to take it from, are dropped. Returns how many were dropped.
size_t shift_accel(std::vector<ImuSample>& imu, double shift_s);

// Rotation for a rotation vector (axis * angle), exact for any angle.
Eigen::Quaterniond quat_exp(const Eigen::Vector3d& rotvec);

class ImuPropagator {
public:
    explicit ImuPropagator(const Eigen::Vector3d& gravity_world) : g_(gravity_world) {}

    // Starts from a known state. The next sample given to add() must be at or after state.t.
    void reset(const State& state);

    // Replaces the current state (after a filter correction) at the same time, keeping the last
    // IMU sample for the next integration step.
    void set_state(const State& state) { state_ = state; }

    // Integrates up to the time of this sample.
    void add(const ImuSample& sample);

    // State at time t >= current time, integrated from the last sample with the last measured
    // rates held constant. Does not change the internal state.
    State predict(Timestamp t) const;

    const State& state() const { return state_; }

private:
    void integrate(State& s, const ImuSample& a, const ImuSample& b) const;

    Eigen::Vector3d g_;
    State state_;
    ImuSample last_;
    bool have_last_ = false;
};

}  // namespace vio
