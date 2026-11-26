// IMU strapdown propagation of the mean state (no covariance yet).
//
// Midpoint integration between two IMU samples: average angular rate for the rotation, and the
// average of the world-frame accelerations at both ends for velocity and position. The filter
// will reuse this for its mean propagation.
#pragma once

#include "vio/types.h"

namespace vio {

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
