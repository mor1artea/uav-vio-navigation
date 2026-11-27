// Fault injection for replays (step 4, piece 2; DESIGN.md section 11, "Step 4 plan"). Faults are
// made while replaying, so they work on every session (synthetic, renders, EuRoC, Gazebo) and
// nothing new is written to disk.
//
// vio_replay --inject <spec>, repeatable. Times in seconds from the session's first IMU sample (the
// same reference as --start):
//   blackout:T:D            no features from T for D seconds (feature sessions: none delivered;
//                           image sessions: the tracker gets a black image)
//   frozen:T:D              the frame at T is repeated until T + D (its features, or its image)
//   outliers:T:D:SHARE:PX   from T for D seconds, each feature observation with probability SHARE is
//                           moved by PX pixels in a random direction (seeded, repeatable)
//   imu_gap:T:D             IMU samples in [T, T + D) removed from the session before the replay, so
//                           the filter, the tracker's gyro prediction and the quality monitor all
//                           see the same gap
//   td_step:T:MS            camera time stamps from T on shifted by MS milliseconds (a sudden change
//                           of the camera-IMU offset); frames that would go back in time are dropped
#pragma once

#include "vio/session.h"
#include "vio/types.h"

#include <random>
#include <string>
#include <vector>

namespace vio {

struct Fault {
    enum Kind { BLACKOUT = 0, FROZEN = 1, OUTLIERS = 2, IMU_GAP = 3, TD_STEP = 4 };
    Kind kind = BLACKOUT;
    double t_s = 0;      // start, s from the first IMU sample
    double dur_s = 0;    // duration (not used by td_step: it lasts)
    double share = 0;    // outliers
    double px = 0;       // outliers
    double td_ms = 0;    // td_step
};

// Parses one spec as above; throws std::runtime_error with a readable message.
Fault parse_fault(const std::string& spec);

// Whether this fault is active at time t (camera time for frame faults); td_step lasts.
bool fault_active(const Fault& f, Timestamp t, Timestamp t_ref);

// Bit (1 << kind) for every fault active at time t (camera time for frame faults); t_ref is the first
// IMU sample. td_step counts as active from its start on.
unsigned active_faults(const std::vector<Fault>& faults, Timestamp t, Timestamp t_ref);

// The faults that change the session itself: imu_gap and td_step. Call once after loading.
void apply_session_faults(Session& s, const std::vector<Fault>& faults, Timestamp t_ref);

// Moves each observation with probability share by px pixels in a random direction.
void move_outliers(std::vector<FeatureObs>& features, double share, double px, std::mt19937& rng);

// Every outliers fault active at time t, each with its own share and distance.
void apply_outliers(const std::vector<Fault>& faults, Timestamp t, Timestamp t_ref,
                    std::vector<FeatureObs>& features, std::mt19937& rng);

}  // namespace vio
