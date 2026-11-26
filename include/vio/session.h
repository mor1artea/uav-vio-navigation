// Reading a recorded or generated session from disk.
//
// The layout is the EuRoC "ASL" folder layout, so EuRoC sequences and our synthetic sessions are
// read by the same code:
//
//   <session>/mav0/imu0/data.csv                         IMU samples
//   <session>/mav0/imu0/sensor.yaml                      IMU noise values
//   <session>/mav0/cam0/sensor.yaml                      camera model and camera-IMU extrinsics
//   <session>/mav0/cam0/data.csv + data/*.png            images (EuRoC), or
//   <session>/mav0/cam0/features.csv                     feature observations (synthetic)
//   <session>/mav0/state_groundtruth_estimate0/data.csv  ground truth (optional)
//   <session>/session.yaml                               optional, e.g. gravity in the world frame
#pragma once

#include "vio/types.h"

#include <functional>
#include <string>
#include <vector>

namespace vio {

struct Session {
    std::string path;
    std::vector<ImuSample> imu;
    std::vector<CameraFrame> frames;
    std::vector<State> ground_truth;  // empty if the session has none
    // FC landed state (EXTENDED_SYS_STATE.landed_state, MAV_LANDED_STATE: 1 on the ground, 2 in the
    // air, 3 taking off, 4 landing) at the FC send time in the IMU clock, from mav0/landed0/data.csv.
    // Empty if the session has none (EuRoC).
    std::vector<std::pair<Timestamp, int>> landed;
    // Baro altitude in m (up, any offset) in the IMU clock, from mav0/baro0/data.csv. Empty if none.
    std::vector<std::pair<Timestamp, double>> baro;
    CameraInfo camera;
    ImuNoise imu_noise;
    // EuRoC uses a z-up world, so gravity is (0, 0, -9.81). Our own sessions use NED and say so
    // in session.yaml: (0, 0, +9.81).
    Eigen::Vector3d gravity_world = Eigen::Vector3d(0, 0, -9.81);
};

// Reads the whole session into memory. Throws std::runtime_error with a readable message if a
// required file is missing or malformed. features_file, if given, replaces the camera data with
// feature observations from that file (for example the output of vio_track on EuRoC images).
Session load_session(const std::string& path, const std::string& features_file = "");

// Calls the handlers for all IMU samples and camera frames in time order. At equal time stamps the
// IMU sample comes first, so the filter has the IMU data up to the frame time.
void replay(const Session& session,
            const std::function<void(const ImuSample&)>& on_imu,
            const std::function<void(const CameraFrame&)>& on_frame);

// Like replay(), but each camera frame is delivered at its time in the IMU clock, frame.t + td(),
// with td read at that moment (it changes when it is estimated online): after the IMU samples at or
// before that time and before the later ones, for either sign of td. The frame handler also gets
// the first IMU sample after the frame time (nullptr at the end of the data), so the IMU can be
// interpolated at the frame time instead of holding the last sample. td = t_imu - t_cam.
void replay_in_imu_time(const Session& session, const std::function<Timestamp()>& td,
                        const std::function<void(const ImuSample&)>& on_imu,
                        const std::function<void(const CameraFrame&, const ImuSample* next)>& on_frame);

// Ground truth interpolated at time t (linear for position and velocity, slerp for rotation).
// Returns false if t is outside the ground truth.
bool interpolate_ground_truth(const std::vector<State>& gt, Timestamp t, State& out);

}  // namespace vio
