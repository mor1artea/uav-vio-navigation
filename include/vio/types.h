// Basic data types shared by the replay, the front end and the filter.
//
// Conventions:
//   - time stamps are int64 nanoseconds;
//   - quaternions are Hamilton, stored w, x, y, z in files;
//   - "body" is the IMU frame, q_wb rotates body vectors into the world frame.
#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <cstdint>
#include <string>
#include <vector>

namespace vio {

using Timestamp = int64_t;  // nanoseconds

inline double to_sec(Timestamp t) { return static_cast<double>(t) * 1e-9; }

struct ImuSample {
    Timestamp t = 0;
    Eigen::Vector3d gyro = Eigen::Vector3d::Zero();   // rad/s, body frame
    Eigen::Vector3d accel = Eigen::Vector3d::Zero();  // specific force, m/s^2, body frame
};

// One pose of the IMU in the world frame, with velocity and biases if known.
struct State {
    Timestamp t = 0;
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();  // q_wb
    Eigen::Vector3d v = Eigen::Vector3d::Zero();
    Eigen::Vector3d bg = Eigen::Vector3d::Zero();  // gyro bias
    Eigen::Vector3d ba = Eigen::Vector3d::Zero();  // accel bias
};

// A feature observation in pixels on the raw (distorted) image.
struct FeatureObs {
    int64_t id = 0;
    Eigen::Vector2d uv = Eigen::Vector2d::Zero();
};

// One camera frame: either an image file (real data) or ready-made feature
// observations (synthetic data, which has no images).
struct CameraFrame {
    Timestamp t = 0;
    std::string image_path;           // empty for synthetic data
    std::vector<FeatureObs> features;  // empty for real data
};

struct CameraInfo {
    std::string model;                // "radial-tangential" or "equidistant"
    double fx = 0, fy = 0, cx = 0, cy = 0;
    std::vector<double> distortion;   // k1 k2 p1 p2 or k1 k2 k3 k4
    int width = 0, height = 0;
    double rate_hz = 0;
    Eigen::Matrix4d T_body_cam = Eigen::Matrix4d::Identity();
};

struct ImuNoise {
    double gyro_noise_density = 0;   // rad/s/sqrt(Hz)
    double gyro_random_walk = 0;     // rad/s^2/sqrt(Hz)
    double accel_noise_density = 0;  // m/s^2/sqrt(Hz)
    double accel_random_walk = 0;    // m/s^3/sqrt(Hz)
    double rate_hz = 0;
};

}  // namespace vio
