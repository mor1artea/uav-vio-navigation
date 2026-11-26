// Camera models: projection of camera-frame points to raw (distorted) pixels and back.
//
// Two models, the same math as OpenCV:
//   radial-tangential  pinhole + k1, k2, p1, p2 (cv::projectPoints with 4 coefficients). EuRoC.
//   Kannala-Brandt     equidistant fisheye, theta_d = theta (1 + k1 theta^2 + ... + k4 theta^8)
//                      (cv::fisheye). Our ~120 deg lens, final choice after calibration.
//
// Camera frame: x right, y down, z along the optical axis. The rest of the system works with unit
// bearing vectors, never with normalized image coordinates, so rays at and beyond 90 deg from the
// optical axis are fine for the fisheye model.
#pragma once

#include "vio/types.h"

#include <string>

namespace vio {

class Camera {
public:
    enum class Model { RadTan, KannalaBrandt };

    // Accepts the EuRoC / Kalibr names ("radial-tangential", "equidistant") and the ones in
    // config/vio.yaml ("radtan", "kannala_brandt"). Throws std::runtime_error for anything else.
    explicit Camera(const CameraInfo& info);

    // Pixel of a camera-frame point. Returns false if the point cannot be projected (behind the
    // camera for radial-tangential, exactly on the axis behind for the fisheye). J, if given, gets
    // d(uv)/d(point). The pixel may be outside the image; check with in_image().
    bool project(const Eigen::Vector3d& p, Eigen::Vector2d& uv,
                 Eigen::Matrix<double, 2, 3>* J = nullptr) const;

    // Unit bearing of a raw pixel. Returns false if the distortion cannot be inverted there.
    bool unproject(const Eigen::Vector2d& uv, Eigen::Vector3d& bearing) const;

    bool in_image(const Eigen::Vector2d& uv, double border = 0) const {
        return uv.x() >= border && uv.y() >= border && uv.x() <= width_ - 1 - border &&
               uv.y() <= height_ - 1 - border;
    }

    Model model() const { return model_; }
    int width() const { return width_; }
    int height() const { return height_; }
    double fx() const { return fx_; }

private:
    // Distortion on normalized coordinates (radial-tangential) with its 2x2 Jacobian.
    Eigen::Vector2d distort_radtan(const Eigen::Vector2d& xy, Eigen::Matrix2d* J) const;
    // theta_d(theta) of the fisheye and its derivative.
    double theta_d(double theta, double* dtheta_d) const;

    Model model_;
    double fx_, fy_, cx_, cy_;
    double k_[4] = {0, 0, 0, 0};  // radtan: k1 k2 p1 p2; fisheye: k1 k2 k3 k4
    int width_, height_;
};

}  // namespace vio
