#include "vio/camera_model.h"

#include <cmath>
#include <stdexcept>

namespace vio {

Camera::Camera(const CameraInfo& info)
    : fx_(info.fx), fy_(info.fy), cx_(info.cx), cy_(info.cy), width_(info.width),
      height_(info.height) {
    if (info.model == "radial-tangential" || info.model == "radtan") {
        model_ = Model::RadTan;
    } else if (info.model == "equidistant" || info.model == "kannala_brandt") {
        model_ = Model::KannalaBrandt;
    } else {
        throw std::runtime_error("unknown camera model '" + info.model + "'");
    }
    if (info.distortion.size() != 4)
        throw std::runtime_error("camera model needs exactly 4 distortion coefficients");
    for (int i = 0; i < 4; ++i) k_[i] = info.distortion[i];
}

Eigen::Vector2d Camera::distort_radtan(const Eigen::Vector2d& xy, Eigen::Matrix2d* J) const {
    const double x = xy.x(), y = xy.y();
    const double k1 = k_[0], k2 = k_[1], p1 = k_[2], p2 = k_[3];
    const double r2 = x * x + y * y;
    const double radial = 1 + k1 * r2 + k2 * r2 * r2;
    const Eigen::Vector2d d(x * radial + 2 * p1 * x * y + p2 * (r2 + 2 * x * x),
                            y * radial + p1 * (r2 + 2 * y * y) + 2 * p2 * x * y);
    if (J) {
        const double dr = 2 * (k1 + 2 * k2 * r2);  // d(radial)/d(x) = x * dr
        (*J)(0, 0) = radial + x * x * dr + 2 * p1 * y + 6 * p2 * x;
        (*J)(0, 1) = x * y * dr + 2 * p1 * x + 2 * p2 * y;
        (*J)(1, 0) = x * y * dr + 2 * p1 * x + 2 * p2 * y;
        (*J)(1, 1) = radial + y * y * dr + 6 * p1 * y + 2 * p2 * x;
    }
    return d;
}

double Camera::theta_d(double th, double* dtheta_d) const {
    const double t2 = th * th;
    const double poly = 1 + t2 * (k_[0] + t2 * (k_[1] + t2 * (k_[2] + t2 * k_[3])));
    if (dtheta_d) *dtheta_d = 1 + t2 * (3 * k_[0] + t2 * (5 * k_[1] + t2 * (7 * k_[2] + t2 * 9 * k_[3])));
    return th * poly;
}

bool Camera::project(const Eigen::Vector3d& p, Eigen::Vector2d& uv,
                     Eigen::Matrix<double, 2, 3>* J) const {
    const double X = p.x(), Y = p.y(), Z = p.z();
    Eigen::Vector2d d;                     // distorted normalized coordinates
    Eigen::Matrix<double, 2, 3> J_d;       // d(d)/d(p)

    if (model_ == Model::RadTan) {
        if (Z < 1e-9) return false;
        Eigen::Matrix2d J_dist;
        d = distort_radtan(Eigen::Vector2d(X / Z, Y / Z), J ? &J_dist : nullptr);
        if (J) {
            Eigen::Matrix<double, 2, 3> J_xy;
            J_xy << 1 / Z, 0, -X / (Z * Z), 0, 1 / Z, -Y / (Z * Z);
            J_d = J_dist * J_xy;
        }
    } else {
        const double rho = std::sqrt(X * X + Y * Y);
        const double n2 = rho * rho + Z * Z;
        if (rho < 1e-9 * std::sqrt(n2)) {
            // On the optical axis: theta_d / rho -> 1 / Z, same as a pinhole.
            if (Z <= 0) return false;
            d = Eigen::Vector2d(X / Z, Y / Z);
            J_d << 1 / Z, 0, -X / (Z * Z), 0, 1 / Z, -Y / (Z * Z);
        } else {
            const double th = std::atan2(rho, Z);  // valid beyond 90 deg, unlike atan(rho / Z)
            double dtd;
            const double td = theta_d(th, &dtd);
            d = Eigen::Vector2d(td * X / rho, td * Y / rho);
            if (J) {
                const Eigen::RowVector3d dth(X * Z / (rho * n2), Y * Z / (rho * n2), -rho / n2);
                const double r3 = rho * rho * rho;
                J_d.row(0) = dtd * (X / rho) * dth + td * Eigen::RowVector3d(Y * Y / r3, -X * Y / r3, 0);
                J_d.row(1) = dtd * (Y / rho) * dth + td * Eigen::RowVector3d(-X * Y / r3, X * X / r3, 0);
            }
        }
    }
    uv = Eigen::Vector2d(fx_ * d.x() + cx_, fy_ * d.y() + cy_);
    if (J) {
        J->row(0) = fx_ * J_d.row(0);
        J->row(1) = fy_ * J_d.row(1);
    }
    return true;
}

bool Camera::unproject(const Eigen::Vector2d& uv, Eigen::Vector3d& bearing) const {
    const Eigen::Vector2d d((uv.x() - cx_) / fx_, (uv.y() - cy_) / fy_);

    if (model_ == Model::RadTan) {
        // Newton on distort(xy) = d, starting from the distorted point.
        Eigen::Vector2d xy = d;
        for (int i = 0; i < 20; ++i) {
            Eigen::Matrix2d J;
            const Eigen::Vector2d r = distort_radtan(xy, &J) - d;
            if (r.norm() < 1e-12) break;
            xy -= J.inverse() * r;
        }
        if ((distort_radtan(xy, nullptr) - d).norm() > 1e-9) return false;
        bearing = Eigen::Vector3d(xy.x(), xy.y(), 1).normalized();
        return true;
    }

    const double td = d.norm();
    if (td < 1e-12) {
        bearing = Eigen::Vector3d(0, 0, 1);
        return true;
    }
    // Newton on theta_d(theta) = td. The model must be increasing on the way, otherwise this pixel
    // is outside the valid part of the lens.
    double th = td;
    for (int i = 0; i < 20; ++i) {
        double dtd;
        const double r = theta_d(th, &dtd) - td;
        if (dtd <= 0) return false;
        th -= r / dtd;
        if (std::abs(r) < 1e-12) break;
    }
    if (th < 0 || th > M_PI || std::abs(theta_d(th, nullptr) - td) > 1e-9) return false;
    const double s = std::sin(th) / td;
    bearing = Eigen::Vector3d(s * d.x(), s * d.y(), std::cos(th));
    return true;
}

}  // namespace vio
