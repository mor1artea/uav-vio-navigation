// Camera models against OpenCV, over the whole field of view:
//   - projection against cv::projectPoints / cv::fisheye::projectPoints;
//   - unprojection against cv::undistortPoints / cv::fisheye::undistortPoints;
//   - Jacobian against a numeric one and against OpenCV's derivative with respect to the
//     translation (with zero rotation and translation it equals the derivative with respect to the
//     point);
//   - round trips, for the fisheye also beyond 90 deg, where OpenCV cannot go.

#include "vio/camera_model.h"

#include <opencv2/calib3d.hpp>
#include <opencv2/core.hpp>

#include <cmath>
#include <cstdio>
#include <random>

namespace {

int failures = 0;

void check(bool ok, const char* what, double value) {
    std::printf("%s %-62s %.3g\n", ok ? "ok  " : "FAIL", what, value);
    if (!ok) ++failures;
}

vio::CameraInfo euroc_cam0() {  // MH_01_easy cam0
    vio::CameraInfo c;
    c.model = "radial-tangential";
    c.fx = 458.654, c.fy = 457.296, c.cx = 367.215, c.cy = 248.375;
    c.distortion = {-0.28340811, 0.07395907, 0.00019359, 1.76187114e-05};
    c.width = 752, c.height = 480;
    return c;
}

vio::CameraInfo fisheye_cam() {  // like the synthetic IMX219 lens, about 120-130 deg
    vio::CameraInfo c;
    c.model = "equidistant";
    c.fx = 340, c.fy = 338, c.cx = 321.5, c.cy = 239.0;
    c.distortion = {0.02, -0.01, 0.003, -0.0005};
    c.width = 640, c.height = 480;
    return c;
}

// Random 3D point in the camera frame at an angle up to max_theta from the optical axis.
Eigen::Vector3d random_point(std::mt19937& rng, double max_theta) {
    std::uniform_real_distribution<double> u(0, 1);
    const double th = max_theta * std::sqrt(u(rng)), phi = 2 * M_PI * u(rng), r = 0.5 + 40 * u(rng);
    return r * Eigen::Vector3d(std::sin(th) * std::cos(phi), std::sin(th) * std::sin(phi), std::cos(th));
}

Eigen::Matrix<double, 2, 3> numeric_jacobian(const vio::Camera& cam, const Eigen::Vector3d& p) {
    Eigen::Matrix<double, 2, 3> J;
    for (int i = 0; i < 3; ++i) {
        const double h = 1e-6 * std::max(1.0, p.norm());
        Eigen::Vector3d a = p, b = p;
        a[i] += h;
        b[i] -= h;
        Eigen::Vector2d ua, ub;
        cam.project(a, ua);
        cam.project(b, ub);
        J.col(i) = (ua - ub) / (2 * h);
    }
    return J;
}

struct Errors {
    double proj = 0, unproj = 0, jac_cv = 0, jac_num = 0, round_trip = 0;
};

// Compares our model with OpenCV on n points up to max_theta from the axis.
Errors compare_with_opencv(const vio::CameraInfo& info, double max_theta, int n) {
    const vio::Camera cam(info);
    const bool fisheye = cam.model() == vio::Camera::Model::KannalaBrandt;
    const cv::Mat K = (cv::Mat_<double>(3, 3) << info.fx, 0, info.cx, 0, info.fy, info.cy, 0, 0, 1);
    const cv::Mat D(info.distortion, true);
    const cv::Mat zero = cv::Mat::zeros(3, 1, CV_64F);
    std::mt19937 rng(7);
    Errors e;
    for (int i = 0; i < n; ++i) {
        const Eigen::Vector3d p = random_point(rng, max_theta);
        Eigen::Vector2d uv;
        Eigen::Matrix<double, 2, 3> J;
        if (!cam.project(p, uv, &J)) {
            e.proj = 1e9;
            continue;
        }
        if (!fisheye && !cam.in_image(uv)) continue;  // radtan is only defined over the image

        // Projection and Jacobian.
        std::vector<cv::Point3d> obj{{p.x(), p.y(), p.z()}};
        std::vector<cv::Point2d> img;
        cv::Mat jac;
        Eigen::Matrix<double, 2, 3> J_cv;
        if (fisheye) {
            cv::fisheye::projectPoints(obj, img, zero, zero, K, D, 0, jac);
            for (int r = 0; r < 2; ++r)
                for (int c = 0; c < 3; ++c) J_cv(r, c) = jac.at<double>(r, 11 + c);  // f c k om T alpha
        } else {
            cv::projectPoints(obj, zero, zero, K, D, img, jac);
            for (int r = 0; r < 2; ++r)
                for (int c = 0; c < 3; ++c) J_cv(r, c) = jac.at<double>(r, 3 + c);  // r t f c dist
        }
        e.proj = std::max(e.proj, (uv - Eigen::Vector2d(img[0].x, img[0].y)).norm());
        e.jac_cv = std::max(e.jac_cv, (J - J_cv).norm() / J.norm());
        e.jac_num = std::max(e.jac_num, (J - numeric_jacobian(cam, p)).norm() / J.norm());

        // Unprojection: OpenCV gives normalized coordinates (x/z, y/z).
        std::vector<cv::Point2d> dist{{uv.x(), uv.y()}}, und;
        const cv::TermCriteria crit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 200, 1e-15);
        if (fisheye)
            cv::fisheye::undistortPoints(dist, und, K, D, cv::noArray(), cv::noArray(), crit);
        else
            cv::undistortPoints(dist, und, K, D, cv::noArray(), cv::noArray(), crit);
        const Eigen::Vector3d b_cv = Eigen::Vector3d(und[0].x, und[0].y, 1).normalized();
        Eigen::Vector3d b;
        if (!cam.unproject(uv, b)) {
            e.unproj = 1e9;
            continue;
        }
        e.unproj = std::max(e.unproj, (b - b_cv).norm());  // = angle for small angles
        e.round_trip = std::max(e.round_trip, (b - p.normalized()).norm());
    }
    return e;
}

// Round trip only (no OpenCV), for rays beyond 90 deg that OpenCV cannot represent.
double fisheye_round_trip(double max_theta) {
    const vio::Camera cam(fisheye_cam());
    std::mt19937 rng(3);
    double worst = 0;
    for (int i = 0; i < 2000; ++i) {
        const Eigen::Vector3d p = random_point(rng, max_theta);
        Eigen::Vector2d uv;
        Eigen::Vector3d b;
        Eigen::Matrix<double, 2, 3> J;
        if (!cam.project(p, uv, &J) || !cam.unproject(uv, b)) return 1e9;
        worst = std::max(worst, (b - p.normalized()).norm());
        worst = std::max(worst, (J - numeric_jacobian(cam, p)).norm() / J.norm());
    }
    return worst;
}

}  // namespace

int main() {
    const double deg = M_PI / 180;

    const Errors rt = compare_with_opencv(euroc_cam0(), 75 * deg, 3000);
    check(rt.proj < 1e-6, "radtan: projection vs cv::projectPoints [px]", rt.proj);
    check(rt.unproj < 1e-8, "radtan: unprojection vs cv::undistortPoints [rad]", rt.unproj);
    check(rt.jac_cv < 1e-8, "radtan: Jacobian vs OpenCV (relative)", rt.jac_cv);
    check(rt.jac_num < 1e-6, "radtan: Jacobian vs numeric (relative)", rt.jac_num);
    check(rt.round_trip < 1e-9, "radtan: round trip over the whole image", rt.round_trip);

    const Errors kb = compare_with_opencv(fisheye_cam(), 85 * deg, 3000);
    check(kb.proj < 1e-6, "Kannala-Brandt: projection vs cv::fisheye, up to 85 deg [px]", kb.proj);
    check(kb.unproj < 1e-8, "Kannala-Brandt: unprojection vs cv::fisheye [rad]", kb.unproj);
    check(kb.jac_cv < 1e-8, "Kannala-Brandt: Jacobian vs OpenCV (relative)", kb.jac_cv);
    check(kb.jac_num < 1e-6, "Kannala-Brandt: Jacobian vs numeric (relative)", kb.jac_num);
    check(kb.round_trip < 1e-9, "Kannala-Brandt: round trip up to 85 deg", kb.round_trip);

    const double beyond = fisheye_round_trip(100 * deg);
    check(beyond < 1e-6, "Kannala-Brandt: round trip and Jacobian up to 100 deg", beyond);

    return failures == 0 ? 0 : 1;
}
