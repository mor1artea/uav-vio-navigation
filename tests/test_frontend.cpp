// Front end parts that do not need real images, checked with synthetic geometry:
//   - sky mask: usable fraction against a direct per-pixel computation, horizon on the right side;
//   - gyro prediction: exact under pure rotation, including the camera mounting rotation;
//   - rolling-shutter correction: points seen through a rolling shutter during a fast rotation;
//   - KLT + refill on a random texture warped by a fast rotation, with and without gyro prediction.

#include "vio/frontend.h"
#include "vio/imu_propagation.h"

#include <opencv2/imgproc.hpp>

#include <cmath>
#include <cstdio>
#include <random>

namespace {

int failures = 0;

void check(bool ok, const char* what, double value) {
    std::printf("%s %-62s %.3g\n", ok ? "ok  " : "FAIL", what, value);
    if (!ok) ++failures;
}

const double deg = M_PI / 180;

vio::CameraInfo fisheye(bool ideal) {
    vio::CameraInfo c;
    c.model = "equidistant";
    c.fx = 340, c.fy = 340, c.cx = 320, c.cy = 240;
    c.distortion = ideal ? std::vector<double>{0, 0, 0, 0}
                         : std::vector<double>{0.02, -0.01, 0.003, -0.0005};
    c.width = 640, c.height = 480;
    return c;
}

// Nominal down-looking camera, image top towards the nose (config/vio.yaml), and a body pitched
// by `pitch` (positive = nose up) in an NED world.
Eigen::Matrix3d R_body_cam() {
    Eigen::Matrix3d R;
    R << 0, -1, 0, 1, 0, 0, 0, 0, 1;
    return R;
}

Eigen::Matrix3d R_world_cam_pitched(double pitch) {
    return Eigen::AngleAxisd(pitch, Eigen::Vector3d::UnitY()).toRotationMatrix() * R_body_cam();
}

// ---------------------------------------------------------------- sky mask

void test_sky_mask() {
    const vio::Camera cam(fisheye(true));
    const Eigen::Vector3d down(0, 0, 1);  // NED
    const double max_off_nadir = 75;
    vio::SkyMask mask(cam, max_off_nadir, 16, 0);

    double worst = 0;
    for (double tilt : {0.0, 30.0, 45.0, 60.0}) {
        const Eigen::Matrix3d R = R_world_cam_pitched(-tilt * deg);  // nose down
        mask.update(R, down);
        // Direct computation for the ideal equidistant lens: theta = r / f, no camera class.
        int usable = 0;
        for (int v = 0; v < 480; ++v) {
            for (int u = 0; u < 640; ++u) {
                const double x = (u - 320) / 340.0, y = (v - 240) / 340.0, th = std::hypot(x, y);
                const double s = th > 0 ? std::sin(th) / th : 1;
                const Eigen::Vector3d ray(s * x, s * y, std::cos(th));
                usable += (R * ray).dot(down) > std::cos(max_off_nadir * deg);
            }
        }
        const double exact = usable / (640.0 * 480.0);
        std::printf("     tilt %2.0f deg: usable %.3f, direct %.3f\n", tilt, mask.usable_fraction(), exact);
        worst = std::max(worst, std::abs(mask.usable_fraction() - exact));
    }
    check(worst < 0.03, "sky mask: usable fraction vs direct per-pixel computation", worst);

    // Nose down 45 deg: the tail side looks at the horizon, that is the image bottom. On the centre
    // column the boundary is where 45 deg + angle below the centre = 75 deg, row 240 + 340 * 30 deg.
    mask.update(R_world_cam_pitched(-45 * deg), down);
    const double row = 240 + 340 * 30 * deg;
    const bool ok = mask.usable(320, 10) && mask.usable(320, row - 20) && !mask.usable(320, row + 20) &&
                    !mask.usable(320, 470);
    check(ok, "sky mask: nose down 45 deg masks the image bottom from row", row);
}

// ---------------------------------------------------------------- gyro prediction

void test_gyro_prediction() {
    const vio::Camera cam(fisheye(false));
    const Eigen::Matrix3d R_bc = R_body_cam() *
                                 Eigen::AngleAxisd(1.0 * deg, Eigen::Vector3d::UnitX()).toRotationMatrix();
    const Eigen::Vector3d w_body(1.5, -2.0, 3.0);  // rad/s, about 220 deg/s
    const Eigen::Vector3d bias(0.01, -0.02, 0.005);

    // IMU at 200 Hz, frames at times that are not on the IMU grid.
    std::vector<vio::ImuSample> imu;
    for (int i = 0; i < 100; ++i) imu.push_back({i * 5'000'000LL, w_body + bias, Eigen::Vector3d::Zero()});
    const vio::Timestamp t0 = 101'300'000, t1 = t0 + 33'333'333;
    const Eigen::Matrix3d R_b0_b1 = vio::integrate_gyro(imu, t0, t1, bias);
    const Eigen::Matrix3d R_c0_c1 = R_bc.transpose() * R_b0_b1 * R_bc;

    // Truth: a far point seen from the camera before and after the rotation.
    const Eigen::Matrix3d R_true_b =
        Eigen::AngleAxisd(w_body.norm() * vio::to_sec(t1 - t0), w_body.normalized()).toRotationMatrix();
    std::mt19937 rng(5);
    std::uniform_real_distribution<double> uu(40, 600), vv(40, 440);
    double worst = 0, motion = 0;
    for (int i = 0; i < 500; ++i) {
        const Eigen::Vector2d uv0(uu(rng), vv(rng));
        Eigen::Vector3d f0;
        cam.unproject(uv0, f0);
        const Eigen::Vector3d p_body0 = R_bc * f0;                         // direction in body at t0
        const Eigen::Vector3d f1 = R_bc.transpose() * (R_true_b.transpose() * p_body0);
        Eigen::Vector2d uv1_true, uv1;
        if (!cam.project(f1, uv1_true) || !vio::predict_pixel(cam, R_c0_c1, uv0, uv1)) continue;
        worst = std::max(worst, (uv1 - uv1_true).norm());
        motion = std::max(motion, (uv1_true - uv0).norm());
    }
    std::printf("     image motion up to %.0f px per frame\n", motion);
    check(worst < 1e-6, "gyro prediction under pure rotation [px]", worst);
}

// ---------------------------------------------------------------- rolling shutter

void test_rolling_shutter() {
    const vio::Camera cam(fisheye(false));
    const Eigen::Vector3d omega(1.0, 3.5, 0.5);  // camera frame, rad/s
    const double line_time = 24.3e-6, ref_row = 239.5;
    std::mt19937 rng(9);
    std::uniform_real_distribution<double> uu(20, 620), vv(20, 460);
    double before = 0, after = 0;
    for (int i = 0; i < 500; ++i) {
        // A direction d in the reference camera frame. The camera at row time is rotated by
        // exp(omega dt), so the shutter sees exp(omega dt)^T d on the row it lands on: iterate.
        Eigen::Vector3d d;
        cam.unproject(Eigen::Vector2d(uu(rng), vv(rng)), d);
        Eigen::Vector2d uv_ref, uv;
        cam.project(d, uv_ref);
        uv = uv_ref;
        for (int k = 0; k < 10; ++k) {
            const double dt = (uv.y() - ref_row) * line_time;
            cam.project(vio::quat_exp(omega * dt).conjugate() * d, uv);
        }
        Eigen::Vector3d b;
        cam.unproject(uv, b);
        Eigen::Vector2d uv_corr;
        cam.project(vio::rs_correct(b, uv.y(), ref_row, line_time, omega), uv_corr);
        before = std::max(before, (uv - uv_ref).norm());
        after = std::max(after, (uv_corr - uv_ref).norm());
    }
    std::printf("     rolling-shutter error without correction up to %.1f px\n", before);
    check(after < 1e-3, "rolling-shutter correction, 3.7 rad/s, 11.7 ms readout [px]", after);
}

// ---------------------------------------------------------------- KLT and refill

struct KltResult {
    double success = 0, err_median = 0, err_max = 0;
    vio::FrameStats first;
};

// Image of a large flat texture (a pinhole image with f = 340 around the camera, so it covers
// rays up to about 77 deg) seen by the fisheye camera with attitude R_world_cam.
cv::Mat render(const vio::Camera& cam, const cv::Mat& texture, const Eigen::Matrix3d& R_world_cam) {
    const double c = texture.cols / 2.0;
    cv::Mat mx(cam.height(), cam.width(), CV_32F), my(cam.height(), cam.width(), CV_32F);
    for (int v = 0; v < cam.height(); ++v) {
        for (int u = 0; u < cam.width(); ++u) {
            Eigen::Vector3d f;
            cam.unproject(Eigen::Vector2d(u, v), f);
            const Eigen::Vector3d w = R_world_cam * f;
            mx.at<float>(v, u) = static_cast<float>(c + 340 * w.x() / w.z());
            my.at<float>(v, u) = static_cast<float>(c + 340 * w.y() / w.z());
        }
    }
    cv::Mat img;
    cv::remap(texture, img, mx, my, cv::INTER_LINEAR, cv::BORDER_CONSTANT, cv::Scalar(0));
    return img;
}

// Frame 0 with the camera along the world axes, frame 1 after a rotation of about 40 px.
KltResult run_klt(bool gyro) {
    const vio::Camera cam(fisheye(false));
    cv::Mat noise(3000, 3000, CV_8U), texture;
    cv::theRNG().state = 12345;
    cv::randu(noise, 0, 255);
    cv::GaussianBlur(noise, texture, cv::Size(0, 0), 2.5);
    cv::normalize(texture, texture, 0, 255, cv::NORM_MINMAX);

    const Eigen::Matrix3d R_c0_c1 =
        Eigen::AngleAxisd(3.5 * 0.0333, Eigen::Vector3d(0.3, 1, 0.2).normalized()).toRotationMatrix();
    const cv::Mat img0 = render(cam, texture, Eigen::Matrix3d::Identity());
    const cv::Mat img1 = render(cam, texture, R_c0_c1);

    vio::TrackerConfig cfg;
    cfg.gyro_prediction = gyro;
    cfg.sky_mask = false;
    vio::Tracker tracker(cam, cfg);
    vio::FrameInput in;
    in.image = img0;
    KltResult r;
    r.first = tracker.process(in);
    std::vector<vio::FeatureObs> before = tracker.features();

    in.t = 33'333'333;
    in.image = img1;
    in.R_cprev_ccur = R_c0_c1;
    tracker.process(in);

    // Truth for every feature that stays well inside the image.
    std::vector<double> errors;
    int expected = 0;
    for (const auto& f : before) {
        Eigen::Vector2d uv1;
        if (!vio::predict_pixel(cam, R_c0_c1, f.uv, uv1) || !cam.in_image(uv1, 20)) continue;
        ++expected;
        for (const auto& g : tracker.features())
            if (g.id == f.id) errors.push_back((g.uv - uv1).norm());
    }
    std::sort(errors.begin(), errors.end());
    r.success = expected ? errors.size() / static_cast<double>(expected) : 0;
    if (!errors.empty()) {
        r.err_median = errors[errors.size() / 2];
        r.err_max = errors.back();
    }
    return r;
}

void test_klt() {
    const KltResult on = run_klt(true), off = run_klt(false);
    std::printf("     first frame: %d features, %d of %d grid cells occupied\n", on.first.total,
                on.first.cells_occupied, on.first.cells_usable);
    std::printf("     40 px rotation: gyro on %.0f %% tracked (median error %.3f px), "
                "gyro off %.0f %% tracked\n", 100 * on.success, on.err_median, 100 * off.success);
    check(on.first.total == 150 && on.first.cells_occupied == on.first.cells_usable,
          "refill: 150 features, every grid cell occupied", on.first.cells_occupied);
    check(on.success > 0.95, "KLT with gyro prediction: share of features tracked", on.success);
    check(on.err_median < 0.1 && on.err_max < 1.0, "KLT with gyro prediction: max error [px]", on.err_max);
}

}  // namespace

int main() {
    test_sky_mask();
    test_gyro_prediction();
    test_rolling_shutter();
    test_klt();
    return failures == 0 ? 0 : 1;
}
