#include "vio/frontend.h"

#include "vio/imu_propagation.h"

#include <opencv2/video/tracking.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>

namespace vio {
namespace {

Eigen::Vector3d gyro_at(const std::vector<ImuSample>& imu, Timestamp t) {
    auto it = std::lower_bound(imu.begin(), imu.end(), t,
                               [](const ImuSample& s, Timestamp tt) { return s.t < tt; });
    if (it == imu.begin()) return imu.front().gyro;
    if (it == imu.end()) return imu.back().gyro;
    const ImuSample& b = *it;
    const ImuSample& a = *(it - 1);
    const double w = static_cast<double>(t - a.t) / static_cast<double>(b.t - a.t);
    return (1 - w) * a.gyro + w * b.gyro;
}

double ms_since(std::chrono::steady_clock::time_point t0) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count();
}

double percentile(std::vector<double> v, double p) {
    if (v.empty()) return 0;
    const size_t k = std::min(v.size() - 1, static_cast<size_t>(p * v.size()));
    std::nth_element(v.begin(), v.begin() + k, v.end());
    return v[k];
}

}  // namespace

// ---------------------------------------------------------------- geometry helpers

Eigen::Matrix3d integrate_gyro(const std::vector<ImuSample>& imu, Timestamp t0, Timestamp t1,
                               const Eigen::Vector3d& gyro_bias) {
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();
    if (imu.empty() || t1 <= t0) return q.toRotationMatrix();
    // Midpoint rule over the pieces between t0, the samples inside (t0, t1), and t1.
    auto it = std::upper_bound(imu.begin(), imu.end(), t0,
                               [](Timestamp tt, const ImuSample& s) { return tt < s.t; });
    Timestamp a = t0;
    Eigen::Vector3d wa = gyro_at(imu, t0);
    while (a < t1) {
        const Timestamp b = (it != imu.end() && it->t < t1) ? it->t : t1;
        const Eigen::Vector3d wb = (b == t1) ? gyro_at(imu, t1) : it->gyro;
        q = q * quat_exp((0.5 * (wa + wb) - gyro_bias) * to_sec(b - a));
        a = b;
        wa = wb;
        if (it != imu.end() && it->t <= a) ++it;
    }
    return q.normalized().toRotationMatrix();
}

Eigen::Vector3d average_gyro(const std::vector<ImuSample>& imu, Timestamp t0, Timestamp t1) {
    const Eigen::AngleAxisd aa(integrate_gyro(imu, t0, t1, Eigen::Vector3d::Zero()));
    return aa.angle() * aa.axis() / to_sec(t1 - t0);
}

bool predict_pixel(const Camera& cam, const Eigen::Matrix3d& R_c0_c1, const Eigen::Vector2d& uv0,
                   Eigen::Vector2d& uv1) {
    Eigen::Vector3d f0;
    if (!cam.unproject(uv0, f0)) return false;
    return cam.project(R_c0_c1.transpose() * f0, uv1);
}

Eigen::Vector3d rs_correct(const Eigen::Vector3d& bearing, double row, double ref_row,
                           double line_time_s, const Eigen::Vector3d& omega_cam) {
    // Camera at row time = camera at reference time rotated by exp(omega * dt), so a direction seen
    // at row time is exp(omega * dt) * bearing in the reference camera frame.
    const double dt = (row - ref_row) * line_time_s;
    return quat_exp(omega_cam * dt) * bearing;
}

// ---------------------------------------------------------------- sky mask

SkyMask::SkyMask(const Camera& cam, double max_off_nadir_deg, int cell_px, int border_px)
    : width_(cam.width()), height_(cam.height()), cell_(cell_px), border_(border_px),
      cos_max_(std::cos(max_off_nadir_deg * M_PI / 180)) {
    const int nx = (width_ + cell_ - 1) / cell_, ny = (height_ + cell_ - 1) / cell_;
    cells_ = cv::Mat(ny, nx, CV_8U);
    rays_.resize(static_cast<size_t>(nx * ny));
    for (int j = 0; j < ny; ++j) {
        for (int i = 0; i < nx; ++i) {
            const double u = std::min((i + 0.5) * cell_, width_ - 1.0);
            const double v = std::min((j + 0.5) * cell_, height_ - 1.0);
            Eigen::Vector3d b;
            const bool ok = cam.unproject(Eigen::Vector2d(u, v), b);
            rays_[j * nx + i] = ok ? Eigen::Vector3f(b.cast<float>()) : Eigen::Vector3f::Zero();
            cells_.at<uint8_t>(j, i) = ok ? 255 : 0;
        }
    }
    build_full_mask();
}

void SkyMask::update(const Eigen::Matrix3d& R_world_cam, const Eigen::Vector3d& down_world) {
    // Angle from nadir < max  <=>  (R_world_cam * ray) . down > cos(max)  <=>  ray . (R^T down) > ...
    const Eigen::Vector3f down_cam = (R_world_cam.transpose() * down_world).cast<float>();
    for (int k = 0; k < static_cast<int>(rays_.size()); ++k) {
        const bool valid = rays_[k].squaredNorm() > 0;
        cells_.data[k] = (valid && rays_[k].dot(down_cam) > cos_max_) ? 255 : 0;
    }
    build_full_mask();
}

void SkyMask::build_full_mask() {
    cv::Mat big;
    cv::resize(cells_, big, cv::Size(cells_.cols * cell_, cells_.rows * cell_), 0, 0,
               cv::INTER_NEAREST);
    full_ = big(cv::Rect(0, 0, width_, height_)).clone();
    if (border_ > 0) {
        full_.rowRange(0, border_).setTo(0);
        full_.rowRange(height_ - border_, height_).setTo(0);
        full_.colRange(0, border_).setTo(0);
        full_.colRange(width_ - border_, width_).setTo(0);
    }
    usable_fraction_ = cv::countNonZero(full_) / static_cast<double>(width_ * height_);
}

bool SkyMask::usable(double u, double v) const {
    if (u < border_ || v < border_ || u > width_ - 1 - border_ || v > height_ - 1 - border_)
        return false;
    return cells_.at<uint8_t>(static_cast<int>(v) / cell_, static_cast<int>(u) / cell_) != 0;
}

// ---------------------------------------------------------------- image metrics

ImageQuality image_quality(const cv::Mat& gray, const cv::Mat& mask) {
    ImageQuality q;
    cv::Scalar mean, stddev;
    cv::meanStdDev(gray, mean, stddev, mask);
    q.brightness = mean[0];
    q.contrast = stddev[0];
    const int n = cv::countNonZero(mask);
    if (n > 0) {
        cv::Mat sat;
        cv::bitwise_and(gray >= 250, mask, sat);
        q.saturated = cv::countNonZero(sat) / static_cast<double>(n);
    }
    // Blur on the half-size image: cheaper, and sensor noise counts less.
    cv::Mat half, half_mask, lap;
    cv::pyrDown(gray, half);
    cv::resize(mask, half_mask, half.size(), 0, 0, cv::INTER_NEAREST);
    cv::Laplacian(half, lap, CV_16S);
    cv::meanStdDev(lap, mean, stddev, half_mask);
    q.blur = stddev[0] * stddev[0];
    return q;
}

// ---------------------------------------------------------------- tracker

Tracker::Tracker(const Camera& cam, const TrackerConfig& cfg)
    : cam_(cam), cfg_(cfg),
      mask_(cam, cfg.max_off_nadir_deg, 16, cfg.klt_window_px / 2 + 1),
      clahe_(cv::createCLAHE(2.0, cv::Size(8, 8))) {}

int Tracker::cell_of(const cv::Point2f& p) const {
    const int cx = static_cast<int>(p.x * cfg_.grid_cols / cam_.width());
    const int cy = static_cast<int>(p.y * cfg_.grid_rows / cam_.height());
    return std::clamp(cy, 0, cfg_.grid_rows - 1) * cfg_.grid_cols + std::clamp(cx, 0, cfg_.grid_cols - 1);
}

FrameStats Tracker::process(const FrameInput& in) {
    using clock = std::chrono::steady_clock;
    const auto t_start = clock::now();
    FrameStats st;
    st.t = in.t;

    // 1-3. Mask, image metrics, CLAHE, pyramid.
    if (cfg_.sky_mask && in.have_attitude) mask_.update(in.R_world_cam, in.down_world);
    const cv::Mat& mask = mask_.image_mask();
    st.usable_fraction = mask_.usable_fraction();
    st.image = image_quality(in.image, mask);
    cv::Mat img;
    if (cfg_.clahe)
        clahe_->apply(in.image, img);
    else
        img = in.image;
    const cv::Size win(cfg_.klt_window_px, cfg_.klt_window_px);
    const int max_level = cfg_.klt_pyramid_levels - 1;
    std::vector<cv::Mat> pyr;
    cv::buildOpticalFlowPyramid(img, pyr, win, max_level);
    st.ms_prepare = ms_since(t_start);

    // 4-5. Gyro prediction, KLT forward and backward.
    const auto t_track = clock::now();
    st.attempted = static_cast<int>(pts_.size());
    if (!pts_.empty()) {
        std::vector<cv::Point2f> pred(pts_.size());
        for (size_t i = 0; i < pts_.size(); ++i) {
            Eigen::Vector2d uv1;
            const bool ok = cfg_.gyro_prediction &&
                            predict_pixel(cam_, in.R_cprev_ccur,
                                          Eigen::Vector2d(pts_[i].x, pts_[i].y), uv1);
            pred[i] = ok ? cv::Point2f(static_cast<float>(uv1.x()), static_cast<float>(uv1.y()))
                         : pts_[i];
        }
        const cv::TermCriteria crit(cv::TermCriteria::COUNT + cv::TermCriteria::EPS, 30, 0.01);
        std::vector<uint8_t> ok_fwd, ok_bwd;
        std::vector<float> err;
        std::vector<cv::Point2f> cur = pred, back = pts_;
        cv::calcOpticalFlowPyrLK(prev_pyr_, pyr, pts_, cur, ok_fwd, err, win, max_level, crit,
                                 cv::OPTFLOW_USE_INITIAL_FLOW);
        cv::calcOpticalFlowPyrLK(pyr, prev_pyr_, cur, back, ok_bwd, err, win, max_level, crit,
                                 cv::OPTFLOW_USE_INITIAL_FLOW);

        std::vector<cv::Point2f> kept_pts;
        std::vector<FeatureObs> kept;
        std::vector<int> kept_ages;
        std::vector<double> fb_errors, pred_errors;
        for (size_t i = 0; i < pts_.size(); ++i) {
            if (!ok_fwd[i] || !ok_bwd[i]) {
                ++st.lost_klt;
                continue;
            }
            const double fb = std::hypot(back[i].x - pts_[i].x, back[i].y - pts_[i].y);
            fb_errors.push_back(fb);
            if (fb > cfg_.forward_backward_max_px) {
                ++st.lost_fb;
            } else if (!mask_.usable(cur[i].x, cur[i].y)) {
                ++st.lost_mask;
            } else if (cfg_.max_track_age > 0 && ages_[i] + 1 >= cfg_.max_track_age &&
                       !keep_.count(features_[i].id)) {
                ++st.retired;  // the refill may detect the same corner again, with its drift reset
            } else {
                pred_errors.push_back(std::hypot(cur[i].x - pred[i].x, cur[i].y - pred[i].y));
                kept_pts.push_back(cur[i]);
                kept.push_back({features_[i].id, Eigen::Vector2d(cur[i].x, cur[i].y)});
                kept_ages.push_back(ages_[i] + 1);
            }
        }
        pts_ = std::move(kept_pts);
        features_ = std::move(kept);
        ages_ = std::move(kept_ages);
        st.fb_median = percentile(fb_errors, 0.5);
        st.fb_p90 = percentile(fb_errors, 0.9);
        st.pred_median = percentile(pred_errors, 0.5);
    }
    st.tracked = static_cast<int>(pts_.size());
    st.ms_track = ms_since(t_track);

    // 6. Refill.
    const auto t_detect = clock::now();
    refill(img, mask, st);
    st.ms_detect = ms_since(t_detect);

    prev_pyr_ = std::move(pyr);
    st.total = static_cast<int>(pts_.size());
    st.ms_total = ms_since(t_start);
    return st;
}

void Tracker::refill(const cv::Mat& img, const cv::Mat& mask, FrameStats& st) {
    const int n_cells = cfg_.grid_cols * cfg_.grid_rows;
    std::vector<int> count(n_cells, 0);
    for (const auto& p : pts_) ++count[cell_of(p)];

    // A cell is usable if its centre is outside the mask.
    std::vector<bool> usable(n_cells);
    for (int c = 0; c < n_cells; ++c) {
        const double u = ((c % cfg_.grid_cols) + 0.5) * cam_.width() / cfg_.grid_cols;
        const double v = ((c / cfg_.grid_cols) + 0.5) * cam_.height() / cfg_.grid_rows;
        usable[c] = mask_.usable(u, v);
        st.cells_usable += usable[c];
    }
    const int quota =
        st.cells_usable > 0 ? (cfg_.max_features + st.cells_usable - 1) / st.cells_usable : 0;

    // Detection costs a full corner map, so only run it when features are missing: fewer than 90 %
    // of the target, or an empty usable cell. A cell that stays empty usually has no texture
    // (sky, water, a white wall), so that case is retried only every 5th frame.
    bool empty_cell = false;
    for (int c = 0; c < n_cells; ++c) empty_cell |= usable[c] && count[c] == 0;
    const int total = static_cast<int>(pts_.size());
    ++frames_since_detect_;
    if (quota > 0 && (total < cfg_.max_features * 9 / 10 || (empty_cell && frames_since_detect_ >= 5))) {
        frames_since_detect_ = 0;
        cv::Mat det_mask = mask.clone();
        for (const auto& p : pts_)
            cv::circle(det_mask, p, static_cast<int>(cfg_.min_distance_px), cv::Scalar(0), -1);
        // Corners are searched on the half-size image: a quarter of the pixels, sensor noise
        // averaged out, and a much shorter candidate list to sort (on rough ground almost every
        // full-size pixel passes the corner threshold, which made detection cost 10-20 ms).
        cv::Mat half, half_mask;
        cv::pyrDown(img, half);
        cv::resize(det_mask, half_mask, half.size(), 0, 0, cv::INTER_NEAREST);
        std::vector<cv::Point2f> corners;  // sorted by corner strength, best first
        cv::goodFeaturesToTrack(half, corners, 4 * cfg_.max_features, 0.01,
                                cfg_.min_distance_px / 2, half_mask, 3);
        for (auto& p : corners) p *= 2.0f;
        // Fill level by level: first every cell up to 1 feature, then up to 2, ... so the strong
        // corners of one cell cannot use up the budget while another cell stays empty.
        std::vector<bool> taken(corners.size(), false);
        for (int level = 1; level <= quota; ++level) {
            for (size_t i = 0; i < corners.size(); ++i) {
                if (static_cast<int>(pts_.size()) >= cfg_.max_features) break;
                const int c = cell_of(corners[i]);
                if (taken[i] || count[c] >= level) continue;
                taken[i] = true;
                ++count[c];
                pts_.push_back(corners[i]);
                features_.push_back({next_id_++, Eigen::Vector2d(corners[i].x, corners[i].y)});
                ages_.push_back(0);
                ++st.added;
            }
        }
    }

    int max_count = 0;
    for (int c = 0; c < n_cells; ++c) {
        st.cells_occupied += count[c] > 0;
        max_count = std::max(max_count, count[c]);
    }
    st.max_cell_share = pts_.empty() ? 0 : max_count / static_cast<double>(pts_.size());
}

}  // namespace vio
