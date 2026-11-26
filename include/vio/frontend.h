// Front end: feature tracking on the raw camera image (DESIGN.md sections 5 and 6).
//
// Per frame:
//   1. sky and horizon mask from the camera attitude (SkyMask), optional;
//   2. image metrics: brightness, contrast, blur, saturation, inside the mask;
//   3. CLAHE (optional) and an image pyramid;
//   4. each feature's start position predicted with the gyro rotation since the last frame;
//   5. pyramidal KLT forward, then backward as a check;
//   6. refill: new Shi-Tomasi corners in the grid cells that have too few features.
// The output is feature id + raw pixel, the same as the synthetic sessions write. Undistortion
// (Camera::unproject) and the rolling-shutter correction (rs_correct) are applied to the raw pixels
// by whoever uses them, so replays from a file and the live system give the same numbers.
#pragma once

#include "vio/camera_model.h"
#include "vio/types.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <set>
#include <vector>

namespace vio {

// ---------------------------------------------------------------- geometry helpers

// Body rotation from t0 to t1 integrated from the gyro minus bias: R_b0_b1 maps vectors in the
// body frame at t1 into the body frame at t0. The gyro is interpolated linearly between samples.
Eigen::Matrix3d integrate_gyro(const std::vector<ImuSample>& imu, Timestamp t0, Timestamp t1,
                               const Eigen::Vector3d& gyro_bias);

// Mean body rotation rate over [t0, t1] (raw gyro, bias not removed), from integrate_gyro.
Eigen::Vector3d average_gyro(const std::vector<ImuSample>& imu, Timestamp t0, Timestamp t1);

// Where a feature seen at uv0 is expected after the camera rotated by R_c0_c1 (camera at t1 in the
// camera frame at t0). Pure rotation: the translation part of the motion is left to KLT.
bool predict_pixel(const Camera& cam, const Eigen::Matrix3d& R_c0_c1, const Eigen::Vector2d& uv0,
                   Eigen::Vector2d& uv1);

// Rolling shutter: a bearing measured on image row `row` (exposed at (row - ref_row) * line_time
// after the reference row) turned into the bearing the camera would have seen at the reference
// time, for a camera rotating at a constant rate omega_cam (camera frame, rad/s) during readout.
// Translation during the readout is ignored (12 cm at 10 m/s, small against 20-50 m).
Eigen::Vector3d rs_correct(const Eigen::Vector3d& bearing, double row, double ref_row,
                           double line_time_s, const Eigen::Vector3d& omega_cam);

// ---------------------------------------------------------------- sky mask

// Masks every pixel whose ray points more than max_off_nadir from straight down: sky, horizon and
// very distant ground. The rays come from a table computed once on a coarse grid (cell_px), so a
// frame only rotates a few thousand vectors. A border of border_px is always masked (KLT window).
class SkyMask {
public:
    SkyMask(const Camera& cam, double max_off_nadir_deg, int cell_px, int border_px);

    // R_world_cam: camera attitude at the frame mid time. down_world: unit vector pointing down in
    // the world frame (gravity direction).
    void update(const Eigen::Matrix3d& R_world_cam, const Eigen::Vector3d& down_world);

    bool usable(double u, double v) const;
    const cv::Mat& image_mask() const { return full_; }  // CV_8U, 255 = usable
    double usable_fraction() const { return usable_fraction_; }

private:
    void build_full_mask();

    int width_, height_, cell_, border_;
    double cos_max_;
    std::vector<Eigen::Vector3f> rays_;  // bearing per cell, camera frame; zero if invalid
    cv::Mat cells_;                       // CV_8U, one value per cell
    cv::Mat full_;
    double usable_fraction_ = 1;
};

// ---------------------------------------------------------------- image metrics

struct ImageQuality {
    double brightness = 0;  // mean grey level 0-255
    double contrast = 0;    // standard deviation of the grey level
    double blur = 0;        // variance of the Laplacian on the half-size image (low = blurred)
    double saturated = 0;   // fraction of pixels at 250 or above
};

// Over the pixels where mask is non-zero.
ImageQuality image_quality(const cv::Mat& gray, const cv::Mat& mask);

// ---------------------------------------------------------------- tracker

struct TrackerConfig {
    int max_features = 150;
    int grid_cols = 8, grid_rows = 6;
    double min_distance_px = 15;
    int klt_window_px = 21;
    int klt_pyramid_levels = 3;     // levels including the full image
    double forward_backward_max_px = 1.0;
    bool gyro_prediction = true;
    bool clahe = false;             // no gain on EuRoC or rendered ground, costs 2 ms (DATASETS.md)
    bool sky_mask = true;           // needs the attitude in FrameInput
    double max_off_nadir_deg = 75;
    int max_track_age = 0;          // frames; older features are dropped and re-detected (0 = off)
};

struct FrameInput {
    Timestamp t = 0;
    cv::Mat image;  // raw 8-bit grey image
    Eigen::Matrix3d R_cprev_ccur = Eigen::Matrix3d::Identity();  // gyro rotation since last frame
    bool have_attitude = false;
    Eigen::Matrix3d R_world_cam = Eigen::Matrix3d::Identity();   // for the sky mask
    Eigen::Vector3d down_world = Eigen::Vector3d(0, 0, 1);
};

// Quality numbers of one frame (written to stats.csv by vio_track).
struct FrameStats {
    Timestamp t = 0;
    int attempted = 0;      // features from the previous frame
    int tracked = 0;        // passed KLT, forward-backward check and mask
    int lost_klt = 0, lost_fb = 0, lost_mask = 0;
    int retired = 0;        // dropped for age (max_track_age)
    int added = 0, total = 0;
    double fb_median = 0, fb_p90 = 0;   // forward-backward error of the KLT successes, px
    double pred_median = 0;  // tracked position minus predicted start position, px. With gyro
                             // prediction on, what the gyro did not explain (translation, timing
                             // and extrinsic errors); with it off, the whole image motion
    int cells_usable = 0, cells_occupied = 0;
    double max_cell_share = 0;          // largest share of all features in one cell
    double usable_fraction = 1;
    ImageQuality image;
    double ms_prepare = 0, ms_track = 0, ms_detect = 0, ms_total = 0;
};

class Tracker {
public:
    Tracker(const Camera& cam, const TrackerConfig& cfg);

    FrameStats process(const FrameInput& in);
    const std::vector<FeatureObs>& features() const { return features_; }

    // Features the filter keeps as landmarks: not retired for age.
    void set_keep(const std::set<int64_t>& ids) { keep_ = ids; }

private:
    void refill(const cv::Mat& img, const cv::Mat& mask, FrameStats& st);
    int cell_of(const cv::Point2f& p) const;

    const Camera& cam_;
    TrackerConfig cfg_;
    SkyMask mask_;
    cv::Ptr<cv::CLAHE> clahe_;
    std::vector<cv::Mat> prev_pyr_;
    std::vector<cv::Point2f> pts_;
    std::vector<FeatureObs> features_;
    std::vector<int> ages_;  // frames since detection, parallel to features_
    std::set<int64_t> keep_;
    int64_t next_id_ = 1;
    int frames_since_detect_ = 1000;
};

}  // namespace vio
