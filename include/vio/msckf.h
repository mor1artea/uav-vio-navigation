// Visual update of the filter, MSCKF style (Mourikis and Roumeliotis 2007; the math follows the
// OpenVINS papers, no code copied).
//
// Per frame:
//   1. clone the IMU pose at the frame time (Filter::augment_clone), unless the drone is still;
//   2. append each feature's pixel (rolling-shutter corrected) to its track;
//   3. use the tracks that ended (feature lost) and the tracks that reach back to the oldest clone
//      when the window is full: triangulate the point from the clones, build the pixel residuals and
//      Jacobians, remove the point from them (nullspace projection), chi2 test per feature;
//   4. stack all features, compress with QR, one EKF update;
//   5. drop the oldest clone when there are more than the configured number.
// Each feature is used once: a track that is used while its feature is still tracked either
// becomes a landmark (slow flight, for hover) or its feature is ignored from then on. Slow KLT
// drift would otherwise make the successive tracks of one feature share their errors, and the filter
// would count them as independent (overconfident; DATASETS.md section 6). The tracker's age cap
// brings fresh features instead.
// Landmarks: updated every frame against the newest clone; dropped when lost, after two chi2
// failures in a row, or when their viewing direction changed too much (drift guard).
//
// Drift-aware feature noise (filter.track_drift, off by default; DESIGN.md section 13.4 a). KLT
// tracks drift: the pixel error is a random walk in the view change of the feature, not white
// noise. With the switch on, the pixel errors of a track get the covariance of track_noise() and the
// track is whitened with its Cholesky factor before the point is removed, so everything after that
// (nullspace, chi2 test, QR compression) works with unit noise. Observations whose modelled sigma
// would exceed drift_sigma_max are not used. A landmark gets process noise before each update: its
// feature drifted by q * (view change since the last update), spread over the landmark position
// as q * dv * J+ J+^T (J: pixel by landmark position, J+ its pseudo-inverse).
//
// Midpoint linearization (filter.midpoint_linearization, off by default; DESIGN.md section 11,
// "Midpoint linearization"). The stacked MSCKF update is done once to get the updated clone poses,
// then the prior is restored and the update is done again with the Jacobians (Hx, Hf) taken at the
// clone poses halfway between prior and update, the point triangulated at those poses. Residuals,
// covariance and state stay at the prior. The second pass keeps the first pass's per-track chi2
// decisions (tested with the prior Jacobians) and does not test the tracks again; the joint gate
// runs in both passes. Landmark updates are not affected.
// Shown on synthetic data: linearized at the prior the visual update pushes the scale up a little on
// every update, at the updated poses it pushes it down, and the push is close to linear in between;
// where it crosses zero depends on the mix of pixel and IMU noise (0.33-0.57 of the way), so 0.5 is
// near zero for our generator's noise, not in general. Likely explanation (not proven): for a
// measurement with constant curvature the slope taken halfway between two points is exactly the slope
// of the line joining them, so the linear model then predicts the step from prior to update.
#pragma once

#include "vio/camera_model.h"
#include "vio/filter.h"
#include "vio/types.h"

#include <map>
#include <set>
#include <vector>

namespace vio {

// Pixel of a world point seen from the IMU pose (R_wb, p_wb) through the camera mounted at
// (R_bc, t_bc), with the Jacobians with respect to the invariant pose error [dtheta, dp] (see
// filter.h) and to the point. Returns false if the point cannot be projected.
bool project_point(const Camera& cam, const Eigen::Matrix3d& R_bc, const Eigen::Vector3d& t_bc,
                   const Eigen::Matrix3d& R_wb, const Eigen::Vector3d& p_wb,
                   const Eigen::Vector3d& p_f, Eigen::Vector2d& uv,
                   Eigen::Matrix<double, 2, 6>* H_pose, Eigen::Matrix<double, 2, 3>* H_point);

// One observation for triangulation: camera pose in the world and the raw pixel.
struct CameraObs {
    Eigen::Matrix3d R_wc;
    Eigen::Vector3d p_wc;
    Eigen::Vector2d uv;
};

// Point seen in all observations: least squares on the ray directions, then Gauss-Newton on the
// pixel errors. Fails if the rays are nearly parallel (less than min_parallax_rad between the
// most different two) or the point ends up behind a camera.
bool triangulate(const Camera& cam, const std::vector<CameraObs>& obs, double min_parallax_rad,
                 Eigen::Vector3d& p_f);

// Covariance of the pixel errors of one track, rows u1, v1, u2, v2, ...: white noise sigma_w on every
// pixel plus, per image axis, a random walk with variance q per degree of view change accumulated
// along the track (s_deg[i] at observation i, s_deg[0] = 0):
//   Cov(e_i, e_j) = sigma_w^2 [i == j] + q min(s_i, s_j)
Eigen::MatrixXd track_noise(const std::vector<double>& s_deg, double sigma_w, double q);

// View change accumulated along a track, deg: the sum of the angles between the rays from
// consecutive camera positions to the point p_f.
std::vector<double> accumulated_view_change(const std::vector<Eigen::Vector3d>& cameras,
                                            const Eigen::Vector3d& p_f);

// Median pixel motion of the features seen in both frames, -1 if there are none in common.
double median_feature_motion(const CameraFrame& prev, const CameraFrame& cur);

struct MsckfConfig {
    int max_clones = 11;
    double pixel_noise = 1.0;       // px, 1 sigma
    double line_time_s = 0;         // rolling shutter, 0 = global shutter
    int min_observations = 4;
    double min_parallax_deg = 1.0;
    double chi2_probability = 0.95;
    int max_landmarks = 25;
    double landmark_max_age_s = 60.0;           // safety limit
    double landmark_max_view_change_deg = 5.0;  // drift guard
    double landmark_speed_max = 4.0;  // m/s: landmarks are only made when this slow (hover)
    double landmark_min_parallax_deg = 3.0;
    int feature_max_age = 12;         // update frames: tracker.max_track_age / filter.update_stride
    bool track_drift = false;         // drift-aware feature noise (see above), off by default
    bool midpoint_linearization = false;  // see above, off by default
    double drift_q = 0;               // px^2 per deg of view change, per image axis
    double drift_sigma_max = 4.0;     // px
};

struct MsckfStats {
    int used = 0;           // MSCKF features in the update
    int rejected = 0;       // failed the chi2 test (outliers, or the filter disagrees with the image)
    int no_geometry = 0;    // too little parallax or behind a camera: not usable, no judgement
    int new_landmarks = 0;
    int landmarks = 0;      // in the state after this frame
    int landmark_rejected = 0;  // landmark updates that failed the chi2 test
    bool update_rejected = false;  // the stacked MSCKF update failed its chi2 test as a whole
};

// Clone pose to take the Jacobians at, by clone key (midpoint linearization).
struct ClonePose {
    Eigen::Quaterniond q;
    Eigen::Vector3d p;
};
using ClonePoses = std::map<Timestamp, ClonePose>;

class Msckf {
public:
    Msckf(const Camera& cam, const Eigen::Matrix4d& T_body_cam, const MsckfConfig& cfg);

    // A camera frame with feature observations. The filter must already be propagated to the
    // frame time + td. While the drone is still, no clone is added and tracks wait. gyro: the body
    // rate at the frame time, averaged over a short window centred on it (Filter::augment_clone).
    MsckfStats process(Filter& filter, const CameraFrame& frame, bool still,
                       const Eigen::Vector3d& gyro);

    // Feature ids that are landmarks: the tracker must not retire them for age.
    std::set<int64_t> landmark_ids(const Filter& filter) const;

private:
    using Track = std::map<Timestamp, Eigen::Vector2d>;  // clone key -> pixel
    // With track_drift the returned Hx, Hf and r are whitened (unit noise). With lin, Hx and Hf are
    // taken at those clone poses and at the point triangulated there; r stays at the estimate.
    bool linearize(const Filter& filter, int n, const Track& track, double min_parallax_deg,
                   Eigen::MatrixXd& Hx, Eigen::MatrixXd& Hf, Eigen::VectorXd& r,
                   Eigen::Vector3d& p_f, const ClonePoses* lin = nullptr) const;
    // Chi2 test per track, stacking, QR compression and one gated update. Counts into st if given.
    // passed: gets the ids of the tracks that pass their chi2 test. only: use just these tracks and
    // do not test them again (the second pass of the midpoint linearization).
    // Returns false if the joint gate rejected the update.
    bool stacked_update(Filter& filter, const std::vector<std::pair<int64_t, Track>>& tracks,
                        const ClonePoses* lin, MsckfStats* st, std::set<int64_t>* passed = nullptr,
                        const std::set<int64_t>* only = nullptr) const;
    bool is_landmark(const Filter& filter, int64_t id) const;
    void update_landmarks(Filter& filter, Timestamp t, const std::map<int64_t, Eigen::Vector2d>& obs,
                          MsckfStats& st);
    bool init_landmark(Filter& filter, int64_t id, Timestamp t, const Track& track, MsckfStats& st);

    const Camera& cam_;
    Eigen::Matrix3d R_bc_;
    Eigen::Vector3d t_bc_;
    MsckfConfig cfg_;
    std::map<int64_t, Track> tracks_;   // rolling-shutter corrected pixels per feature id
    std::set<int64_t> used_;            // features already used, ignored while still tracked
    std::map<int64_t, int> chi2_fails_;  // landmark id -> failures in a row
    std::map<int64_t, int> age_;         // frames each feature has been seen
    std::map<int64_t, Eigen::Vector3d> landmark_ray_;  // track_drift: viewing direction at the last update
};

}  // namespace vio
