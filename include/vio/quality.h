// Quality monitor: GOOD / DEGRADED / LOST, a score, and what would be sent (DESIGN.md section 7 and
// section 11, "Step 4 plan"). Passive: it reads what the tracker, the MSCKF and the filter already
// produce and never changes the filter.
//
// Per update frame:
//   - frame checks (each a pass or a named fail, limits from quality: in config/vio.yaml):
//     features in the frame, occupied grid cells, the tracker's forward-backward pass rate (only
//     when the tracker runs), share of MSCKF tracks rejected by chi2 (only with enough tracks),
//     the joint gate, IMU rate and largest IMU gap since the last update frame, td change since the
//     last update frame, speed. A frame is good when every check passes.
//   - velocity sigma: the square root of the largest eigenvalue of the velocity covariance (vz
//     included), so the worst direction counts.
//   - state: DEGRADED when the frame is not good or the velocity sigma is above vel_sigma_degraded
//     (where the velocity is sent as NaN); LOST when there was no good frame for lost_after_bad_s,
//     or the velocity sigma is above vel_sigma_lost. Back to GOOD only after good_frames_to_recover
//     good frames in a row, each with the velocity sigma at or below vel_sigma_degraded. LOST is
//     left only to GOOD. The monitor starts in DEGRADED.
//   - the position sigma does not change the state or the score (your decision, 2026-11-26).
//
// Score (DESIGN.md section 11): with m_v = clamp(1 - sigma_v / vel_sigma_degraded) and
// m_l = clamp(1 - sigma_v / vel_sigma_lost), c = share of the applicable checks that pass, f = the
// floor (output.viso_qual_min, the FC's VISO_QUAL_MIN):
//   GOOD 50 + 50 m_v (50-100), DEGRADED f + (49 - f) c m_l (f-49), LOST -1.
// So only LOST stops the FC from fusing (your decision, 2026-11-27).
//
// Low excitation (E1 of the step 4 plan): logged only, changes neither state, score nor output. The
// acceleration across the direction of flight (horizontal, gravity removed) at each update frame;
// the flag is set when it varied by less than min_range over the last window_s of flight above
// min_speed. On synthetic data the speed comes out biased on straight legs and constant circles,
// where this acceleration hardly changes; turns in and out are what makes the speed observable.
#pragma once

#include "vio/types.h"

#include <Eigen/Core>

#include <deque>
#include <string>
#include <vector>

namespace vio {

struct QualityConfig {
    double lost_after_bad_s = 2.0;
    int min_features = 30;           // quality.min_tracks
    int min_grid_cells = 12;
    double min_tracked_ratio = 0.5;  // tracker forward-backward pass rate
    double min_inlier_ratio = 0.6;   // MSCKF: used / (used + chi2 rejected)
    int min_tracks_for_ratio = 10;   // fewer tracks in the update: the ratio is not checked
    double imu_rate_min_hz = 150;
    double imu_gap_max_ms = 20;
    double td_change_max_ms = 10;
    double max_speed = 25;
    double vel_sigma_degraded = 0.9;  // output.velocity_nan_sigma_factor * output.viso_vel_m_nse
    double vel_sigma_lost = 2.0;
    int good_frames_to_recover = 5;
    int score_floor = 10;              // output.viso_qual_min: lowest DEGRADED score
    int grid_cols = 8, grid_rows = 6;  // tracker.grid, for counting occupied cells
    // E1, low excitation (log only)
    double excitation_window_s = 10;
    double excitation_min_range = 1.0;  // m/s^2
    double excitation_min_speed = 3.0;  // m/s
};

// Everything the checks need for one update frame. Negative values mean "not known" for
// tracked_ratio, imu_rate_hz and imu_gap_ms (that check is then not applied).
struct FrameSignals {
    Timestamp t = 0;
    int features = 0;
    int cells = 0;
    double tracked_ratio = -1;
    int msckf_used = 0, msckf_rejected = 0;
    bool joint_rejected = false;
    double imu_rate_hz = -1, imu_gap_ms = -1;
    double td_change_ms = 0;
    double speed = 0;
    Eigen::Matrix3d C_vel = Eigen::Matrix3d::Zero();
};

enum class QualityState { GOOD = 0, DEGRADED = 1, LOST = 2 };

// Bits of QualityResult::fails, one per frame check.
enum QualityFail : unsigned {
    FAIL_FEATURES = 1, FAIL_CELLS = 2, FAIL_TRACKED = 4, FAIL_INLIERS = 8, FAIL_JOINT_GATE = 16,
    FAIL_IMU_RATE = 32, FAIL_IMU_GAP = 64, FAIL_TD_CHANGE = 128, FAIL_SPEED = 256,
};

struct QualityResult {
    QualityState state = QualityState::DEGRADED;
    int score = 1;
    unsigned fails = 0;     // QualityFail bits of this frame
    double sigma_v = 0;     // m/s
};

class QualityMonitor {
public:
    explicit QualityMonitor(const QualityConfig& cfg) : cfg_(cfg) {}
    QualityResult update(const FrameSignals& s);
    // The output found the covariance unusable: LOST from here on, left only to GOOD as usual.
    void force_lost() { state_ = QualityState::LOST; good_in_row_ = 0; }

private:
    QualityConfig cfg_;
    QualityState state_ = QualityState::DEGRADED;
    bool started_ = false;
    Timestamp last_good_ = 0;
    int good_in_row_ = 0;
};

// What would be sent for this frame (no MAVLink here). The covariance is the filter's, unchanged.
struct OutputDecision {
    bool position_valid = false;  // false: position NaN (LOST)
    bool velocity_valid = false;  // false: velocity NaN
    int quality = -1;
};

// Position: valid unless LOST, and only with a finite, positive definite position covariance
// (never a NaN covariance with a valid position). Velocity: valid with the position and the
// velocity sigma at or below vel_sigma_degraded. If the covariance is unusable the monitor is
// forced to LOST and the result says so.
OutputDecision decide_output(QualityResult& q, QualityMonitor& monitor, const Eigen::Matrix3d& C_pos,
                             const QualityConfig& cfg);

// posErr as ArduPilot 4.6.3 computes it from our position variances, before its clamps (floor
// VISO_POS_M_NSE, EKF cap 10 m): cbrt(c0^2 + c6^2 + c11^2). For the log only.
double poserr_463(const Eigen::Matrix3d& C_pos);

// Square root of the largest eigenvalue of a 3x3 covariance.
double largest_sigma(const Eigen::Matrix3d& C);

// Grid cells (cols x rows over the image) with at least one feature.
int occupied_cells(const std::vector<FeatureObs>& features, int width, int height, int cols, int rows);

// IMU samples in (t0, t1]: count, rate, the largest gap (including the one from the last sample at
// or before t0), and the mean specific force.
struct ImuWindow {
    int n = 0;
    double rate_hz = -1, max_gap_ms = -1;
    Eigen::Vector3d mean_accel = Eigen::Vector3d::Zero();
};
ImuWindow summarize_imu(const std::vector<ImuSample>& imu, Timestamp t0, Timestamp t1);

// E1, low excitation. add(): t, the acceleration in the world frame (gravity removed), the velocity,
// and the unit vector pointing down. Log only.
class ExcitationMonitor {
public:
    explicit ExcitationMonitor(const QualityConfig& cfg) : cfg_(cfg) {}
    struct Result {
        bool flag = false;
        double a_lat = 0;   // m/s^2, NaN below min speed
        double range = 0;   // max - min of a_lat over the window, NaN until the window is full
    };
    Result add(Timestamp t, const Eigen::Vector3d& a_world, const Eigen::Vector3d& v_world,
               const Eigen::Vector3d& down);

private:
    QualityConfig cfg_;
    std::deque<std::pair<Timestamp, double>> buf_;
    Timestamp since_ = -1;  // start of the current stretch above min speed
};

}  // namespace vio
