// The VIO filter: an error-state EKF (DESIGN.md sections 2 and 6).
//
// Mean state: attitude q_wb, position p, velocity v, gyro bias bg, accel bias ba (the IMU state),
// the camera-IMU time offset td, the baro bias, and a window of clones: the IMU pose at the times of
// the last camera frames. Landmarks come in a later stage.
//
// Error state, in this order:
//   dtheta (3), dp (3), dv (3), dbg (3), dba (3), dtd (1), dbaro (1)    17 values
//   per landmark: dp_f (3)                                             3 values each
//   per clone: dtheta (3), dp (3)                                     6 values each
// The errors are right-invariant (world frame):
//   R_true = Exp(dtheta) R,   p_true = Exp(dtheta) p + dp,   v_true = Exp(dtheta) v + dv
// the same for the clone poses (each with its own dtheta), and for landmarks with the IMU's
// dtheta: p_f_true = Exp(dtheta) p_f + dp_f. Because of that last one, a landmark error moves
// with the IMU attitude error: dp_f(t) - dp_f(0) = [p_f]x (dtheta(t) - dtheta(0)). In these coordinates the four directions VIO cannot observe
// (position and yaw) are constant, so a plain EKF cannot gain fake information about them, at any
// linearization point. With ordinary errors (p_true = p + dp) the filter became overconfident in
// yaw: on EuRoC one burst of updates at takeoff turned yaw by 25 deg and then held it there.
// First-estimate Jacobians (FEJ), the usual fix for ordinary errors, were tried first and made the
// filter inconsistent on synthetic Monte Carlo runs (DESIGN.md section 11, step 3).
// The covariance written to files is converted back to ordinary errors (ordinary_covariance).
//
// Moving origin. The invariant errors rotate positions about the origin, so far from it a small
// attitude error becomes a large position term, and the part the linear model ignores (about
// 1/2 |dtheta|^2 |p|) grows with the distance: estimated 0.7-2.3 px at 5 km, the likely reason why
// results got worse far from the origin (DESIGN.md section 11). All positions inside
// the filter (IMU, clones, landmarks) are therefore kept relative to origin(), which starts at the
// start position and is moved to the current position (height included) whenever the drone is
// more than origin_shift_distance_m away. world_state() and the output add the origin back.

#pragma once

#include "vio/imu_propagation.h"
#include "vio/types.h"

#include <Eigen/Dense>

#include <deque>
#include <string>
#include <vector>

namespace vio {

// IMU pose at the time of a camera frame, kept in the state while features use it.
struct Clone {
    Timestamp key = 0;  // camera frame time stamp (camera clock)
    Eigen::Quaterniond q = Eigen::Quaterniond::Identity();  // q_wb
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
};

// Point on the ground kept in the state while it is tracked (for hover, DESIGN.md section 5).
struct Landmark {
    int64_t id = 0;     // feature id from the tracker
    Eigen::Vector3d p = Eigen::Vector3d::Zero();
    Timestamp born = 0;
    Eigen::Vector3d camera_at_birth = Eigen::Vector3d::Zero();  // for the view-change guard
};

struct FilterConfig {
    ImuNoise imu;                    // continuous-time noise densities
    double chi2_probability = 0.95;  // gate for the updates
    std::string init_mode = "static";  // "static" (on the ground) or "ground_truth"
    double static_window_s = 1.0;    // still for this long before a static start
    double sigma_yaw_deg = 3.0;      // yaw prior (compass, or ground truth standing in for it)
    double sigma_att_deg = 0.5;      // attitude prior for a ground-truth start
    double sigma_pos = 0.01;         // position prior (the origin is defined by the start)
    double sigma_vel = 0.05;         // velocity prior for a ground-truth start
    double sigma_bg = 0.01;          // gyro bias prior when it is not measured (ground-truth start)
    double sigma_ba = 0.1;           // accel bias prior
    double td_initial_s = 0.0;       // camera-IMU time offset, start value
    double sigma_td_s = 0.03;        // and its prior (0 when td is not estimated online)
    double sigma_baro_bias = 10.0;   // m, before the first baro reading
    // Baro (DESIGN.md section 4): a weak vertical measurement with its bias in the state, never a
    // source of scale by design.
    bool baro = true;
    double baro_noise = 1.0;               // m, 1 sigma
    double baro_bias_random_walk = 0.05;   // m per sqrt(s)
    double baro_inflate_below_agl = 3.0;   // m above the takeoff height: ground effect, prop wash
    double baro_inflate_above_climb = 2.0; // m/s climb or sink rate
    double baro_inflate_factor = 3.0;      // noise times this in those cases (a guess)
    double origin_shift_distance_m = 100.0;  // move the origin to the drone beyond this; <= 0: never
    bool zupt = true;                // zero-velocity updates while still
    double zupt_accel_std = 0.1;     // m/s^2, per axis, over the static window
    double zupt_gyro_std = 0.01;     // rad/s, per axis
    double zupt_velocity_noise = 0.01;  // m/s
    // Zero-velocity updates while the FC says landed, also with the motors running (vibration).
    bool zupt_landed = false;  // off until our own takeoff recordings (DESIGN.md section 11)
    double zupt_landed_accel_max = 0.3;  // m/s^2: | |mean specific force| - g | on the ground
};

// Decides "still" from the IMU alone: small spread of accel and gyro over a sliding window.
class StaticDetector {
public:
    StaticDetector(double window_s, double accel_std_max, double gyro_std_max)
        : window_(static_cast<Timestamp>(window_s * 1e9)), accel_max_(accel_std_max),
          gyro_max_(gyro_std_max) {}
    void add(const ImuSample& s);
    bool is_static() const;  // false until the window is full
    Eigen::Vector3d mean_accel() const;
    Eigen::Vector3d mean_gyro() const;

private:
    Timestamp window_;
    double accel_max_, gyro_max_;
    std::deque<ImuSample> buf_;
};

class Filter {
public:
    // Error-state indices.
    static constexpr int TH = 0, P = 3, V = 6, BG = 9, BA = 12, TD = 15, BB = 16, CORE = 17;

    Filter(const FilterConfig& cfg, const Eigen::Vector3d& gravity_world);

    // Every IMU sample goes here, before and after initialization.
    void add_imu(const ImuSample& s);

    // Static start: if the IMU has been still for the configured window, initializes roll and pitch
    // from the accelerometer, the gyro bias from the mean gyro, and takes yaw from R_ref (the compass
    // on the drone, ground truth on datasets) and the position p0. Returns true once initialized.
    bool try_init_static(const Eigen::Matrix3d& R_ref, const Eigen::Vector3d& p0);

    // Start from a known state (sessions that start in flight), with the configured priors.
    void init_from_state(const State& s);

    bool initialized() const { return initialized_; }

    // Propagates the mean and covariance to time t. With next (the first IMU sample after t) the
    // IMU is interpolated at t; without it the last sample is held (only at the end of the data).
    void propagate_to(Timestamp t, const ImuSample* next);

    // Zero-velocity update if the drone is still: v = 0, and the mean gyro since the last call
    // equals the gyro bias. "Still" needs the IMU (small spread) and the images (image_still: the
    // features did not move). The IMU alone cannot tell rest from flight at constant velocity.
    // Returns true if the update was applied (it is skipped if it fails its chi2 test).
    bool zupt_if_static(bool image_still) { return zero_velocity_update(image_still, false, false) == 1; }

    // Zero-velocity update, one call per update frame, over the IMU samples since the last call.
    // Returns 1 for a "still" update, 2 for a "landed" update, 0 for none.
    //  - still: as zupt_if_static.
    //  - landed (zupt_landed): the FC says landed (landed), the image does not move (image_moving
    //    false; without features the image says nothing), and the mean specific force has the
    //    magnitude of gravity within zupt_landed_accel_max: on the ground the true acceleration is
    //    zero whatever the vibration, at liftoff it is not. Only v = 0; with the motors running the
    //    gyro mean is not a clean bias measurement. Not chi2 gated (filter.cpp says why).
    // The IMU spread is not used for "landed": the motors make the IMU shake on the ground.
    int zero_velocity_update(bool image_still, bool image_moving, bool landed);

    // Generic EKF update: residual r = z - h(x), Jacobian H (rows x covariance size), noise R.
    // With gate = true it is skipped when it fails the chi2 test. Returns true if applied.
    bool update(const Eigen::MatrixXd& H, const Eigen::VectorXd& r, const Eigen::MatrixXd& R,
                bool gate);

    // Adds a clone of the current IMU pose (call after propagate_to(frame time + td)). Its
    // covariance includes the td term: a clone at the wrong time is off by velocity and rotation
    // rate. gyro: the measured body rate at the clone time, averaged over a short window centred
    // on it (a single raw sample is too noisy: its noise looks like rotation, and the filter then
    // believes it learns td; a past average lags and biases td).
    void augment_clone(Timestamp key, const Eigen::Vector3d& gyro);
    void remove_clone(size_t k);
    const std::vector<Clone>& clones() const { return clones_; }
    int clone_index(size_t k) const {  // dtheta, then dp
        return CORE + 3 * static_cast<int>(landmarks_.size()) + 6 * static_cast<int>(k);
    }

    // Adds a landmark whose error is dp_f = A * (current error) + w, w ~ N(0, noise): the
    // delayed initialization from a feature track (Msckf). A has as many columns as the state.
    void add_landmark(const Landmark& l, const Eigen::MatrixXd& A, const Eigen::Matrix3d& noise);
    void remove_landmark(size_t j);
    // Adds Q to the covariance of landmark j's position error (process noise: its feature drifted).
    void add_landmark_noise(size_t j, const Eigen::Matrix3d& Q);
    const std::vector<Landmark>& landmarks() const { return landmarks_; }
    static int landmark_index(size_t j) { return CORE + 3 * static_cast<int>(j); }


    const State& state() const { return prop_.state(); }  // position relative to origin()
    State world_state() const;                              // position in the world frame
    const Eigen::Vector3d& origin() const { return origin_; }

    // Moves the origin by d (world frame): every position inside the filter decreases by d and the
    // invariant position errors change by -[d]x dtheta. This is exact for the means and first order
    // for the errors: the neglected term is 1/2 dtheta x (dtheta x d), at most 1/2 |dtheta|^2 |d|.
    void shift_origin(Eigen::Vector3d d);  // by value: d may be our own position
    // Moves the origin to the current position if it is farther than origin_shift_distance_m.
    bool shift_origin_if_far();
    double td() const { return td_; }
    double baro_bias() const { return baro_bias_; }

    // Baro update: altitude = up . (world position) + baro bias + noise, with up = -gravity
    // direction. The first reading sets the bias (residual 0); after that the bias does a random
    // walk. The noise is baro_noise, times baro_inflate_factor below baro_inflate_below_agl above the
    // takeoff height or above baro_inflate_above_climb climb rate. The takeoff height is the height
    // at a start from rest (static start, or a start from a state at rest); a start in flight has
    // none. Chi2 gated. Returns true if applied.
    bool baro_update(double altitude);
    const Eigen::MatrixXd& covariance();  // invariant coordinates, applies pending propagation
    // Covariance of position, attitude and velocity as ordinary errors, order [p, theta, v].
    Eigen::Matrix<double, 9, 9> ordinary_covariance();
    bool is_static() const { return detector_.is_static(); }

    // Transition of the 15 IMU error states accumulated since the last covariance update (tests).
    const Eigen::Matrix<double, 15, 15>& pending_transition() const { return Phi_acc_; }

    // Time spent since the last call, ms (for the per-frame numbers).
    double take_ms_propagate();
    double take_ms_update();
    // Back to a saved copy of this filter (midpoint linearization redoes an update from the prior).
    // The time counters are kept, so the time of the undone update still counts.
    void restore(const Filter& saved);

private:
    void propagate_one(const ImuSample& s);
    void priors_to_invariant();
    void apply_propagation();
    void correct(const Eigen::VectorXd& dx);
    void remove_block(int i, int size);  // drop rows and columns (marginalization)

    FilterConfig cfg_;
    Eigen::Vector3d g_;
    ImuPropagator prop_;
    StaticDetector detector_;
    bool initialized_ = false;
    bool have_last_ = false;
    ImuSample last_;
    double td_ = 0, baro_bias_ = 0;
    bool baro_seen_ = false, have_ground_ = false;
    double ground_height_ = 0;     // up . world position at a start from rest
    Timestamp last_baro_t_ = 0;
    Eigen::Vector3d origin_ = Eigen::Vector3d::Zero();  // world position of the internal origin

    Eigen::MatrixXd P_;
    // Propagation since the last covariance update, accumulated over the IMU samples.
    Eigen::Matrix<double, 15, 15> Phi_acc_, Q_acc_;
    double dt_acc_ = 0;
    std::vector<Clone> clones_;
    std::vector<Landmark> landmarks_;

    // Gyro sum since the last zero-velocity update (non-overlapping, so the updates are independent).
    Eigen::Vector3d zupt_gyro_sum_ = Eigen::Vector3d::Zero();
    Eigen::Vector3d zupt_accel_sum_ = Eigen::Vector3d::Zero();
    int zupt_gyro_n_ = 0;

    double ms_propagate_ = 0, ms_update_ = 0;
};

// d(clone error) / d(td) for a clone of state s, rate_body = gyro minus bias at the clone time.
Eigen::Matrix<double, 6, 1> clone_td_jacobian(const State& s, const Eigen::Vector3d& rate_body);

// Chi-squared quantile for the given probability and degrees of freedom (Wilson-Hilferty
// approximation, within about 2 % of the exact value for 1 degree of freedom, better above).
double chi2_quantile(double probability, int dof);

}  // namespace vio
