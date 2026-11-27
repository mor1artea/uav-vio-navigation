// vio_replay: replays a session (synthetic or EuRoC) in time order and writes a trajectory.
//
//   --mode gt   writes the ground truth at the camera times (checks the replay and metrics chain)
//   --mode imu  pure IMU integration from the ground-truth start state (dead reckoning)
//   --mode vio  the filter (settings from --config, see config/replay_*.yaml for datasets). Feature
//               tracks come from the session (synthetic), from --features (vio_track output), or
//               from the tracker run here on the session's images. Only the last one is the full
//               system: the filter gives the tracker its attitude (sky mask), gyro bias and the
//               landmarks it must keep.
//
// Output: CSV, one row per camera frame. The first 8 columns are always
// "timestamp [ns],p_x,p_y,p_z,q_w,q_x,q_y,q_z", so tools/metrics.py reads every mode. The vio mode
// adds velocity, biases, td, baro bias, the pose covariance (upper triangle of the 6x6 block in
// the order p_x p_y p_z th_x th_y th_z, attitude error in the world frame), the velocity
// covariance, and per-frame numbers; the header names every column. With quality.enabled the
// quality monitor (vio/quality.h) adds its columns at the end; the columns before them do not change:
//   q_state 0 GOOD / 1 DEGRADED / 2 LOST, q_score (-1 in LOST), q_fails (QualityFail bits: 1 features,
//   2 grid cells, 4 forward-backward rate, 8 inlier ratio, 16 joint gate, 32 IMU rate, 64 IMU gap,
//   128 td change, 256 speed), q_sigma_v (m/s, the velocity covariance in its worst direction), q_features,
//   q_cells, q_pos_valid and q_vel_valid (what would be sent), q_poserr463 (posErr of ArduPilot
//   4.6.3 from our position variances, before its clamps), e1_flag, e1_a_lat (m/s^2, NaN below the
//   speed limit), e1_range (NaN until the window is full), img_brightness, img_blur (NaN without
//   images).
// With --inject (vio/inject.h) one more column at the very end: fault, the bits (1 << kind) of the
// faults active at the frame's camera time (1 blackout, 2 frozen, 4 outliers, 8 imu_gap, 16 td_step).

#include "vio/camera_model.h"
#include "vio/config.h"
#include "vio/filter.h"
#include "vio/frontend.h"
#include "vio/inject.h"
#include "vio/msckf.h"
#include "vio/quality.h"

#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>
#include "vio/imu_propagation.h"
#include "vio/session.h"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <random>
#include <iostream>
#include <stdexcept>
#include <string>
#include <vector>

namespace {

void usage() {
    std::cerr << "usage: vio_replay <session folder> --out trajectory.csv [--mode gt|imu|vio]\n"
                 "                  [--start seconds] [--duration seconds] [--features features.csv]\n"
                 "                  [--config file] ...  (default config/vio.yaml; later files override)\n"
                 "  --start     skip this many seconds from the first IMU sample (e.g. hand-carried part)\n"
                 "  --features  read feature tracks from this file instead of the session's images\n"
                 "  --inject    a fault, repeatable (vio/inject.h): blackout:T:D, frozen:T:D,\n"
                 "              outliers:T:D:SHARE:PX, imu_gap:T:D, td_step:T:MS (T in s from the first IMU sample)\n";
}

void write_row(std::ofstream& out, const vio::State& s) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), "%lld,%.6f,%.6f,%.6f,%.9f,%.9f,%.9f,%.9f\n",
                  static_cast<long long>(s.t), s.p.x(), s.p.y(), s.p.z(), s.q.w(), s.q.x(),
                  s.q.y(), s.q.z());
    out << buf;
}

std::string vio_header(bool quality, bool fault_column) {
    std::string h = "#timestamp [ns],p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,bg_x,bg_y,bg_z,"
                    "ba_x,ba_y,ba_z,td_ms,td_sigma_ms,baro_bias";
    for (int i = 0; i < 6; ++i)
        for (int j = i; j < 6; ++j) h += ",Cpose_" + std::to_string(i) + std::to_string(j);
    for (int i = 0; i < 3; ++i)
        for (int j = i; j < 3; ++j) h += ",Cvel_" + std::to_string(i) + std::to_string(j);
    return h + ",static,n_msckf,n_landmarks,n_rejected,n_no_geometry,n_landmark_rejected,update_rejected,"
               "ms_track,ms_propagate,ms_update" +
           (quality ? std::string(",q_state,q_score,q_fails,q_sigma_v,q_features,q_cells,q_pos_valid,"
                                  "q_vel_valid,q_poserr463,e1_flag,e1_a_lat,e1_range,img_brightness,img_blur")
                    : std::string()) +
           (fault_column ? ",fault" : "") + "\n";
}

struct FrameNumbers {
    int zupt = 0;  // zero-velocity update: 0 none, 1 still, 2 landed (Filter::zero_velocity_update)
    bool update_rejected = false;
    int n_msckf = 0, n_landmarks = 0, n_rejected = 0, n_no_geometry = 0, n_landmark_rejected = 0;
    double ms_track = 0, ms_propagate = 0, ms_update = 0;
};

// C: covariance of [p, theta, v] as ordinary errors (Filter::ordinary_covariance). No line end.
void write_vio_row(std::ofstream& out, const vio::State& s, double td, double td_sigma,
                   double baro_bias, const Eigen::Matrix<double, 9, 9>& C, const FrameNumbers& k) {
    char buf[256];
    std::snprintf(buf, sizeof(buf),
                  "%lld,%.6f,%.6f,%.6f,%.9f,%.9f,%.9f,%.9f,%.6f,%.6f,%.6f,%.7f,%.7f,%.7f,%.6f,%.6f,"
                  "%.6f,%.4f,%.4f,%.4f",
                  static_cast<long long>(s.t), s.p.x(), s.p.y(), s.p.z(), s.q.w(), s.q.x(),
                  s.q.y(), s.q.z(), s.v.x(), s.v.y(), s.v.z(), s.bg.x(), s.bg.y(), s.bg.z(),
                  s.ba.x(), s.ba.y(), s.ba.z(), td * 1e3, td_sigma * 1e3, baro_bias);
    out << buf;
    for (int i = 0; i < 6; ++i)
        for (int j = i; j < 6; ++j) {
            std::snprintf(buf, sizeof(buf), ",%.6e", C(i, j));
            out << buf;
        }
    for (int i = 6; i < 9; ++i)
        for (int j = i; j < 9; ++j) {
            std::snprintf(buf, sizeof(buf), ",%.6e", C(i, j));
            out << buf;
        }
    std::snprintf(buf, sizeof(buf), ",%d,%d,%d,%d,%d,%d,%d,%.3f,%.3f,%.3f", k.zupt, k.n_msckf,
                  k.n_landmarks, k.n_rejected, k.n_no_geometry, k.n_landmark_rejected,
                  k.update_rejected ? 1 : 0, k.ms_track, k.ms_propagate, k.ms_update);
    out << buf;
}

struct QualityNumbers {
    vio::QualityResult q;
    vio::OutputDecision out;
    int features = 0, cells = 0;
    double poserr = 0;
    vio::ExcitationMonitor::Result e1;
    double brightness = NAN, blur = NAN;
};

void write_quality(std::ofstream& out, const QualityNumbers& n) {
    char buf[256];
    std::snprintf(buf, sizeof(buf), ",%d,%d,%u,%.4f,%d,%d,%d,%d,%.4f,%d,%.4f,%.4f,%.2f,%.2f",
                  static_cast<int>(n.q.state), n.q.score, n.q.fails, n.q.sigma_v, n.features, n.cells,
                  n.out.position_valid ? 1 : 0, n.out.velocity_valid ? 1 : 0, n.poserr, n.e1.flag ? 1 : 0,
                  n.e1.a_lat, n.e1.range, n.brightness, n.blur);
    out << buf;
}

// Short summary of the session, to spot broken data before looking at any estimate.
void print_summary(const vio::Session& s) {
    const double dur = vio::to_sec(s.imu.back().t - s.imu.front().t);
    double max_imu_gap = 0, max_frame_gap = 0;
    for (size_t i = 1; i < s.imu.size(); ++i)
        max_imu_gap = std::max(max_imu_gap, vio::to_sec(s.imu[i].t - s.imu[i - 1].t));
    for (size_t i = 1; i < s.frames.size(); ++i)
        max_frame_gap = std::max(max_frame_gap, vio::to_sec(s.frames[i].t - s.frames[i - 1].t));
    size_t n_obs = 0;
    for (const auto& f : s.frames) n_obs += f.features.size();

    std::printf("session      %s\n", s.path.c_str());
    std::printf("duration     %.1f s\n", dur);
    std::printf("imu          %zu samples, %.1f Hz, max gap %.1f ms\n", s.imu.size(),
                (s.imu.size() - 1) / dur, max_imu_gap * 1e3);
    std::printf("camera       %zu frames, %s, %dx%d, max gap %.1f ms\n", s.frames.size(),
                s.camera.model.c_str(), s.camera.width, s.camera.height, max_frame_gap * 1e3);
    if (n_obs > 0)
        std::printf("features     %.1f per frame (tracks from a file, no images)\n",
                    static_cast<double>(n_obs) / s.frames.size());
    std::printf("ground truth %zu poses\n", s.ground_truth.size());
    std::printf("gravity      %.2f %.2f %.2f\n", s.gravity_world.x(), s.gravity_world.y(),
                s.gravity_world.z());
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 1;
    }
    const std::string session_path = argv[1];
    std::string mode = "imu";
    std::string out_path;  // required: no default file in the current folder
    std::string features_file;
    std::vector<std::string> config_files;
    std::vector<vio::Fault> faults;
    double start_offset = 0;  // seconds after the first IMU sample
    double max_duration = 0;  // 0 = whole session
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--mode" && i + 1 < argc) {
            mode = argv[++i];
        } else if (a == "--out" && i + 1 < argc) {
            out_path = argv[++i];
        } else if (a == "--config" && i + 1 < argc) {
            config_files.push_back(argv[++i]);
        } else if (a == "--features" && i + 1 < argc) {
            features_file = argv[++i];
        } else if (a == "--inject" && i + 1 < argc) {
            try {
                faults.push_back(vio::parse_fault(argv[++i]));
            } catch (const std::exception& e) {
                std::cerr << "error: " << e.what() << "\n";
                return 1;
            }
        } else if (a == "--start" && i + 1 < argc) {
            start_offset = std::stod(argv[++i]);
        } else if (a == "--duration" && i + 1 < argc) {
            max_duration = std::stod(argv[++i]);
        } else {
            usage();
            return 1;
        }
    }
    if (out_path.empty() || (mode != "gt" && mode != "imu" && mode != "vio")) {
        usage();
        return 1;
    }

    if (config_files.empty()) config_files.push_back("config/vio.yaml");
    vio::Session session;
    vio::FilterConfig filter_cfg;
    vio::MsckfConfig msckf_cfg;
    vio::TrackerConfig tracker_cfg;
    double image_still_px = 1.0;
    double max_flag_age_s = 0.1;
    int update_stride = 1;
    bool quality_on = true;
    vio::QualityConfig quality_cfg;
    double rate_window_s = 0.03;
    try {
        session = vio::load_session(session_path, features_file);
        const YAML::Node cfg = vio::load_config(config_files);
        filter_cfg = vio::filter_config(cfg, session.imu_noise);
        msckf_cfg = vio::msckf_config(cfg);
        tracker_cfg = vio::tracker_config(cfg);
        quality_cfg = vio::quality_config(cfg);
        if (cfg["quality"] && cfg["quality"]["enabled"]) quality_on = cfg["quality"]["enabled"].as<bool>();
        if (cfg["filter"]["update_stride"]) update_stride = std::max(1, cfg["filter"]["update_stride"].as<int>());
        if (cfg["imu"]["td_rate_window_ms"]) rate_window_s = cfg["imu"]["td_rate_window_ms"].as<double>() * 1e-3;
        if (cfg["filter"]["zupt"]["image_motion_max_px"])
            image_still_px = cfg["filter"]["zupt"]["image_motion_max_px"].as<double>();
        if (cfg["filter"]["zupt"]["landed"] && cfg["filter"]["zupt"]["landed"]["max_flag_age_s"])
            max_flag_age_s = cfg["filter"]["zupt"]["landed"]["max_flag_age_s"].as<double>();
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    // Faults are timed from the first IMU sample of the session as loaded (before any gap is cut).
    const vio::Timestamp t_fault_ref = session.imu.empty() ? 0 : session.imu.front().t;
    vio::apply_session_faults(session, faults, t_fault_ref);
    print_summary(session);
    for (const auto& fl : faults) {
        const char* names[] = {"blackout", "frozen", "outliers", "imu_gap", "td_step"};
        char extra[96] = "";
        if (fl.kind == vio::Fault::TD_STEP) std::snprintf(extra, sizeof(extra), ", td step %.1f ms", fl.td_ms);
        if (fl.kind == vio::Fault::OUTLIERS)
            std::snprintf(extra, sizeof(extra), ", share %.2f, %.1f px", fl.share, fl.px);
        std::printf("fault        %s at %.2f s for %.2f s%s\n", names[fl.kind], fl.t_s, fl.dur_s, extra);
    }
    if (session.ground_truth.empty()) {
        std::cerr << "error: replay needs ground truth (start state, yaw reference or output)\n";
        return 1;
    }
    // Sessions with images run the tracker; feature sessions may start with frames without features.
    const bool run_tracker = mode == "vio" && !session.frames.front().image_path.empty();
    const bool any_features = std::any_of(session.frames.begin(), session.frames.end(),
                                          [](const vio::CameraFrame& f) { return !f.features.empty(); });
    if (mode == "vio" && !run_tracker && !any_features) {
        std::cerr << "error: vio mode needs feature tracks or images\n";
        return 1;
    }
    cv::setNumThreads(1);

    // Start at the first IMU sample after --start that has ground truth.
    const vio::Timestamp t_skip =
        session.imu.front().t + static_cast<vio::Timestamp>(start_offset * 1e9);
    vio::State start;
    size_t first_imu = 0;
    while (first_imu < session.imu.size() &&
           (session.imu[first_imu].t < t_skip ||
            !vio::interpolate_ground_truth(session.ground_truth, session.imu[first_imu].t, start)))
        ++first_imu;
    if (first_imu == session.imu.size()) {
        std::cerr << "error: IMU and ground truth do not overlap\n";
        return 1;
    }
    const vio::Timestamp t_start = session.imu[first_imu].t;
    const vio::Timestamp t_end =
        max_duration > 0 ? t_start + static_cast<vio::Timestamp>(max_duration * 1e9)
                         : session.imu.back().t;

    std::ofstream out(out_path);
    if (!out) {
        std::cerr << "error: cannot write " << out_path << "\n";
        return 1;
    }
    out << (mode == "vio" ? vio_header(quality_on, !faults.empty()) : "#timestamp [ns],p_x,p_y,p_z,q_w,q_x,q_y,q_z\n");

    vio::ImuPropagator prop(session.gravity_world);
    prop.reset(start);
    vio::Filter filter(filter_cfg, session.gravity_world);
    const vio::Camera camera(session.camera);
    vio::Msckf msckf(camera, session.camera.T_body_cam, msckf_cfg);
    vio::Timestamp t_init = -1;
    size_t n_out = 0, n_static = 0, n_used = 0, n_rejected = 0, n_no_geometry = 0;
    double ms_sum = 0, ms_visual_sum = 0;

    // Frames are processed at their time in the IMU clock (frame time + td, replay_in_imu_time), and
    // the IMU is interpolated there, so the clone is taken exactly at that time for either sign of td.
    auto td_ns = [&] { return static_cast<vio::Timestamp>(std::llround(filter.td() * 1e9)); };
    vio::Tracker tracker(camera, tracker_cfg);
    const Eigen::Matrix3d R_bc = session.camera.T_body_cam.block<3, 3>(0, 0);
    vio::Timestamp t_prev_imu = -1;
    int frame_index = 0, n_tracked = 0;
    double ms_track_sum = 0;

    vio::CameraFrame prev_frame;
    // Quality monitor (passive): signals since the last update frame.
    vio::QualityMonitor monitor(quality_cfg);
    vio::ExcitationMonitor excitation(quality_cfg);
    vio::Timestamp t_prev_update = -1;
    double td_prev_update = 0;
    double frame_tracked_ratio = -1, frame_brightness = NAN, frame_blur = NAN;
    size_t n_state[3] = {0, 0, 0}, n_e1 = 0, n_e1_window = 0;
    // Fault injection: the frame or features held by a frozen fault, the outlier generator.
    cv::Mat frozen_image;
    std::vector<vio::FeatureObs> frozen_features;
    bool frozen_held = false;
    std::mt19937 outlier_rng(12345);
    unsigned frame_faults = 0;
    size_t n_shifts = 0;
    size_t n_late = 0;
    long n_baro_seen = 0;  // baro samples up to here are used or skipped
    size_t n_baro = 0;     // baro updates applied  // frames reached when the filter was already past their time: must stay 0
    auto process_frame = [&](const vio::CameraFrame& frame, const vio::ImuSample* next) {
        const vio::Timestamp t_imu = frame.t + td_ns();
        n_late += filter.state().t > t_imu;
        filter.propagate_to(t_imu, next);
        FrameNumbers k;
        vio::CameraFrame f = frame;
        frame_faults = vio::active_faults(faults, frame.t, t_fault_ref);
        const bool blackout = frame_faults & (1u << vio::Fault::BLACKOUT);
        const bool frozen = frame_faults & (1u << vio::Fault::FROZEN);
        if (!frozen) {
            frozen_held = false;
            frozen_image.release();
        }
        if (!run_tracker) {
            if (frozen) {
                if (!frozen_held) frozen_features = f.features;
                f.features = frozen_features;
                frozen_held = true;
            }
            if (blackout) f.features.clear();
        }
        if (run_tracker) {
            const auto t0 = std::chrono::steady_clock::now();
            vio::FrameInput in;
            in.t = f.t;
            in.image = cv::imread(f.image_path, cv::IMREAD_GRAYSCALE);
            if (frozen) {
                if (!frozen_held) frozen_image = in.image.clone();
                in.image = frozen_image.clone();
                frozen_held = true;
            }
            if (blackout) in.image.setTo(0);
            if (t_prev_imu >= 0)
                in.R_cprev_ccur = R_bc.transpose() *
                                  vio::integrate_gyro(session.imu, t_prev_imu, t_imu, filter.state().bg) *
                                  R_bc;
            in.have_attitude = true;
            in.R_world_cam = filter.state().q.toRotationMatrix() * R_bc;
            in.down_world = session.gravity_world.normalized();
            tracker.set_keep(msckf.landmark_ids(filter));
            const vio::FrameStats fs = tracker.process(in);
            frame_tracked_ratio = fs.attempted > 0 ? static_cast<double>(fs.tracked) / fs.attempted : -1;
            frame_brightness = fs.image.brightness;
            frame_blur = fs.image.blur;
            f.features = tracker.features();
            k.ms_track = std::chrono::duration<double, std::milli>(
                             std::chrono::steady_clock::now() - t0).count();
            ms_track_sum += k.ms_track;
            ++n_tracked;
        }
        for (const auto& fl : faults)
            if (fl.kind == vio::Fault::OUTLIERS && (frame_faults & (1u << vio::Fault::OUTLIERS)))
                vio::move_outliers(f.features, fl.share, fl.px, outlier_rng);
        t_prev_imu = t_imu;
        // The visual update runs on every update_stride-th frame only (the tracker on all of them):
        // the same number of clones then spans a longer time, which gives slow flight and the
        // approach to a hover enough parallax.
        if (frame_index++ % update_stride != 0) return;
        const double motion = vio::median_feature_motion(prev_frame, f);  // -1: no features in common
        prev_frame = f;
        // FC landed flag: the newest message at or before this time says "on the ground" and is at
        // most max_flag_age_s old. The flag can clear after the liftoff (AltHold, a fast throttle
        // push), so the image and the accelerometer must agree as well (Filter::zero_velocity_update).
        bool landed = false;
        const auto msg = std::upper_bound(session.landed.begin(), session.landed.end(), t_imu,
                                          [](vio::Timestamp t, const auto& m) { return t < m.first; });
        if (msg != session.landed.begin()) {
            const auto& m = *std::prev(msg);
            landed = m.second == 1 && vio::to_sec(t_imu - m.first) <= max_flag_age_s;
        }
        k.zupt = filter.zero_velocity_update(motion >= 0 && motion < image_still_px,
                                             motion >= image_still_px, landed);
        // Baro: the newest sample at or before this time, each sample used once.
        const auto b = std::upper_bound(session.baro.begin(), session.baro.end(), t_imu,
                                        [](vio::Timestamp t, const auto& m) { return t < m.first; });
        if (b != session.baro.begin() && b - session.baro.begin() > n_baro_seen) {
            n_baro_seen = b - session.baro.begin();
            n_baro += filter.baro_update(std::prev(b)->second);
        }
        // Body rate at the frame time for the td Jacobian and the rolling-shutter correction: the
        // mean over a window centred on it (needs the IMU a little after the frame).
        const auto half = static_cast<vio::Timestamp>(rate_window_s * 0.5e9);
        const Eigen::Vector3d gyro = vio::average_gyro(session.imu, t_imu - half, t_imu + half);
        const auto t_visual = std::chrono::steady_clock::now();
        const vio::MsckfStats ms = msckf.process(filter, f, k.zupt != 0, gyro);
        ms_visual_sum += std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t_visual).count();
        n_shifts += filter.shift_origin_if_far();
        k.n_msckf = ms.used;
        k.n_rejected = ms.rejected;
        k.n_landmarks = ms.landmarks;
        k.n_landmark_rejected = ms.landmark_rejected;
        k.n_no_geometry = ms.no_geometry;
        k.update_rejected = ms.update_rejected;
        k.ms_propagate = filter.take_ms_propagate();
        k.ms_update = filter.take_ms_update();
        n_static += k.zupt != 0;
        n_used += ms.used;
        n_rejected += ms.rejected;
        n_no_geometry += ms.no_geometry;
        ms_sum += k.ms_propagate + k.ms_update;
        const double td_sigma = std::sqrt(filter.covariance()(vio::Filter::TD, vio::Filter::TD));
        const Eigen::Matrix<double, 9, 9> C = filter.ordinary_covariance();
        write_vio_row(out, filter.world_state(), filter.td(), td_sigma, filter.baro_bias(), C, k);
        if (quality_on) {
            QualityNumbers qn;
            const vio::ImuWindow iw = t_prev_update >= 0 ? vio::summarize_imu(session.imu, t_prev_update, t_imu)
                                                         : vio::ImuWindow{};
            const vio::State& s = filter.state();
            vio::FrameSignals sig;
            sig.t = t_imu;
            sig.features = qn.features = static_cast<int>(f.features.size());
            sig.cells = qn.cells = vio::occupied_cells(f.features, session.camera.width, session.camera.height,
                                                       quality_cfg.grid_cols, quality_cfg.grid_rows);
            sig.tracked_ratio = run_tracker ? frame_tracked_ratio : -1;
            sig.msckf_used = ms.used;
            sig.msckf_rejected = ms.rejected;
            sig.joint_rejected = ms.update_rejected;
            sig.imu_rate_hz = iw.rate_hz;
            sig.imu_gap_ms = iw.max_gap_ms;
            sig.td_change_ms = t_prev_update >= 0 ? (filter.td() - td_prev_update) * 1e3 : 0;
            sig.speed = s.v.norm();
            sig.C_vel = C.block<3, 3>(6, 6);
            qn.q = monitor.update(sig);
            qn.out = vio::decide_output(qn.q, monitor, C.block<3, 3>(0, 0), quality_cfg);
            qn.poserr = vio::poserr_463(C.block<3, 3>(0, 0));
            // E1: mean specific force since the last update frame, rotated with the attitude now.
            const Eigen::Vector3d a_world = iw.n > 0 ? Eigen::Vector3d(s.q * (iw.mean_accel - s.ba) + session.gravity_world)
                                                     : Eigen::Vector3d::Constant(NAN);
            qn.e1 = excitation.add(t_imu, a_world, s.v, session.gravity_world.normalized());
            if (run_tracker) {
                qn.brightness = frame_brightness;
                qn.blur = frame_blur;
            }
            write_quality(out, qn);
            ++n_state[static_cast<int>(qn.q.state)];
            n_e1 += qn.e1.flag;
            n_e1_window += std::isfinite(qn.e1.range);
            t_prev_update = t_imu;
            td_prev_update = filter.td();
        }
        if (!faults.empty()) out << ',' << frame_faults;
        out << '\n';
        ++n_out;
    };

    auto on_imu = [&](const vio::ImuSample& m) {
        if (m.t < t_start || m.t > t_end) return;
        if (mode != "vio") {
            prop.add(m);
            return;
        }
        if (!filter.initialized() && filter_cfg.init_mode == "ground_truth") {
            vio::State gt;
            if (vio::interpolate_ground_truth(session.ground_truth, m.t, gt)) {
                filter.init_from_state(gt);
                t_init = m.t;
            }
        }
        filter.add_imu(m);
        if (!filter.initialized()) {
            // Static start; yaw and the origin from the ground truth (the compass on the drone).
            vio::State gt;
            if (vio::interpolate_ground_truth(session.ground_truth, m.t, gt) &&
                filter.try_init_static(gt.q.toRotationMatrix(), gt.p))
                t_init = m.t;
        }
    };

    if (mode == "vio") {
        vio::replay_in_imu_time(
            session, td_ns, on_imu, [&](const vio::CameraFrame& f, const vio::ImuSample* next) {
                if (f.t < t_start || f.t > t_end || !filter.initialized()) return;
                process_frame(f, next);
            });
    } else {
        vio::replay(session, on_imu, [&](const vio::CameraFrame& f) {
            if (f.t < t_start || f.t > t_end) return;
            vio::State s;
            if (mode == "gt") {
                if (!vio::interpolate_ground_truth(session.ground_truth, f.t, s)) return;
            } else {
                s = prop.predict(f.t);
            }
            write_row(out, s);
            ++n_out;
        });
    }

    std::printf("mode         %s\n", mode.c_str());
    if (mode == "vio") {
        if (t_init < 0) {
            std::cerr << "error: the filter never started (no still period for a static start?)\n";
            return 1;
        }
        std::printf("filter       started at %.2f s (%s), zero-velocity updates on %zu frames, "
                    "%zu frames late, origin moved %zu times, %zu baro updates\n",
                    vio::to_sec(t_init - session.imu.front().t), filter_cfg.init_mode.c_str(),
                    n_static, n_late, n_shifts, n_baro);
        auto per_frame = [&](size_t n) { return n_out ? static_cast<double>(n) / n_out : 0.0; };
        std::printf("features     per frame: %.1f in updates, %.1f rejected by chi2 (%.1f %%), "
                    "%.1f without enough parallax\n",
                    per_frame(n_used), per_frame(n_rejected),
                    100.0 * n_rejected / std::max<size_t>(1, n_used + n_rejected),
                    per_frame(n_no_geometry));
        std::printf("time         %.3f ms per update frame (propagation + updates), visual update "
                    "%.3f ms (all of it: tracks, triangulation, update)",
                    n_out ? ms_sum / n_out : 0.0, n_out ? ms_visual_sum / n_out : 0.0);
        if (run_tracker)
            std::printf(", tracker %.2f ms per camera frame (with PNG decoding)",
                        n_tracked ? ms_track_sum / n_tracked : 0.0);
        std::printf("\n");
    }
    if (mode == "vio" && quality_on && n_out > 0)
        std::printf("quality      GOOD %.1f %%, DEGRADED %.1f %%, LOST %.1f %% of update frames; low "
                    "excitation flag on %.1f %% (of %zu frames with a full window)\n",
                    100.0 * n_state[0] / n_out, 100.0 * n_state[1] / n_out, 100.0 * n_state[2] / n_out,
                    100.0 * n_e1 / std::max<size_t>(1, n_e1_window), n_e1_window);
    std::printf("output       %zu poses -> %s\n", n_out, out_path.c_str());
    return 0;
}
