#include "vio/config.h"

#include <algorithm>
#include <stdexcept>

namespace vio {
namespace {

void merge(YAML::Node base, const YAML::Node& over) {
    for (const auto& kv : over) {
        const std::string key = kv.first.as<std::string>();
        if (kv.second.IsMap() && base[key] && base[key].IsMap())
            merge(base[key], kv.second);
        else
            base[key] = YAML::Clone(kv.second);
    }
}

// Value at a path like "filter.init.sigma_ba", or the default if it is missing.
template <typename T>
T get(const YAML::Node& root, const std::string& path, T fallback) {
    YAML::Node n = YAML::Clone(root);
    size_t start = 0;
    while (true) {
        const size_t dot = path.find('.', start);
        const std::string key = path.substr(start, dot - start);
        if (!n.IsMap() || !n[key]) return fallback;
        n = n[key];
        if (dot == std::string::npos) break;
        start = dot + 1;
    }
    return n.as<T>();
}

}  // namespace

YAML::Node load_config(const std::vector<std::string>& files) {
    if (files.empty()) throw std::runtime_error("no config file given");
    YAML::Node cfg = YAML::LoadFile(files[0]);
    for (size_t i = 1; i < files.size(); ++i) merge(cfg, YAML::LoadFile(files[i]));
    return cfg;
}

FilterConfig filter_config(const YAML::Node& c, const ImuNoise& session_noise) {
    FilterConfig f;
    if (get(c, "filter.imu_noise_from_session", false) && session_noise.rate_hz > 0) {
        f.imu = session_noise;
    } else {
        f.imu.gyro_noise_density = get(c, "imu.gyro_noise_density", 0.004);
        f.imu.gyro_random_walk = get(c, "imu.gyro_random_walk", 0.0002);
        f.imu.accel_noise_density = get(c, "imu.accel_noise_density", 0.04);
        f.imu.accel_random_walk = get(c, "imu.accel_random_walk", 0.002);
        f.imu.rate_hz = get(c, "imu.rate_hz", 200.0);
    }
    f.td_initial_s = get(c, "imu.td_initial_ms", 0.0) * 1e-3;
    f.sigma_td_s = get(c, "imu.td_prior_sigma_ms", 30.0) * 1e-3;
    if (!get(c, "imu.td_estimate_online", true)) f.sigma_td_s = 0;  // td stays at td_initial_ms
    f.chi2_probability = get(c, "filter.chi2_probability", f.chi2_probability);
    f.init_mode = get(c, "filter.init.mode", f.init_mode);
    f.static_window_s = get(c, "filter.init.static_window_s", f.static_window_s);
    f.sigma_yaw_deg = get(c, "filter.init.sigma_yaw_deg", f.sigma_yaw_deg);
    f.sigma_att_deg = get(c, "filter.init.sigma_att_deg", f.sigma_att_deg);
    f.sigma_pos = get(c, "filter.init.sigma_pos", f.sigma_pos);
    f.sigma_vel = get(c, "filter.init.sigma_vel", f.sigma_vel);
    f.sigma_bg = get(c, "filter.init.sigma_bg", f.sigma_bg);
    f.sigma_ba = get(c, "filter.init.sigma_ba", f.sigma_ba);
    f.origin_shift_distance_m = get(c, "filter.origin_shift_distance_m", f.origin_shift_distance_m);
    f.zupt = get(c, "filter.zupt.enabled", f.zupt);
    f.zupt_accel_std = get(c, "filter.zupt.accel_std_max", f.zupt_accel_std);
    f.zupt_gyro_std = get(c, "filter.zupt.gyro_std_max", f.zupt_gyro_std);
    f.zupt_velocity_noise = get(c, "filter.zupt.velocity_noise", f.zupt_velocity_noise);
    f.zupt_landed = get(c, "filter.zupt.landed.enabled", f.zupt_landed);
    f.baro = get(c, "filter.baro.enabled", f.baro);
    f.baro_noise = get(c, "filter.baro.noise_m", f.baro_noise);
    f.baro_bias_random_walk = get(c, "filter.baro.bias_random_walk_m_s", f.baro_bias_random_walk);
    f.baro_inflate_below_agl = get(c, "filter.baro.inflate_below_agl_m", f.baro_inflate_below_agl);
    f.baro_inflate_above_climb = get(c, "filter.baro.inflate_above_climb_m_s", f.baro_inflate_above_climb);
    f.baro_inflate_factor = get(c, "filter.baro.inflate_factor", f.baro_inflate_factor);
    f.zupt_landed_accel_max = get(c, "filter.zupt.landed.accel_max", f.zupt_landed_accel_max);
    if (f.init_mode != "static" && f.init_mode != "ground_truth")
        throw std::runtime_error("filter.init.mode must be static or ground_truth");
    return f;
}

MsckfConfig msckf_config(const YAML::Node& c) {
    MsckfConfig m;
    m.max_clones = get(c, "filter.clones", m.max_clones);
    m.pixel_noise = get(c, "filter.pixel_noise_px", m.pixel_noise);
    m.line_time_s = get(c, "camera.line_time_us", 0.0) * 1e-6;
    m.min_observations = get(c, "filter.min_observations", m.min_observations);
    m.min_parallax_deg = get(c, "filter.min_parallax_deg", m.min_parallax_deg);
    m.chi2_probability = get(c, "filter.chi2_probability", m.chi2_probability);
    m.max_landmarks = get(c, "filter.max_landmarks", m.max_landmarks);
    m.landmark_max_age_s = get(c, "filter.landmark_max_age_s", m.landmark_max_age_s);
    m.landmark_max_view_change_deg =
        get(c, "filter.landmark_max_view_change_deg", m.landmark_max_view_change_deg);
    m.landmark_speed_max = get(c, "filter.landmark_speed_max", m.landmark_speed_max);
    // The age cap counts camera frames; the filter sees every update_stride-th frame only.
    const int stride = std::max(1, get(c, "filter.update_stride", 1));
    m.feature_max_age = (get(c, "tracker.max_track_age", m.feature_max_age) + stride - 1) / stride;
    m.landmark_min_parallax_deg = get(c, "filter.landmark_min_parallax_deg", m.landmark_min_parallax_deg);
    m.track_drift = get(c, "filter.track_drift.enabled", m.track_drift);
    m.drift_q = get(c, "filter.track_drift.q_px2_per_deg", m.drift_q);
    m.drift_sigma_max = get(c, "filter.track_drift.sigma_max_px", m.drift_sigma_max);
    m.midpoint_linearization = get(c, "filter.midpoint_linearization", m.midpoint_linearization);
    return m;
}

TrackerConfig tracker_config(const YAML::Node& y) {
    TrackerConfig c;
    if (const YAML::Node t = y["tracker"]) {
        c.max_features = t["max_features"].as<int>(c.max_features);
        if (t["grid"]) {
            c.grid_cols = t["grid"][0].as<int>();
            c.grid_rows = t["grid"][1].as<int>();
        }
        c.min_distance_px = t["min_distance_px"].as<double>(c.min_distance_px);
        c.klt_window_px = t["klt_window_px"].as<int>(c.klt_window_px);
        c.klt_pyramid_levels = t["klt_pyramid_levels"].as<int>(c.klt_pyramid_levels);
        c.forward_backward_max_px = t["forward_backward_max_px"].as<double>(c.forward_backward_max_px);
        c.gyro_prediction = t["gyro_prediction"].as<bool>(c.gyro_prediction);
        c.clahe = t["clahe"].as<bool>(c.clahe);
        c.max_track_age = t["max_track_age"].as<int>(c.max_track_age);
    }
    if (const YAML::Node m = y["sky_mask"]) {
        c.sky_mask = m["enabled"].as<bool>(c.sky_mask);
        c.max_off_nadir_deg = m["max_off_nadir_deg"].as<double>(c.max_off_nadir_deg);
    }
    return c;
}

QualityConfig quality_config(const YAML::Node& c) {
    QualityConfig q;
    q.lost_after_bad_s = get(c, "quality.lost_after_bad_s", q.lost_after_bad_s);
    q.min_features = get(c, "quality.min_tracks", q.min_features);
    q.min_grid_cells = get(c, "quality.min_grid_cells", q.min_grid_cells);
    q.min_tracked_ratio = get(c, "quality.min_tracked_ratio", q.min_tracked_ratio);
    q.min_inlier_ratio = get(c, "quality.min_inlier_ratio", q.min_inlier_ratio);
    q.min_tracks_for_ratio = get(c, "quality.min_tracks_for_ratio", q.min_tracks_for_ratio);
    q.imu_rate_min_hz = get(c, "quality.imu_rate_min_hz", q.imu_rate_min_hz);
    q.imu_gap_max_ms = get(c, "quality.imu_gap_max_ms", q.imu_gap_max_ms);
    q.td_change_max_ms = get(c, "quality.td_change_max_ms", q.td_change_max_ms);
    q.max_speed = get(c, "quality.max_speed_m_s", q.max_speed);
    q.vel_sigma_degraded = get(c, "output.velocity_nan_sigma_factor", 3.0) * get(c, "output.viso_vel_m_nse", 0.3);
    q.vel_sigma_lost = get(c, "quality.lost_velocity_sigma_m_s", q.vel_sigma_lost);
    q.good_frames_to_recover = get(c, "quality.good_frames_to_recover", q.good_frames_to_recover);
    q.score_floor = get(c, "output.viso_qual_min", q.score_floor);
    q.excitation_window_s = get(c, "quality.excitation.window_s", q.excitation_window_s);
    q.excitation_min_range = get(c, "quality.excitation.min_lateral_accel_range", q.excitation_min_range);
    q.excitation_min_speed = get(c, "quality.excitation.min_speed_m_s", q.excitation_min_speed);
    const TrackerConfig t = tracker_config(c);
    q.grid_cols = t.grid_cols;
    q.grid_rows = t.grid_rows;
    return q;
}

}  // namespace vio
