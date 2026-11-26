// vio_track: runs the front end on the images of a session (EuRoC or a rendered synthetic one) and
// writes the feature tracks and the per-frame quality numbers.
//
//   features.csv  "#timestamp [ns],feature_id,u [px],v [px]", raw pixels, the same format as the
//                 synthetic sessions, so vio_replay --features reads it
//   stats.csv     one row per frame, see FrameStats in include/vio/frontend.h (only with --stats)
//
// The gyro bias and the attitude for the sky mask come from the ground truth for now; in the full
// system they come from the filter. OpenCV runs in one thread, so the times are per core.

#include "vio/camera_model.h"
#include "vio/config.h"
#include "vio/frontend.h"
#include "vio/session.h"

#include <opencv2/core/utility.hpp>
#include <opencv2/imgcodecs.hpp>

#include <algorithm>
#include <cstdio>
#include <fstream>
#include <iostream>
#include <string>
#include <vector>

namespace {

void usage() {
    std::cerr
        << "usage: vio_track <session folder> --out features.csv [--config config/vio.yaml]\n"
           "                 [--stats stats.csv] [--no-gyro] [--clahe | --no-clahe] [--no-sky-mask]\n"
           "                 [--frame-step N] [--start seconds] [--duration seconds] [--max-track-age N]\n"
           "  --no-sky-mask  needed for EuRoC: its camera looks forward, not down\n"
           "  --frame-step   use every N-th image (more motion between frames)\n";
}

void write_stats_row(std::ofstream& out, const vio::FrameStats& s) {
    char buf[512];
    std::snprintf(buf, sizeof(buf),
                  "%lld,%d,%d,%d,%d,%d,%d,%d,%.4f,%.4f,%.4f,%d,%d,%.4f,%.4f,%.2f,%.2f,%.2f,%.5f,%.3f,%.3f,"
                  "%.3f,%.3f\n",
                  static_cast<long long>(s.t), s.attempted, s.tracked, s.lost_klt, s.lost_fb,
                  s.lost_mask, s.added, s.total, s.fb_median, s.fb_p90, s.pred_median, s.cells_usable,
                  s.cells_occupied, s.max_cell_share, s.usable_fraction, s.image.brightness,
                  s.image.contrast, s.image.blur, s.image.saturated, s.ms_prepare, s.ms_track,
                  s.ms_detect, s.ms_total);
    out << buf;
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        usage();
        return 1;
    }
    const std::string session_path = argv[1];
    std::string config_path = "config/vio.yaml", out_path, stats_path;  // no default files
    bool no_gyro = false, no_sky_mask = false;
    int clahe = -1;  // -1 = as in the config
    int frame_step = 1, max_track_age = -1;
    double start_offset = 0, max_duration = 0;
    for (int i = 2; i < argc; ++i) {
        const std::string a = argv[i];
        const bool has_value = i + 1 < argc;
        if (a == "--config" && has_value) config_path = argv[++i];
        else if (a == "--out" && has_value) out_path = argv[++i];
        else if (a == "--stats" && has_value) stats_path = argv[++i];
        else if (a == "--frame-step" && has_value) frame_step = std::max(1, std::stoi(argv[++i]));
        else if (a == "--start" && has_value) start_offset = std::stod(argv[++i]);
        else if (a == "--duration" && has_value) max_duration = std::stod(argv[++i]);
        else if (a == "--max-track-age" && has_value) max_track_age = std::stoi(argv[++i]);
        else if (a == "--no-gyro") no_gyro = true;
        else if (a == "--clahe") clahe = 1;
        else if (a == "--no-clahe") clahe = 0;
        else if (a == "--no-sky-mask") no_sky_mask = true;
        else {
            usage();
            return 1;
        }
    }
    if (out_path.empty()) {
        usage();
        return 1;
    }

    vio::Session session;
    vio::TrackerConfig cfg;
    try {
        session = vio::load_session(session_path);
        cfg = vio::tracker_config(vio::load_config({config_path}));
    } catch (const std::exception& e) {
        std::cerr << "error: " << e.what() << "\n";
        return 1;
    }
    if (session.frames.front().image_path.empty()) {
        std::cerr << "error: this session has feature tracks, not images\n";
        return 1;
    }
    if (no_gyro) cfg.gyro_prediction = false;
    if (clahe >= 0) cfg.clahe = clahe == 1;
    if (no_sky_mask) cfg.sky_mask = false;
    if (max_track_age >= 0) cfg.max_track_age = max_track_age;
    const bool have_gt = !session.ground_truth.empty();
    if (cfg.sky_mask && !have_gt) {
        std::cerr << "note: no ground truth for the attitude, sky mask off\n";
        cfg.sky_mask = false;
    }
    cv::setNumThreads(1);

    const vio::Camera cam(session.camera);
    const Eigen::Matrix3d R_bc = session.camera.T_body_cam.block<3, 3>(0, 0);
    const Eigen::Vector3d down_world = session.gravity_world.normalized();
    vio::Tracker tracker(cam, cfg);

    std::ofstream out(out_path), stats;
    if (!stats_path.empty()) stats.open(stats_path);
    if (!out || (!stats_path.empty() && !stats)) {
        std::cerr << "error: cannot write " << out_path << " or " << stats_path << "\n";
        return 1;
    }
    out << "#timestamp [ns],feature_id,u [px],v [px]\n";
    if (stats.is_open())
        stats << "#timestamp [ns],attempted,tracked,lost_klt,lost_fb,lost_mask,added,total,fb_median,"
                 "fb_p90,pred_median,cells_usable,cells_occupied,max_cell_share,usable_fraction,brightness,"
                 "contrast,blur,saturated,ms_prepare,ms_track,ms_detect,ms_total\n";

    auto ns = [](double sec) { return static_cast<vio::Timestamp>(sec * 1e9); };
    const vio::Timestamp t_first = session.imu.front().t + ns(start_offset);
    const vio::Timestamp t_last =
        max_duration > 0 ? t_first + ns(max_duration) : session.frames.back().t;
    std::vector<double> ms;
    double success_sum = 0;
    int n_frames = 0, n_success = 0;
    vio::Timestamp t_prev = -1;
    int k = 0;
    for (const auto& frame : session.frames) {
        if (frame.t < t_first || frame.t > t_last || k++ % frame_step != 0) continue;
        vio::FrameInput in;
        in.t = frame.t;
        in.image = cv::imread(frame.image_path, cv::IMREAD_GRAYSCALE);
        if (in.image.empty()) {
            std::cerr << "error: cannot read " << frame.image_path << "\n";
            return 1;
        }
        vio::State gt;
        const bool gt_ok = have_gt && vio::interpolate_ground_truth(session.ground_truth, frame.t, gt);
        if (t_prev >= 0) {
            const Eigen::Matrix3d R_b = vio::integrate_gyro(session.imu, t_prev, frame.t, gt.bg);
            in.R_cprev_ccur = R_bc.transpose() * R_b * R_bc;
        }
        if (gt_ok) {
            in.have_attitude = true;
            in.R_world_cam = gt.q.toRotationMatrix() * R_bc;
            in.down_world = down_world;
        }
        const vio::FrameStats st = tracker.process(in);
        t_prev = frame.t;

        for (const auto& f : tracker.features()) {
            char buf[128];
            std::snprintf(buf, sizeof(buf), "%lld,%lld,%.4f,%.4f\n", static_cast<long long>(frame.t),
                          static_cast<long long>(f.id), f.uv.x(), f.uv.y());
            out << buf;
        }
        if (stats.is_open()) write_stats_row(stats, st);
        ms.push_back(st.ms_total);
        if (st.attempted > 0) {
            success_sum += static_cast<double>(st.tracked) / st.attempted;
            ++n_success;
        }
        ++n_frames;
    }
    if (n_frames == 0) {
        std::cerr << "error: no frames in the selected time range\n";
        return 1;
    }

    std::sort(ms.begin(), ms.end());
    std::printf("frames       %d (every %d. image), %dx%d %s\n", n_frames, frame_step, cam.width(),
                cam.height(), session.camera.model.c_str());
    std::printf("settings     gyro prediction %s, CLAHE %s, sky mask %s\n",
                cfg.gyro_prediction ? "on" : "off", cfg.clahe ? "on" : "off", cfg.sky_mask ? "on" : "off");
    std::printf("success rate %.3f (mean over frames)\n", n_success ? success_sum / n_success : 0.0);
    std::printf("time         mean %.2f ms, p95 %.2f ms, max %.2f ms per frame\n",
                [&] { double s = 0; for (double m : ms) s += m; return s / ms.size(); }(),
                ms[std::min(ms.size() - 1, ms.size() * 95 / 100)], ms.back());
    std::printf("output       %s, %s\n", out_path.c_str(), stats_path.c_str());
    return 0;
}
