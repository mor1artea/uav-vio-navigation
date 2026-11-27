#include "vio/session.h"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <filesystem>
#include <fstream>
#include <sstream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace vio {
namespace {

// Reads a CSV file, skipping empty lines and lines starting with '#'. Every row must have at least
// min_cols numeric columns.
std::vector<std::vector<double>> read_csv(const fs::path& file, size_t min_cols) {
    std::ifstream in(file);
    if (!in) throw std::runtime_error("cannot open " + file.string());
    std::vector<std::vector<double>> rows;
    std::string line;
    size_t line_no = 0;
    while (std::getline(in, line)) {
        ++line_no;
        if (line.empty() || line[0] == '#' || line[0] == '\r') continue;
        std::vector<double> row;
        std::stringstream ss(line);
        std::string cell;
        while (std::getline(ss, cell, ',')) row.push_back(std::stod(cell));
        if (row.size() < min_cols) {
            throw std::runtime_error(file.string() + ":" + std::to_string(line_no) +
                                     ": expected at least " + std::to_string(min_cols) + " columns");
        }
        rows.push_back(std::move(row));
    }
    return rows;
}

// EuRoC yaml files start with "%YAML:1.0" (OpenCV style), which yaml-cpp does not accept.
YAML::Node load_yaml(const fs::path& file) {
    std::ifstream in(file);
    if (!in) throw std::runtime_error("cannot open " + file.string());
    std::stringstream text;
    std::string line;
    while (std::getline(in, line)) {
        if (line.rfind("%YAML", 0) == 0) continue;
        text << line << "\n";
    }
    return YAML::Load(text.str());
}

// Time stamps are integer nanoseconds. EuRoC uses Unix time (about 1.4e18 ns), which a double cannot
// hold exactly (exact only up to 2^53, about 9e15), so the first column is parsed again as an integer.
std::vector<Timestamp> read_timestamps(const fs::path& file) {
    std::ifstream in(file);
    std::vector<Timestamp> out;
    std::string line;
    while (std::getline(in, line)) {
        if (line.empty() || line[0] == '#' || line[0] == '\r') continue;
        out.push_back(std::stoll(line.substr(0, line.find(','))));
    }
    return out;
}

void load_imu(Session& s, const fs::path& dir) {
    const auto rows = read_csv(dir / "data.csv", 7);
    const auto stamps = read_timestamps(dir / "data.csv");
    s.imu.resize(rows.size());
    for (size_t i = 0; i < rows.size(); ++i) {
        const auto& r = rows[i];
        s.imu[i].t = stamps[i];
        s.imu[i].gyro = Eigen::Vector3d(r[1], r[2], r[3]);
        s.imu[i].accel = Eigen::Vector3d(r[4], r[5], r[6]);
    }
    const YAML::Node y = load_yaml(dir / "sensor.yaml");
    s.imu_noise.gyro_noise_density = y["gyroscope_noise_density"].as<double>();
    s.imu_noise.gyro_random_walk = y["gyroscope_random_walk"].as<double>();
    s.imu_noise.accel_noise_density = y["accelerometer_noise_density"].as<double>();
    s.imu_noise.accel_random_walk = y["accelerometer_random_walk"].as<double>();
    s.imu_noise.rate_hz = y["rate_hz"].as<double>();
    s.imu_fc_filtered = y["fc_filtered"] && y["fc_filtered"].as<bool>();
}

// Feature observations: timestamp, feature id, u, v. Rows of one frame are consecutive. Written by
// the synthetic generator and by vio_track. A row with feature id -1 only marks a frame without
// features (the camera still delivered it).
std::vector<CameraFrame> read_features(const fs::path& file) {
    const auto rows = read_csv(file, 4);
    const auto stamps = read_timestamps(file);
    std::vector<CameraFrame> frames;
    for (size_t i = 0; i < rows.size(); ++i) {
        if (frames.empty() || frames.back().t != stamps[i]) {
            frames.emplace_back();
            frames.back().t = stamps[i];
        }
        if (rows[i][1] >= 0)
            frames.back().features.push_back(
                {static_cast<int64_t>(rows[i][1]), Eigen::Vector2d(rows[i][2], rows[i][3])});
    }
    return frames;
}

void load_camera(Session& s, const fs::path& dir, const std::string& features_file) {
    const YAML::Node y = load_yaml(dir / "sensor.yaml");
    CameraInfo& c = s.camera;
    const auto intr = y["intrinsics"].as<std::vector<double>>();
    if (intr.size() != 4) throw std::runtime_error("cam0 intrinsics must have 4 values");
    c.fx = intr[0];
    c.fy = intr[1];
    c.cx = intr[2];
    c.cy = intr[3];
    c.model = y["distortion_model"].as<std::string>();
    c.distortion = y["distortion_coefficients"].as<std::vector<double>>();
    const auto res = y["resolution"].as<std::vector<int>>();
    c.width = res.at(0);
    c.height = res.at(1);
    c.rate_hz = y["rate_hz"].as<double>();
    const auto T = y["T_BS"]["data"].as<std::vector<double>>();
    if (T.size() != 16) throw std::runtime_error("cam0 T_BS must have 16 values");
    for (int r = 0; r < 4; ++r)
        for (int col = 0; col < 4; ++col) c.T_body_cam(r, col) = T[r * 4 + col];

    if (!features_file.empty() || fs::exists(dir / "features.csv")) {
        s.frames = read_features(features_file.empty() ? dir / "features.csv" : fs::path(features_file));
    } else {
        // EuRoC: timestamp, file name.
        std::ifstream in(dir / "data.csv");
        if (!in) throw std::runtime_error("cam0 has neither features.csv nor data.csv");
        std::string line;
        while (std::getline(in, line)) {
            if (line.empty() || line[0] == '#') continue;
            if (!line.empty() && line.back() == '\r') line.pop_back();
            const auto comma = line.find(',');
            CameraFrame f;
            f.t = std::stoll(line.substr(0, comma));
            f.image_path = (dir / "data" / line.substr(comma + 1)).string();
            s.frames.push_back(std::move(f));
        }
    }
}

void load_ground_truth(Session& s, const fs::path& file) {
    if (!fs::exists(file)) return;
    const auto rows = read_csv(file, 8);
    const auto stamps = read_timestamps(file);
    s.ground_truth.resize(rows.size());
    for (size_t i = 0; i < rows.size(); ++i) {
        const auto& r = rows[i];
        State& g = s.ground_truth[i];
        g.t = stamps[i];
        g.p = Eigen::Vector3d(r[1], r[2], r[3]);
        g.q = Eigen::Quaterniond(r[4], r[5], r[6], r[7]).normalized();
        if (r.size() >= 17) {
            g.v = Eigen::Vector3d(r[8], r[9], r[10]);
            g.bg = Eigen::Vector3d(r[11], r[12], r[13]);
            g.ba = Eigen::Vector3d(r[14], r[15], r[16]);
        }
    }
}

}  // namespace

Session load_session(const std::string& path, const std::string& features_file) {
    Session s;
    s.path = path;
    const fs::path root(path);
    if (!fs::exists(root / "mav0")) throw std::runtime_error(path + " has no mav0 folder");

    load_imu(s, root / "mav0" / "imu0");
    load_camera(s, root / "mav0" / "cam0", features_file);
    load_ground_truth(s, root / "mav0" / "state_groundtruth_estimate0" / "data.csv");
    const fs::path landed = root / "mav0" / "landed0" / "data.csv";
    if (fs::exists(landed)) {
        const auto rows = read_csv(landed, 2);
        const auto stamps = read_timestamps(landed);
        for (size_t i = 0; i < rows.size(); ++i) s.landed.emplace_back(stamps[i], static_cast<int>(rows[i][1]));
    }
    const fs::path baro = root / "mav0" / "baro0" / "data.csv";
    if (fs::exists(baro)) {
        const auto rows = read_csv(baro, 2);
        const auto stamps = read_timestamps(baro);
        for (size_t i = 0; i < rows.size(); ++i) s.baro.emplace_back(stamps[i], rows[i][1]);
    }

    if (fs::exists(root / "session.yaml")) {
        const YAML::Node y = load_yaml(root / "session.yaml");
        if (y["gravity_world"]) {
            const auto g = y["gravity_world"].as<std::vector<double>>();
            s.gravity_world = Eigen::Vector3d(g.at(0), g.at(1), g.at(2));
        }
    }

    auto by_time = [](const auto& a, const auto& b) { return a.t < b.t; };
    if (!std::is_sorted(s.imu.begin(), s.imu.end(), by_time) ||
        !std::is_sorted(s.frames.begin(), s.frames.end(), by_time)) {
        throw std::runtime_error(path + ": time stamps are not sorted");
    }
    if (s.imu.empty() || s.frames.empty()) throw std::runtime_error(path + ": no IMU or camera data");
    return s;
}

void replay(const Session& session,
            const std::function<void(const ImuSample&)>& on_imu,
            const std::function<void(const CameraFrame&)>& on_frame) {
    size_t i = 0, k = 0;
    while (i < session.imu.size() || k < session.frames.size()) {
        const bool imu_next = k == session.frames.size() ||
                              (i < session.imu.size() && session.imu[i].t <= session.frames[k].t);
        if (imu_next) {
            on_imu(session.imu[i++]);
        } else {
            on_frame(session.frames[k++]);
        }
    }
}

void replay_in_imu_time(const Session& session, const std::function<Timestamp()>& td,
                        const std::function<void(const ImuSample&)>& on_imu,
                        const std::function<void(const CameraFrame&, const ImuSample* next)>& on_frame) {
    const auto& imu = session.imu;
    const auto& frames = session.frames;
    size_t i = 0, k = 0;
    while (i < imu.size() || k < frames.size()) {
        const bool frame_next = k < frames.size() && (i == imu.size() || imu[i].t > frames[k].t + td());
        if (frame_next) {
            on_frame(frames[k++], i < imu.size() ? &imu[i] : nullptr);
        } else {
            on_imu(imu[i++]);
        }
    }
}

bool interpolate_ground_truth(const std::vector<State>& gt, Timestamp t, State& out) {
    if (gt.empty() || t < gt.front().t || t > gt.back().t) return false;
    auto it = std::lower_bound(gt.begin(), gt.end(), t,
                               [](const State& s, Timestamp tt) { return s.t < tt; });
    if (it->t == t) {
        out = *it;
        return true;
    }
    const State& b = *it;
    const State& a = *(it - 1);
    const double w = static_cast<double>(t - a.t) / static_cast<double>(b.t - a.t);
    out.t = t;
    out.p = (1 - w) * a.p + w * b.p;
    out.v = (1 - w) * a.v + w * b.v;
    out.q = a.q.slerp(w, b.q);
    out.bg = (1 - w) * a.bg + w * b.bg;
    out.ba = (1 - w) * a.ba + w * b.ba;
    return true;
}

}  // namespace vio
