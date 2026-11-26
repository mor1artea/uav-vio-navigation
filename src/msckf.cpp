#include "vio/msckf.h"

#include "vio/frontend.h"

#include <Eigen/QR>

#include <algorithm>
#include <cmath>
#include <set>
#include <unordered_map>

namespace vio {
namespace {

Eigen::Matrix3d skew(const Eigen::Vector3d& v) {
    Eigen::Matrix3d m;
    m << 0, -v.z(), v.y(), v.z(), 0, -v.x(), -v.y(), v.x(), 0;
    return m;
}

}  // namespace

bool project_point(const Camera& cam, const Eigen::Matrix3d& R_bc, const Eigen::Vector3d& t_bc,
                   const Eigen::Matrix3d& R_wb, const Eigen::Vector3d& p_wb,
                   const Eigen::Vector3d& p_f, Eigen::Vector2d& uv,
                   Eigen::Matrix<double, 2, 6>* H_pose, Eigen::Matrix<double, 2, 3>* H_point) {
    // p_c = R_bc^T (R_wb^T (p_f - p_wb) - t_bc). With invariant pose errors
    // (R_true = Exp(dtheta) R_wb, p_true = Exp(dtheta) p_wb + dp):
    //   d p_c / d dtheta = R_cw [p_f]x,  d p_c / d dp = -R_cw,  d p_c / d p_f = R_cw
    const Eigen::Matrix3d R_cw = R_bc.transpose() * R_wb.transpose();
    const Eigen::Vector3d d = p_f - p_wb;
    const Eigen::Vector3d p_c = R_bc.transpose() * (R_wb.transpose() * d - t_bc);
    Eigen::Matrix<double, 2, 3> J;
    if (!cam.project(p_c, uv, &J)) return false;
    if (H_pose) {
        H_pose->leftCols<3>() = J * R_cw * skew(p_f);
        H_pose->rightCols<3>() = -J * R_cw;
    }
    if (H_point) *H_point = J * R_cw;
    return true;
}

bool triangulate(const Camera& cam, const std::vector<CameraObs>& obs, double min_parallax_rad,
                 Eigen::Vector3d& p_f) {
    const size_t m = obs.size();
    std::vector<Eigen::Vector3d> dir(m);
    for (size_t i = 0; i < m; ++i) {
        Eigen::Vector3d b;
        if (!cam.unproject(obs[i].uv, b)) return false;
        dir[i] = obs[i].R_wc * b;
    }
    double max_angle = 0;
    for (size_t i = 0; i < m; ++i)
        for (size_t j = i + 1; j < m; ++j)
            max_angle = std::max(max_angle, std::acos(std::min(1.0, dir[i].dot(dir[j]))));
    if (max_angle < min_parallax_rad) return false;

    // Closest point to all rays, weighted by 1 / distance^2 so it approximates angular errors.
    std::vector<double> w(m, 1.0);
    for (int iter = 0; iter < 3; ++iter) {
        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d rhs = Eigen::Vector3d::Zero();
        for (size_t i = 0; i < m; ++i) {
            const Eigen::Matrix3d Pi = Eigen::Matrix3d::Identity() - dir[i] * dir[i].transpose();
            A += w[i] * Pi;
            rhs += w[i] * Pi * obs[i].p_wc;
        }
        p_f = A.ldlt().solve(rhs);
        for (size_t i = 0; i < m; ++i) w[i] = 1.0 / std::max(1e-6, (p_f - obs[i].p_wc).squaredNorm());
    }

    // Gauss-Newton on the pixel errors.
    for (int iter = 0; iter < 5; ++iter) {
        Eigen::Matrix3d A = Eigen::Matrix3d::Zero();
        Eigen::Vector3d b = Eigen::Vector3d::Zero();
        for (const auto& o : obs) {
            const Eigen::Vector3d p_c = o.R_wc.transpose() * (p_f - o.p_wc);
            Eigen::Vector2d uv;
            Eigen::Matrix<double, 2, 3> J;
            if (p_c.z() <= 0 || !cam.project(p_c, uv, &J)) return false;
            const Eigen::Matrix<double, 2, 3> Jw = J * o.R_wc.transpose();
            A += Jw.transpose() * Jw;
            b += Jw.transpose() * (o.uv - uv);
        }
        const Eigen::Vector3d step = A.ldlt().solve(b);
        p_f += step;
        if (step.norm() < 1e-6 * (1 + p_f.norm())) break;
    }
    for (const auto& o : obs)
        if ((o.R_wc.transpose() * (p_f - o.p_wc)).z() <= 0.1) return false;
    return true;
}

Eigen::MatrixXd track_noise(const std::vector<double>& s_deg, double sigma_w, double q) {
    const int m = static_cast<int>(s_deg.size());
    Eigen::MatrixXd R = Eigen::MatrixXd::Zero(2 * m, 2 * m);
    for (int i = 0; i < m; ++i)
        for (int j = 0; j < m; ++j)
            R(2 * i, 2 * j) = R(2 * i + 1, 2 * j + 1) = q * std::min(s_deg[i], s_deg[j]);
    R.diagonal().array() += sigma_w * sigma_w;
    return R;
}

std::vector<double> accumulated_view_change(const std::vector<Eigen::Vector3d>& cameras,
                                            const Eigen::Vector3d& p_f) {
    std::vector<double> s(cameras.size(), 0.0);
    for (size_t i = 1; i < cameras.size(); ++i) {
        const Eigen::Vector3d a = (p_f - cameras[i - 1]).normalized();
        const Eigen::Vector3d b = (p_f - cameras[i]).normalized();
        s[i] = s[i - 1] + std::acos(std::min(1.0, a.dot(b))) * 180 / M_PI;
    }
    return s;
}

double median_feature_motion(const CameraFrame& prev, const CameraFrame& cur) {
    std::unordered_map<int64_t, Eigen::Vector2d> before;
    for (const auto& f : prev.features) before[f.id] = f.uv;
    std::vector<double> d;
    for (const auto& f : cur.features) {
        const auto it = before.find(f.id);
        if (it != before.end()) d.push_back((f.uv - it->second).norm());
    }
    if (d.empty()) return -1;
    std::nth_element(d.begin(), d.begin() + d.size() / 2, d.end());
    return d[d.size() / 2];
}

Msckf::Msckf(const Camera& cam, const Eigen::Matrix4d& T_body_cam, const MsckfConfig& cfg)
    : cam_(cam), R_bc_(T_body_cam.block<3, 3>(0, 0)), t_bc_(T_body_cam.block<3, 1>(0, 3)),
      cfg_(cfg) {}

bool Msckf::linearize(const Filter& filter, int n, const Track& track, double min_parallax_deg,
                      Eigen::MatrixXd& Hx, Eigen::MatrixXd& Hf, Eigen::VectorXd& r,
                      Eigen::Vector3d& p_f, const ClonePoses* lin) const {
    const std::vector<Clone>& clones = filter.clones();
    std::vector<CameraObs> obs;
    std::vector<size_t> ks;
    for (const auto& [key, uv] : track) {
        for (size_t k = 0; k < clones.size(); ++k) {
            if (clones[k].key != key) continue;
            const Eigen::Matrix3d R_wb = clones[k].q.toRotationMatrix();
            obs.push_back({R_wb * R_bc_, clones[k].p + R_wb * t_bc_, uv});
            ks.push_back(k);
        }
    }
    if (static_cast<int>(obs.size()) < cfg_.min_observations) return false;
    if (!triangulate(cam_, obs, min_parallax_deg * M_PI / 180, p_f)) return false;

    // Drift: view change along the track; later observations whose modelled sigma would exceed
    // drift_sigma_max are left out (the point is then triangulated again without them).
    std::vector<double> s;
    if (cfg_.track_drift) {
        auto view_change = [&] {
            std::vector<Eigen::Vector3d> cams;
            for (const auto& o : obs) cams.push_back(o.p_wc);
            return accumulated_view_change(cams, p_f);
        };
        s = view_change();
        const double var_max = cfg_.drift_sigma_max * cfg_.drift_sigma_max - cfg_.pixel_noise * cfg_.pixel_noise;
        size_t keep = 0;
        while (keep < s.size() && cfg_.drift_q * s[keep] <= var_max) ++keep;
        if (keep < obs.size()) {
            if (static_cast<int>(keep) < cfg_.min_observations) return false;
            obs.resize(keep);
            ks.resize(keep);
            if (!triangulate(cam_, obs, min_parallax_deg * M_PI / 180, p_f)) return false;
            s = view_change();
        }
    }

    // Midpoint linearization: Jacobians at the given clone poses and the point triangulated there.
    // If that fails, at the estimate as usual.
    const int m = static_cast<int>(obs.size());
    std::vector<Eigen::Matrix3d> R_lin;
    std::vector<Eigen::Vector3d> p_lin;
    std::vector<CameraObs> lin_obs;
    Eigen::Vector3d pf_lin;
    bool use_lin = false;
    if (lin) {
        for (int j = 0; j < m; ++j) {
            const auto it = lin->find(clones[ks[j]].key);
            if (it == lin->end()) break;
            R_lin.push_back(it->second.q.toRotationMatrix());
            p_lin.push_back(it->second.p);
            lin_obs.push_back({R_lin.back() * R_bc_, p_lin.back() + R_lin.back() * t_bc_, obs[j].uv});
        }
        use_lin = static_cast<int>(lin_obs.size()) == m && triangulate(cam_, lin_obs, 0.0, pf_lin);
    }

    // Pixel residuals and Jacobians for every observation of the track (residuals always at the
    // estimate).
    Hx = Eigen::MatrixXd::Zero(2 * m, n);
    Hf.resize(2 * m, 3);
    r.resize(2 * m);
    for (int j = 0; j < m; ++j) {
        const Clone& c = clones[ks[j]];
        Eigen::Vector2d uv_hat;
        Eigen::Matrix<double, 2, 6> Hp;
        Eigen::Matrix<double, 2, 3> Hpt;
        if (!project_point(cam_, R_bc_, t_bc_, c.q.toRotationMatrix(), c.p, p_f, uv_hat, &Hp, &Hpt))
            return false;
        if (use_lin) {
            Eigen::Vector2d uv_lin;
            if (!project_point(cam_, R_bc_, t_bc_, R_lin[j], p_lin[j], pf_lin, uv_lin, &Hp, &Hpt))
                return false;
        }
        r.segment<2>(2 * j) = obs[j].uv - uv_hat;
        Hx.block<2, 6>(2 * j, filter.clone_index(ks[j])) = Hp;
        Hf.block<2, 3>(2 * j, 0) = Hpt;
    }
    if (cfg_.track_drift) {  // whiten: L^-1 (noise) L^-T = I
        const Eigen::LLT<Eigen::MatrixXd> llt(track_noise(s, cfg_.pixel_noise, cfg_.drift_q));
        const auto L = llt.matrixL();
        L.solveInPlace(r);
        L.solveInPlace(Hx);
        L.solveInPlace(Hf);
    }
    return true;
}

bool Msckf::is_landmark(const Filter& filter, int64_t id) const {
    for (const auto& l : filter.landmarks())
        if (l.id == id) return true;
    return false;
}

std::set<int64_t> Msckf::landmark_ids(const Filter& filter) const {
    std::set<int64_t> ids;
    for (const auto& l : filter.landmarks()) ids.insert(l.id);
    return ids;
}

void Msckf::update_landmarks(Filter& filter, Timestamp t,
                             const std::map<int64_t, Eigen::Vector2d>& obs, MsckfStats& st) {
    // Each landmark seen in this frame gives one pixel residual against the newest clone.
    const std::vector<Landmark>& L = filter.landmarks();
    const size_t k = filter.clones().size() - 1;
    const Clone& c = filter.clones()[k];
    const int n = static_cast<int>(filter.covariance().rows());
    const Eigen::MatrixXd& P = filter.covariance();
    const double var = cfg_.pixel_noise * cfg_.pixel_noise;  // landmark pixels: white noise only
    std::vector<size_t> drop;
    Eigen::MatrixXd H(2 * L.size(), n);
    Eigen::VectorXd r(2 * L.size());
    int rows = 0;
    for (size_t j = 0; j < L.size(); ++j) {
        const auto it = obs.find(L[j].id);
        if (it == obs.end()) {  // lost by the tracker
            drop.push_back(j);
            continue;
        }
        // Drift guard: KLT drifts when the view of a point changes, so a landmark is retired once
        // its viewing direction has turned more than landmark_max_view_change_deg since birth (in
        // hover that takes long, in flight it is quick), or after landmark_max_age_s at the latest.
        const Eigen::Vector3d cam_now = c.p + c.q * t_bc_;
        const Eigen::Vector3d ray0 = (L[j].p - L[j].camera_at_birth).normalized();
        const Eigen::Vector3d ray1 = (L[j].p - cam_now).normalized();
        const double view_change = std::acos(std::min(1.0, ray0.dot(ray1)));
        if (view_change > cfg_.landmark_max_view_change_deg * M_PI / 180 ||
            to_sec(t - L[j].born) > cfg_.landmark_max_age_s) {
            drop.push_back(j);
            used_.insert(L[j].id);
            continue;
        }
        Eigen::Vector2d uv_hat;
        Eigen::Matrix<double, 2, 6> Hp;
        Eigen::Matrix<double, 2, 3> Hf;
        if (!project_point(cam_, R_bc_, t_bc_, c.q.toRotationMatrix(), c.p, L[j].p, uv_hat, &Hp, &Hf)) {
            drop.push_back(j);
            continue;
        }
        if (cfg_.track_drift) {  // the feature drifted with the view change since the last update
            Eigen::Vector3d& ray_last = landmark_ray_[L[j].id];
            const double dv = std::acos(std::min(1.0, ray_last.dot(ray1))) * 180 / M_PI;
            ray_last = ray1;
            const Eigen::Matrix<double, 3, 2> J_pinv = Hf.transpose() * (Hf * Hf.transpose()).inverse();
            filter.add_landmark_noise(j, cfg_.drift_q * dv * J_pinv * J_pinv.transpose());
        }
        // p_f_true = Exp(dtheta) p_f + dp_f: the landmark also depends on the IMU attitude error.
        Eigen::MatrixXd Hj = Eigen::MatrixXd::Zero(2, n);
        Hj.middleCols<6>(filter.clone_index(k)) = Hp;
        Hj.middleCols<3>(Filter::landmark_index(j)) = Hf;
        Hj.middleCols<3>(Filter::TH) = -Hf * skew(L[j].p);
        const Eigen::Vector2d rj = it->second - uv_hat;
        const Eigen::Matrix2d S = Hj * P * Hj.transpose() + var * Eigen::Matrix2d::Identity();
        if (rj.dot(S.ldlt().solve(rj)) > chi2_quantile(cfg_.chi2_probability, 2)) {
            ++st.landmark_rejected;
            if (++chi2_fails_[L[j].id] >= 2) {  // drift guard: two failures in a row
                drop.push_back(j);
                used_.insert(L[j].id);
            }
            continue;
        }
        chi2_fails_[L[j].id] = 0;
        H.middleRows(rows, 2) = Hj;
        r.segment<2>(rows) = rj;
        rows += 2;
    }
    if (rows > 0)
        filter.update(H.topRows(rows), r.head(rows), var * Eigen::MatrixXd::Identity(rows, rows), false);
    for (auto j = drop.rbegin(); j != drop.rend(); ++j) {
        chi2_fails_.erase(L[*j].id);
        landmark_ray_.erase(L[*j].id);
        filter.remove_landmark(*j);
    }
}

bool Msckf::init_landmark(Filter& filter, int64_t id, Timestamp t, const Track& track,
                          MsckfStats& st) {
    const int n = static_cast<int>(filter.covariance().rows());
    Eigen::MatrixXd Hx, Hf;
    Eigen::VectorXd r;
    Eigen::Vector3d p_f;
    // Landmarks need more parallax than MSCKF features: their depth stays in the state, and a
    // point triangulated from 1-2 deg has a depth too uncertain for a linear (Gaussian) model.
    if (!linearize(filter, n, track, cfg_.landmark_min_parallax_deg, Hx, Hf, r, p_f)) return false;
    // The new landmark's error is invariant (p_f_true = Exp(dtheta) p_f + dp_f), so the residual
    // also depends on the IMU attitude error through it.
    Hx.middleCols<3>(Filter::TH) += -Hf * skew(p_f);

    // Split with the QR of Hf: the first 3 rows determine the landmark, the others are a normal
    // MSCKF measurement of the clones.
    const int m2 = static_cast<int>(r.size());
    const Eigen::HouseholderQR<Eigen::MatrixXd> qr(Hf);
    const Eigen::MatrixXd QtH = qr.householderQ().adjoint() * Hx;
    const Eigen::VectorXd Qtr = qr.householderQ().adjoint() * r;
    const Eigen::Matrix3d R1 = qr.matrixQR().topRows<3>().triangularView<Eigen::Upper>();
    const double var = cfg_.track_drift ? 1.0 : cfg_.pixel_noise * cfg_.pixel_noise;  // whitened: 1
    const Eigen::MatrixXd& P = filter.covariance();
    const Eigen::MatrixXd Ho = QtH.bottomRows(m2 - 3);
    const Eigen::VectorXd ro = Qtr.tail(m2 - 3);
    const Eigen::MatrixXd S = Ho * P * Ho.transpose() + var * Eigen::MatrixXd::Identity(m2 - 3, m2 - 3);
    if (ro.dot(S.ldlt().solve(ro)) > chi2_quantile(cfg_.chi2_probability, m2 - 3)) {
        ++st.rejected;
        return false;
    }
    const Eigen::Matrix3d R1inv = R1.inverse();
    const Eigen::MatrixXd A = -R1inv * QtH.topRows<3>();
    const Clone& newest = filter.clones().back();
    const Eigen::Vector3d p_new = p_f + R1inv * Qtr.head<3>();
    const Eigen::Vector3d camera = newest.p + newest.q * t_bc_;
    filter.add_landmark({id, p_new, t, camera}, A, var * R1inv * R1inv.transpose());
    if (cfg_.track_drift) landmark_ray_[id] = (p_new - camera).normalized();

    // The rest as an update; the new landmark got 3 columns in the state, at its index.
    const int at = Filter::landmark_index(filter.landmarks().size() - 1);
    Eigen::MatrixXd Ho2 = Eigen::MatrixXd::Zero(Ho.rows(), n + 3);
    Ho2.leftCols(at) = Ho.leftCols(at);
    Ho2.rightCols(n - at) = Ho.rightCols(n - at);
    filter.update(Ho2, ro, var * Eigen::MatrixXd::Identity(m2 - 3, m2 - 3), false);
    chi2_fails_[id] = 0;
    ++st.new_landmarks;
    return true;
}

bool Msckf::stacked_update(Filter& filter, const std::vector<std::pair<int64_t, Track>>& tracks,
                           const ClonePoses* lin, MsckfStats* st, std::set<int64_t>* passed,
                           const std::set<int64_t>* only) const {
    const Eigen::MatrixXd& P = filter.covariance();
    const int n = static_cast<int>(P.rows());
    const double var = cfg_.track_drift ? 1.0 : cfg_.pixel_noise * cfg_.pixel_noise;  // whitened: 1
    std::vector<Eigen::MatrixXd> Hs;
    std::vector<Eigen::VectorXd> rs;
    int rows = 0;
    for (const auto& [id, track] : tracks) {
        if (only && !only->count(id)) continue;
        Eigen::MatrixXd Hx, Hf;
        Eigen::VectorXd r;
        Eigen::Vector3d p_f;
        if (!linearize(filter, n, track, cfg_.min_parallax_deg, Hx, Hf, r, p_f, lin)) {
            if (st && static_cast<int>(track.size()) >= cfg_.min_observations) ++st->no_geometry;
            continue;
        }
        // Remove the point: project onto the left nullspace of Hf (an orthonormal basis, so the
        // pixel noise stays var * I; whitened tracks: I).
        const int m2 = static_cast<int>(r.size());
        const Eigen::HouseholderQR<Eigen::MatrixXd> qr(Hf);
        const Eigen::MatrixXd Ho = (qr.householderQ().adjoint() * Hx).bottomRows(m2 - 3);
        const Eigen::VectorXd ro = (qr.householderQ().adjoint() * r).tail(m2 - 3);
        const Eigen::MatrixXd S = Ho * P * Ho.transpose() + var * Eigen::MatrixXd::Identity(m2 - 3, m2 - 3);
        if (!only && ro.dot(S.ldlt().solve(ro)) > chi2_quantile(cfg_.chi2_probability, m2 - 3)) {
            if (st) ++st->rejected;
            continue;
        }
        if (passed) passed->insert(id);
        Hs.push_back(Ho);
        rs.push_back(ro);
        rows += m2 - 3;
        if (st) ++st->used;
    }

    if (rows > 0) {
        Eigen::MatrixXd H(rows, n);
        Eigen::VectorXd r(rows);
        for (size_t i = 0, row = 0; i < Hs.size(); row += Hs[i].rows(), ++i) {
            H.middleRows(row, Hs[i].rows()) = Hs[i];
            r.segment(row, rs[i].size()) = rs[i];
        }
        if (rows > n) {  // compress: same information in n rows
            const Eigen::HouseholderQR<Eigen::MatrixXd> qr(H);
            r = (qr.householderQ().adjoint() * r).head(n).eval();
            H = qr.matrixQR().topRows(n).triangularView<Eigen::Upper>();
        }
        // Joint gate: each track passed its own test, but tracks that are each plausible can still
        // contradict each other (very near points after a long time on the IMU alone, EuRoC
        // liftoff, DESIGN.md section 11). The whole update is skipped if it fails the test.
        return filter.update(H, r, var * Eigen::MatrixXd::Identity(H.rows(), H.rows()), true);
    }
    return true;
}

MsckfStats Msckf::process(Filter& filter, const CameraFrame& frame, bool still,
                          const Eigen::Vector3d& gyro) {
    MsckfStats st;
    std::set<int64_t> seen;
    if (!still) {
        filter.augment_clone(frame.t, gyro);
        const Eigen::Vector3d w_cam = R_bc_.transpose() * (gyro - filter.state().bg);
        const double ref_row = 0.5 * (cam_.height() - 1);
        std::map<int64_t, Eigen::Vector2d> landmark_obs;
        for (const auto& f : frame.features) {
            Eigen::Vector2d uv = f.uv;
            Eigen::Vector3d b;
            if (cfg_.line_time_s > 0 && cam_.unproject(uv, b))
                cam_.project(rs_correct(b, uv.y(), ref_row, cfg_.line_time_s, w_cam), uv);
            seen.insert(f.id);
            if (is_landmark(filter, f.id))
                landmark_obs[f.id] = uv;
            else if (!used_.count(f.id))
                tracks_[f.id][frame.t] = uv;
        }
        // Features used once and now gone can be forgotten. A used feature older than the age cap
        // counts as a new one (the tracker re-detects and renames it; feature files from the
        // generator never do, so this does it for them).
        for (auto it = used_.begin(); it != used_.end();) {
            const bool expired = cfg_.feature_max_age > 0 && age_[*it] >= cfg_.feature_max_age;
            if (!seen.count(*it) || expired) {
                age_.erase(*it);
                it = used_.erase(it);
            } else {
                ++it;
            }
        }
        for (auto it = age_.begin(); it != age_.end();) it = seen.count(it->first) ? std::next(it) : age_.erase(it);
        for (const int64_t id : seen) ++age_[id];
        update_landmarks(filter, frame.t, landmark_obs, st);
    }

    // Tracks to use now: lost ones, and those that reach back to the oldest clone when it is about
    // to be dropped. While still, nothing is lost: the tracks wait. A track that is used while its
    // feature is still tracked becomes a landmark (slow flight, room left), or its feature is never
    // used again: slow KLT drift would make the next tracks of the same feature share errors.
    const bool full = static_cast<int>(filter.clones().size()) > cfg_.max_clones;
    const Timestamp oldest = filter.clones().empty() ? -1 : filter.clones().front().key;
    const bool slow = filter.state().v.norm() < cfg_.landmark_speed_max;
    std::vector<int64_t> use;
    for (const auto& [id, track] : tracks_) {
        const bool lost = !still && !seen.count(id);
        if (lost || (full && track.count(oldest))) use.push_back(id);
    }

    std::vector<std::pair<int64_t, Track>> for_msckf;
    for (const int64_t id : use) {
        Track track = std::move(tracks_[id]);
        tracks_.erase(id);
        if (seen.count(id)) {
            if (slow && static_cast<int>(filter.landmarks().size()) < cfg_.max_landmarks &&
                init_landmark(filter, id, frame.t, track, st))
                continue;
            used_.insert(id);
        }
        for_msckf.emplace_back(id, std::move(track));
    }

    if (cfg_.midpoint_linearization && !for_msckf.empty()) {
        // Once to get the updated clone poses, then again from the prior with the Jacobians halfway,
        // with the tracks that passed their chi2 test in the first pass (not tested again: which
        // tracks are outliers is decided once, against the prior).
        const Filter prior = filter;
        MsckfStats first;
        std::set<int64_t> passed;
        if (stacked_update(filter, for_msckf, nullptr, &first, &passed)) {
            ClonePoses mid;
            for (size_t k = 0; k < filter.clones().size(); ++k) {
                const Clone &a = prior.clones()[k], &b = filter.clones()[k];
                mid[a.key] = {a.q.slerp(0.5, b.q), 0.5 * (a.p + b.p)};
            }
            filter.restore(prior);
            st.update_rejected = !stacked_update(filter, for_msckf, &mid, &st, nullptr, &passed);
            st.rejected = first.rejected;
            st.no_geometry = first.no_geometry;
        } else {  // rejected as a whole: the state did not change, keep the counts of this pass
            st.used = first.used;
            st.rejected = first.rejected;
            st.no_geometry = first.no_geometry;
            st.update_rejected = true;
        }
    } else {
        st.update_rejected = !stacked_update(filter, for_msckf, nullptr, &st);
    }

    if (static_cast<int>(filter.clones().size()) > cfg_.max_clones) {
        filter.remove_clone(0);
        for (auto& [id, track] : tracks_) track.erase(oldest);
    }
    st.landmarks = static_cast<int>(filter.landmarks().size());
    return st;
}

}  // namespace vio
