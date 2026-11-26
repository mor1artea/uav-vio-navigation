#!/usr/bin/env python3
"""Trajectory metrics as in the project spec.

  ATE RMSE and max, in m and in % of the distance travelled, two variants:
    navigation  only the start point is matched (what the drone really experiences, includes the
                heading error), used for the grades
    aligned     best-fit rotation and translation (no scale), shows the pure drift
  RPE on segments of 100 m (translation error / segment length, %)
  Loop closure error (closed routes): end-to-start error / distance travelled, %
  Hover drift (optional time window): change of the position error per minute

Estimate:     CSV "timestamp_ns,px,py,pz,qw,qx,qy,qz" (vio_replay output)
Ground truth: a session folder (mav0/state_groundtruth_estimate0/data.csv) or a CSV with the same
              first 8 columns. Ground truth is interpolated at the estimate time stamps.

Example:
  tools/metrics.py trajectory.csv ~/datasets/synthetic/fig8
"""

import argparse
import json
import os
import sys

import numpy as np

# Spec thresholds: (excellent, satisfactory). Smaller is better.
THRESHOLDS = {
    "loop_closure_pct": (1.0, 3.0),
    "ate_rmse_pct": (1.0, 5.0),
    "ate_max_pct": (2.0, 8.0),
    "rpe_mean_pct": (1.5, 5.0),
    "hover_drift_m_per_min": (0.5, 2.0),
}


# ---------------------------------------------------------------- loading

def load_trajectory(path):
    """Returns t (int64 ns), p (N, 3), q (N, 4, w first)."""
    if os.path.isdir(path):
        path = os.path.join(path, "mav0", "state_groundtruth_estimate0", "data.csv")
    t, rows = [], []
    with open(path) as f:
        for line in f:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            cells = line.split(",")
            t.append(int(cells[0]))  # exact: EuRoC time stamps do not fit a double
            rows.append([float(c) for c in cells[1:8]])
    if not rows:
        sys.exit(f"error: no data in {path}")
    rows = np.array(rows)
    q = rows[:, 3:7]
    return np.array(t, dtype=np.int64), rows[:, 0:3], q / np.linalg.norm(q, axis=1)[:, None]


def quat_to_rot(q):
    w, x, y, z = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], axis=1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], axis=1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], axis=1),
    ], axis=1)


def slerp(q0, q1, w):
    dot = np.sum(q0 * q1, axis=1)
    q1 = np.where(dot[:, None] < 0, -q1, q1)
    dot = np.abs(dot)
    angle = np.arccos(np.clip(dot, -1.0, 1.0))
    small = angle < 1e-8
    s = np.sin(np.where(small, 1.0, angle))
    a = np.where(small, 1 - w, np.sin((1 - w) * angle) / s)
    b = np.where(small, w, np.sin(w * angle) / s)
    q = a[:, None] * q0 + b[:, None] * q1
    return q / np.linalg.norm(q, axis=1)[:, None]


def associate(est, gt):
    """Ground truth interpolated at the estimate time stamps. Estimate poses outside the ground
    truth time range are dropped. Returns (t, p_est, R_est, p_gt, R_gt)."""
    t_e, p_e, q_e = est
    t_g, p_g, q_g = gt
    keep = (t_e >= t_g[0]) & (t_e <= t_g[-1])
    t_e, p_e, q_e = t_e[keep], p_e[keep], q_e[keep]
    if len(t_e) < 2:
        sys.exit("error: estimate and ground truth do not overlap in time")
    i = np.clip(np.searchsorted(t_g, t_e), 1, len(t_g) - 1)
    span = (t_g[i] - t_g[i - 1]).astype(float)
    w = np.where(span > 0, (t_e - t_g[i - 1]) / np.where(span > 0, span, 1), 0.0)
    p_i = (1 - w)[:, None] * p_g[i - 1] + w[:, None] * p_g[i]
    q_i = slerp(q_g[i - 1], q_g[i], w)
    return t_e, p_e, quat_to_rot(q_e), p_i, quat_to_rot(q_i)


# ---------------------------------------------------------------- metrics

def path_length(p):
    return float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))


def ate_navigation(p_est, p_gt):
    """Position errors after matching only the start point."""
    return np.linalg.norm((p_est - p_est[0]) - (p_gt - p_gt[0]), axis=1)


def umeyama_rt(src, dst):
    """Rotation R and translation t minimizing |R src + t - dst| (no scale)."""
    mu_s, mu_d = src.mean(axis=0), dst.mean(axis=0)
    cov = (dst - mu_d).T @ (src - mu_s) / len(src)
    u, _, vt = np.linalg.svd(cov)
    s = np.eye(3)
    if np.linalg.det(u) * np.linalg.det(vt) < 0:
        s[2, 2] = -1
    r = u @ s @ vt
    return r, mu_d - r @ mu_s


def ate_aligned(p_est, p_gt):
    r, t = umeyama_rt(p_est, p_gt)
    return np.linalg.norm(p_est @ r.T + t - p_gt, axis=1)


def rpe(p_est, R_est, p_gt, R_gt, length):
    """Relative translation error (%) for every segment of `length` metres of ground-truth path,
    each relative pose expressed in its own start frame."""
    d = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(p_gt, axis=0), axis=1))])
    j = np.searchsorted(d, d + length)
    i = np.nonzero(j < len(d))[0]
    if len(i) == 0:
        return np.array([])
    j = j[i]
    dp_gt = np.einsum("nji,nj->ni", R_gt[i], p_gt[j] - p_gt[i])
    dp_est = np.einsum("nji,nj->ni", R_est[i], p_est[j] - p_est[i])
    return 100.0 * np.linalg.norm(dp_est - dp_gt, axis=1) / (d[j] - d[i])


def loop_closure(p_est, p_gt):
    """End-to-start error relative to the distance travelled, %."""
    err = np.linalg.norm((p_est[-1] - p_est[0]) - (p_gt[-1] - p_gt[0]))
    return 100.0 * err / path_length(p_gt)


def hover_drift(t, p_est, p_gt, start_s, end_s):
    """Change of the position error over a time window, m/min, and its maximum within the window."""
    ts = (t - t[0]) * 1e-9
    w = (ts >= start_s) & (ts <= end_s)
    if w.sum() < 2:
        sys.exit("error: hover window has no data")
    e = (p_est - p_gt)[w]
    de = np.linalg.norm(e - e[0], axis=1)
    minutes = (ts[w][-1] - ts[w][0]) / 60.0
    return float(de[-1] / minutes), float(de.max())


def grade(name, value):
    if value is None or name not in THRESHOLDS:
        return ""
    excellent, satisfactory = THRESHOLDS[name]
    return "excellent" if value < excellent else "satisfactory" if value < satisfactory else "FAIL"


def evaluate(est, gt, rpe_length=100.0, hover=None):
    t, p_e, R_e, p_g, R_g = associate(est, gt)
    length = path_length(p_g)
    nav = ate_navigation(p_e, p_g)
    ali = ate_aligned(p_e, p_g)
    r = rpe(p_e, R_e, p_g, R_g, rpe_length)
    # A route counts as closed if it ends near its start and is long enough to mean something.
    closed = length >= 100.0 and np.linalg.norm(p_g[-1] - p_g[0]) < max(2.0, 0.01 * length)
    res = {
        "poses": int(len(t)),
        "duration_s": float((t[-1] - t[0]) * 1e-9),
        "distance_m": length,
        "ate_rmse_m": float(np.sqrt(np.mean(nav ** 2))),
        "ate_max_m": float(nav.max()),
        "ate_aligned_rmse_m": float(np.sqrt(np.mean(ali ** 2))),
        "ate_aligned_max_m": float(ali.max()),
        "rpe_length_m": rpe_length,
        "rpe_segments": int(len(r)),
        "rpe_mean_pct": float(r.mean()) if len(r) else None,
        "rpe_rmse_pct": float(np.sqrt(np.mean(r ** 2))) if len(r) else None,
        "rpe_max_pct": float(r.max()) if len(r) else None,
        "loop_closure_pct": float(loop_closure(p_e, p_g)) if closed else None,
    }
    if length > 0:
        res["ate_rmse_pct"] = 100.0 * res["ate_rmse_m"] / length
        res["ate_max_pct"] = 100.0 * res["ate_max_m"] / length
    if hover is not None:
        res["hover_drift_m_per_min"], res["hover_max_dev_m"] = hover_drift(t, p_e, p_g, *hover)
    return res


def print_report(res):
    def line(label, value, unit, key=None):
        txt = "n/a" if value is None else f"{value:.3f} {unit}"
        print(f"  {label:<26} {txt:<18} {grade(key, value) if key else ''}")

    print(f"  poses {res['poses']}, {res['duration_s']:.1f} s, distance {res['distance_m']:.1f} m")
    line("ATE RMSE (navigation)", res["ate_rmse_m"], "m")
    line("ATE RMSE (navigation)", res.get("ate_rmse_pct"), "%", "ate_rmse_pct")
    line("ATE max (navigation)", res.get("ate_max_pct"), "%", "ate_max_pct")
    line("ATE RMSE (aligned)", res["ate_aligned_rmse_m"], "m")
    line("ATE max (aligned)", res["ate_aligned_max_m"], "m")
    print(f"  RPE on {res['rpe_length_m']:.0f} m segments ({res['rpe_segments']} segments)")
    line("  mean", res["rpe_mean_pct"], "%", "rpe_mean_pct")
    line("  RMSE", res["rpe_rmse_pct"], "%")
    line("  max", res["rpe_max_pct"], "%")
    line("Loop closure", res["loop_closure_pct"], "%", "loop_closure_pct")
    if "hover_drift_m_per_min" in res:
        line("Hover drift", res["hover_drift_m_per_min"], "m/min", "hover_drift_m_per_min")
        line("Hover max deviation", res["hover_max_dev_m"], "m")


def main():
    ap = argparse.ArgumentParser(description="ATE / RPE / loop closure / hover drift.")
    ap.add_argument("estimate", help="estimated trajectory CSV")
    ap.add_argument("ground_truth", help="session folder or ground truth CSV")
    ap.add_argument("--rpe-length", type=float, default=100.0, help="RPE segment length, m")
    ap.add_argument("--hover", type=float, nargs=2, metavar=("START", "END"),
                    help="hover window in seconds from the start of the estimate")
    ap.add_argument("--json", help="also write the results to this JSON file")
    args = ap.parse_args()

    res = evaluate(load_trajectory(args.estimate), load_trajectory(args.ground_truth),
                   args.rpe_length, args.hover)
    print_report(res)
    if args.json:
        with open(args.json, "w") as f:
            json.dump(res, f, indent=2)


if __name__ == "__main__":
    main()
