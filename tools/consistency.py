#!/usr/bin/env python3
"""Is the filter's covariance honest? Compares vio_replay --mode vio output with the ground truth.

For every output row: the position error and the attitude error (world frame, the filter's own
error definition: R_true = Exp(dtheta) R_est) against the reported covariance.
  NEES            normalized estimation error squared, e^T C^-1 e. For an honest filter the average
                  is the dimension: 3 for position, 3 for attitude, 6 for the pose. Much larger
                  means overconfident (the dangerous case), much smaller means too cautious.
  1 sigma / 3 sigma  share of the per-axis errors inside the reported 1 and 3 sigma. Honest:
                  about 68 % and 99.7 %.
  error / sigma   RMS error divided by RMS sigma per axis, honest about 1.
With several runs of the same scenario (Monte Carlo, different seeds) it also reports the share of
frames whose NEES averaged over the runs lies inside the 95 % band.

Example:
  tools/consistency.py fig8_vio.csv ~/datasets/synthetic/fig8
  tools/consistency.py run1.csv ~/datasets/synthetic/mc1 run2.csv ~/datasets/synthetic/mc2 ...
"""

import argparse
import os
import sys

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from metrics import associate, load_trajectory  # noqa: E402


def load_vio(path):
    with open(path) as f:
        names = f.readline().lstrip("#").strip().split(",")
    t = np.loadtxt(path, delimiter=",", skiprows=1, usecols=0, dtype=np.int64, ndmin=1)
    v = np.loadtxt(path, delimiter=",", skiprows=1, ndmin=2)
    return t, {n: v[:, i] for i, n in enumerate(names)}


def load_gt_biases(session):
    path = os.path.join(session, "mav0", "state_groundtruth_estimate0", "data.csv")
    t = np.loadtxt(path, delimiter=",", comments="#", usecols=0, dtype=np.int64)
    v = np.loadtxt(path, delimiter=",", comments="#")
    return (t, v[:, 11:14], v[:, 14:17]) if v.shape[1] >= 17 else None


def log_so3(R):
    cos_a = np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1.0, 1.0)
    angle = np.arccos(cos_a)
    w = np.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0], R[:, 1, 0] - R[:, 0, 1]], axis=1)
    small = angle < 1e-7
    return w * np.where(small, 0.5, angle / (2 * np.sin(np.where(small, 1.0, angle))))[:, None]


def chi2_band(dof, prob):
    """Wilson-Hilferty chi2 quantile, enough for the band of an average NEES."""
    z = {0.025: -1.959964, 0.975: 1.959964}[prob]
    a = 2.0 / (9 * dof)
    return dof * (1 - a + z * np.sqrt(a)) ** 3


def evaluate(est_path, session):
    t, c = load_vio(est_path)
    t_g, p_g, q_g = load_trajectory(session)
    keep = (t >= t_g[0]) & (t <= t_g[-1])
    t = t[keep]
    c = {k: v[keep] for k, v in c.items()}
    p_e = np.stack([c["p_x"], c["p_y"], c["p_z"]], axis=1)
    q_e = np.stack([c["q_w"], c["q_x"], c["q_y"], c["q_z"]], axis=1)
    _, _, R_e, p_gt, R_gt = associate((t, p_e, q_e), (t_g, p_g, q_g))

    C = np.zeros((len(t), 6, 6))
    for i in range(6):
        for j in range(i, 6):
            C[:, i, j] = C[:, j, i] = c[f"Cpose_{i}{j}"]
    e = np.concatenate([p_gt - p_e, log_so3(np.einsum("nij,nkj->nik", R_gt, R_e))], axis=1)
    ok = np.linalg.eigvalsh(C)[:, 0] > 0
    nees = {}
    for name, sl in (("pos", slice(0, 3)), ("att", slice(3, 6)), ("pose", slice(0, 6))):
        Cs, es = C[ok][:, sl, sl], e[ok][:, sl]
        nees[name] = np.einsum("ni,ni->n", es, np.linalg.solve(Cs, es[:, :, None])[:, :, 0])
    sigma = np.sqrt(np.einsum("nii->ni", C[ok]))
    r = {"t": t[ok], "nees": nees, "err": e[ok], "sigma": sigma, "c": c, "t_all": t}
    gb = load_gt_biases(session)
    if gb is not None:
        i = np.clip(np.searchsorted(gb[0], t[-1]), 0, len(gb[0]) - 1)
        r["bg_err"] = gb[1][i] - np.array([c["bg_x"][-1], c["bg_y"][-1], c["bg_z"][-1]])
        r["ba_err"] = gb[2][i] - np.array([c["ba_x"][-1], c["ba_y"][-1], c["ba_z"][-1]])
    return r


def main():
    ap = argparse.ArgumentParser(description="Covariance consistency of vio_replay output.")
    ap.add_argument("pairs", nargs="+", help="estimate.csv session [estimate.csv session ...]")
    args = ap.parse_args()
    if len(args.pairs) % 2:
        sys.exit("error: give pairs of estimate file and session folder")
    runs = [evaluate(args.pairs[i], args.pairs[i + 1]) for i in range(0, len(args.pairs), 2)]

    print(f"runs {len(runs)}, rows {sum(len(r['t']) for r in runs)}")
    for name, dof in (("pos", 3), ("att", 3), ("pose", 6)):
        allv = np.concatenate([r["nees"][name] for r in runs])
        print(f"  NEES {name:4s} mean {allv.mean():7.2f} (honest {dof}), median {np.median(allv):6.2f}")
    err = np.concatenate([r["err"] for r in runs])
    sig = np.concatenate([r["sigma"] for r in runs])
    for name, sl in (("position", slice(0, 3)), ("attitude", slice(3, 6))):
        inside1 = np.mean(np.abs(err[:, sl]) < sig[:, sl]) * 100
        inside3 = np.mean(np.abs(err[:, sl]) < 3 * sig[:, sl]) * 100
        ratio = np.sqrt(np.mean(err[:, sl] ** 2, axis=0) / np.mean(sig[:, sl] ** 2, axis=0))
        print(f"  {name:8s} inside 1 sigma {inside1:5.1f} %, 3 sigma {inside3:5.1f} % "
              f"(honest 68.3 / 99.7), error/sigma per axis {np.array2string(ratio, precision=2)}")
    last = runs[0]
    print(f"  end of run 1: position error {np.array2string(last['err'][-1, :3], precision=3)} m, "
          f"sigma {np.array2string(last['sigma'][-1, :3], precision=3)} m")
    print(f"                attitude error {np.array2string(np.degrees(last['err'][-1, 3:]), precision=3)} deg, "
          f"sigma {np.array2string(np.degrees(last['sigma'][-1, 3:]), precision=3)} deg")
    if "bg_err" in last:
        print(f"                gyro bias error {np.array2string(last['bg_err'], precision=5)} rad/s, "
              f"accel bias error {np.array2string(last['ba_err'], precision=4)} m/s^2")
    print(f"                td {last['c']['td_ms'][-1]:.2f} ms")

    n = len(runs)
    if n > 1 and len({len(r["t"]) for r in runs}) == 1:
        for name, dof in (("pos", 3), ("att", 3)):
            avg = np.mean([r["nees"][name] for r in runs], axis=0)
            lo, hi = chi2_band(dof * n, 0.025) / n, chi2_band(dof * n, 0.975) / n
            inside = np.mean((avg > lo) & (avg < hi)) * 100
            print(f"  average NEES {name} over {n} runs inside [{lo:.2f}, {hi:.2f}] on {inside:.0f} % of "
                  f"frames (honest about 95 %), mean {avg.mean():.2f}")


if __name__ == "__main__":
    main()
