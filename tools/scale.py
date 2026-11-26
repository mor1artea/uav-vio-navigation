#!/usr/bin/env python3
"""Signed scale of an estimated trajectory against the ground truth, also on closed loops.

Scale = distance moved in the estimate / true distance moved, measured on short pieces of the true
path (chords of --length m, default 100). A chord length does not depend on the frame, so no
alignment is needed, and on a circle that is too large every piece is too long by the same factor
(the along-track error of a closed loop hides that: a scaled circle comes back to its start).
Above 1: the estimate moves too far, i.e. its speed is too high.

Per run: the mean scale over the whole flight (estimated path / true path, chord by chord), the
scale over the last 10 % of the path, and the horizontal speed ratio |v_est| / |v_true| at 95 % of
the flight (vio_replay --mode vio output; skipped when the true speed is below 1 m/s, as in hover).
With several runs: median (range).

Examples:
  tools/scale.py /tmp/eval/line20                      (eval_runs.py folder with seed* runs)
  tools/scale.py /tmp/eval/tilt --session ~/datasets/synthetic/fig8_tilt_img   (start* runs)
  tools/scale.py run.csv ~/datasets/synthetic/line                             (one pair)
"""

import argparse
import glob
import os
import sys

import numpy as np


def load_truth(session):
    """t (int64 ns), p (N, 3), v (N, 3) from the session's ground truth (EuRoC layout)."""
    path = os.path.join(session, "mav0", "state_groundtruth_estimate0", "data.csv")
    t, p, v = [], [], []
    with open(path) as f:
        for line in f:
            if not line.strip() or line.startswith("#"):
                continue
            c = line.split(",")
            t.append(int(c[0]))  # exact: EuRoC time stamps do not fit a double
            p.append([float(x) for x in c[1:4]])
            v.append([float(x) for x in c[8:11]])
    return np.array(t, dtype=np.int64), np.array(p), np.array(v)


def scale_of_run(est_csv, session, length=100.0):
    e = np.genfromtxt(est_csv, delimiter=",", names=True)
    tg, pg_all, vg_all = load_truth(session)
    te = e["timestamp_ns"].astype(np.int64)
    keep = (te >= tg[0]) & (te <= tg[-1])
    t0 = tg[0]
    x, xg = (te[keep] - t0).astype(float), (tg - t0).astype(float)
    pe = np.stack([e["p_x"], e["p_y"], e["p_z"]], 1)[keep]
    pg = np.stack([np.interp(x, xg, pg_all[:, k]) for k in range(3)], 1)
    vg = np.stack([np.interp(x, xg, vg_all[:, k]) for k in range(3)], 1)
    s = np.r_[0, np.cumsum(np.linalg.norm(np.diff(pg, axis=0), axis=1))]
    out = {"path_m": s[-1], "mean": np.nan, "end": np.nan, "speed95": np.nan}
    marks = np.searchsorted(s, np.arange(0.0, s[-1], length))
    if len(marks) >= 3:
        de = np.linalg.norm(np.diff(pe[marks], axis=0), axis=1)
        dg = np.linalg.norm(np.diff(pg[marks], axis=0), axis=1)
        out["mean"] = de.sum() / dg.sum()
        last = s[marks[1:]] >= 0.9 * s[-1]
        if last.any():
            out["end"] = de[last].sum() / dg[last].sum()
    if "v_x" in e.dtype.names:
        i = int(0.95 * (len(x) - 1))
        ve = np.array([e["v_x"][keep][i], e["v_y"][keep][i]])
        if np.linalg.norm(vg[i, :2]) >= 1.0:
            out["speed95"] = np.linalg.norm(ve) / np.linalg.norm(vg[i, :2])
    return out


def main():
    ap = argparse.ArgumentParser(description="Signed scale of vio_replay output against the ground truth.")
    ap.add_argument("inputs", nargs="+", help="eval_runs.py folders, or pairs: estimate.csv session")
    ap.add_argument("--session", help="session of start* runs in a folder (one session, several starts)")
    ap.add_argument("--length", type=float, default=100.0, help="chord length, m")
    args = ap.parse_args()

    runs = []  # (name, csv, session)
    i = 0
    while i < len(args.inputs):
        a = args.inputs[i]
        if os.path.isdir(a) and not os.path.isdir(os.path.join(a, "mav0")):
            for csv in sorted(glob.glob(os.path.join(a, "*.csv"))):
                ses = csv[:-4] if os.path.isdir(csv[:-4]) else args.session
                if ses:
                    runs.append((os.path.basename(a.rstrip("/")), csv, ses))
            i += 1
        elif i + 1 < len(args.inputs):
            runs.append((os.path.basename(a), a, args.inputs[i + 1]))
            i += 2
        else:
            sys.exit(f"error: {a} is neither a run folder nor followed by a session")
    groups = {}
    for name, csv, ses in runs:
        groups.setdefault(name, []).append(scale_of_run(csv, ses, args.length))

    def fmt(v, p):
        v = v[~np.isnan(v)]
        return f"{np.median(v):.{p}f} ({v.min():.{p}f}-{v.max():.{p}f})" if len(v) else "-"

    for name, rs in groups.items():
        col = {k: np.array([r[k] for r in rs]) for k in ("mean", "end", "speed95", "path_m")}
        print(f"{name}: {len(rs)} runs, path {np.median(col['path_m']):.0f} m, scale mean {fmt(col['mean'], 3)}, "
              f"last 10 % {fmt(col['end'], 3)}, speed ratio at 95 % {fmt(col['speed95'], 3)}")


if __name__ == "__main__":
    main()
