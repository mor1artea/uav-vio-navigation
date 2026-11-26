#!/usr/bin/env python3
"""Front end evaluation: feature tracks and per-frame numbers from vio_track.

Prints:
  track length distribution, success rate, forward-backward error, spread over the grid, usable
  image fraction, time per frame (from stats.csv);
  with the session: track correctness against the ground truth. Every track of 5 or more frames is
  triangulated from the ground truth camera poses and reprojected. A track whose RMS reprojection
  error is above 2 px counts as wrong. Done twice: on pieces of 10 frames (about what the filter's
  window sees) and on whole tracks (what long-lived landmarks see, includes slow KLT drift).
  Checked on perfect synthetic tracks with 0.5 px noise: median 0.7 px, none above 2 px.

Example:
  tools/eval_tracks.py features.csv stats.csv ~/datasets/euroc/MH_01_easy
"""

import argparse
import os
import sys

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from gen_synthetic import project  # noqa: E402  same projection as the generator
from metrics import associate, load_trajectory  # noqa: E402


def load_csv(path):
    """Rows of a numeric CSV with an int64 first column (time stamps in ns)."""
    t, rows = [], []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            cells = line.split(",")
            t.append(int(cells[0]))
            rows.append([float(c) for c in cells[1:]])
    return np.array(t, dtype=np.int64), np.array(rows)


def load_camera(session):
    with open(os.path.join(session, "mav0", "cam0", "sensor.yaml")) as f:
        y = yaml.safe_load("".join(l for l in f if not l.startswith("%YAML")))
    fx, fy, cx, cy = y["intrinsics"]
    cam = dict(model=y["distortion_model"], fx=fx, fy=fy, cx=cx, cy=cy,
               distortion=y["distortion_coefficients"], max_angle_deg=89.0,
               width=y["resolution"][0], height=y["resolution"][1])
    return cam, np.array(y["T_BS"]["data"]).reshape(4, 4)


def unproject(uv, cam):
    """Unit bearings of raw pixels (N, 2), Newton iterations on the distortion."""
    d = np.stack([(uv[:, 0] - cam["cx"]) / cam["fx"], (uv[:, 1] - cam["cy"]) / cam["fy"]], axis=1)
    k = cam["distortion"]
    if cam["model"] == "equidistant":
        td = np.linalg.norm(d, axis=1)
        th = td.copy()
        for _ in range(20):
            t2 = th * th
            f = th * (1 + t2 * (k[0] + t2 * (k[1] + t2 * (k[2] + t2 * k[3])))) - td
            df = 1 + t2 * (3 * k[0] + t2 * (5 * k[1] + t2 * (7 * k[2] + t2 * 9 * k[3])))
            th -= f / df
        s = np.where(td > 1e-12, np.sin(th) / np.where(td > 1e-12, td, 1), 1.0)
        return np.stack([s * d[:, 0], s * d[:, 1], np.cos(th)], axis=1)
    x, y = d[:, 0].copy(), d[:, 1].copy()
    for _ in range(20):  # fixed point, as cv::undistortPoints
        r2 = x * x + y * y
        radial = 1 + k[0] * r2 + k[1] * r2 * r2
        dx = 2 * k[2] * x * y + k[3] * (r2 + 2 * x * x)
        dy = k[2] * (r2 + 2 * y * y) + 2 * k[3] * x * y
        x, y = (d[:, 0] - dx) / radial, (d[:, 1] - dy) / radial
    b = np.stack([x, y, np.ones_like(x)], axis=1)
    return b / np.linalg.norm(b, axis=1)[:, None]


def correctness(t, ids, uv, session, window=0, min_len=5):
    """Triangulates every track of min_len frames or more from the ground truth camera poses.
    With window > 0 each track is first cut into pieces of that many frames (what a filter with a
    window of that size sees). Returns per-piece RMS reprojection errors (px) and the number of
    pieces without parallax."""
    cam, T_bc = load_camera(session)
    frames = np.unique(t)
    dummy = (frames, np.zeros((len(frames), 3)), np.tile([1.0, 0, 0, 0], (len(frames), 1)))
    t_f, _, _, p_wb, R_wb = associate(dummy, load_trajectory(session))  # ground truth at the frames
    idx = np.searchsorted(t_f, t)
    keep = (idx < len(t_f)) & (t_f[np.minimum(idx, len(t_f) - 1)] == t)
    t, ids, uv, idx = t[keep], ids[keep], uv[keep], idx[keep]
    R_wc = R_wb[idx] @ T_bc[:3, :3]
    c_w = p_wb[idx] + R_wb[idx] @ T_bc[:3, 3]
    d_w = np.einsum("nij,nj->ni", R_wc, unproject(uv, cam))

    order = np.lexsort((t, ids))
    ids, uv, R_wc, c_w, d_w = ids[order], uv[order], R_wc[order], c_w[order], d_w[order]
    if window > 0:
        first = np.r_[0, np.nonzero(np.diff(ids))[0] + 1]
        pos = np.arange(len(ids)) - np.repeat(first, np.diff(np.r_[first, len(ids)]))
        ids = ids * 100000 + pos // window
    starts = np.r_[0, np.nonzero(np.diff(ids))[0] + 1, len(ids)]
    rms, no_parallax = [], 0
    for a, b in zip(starts[:-1], starts[1:]):
        if b - a < min_len:
            continue
        # Point closest to all rays: sum w (I - d d^T)(X - c) = 0. With w = 1 / distance^2 this
        # is the least squares of the angular (roughly pixel) errors; a few rounds of reweighting.
        P = np.eye(3)[None] - d_w[a:b, :, None] * d_w[a:b, None, :]
        w = np.ones(b - a)
        for _ in range(4):
            A = np.einsum("n,nij->ij", w, P)
            ev = np.linalg.eigvalsh(A)
            if ev[0] < 1e-6 * ev[-1]:
                break
            X = np.linalg.solve(A, np.einsum("n,nij,nj->i", w, P, c_w[a:b]))
            w = 1 / np.maximum(np.sum((X - c_w[a:b]) ** 2, axis=1), 1e-6)
        if ev[0] < 1e-6 * ev[-1]:
            no_parallax += 1
            continue
        pc = np.einsum("nji,nj->ni", R_wc[a:b], X - c_w[a:b])  # R_wc^T (X - c)
        proj, _ = project(pc, cam)  # its validity flag also rejects points just off the image
        err = np.where(pc[:, 2] > 0, np.linalg.norm(proj - uv[a:b], axis=1), 1e3)
        rms.append(np.sqrt(np.mean(err ** 2)))
    return np.array(rms), no_parallax


def main():
    ap = argparse.ArgumentParser(description="Front end evaluation of vio_track output.")
    ap.add_argument("features", help="features.csv from vio_track")
    ap.add_argument("stats", help="stats.csv from vio_track")
    ap.add_argument("session", nargs="?", help="session folder with ground truth, for track correctness")
    args = ap.parse_args()

    t, f = load_csv(args.features)
    ids, uv = f[:, 0].astype(np.int64), f[:, 1:3]
    _, s = load_csv(args.stats)
    col = {name: i for i, name in enumerate(
        "attempted tracked lost_klt lost_fb lost_mask added total fb_median fb_p90 pred_median cells_usable "
        "cells_occupied max_cell_share usable_fraction brightness contrast blur saturated "
        "ms_prepare ms_track ms_detect ms_total".split())}

    def c(name):
        return s[:, col[name]]

    lengths = np.bincount(np.unique(ids, return_inverse=True)[1])
    tracked = lengths[lengths >= 2]
    att = c("attempted")
    has = att > 0
    print(f"frames {len(s)}, features per frame {c('total').mean():.1f}, tracks {len(lengths)}")
    print(f"  track length [frames]   median {np.median(tracked):.0f}, p90 {np.percentile(tracked, 90):.0f}, "
          f">= 10 frames {np.mean(tracked >= 10) * 100:.0f} % (tracks seen in 2+ frames)")
    print(f"  success rate            {np.mean(c('tracked')[has] / att[has]):.3f}  "
          f"(lost per frame: KLT {c('lost_klt').mean():.2f}, forward-backward {c('lost_fb').mean():.2f}, "
          f"mask/border {c('lost_mask').mean():.2f})")
    print(f"  forward-backward [px]   median {np.median(c('fb_median')[has]):.3f}, p90 {np.median(c('fb_p90')[has]):.3f}")
    print(f"  tracked - predicted     median {np.median(c('pred_median')[has]):.2f} px "
          f"(gyro on: motion the gyro did not explain; off: the whole image motion)")
    print(f"  grid cells occupied     {c('cells_occupied').mean():.1f} of {c('cells_usable').mean():.1f} usable, "
          f"largest cell share {c('max_cell_share').mean():.3f}")
    print(f"  usable image fraction   {c('usable_fraction').mean():.3f}")
    print(f"  image                   brightness {c('brightness').mean():.0f}, contrast {c('contrast').mean():.0f}, "
          f"blur {c('blur').mean():.0f}")
    ms = c("ms_total")
    print(f"  time per frame [ms]     mean {ms.mean():.2f}, p95 {np.percentile(ms, 95):.2f}, max {ms.max():.2f}  "
          f"(prepare {c('ms_prepare').mean():.2f}, track {c('ms_track').mean():.2f}, detect {c('ms_detect').mean():.2f})")

    if args.session:
        print("  correctness vs ground truth (RMS reprojection after triangulation, pieces of 5+ frames)")
        for window, name in ((10, "10-frame windows"), (0, "whole tracks")):
            rms, no_parallax = correctness(t, ids, uv, args.session, window)
            print(f"    {name:17s} {len(rms):6d}: median {np.median(rms):.2f} px, p90 {np.percentile(rms, 90):.2f} px, "
                  f"wrong > 2 px {np.mean(rms > 2) * 100:.1f} %, > 5 px {np.mean(rms > 5) * 100:.1f} %"
                  f" ({no_parallax} without parallax)")


if __name__ == "__main__":
    main()
