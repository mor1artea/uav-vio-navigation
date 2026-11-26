#!/usr/bin/env python3
"""Turns the raw files of a simulated flight (sim/fly_record.py) into a session that vio_replay reads
(EuRoC layout, NED world, FRD body), and checks it.

Input, in <session>/raw:
  truth.csv    sim time [ns], position x y z (Gazebo ENU), quaternion w x y z (model FLU in ENU)
  imu.csv      FC time [us], gyro x y z [rad/s], accel x y z [m/s^2] (FRD, HIGHRES_IMU)
  baro.csv     FC time [us], altitude [m]
  landed.csv   FC time [us], MAV_LANDED_STATE
  clock.csv    FC time [us], latest sim time [ns] when that IMU message arrived
  images.csv   sim time [ns], file name (images are already in mav0/cam0/data)
  camera.yaml  camera model and mounting, written by sim/make_world.py

Two clocks: the images and the ground truth carry Gazebo simulation time, the MAVLink messages the
FC time since boot. The offset is found by matching the rotation rate of the ground truth with the
gyro. The gyro filter delay of the FC is part of that offset, so the filter should estimate a td near
zero on these sessions.

The checks compare the IMU with the ground truth. A wrong frame, sign or clock shows there.

Example (fly_record.py runs this itself):
  sim/make_session.py ~/datasets/sim/line1
"""

import argparse
import os
import sys

import numpy as np
import yaml

GRAVITY = 9.8  # m/s^2, the same as in the world file
ENU_TO_NED = np.array([[0.0, 1, 0], [1, 0, 0], [0, 0, -1]])
FRD_TO_FLU = np.diag([1.0, -1, -1])


def quat_to_rot(q):
    w, x, y, z = q.T
    return np.stack([
        np.stack([1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)], -1),
        np.stack([2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)], -1),
        np.stack([2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)], -1)], 1)


def rot_to_quat(R):
    """Quaternions w x y z of rotation matrices (N, 3, 3), kept continuous in sign."""
    out = np.zeros((len(R), 4))
    for i, m in enumerate(R):
        t = np.trace(m)
        if t > 0:
            s = 2 * np.sqrt(1 + t)
            q = [s / 4, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
        else:
            a = int(np.argmax(np.diag(m)))
            b, c = (a + 1) % 3, (a + 2) % 3
            s = 2 * np.sqrt(1 + m[a, a] - m[b, b] - m[c, c])
            q = [(m[c, b] - m[b, c]) / s, 0, 0, 0]
            q[1 + a], q[1 + b], q[1 + c] = s / 4, (m[b, a] + m[a, b]) / s, (m[c, a] + m[a, c]) / s
        q = np.array(q)
        if i > 0 and q @ out[i - 1] < 0:
            q = -q
        out[i] = q
    return out


def body_rates(t_s, R):
    """Body rates between consecutive rotations, at the middle times."""
    dR = np.einsum("nji,njk->nik", R[:-1], R[1:])
    angle = np.arccos(np.clip((np.trace(dR, axis1=1, axis2=2) - 1) / 2, -1, 1))
    axis = np.stack([dR[:, 2, 1] - dR[:, 1, 2], dR[:, 0, 2] - dR[:, 2, 0], dR[:, 1, 0] - dR[:, 0, 1]], 1)
    scale = np.where(angle < 1e-6, 0.5, angle / (2 * np.sin(np.where(angle < 1e-6, 1.0, angle))))
    return 0.5 * (t_s[:-1] + t_s[1:]), axis * (scale / np.diff(t_s))[:, None]


def read_rows(path, columns):
    """Rows of a csv: first column int64 (time), the rest as floats."""
    t, rows = [], []
    with open(path) as f:
        for line in f:
            if line.startswith("#") or not line.strip():
                continue
            cells = line.strip().split(",")
            t.append(int(cells[0]))
            rows.append([float(c) for c in cells[1:1 + columns]])
    return np.array(t, dtype=np.int64), np.array(rows)


def clock_offset(t_truth, R, t_imu_us, gyro, t_clock_us, sim_clock_ns):
    """Offset [ns] that turns FC time into sim time, and the correlation at that offset. R: truth
    attitude (FRD body in NED), gyro: FRD, so each axis is compared with itself."""
    coarse = int(np.median(sim_clock_ns - t_clock_us * 1000))
    t0 = t_truth[0]
    tm, w = body_rates((t_truth - t0) * 1e-9, R)
    ti = (t_imu_us * 1000 + coarse - t0) * 1e-9
    n = max(1, int(round(0.05 / np.median(np.diff(ti)))))  # 50 ms average against the noise
    w = smooth(w, max(1, int(round(0.05 / np.median(np.diff(tm))))))
    keep = (ti > tm[0] + 0.6) & (ti < tm[-1] - 0.6)
    g = smooth(gyro, n)[keep]
    ti, g = ti[keep], g - g.mean(axis=0)  # without the mean: gyro bias
    lags = np.arange(-0.5, 0.5, 0.001)
    score = np.zeros(len(lags))
    for i, lag in enumerate(lags):
        r = np.stack([np.interp(ti + lag, tm, w[:, k]) for k in range(3)], 1)
        r -= r.mean(axis=0)
        score[i] = np.sum(r * g) / np.sqrt(np.sum(r * r) * np.sum(g * g) + 1e-30)
    i = int(np.clip(np.argmax(score), 1, len(lags) - 2))
    a, b, c = score[i - 1:i + 2]
    fine = lags[i] + 0.001 * 0.5 * (a - c) / (a - 2 * b + c - 1e-30)
    return coarse + int(round(fine * 1e9)), float(score[i])


def smooth(x, n):
    k = np.ones(n) / n
    return np.stack([np.convolve(x[:, i], k, "same") for i in range(x.shape[1])], 1)


def write_csv(path, header, rows):
    with open(path, "w") as f:
        f.write(header + "\n")
        f.writelines(",".join(r) + "\n" for r in rows)


def convert(session):
    raw = os.path.join(session, "raw")
    t_tr, tr = read_rows(os.path.join(raw, "truth.csv"), 7)
    t_imu, imu = read_rows(os.path.join(raw, "imu.csv"), 6)
    t_baro, baro = read_rows(os.path.join(raw, "baro.csv"), 1)
    t_land, land = read_rows(os.path.join(raw, "landed.csv"), 1)
    t_clk, clk = read_rows(os.path.join(raw, "clock.csv"), 1)
    if len(t_tr) < 100 or len(t_imu) < 100:
        sys.exit(f"error: not enough data ({len(t_tr)} truth rows, {len(t_imu)} IMU rows)")
    _, first = np.unique(t_tr, return_index=True)
    t_tr, tr = t_tr[first], tr[first]

    # Ground truth: Gazebo ENU world and FLU body to NED world and FRD body.
    R_enu_flu = quat_to_rot(tr[:, 3:7] / np.linalg.norm(tr[:, 3:7], axis=1)[:, None])
    p = tr[:, 0:3] @ ENU_TO_NED.T
    R = ENU_TO_NED @ R_enu_flu @ FRD_TO_FLU
    ts = (t_tr - t_tr[0]) * 1e-9
    v = np.stack([np.gradient(p[:, k], ts) for k in range(3)], 1)

    offset, corr = clock_offset(t_tr, R, t_imu, imu[:, 0:3],
                                t_clk, clk[:, 0].astype(np.int64))
    to_sim = lambda t_us: t_us * 1000 + offset
    ti = to_sim(t_imu)

    mav = os.path.join(session, "mav0")
    for d in ("imu0", "state_groundtruth_estimate0", "baro0", "landed0", "cam0"):
        os.makedirs(os.path.join(mav, d), exist_ok=True)
    write_csv(os.path.join(mav, "imu0", "data.csv"), "#timestamp [ns],w_x,w_y,w_z,a_x,a_y,a_z",
              ([str(t)] + [f"{x:.9f}" for x in r] for t, r in zip(ti, imu)))
    q = rot_to_quat(R)
    zeros = ["0"] * 6  # biases: unknown, the metrics do not use them
    write_csv(os.path.join(mav, "state_groundtruth_estimate0", "data.csv"),
              "#timestamp,p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z,bg_x,bg_y,bg_z,ba_x,ba_y,ba_z",
              ([str(t)] + [f"{x:.9f}" for x in np.r_[p[i], q[i], v[i]]] + zeros
               for i, t in enumerate(t_tr)))
    if len(t_baro):
        every = max(1, int(round(len(t_baro) / ((t_baro[-1] - t_baro[0]) * 1e-6) / 20)))  # to 20 Hz
        write_csv(os.path.join(mav, "baro0", "data.csv"), "#timestamp [ns],altitude [m]",
                  ([str(to_sim(t)), f"{a[0]:.3f}"] for t, a in zip(t_baro[::every], baro[::every])))
    if len(t_land):
        write_csv(os.path.join(mav, "landed0", "data.csv"), "#timestamp [ns],landed_state",
                  ([str(to_sim(t)), str(int(s[0]))] for t, s in zip(t_land, land)))

    with open(os.path.join(raw, "images.csv")) as f:
        images = sorted((int(l.split(",")[0]), l.strip().split(",")[1]) for l in f if l[0] != "#")
    # Only frames inside the IMU and ground truth time.
    lo, hi = max(ti[0], t_tr[0]), min(ti[-1], t_tr[-1])
    images = [(t, n) for t, n in images if lo <= t <= hi]
    write_csv(os.path.join(mav, "cam0", "data.csv"), "#timestamp [ns],filename",
              ([str(t), n] for t, n in images))

    with open(os.path.join(raw, "camera.yaml")) as f:
        cam = yaml.safe_load(f)
    with open(os.path.join(mav, "cam0", "sensor.yaml"), "w") as f:
        yaml.safe_dump(cam, f, sort_keys=False, default_flow_style=None)
    rate = (len(ti) - 1) / ((ti[-1] - ti[0]) * 1e-9)
    # The noise values are the placeholders of config/vio.yaml, not measured from the simulator.
    with open(os.path.join(mav, "imu0", "sensor.yaml"), "w") as f:
        yaml.safe_dump(dict(sensor_type="imu", comment="SITL HIGHRES_IMU", rate_hz=round(rate),
                            gyroscope_noise_density=0.004, gyroscope_random_walk=0.0002,
                            accelerometer_noise_density=0.04, accelerometer_random_walk=0.002),
                       f, sort_keys=False)
    with open(os.path.join(session, "session.yaml"), "w") as f:
        yaml.safe_dump(dict(source="sim/fly_record.py", world_frame="NED",
                            gravity_world=[0.0, 0.0, GRAVITY]), f, sort_keys=False, default_flow_style=None)

    # Checks: the IMU against the ground truth.
    tm, w_true = body_rates(ts, R)
    a_world = np.stack([np.gradient(v[:, k], ts) for k in range(3)], 1) - np.array([0, 0, GRAVITY])
    f_true = np.einsum("nji,nj->ni", R, a_world)  # specific force in the body frame
    t_rel = (ti - t_tr[0]) * 1e-9
    inside = (t_rel > ts[0] + 1) & (t_rel < ts[-1] - 1)
    n = max(1, int(round(rate * 0.2)))  # 0.2 s average: noise and vibration out
    gyro_s = smooth(imu[inside, 0:3], n)
    acc_s = smooth(imu[inside, 3:6], n)
    gyro_t = smooth(np.stack([np.interp(t_rel[inside], tm, w_true[:, k]) for k in range(3)], 1), n)
    acc_t = smooth(np.stack([np.interp(t_rel[inside], ts, f_true[:, k]) for k in range(3)], 1), n)
    gyro_err = np.sqrt(np.median((gyro_s - gyro_t) ** 2, axis=0))
    acc_err = np.sqrt(np.median((acc_s - acc_t) ** 2, axis=0))

    frame_gaps = np.diff([t for t, _ in images]) * 1e-9
    checks = [
        ("clock: correlation of gyro and ground truth rate", corr, corr > 0.8, "> 0.8"),
        ("gyro minus truth, worst axis [rad/s]", gyro_err.max(), gyro_err.max() < 0.05, "< 0.05"),
        ("accel minus truth, worst axis [m/s^2]", acc_err.max(), acc_err.max() < 0.8, "< 0.8"),
        ("IMU rate [Hz]", rate, rate > 150, "> 150"),
        ("largest IMU gap [s]", np.diff(ti).max() * 1e-9, np.diff(ti).max() < 5e7, "< 0.05"),
        ("camera frames", len(images), len(images) > 100, "> 100"),
        ("largest camera gap [s]", frame_gaps.max() if len(frame_gaps) else 0,
         len(frame_gaps) > 0 and frame_gaps.max() < 0.1, "< 0.1"),
    ]
    print(f"clock offset (sim - FC): {offset * 1e-9:.4f} s")
    for name, value, ok, want in checks:
        print(f"  {'ok  ' if ok else 'FAIL'} {name:52s} {value:10.4f}  (want {want})")
    return all(ok for _, _, ok, _ in checks)


def main():
    ap = argparse.ArgumentParser(description="Convert and check a simulated flight.")
    ap.add_argument("session", help="folder with raw/ and mav0/cam0/data")
    args = ap.parse_args()
    if not convert(os.path.expanduser(args.session)):
        sys.exit("some checks failed: do not trust numbers from this session")


if __name__ == "__main__":
    main()
