#!/usr/bin/env python3
"""Small synthetic session generator for the offline VIO work.

Writes a session in the EuRoC folder layout that vio_replay reads:
  session.yaml                                     gravity, world frame, generator settings
  mav0/imu0/data.csv, sensor.yaml                  IMU at 200 Hz (gyro rad/s, specific force m/s^2)
  mav0/cam0/features.csv, sensor.yaml              feature observations at 30 Hz (no images)
  mav0/state_groundtruth_estimate0/data.csv        exact pose, velocity and IMU biases
  landmarks.csv                                    the 3D points, for debugging

Feature ids are landmark ids, unless --track-age or --drift-q is given: then a feature is re-detected
(new id) after --track-age frames or when its landmark was out of view for a frame, like the tracker
does, and its id is landmark id + n_landmarks * (number of earlier tracks of that landmark).

KLT-like drift (--drift-q, DESIGN.md section 13.4 a): each track gets a pixel offset that does a random
walk in the view change of its landmark, with variance q per degree per image axis. The view change
of a step is the angle between the rays from the camera to the landmark in two consecutive frames,
so it adds up along the track and does not go back when the view comes back (chained KLT). The
offset starts at 0 when the track starts; white pixel noise (--pixel-noise) comes on top.

World frame NED (z down), body frame FRD at the IMU, camera looking down. The drone attitude follows
the acceleration like a multirotor (thrust along -z body), yaw follows the flight direction.

Scenarios:
  line     straight leg north for --duration s at constant speed (scale drift); loop: a circle of
           --radius. Both start in flight; --manoeuvre speed or sturn adds speed changes (+-20 %)
           or S-turns (30 m) every --manoeuvre-period s
  figure8  closed figure-eight at constant height with speed changes (default)
  hover    hover with small wobbles, for the hover drift metric
  approach_hover  straight flight, smooth stop, hover (--duration s): landmarks and hover drift
  liftoff  on the ground with motors off (--still-time, default 0), then running on the ground
           (--ground-time), vertical climb to --height, straight flight north at --speed for
           --duration s. See below.

Simulated baro (--baro, mav0/baro0/data.csv, 20 Hz, altitude in m, up). A guess, not measured
(DATASETS.md section 1): true height above the ground plane plus 300 m, white noise 0.3 m, a bias
random walk of 0.03 m per sqrt(s), prop wash -0.2 m per m/s^2 of thrust above hover, a speed term
-0.01 m per (m/s)^2 of horizontal speed, and ground effect +0.5 m * exp(-height / 0.5 m) while the
motors run. Own random stream: everything else in the session stays the same.

Liftoff scenario (DESIGN.md section 11, "Takeoff"): the camera stands --ground-height above the
ground (default: drawn per seed between 0.05 and 0.2 m). While the motors run, the IMU gets
vibration the filter's noise model does not know: white accel noise of --vibration m/s^2 per
sample, a constant accel offset of --vibration-offset m/s^2 in a random direction (rectified
vibration) and white gyro noise of --vibration-gyro rad/s. Extra points around the takeoff spot,
denser towards it (log-uniform distance 0.02-20 m), so features are seen at every height;
--blind-below H removes every observation while the camera is lower than H m (out of focus, in the
drone's shadow); such frames are written as one row with feature id -1. The FC landed flag goes to mav0/landed0/data.csv at --landed-rate Hz: 1 =
on the ground (MAV_LANDED_STATE_ON_GROUND), 2 = in the air; it clears --landed-clear s after
the liftoff (negative: before it, as in Stabilize and Acro; positive: late, as AltHold can be).

Example:
  tools/gen_synthetic.py ~/datasets/synthetic/fig8 --scenario figure8 --speed 10 --laps 1
"""

import argparse
import math
import os
import shutil
import sys

import numpy as np
import yaml

GRAVITY = np.array([0.0, 0.0, 9.81])  # NED

# Nominal down-looking camera: image top towards the nose (DESIGN.md / config/vio.yaml).
R_BODY_CAM_NOMINAL = np.array([[0.0, -1.0, 0.0], [1.0, 0.0, 0.0], [0.0, 0.0, 1.0]])


# ---------------------------------------------------------------- trajectories (p, v, a, yaw)

def figure8(t, args):
    """Figure eight, closed after each lap, with +-20 % speed changes for scale information."""
    r, h, dh = args.radius, args.height, args.height_change
    # Mean |dc/dtheta| over a lap, to get the requested average speed.
    th = np.linspace(0, 2 * math.pi, 2000, endpoint=False)
    mean_dc = np.mean(np.sqrt((r * np.cos(th)) ** 2 + (r * np.cos(2 * th)) ** 2 + (dh * np.cos(th)) ** 2))
    w0 = args.speed / mean_dc
    total = args.laps * 2 * math.pi / w0
    tm = total / max(1, round(total / 20.0))  # speed modulation period, divides the total time
    amp = 0.2 * w0 * tm / (2 * math.pi)
    k = 2 * math.pi / tm

    theta = w0 * t + amp * np.sin(k * t)
    dtheta = w0 + amp * k * np.cos(k * t)
    ddtheta = -amp * k * k * np.sin(k * t)

    c = np.stack([r * np.sin(theta), 0.5 * r * np.sin(2 * theta), -(h + dh * np.sin(theta))], axis=1)
    dc = np.stack([r * np.cos(theta), r * np.cos(2 * theta), -dh * np.cos(theta)], axis=1)
    ddc = np.stack([-r * np.sin(theta), -2 * r * np.sin(2 * theta), dh * np.sin(theta)], axis=1)

    p = c
    v = dc * dtheta[:, None]
    a = ddc * (dtheta ** 2)[:, None] + dc * ddtheta[:, None]
    yaw = np.arctan2(v[:, 1], v[:, 0])
    return p, v, a, yaw


def figure8_duration(args):
    r, dh = args.radius, args.height_change
    th = np.linspace(0, 2 * math.pi, 2000, endpoint=False)
    mean_dc = np.mean(np.sqrt((r * np.cos(th)) ** 2 + (r * np.cos(2 * th)) ** 2 + (dh * np.cos(th)) ** 2))
    return args.laps * 2 * math.pi * mean_dc / args.speed


def hover(t, args):
    """Hover at the given height with small slow wobbles in position and yaw."""
    def wobble(amp, period):
        k = 2 * math.pi / period
        return amp * np.sin(k * t), amp * k * np.cos(k * t), -amp * k * k * np.sin(k * t)

    xs, ys, zs = wobble(0.3, 7.0), wobble(0.3, 11.0), wobble(0.2, 5.0)
    p = np.stack([xs[0], ys[0], -args.height + zs[0]], axis=1)
    v = np.stack([xs[1], ys[1], zs[1]], axis=1)
    a = np.stack([xs[2], ys[2], zs[2]], axis=1)
    yaw = 0.2 * np.sin(2 * math.pi * t / 13.0)
    return p, v, a, yaw


APPROACH_CRUISE_S, APPROACH_STOP_S = 5.0, 4.0


def approach_hover(t, args):
    """Straight flight north at --speed, a smooth stop, then hover (with the hover wobbles) for
    --duration seconds. Features seen while slowing down have parallax, so landmarks can be
    initialized before the hover, as on the real drone."""
    v0, t1, T = args.speed, APPROACH_CRUISE_S, APPROACH_STOP_S
    tau = np.clip((t - t1) / T, 0.0, 1.0)
    # Speed falls from v0 to 0 with a quintic smoothstep s(tau): continuous acceleration.
    s = 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5
    S = 2.5 * tau ** 4 - 3 * tau ** 5 + tau ** 6          # integral of s over tau
    ds = (30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4) * ((t > t1) & (t < t1 + T))
    x = np.where(t < t1, v0 * t, v0 * t1 + v0 * T * (np.minimum(tau, 1.0) - S))
    vx = v0 * (1 - s)
    ax = -v0 * ds / T
    p, v, a, yaw = hover(t, args)
    p[:, 0] += x
    v[:, 0] += vx
    a[:, 0] += ax
    return p, v, a, yaw


def approach_hover_duration(args):
    return APPROACH_CRUISE_S + APPROACH_STOP_S + args.duration


def velocity_step(t, t0, T, dv):
    """Velocity change by dv between t0 and t0 + T (quintic smoothstep): position, velocity and
    acceleration it adds."""
    tau = np.clip((t - t0) / T, 0.0, 1.0)
    s = 10 * tau ** 3 - 15 * tau ** 4 + 6 * tau ** 5
    S = 2.5 * tau ** 4 - 3 * tau ** 5 + tau ** 6          # integral of s over tau
    ds = (30 * tau ** 2 - 60 * tau ** 3 + 30 * tau ** 4) * ((t > t0) & (t < t0 + T))
    x = dv * np.where(t < t0 + T, T * S, T * 0.5 + (t - t0 - T))
    return x, dv * s, dv * ds / T


LIFTOFF_IMU_ABOVE_CAMERA = 0.05


def liftoff_times(args):
    """Liftoff time, end of the climb, start of the horizontal flight."""
    t_lift = args.still_time + args.ground_time
    t_climb = (args.height - args.ground_height) / args.climb_speed  # at full climb speed
    t_level = t_lift + t_climb + args.climb_step
    return t_lift, t_level, t_level + 1.0


def liftoff(t, args):
    """On the ground, then a vertical climb at --climb-speed to --height (the climb speed is reached
    and left over --climb-step seconds each), then north at --speed."""
    t_lift, _, t_go = liftoff_times(args)
    t_climb = (args.height - args.ground_height) / args.climb_speed
    z0 = -(args.ground_height + LIFTOFF_IMU_ABOVE_CAMERA)  # NED, the IMU is above the camera
    up1 = velocity_step(t, t_lift, args.climb_step, args.climb_speed)
    up2 = velocity_step(t, t_lift + t_climb, args.climb_step, -args.climb_speed)
    fwd = velocity_step(t, t_go, 3.0, args.speed)
    zero = np.zeros_like(t)
    p = np.stack([fwd[0], zero, z0 - up1[0] - up2[0]], axis=1)
    v = np.stack([fwd[1], zero, -up1[1] - up2[1]], axis=1)
    a = np.stack([fwd[2], zero, -up1[2] - up2[2]], axis=1)
    return p, v, a, zero


def liftoff_duration(args):
    return liftoff_times(args)[2] + 3.0 + args.duration


def from_position(pos, t, args):
    """Velocity and acceleration of a position function by central differences (exact enough for
    smooth paths: the error is far below the IMU noise)."""
    h = 1e-3
    p0, pm, pp = pos(t, args), pos(t - h, args), pos(t + h, args)
    v = (pp - pm) / (2 * h)
    a = (pp - 2 * p0 + pm) / (h * h)
    return p0, v, a, np.arctan2(v[:, 1], v[:, 0])


def manoeuvre(t, args):
    """Along-track distance factor and lateral offset for --manoeuvre: none, speed (speed +-20 %
    with period --manoeuvre-period) or sturn (lateral S-turns of 30 m with that period)."""
    k = 2 * math.pi / args.manoeuvre_period
    along = args.speed * t
    side = np.zeros_like(t)
    if args.manoeuvre == "speed":
        along = args.speed * (t + 0.2 * (1 - np.cos(k * t)) / k)
    elif args.manoeuvre == "sturn":
        side = 30.0 * np.sin(k * t)
    return along, side


def line_position(t, args):
    along, side = manoeuvre(t, args)
    return np.stack([along, side, np.full_like(t, -args.height)], axis=1)


def loop_position(t, args):
    along, side = manoeuvre(t, args)
    th = along / args.radius
    r = args.radius + side
    return np.stack([r * np.sin(th), args.radius - r * np.cos(th), np.full_like(t, -args.height)], axis=1)


def line(t, args):
    """Straight leg north at --speed and --height for --duration s (scale drift), from the start in
    flight; --manoeuvre adds speed changes or S-turns."""
    return from_position(line_position, t, args)


def loop(t, args):
    """Circle of --radius at --speed and --height for --duration s, from the start in flight;
    --manoeuvre as for line."""
    return from_position(loop_position, t, args)


SCENARIOS = {"figure8": figure8, "hover": hover, "approach_hover": approach_hover, "liftoff": liftoff,
             "line": line, "loop": loop}


# ---------------------------------------------------------------- rotations

def attitude(a, yaw):
    """R_wb from the acceleration (thrust direction) and yaw. Shape (N, 3, 3)."""
    f = a - GRAVITY                              # specific force in the world frame
    z_b = -f / np.linalg.norm(f, axis=1)[:, None]  # body z (down) opposite to the thrust
    x_c = np.stack([np.cos(yaw), np.sin(yaw), np.zeros_like(yaw)], axis=1)
    y_b = np.cross(z_b, x_c)
    y_b /= np.linalg.norm(y_b, axis=1)[:, None]
    x_b = np.cross(y_b, z_b)
    return np.stack([x_b, y_b, z_b], axis=2)


def log_so3(R):
    """Rotation vectors of rotation matrices, shape (N, 3)."""
    cos_a = np.clip((np.trace(R, axis1=1, axis2=2) - 1) / 2, -1.0, 1.0)
    angle = np.arccos(cos_a)
    w = np.stack([R[:, 2, 1] - R[:, 1, 2], R[:, 0, 2] - R[:, 2, 0], R[:, 1, 0] - R[:, 0, 1]], axis=1)
    small = angle < 1e-7
    scale = np.where(small, 0.5, angle / (2 * np.sin(np.where(small, 1.0, angle))))
    return w * scale[:, None]


def rot_to_quat(R):
    """Hamilton quaternions (w, x, y, z) of rotation matrices, shape (N, 4)."""
    q = np.empty((len(R), 4))
    for i, m in enumerate(R):
        tr = np.trace(m)
        if tr > 0:
            s = 2 * math.sqrt(tr + 1)
            q[i] = [0.25 * s, (m[2, 1] - m[1, 2]) / s, (m[0, 2] - m[2, 0]) / s, (m[1, 0] - m[0, 1]) / s]
        else:
            j = int(np.argmax(np.diag(m)))
            k, l = (j + 1) % 3, (j + 2) % 3
            s = 2 * math.sqrt(1 + m[j, j] - m[k, k] - m[l, l])
            v = np.zeros(3)
            v[j] = 0.25 * s
            v[k] = (m[k, j] + m[j, k]) / s
            v[l] = (m[l, j] + m[j, l]) / s
            q[i] = [(m[l, k] - m[k, l]) / s, v[0], v[1], v[2]]
    q[q[:, 0] < 0] *= -1
    return q


def rot_x(a):
    return np.array([[1, 0, 0], [0, math.cos(a), -math.sin(a)], [0, math.sin(a), math.cos(a)]])


def rot_y(a):
    return np.array([[math.cos(a), 0, math.sin(a)], [0, 1, 0], [-math.sin(a), 0, math.cos(a)]])


# ---------------------------------------------------------------- camera

def project(pc, cam):
    """Pixel coordinates of camera-frame points (N, 3). Returns uv and a validity mask."""
    x, y, z = pc[:, 0], pc[:, 1], pc[:, 2]
    rho = np.sqrt(x * x + y * y)
    theta = np.arctan2(rho, z)                     # angle from the optical axis
    valid = (z > 0) & (theta < math.radians(cam["max_angle_deg"]))
    a, b = x / np.where(z > 0, z, 1), y / np.where(z > 0, z, 1)
    d = cam["distortion"]
    if cam["model"] == "equidistant":
        th_d = theta * (1 + d[0] * theta ** 2 + d[1] * theta ** 4 + d[2] * theta ** 6 + d[3] * theta ** 8)
        r = np.sqrt(a * a + b * b)
        s = np.where(r > 1e-12, th_d / np.where(r > 1e-12, r, 1), 1.0)
        xd, yd = a * s, b * s
    else:  # radial-tangential
        r2 = a * a + b * b
        radial = 1 + d[0] * r2 + d[1] * r2 * r2
        xd = a * radial + 2 * d[2] * a * b + d[3] * (r2 + 2 * a * a)
        yd = b * radial + d[2] * (r2 + 2 * b * b) + 2 * d[3] * a * b
    u = cam["fx"] * xd + cam["cx"]
    v = cam["fy"] * yd + cam["cy"]
    valid &= (u >= 0) & (u < cam["width"]) & (v >= 0) & (v < cam["height"])
    return np.stack([u, v], axis=1), valid


def camera_setup(model):
    if model == "equidistant":  # about 120-130 deg, like the IMX219 wide lens
        return dict(model="equidistant", fx=340.0, fy=340.0, cx=320.0, cy=240.0,
                    distortion=[0.02, -0.01, 0.003, -0.0005], max_angle_deg=65.0, width=640, height=480)
    # pinhole with moderate distortion, about 90 deg
    return dict(model="radial-tangential", fx=320.0, fy=320.0, cx=320.0, cy=240.0,
                distortion=[-0.25, 0.06, 0.0005, -0.0003], max_angle_deg=50.0, width=640, height=480)


# ---------------------------------------------------------------- output helpers

def write_csv(path, header, rows, fmt):
    with open(path, "w") as f:
        f.write(header + "\n")
        np.savetxt(f, rows, fmt=fmt, delimiter=",")


def write_yaml(path, data):
    with open(path, "w") as f:
        yaml.safe_dump(data, f, sort_keys=False, default_flow_style=None)


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser(description="Generate a synthetic VIO session (EuRoC layout).")
    ap.add_argument("out", help="output folder, e.g. ~/datasets/synthetic/fig8")
    ap.add_argument("--scenario", choices=sorted(SCENARIOS), default="figure8")
    ap.add_argument("--speed", type=float, default=10.0, help="figure8 average speed, approach_hover cruise speed, m/s")
    ap.add_argument("--laps", type=int, default=1, help="figure8 laps")
    ap.add_argument("--radius", type=float, default=150.0, help="figure8 size, m")
    ap.add_argument("--height", type=float, default=30.0, help="height above ground, m")
    ap.add_argument("--height-change", type=float, default=5.0, help="figure8 height variation, m")
    ap.add_argument("--duration", type=float, default=60.0, help="hover duration, s (hover, approach_hover)")
    ap.add_argument("--camera", choices=["equidistant", "radial-tangential"], default="equidistant")
    ap.add_argument("--imu-rate", type=float, default=200.0)
    ap.add_argument("--cam-rate", type=float, default=30.0)
    ap.add_argument("--density", type=float, default=0.02, help="landmarks per m^2")
    ap.add_argument("--pixel-noise", type=float, default=0.5, help="pixels, 1 sigma")
    ap.add_argument("--no-noise", action="store_true", help="perfect IMU and pixels, zero biases")
    ap.add_argument("--td", type=float, default=0.0,
                    help="camera-IMU time offset td = t_imu - t_cam, ms: camera stamps are written td earlier")
    ap.add_argument("--track-age", type=int, default=0,
                    help="re-detect a feature (new id) after this many frames, 0 = never")
    ap.add_argument("--drift-q", type=float, default=0.0,
                    help="track drift: random walk variance per deg of view change, px^2 per axis")
    ap.add_argument("--still-time", type=float, default=0.0, help="liftoff: motors off on the ground, s")
    ap.add_argument("--ground-time", type=float, default=7.0, help="liftoff: motors running on the ground, s")
    ap.add_argument("--ground-height", type=float, default=0.0,
                    help="liftoff: camera height on the ground, m (0: drawn per seed, 0.05-0.2)")
    ap.add_argument("--vibration", type=float, default=1.0, help="liftoff: accel vibration, m/s^2 per sample")
    ap.add_argument("--vibration-offset", type=float, default=0.015, help="liftoff: rectified vibration, m/s^2")
    ap.add_argument("--vibration-gyro", type=float, default=0.03, help="liftoff: gyro vibration, rad/s per sample")
    ap.add_argument("--blind-below", type=float, default=0.0,
                    help="liftoff: no observations while the camera is lower than this, m")
    ap.add_argument("--landed-rate", type=float, default=2.0, help="liftoff: landed flag rate, Hz")
    ap.add_argument("--landed-clear", type=float, default=-0.3,
                    help="liftoff: the landed flag clears this many s after the liftoff (negative: before)")
    ap.add_argument("--climb-speed", type=float, default=2.0, help="liftoff: climb speed, m/s")
    ap.add_argument("--climb-step", type=float, default=1.0,
                    help="liftoff: time to reach (and to leave) the climb speed, s")
    ap.add_argument("--manoeuvre", choices=["none", "speed", "sturn"], default="none",
                    help="line, loop: speed changes or S-turns")
    ap.add_argument("--manoeuvre-period", type=float, default=40.0, help="line, loop: period, s")
    ap.add_argument("--baro", action="store_true", help="also write a simulated baro (mav0/baro0), see above")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--force", action="store_true", help="overwrite the output folder")
    args = ap.parse_args()

    out = os.path.expanduser(args.out)
    if os.path.exists(out):
        if not args.force:
            sys.exit(f"error: {out} exists, use --force to overwrite")
        shutil.rmtree(out)
    rng = np.random.default_rng(args.seed)
    is_liftoff = args.scenario == "liftoff"
    if is_liftoff and args.ground_height <= 0:  # own stream: the other noise stays as without it
        args.ground_height = float(np.random.default_rng([args.seed, 3]).uniform(0.05, 0.2))
    td_ns = round(args.td * 1e6)
    traj = SCENARIOS[args.scenario]
    duration = {"figure8": figure8_duration, "approach_hover": approach_hover_duration,
                "liftoff": liftoff_duration}.get(args.scenario, lambda a: a.duration)(args)
    t0_ns = 1_000_000_000  # start at 1 s, not 0

    # IMU and ground truth at the IMU rate.
    n_imu = int(duration * args.imu_rate) + 1
    t = np.arange(n_imu) / args.imu_rate
    p, v, a, yaw = traj(t, args)
    R = attitude(a, yaw)
    h = 1e-4  # central difference for the body rates
    R_minus = attitude(traj(t - h, args)[2], traj(t - h, args)[3])
    R_plus = attitude(traj(t + h, args)[2], traj(t + h, args)[3])
    gyro = log_so3(np.einsum("nji,njk->nik", R_minus, R_plus)) / (2 * h)
    accel = np.einsum("nji,nj->ni", R, a - GRAVITY)  # R^T (a - g)

    # IMU noise (config/vio.yaml placeholders) and bias random walk.
    noise = dict(gyroscope_noise_density=0.004, gyroscope_random_walk=0.0002,
                 accelerometer_noise_density=0.04, accelerometer_random_walk=0.002)
    dt = 1 / args.imu_rate
    if args.no_noise:
        bg = np.zeros((n_imu, 3))
        ba = np.zeros((n_imu, 3))
    else:
        bg = rng.normal(0, 0.003, 3) + np.cumsum(rng.normal(0, noise["gyroscope_random_walk"] * math.sqrt(dt), (n_imu, 3)), axis=0)
        ba = rng.normal(0, 0.05, 3) + np.cumsum(rng.normal(0, noise["accelerometer_random_walk"] * math.sqrt(dt), (n_imu, 3)), axis=0)
        gyro = gyro + bg + rng.normal(0, noise["gyroscope_noise_density"] / math.sqrt(dt), (n_imu, 3))
        accel = accel + ba + rng.normal(0, noise["accelerometer_noise_density"] / math.sqrt(dt), (n_imu, 3))
    if is_liftoff:  # vibration while the motors run, unknown to the filter (own random stream)
        rng_vib = np.random.default_rng([args.seed, 2])
        on = (t >= args.still_time)[:, None]
        offset = rng_vib.normal(0, 1, 3)
        offset *= args.vibration_offset / np.linalg.norm(offset)
        accel = accel + on * (offset + rng_vib.normal(0, args.vibration, (n_imu, 3)))
        gyro = gyro + on * rng_vib.normal(0, args.vibration_gyro, (n_imu, 3))

    # Camera mounting: nominal down-looking rotation with a small misalignment, 3 cm behind and 5 cm
    # below the IMU.
    cam = camera_setup(args.camera)
    R_bc = R_BODY_CAM_NOMINAL @ rot_x(math.radians(1.0)) @ rot_y(math.radians(-0.7))
    t_bc = np.array([-0.03, 0.0, 0.05])

    # Landmarks: mostly on the ground (z = 0), 10 % on objects up to 8 m high.
    margin = min(150.0, args.height * math.tan(math.radians(70)))
    lo, hi = p[:, :2].min(axis=0) - margin, p[:, :2].max(axis=0) + margin
    n_lm = int(args.density * np.prod(hi - lo))
    lm = np.zeros((n_lm, 3))
    lm[:, :2] = rng.uniform(lo, hi, (n_lm, 2))
    tall = rng.random(n_lm) < 0.1
    lm[tall, 2] = -rng.uniform(0, 8, tall.sum())
    if is_liftoff:  # texture around the takeoff spot, denser towards it (own random stream)
        rng_near = np.random.default_rng([args.seed, 4])
        r = 0.02 * (20.0 / 0.02) ** rng_near.uniform(0, 1, 400)
        phi = rng_near.uniform(0, 2 * math.pi, 400)
        lm = np.vstack([lm, np.stack([r * np.cos(phi), r * np.sin(phi), np.zeros(400)], axis=1)])
        n_lm = len(lm)

    # Camera observations.
    n_cam = int(duration * args.cam_rate) + 1
    tc = np.arange(n_cam) / args.cam_rate
    pc_, _, ac_, yc_ = traj(tc, args)
    Rc = attitude(ac_, yc_)
    obs = []
    # Only landmarks within 250 m are visible: look at the ones within 250 m in x (landmarks sorted
    # by x once). Same result as checking all of them, much faster for km-long flights.
    by_x = np.argsort(lm[:, 0], kind="stable")
    xs = lm[by_x, 0]
    tracks = args.track_age > 0 or args.drift_q > 0
    rng_drift = np.random.default_rng([args.seed, 1])  # own stream: the other noise stays the same
    drift = np.zeros((n_lm, 2))
    prev_ray = np.zeros((n_lm, 3))
    last_frame = np.full(n_lm, -2)
    age = np.zeros(n_lm, dtype=np.int64)
    n_tracks = np.zeros(n_lm, dtype=np.int64)  # tracks so far per landmark
    for k in range(n_cam):
        R_wc = Rc[k] @ R_bc
        p_wc = pc_[k] + Rc[k] @ t_bc
        tk = t0_ns + round(tc[k] * 1e9) - td_ns  # camera clock: td = t_imu - t_cam
        if is_liftoff and -p_wc[2] < args.blind_below:  # too close to the ground to see anything
            obs.append(np.array([[tk, -1, 0.0, 0.0]]))   # id -1: a frame without features
            continue
        lo, hi = np.searchsorted(xs, [p_wc[0] - 250.0, p_wc[0] + 250.0])
        cand = np.sort(by_x[lo:hi])         # landmark ids, ascending as before
        pts = (lm[cand] - p_wc) @ R_wc      # rows: R_wc^T (P - p_wc)
        near = np.linalg.norm(pts, axis=1) < 250.0
        uv, ok = project(pts, cam)
        ok &= near
        ids = cand[ok]
        if len(ids) == 0:
            if is_liftoff:
                obs.append(np.array([[tk, -1, 0.0, 0.0]]))
            continue
        uvk = uv[ok]
        if not args.no_noise:
            uvk = uvk + rng.normal(0, args.pixel_noise, uvk.shape)
        fid = ids
        if tracks:
            ray = lm[ids] - p_wc
            ray /= np.linalg.norm(ray, axis=1)[:, None]
            go_on = last_frame[ids] == k - 1
            if args.track_age > 0:
                go_on &= age[ids] < args.track_age
            new = ids[~go_on]
            n_tracks[new] += last_frame[new] >= 0
            drift[new] = 0
            age[new] = 0
            cont = ids[go_on]
            step = np.degrees(np.arccos(np.clip(np.sum(ray[go_on] * prev_ray[cont], axis=1), -1, 1)))
            drift[cont] += rng_drift.normal(0, 1, (len(cont), 2)) * np.sqrt(args.drift_q * step)[:, None]
            prev_ray[ids] = ray
            last_frame[ids] = k
            age[ids] += 1
            uvk = uvk + drift[ids]
            fid = ids + n_lm * n_tracks[ids]
        obs.append(np.column_stack([np.full(len(ids), tk), fid, uvk]))
    obs = np.vstack(obs) if obs else np.zeros((0, 4))

    # Write everything.
    for d in ("mav0/imu0", "mav0/cam0", "mav0/state_groundtruth_estimate0"):
        os.makedirs(os.path.join(out, d), exist_ok=True)
    t_ns = t0_ns + np.round(t * 1e9).astype(np.int64)

    write_csv(os.path.join(out, "mav0/imu0/data.csv"),
              "#timestamp [ns],w_RS_S_x [rad s^-1],w_RS_S_y [rad s^-1],w_RS_S_z [rad s^-1],"
              "a_RS_S_x [m s^-2],a_RS_S_y [m s^-2],a_RS_S_z [m s^-2]",
              np.column_stack([t_ns, gyro, accel]), ["%d"] + ["%.9f"] * 6)
    write_yaml(os.path.join(out, "mav0/imu0/sensor.yaml"),
               dict(sensor_type="imu", comment="synthetic", rate_hz=args.imu_rate,
                    T_BS=dict(cols=4, rows=4, data=np.eye(4).ravel().tolist()), **noise))

    q = rot_to_quat(R)
    write_csv(os.path.join(out, "mav0/state_groundtruth_estimate0/data.csv"),
              "#timestamp, p_RS_R_x [m], p_RS_R_y [m], p_RS_R_z [m], q_RS_w [], q_RS_x [], q_RS_y [], "
              "q_RS_z [], v_RS_R_x [m s^-1], v_RS_R_y [m s^-1], v_RS_R_z [m s^-1], b_w_RS_S_x [rad s^-1], "
              "b_w_RS_S_y [rad s^-1], b_w_RS_S_z [rad s^-1], b_a_RS_S_x [m s^-2], b_a_RS_S_y [m s^-2], "
              "b_a_RS_S_z [m s^-2]",
              np.column_stack([t_ns, p, q, v, bg, ba]), ["%d"] + ["%.9f"] * 16)

    T_bc = np.eye(4)
    T_bc[:3, :3], T_bc[:3, 3] = R_bc, t_bc
    write_csv(os.path.join(out, "mav0/cam0/features.csv"), "#timestamp [ns],feature_id,u [px],v [px]",
              obs, ["%d", "%d", "%.4f", "%.4f"])
    write_yaml(os.path.join(out, "mav0/cam0/sensor.yaml"),
               dict(sensor_type="camera", comment="synthetic, features only", rate_hz=args.cam_rate,
                    resolution=[cam["width"], cam["height"]], camera_model="pinhole",
                    intrinsics=[cam["fx"], cam["fy"], cam["cx"], cam["cy"]],
                    distortion_model=cam["model"], distortion_coefficients=cam["distortion"],
                    T_BS=dict(cols=4, rows=4, data=[float(x) for x in T_bc.ravel()])))

    if args.baro:  # simulated baro, a guess (see the top of this file)
        rng_baro = np.random.default_rng([args.seed, 5])
        k = np.arange(0, n_imu, int(round(args.imu_rate / 20)))  # 20 Hz
        height = -p[k, 2]
        motors = t[k] >= args.still_time if is_liftoff else np.ones(len(k), bool)
        thrust_excess = np.linalg.norm(a[k] - GRAVITY, axis=1) - np.linalg.norm(GRAVITY)
        alt = (300.0 + height
               + np.cumsum(rng_baro.normal(0, 0.03 * math.sqrt(1 / 20), len(k)))   # bias drift
               + rng_baro.normal(0, 0.3, len(k))                                      # white noise
               - 0.2 * thrust_excess * motors                                         # prop wash
               - 0.01 * np.sum(v[k, :2] ** 2, axis=1)                                 # speed term
               + 0.5 * np.exp(-height / 0.5) * motors)                                # ground effect
        os.makedirs(os.path.join(out, "mav0/baro0"), exist_ok=True)
        write_csv(os.path.join(out, "mav0/baro0/data.csv"), "#timestamp [ns],altitude [m]",
                  np.column_stack([t_ns[k], alt]), ["%d", "%.3f"])

    if is_liftoff:  # FC landed flag (EXTENDED_SYS_STATE.landed_state), in the IMU clock
        t_clear = liftoff_times(args)[0] + args.landed_clear
        tf = np.arange(0.0, duration, 1.0 / args.landed_rate)
        os.makedirs(os.path.join(out, "mav0/landed0"), exist_ok=True)
        write_csv(os.path.join(out, "mav0/landed0/data.csv"), "#timestamp [ns],landed_state",
                  np.column_stack([t0_ns + np.round(tf * 1e9).astype(np.int64), np.where(tf < t_clear, 1, 2)]),
                  ["%d", "%d"])

    write_csv(os.path.join(out, "landmarks.csv"), "#id,x [m],y [m],z [m]",
              np.column_stack([np.arange(n_lm), lm]), ["%d", "%.4f", "%.4f", "%.4f"])
    write_yaml(os.path.join(out, "session.yaml"),
               dict(source="tools/gen_synthetic.py", world_frame="NED", gravity_world=GRAVITY.tolist(),
                    settings={k: v for k, v in vars(args).items() if k not in ("out", "force")}))

    length = float(np.sum(np.linalg.norm(np.diff(p, axis=0), axis=1)))
    tilt = np.degrees(np.arccos(np.clip(R[:, 2, 2], -1, 1)))
    per_frame = len(obs) / n_cam
    print(f"wrote {out}")
    print(f"  scenario {args.scenario}, {duration:.1f} s, path {length:.0f} m, "
          f"speed max {np.linalg.norm(v, axis=1).max():.1f} m/s, tilt max {tilt.max():.1f} deg")
    print(f"  imu {n_imu} samples, camera {n_cam} frames, {n_lm} landmarks, {per_frame:.0f} features per frame")


if __name__ == "__main__":
    main()
