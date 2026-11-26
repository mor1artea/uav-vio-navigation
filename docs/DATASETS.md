# Data for the offline work (Stage 1a)

The algorithm (front end, filter, quality module, metrics) is developed on the laptop. Three kinds of
data, in this order:

1. Synthetic data from our own small generator: known ground truth, available now.
2. One EuRoC sequence as the known-good real reference.
3. Later: our own recordings (Stage 1b) and the Gazebo simulation (SIMULATION.md).

Everything lives outside the repo, in `~/datasets`. This document also says, just as important,
what each kind of data can and cannot tell us about our drone.

## 1. Synthetic data (first)

A small generator in `tools/` writes a session in the same layout the replay reads:

- a smooth trajectory with known pose, velocity and acceleration at any time (for example a flight
  at 20-50 m over a ground plane, with turns, speed changes, hover and tilts up to 45 deg, plus a
  closed loop);
- IMU at 200 Hz from the exact derivatives, with white noise and bias random walk, and optionally the
  "FC IMU" degradation (separate gyro and accel delays, low-pass, send-time jitter, gaps);
- camera observations at 30 Hz: random 3D points, mostly on the ground plane, projected through the
  camera model (pinhole-radtan or Kannala-Brandt) with pixel noise and track lifetimes. No images:
  the generator produces feature tracks directly, so it tests everything after the front end.
  Optionally like a tracker (2026-11-26): features re-detected after `--track-age` frames, and
  KLT-like drift with `--drift-q` (a random walk per degree of view change with a known q, DESIGN.md
  section 13.4 a). Real KLT drift does not follow this model everywhere (in hover it grows more
  slowly), so these sessions test the filter's drift handling, not the tracker;
- a liftoff scenario (2026-11-26, `--scenario liftoff`): armed on the ground with the motors
  running, the camera 0.05-0.2 m above the ground, then a climb to 25 m and straight flight. The
  vibration is a model, not measured: white accel noise of 1 m/s^2 per sample (EuRoC shows a spread
  of 0.3-1.8 m/s^2 on the ground, but not white), 0.015 m/s^2 of rectified offset (about what the
  EuRoC velocity drift on the ground suggests) and 0.03 rad/s gyro noise. Extra points around the
  takeoff spot, a variant without any features below 1 m (`--blind-below`), and the FC landed flag
  (`mav0/landed0/data.csv`) with a chosen rate and clear time. Frames without features are written
  as a row with feature id -1;
- a simulated baro (2026-11-26, `--baro`, `mav0/baro0/data.csv`, 20 Hz). The model is a guess, not
  measured: true height plus a constant offset, white noise 0.3 m, a bias random walk of 0.03 m per
  sqrt(s), prop wash -0.2 m per m/s^2 of thrust above hover, a speed term -0.01 m per (m/s)^2 of
  horizontal speed and ground effect +0.5 m * exp(-height / 0.5 m). Real baro behaviour (DPS310 on
  our FC, in prop wash, at speed, near the ground) comes from our recordings. The speed term turned
  out to matter for the scale (DESIGN.md section 11, stage 4);
- km-scale sessions (2026-11-26, `--scenario line` or `loop`): a straight leg or a circle at
  constant speed, with `--manoeuvre speed` (+-20 % every `--manoeuvre-period` s) or `sturn` (30 m
  lateral S-turns), for the scale drift;
- ground truth in the same format as EuRoC;
- optionally images: `tools/render_ground.py` renders a flat textured ground for a generated
  session, so the front end can be checked in our geometry (section 5).

The metrics tool is tested on trajectories with exactly known errors (shifted, rotated, scaled,
drifting copies of the ground truth), before it is used on any estimator output.

## 2. EuRoC MAV, one sequence (download by hand)

Source: ETH Research Collection, https://doi.org/10.3929/ethz-b-000690084, ASL format (csv and png
folders), not the rosbags. One easy sequence is enough as the known-good reference. We use
`MH_01_easy` (Machine Hall, 1.57 GB zip). Only the full zip can be downloaded;
`tools/prepare_euroc.sh` extracts just the parts below (about 1.3 GB), the rest stays in the zip:

| Path inside the sequence | What it is | Extracted |
|---|---|---|
| `mav0/cam0/data.csv`, `mav0/cam0/data/*.png`, `mav0/cam0/sensor.yaml` | left camera images, time stamps, intrinsics and camera-IMU extrinsics | yes |
| `mav0/imu0/data.csv`, `mav0/imu0/sensor.yaml` | IMU at 200 Hz and its noise values | yes |
| `mav0/state_groundtruth_estimate0/data.csv` | ground truth pose, velocity and IMU biases | yes, the metrics need it |
| `mav0/body.yaml` | only a comment naming the MAV | yes (tiny) |
| `mav0/cam1`, `mav0/leica0` | right camera, raw Leica positions. The other sequences have `vicon0` and `pointcloud0` instead of `leica0` | no |

Layout expected by the replay:

```
~/datasets/euroc/MH_01_easy/mav0/cam0/data.csv
~/datasets/euroc/MH_01_easy/mav0/cam0/data/<timestamp>.png
~/datasets/euroc/MH_01_easy/mav0/cam0/sensor.yaml
~/datasets/euroc/MH_01_easy/mav0/imu0/data.csv
~/datasets/euroc/MH_01_easy/mav0/imu0/sensor.yaml
~/datasets/euroc/MH_01_easy/mav0/state_groundtruth_estimate0/data.csv
```

`tools/prepare_euroc.sh` unpacks the zip into this layout, or checks a folder that is already
unpacked: required files present, image count equal to the rows in `cam0/data.csv`.

### MH_01_easy, checked 2026-11-24

- 184.1 s of IMU (36820 samples, 200 Hz, no gaps), 3682 images at 20 Hz, 752x480,
  radial-tangential. Ground truth from 1.08 s to 183.0 s (times from the first IMU sample), 80.6 m
  of path.
- World frame and gravity: z up, gravity (0, 0, -9.81), as the session reader assumes. Checked
  directly: ground truth acceleration (from the ground truth velocity) minus the IMU specific force
  rotated by the ground truth attitude gives a median gravity of (-0.003, 0.001, -9.818) m/s^2. The
  ground truth pose is the IMU (body) pose (`T_BS` is identity).
- What happens in the sequence: 1-20 s the drone is carried and moved by hand (no motor vibration),
  20-38 s it stands still on the floor, then the motors spin up and it takes off at about 44 s,
  flies until the end of the ground truth at 183.0 s and is caught or set down at 183.0-183.5 s
  (4 rad/s and a 12 m/s^2 shock in the IMU), after the ground truth has already stopped.
- How the metrics handle that: runs start in the still part with `vio_replay --start 22`, which is
  also how our filter starts (static, on the ground). The hand-carried part is skipped, the 22 s of
  standing still and the takeoff stay in. The end needs nothing: the metrics only use estimate rows
  inside the ground truth time range, and the ground truth ends before the touchdown shock. On a
  sequence this short (80 m) the percentage grades mean little; look at the metres.
- IMU only, from the ground truth state including its biases (`--mode imu`):

  | Start | after 1 s | 2 s | 5 s | 10 s | 20 s |
  |---|---|---|---|---|---|
  | 0 s (hand-carried) | 0.02 m | 0.07 m | 0.28 m | 0.59 m | 2.7 m |
  | 22 s (standing still) | 0.01 m | 0.04 m | 0.32 m | 2.1 m | 15.7 m |
  | 44 s (takeoff) | 0.03 m | 0.10 m | 0.54 m | 1.4 m | 5.9 m |

  Maximum position error (navigation ATE max). It does not run away at once, so frames and gravity
  are right. The growth is the expected cubic drift from small bias errors: the ground truth gyro
  bias differs from the gyro average over the still part by 1.4e-3 rad/s, which alone explains
  most of the 15.7 m after 20 s. The filter will estimate the biases itself.

### Front end on MH_01_easy (step 2, 2026-11-24)

`vio_track --no-sky-mask --start 22` (cam0 looks forward, so no sky mask), 3242 frames,
`eval_tracks.py` with the ground truth. These runs had CLAHE on (it is off by default now, see the
last column).

| | gyro on, 20 Hz | gyro off, 20 Hz | gyro on, 6.7 Hz | gyro off, 6.7 Hz | CLAHE off, 20 Hz |
|---|---|---|---|---|---|
| features per frame | 147 | 147 | 146 | 146 | 147 |
| success rate | 0.988 | 0.988 | 0.949 | 0.918 | 0.988 |
| track length, median / p90 [frames] | 32 / 216 | 31 / 212 | 10 / 62 | 8 / 42 | 34 / 208 |
| forward-backward error, median / p90 [px] | 0.003 / 0.016 | 0.003 / 0.016 | 0.007 / 0.078 | 0.008 / 0.097 | 0.002 / 0.014 |
| tracked minus predicted position, median [px] | 1.1 | 3.0 | 3.4 | 8.7 | 1.1 |
| grid cells occupied (of 48) | 39.7 | 39.6 | 43.0 | 44.2 | 39.0 |
| wrong tracks over 10 frames, > 2 px / > 5 px | 2.5 / 0.5 % | 2.4 / 0.5 % | 6.5 / 1.7 % | 8.4 / 3.5 % | 2.2 / 0.5 % |
| wrong whole tracks, > 2 px / > 5 px | 36 / 15 % | 36 / 16 % | 32 / 11 % | 29 / 12 % | 35 / 14 % |

- "6.7 Hz" is every 3rd image, three times the motion between frames.
- "Wrong": the RMS reprojection error after triangulating the track from the ground truth poses is
  above 2 px (or 5 px). On 5-frame pieces the median error is 0.10 px, so ground truth and
  calibration are good to that level, and the growth with length is the tracker's own drift. The
  check was verified on perfect synthetic tracks with 0.5 px noise: median 0.7 px, none above 2 px.
- "Tracked minus predicted": how far KLT had to move from its start position. With the gyro off it
  is the whole image motion; with it on, the part the gyro did not explain (translation, timing and
  extrinsic errors). On our own recordings this number is the first sign of a wrong time offset or
  camera-IMU rotation.

What this says:

- On 10-frame pieces (about the filter's window) 2.5 % of the tracks are wrong, 0.5 % badly. That is
  within what the filter's chi2 test can handle, so no RANSAC in the front end for now.
- Over their whole life (median 32, up to hundreds of frames) a third of the tracks drift above 2 px.
  This is the normal frame-to-frame KLT drift, made worse here by the forward camera moving towards
  the scene (patches grow). It matters for long-lived landmarks (DESIGN.md section 11, open item 1).
- The gyro explains about 60 % of the image motion. On this gentle sequence (under 0.85 rad/s) that
  changes nothing at 20 Hz. At 6.7 Hz it gives a success rate of 0.949 instead of 0.918 and a
  quarter fewer wrong pieces. The synthetic test with a 40 px rotation shows the large case: 100 %
  tracked with the prediction, 26 % without.
- CLAHE does not help: slightly longer tracks and fewer wrong ones without it.

Time per frame on the laptop (AMD Ryzen 7 7435HS, one thread), measured twice back to back: 6.3-6.5
ms without CLAHE, 8.3-8.7 ms with it, 95th percentile 8.4-9.4 ms without (also on the rendered
sequences below). Without CLAHE: preparation
(metrics, pyramid) about 1.4 ms, KLT forward and backward about 4.2 ms, detection 0.4 ms on average
(it runs on about one frame in five, on the half-size image). Laptop times vary by up to 30 %
between sessions (boost clock under WSL), so compare only runs made together.

## 3. Testing the fisheye model without TUM-VI

TUM-VI is dropped. The Kannala-Brandt model is tested in four independent ways instead:

1. Against OpenCV: our projection, unprojection and Jacobians compared with `cv::fisheye::projectPoints`
   and `cv::fisheye::undistortPoints` on random points over the whole field of view, including the
   edges at 60 deg and more. Numeric Jacobians check the analytic ones.
2. Round trips: unproject(project(x)) = x for rays up to the edge of the 120 deg field of view.
3. Synthetic data: the generator projects through Kannala-Brandt, so the whole filter runs on a wide
   field of view with known truth.
4. On the real camera: the reprojection error per image region from the IMX219 calibration
   (CALIBRATION.md step 2), and later the Gazebo wide-angle camera with an equidistant lens.

What this does not cover is a real fisheye image through the real front end before our own camera
is calibrated. That comes with the first recordings (Stage 1b).

## 4. What each kind of data can and cannot validate

| Property | Synthetic | EuRoC | Our drone |
|---|---|---|---|
| Shutter | none (feature tracks) or global (rendered images) | global | rolling, about 12 ms readout |
| Lens | any model we choose | pinhole with radial-tangential distortion, about 80 deg horizontal | about 120 deg, model chosen after calibration |
| Camera direction | down-facing, as we choose | forward on a MAV | straight down, tilting with the drone |
| IMU | ideal or degraded on purpose | ADIS16448, 200 Hz, hardware synchronized | FC filtered, 5-13 ms delay, stamped at send time, some aliasing, possible USB gaps |
| Scene | points on a plane, as we choose; rendered: flat procedural ground and sky | rooms and a machine hall, strong 3D structure, 1-10 m | mostly flat ground 20-50 m away, sky at high tilt |
| Motion and length | anything, including km | slow to moderate, roughly 35-130 m, 2-3 min | 2-5 km at 8-12 m/s, fast FPV rotations |
| Ground truth | exact | Vicon / Leica, mm | external GNSS logger (planned) |

Synthetic data can validate: the filter math (propagation, MSCKF update, triangulation, landmarks),
covariance consistency (NEES), the time offset estimate, the camera model math, scale behaviour on
long legs over a plane, the quality module on injected faults, and the metrics tool. All under the
assumptions of the generator: perfect data association and a noise model we chose ourselves.

EuRoC can validate, on real images and a real IMU: the front end (KLT, forward-backward check, grid),
the full pipeline end to end, and that the filter stays consistent with real, imperfect data.

Neither can validate: the rolling-shutter correction, real blur and exposure, the sky mask on real
images, real vibration and aliasing, our FC IMU path, drift over kilometres on real data, baro and
compass behaviour, latency and CPU on the Pi, and anything with ArduPilot. Those need our own
recordings, the Gazebo simulation (partly) and flights.

## 5. Rendered ground sequences (front end in our geometry)

EuRoC looks forward into rooms. Our camera looks down at mostly flat ground from 20-50 m and tilts
up to 45 deg. `tools/render_ground.py` renders that case for a session from `gen_synthetic.py`:
every pixel is unprojected through the fisheye model, intersected with the ground plane, and the
texture is evaluated there. Texture: fields with sharp borders plus value noise from 64 m down to
0.25 m, finer octaves faded out where a pixel covers more ground (no aliasing). Rays above the
horizon see a bright sky. Sensor noise is added. Rendering takes about 1 s per frame.

Sequences (2026-11-24), both from `gen_synthetic.py ~/datasets/synthetic/fig8_tilt --speed 12
--radius 50 --height 25`: figure eight at 25 m, 306 m in 25.5 s, up to 21 m/s and 43 deg tilt,
Kannala-Brandt lens, 640x480 at 30 Hz, 764 frames, global shutter.

- `fig8_tilt_img`: normal contrast (grey-level standard deviation 25 in the images, noise 2);
- `fig8_tilt_low`: low contrast (standard deviation 7, noise 3), like a uniform field or dusk.

`vio_track` with the sky mask on (attitude from the ground truth, 75 deg); the usable image fraction
is 0.87 on average. CLAHE on in these runs (it changes nothing that matters here, see the last
column).

| | normal, gyro on | normal, gyro off | normal, 10 Hz gyro on / off | low contrast, gyro on | low, gyro off | low, 10 Hz gyro on / off | low, CLAHE off |
|---|---|---|---|---|---|---|---|
| success rate | 0.982 | 0.982 | 0.945 / 0.930 | 0.957 | 0.955 | 0.916 / 0.902 | 0.957 |
| track length median / p90 [frames] | 49 / 108 | 49 / 108 | 17 / 36 | 15 / 61 | 15 / 59 | 11 / 27 | 16 / 62 |
| forward-backward error median / p90 [px] | 0.007 / 0.025 | 0.007 / 0.025 | 0.020 / 0.094 | 0.15 / 0.48 | 0.15 / 0.48 | 0.17 / 0.59 | 0.15 / 0.47 |
| tracked minus predicted, median [px] | 3.8 | 5.6 | 11.7 / 16.8 | 4.1 | 5.7 | 12.1 / 17.2 | 4.1 |
| grid cells occupied / usable | 45.5 / 45.1 | 45.5 / 45.1 | 45.2 / 45.1 | 46.1 / 45.1 | 46.1 / 45.1 | 46.0 / 45.1 | 46.1 / 45.1 |
| wrong over 10 frames, > 2 px / > 5 px | 0.1 / 0.0 % | 0.1 / 0.0 % | 0.1 / 0.1 % vs 0.5 / 0.3 % | 0.1 / 0.0 % | 0.2 / 0.1 % | 0.2 / 0.1 % vs 0.7 / 0.6 % | 0.0 / 0.0 % |
| wrong whole tracks, > 2 px / > 5 px | 0.9 / 0.1 % | 0.9 / 0.1 % | 0.9 / 0.2 % vs 1.7 / 0.7 % | 2.6 / 0.0 % | 2.6 / 0.1 % | 1.5 / 0.1 % vs 2.6 / 1.1 % | 2.1 / 0.0 % |

("Occupied" can exceed "usable": a cell counts as usable when its centre is outside the mask, and
some features sit in cells whose centre is masked.)

What this says:

- In our geometry the share of wrong tracks is 20-40 times lower than on EuRoC: 0.1 % over 10
  frames, about 1 % over whole tracks with normal contrast and 2-3 % with low contrast. Far, flat
  ground changes its appearance slowly, so KLT drifts much less than in a room.
- Low contrast costs track length (median 15 frames instead of 49) and raises the forward-backward
  error about 20 times, but the tracks that pass are still correct. The forward-backward check does
  its job: it drops the uncertain ones instead of keeping wrong ones.
- In this flight the image motion is mostly translation (12-21 m/s at 25 m is 5-10 px per frame), so
  the gyro removes only a third of it. At 30 Hz that makes no difference; at 10 Hz the gyro gives
  fewer lost and fewer wrong tracks.
- Time per frame as on EuRoC: 6.4-6.5 ms mean without CLAHE, 8.4-8.6 ms with it.

How far this goes: the rendered ground is optimistic. No 3D structure, occlusion or parallax
between objects, no moving water or leaves, no repetitive patterns, no lighting or exposure change,
no motion blur, no rolling shutter. It shows that the tracker works in our viewing geometry and with
low contrast; real ground comes with our recordings (Stage 1b).

## 6. Filter results (step 3)

Every number is a median with its range over several runs. Since 2026-11-26: 20 start times for
real images (`fig8_tilt_img`, `fig8_tilt_low` 0, 0.5, ..., 9.5 s; `approach_hover_img` 0, 0.2, ...,
3.8 s; EuRoC 21, 21.25, ..., 25.75 s) or 20 seeds. Before that six runs (renders 0, 0.034, 0.067,
0.1, 0.134, 0.167 s; EuRoC 21.5, 22, 22.05, 22.1, 22.5, 23 s; seeds as given). A start one frame later
changes which frames get the visual update, and single runs differ a lot; six runs were not enough
either (below). Numbers from single runs, as in earlier versions of this section, are not
comparable. Made with `tools/eval_runs.py`, for example:

```
tools/eval_runs.py ~/datasets/synthetic/fig8_tilt_img --starts 0 0.034 0.067 0.1 0.134 0.167 \
    --configs config/vio.yaml config/replay_synthetic.yaml config/replay_render.yaml --out /tmp/eval/tilt
tools/eval_runs.py ~/datasets/euroc/MH_01_easy --starts 21.5 22 22.05 22.1 22.5 23 \
    --configs config/vio.yaml config/replay_euroc.yaml --rpe-length 10 --out /tmp/eval/euroc
```

Settings: right-invariant errors, visual update on every 6th frame, 11 clones, tracker age cap 72
frames, each feature used once, landmarks for hover; EuRoC every 3rd frame, cap 36. Update stride
and age cap are provisional (the comparison below that chose them rests on single runs). Real-image
runs use the tracker inside `vio_replay`. Consistency is honest when the mean NEES is about 3 and
about 99.7 % of the errors are inside 3 sigma.

Results over 20 start times, today's code (2026-11-26: review fixes and moving origin; part a of
DESIGN.md section 13 off, as by default). The second line of each render and EuRoC is the same with
part a switched on (q = 0.05 px^2/deg, measured on the normal render; DESIGN.md section 13.11):

| Data | ATE (navigation) | ATE aligned | other | NEES position | NEES attitude |
|---|---|---|---|---|---|
| rendered `fig8_tilt_img` | 0.17 % (0.11-0.27) | 0.23 m (0.15-0.33) | RPE 0.49 % / 100 m | 3.0 (0.80-11) | 2.4 (1.3-4.7) |
| part a on | 0.18 % (0.11-0.28) | 0.24 m (0.15-0.37) | RPE 0.48 % / 100 m | 2.2 (0.81-4.2) | 1.7 (1.0-2.9) |
| rendered `fig8_tilt_low` | 0.45 % (0.14-0.80) | 0.48 m (0.25-1.19) | RPE 1.03 % / 100 m | 10 (2.0-28) | 7.9 (3.8-16) |
| part a on | 0.38 % (0.16-0.47) | 0.44 m (0.27-0.66) | RPE 0.79 % / 100 m | 4.9 (1.2-8.5) | 3.8 (2.1-12) |
| rendered `approach_hover_img` | 0.57 % (0.28-1.25) | 0.16 m (0.10-0.24) | hover 0.22 m/min (0.05-0.66), window 12-60 s; 0.40 m/min over 12-69 s | 4.6 (1.5-13) | 3.5 (1.5-7.3) |
| part a on | 0.85 % (0.47-1.47) | 0.24 m (0.15-0.43) | hover 0.67 m/min (0.14-1.19) | 5.4 (3.0-13) | 1.8 (0.95-2.5) |
| EuRoC `MH_01_easy` (73 m), td online | 0.72 % (0.25-3457) | 0.21 m (0.11-1754) | RPE 1.8 % / 10 m | 71 (8.1-1654) | 40 (33-443) |
| part a on | 0.66 % (0.26-2831) | 0.20 m (0.13-1521) | RPE 1.7 % / 10 m | 59 (7.5-1150) | 37 (32-234) |
| EuRoC, td fixed at 0 | 0.70 % (0.23-1.63) | 0.20 m (0.11-0.37) | RPE 1.8 % / 10 m | 69 (7.4-281) | 42 (33-69) |
| part a on | 0.73 % (0.25-517) | 0.21 m (0.13-253) | RPE 1.8 % / 10 m | 70 (6.4-362) | 37 (33-92) |

- EuRoC: start 25.75 diverges with online td (2.5 km), with part a off and on; with td fixed at 0
  none does with part a off, start 22.25 does with it on. EuRoC is reported, not graded.
- EuRoC divergence (2026-11-26): with starts every 0.05 s, 9 of 61 runs between 21 and 24 s and 9 of
  61 between 24 and 27 s end above 1 m aligned ATE. Cause, shown run by run: about 7 s on the IMU
  alone while the rotors spin on the floor (no zero-velocity update, no parallax), then at liftoff
  tracks only 0.2 m away whose depth is as small as the pose error; a few pass the per-track gate,
  one inconsistent update moves the accel bias by 0.4-0.7 m/s^2, and later tracks are all
  rejected. Not a coding error; the options need your decision (DESIGN.md section 11).
- With the joint gate (B, default since 2026-11-26), 121 starts 21-27 s: 2 above 1 m (22.35, 25.45)
  instead of 18, 90th percentile of the aligned ATE 0.24 m instead of 1.60, worst 3.8 m instead of
  1830, median 0.21 m. The two left go wrong through updates that pass every gate (DESIGN.md
  section 11). The table above is from before B.
- Liftoff (synthetic, 20 seeds, DESIGN.md section 11): aligned ATE with features near the ground
  10.3 m today, 8.0 m with B, 1.5 m with B and the landed zero-velocity update (C); without
  features below 1 m 452 m, 23 m and 2.0 m. Velocity error at liftoff 0.8-1.6 m/s before, about
  0.1 m/s with C.
- Synthetic tracks with KLT-like drift (`gen_synthetic.py --track-age 72 --drift-q q`, figure8, 20
  seeds): with part a off the filter is overconfident from q = 0.03 px^2/deg on (position NEES 9.4;
  59 at q = 0.1); with part a on and the true q, position NEES 2.5-3.1 for q = 0 to 0.3 and ATE 0.31
  instead of 1.4 % at q = 0.1 (DESIGN.md section 13.11). Approach and hover with drift (landmarks,
  q = 0.05, 20 seeds): part a off position NEES 12 and hover drift 0.81 m/min, on 2.7 and 0.35 m/min.
- How much a median over 20 starts moves by chance: pixel noise 0.999 / 1.000 / 1.001 px gives
  position NEES medians 8.9 / 10.5 / 9.7 on the low-contrast render (six starts: 20.5 / 18 / 11,
  review of 2026-11-26), 3.0-3.1 on the normal render and 4.6-5.0 on the hover render.
- Low-contrast render and the moving origin (correction, 2026-11-26): the previous version of this
  section said the render "got worse" with the moving origin, from six starts (12 to 18). That is
  not a result. Over the 20 starts the binary from before the moving origin gives 8.7 (2.8-27), the
  current one 10.5 (2.0-28), no more apart than the pixel noise change above moves it on its own.

Results with the moving origin over six start times (2026-11-25, kept for the record), the previous
stage (review fixes) in brackets:

| Data | ATE (navigation) | ATE aligned | other | NEES position | NEES attitude |
|---|---|---|---|---|---|
| synthetic `figure8`, 8 seeds | 0.15 % (0.10-0.24) [0.16] | 0.81 m (0.57-1.71) | RPE 0.74 % / 100 m | 3.0 (1.7-4.9) [3.1] | 2.4 (2.2-3.5) [2.4] |
| synthetic `approach_hover`, 20 seeds, not rerun | 0.26 % (0.13-0.70) | 0.09 m | hover 0.15 m/min (0.07-1.03) | 1.5 (0.5-6.0) | 1.9 (1.0-3.7) |
| rendered `fig8_tilt_img` | 0.18 % (0.12-0.24) [0.18] | 0.28 m (0.22-0.32) | RPE 0.46 % / 100 m | 5.4 (3.4-9.5) [5.5 (4.1-9.5)] | 3.9 (3.0-6.4) [3.9] |
| rendered `fig8_tilt_low` | 0.45 % (0.18-0.77) [0.36] | 0.61 m (0.30-1.19) | RPE 1.06 % / 100 m | 18 (4.0-47) [12 (5.7-17)] | 15 (4.1-30) [11] |
| rendered `approach_hover_img` | 0.62 % (0.54-1.08) [0.62] | 0.21 m (0.15-0.27) | hover 0.32 m/min (0.24-0.53) [0.32] | 4.0 (2.5-9.4) [4.0] | 4.7 (1.6-7.3) [4.8] |
| EuRoC `MH_01_easy` (73 m) | 0.60 % (0.25-0.70) [0.62] | 0.18 m (0.11-0.21) [0.19] | RPE 1.6 % / 10 m | 51 (8.4-64) [52 (6.0-66)] | 37 (33-40) [37] |

- Low contrast 18 against 12: six starts are too few on this render, see the correction above.
  The flight stays within about 50 m of the start, so the origin never moves there.
- EuRoC with the camera stamps moved 5 ms or 2.5 ms earlier and td fixed at that value gives exactly
  the numbers of the original data with td fixed at 0 (review-fix stage).
- Online td: on the renders (true td 0) it ends at -3.3 ms (normal contrast, sigma 1.9 ms), -1.8 ms
  (low contrast) and +1.5 ms (hover). The -3.3 ms is not investigated (your decision).
- Time: filter about 0.5-2 ms per update frame, tracker about 5-7 ms per camera frame including PNG
  decoding (laptop, one thread). Moving the origin costs nothing measurable.

Long flights, far from the start (`tools/gen_synthetic.py --radius R --speed 12 --height 40
--density 0.004`, synthetic tracks, `config/vio.yaml` + `config/replay_synthetic.yaml`). ATE median
(range); position NEES as the mean over the runs. "Before" is the previous stage (positions about
the world origin):

| Flight | before: ATE, NEES | after, origin moved beyond 10 m | beyond 100 m (default) | beyond 1000 m |
|---|---|---|---|---|
| R = 1500 m, 9.1 km, at most 1.5 km from the start, 8 seeds | 40.1 m (28.7-69.7), 5.0 | 38.3 m (29.0-62.5), 4.6 | 38.3 m (28.7-62.6), 4.6 | 38.2 m (27.0-63.8), 4.7 |
| the same, ground truth moved 5 km | 188 m (144-247), 71 | as above | as above | as above |
| R = 4000 m, 24.4 km, at most 4 km from the start, 4 seeds | 224 m (201-258), 15 | 85.0 m (69.8-101), 4.6 | 84.8 m (69.2-102), 4.5 | 82.5 m (66.9-105), 4.4 |

- With the moving origin the result no longer depends on where the world origin is (the 5 km row
  is the same as the unmoved one), and hardly on the threshold. A moved start alone would pass
  trivially; the real test is the flight that gets 4 km away from its start: ATE 2.6 times smaller
  and NEES from 15 to 4.5.
- The origin moved 2178 / 244 / 24 times on one 4 km flight for the three thresholds.
- What is left is the normal drift of VIO over 24 km (0.35 % of the distance here, with the large
  placeholder IMU noise of the drone config). It is synthetic data with perfect tracks: it says
  nothing yet about drift on real flights.

How the step 3 settings were chosen. Single runs, kept for the record; the median over starts is
not known for these settings and the conclusions below are provisional:

| | hover render: drift, NEES pos / att | normal render: ATE, RPE | low contrast: NEES pos / att | EuRoC: ATE aligned, NEES pos |
|---|---|---|---|---|
| stride 1, cap 30 | 0.76 m/min, 26 / 34 | 0.23 %, 0.83 % | 15 / 8.5 | 0.16 m, 15 |
| stride 3, cap 30 | no landmarks, diverges | 0.21 %, 0.66 % | 17 / 11 | 0.19 m, 37 |
| stride 3, cap 36 | 0.61 m/min, 9.7 / 1.4 | 0.22 %, 0.53 % | 19 / 9.9 | 0.13 m, 14 |
| stride 6, cap 30 | no landmarks, diverges | 0.25 %, 0.72 % | 8 / 13 | diverges |
| stride 6, cap 72 | 0.36 m/min, 4.3 / 5.1 | 0.12 %, 0.42 % | 35 / 22 | diverges |

- The visual update every frame (stride 1) gives a window of only 0.37 s: in slow flight and on the
  way into a hover nothing has enough parallax. Every 6th frame (5 Hz) makes the window 2.2 s long.
- The age cap must be at least one window (12 update frames = 12 x stride camera frames), otherwise
  no track is long enough to become a landmark and the hover drifts away.
- EuRoC (20 fps, indoors at 3-5 m depth): at stride 6 the filter starves and diverges. The right
  stride depends on the scene; a motion-based rule should replace the fixed stride.
- Landmark drift guard: a landmark is dropped when its viewing direction turned more than 5 deg
  since it was made, after two chi2 failures in a row, or after 60 s. Landmarks are only made below
  4 m/s and with at least 3 deg of parallax.

### Why the filter is overconfident on some data: what is measured and what is not

The filter is overconfident on the low-contrast render and on EuRoC, honest on synthetic tracks
and about honest on the normal-contrast and hover renders (table above). The reasons are not
established; this is what the data shows so far:

1. Track drift. Measured, without the filter (DESIGN.md section 13.3): the KLT track error is a
   random walk whose variance grows with the view change of the feature; its white part is only
   0.17-1.0 px. Measured with the filter, single runs: on the low-contrast render, re-detecting
   every feature after 15 frames lowered position NEES from 11 to 6.7. Not established: how much of
   the NEES drift explains on each dataset. The fixes of step 3 (each feature used once, age cap)
   lower it on the low-contrast render (median 19 before the review fixes, 12 after), but hover
   needs tracks of at least one window, so the cap cannot be short. With the filter (2026-11-26,
   part a of DESIGN.md section 13 switched on, q measured on the normal render, 20 starts): position
   NEES on the low-contrast render 10 to 4.9, attitude 7.9 to 3.8, chi2 rejections 6.6 to 1.4 %. So
   drift is a large part of the overconfidence there. On EuRoC the same switch leaves position NEES
   where it is (69 to 70 with td 0) and lowers the rejections only from 34 to 29 %: drift does not
   explain EuRoC, at least not with this q.
2. On EuRoC: IMU noise, and a ground truth that cannot judge it. Measured: the filter claims about
   0.06 deg of tilt sigma while its tilt error varies by about 0.2 deg; three times the IMU noise
   brought attitude NEES from 30 to 7.7 (single runs). Not established: whether the EuRoC IMU really
   has larger errors in flight (vibration, scale factor, axis misalignment) or the EuRoC ground truth
   attitude is off by 0.1-0.2 deg (it is estimated from the same IMU in a batch fit, not measured
   independently). EuRoC cannot separate the two. Ruled out: td (with td fixed at 0 the picture is
   the same).
3. A guess, not measured: the 33 % of MSCKF features that fail the chi2 test on EuRoC were said to
   come from track drift. That was never shown, and part a (above) makes it unlikely: with the drift
   modelled 29 % still fail.
4. Distance from the world origin (measured, synthetic): far from the origin the filter became
   overconfident, likely through a linearization error of the invariant errors (DESIGN.md section
   11). Handled by the moving origin since 2026-11-25 (long-flight table above).

What follows: EuRoC cannot validate IMU noise values for our drone. Our IMU noise values come from
our own filtered FC stream (RECORDING.md section 7), and the consistency of the full system is
judged on our own recordings with independent ground truth (GROUND_TRUTH.md).

## 7. Step 3 report (2026-11-26)

Step 3 (the filter) is complete: stages 1-4, the review fixes, the moving origin, part a of the
dynamic covariance (off by default), the joint gate and the landed update (off by default), the
baro. Every number is a median and range over many runs, today's defaults. Raw data and scripts:
`~/datasets/analysis/` (one folder per stage, HANDOFF.md section 8).

Monte Carlo summary:

| Set | runs | ATE (navigation) | ATE aligned | NEES position | NEES attitude |
|---|---|---|---|---|---|
| synthetic figure8 (tracks) | 8 seeds | 0.154 % (0.104-0.240) | 0.785 m (0.577-1.738) | 2.85 (1.67-5.07) | 2.50 (2.23-3.43) |
| synthetic approach_hover (tracks) | 20 seeds | 0.257 % (0.133-0.699) | 0.090 m (0.059-0.171) | 1.47 (0.50-6.19) | 1.85 (0.99-3.69) |
| render fig8_tilt_img | 20 starts | 0.172 % (0.110-0.269) | 0.228 m (0.153-0.327) | 3.02 (0.80-11.44) | 2.36 (1.27-4.65) |
| render fig8_tilt_low | 20 starts | 0.446 % (0.225-0.800) | 0.467 m (0.281-1.191) | 10.50 (3.40-28.07) | 7.73 (3.05-17.07) |
| render approach_hover_img | 20 starts | 0.568 % (0.276-1.249) | 0.161 m (0.102-0.241) | 4.59 (1.53-12.92) | 3.45 (1.54-7.30) |
| EuRoC MH_01_easy, online td | 121 starts | 0.718 % (0.338-14.216) | 0.210 m (0.130-3.789) | 67.24 (12.16-908.03) | 40.20 (33.60-72.79) |
| long flight 9 km (R 1.5 km) | 8 seeds | 0.410 % (0.290-0.671) | 26.509 m (13.870-36.710) | 4.18 (2.25-7.99) | 2.82 (1.77-5.82) |
| long flight 24 km (R 4 km) | 4 seeds | 0.353 % (0.294-0.417) | 67.934 m (52.510-82.225) | 4.85 (3.11-5.68) | 3.93 (2.40-4.50) |
| 5 km straight, constant speed, baro | 8 seeds | 13.042 % (10.842-17.354) | 466.568 m (391.917-633.156) | 56.10 (38.64-86.86) | 2.32 (1.86-5.75) |
| 5 km straight, S-turns, baro | 8 seeds | 1.407 % (0.562-1.862) | 39.160 m (14.213-56.719) | 11.43 (4.32-19.87) | 2.62 (1.92-5.69) |
| 5 km loop, constant speed, baro | 8 seeds | 4.571 % (3.632-6.269) | 198.651 m (160.958-269.453) | 31.60 (22.26-50.32) | 3.17 (1.91-3.93) |
| 5 km loop, S-turns, baro | 8 seeds | 0.493 % (0.268-0.775) | 15.361 m (11.769-28.455) | 3.76 (1.18-8.33) | 3.24 (1.93-4.83) |
| liftoff sharp, baro | 20 seeds | 3.668 % (1.056-74.281) | 4.497 m (0.862-88.103) | 55.04 (11.10-508.03) | 7.84 (3.11-42.64) |
| liftoff gentle, blind below 1 m, baro | 20 seeds | 88.578 % (5.752-549.557) | 50.847 m (5.982-719.707) | 94.00 (43.11-621.49) | 19.83 (4.35-345.41) |

(Renders and EuRoC: start times; synthetic: seeds. Liftoff: C off, landed flag not used. The
renders, EuRoC and long-flight rows are from the takeoff stage, whose outputs are identical to
today's for sessions without a baro or landed flag.)

Midpoint linearization: off, as first built, and with keep-gate (2026-11-26, DESIGN.md section 11,
"Midpoint linearization"; the switch is off by default, keep-gate is the repo version). Today's code
with the switch off, the first build on, keep-gate on. 20 seeds or starts per set, except the 24 km
flight (its 4 seeds, the set of DESIGN.md 13.8) and EuRoC with 121 starts; the line and loop sets have
20 seeds (8 in the table above). Median (range). The verdicts compare the medians against the noise
band of each version (pixel noise times 0.999 and 1.001, DESIGN.md 13.8): "same" when the difference
stays inside a band; NEES "closer" / "further" means closer to or further from 3; the 121-start EuRoC
set has no band. Baro on unless stated; liftoff C off. Runs:
`~/datasets/analysis/midpoint_2026-11-26/` (`run_eval.sh`, `run_kg2.sh`, `summarize.py`,
`table3_md.py`).

| Set | runs | ATE off | ATE midpoint | ATE keep-gate | NEES pos off | midpoint | keep-gate | keep-gate vs off | keep-gate vs midpoint |
|---|---|---|---|---|---|---|---|---|---|
| synthetic figure8 (tracks) | 20 | 0.148 (0.104-0.298) % | 0.129 (0.087-0.254) % | 0.129 (0.082-0.254) % | 2.98 (1.67-5.61) | 2.94 (1.43-4.78) | 2.94 (1.43-4.78) | ATE better, NEES further | ATE same, NEES same |
| synthetic approach_hover (tracks) | 20 | 0.258 (0.133-0.699) % | 0.254 (0.119-0.657) % | 0.254 (0.119-0.657) % | 1.47 (0.50-6.19) | 1.55 (0.47-5.63) | 1.55 (0.47-5.63) | ATE better, NEES closer | ATE same, NEES same |
| render fig8_tilt_img | 20 | 0.172 (0.110-0.269) % | 0.163 (0.111-0.264) % | 0.163 (0.111-0.264) % | 3.02 (0.80-11.44) | 2.88 (0.81-11.56) | 2.88 (0.81-11.56) | ATE better, NEES further | ATE same, NEES same |
| render fig8_tilt_low | 20 | 0.446 (0.225-0.800) % | 0.359 (0.231-0.680) % | 0.359 (0.231-0.680) % | 10.51 (3.40-28.07) | 8.43 (2.49-28.89) | 8.43 (2.49-28.89) | ATE better, NEES closer | ATE same, NEES same |
| render approach_hover_img | 20 | 0.569 (0.276-1.249) % | 0.609 (0.298-1.283) % | 0.609 (0.298-1.283) % | 4.59 (1.53-12.92) | 5.52 (1.76-13.63) | 5.52 (1.76-13.63) | ATE worse, NEES further | ATE same, NEES same |
| EuRoC MH_01_easy, online td | 20 | 0.718 (0.576-1.076) % | 0.711 (0.496-3.668) % | 0.729 (0.480-3.668) % | 66.46 (45.00-157.75) | 66.45 (37.52-124.76) | 64.19 (33.63-121.09) | ATE worse, NEES same | ATE same, NEES same |
| EuRoC MH_01_easy, 121 starts | 121 | 0.718 (0.338-14.216) % | 0.720 (0.496-1378.329) % | 0.728 (0.480-3.668) % | 67.24 (12.16-908.03) | 66.45 (37.52-159.15) | 67.18 (33.63-143.67) | ATE no band, NEES no band | ATE no band, NEES no band |
| long flight 9 km (R 1.5 km) | 20 | 0.442 (0.271-0.701) % | 0.333 (0.208-0.624) % | 0.334 (0.205-0.619) % | 4.19 (2.25-9.74) | 2.62 (0.92-6.26) | 2.70 (0.93-6.27) | ATE better, NEES closer | ATE same, NEES same |
| long flight 24 km (R 4 km) | 4 | 0.354 (0.294-0.417) % | 0.323 (0.159-0.370) % | 0.331 (0.151-0.378) % | 4.85 (3.11-5.68) | 2.34 (1.74-3.20) | 2.40 (1.59-3.32) | ATE better, NEES closer | ATE same, NEES same |
| 5 km straight, constant speed | 20 | 13.106 (9.515-17.476) % | 1.876 (0.528-5.546) % | 1.885 (0.528-5.466) % | 54.72 (31.50-86.86) | 4.53 (1.16-10.69) | 4.42 (1.16-10.42) | ATE better, NEES closer | ATE same, NEES same |
| 5 km straight, speed +-20 % | 20 | 6.434 (5.280-6.820) % | 3.330 (2.462-4.043) % | 3.349 (2.464-4.002) % | 56.75 (39.51-65.29) | 17.03 (9.51-23.22) | 17.01 (9.51-23.31) | ATE better, NEES closer | ATE same, NEES same |
| 5 km straight, S-turns | 20 | 0.992 (0.303-1.862) % | 0.616 (0.178-1.172) % | 0.619 (0.178-1.171) % | 6.08 (1.10-19.87) | 3.00 (0.59-8.03) | 3.01 (0.59-8.04) | ATE better, NEES closer | ATE same, NEES same |
| 5 km loop, constant speed | 20 | 4.861 (3.610-7.327) % | 0.841 (0.357-2.209) % | 0.838 (0.357-2.207) % | 34.97 (22.26-58.87) | 2.54 (1.02-7.38) | 2.54 (1.02-7.37) | ATE better, NEES closer | ATE same, NEES same |
| 5 km loop, speed changes | 20 | 2.508 (1.807-2.980) % | 1.357 (0.684-1.706) % | 1.371 (0.678-1.705) % | 31.03 (16.54-40.14) | 10.13 (3.20-14.94) | 10.11 (3.32-14.93) | ATE better, NEES closer | ATE worse, NEES same |
| 5 km loop, S-turns | 20 | 0.492 (0.196-0.894) % | 0.418 (0.207-0.610) % | 0.418 (0.207-0.619) % | 4.17 (0.91-9.34) | 3.35 (0.97-5.82) | 3.32 (0.98-5.69) | ATE better, NEES closer | ATE same, NEES same |
| 5 km straight, constant speed, baro off | 20 | 13.646 (9.690-17.770) % | 1.806 (0.528-5.842) % | 1.769 (0.528-5.878) % | 60.07 (31.19-92.81) | 5.47 (1.13-14.44) | 5.50 (1.13-14.55) | ATE better, NEES closer | ATE same, NEES same |
| 5 km straight, speed +-20 %, baro off | 20 | 4.097 (3.039-4.683) % | 0.663 (0.299-1.200) % | 0.661 (0.305-1.232) % | 20.86 (12.96-28.08) | 2.34 (0.55-7.54) | 2.35 (0.57-7.65) | ATE better, NEES closer | ATE same, NEES same |
| 5 km straight, S-turns, baro off | 20 | 0.960 (0.271-1.846) % | 0.587 (0.184-1.126) % | 0.606 (0.185-1.138) % | 5.72 (1.10-18.92) | 3.14 (1.08-6.85) | 3.15 (1.07-6.98) | ATE better, NEES closer | ATE worse, NEES same |
| 5 km loop, constant speed, baro off | 20 | 5.101 (3.632-8.002) % | 0.904 (0.358-2.598) % | 0.895 (0.366-2.584) % | 36.52 (23.26-66.18) | 3.00 (1.07-10.48) | 3.01 (1.07-10.44) | ATE better, NEES closer | ATE same, NEES same |
| 5 km loop, speed changes, baro off | 20 | 1.556 (0.877-1.966) % | 0.599 (0.283-1.180) % | 0.581 (0.281-1.183) % | 13.09 (5.35-17.44) | 2.98 (1.00-8.85) | 2.83 (0.95-8.92) | ATE better, NEES closer | ATE same, NEES same |
| 5 km loop, S-turns, baro off | 20 | 0.510 (0.233-0.873) % | 0.438 (0.199-0.621) % | 0.437 (0.199-0.607) % | 4.77 (1.21-11.62) | 3.62 (1.15-10.95) | 3.63 (1.11-10.75) | ATE better, NEES closer | ATE same, NEES same |
| liftoff sharp | 20 | 3.668 (1.056-74.281) % | 3.551 (0.681-80.419) % | 3.138 (0.681-31.495) % | 55.04 (11.10-508.03) | 46.36 (3.03-731.69) | 46.82 (3.03-337.35) | ATE same, NEES closer | ATE same, NEES same |
| liftoff gentle, blind below 1 m | 20 | 88.578 (5.752-549.557) % | 35.361 (5.975-205.751) % | 32.895 (5.973-205.751) % | 94.00 (43.11-621.49) | 60.50 (12.29-594.93) | 60.50 (12.29-546.63) | ATE better, NEES closer | ATE same, NEES same |

Signed scale (`tools/scale.py`, estimated / true distance on 100 m pieces of the path, mean over
the flight), off and on: straight constant speed 1.33 / 1.03 (baro off), loop constant speed 1.23 /
1.02 (baro off), straight with speed changes 1.08 / 1.01 (baro off), figure8 1.004 / 1.000, long
flights 1.017 / 0.999 (9 km) and 1.020 / 1.001 (24 km), normal render 1.003 / 1.003. Keep-gate gives
the same (straight 1.024 with the baro, loop 1.013). The straight-leg bias is smaller, not gone: with
the baro on, speed ratio at 95 % 1.055 (0.92-1.21) for the first build and 1.053 (0.93-1.20) with
keep-gate, scale over the last 10 % of the path 1.107 / 1.112.

What is honest and what is not:

- Honest (position NEES about 3, attitude 2-3): synthetic figure8 and approach_hover, the normal
  render; the loop with S-turns is at the upper edge (NEES 4.2 over 20 seeds). The long flights are
  mildly overconfident (NEES 4-5). Loops without S-turns are not honest (constant speed NEES 35,
  speed changes 31; corrected 2026-11-26, 20 seeds).
- Overconfident: the low-contrast render (10; drift-aware noise brings it to 4.9, switch off),
  EuRoC (67; IMU side not separable from its reference), straight legs and loops at constant speed
  (31-56; the scale error is not in the covariance), takeoff (55-94 even with the baro).
- Scale on flights without accelerations (found 2026-11-26, DESIGN.md section 11, "Straight-leg
  scale bias"): the visual update, linearized at the estimated clone positions, pushes the speed up
  a little on every update, and with nothing accelerating nothing pulls it back. Straight leg at
  constant speed: speed 1.52 times the true one at the end (20 of 20 seeds above 1); loop at
  constant speed: 1.40 times (its along-track error at 5 km looks small only because the loop is
  almost closed there). With the Jacobians at the true window shape: 2.0 % and 0.9 % ATE, NEES
  about 3-5. Not a coding error. With the midpoint linearization (switch, off by default): 1.9 % and
  0.84 %, NEES 4.5 and 2.5 (baro on); still overconfident along the track on the straight leg
  (error/sigma rms 2.7).
- Accurate within 1 % of the distance (corrected 2026-11-26, review; 20 seeds, table above), with
  today's defaults: only flights that turn often: figure8, the renders, the long flights, the S-turn
  legs (straight 0.99 %, loop 0.49 %). Not: straight or loop at constant speed (13 % / 4.9 %), speed
  changes without turns (straight 6.4 %, loop 2.5 % with the baro; 4.1 % / 1.6 % without, the baro
  speed term leaks into the scale), the takeoff. With the midpoint switch on: also the constant-speed
  straight leg and loop come near it (1.9 % / 0.84 %), speed changes 3.3 % / 1.4 % with the baro and
  0.66 % / 0.60 % without (all with white pixel noise and the filter told the right value; with
  tracks that drift like the rendered ones the midpoint over-corrects, DESIGN.md section 11, "Pixel
  noise study"). The straight-leg bias is smaller, not gone: speed 1.055 (0.92-1.21) times
  the true one at 95 % of the flight, scale over the last 10 % of the path 1.107 (0.96-1.26).

CPU per frame (laptop, one thread): filter 0.5-1.7 ms per update frame median, at most 6.6 ms;
tracker 6.5-8.2 ms per camera frame with PNG decoding (DESIGN.md section 11, stage 4). Midpoint
linearization on: the whole visual update per update frame 1.4-2 times longer (straight leg 0.36 to
0.71 ms, normal render 1.18 to 2.28 ms, EuRoC 3.14 to 4.43 ms). On the Pi 5: not measured yet (point A).

What rests on synthetic data only (and how far it goes):

- Everything about the baro (model a guess), the takeoff (vibration and flag timing are models,
  near points are exact), the scale drift over km (generator tracks without drift on a flat
  textured ground, no wind, no turbulence: real flight gives more accelerations, which helps, and
  drifting tracks, which hurts), the long flights, the moving origin.
- The renders add real KLT tracks, but on an idealised flat ground with no blur, no exposure change
  and no 3D structure.
- EuRoC is real data with a forward camera indoors and another IMU: it shows the filter survives
  real images and a real IMU, not how our drone will do.

Left before step 4 (DESIGN.md section 11 lists the open items): the straight-leg scale problem
(cause found 2026-11-26, a bias of the visual update on flights without accelerations; the
midpoint linearization removes most of it on synthetic data, switch off by default, decision open,
DESIGN.md section 11), the takeoff (7 s on the IMU alone; C waits for our recordings), the baro speed term
(measure on our recordings), td and the IMU noise values from our recordings.
