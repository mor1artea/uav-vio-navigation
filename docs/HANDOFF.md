# Handoff

For developers joining the project. Read `README.md` first. Updated 2026-11-27. The history of how
we got here is in git and in `docs/DESIGN.md` section 11; this file is only the current state.

"The owner" is the person responsible for the drone and the project; "you" and "your decision" in
the other documents mean the owner. Folders named `~/datasets/analysis/...` in the documents are on
the owner's machine, not in the repo.

## 1. What we have

All offline, on a Linux PC. Nothing runs on the drone or the Pi, nothing has been flown.

| Part | State | Where |
|---|---|---|
| Session reader, replay, metrics | done, reviewed | `src/session.cpp`, `src/tools/vio_replay.cpp`, `tools/metrics.py` |
| Synthetic generator, ground renderer | done, reviewed | `tools/gen_synthetic.py`, `tools/render_ground.py` |
| Front end: camera models, KLT tracker, sky mask | done, reviewed | `src/frontend.cpp`, `src/camera_model.cpp`, `vio_track` |
| Filter: right-invariant error-state EKF, MSCKF update, landmarks, online td, baro, joint gate, moving origin | done, reviewed | `src/filter.cpp`, `src/msckf.cpp` |
| Quality monitor (GOOD / DEGRADED / LOST, score, logged only) | done, reviewed | `src/quality.cpp` |
| Fault injection (`vio_replay --inject`) | done, reviewed | `src/inject.cpp` |
| IMU gap noise switch (off) | implemented, **not reviewed** | `src/filter.cpp`, DESIGN.md 11 "IMU gap noise" |
| Accel delay compensation, `gen_synthetic.py --fc-imu` | implemented, **not reviewed** | `src/imu_propagation.cpp`, DESIGN.md 11 "Piece 3" |
| `sim/`: SITL + Gazebo open-loop recording | written and checked against the sources only; **never run** with Gazebo, `sim/setup.sh` never run | `sim/README.md` |

Tests: `ctest` runs 7 (C++ unit tests and `tools/test_metrics.py`); `sim/test_make_session.py` is
separate. Data used: synthetic sessions, rendered ground images (real KLT tracks on idealised flat
ground), and one real public sequence, EuRoC MH_01_easy (forward camera, indoors, another IMU).
Everything about our drone's own camera, IMU path, vibration and km-long flights is synthetic only.

## 2. What is left, in order

| # | Item | Needs |
|---|---|---|
| 1 | Owner review of the IMU gap noise switch and of piece 3 | owner decision |
| 2 | Step 4 piece 4: recovery when every track is rejected (D3 = report LOST only, as a baseline; D2 = restart velocity, biases and td; D1 inflation only with the owner's OK). The same trigger, "every track rejected for N updates in a row", also catches frozen images and outliers; report its false alarm rate on runs without faults | nothing new; plan in DESIGN.md 11 "Step 4 plan" |
| 3 | Step 5: one results summary on synthetic data and EuRoC | nothing new |
| 4 | First run of `sim/` (open-loop SITL + Gazebo flights, straight leg with and without wind) | native Linux with a GPU; expect fixes on the first run |
| 5 | Point A: build on the Pi 5 (aarch64), CPU per frame with replay | the Pi; owner's OK before any SSH or install on it |
| 6 | Stage 1b: Pi recorder and the converter from recordings to sessions (both not written, spec in RECORDING.md); bench recordings, 2-3 h static IMU recording | Pi + camera + FC on USB + fast SD card |
| 7 | Calibration: camera model and intrinsics, camera-IMU extrinsics and td, IMU noise (CALIBRATION.md) | the camera mount, an Aprilgrid, Kalibr |
| 8 | Stage 2: replays of our own flights with ground truth; then the decisions that wait for real data (section 4 below, pixel noise, update stride, td start value, LOST exit rule, E1 limits) | flights, a GNSS logger (not chosen, GROUND_TRUTH.md) |
| 9 | Stage 2b: closed loop in SITL (our ODOMETRY into EKF3, LOST, failsafes) | MAVLink output code |
| 10 | Stage 3: the live module (capture, MAVLink I/O, ODOMETRY output), bench without props | Pi + camera + FC; FC parameters applied by the owner |
| 11 | Stage 4: field tests | everything above |

Open owner decisions: firmware 4.6.3 or 4.7.3, MAVLink logging (`LOG_BACKEND_TYPE = 2`), when to add
Loiter to the mode switch, the GNSS logger, a license for the repo (no LICENSE file yet), whether
SITL sessions are marked as FC-filtered (DESIGN.md 11 "Piece 3"), a more dynamic generated scenario.

## 3. Known problems (median and range over 20 runs unless noted)

- **Scale error on straight legs.** Synthetic 5 km leg at constant 12 m/s: ATE 13.1 % (9.5-17.5),
  position NEES 55 (31-87), true velocity error at the end 6.6 m/s (4.1-8.1). Loop at constant speed:
  4.9 % ATE, 5.8 m/s. Speed changes without turns: 6.4 %. S-turns every 40 s: 0.99 % (0.30-1.86).
  With track drift like real KLT tracks it is worse: constant speed 16 m/s error at the end, speed
  changes 3.1 m/s, S-turns 0.73 m/s. Cause: the visual update, linearized at the estimated window,
  pushes the speed up a little every update and nothing pulls it back without accelerations. Not a
  coding error. Midpoint linearization removes most of it on synthetic data (13.1 % to 1.9 %), off.
  Flight rule for now: weave on long legs (ARDUPILOT_SETUP.md section 8).
- **The quality monitor does not see that error.** On those sets 78-84 % of the rows with more than
  1 m/s velocity error are GOOD, the same share as overall; the velocity sigma stays at 0.66 m/s
  (0.56-0.79). Only the 25 m/s speed check reacts, when the estimate is about twice the true speed.
  The logged "low excitation" flag (E1) fires on these sets but also on 57 % of S-turn frames and on
  99 % of a 9 km flight whose error stays small.
- **Overconfident covariance:** EuRoC position NEES about 60-70 (honest is 3); low-contrast render
  10 (2.0-28); hover render 4.6; long synthetic flights 4-5; 1-3 s after a 10 s camera blackout on the
  render 5-7; after a 0.5 s IMU gap (switch off) velocity NEES 10-25 for up to 4 s.
- **EuRoC:** 33-38 % of the tracks fail the chi2 test (5 % expected), cause unknown; 2 of 121 start
  times end above 1 m error (starts 22.35 and 25.45 s); the monitor is DEGRADED 36 % of the time with
  0.03 m/s velocity error (a false alarm for navigation).
- **Takeoff is poor:** 7 s on the IMU alone while the rotors spin on the ground; synthetic gentle
  takeoff 62 % LOST with a large velocity error; aligned ATE 4.5-51 m in the takeoff sets.
- **td (camera-IMU time offset):** the online estimate is unreliable. On synthetic data it ends
  within about +-8 ms of the truth and does not find a 7-8 ms IMU delay; the filter treats td as
  constant, so a later step is neither followed nor detected. Its start value must come from
  calibration.
- **Monitor limits on synthetic data:** the generated ground is sparse (about 37 features per frame),
  so the 30-feature check fails on 7-19 % of the frames and gives LOST stretches of up to 34 s on the
  24 km flight. With fewer than 10 tracks per update the inlier check is skipped, so frozen images
  and outliers are not seen there.

## 4. Switches in `config/vio.yaml`

| Key | Default | Why |
|---|---|---|
| `filter.midpoint_linearization` | off | helps the scale error on synthetic data, over-corrects with real-like track noise; off until real data |
| `filter.track_drift.enabled` | off | drift-aware feature noise; its q is measured only on renders, one q does not fit hover and flight |
| `filter.zupt.landed.enabled` | off | zero velocity while the FC says landed; fired while climbing in a gentle takeoff, EuRoC worse with it |
| `filter.imu_gap_noise.enabled` | off | extra noise across IMU gaps; not reviewed, it is added process noise, so the owner decides |
| `filter.mag_heading.enabled` | off | config key only, no code |
| `filter.baro.enabled` | on | weak height update; a baro error that follows the speed leaks into the scale |
| `quality.enabled` | on | passive: adds columns to the replay output, changes nothing else |
| `imu.accel_delay_compensation` | on | acts only on sessions marked `fc_filtered: true`; not reviewed |
| `imu.td_estimate_online` | on | see td above |

Placeholders, not tuned: all quality limits, the gap noise values, the IMU noise densities (the FC
stream has not been measured), `pixel_noise_px` 1.0, update stride 6, track age cap 72.

## 5. Things to remember

- Single runs mislead: NEES can change 40 times between start times seconds apart. Use 20 start
  times or seeds (`tools/eval_runs.py`), report median and range, and judge a change against the noise
  band: the same runs with `pixel_noise_px` times 0.999 and 1.001 (DESIGN.md 13.8).
- Replays take `config/vio.yaml` plus one override (DATASETS.md section 8). Synthetic sessions start
  in flight from the ground truth; EuRoC and `sim/` sessions start still on the ground.
- Generated sessions with features only cannot show track drift; use the renders with the tracker
  running inside `vio_replay` for anything about real tracks.
- The generator's IMU noise is above the still detector's limit: generated sessions never get still
  updates, and the liftoff scenario starts armed.
- EuRoC: z-up world; MH_01 is carried by hand at first (use `--start 22`), rotors spin on the floor
  from 38.8 s, liftoff at 45.2 s; its camera looks forward, so `vio_track --no-sky-mask`.
- `vio_replay` and `vio_track` write only where `--out` says. The CSV header names every column;
  the quality columns are listed at the top of `src/tools/vio_replay.cpp`.
- Results depend on the distance from the start without the moving origin; a km-scale test must
  really fly far from its start.
- Long runs: start them detached (`setsid nohup ... &`). On WSL2 with about 8 GB of RAM, run at most 4
  replays at a time (one replay 70-360 MB, one generator up to 300 MB), do not generate sessions while
  replaying, and stop when free disk drops below 10 GB.
- ArduPilot behaviour is checked against the 4.6.3 source, not on the FC; the quirks we rely on need a
  bench check (ARDUPILOT_SETUP.md).

## 6. Rules

These protect the drone and the project. Keep them.

- Never change anything on the flight controller and never put files on it. FC parameters are
  applied by hand by the owner, from ARDUPILOT_SETUP.md section 10.
- Nothing on the Pi (no SSH setup, no installs) until the owner says so (point A).
- The Pi recorder, once written, is passive: the only message it may send to the FC is
  `SET_MESSAGE_INTERVAL` (RECORDING.md section 1).
- Real site coordinates only in `config/local.yaml` (in `.gitignore`). Never commit the FC
  parameter backup file `stellar_h7v2_before_vio`.
- Ask before copying any GPL code (OpenVINS, VINS, ORB-SLAM). Permissive code may be borrowed in
  small pieces with attribution; list it in `docs/THIRD_PARTY.md`.
- No commit or push without the owner's OK. No co-author lines, signatures or watermarks.
- Noise inflation and covariance scaling are the owner's decision: propose them with their cost, do
  not enable them.
- Code simple and readable; few tests, only the key ones. Docs and comments in plain language, no
  long dashes. Check before finishing:
  `grep -rnP "\x{2014}|\x{2013}" docs config tools src include tests sim README.md`
