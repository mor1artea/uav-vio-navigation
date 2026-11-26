# VIO module design (Stage 0)

Status: Stage 0 approved 2026-11-23. Code in progress, Stage 1a; where it stands is in section 11.

Goal: a VIO module on a Raspberry Pi 5 that gives ArduCopter position and velocity over MAVLink
on a drone that has no GNSS at all, for real 2-5 km flights. The FC keeps attitude (IMU + external
compass). We give position and velocity with an honest covariance and an explicit quality signal.

Hardware: Stingbee Stellar H7V2 FC (STM32H743, 2 MB flash, one ICM-42688-P IMU, DPS310 baro,
external compass, ESC telemetry with per-motor RPM notches), quad X (QUAD/X_REV), Raspberry Pi 5
on the FC USB port, IMX219 with a ~120 deg lens looking straight down, rigid mount. No GPS, no
rangefinder, no optical flow sensor, no confirmed telemetry radio (the design does not need one).

Firmware: ArduCopter V4.6.3 (92b0cd78), the version on the drone. The hash is exactly the official
`Copter-4.6.3` tag, but that tag has no StellarH7V2 board, so the binary is a vendor or custom build:
the 4.6.3 tag plus locally added board files. The EKF3, VISO and MAVLink code is therefore expected
to be the tag code, and the important quirks get a bench check (ARDUPILOT_SETUP.md section 1).
Everything is designed and checked against the 4.6.3 source. Where 4.7.3 would behave differently, it is marked in
[ARDUPILOT_SETUP.md](ARDUPILOT_SETUP.md), which also has the details, the file references and the
full parameter list. Calibration is in
[CALIBRATION.md](CALIBRATION.md), ground truth in [GROUND_TRUTH.md](GROUND_TRUTH.md), borrowed
code in [THIRD_PARTY.md](THIRD_PARTY.md).

## 1. Existing VIO projects

CPU numbers on ARM come from Delmerico and Scaramuzza, ICRA 2018 (ODROID XU4 and UP board,
EuRoC), Jeon et al. 2021 (Jetson boards), and one paper with OpenVINS timings on a Pi 5. They are
rough guides. Our own numbers will come from profiling on the Pi 5.

| Project | Method | CPU on ARM | Covariance out of the box | Feature loss | Dependencies | License (checked in repo) |
|---|---|---|---|---|---|---|
| OpenVINS | MSCKF (EKF + sliding window), optional SLAM landmarks | Light to medium. About 40-52 ms/frame stereo on a Pi 5 | Yes | Good: IMU carries the state, chi2 gating, ZUPT | Eigen, OpenCV, Boost, Ceres. ROS optional | GPL-3.0 |
| VINS-Mono / VINS-Fusion | Sliding window optimization | Heavy, most accurate in the 2018 benchmark | No | Good, plus loop closure | ROS 1, Ceres | GPL-3.0 |
| Basalt | Optimization, fast patch KLT front end | Front end very fast, back end medium | Not directly | Good | Eigen, TBB, Pangolin | BSD-3-Clause |
| ORB-SLAM3 | Feature SLAM + visual-inertial BA | Heavy | No | Weak on low texture, has relocalization | Pangolin, DBoW2, g2o | GPL-3.0 |
| Kimera-VIO | Factor graph (GTSAM) | Heavy | Not exported | Good | GTSAM, many libs | BSD-2-Clause |
| ROVIO | Iterated EKF on image patches | Medium, bounded per frame | Yes | Medium, weak on low texture and blur | kindr, lightweight_filtering | BSD-3-Clause |
| SVO Pro | Semi-direct + optimization | Front end very light | No | Medium | ROS, Ceres | GPL-3.0 |
| OKVIS2 | Keyframe optimization + loop closure | Heavy | No | Good | Ceres, DBoW2 | BSD-3-Clause |
| msckf_vio (Penn) | Stereo MSCKF | Light | Yes | Medium | ROS | Non-commercial research only |

What can be reused and how is in [THIRD_PARTY.md](THIRD_PARTY.md). Short version: BSD code
(Basalt, Kimera, ROVIO, OKVIS) can be borrowed in pieces with attribution. GPL code (OpenVINS,
VINS, ORB-SLAM3, SVO Pro) only after you decide on a GPL license, so for now it is ideas only.

### Prior art for VIO -> ArduPilot

- `AP_VisualOdom` with backends MAVLink (`VISO_TYPE=1`), T265 (`2`), VOXL (`3`).
- Official docs: Non-GPS Navigation, T265, OAK-D, VOXL, GPS / Non-GPS Transitions, EKF Source
  Selection. They all say the EKF origin must be set by hand when there is no GPS.
- Lua applets in the ArduPilot repo (`ahrs-source-extnav-optflow.lua` and others) that watch
  `visual_odom:quality()` and switch sources. Not usable here: the FC has no SD card for scripts.
- thien94/vision_to_mavros (`t265_to_mavlink.py`, GPL-3.0): the reference companion script.

## 2. Decision

We write our own compact monocular MSCKF (error-state EKF with a sliding window of camera clones
and a few persistent landmarks) with a gyro-aided KLT front end. OpenVINS math is the reference for
the filter, Basalt and Kimera are the reference for the tracker. The filter approach fits the Pi 5
(one update per frame, linear cost in the number of features, no iterative solver), it gives a
real covariance for free, and the IMU carries the state through short feature losses, which is
what the quality and covariance requirements need. Optimization methods are a bit more accurate on
benchmarks, but they are heavier, do not export a usable covariance, and their visual SfM
initializers are badly conditioned over flat ground. We avoid visual initialization entirely: the
filter starts on the ground, static, with gravity from the accelerometer, yaw from the FC compass
and position (0, 0, 0). OpenVINS is used only as an offline baseline on our recordings.

## 3. Startup without GNSS

- The module starts on the ground with the drone still. Position = (0, 0, 0) at the FC IMU, yaw =
  FC compass yaw read with motors off (cleanest compass reading), gravity from the accelerometer,
  gyro bias from the static average.
- While the drone stands still with the motors off, zero velocity updates come from the IMU and the
  image (OpenVINS idea). With the motors running the IMU shakes too much for that; then the FC
  "landed" flag (`EXTENDED_SYS_STATE`), a still image and a mean accel equal to gravity give the
  update instead (section 11, "Takeoff robustness"). The state stays close to perfect until liftoff.
- If the drone is moved while disarmed (carried to the takeoff spot), the module re-zeroes when it
  is still again and disarmed. This increments `reset_counter`, which is harmless on the ground.
- The EKF origin is set without GPS: if the FC has none, the module sends `SET_GPS_GLOBAL_ORIGIN`
  at boot with the coordinates from `config/local.yaml` (not in git; the repo only has a
  placeholder). This is required: without an origin, Copter never runs its EKF failsafe check at
  all. Coordinates rounded to the nearest city are enough (analysis in ARDUPILOT_SETUP.md,
  section 6).
- Takeoff in AltHold. The first 1-2 m the camera is too close to the ground (blur, dust, huge
  flow), so the module is DEGRADED for a moment and the IMU carries it. Switch to Loiter once the
  module reports GOOD above 3-5 m.

## 4. Scale with IMU only, over a mostly flat scene

How the reference projects get metric scale: all of them (OpenVINS, VINS, Basalt, ROVIO,
ORB-SLAM3) get it only from the accelerometer. VINS and ORB-SLAM3 solve scale in a visual-inertial
alignment step that needs a few seconds of motion with acceleration, then keep refining it in the
window. OpenVINS and ROVIO carry it in the filter state. None of them uses a barometer.

What the theory says (Martinelli, Hesch et al., Wu and Roumeliotis "VINS on wheels"): VIO has four
unobservable directions (3D position and yaw). Scale is observable only when the platform
accelerates. At constant velocity it is not observable, and the scene structure does not change
that.

What the flat scene changes: with rotation known from the gyro, the image of a plane gives
velocity divided by distance to the plane (`v / d`) and the plane normal. So on a flat scene scale
and height above ground are the same unknown. The IMU measures changes of `v`. Every time the drone
accelerates, `delta_v / d` is seen in the image and `delta_v` is measured by the IMU, which fixes `d`
and with it the scale.

How well that works for us:

- During a maneuver: a speed change of 4 m/s over 2 s with an accel bias uncertainty of
  0.03-0.05 m/s^2 gives a velocity error of about 0.1 m/s, so 2-3% scale information per maneuver.
  Several maneuvers average this down.
- In a long straight leg at constant speed, there is no new scale information. The scale error
  then drifts with the accelerometer bias error, which itself is poorly observable in that
  direction. The filter covariance shows this honestly.
- Real FPV flight has turbulence and pilot inputs (0.2-1 m/s^2 of random acceleration), which keeps
  giving information. Our worst case is a very smooth autonomous straight line.
- Terrain changes (hills) are fine as long as the velocity is right: the IMU says velocity did not
  jump, so a change in flow is read as a change in `d`.

So the answer: scale is observable, weakly, and how well depends on how much the drone accelerates.
Expect a few percent scale error on long smooth legs. Practical measures:

1. Track the scale uncertainty explicitly (the velocity magnitude direction of the covariance) and
   send it as a `NAMED_VALUE_FLOAT`. The quality score uses it.
2. Field procedure: on long legs, do a speed change or an S-turn every 30-60 s. It costs nothing and
   gives the filter fresh scale information. Every 80 s or more is not enough (synthetic, section
   11, "Straight-leg scale bias").
3. Baro is used only as a weak vertical measurement with a bias random walk (noise around 1 m,
   inflated near the ground and at high climb rates or throttle, chi2 gated). It damps vertical
   drift. It is not trusted for scale, as you asked. Built in stage 4 of step 3 (section 11): on a
   level leg it gives no scale, as intended, but a baro error that follows the speed (dynamic
   pressure at the port) does leak into the scale during speed changes.
4. Later, if needed: the rotor drag effect in the accelerometer (which EKF3 also uses for
   multicopters) gives a weak airspeed signal. Not in the first version.

## 5. Down-facing camera at up to 45 deg tilt

- Frames: filter state in a gravity-aligned NED world frame (origin at takeoff, north from the
  compass), body = FC IMU (FRD). Camera pose from the full extrinsic `T_body_cam` in the config,
  including the small misalignment from calibration. Depth of a ground point is along its own ray:
  `depth = d / (ray_world . down)`, never the height along the optical axis.
- Sky and horizon masking (mandatory): at 45 deg tilt the image edge looks about 105 deg away from
  nadir, so horizon and sky are in the frame. For every frame we take the attitude at the frame mid
  time, rotate a precomputed per-pixel ray table (from the fisheye model, on a coarse grid) into the
  world frame, and mask every ray more than 75 deg from nadir (config). This removes sky, horizon
  and very distant ground in one test. No feature is detected or kept inside the mask. The usable
  image fraction goes into the quality score. Built in step 2 (16 px ray grid).
- Open item for Stage 2: an image check at the mask boundary (bright, low texture) to catch cases
  the geometry misses (attitude error, mountains, tall trees). Postponed because neither EuRoC nor
  the synthetic data has a real sky, so it could not be tested; it needs our own recordings.
- Planar degeneracy: MSCKF triangulation works on planes, it does not use the essential matrix.
  The classic planar problem only hits visual initializers, and we do not use one.
- Hover and low parallax: 15-25 long-lived features are kept in the state as landmarks (OpenVINS
  idea). In hover they stay in view and hold the position, because zero motion gives zero flow
  whatever the scale is.
- Degenerate cases: pure rotation (features without baseline are not triangulated, landmarks still
  update), hover (landmarks), featureless ground (uniform field, water, dusk: too few tracks or low
  inlier ratio -> DEGRADED -> LOST, no fake updates, covariance stays honest). Moving water gives
  non-static features that the chi2 gating rejects, and the quality drops.

### Tracker check for 20-50 m and 8-12 m/s (your defaults, flagged as assumptions)

Processing image 640x480, equidistant fisheye, focal length about 300-380 px depending on whether
the 120 deg is horizontal or diagonal (calibration will tell). Frame rate 30 fps.

| Case | Image motion per frame | Problem? |
|---|---|---|
| Translation, worst (20 m, 12 m/s): 0.6 rad/s at nadir | 6-8 px | No |
| Translation, 50 m, 8 m/s | 2-3 px | No, but parallax per window is small, landmarks help |
| Fast rotation, 200 deg/s (3.5 rad/s) | 35-45 px | Yes for plain KLT. Fixed by predicting feature positions with the gyro before KLT (Kimera idea), then only the translation part is left |
| Low altitude, 5 m at 12 m/s | 25-30 px | OK with a 3-level pyramid and gyro prediction |

So 20-50 m at 8-12 m/s is fine for the tracker, even at 15-20 fps. Rotation, not translation, is
the hard part, and these are the costs:

- Motion blur: at 3.5 rad/s and a 2 ms exposure, blur is about 2.5 px (acceptable); at 4 ms it is
  5 px (bad). Exposure cap 2 ms by default, gain allowed to rise. Translational blur is under 0.5 px.
  At dusk a 2 ms exposure is too dark, so dusk will be DEGRADED or LOST.
- Rolling shutter: readout about 12 ms in the 8-bit binned mode (23 ms in 10-bit). At 3.5 rad/s that
  is 15 px of skew across the frame (28 px in 10-bit). Each feature gets its row time and is rotated
  to the frame mid time with the gyro. Translation during readout is ignored (12 cm at 10 m/s,
  small against 20-50 m).
- Ground resolution: 5-6 cm/px at 20 m, 13-17 cm/px at 50 m. At 50 m over a uniform field, texture
  is the real limit.
- At 45 deg tilt the far part of the usable image (60-75 deg off nadir) sees ground 2-4x farther
  away, with less parallax. Those features still help rotation and are weighted by their depth
  uncertainty automatically.

## 6. Architecture

```
            libcamera (CSI)                         FC over USB (MAVLink 2)
                 |                                          |
        [1 capture thread]                         [4 MAVLink I/O thread]
   frame + SensorTimestamp + exposure,             RX: HIGHRES_IMU 200 Hz (accel, gyro,
   gain metadata, drop if busy                     mag, baro), ATTITUDE_QUATERNION,
                 |                                 LOCAL_POSITION_NED, EXTENDED_SYS_STATE,
                 v                                 EKF_STATUS_REPORT, VIBRATION, ESC_TELEMETRY,
        [2 tracking thread]                        GPS_GLOBAL_ORIGIN, TIMESYNC, HEARTBEAT
   sky mask, CLAHE, gyro-predicted KLT,            TX: ODOMETRY 30 Hz, STATUSTEXT,
   forward-backward check, grid refill,            NAMED_VALUE_FLOAT, TIMESYNC,
   RS correction, frame metrics                    SET_MESSAGE_INTERVAL, SET_GPS_GLOBAL_ORIGIN
                 |                                          |
                 v                                          v
        [3 estimator thread]  <-------  IMU ring buffer (FC time mapped to Pi clock)
   IMU propagation, MSCKF update, landmarks,
   ZUPT on ground, weak baro update, td estimate,
   quality module (score, GOOD/DEGRADED/LOST),
   output: propagate to latest IMU sample -> ODOMETRY

        [5 logger thread]  raw recording to SD (frames, IMU, FC state), async, low priority
```

- State: attitude, position, velocity, gyro bias, accel bias, camera-IMU time offset `td`, baro
  bias, a window of 8-11 camera clones, up to about 25 landmarks. Intrinsics and extrinsics come
  from calibration and are fixed in flight.
- Camera: 1640x1232 binned, ISP downscale to 640x480 grayscale, 30 fps, about 150 tracked features
  on a grid.
- Camera model: chosen by the IMX219 calibration result. Radial-tangential if the distortion is
  moderate and fits to the corners, Kannala-Brandt (equidistant) if it is strong. Both are in the code
  behind one interface, the filter only works with unit bearing vectors.
- No full-image undistortion in the main pipeline: features are detected and tracked on the raw
  image, and only the feature points are undistorted (about 150 points, microseconds). The row of
  each point on the raw image is also exactly what the rolling-shutter correction needs.
- Alternative to be compared on recordings: a rectified pinhole crop of about 80-90 deg.

  | | Raw image + point undistortion (main) | Rectified crop 80-90 deg |
  |---|---|---|
  | Field of view | full ~120 deg | about 45-55% of the image area left (equidistant: (45/60)^2 for 90 deg) |
  | Features at low height, feature loss | more ground in view, more margin | fewer features, earlier loss |
  | Sky mask, tilt | needed: at 45 deg tilt the edge looks 105 deg from nadir | at 45 deg tilt the edge of an 85 deg crop is at about 88 deg, so the horizon is only at the very edge; the mask matters much less |
  | CPU on the Pi | point undistortion only | an extra `cv::remap` of the whole image every frame (about 1-2 ms, to be measured) |
  | Image | untouched | interpolated, edges stretched and softened |
  | Rolling shutter | row known per point | each output pixel comes from a different raw row; needs a row map |
  | Tracking at the edges | KLT windows see distorted patches at the far edge | uniform patches |

  The crop is the fallback if tracking on the distorted edges turns out poor on real recordings.
- IMU: `HIGHRES_IMU` from the FC (the board has 2 MB flash, so it is compiled in). Float values,
  no clipping (`RAW_IMU` would clip the gyro at 1877 deg/s because it sends int16 mrad/s). It also
  carries the calibrated magnetometer and the baro, so one message gives everything. We get the FC's
  filtered value, stamped at send time.
- IMU time model (numbers in ARDUPILOT_SETUP.md section 2, from the real filter settings):
  - gyro delay 4.8-9.9 ms: 80 Hz low-pass plus 8 per-motor notches whose delay depends on motor RPM;
  - accel delay 12.8-13.4 ms: 20 Hz low-pass, no notches;
  - accel samples are shifted by `accel_delay - gyro_delay` (config, set from the typical hover RPM)
    so both line up before integration;
  - the camera-IMU offset `td` is estimated online against the gyro. It absorbs the average gyro delay,
    the FC send lag and the camera timing. The RPM-dependent swing of up to 5 ms is left to the online
    estimate for now; the logged ESC RPM lets us add an explicit model later if replays show the need.
- Vibration and saturation: from the FC's `VIBRATION` message (levels and accel clipping counts at
  the full sensor rate). With `INS_GYRO_FILTER = 80` some vibration between 100 and 200 Hz folds into
  our 200 Hz gyro stream; the first hover recordings measure how much, and the gyro noise in the
  filter is set from that.
- Yaw: starts from the FC compass, then propagated by the gyro. A weak heading update from the
  calibrated magnetometer is planned, but off by default until the logs show how much the motors
  disturb the compass in flight.
- Clocks: everything on the Pi uses CLOCK_BOOTTIME (the libcamera SensorTimestamp clock, exposure
  start of the first row). FC time is mapped to it passively, from the FC timestamps in the messages
  and their arrival time (minimum delay plus a slow drift fit, RECORDING.md section 3). TIMESYNC can
  be added from Stage 3 if needed.

## 7. Output, quality states and who controls what

- How the covariance itself becomes dynamic, and the proposed thresholds between the states:
  section 13 (proposal, 2026-11-25). It would replace the time-based inflation below.
- GOOD: ODOMETRY with position, velocity, honest covariance, quality 50-100.
- DEGRADED: covariance inflated with time and score, quality 1-49. Velocity is sent as NaN once its
  sigma is more than about 3x `VISO_VEL_M_NSE`, because ArduPilot ignores our velocity covariance
  and NaN is the only way to stop it fusing a bad velocity. In 4.6.3 the vertical velocity is always
  fused together with the horizontal one, so the vz sigma counts in that test too.
- LOST (no good frames for longer than the configured time, default 2 s): ODOMETRY with quality -1
  and NaN position and velocity. ArduPilot does not fuse it. We keep sending, so VISO stays
  "healthy" on the FC, and our own recording shows why. Internally the filter keeps propagating, but
  nothing goes out.
- Recovery: for a short gap (under about 5 s, config) the filter restarts from the last good state
  propagated through the gap by the IMU, with its own grown covariance. For a longer gap IMU-only
  propagation is not trustworthy, so position restarts from the last good position with the
  uncertainty grown by `v_max * t_gap` (an honest bound), and velocity is re-estimated from the first
  good frames. Either way there is no jump against the last value the FC accepted, and
  `reset_counter` stays unchanged (EKF3 resets to our position anyway when it re-enters aiding). The absolute
  position error accumulated during LOST cannot be removed later (VIO is relative), so the covariance
  stays large. Only velocity and short-term accuracy recover fast. Target: GOOD again within 1-3 s
  after the features come back.
- What the module controls: only what it sends (data, quality, NaN). It never changes EKF source
  sets and never changes flight modes. This keeps one clear owner for each decision.
- What the pilot controls: the flight mode switch (Stabilize / AltHold / Loiter) and an EKF source
  switch (`RCx_OPTION=90`): "vision on" (set 1) or "vision off" (set 2, no horizontal source). The
  vision-off switch is for the case where vision looks fine to the module but the drone clearly
  drifts. It removes vision from the EKF completely, including its effect on the EKF tilt estimate.
- What the FC does on its own: only its standard EKF failsafe. After vision stops, EKF3 keeps
  claiming a valid position for 7 s, and the failsafe (`FS_EKF_ACTION`) fires about 1 s after that.
  A Lua script on the FC could react in under a second, but this FC has no SD card, so scripting is
  not usable on this hardware and is not part of the plan.
- So the protection against a vision loss is the pilot (procedure in ARDUPILOT_SETUP.md section 8).
  Two facts shape it:
  - the pilot cannot see the module's state in flight: with no telemetry radio the only display is
    the FC's OSD, and ArduPilot does not show a companion's STATUSTEXT there (it only writes it to the
    FC log). The pilot sees the FC's own messages ("EKF3 ... stopped aiding", "EKF Failsafe") and the
    drone's behaviour;
  - modes that do not need a position (AltHold, Stabilize, Acro) do not use vision for horizontal
    control.
- Three kinds of flights, from the current switch setup (`FLTMODE_CH = 7`: Acro, Stabilize, AltHold,
  no Loiter, ARDUPILOT_SETUP.md section 8):
  1. Recording only: the module records, the FC does not fuse vision (`VISO_TYPE = 0` or the source
     set without ExternalNav). Vision has no influence at all. All of Stage 1b and Stage 2 are this.
  2. Vision fused, flown in AltHold / Stabilize / Acro: vision cannot steer the drone horizontally,
     but it is not completely without effect. It feeds the EKF tilt estimate, and in 4.6.3 our
     vertical velocity is always fused, so it also feeds the EKF climb rate that AltHold uses. A bad
     vision velocity can therefore make AltHold lean slightly or hold height worse. The source switch
     "vision off" removes that.
  3. Vision controls the drone: Loiter (or PosHold) on the mode switch. Only when you decide to add
     it, after the simulation stage. Loiter is then for planned tests, low, close and with a hand on
     the mode switch. Any unexpected drift: switch to AltHold at once; if it still leans or drifts,
     switch the EKF source to "vision off".

## 8. Latency budget

Measured from SensorTimestamp (exposure start of the first row) to the ODOMETRY packet leaving the
Pi.

| Step | Budget |
|---|---|
| Exposure (half of 2 ms) | 1 ms |
| Sensor readout (8-bit binned ~12 ms; 10-bit ~23 ms) | 12 ms |
| CSI + ISP + buffer to user space | 8 ms (to measure) |
| Tracking (KLT 640x480, 150 features, mask, RS correction) | 8 ms |
| Filter update + quality | 10 ms |
| Propagate to latest IMU sample + pack + write to USB | 2 ms |
| Margin | 9 ms |
| Total | 50 ms |

Between camera updates we send IMU-propagated outputs, so the 30 Hz rate does not depend on the
frame rate. Build flags `-O2 -mcpu=cortex-a76`. Optimization only after `perf` profiling.

## 9. Repository layout

```
src/         C++ sources (capture, tracking, estimator, quality, mavlink_io, logger, main)
include/     headers
config/      vio.yaml (camera, extrinsics, quality thresholds, output),
             local.yaml (private values, not in git, template local.example.yaml)
tests/       the few key tests (C++ and Python, run with ctest)
tools/       gen_synthetic.py, metrics.py, prepare_euroc.sh, later align_gnss.py and calibration helpers
docs/        DESIGN, ARDUPILOT_SETUP, DATASETS, RECORDING, SIMULATION, CALIBRATION,
             GROUND_TRUTH, THIRD_PARTY, later FIELD_CHECKLIST
~/datasets/  EuRoC, TUM-VI (outside the repo)
third_party/ mavlink c_library_v2 (git submodule), any borrowed code with its license header
```

## 10. Risks

1. Firmware: a vendor or custom build of the official 4.6.3 tag (hash 92b0cd78) with locally added
   board files. We design for the 4.6.3 quirks (odd posErr formula capped at 10 m, ExtNav vz always
   fused, no origin parameters). The hash cannot show local edits beyond the board files, so the
   quirks we rely on get a bench check.
2. Scale drift on long smooth legs (section 4). Expect a few percent there, which conflicts with the
   1% targets unless the flight profile has regular accelerations. Measured on synthetic tracks
   (2026-11-26, section 11): a 5 km straight leg at constant speed ends 31-33 % off along the track
   and the covariance does not show it (14 sigma); S-turns every 40 s bring it to 2.2 % (1 % ATE over
   20 seeds), manoeuvres every 80 s or more do not help much; a loop at constant speed has the same
   bias (speed 1.4 times too high), with S-turns 0.5 % ATE. Worse than "a few percent" for smooth
   flights (corrected 2026-11-26; section 11).
3. Yaw: compass yaw on an FPV drone is disturbed by motor currents. 1 deg of heading error over
   2.5 km is 44 m (1.7%). Compass motor calibration (`COMPASS_MOT_*`) matters a lot.
4. The FC's IMU over MAVLink is the weak link: filtered, stamped at send time, shared USB link. Any
   USB hiccup is a gap in IMU data. The gyro delay moves by up to 5 ms with motor RPM (notches), and
   with the 80 Hz gyro low-pass some vibration aliases into our 200 Hz stream. A separate IMU on the
   Pi would be clearly better.
5. USB link in the field: a Type-C connector under vibration and ESC noise can disconnect, and the
   Pi can back-power the FC over VBUS. Short secured cable, consider cutting VBUS, auto reconnect.
   On the FC side nothing reacts quickly to a disconnect (see risk 11).
6. No absolute recovery: after LOST, the absolute position error stays. Every LOST event adds
   directly to ATE and loop closure error.
7. Latency: readout plus ISP use about 20 ms of the 50 ms. Must be measured on the Pi first.
8. Low light and water: dusk with a 2 ms exposure is noisy, longer exposure blurs at 45 deg tilt
   maneuvers. Expect DEGRADED or LOST; the "excellent" robustness grade is a stretch.
9. Pi 5 power and heat: brownouts on throttle punches, thermal throttling in a closed frame. 5V 5A
   regulator, heatsink with airflow, `vcgencmd get_throttled` in the quality module.
10. Ground truth: without RTK or PPK the 100 m RPE "excellent" grade and the hover test cannot be
    verified properly (see GROUND_TRUTH.md).
11. Slow reaction to a vision loss: no FC script is possible (no SD card), the FC itself reacts only
    after about 8 s, and the pilot cannot see the module's state in the goggles. In Loiter the drone
    can hold a drifting position for those 8 s. Mitigation is procedural: AltHold by default, Loiter
    only for planned hover tests.
12. No FC logs (until confirmed): FC-internal data (VISP/VISV, per-axis EKF innovations, full-rate
    IMU) is not available. We record the MAVLink equivalents on the Pi (RECORDING.md section 2) and
    verify FC-internal behaviour in SITL on the 4.6.3 tag.

## 11. Work plan

Stage 1 is split. The algorithm comes first, on the laptop, with no hardware.

Stage 1a, offline VIO (laptop only, see DATASETS.md):

1. Replay and metrics: a small synthetic data generator (trajectory, IMU, feature observations,
   ground truth), the session reader and replay loop, and the metrics tool (ATE RMSE / max, RPE on
   100 m segments, loop closure error), tested on trajectories with known errors. Then the EuRoC
   reader, checked on the one EuRoC sequence when it is there.
2. Front end: camera models (radial-tangential and Kannala-Brandt), KLT tracking with forward-backward
   check, grid refill, gyro prediction. Camera models tested against OpenCV (DATASETS.md section 3).
   Done 2026-11-24, results below.
3. Filter: IMU propagation, MSCKF update, landmarks, time offset, static initialization.
4. Quality module: score and GOOD / DEGRADED / LOST, fault injection, "FC IMU" degradation mode.
5. Results on synthetic data and the EuRoC sequence.

### Step 2 result (front end)

- Code: `camera_model` (both models), `frontend` (gyro rotation, gyro prediction, rolling-shutter
  correction, sky mask, image metrics, tracker), tools `vio_track` and `eval_tracks.py`,
  `render_ground.py` for rendered down-looking sequences.
- Per frame: mask, image metrics, optional CLAHE, pyramid, gyro-predicted KLT forward and backward,
  refill on the 8x6 grid. Refill fills the cells level by level (every cell gets one feature before
  any gets a second). Detection runs when fewer than 90 % of the features are left, or every 5th
  frame while a usable cell is empty (an empty cell usually has no texture, retrying it every frame
  cost 4-5 ms for nothing). Corners are searched on the half-size image: on rough ground almost
  every full-size pixel is a corner candidate and detection took 10-20 ms; at half size it takes a
  fraction of that, with the same or better tracks.
- CLAHE is off by default: on EuRoC and on rendered ground it gave no better (slightly worse)
  tracks and cost 2 ms per frame. The switch stays for our own recordings.
- No RANSAC in the front end: the forward-backward check catches failed tracks, the filter's chi2
  test the rest. Whether that is enough is decided by the numbers in DATASETS.md section 2.
- Output: feature id and raw pixel, the same `features.csv` format as the synthetic sessions.
  Undistortion and the rolling-shutter correction are functions on the raw pixel, used by the
  filter, so file replays and the live system agree. Frame time convention for the rolling shutter:
  row time = time of the reference row + (row - reference row) * `line_time_us`.
- Numbers on EuRoC and the rendered sequences: DATASETS.md sections 2 and 5. Short version: about
  2.5 % wrong tracks over 10 frames on EuRoC, 0.1 % on rendered ground (also at low contrast),
  6.4 ms per frame on the laptop.

### Open items from step 2

1. Slow KLT drift on long tracks: frame-to-frame KLT drifts by up to a few pixels over hundreds of
   frames. EuRoC: 2.5 % of tracks wrong over 10 frames, about 35 % over their whole life. Rendered
   ground: 0.1 % and 1-3 %. Harmless for the MSCKF window, but the long-lived landmarks for hover
   (section 5) need a guard in step 3: check them against their first patch now and then, or let
   the chi2 test drop and re-initialize them.
2. CLAHE: off by default now (see above). Re-check on our recordings, where exposure changes and
   sun and shadow might change the picture.
3. CPU on the Pi (point A): 6.4 ms per frame mean, 8.4-9.4 ms p95, on the laptop, one thread (752x480
   and 640x480). The Pi 5 is probably 2-3x slower per core, which is over the 8 ms budget of section
   8. KLT forward plus backward is two thirds of it. Options if needed: KLT window 15 instead of 21
   (about half the KLT cost), backward check only every other frame, fewer pyramid levels.
4. Image check at the sky mask boundary (section 5), Stage 2.
5. Not testable before our recordings: rolling-shutter correction on real images (only tested on
   synthetic geometry), sky mask on a real horizon, real blur and exposure changes.

### Step 3 progress (filter), 2026-11-24

Stages 1-3 are done (propagation, static start, zero-velocity updates; clones and MSCKF update;
landmarks and drift handling). Numbers: DATASETS.md section 6. Deviations from the approved plan:

- Right-invariant errors (filter.h) instead of first-estimate Jacobians (FEJ), approved. FEJ as
  implemented made the filter inconsistent on synthetic Monte Carlo runs (NEES about 200, speed
  biased 30 % high); the plain EKF with ordinary errors was honest on synthetic runs but on EuRoC
  gained fake yaw information at takeoff (25 deg yaw error, sigma 0.7 deg). With invariant errors
  yaw keeps its prior uncertainty, as it must, and EuRoC ATE fell from 3.6 m to 0.33 m. Invariant
  errors are consistent by construction for the four directions VIO cannot observe, at any
  linearization point. Landmark errors use the same form, tied to the IMU attitude error.
- "Still" for zero-velocity updates needs the image too (median feature motion under 1 px): the IMU
  alone cannot tell rest from flight at constant velocity, and a smooth glide would otherwise get
  zero-velocity updates. On the drone the FC "landed" flag will be added.
- The visual update runs on every 6th frame (5 Hz at 30 fps), the tracker on every frame. With an
  update every frame the 11 clones span only 0.37 s, and slow flight and the approach to a hover
  have no parallax at all (hover drift of km per minute). EuRoC (20 fps, 3-5 m depth) needs every
  3rd frame. A motion-based rule (new clone when the image moved enough) should replace the fixed
  stride later.
- Each feature is used once in the MSCKF; the tracker retires features after 72 frames (one window)
  and re-detects them (resets KLT drift). The proposed cap of 30 frames left no track long enough
  for a landmark at the new stride.
- Landmarks for hover: made below 4 m/s from tracks with at least 3 deg of parallax, dropped when
  their viewing direction turned more than 5 deg (the drift guard: KLT drifts when the view
  changes), after two chi2 failures in a row, or after 60 s. A fixed 10 s age limit killed all
  landmarks early in the hover. In replays with images the tracker runs inside `vio_replay` and is
  told which features to keep, as in the live system.

### Review fixes (2026-11-25)

Evaluation is now repeatable: `tools/eval_runs.py` runs a replay over six start times or several
seeds and reports median and range; `config/replay_render.yaml` holds the 1 px pixel noise for the
renders. Results before and after: DATASETS.md section 6.

Frame time reached by holding the last IMU sample (fixed). The IMU is now interpolated at the frame
time (`Filter::propagate_to(t, next)`). Cause of the EuRoC failure with camera stamps 5 ms earlier
and td fixed at 5 ms: EuRoC camera and IMU are synchronized, so every frame lands exactly on an IMU
sample, but the old order processed the frame before that sample was given to the filter. The
filter then held the sample 5 ms older over the whole gap. In flight that adds 5.8 mm/s of velocity
error and 0.23 mrad of attitude error per frame (rms, measured on MH_01): random, unmodeled, and in
attitude about 6 times the gyro noise the filter assumes for that IMU (1 against 0.17 mrad per
square-root second), adding up to 14 mrad over the flight. The filter became confident in a wrong
attitude, rejected good features, and on some start times diverged (aligned ATE up to 359 m). After
the fix the shifted runs give exactly the numbers of the original data. Test:
`test_frame_between_samples`.

Online camera-IMU time offset (partly fixed):

- Clipping fixed. Frames are now delivered at their time in the IMU clock, frame time + td, for
  either sign of td (`replay_in_imu_time` in session.cpp), so the clone is always taken at that
  time. Late frames are counted and reported by `vio_replay`: 0 on EuRoC and the renders (before:
  921 of 1075 and 110 of 128 update frames). Test: `test_frame_order`.
- The td Jacobian (`clone_td_jacobian`, tested against a numeric derivative in `test_td_jacobian`)
  used a single raw gyro sample. Its noise looks like rotation, and the filter believed it learned td
  where it cannot. It now uses the gyro mean over 30 ms centred on the frame time
  (`imu.td_rate_window_ms`; a past mean lags and biases td, as the review found). This needs the IMU
  15 ms after the frame, a small extra delay in the live system.
- `imu.td_estimate_online: false` now keeps td at `td_initial_ms`.
- Recovery of known offsets, high-rotation figure eight (`--speed 12 --radius 50 --height 25
  --laps 3`), 8 seeds each, start at 0 with a 30 ms prior:

  | true td | error after, median (range) | sigma | inside 3 sigma | within 1 ms | error before, median (range) |
  |---|---|---|---|---|---|
  | -20 ms | +1.8 (-1.0..+4.8) ms | 1.2 ms | 7/8 | 4/8 | -2.9 (-11.2..+2.3) ms |
  | -10 ms | +1.1 (-1.1..+4.5) ms | 1.2 ms | 7/8 | 3/8 | +0.1 (-9.1..+5.2) ms |
  | 0 ms | +0.7 (-1.3..+4.4) ms | 1.2 ms | 7/8 | 3/8 | -0.4 (-7.3..+1.4) ms |
  | +10 ms | +0.7 (-3.4..+3.6) ms | 1.2 ms | 8/8 | 2/8 | -6.6 (-12.0..-1.3) ms |
  | +20 ms | +0.4 (-3.3..+3.1) ms | 1.2 ms | 8/8 | 3/8 | -10.3 (-15.9..-6.2) ms |

  The error is now within about 5 ms everywhere and mostly inside 3 sigma, but the 1 ms target is
  not met (15 of 40). It is not reachable on this data at all: with the true rate in the Jacobian
  (an experiment using the ground truth) sigma is 1.7 ms and 7 of 24 runs are within 1 ms. The
  generator uses the drone's placeholder gyro noise, which is large.
- Not solved: false confidence on gentle motion. On the gentle figure eight td cannot be observed
  (noise-free data keeps the 4.8 ms prior). With IMU noise sigma still falls to 1.5 ms (before:
  0.6 ms), and errors reach 10 ms (5 of 8 inside 3 sigma). With the true rate sigma stays at 4.4 ms,
  so the rest is the remaining gyro noise in the averaged rate. A 100 ms window keeps all 32 tested
  runs inside 3 sigma (gentle: sigma 2.4 ms) but is still twice too confident on gentle motion and
  smooths real fast rotations. Options are in the report; the default stays at 30 ms until you
  decide.

Results depend on the distance from the world origin. Reproduced: figure eight, 8 seeds, ground
truth moved 0 / 1 / 3 / 5 km: median ATE 1.4 / 2.2 / 4.9 / 9.0 m, position NEES 3.1 / 5.8 / 23 /
67; the fixes above do not change it. Likely cause (not proven): the right-invariant errors rotate
positions about the world origin (p_true = Exp(dtheta) p + dp). For a feature at distance |p_f|
from the origin the pixel prediction then has a second-order term of about 1/2 |dtheta|^2 |p_f|
that the linear (EKF) model ignores; with ordinary errors it would be 1/2 |dtheta|^2 |p_f - p|, a
few tens of metres. At 5 km with the measured attitude errors (mostly yaw, which drifts: 0.29 deg
median, 0.52 deg p90) that is 0.7-2.3 px at 30 m, more than the 0.5 px pixel noise. What supports
it: with ten times less IMU noise (smaller attitude errors) the effect almost disappears (NEES 2.9
at 0 km, 6.1 at 5 km), as the square of the attitude error predicts. What does not confirm it:
running the filter in a frame centred on its start gives at 5 km exactly the 0 km result, but that
only gives the filter the same input as the 0 km run; it shows that absolute coordinates matter,
not why.

Moving origin (approved 2026-11-25, implemented, `filter.h`). All positions inside the filter (IMU,
clones, landmarks, the landmarks' birth positions) are kept relative to an origin. It starts at the
start position, and whenever the drone is more than `filter.origin_shift_distance_m` (100 m) from
it, it moves to the current position, height included (the term above grows with any distance; the
height part is small but there is no reason to treat it differently). Every internal position
decreases by the shift d, velocity does not change. The invariant position errors change by
dp' = dp - [d]x dtheta (the IMU attitude error for the IMU and the landmarks, each clone's own for
the clones), applied once to the covariance. This is not exact: from p_true - d = Exp(dtheta)
(p - d) + dp' the exact change is dp' = dp + (Exp(dtheta) - I) d, and the shift keeps only its first
order. The neglected part, 1/2 dtheta x (dtheta x d), is at most 1/2 |dtheta|^2 |d|: about 1.4 mm
for an attitude error of 0.3 deg at d = 100 m (0.14 mm at 10 m, 14 mm at 1000 m), far below the
position uncertainty. The means move exactly, and the covariance of the ordinary errors
(p_true = p + dp) is unchanged by the shift (test `test_origin_shift`: world positions and that
covariance agree to 5e-13 before and after). The output adds the origin back, so nothing jumps for
ArduPilot. A shifted start alone is not a valid test of this (a moving origin passes it trivially);
the test is long flights, far from the start, with the result independent of the threshold
(DATASETS.md section 6). Results: on a 24 km figure eight that gets 4 km from its start (4 seeds)
ATE 224 m before, 85 / 85 / 83 m with the origin moved beyond 10 / 100 / 1000 m, position NEES 15
before, 4.4-4.6 after; on 9 km flights within 1.5 km of the start 40 m before, 38 m after for all
three thresholds, and the same with the ground truth moved 5 km (188 m before). No measurable cost.
The short figure eight, the normal-contrast and hover renders and EuRoC stay within their earlier
spread. The low-contrast render seemed to move (six-start position NEES 12 to 18) without any
shift happening; over 20 starts it is 8.7 before and 10.5 after, while pixel noise 0.999 / 1.001
px alone moves it between 8.9 and 10.5 on the same code: no measurable change (corrected
2026-11-26, review; DATASETS.md section 6).
A first version passed the shift distance by reference while changing the position it referred
to, so the origin never moved: ATE about 1.2 km on every long flight. The test now also shifts to
the current position the way the replay does, and fails on that version.

**Drift-aware feature noise, part a of section 13 (approved 2026-11-26, implemented, off by
default).** The 13.3 baseline was measured again on today's code over 20 start times, the
acceptance test (13.8) now uses 20 runs, quantiles and a noise band, the drift model uses the view
change accumulated along a track (it fits the true pixel error better than the change since the
first observation, 13.4 a), and the generator makes KLT-like drift with a known q. With the switch
off the output is identical to before. On synthetic drift with the true q the filter is honest at
every q (NEES about 3, today's filter 9-64); on the low-contrast render position NEES goes from 10
to 4.9 with a q measured on the normal render; the hover render gets worse (drift 0.22 to 0.67
m/min). Details and numbers: section 13.11.

**EuRoC divergence at liftoff (2026-11-26, investigated, not fixed: it needs a design choice).**
With part a off and online td, starts 24 to 27 s every 0.05 s: 9 of 61 runs end with an aligned
ATE above 1 m (24.4, 24.6, 24.75, 24.9, 25.75, 25.9, 26.2, 26.4, 26.7), 5 of 61 with part a on, as
the review measured. Starts 21 to 24 s: also 9 of 61 (21.6, 21.85, 22.15, 22.35, 22.8, 22.9,
22.95, 23.1, 23.65). Runs and scripts: `~/datasets/analysis/euroc_divergence_2026-11-26/`.

What happens. Shown on run 24.75 against its healthy neighbour 24.8, with a diagnostic copy of the
code that prints every track and update (same trajectories, file for file), and over all 61 runs:

1. From 38.8 s the rotors run while the MAV stands on the floor; it lifts off at about 45.5 s. The
   accel spread is 0.3-1.7 m/s^2 (0.03-0.1 when still), so the still detector says "moving" and
   there are no zero-velocity updates. The camera does not move, so no track has parallax (630-650
   tracks without geometry). For about 7 s the filter runs on the IMU alone.
2. At liftoff the velocity is off by about 0.1 m/s, mostly vertical, about 2 sigma (velocity NEES
   about 7): more than the IMU noise model allows, consistent with vibration it does not model. The
   motion inside the 1.65 s clone window is off by 0.18 m (filter 0.34 m, ground truth 0.16 m).
   The velocity sigma grows as the noise model says; no coding error there.
3. The first tracks with parallax are very close. Triangulated with the ground truth poses they lie
   0.2-0.25 m from the camera with 1-3 px residuals: the tracks are right. With the filter's poses
   the 0.18 m error is as large as the depth, the points land at 0.12-0.17 m and the residuals are
   20-50 px. The linear model of the update is not valid there.
4. Most of these tracks fail the chi2 gate, a few pass: so close, the pose part of their predicted
   covariance is large, and each track is tested alone. Together they are inconsistent (24.75: 7
   tracks, chi2 950 on 59 rows), and this one update moves the accel bias by 0.42 m/s^2 and td by
   -7 ms.
5. Afterwards nearly every track fails the gate; the filter runs on the IMU with a wrong bias and
   diverges. Nothing brings it back.

Over the 61 runs (proven as counts): the first accepted update after liftoff uses tracks 0.12-0.4 m
away in nearly every run. In 8 of the 9 diverging runs it is jointly inconsistent (chi2 per row
11.6-35.8, bias change 0.36-0.68 m/s^2); 15 healthy runs have such an update too and recover
(healthy median chi2 per row 2.0). Run 26.2 goes wrong later. What decides whether a run recovers
after such an update is not established (guess: whether well-conditioned tracks arrive before the
bias error has grown too large). No coding error was found: propagation, gate and update do what
they are designed to do. The gap is in the design: a per-track gate only, no check that a point
is far compared with the pose uncertainty, no zero-velocity update with motors running, and no
recovery when every track is rejected.

Our drone: exposed, probably more (reasoning, not measured; our synthetic sessions start in
flight). It also stands armed with spinning props (vibration, no zero-velocity update) for as long
as the pilot waits, then lifts off. Under a nadir camera the ground is 0.05-0.2 m away at liftoff,
closer than in EuRoC, and the depth grows from centimetres to metres within a second or two, with
very large image motion.

Options. None is implemented. Diagnostics were run only in a throwaway copy of the code
(`diag_src` in the folder above, switched by environment variables); part a off, online td:

| Option | What | Diagnostic, runs above 1 m: starts 24-27 / 21-24 (today 9 / 9) | Cost | Risk |
|---|---|---|---|---|
| A. Depth guard | skip tracks whose point is closer than d_min (0.5 m tried) | 1 (25.45: 1.2 m) / 2 (22.45: 2.4 m, 23.9: 1.3 m) | a few lines | a nadir camera gets no updates until about d_min above the ground (a second more on the IMU); a fixed d_min does not scale with the pose uncertainty, a ratio (depth against the position sigma inside the window) would |
| B. Joint gate | test the stacked update as a whole, skip it if it fails | 1 (25.45: 3.8 m) / 1 (22.35: 1.5 m) | one line | throws away the good tracks with the bad; after a long gap it may reject everything for a while (25.45: 15 s) |
| C. Zero-velocity update on the ground with motors running | still image plus the FC's landed state (`EXTENDED_SYS_STATE`) instead of the IMU spread | not testable on EuRoC (no landed flag) | small; the recorder must log the message (Stage 1b) | relies on the FC's landed detector; a still image alone is not enough, a hover looks still too |
| D. Recovery | when every track fails the gate for N updates, enlarge the velocity and bias covariance, or restart | not tested | small to medium; belongs to the quality module (step 4) | can hide real faults; must show in the quality state |
| E. Motion-dependent IMU noise (13.4 b) | more accel noise with vibration | review: 10 times the noise, 4 of 61 still above 1 m | in the design, on hold | not enough alone |

Neither A nor B alone gets to 0: in the runs they leave (25.45 with both) the state goes wrong
anyway and every track is rejected for a while; only something like D brings it back.

**Takeoff robustness (2026-11-26: B and C approved and implemented; A not wanted, D goes to step 4,
E stays on hold).** Runs and scripts: `~/datasets/analysis/takeoff_2026-11-26/` (`run_liftoff.sh`,
`liftoff_metrics.py`, `run_regress.sh`, `compare.py`).

- B, joint gate (`msckf.cpp`): the stacked MSCKF update (after QR compression, as in the
  diagnostic) gets the same chi2 test as a single track; if it fails, the whole update is skipped
  and the output column `update_rejected` says so. Landmark updates and landmark births are gated
  as before.
- C, zero-velocity update with the motors running (`Filter::zero_velocity_update`, replay
  `mav0/landed0/data.csv`): v = 0 when (1) the newest FC landed message at or before the frame says
  "on the ground" and is at most `filter.zupt.landed.max_flag_age_s` (0.1 s) old; (2) the image does
  not move: if features are seen in this and the previous update frame, their median motion must be
  below `image_motion_max_px` (1 px), and without features the image says nothing; (3) the mean
  specific force since the last update frame has the magnitude of gravity within
  `filter.zupt.landed.accel_max` (0.3 m/s^2): on the ground the true acceleration is zero whatever
  the vibration (EuRoC on the floor with the rotors running: at most 0.11 m/s^2 over 0.15 s; at
  liftoff 0.5-1.8). The IMU spread is not used. Only the velocity is updated (the gyro mean is not a
  clean bias with the motors running), and the update is not chi2 gated: the vibration is not in the
  noise model, the predicted velocity is too confident on the ground, and a gate rejected the true
  v = 0 (unit test: 7 of 25 applied with the gate, 23 of 25 without). The output column `static`
  is 1 for a still update, 2 for a landed one. Sessions without a landed file (EuRoC, all older
  synthetic ones) are not affected.
- When the FC sets and clears the flag (4.6.3 source): ARDUPILOT_SETUP.md section 2. In Stabilize and
  Acro it clears as soon as the throttle is above zero after the spool-up, normally before the
  liftoff, but a fast throttle push from zero can lift off during the spool-up, before the flag
  clears. In AltHold it clears during the takeoff throttle ramp, normally after the liftoff. So no
  rate and no hold-off make the flag safe on its own: the hold-off only bounds how long after the
  flag clears an update can still happen (0.1 s plus the USB delay), the rate only how often an
  update can happen on the ground (2 Hz with 0.1 s: about one update frame in three; 10 Hz: nearly
  all). The image and the accelerometer must stop the updates after the liftoff.

Liftoff scenario (synthetic, 20 seeds; DATASETS.md section 1): armed on the ground for 7 s with
vibration the filter does not know, camera 0.05-0.2 m above the ground, climb to 25 m at 2 m/s,
then 20 s north at 8 m/s; with features near the ground ("near") and with none below 1 m
("blind"). Flag at 2 Hz, clearing 0.3 s before the liftoff unless stated. Velocity error at the last
output before the liftoff, aligned ATE over the whole run; median (range):

| Filter | near: velocity error at liftoff | near: ATE aligned | blind: velocity error | blind: ATE aligned |
|---|---|---|---|---|
| today | 0.83 m/s (0.47-1.7) | 10.3 m (2.2-349) | 1.6 m/s (0.50-2.4) | 452 m (4.4-2555) |
| B | 0.81 m/s (0.47-1.7) | 8.0 m (1.1-74) | 1.6 m/s (0.50-2.4) | 23 m (5.7-231) |
| B + C | 0.12 m/s (0.03-0.21) | 1.5 m (0.23-3.6) | 0.11 m/s (0.03-0.31) | 2.0 m (0.22-3.2) |
| B + C, flag 10 Hz | 0.10 m/s (0.03-0.18) | 1.7 m (0.28-4.3) | 0.09 m/s (0.03-0.18) | 1.8 m (0.29-4.2) |
| B + C, flag 0.5 s late (AltHold) | 0.04 m/s (0.003-0.18) | 1.4 m (0.27-3.1) | 0.006 m/s (0.003-0.13) | 1.2 m (0.18-4.3) |
| B + C, late, 10 Hz | 0.009 m/s (0.002-0.08) | 2.0 m (0.47-3.9) | 0.009 m/s (0.002-0.08) | 1.1 m (0.22-3.7) |

- No landed update happened while the drone was really moving (true speed above 0.01 m/s) in any
  of the 160 B + C runs, also with the late flag and no features, where only the accelerometer can
  stop it. The few counted "after the liftoff" were the update frame at the liftoff instant, moved
  by up to 3.5 ms through the online td, with the drone still at rest.
- A late flag gives a smaller error at the liftoff than an early one: the updates go on until the
  drone moves, while an early flag leaves 0.3 s or more on the IMU alone.
- The before numbers are pessimistic in one way: the scenario starts armed (no still phase), and the
  generator's IMU noise (0.57 m/s^2 per sample) is above the still detector's limit anyway, so no
  still update ever happens in generated sessions. On the drone the motors-off phase before arming
  gets still updates.
- The synthetic vibration is a model: white noise is harsher on the 0.2 s mean than EuRoC's real
  vibration, which is why 2 of 25 updates fail the accelerometer check in the unit test. Real
  values: our recordings with the motors running on the ground.

EuRoC, 121 start times 21-27 s every 0.05 s (the review's 122 counts 24.00 twice), online td: above
1 m aligned ATE today 18, with B 2 (22.35: 1.5 m, 25.45: 3.8 m); 90th percentile 1.60 to 0.24 m,
worst 1830 to 3.8 m, median 0.22 to 0.21 m, position NEES median 78 to 67. C cannot act there (no
landed flag). Why the two still fail: in both, a series of updates that pass the joint gate easily
(chi2 per row 0.6-2.0) moves the accel bias by 0.3-0.5 m/s^2 within 1-4 s after the liftoff
(22.35: at 49.2-49.5 s, while the first landmarks are made; 25.45: at 45.7-46.2 s). The joint gate is
right to pass them: the wrong bias is inside the filter's own covariance after 7 s on the IMU alone.
Afterwards nearly every track fails its gate for 10 s (22.35) or 25 s (25.45), the velocity error
grows to 1.2 and 2.0 m/s, and both recover by themselves (65 s and 80 s). Neither B nor C can stop
this; it is the missing recovery (D, step 4), and the 7 s on the IMU alone that C removes on our
drone.

C on EuRoC, a diagnostic only (EuRoC has no landed flag; a copy got one from the ground truth: on
the ground at 2 Hz until 0.3 s before the liftoff at 45.21 s, `euroc_with_flag/`): median aligned
ATE 0.16 m (B alone 0.21), velocity at the liftoff about 0.01 m/s instead of 0.12 (start 24.65), and
no landed update with the drone moving faster than 0.03 m/s (the ground truth itself reads up to
0.02 m/s on the floor). But 11 of 121 starts end above 1 m (B alone 2), worst 951 m. In 24.65 the
first visual updates after the liftoff, from points 0.2 m away, move the accel bias by 0.3-0.4
m/s^2 and pass the joint gate, then the filter drifts away. Not the clone and track handling: with
landed frames not counted as "still" for the MSCKF, again 11 of 121. Not established why C makes
this more likely; a guess: the many ungated velocity updates leave the filter too sure of the
velocity and the attitude at the liftoff, so wrong near-point updates look consistent. This does
not show on the synthetic liftoff, whose near points are exact. Open question to you (below).

Review of the takeoff stage (2026-11-26). B confirmed (EuRoC 121 starts: 2 above 1 m) and the sharp
synthetic liftoff numbers reproduce. Three findings, recorded here as the review measured them:

1. EuRoC with the ground-truth landed flag, 121 starts: without C 2 above 1 m (worst 3.8 m); with C
   and `filter.zupt.velocity_noise` 0.01 / 0.05 / 0.1 / 0.3 m/s: 11 / 3 / 5 / 3 above 1 m, worst
   951 / 802 / 1336 / 41 m. Overconfidence from the velocity updates is at most part of the cause;
   the guess above is not enough.
2. "No landed update happened while the drone was moving" holds only for the sharp liftoff. In a
   gentle takeoff (climb 0.3 m/s, 2 s velocity step, 6 m, no features below 1 m, 10 seeds, flag at
   10 Hz) landed updates happen while moving in 8 of 10 runs with the flag 1.0 s late and in 10 of
   10 with it 2.0 s late, up to 1.2 s after the liftoff. The image cannot see it (no features) and
   the accelerometer check cannot either (the climb acceleration is below 0.3 m/s^2). The update is
   not chi2 gated (`src/filter.cpp`, `zero_velocity_update`), so nothing else stops it.
3. The gentle takeoff is not solved: with C the median ATE is 5-7 m (worst 42-64 m), without C
   about 67 m.

Your decision (2026-11-26): C off by default (`filter.zupt.landed.enabled: false`), the code stays,
no more work on it until our own takeoff recordings. `EXTENDED_SYS_STATE` at 10 Hz.

Regression, today against B + C, medians with the noise band (pixel noise 0.999 / 1.001 px on the
renders, 0.4995 / 0.5005 px on the synthetic sets, 13.8); C does nothing on these sets (no landed
flag), so this is B. The three renders (20 starts): the same, inside the bands (normal and hover
render identical). Short figure8 (8 seeds), aligned ATE 0.81 m [0.79-0.82] against 0.79 [0.77-0.82]:
no change. Long flights 9 km (8 seeds) and 24 km (4 seeds), aligned ATE 27.2 m [27.0-27.4] against
26.5 [26.5-27.2] and 67.7 [67.7-68.2] against 67.9 [66.7-68.0]: no change. Outside the bands, all
small: approach_hover (20 seeds) hover drift 0.120 [0.116-0.120] to 0.111 [0.111-0.115] m/min and ATE
0.262 to 0.257 %, better; 9 km position NEES 4.39 [4.30-4.43] to 4.18 [4.18-4.28] and attitude 3.05
to 2.82, better; 24 km attitude NEES 3.84 [3.81-3.87] to 3.93 [3.93-3.98], 3 % worse.

**Stage 4 of step 3: baro, takeoff options, km-scale scale drift (2026-11-26).** Runs and scripts:
`~/datasets/analysis/step3_stage4_2026-11-26/` (`run_stage.sh`, `liftoff_metrics.py`,
`scale_metrics.py`, `bands.py`, `montecarlo.md`). All of it rests on synthetic data only: the baro
model, the vibration, the takeoff and the flat textured ground are guesses or idealisations, and
the feature tracks of the generator have no drift. Nothing here says how our drone will do.

- Baro in the filter (`Filter::baro_update`, section 4): altitude = up . (position + moving origin)
  + baro bias + noise, one update per update frame with the newest baro sample, chi2 gated. The
  first reading sets the bias; then it does a random walk of 0.05 m per sqrt(s). Noise 1 m, times
  3 (`filter.baro.inflate_factor`, a guess) below 3 m above the takeoff height or above 2 m/s
  climb or sink. The takeoff height is the height at a start from rest; a start in flight has none.
  The attitude term of the Jacobian uses the position relative to the moving origin, so it stays
  small. Sessions without `mav0/baro0/data.csv` give the same output as before, file for file (88
  replays checked: figure8, liftoff, approach_hover, a render, EuRoC). Unit test: the baro keeps
  the height of a drifting IMU (0.015 m against 30+ m without), finds its offset, and leaves the
  horizontal sigma unchanged (0.5 %).
- Generator: `--baro` (model in DATASETS.md section 1, a guess), `--climb-speed`, `--climb-step`
  (liftoff), scenarios `line` and `loop` with `--manoeuvre none | speed | sturn`.
- C (landed update) is off by default from this stage on, so the takeoff numbers are without it.

Takeoff, 20 seeds, aligned ATE median (range), position NEES median, noise band in brackets
(pixel noise 0.4995 / 0.5005 px). Sharp: climb 2 m/s to 25 m; gentle: 0.3 m/s to 6 m with a 2 s
velocity step; landed flag at 10 Hz (not used, C off):

| Takeoff | without baro | with baro |
|---|---|---|
| sharp, features near the ground | 8.0 m (1.1-74) [7.8-8.0], NEES 201 | 4.5 m (0.86-88) [4.5-4.7], NEES 55 |
| sharp, no features below 1 m | 23 m (5.7-231) [23-25], NEES 447 | 15 m (3.1-333) [13-15], NEES 80 |
| gentle, features near the ground | 18 m (4.1-204) [17-18], NEES 890 | 7.3 m (3.2-281) [6.8-7.9], NEES 137 |
| gentle, no features below 1 m | 93 m (14-611) [82-93], NEES 591 | 51 m (6.0-720) [51-70], NEES 94 |

The baro halves the takeoff error and cuts the position NEES by 4-7 times, but the takeoff stays
far from honest and far from accurate: the velocity error at the liftoff is still 0.8-1.6 m/s
(7 s on the IMU alone with vibration), the baro near the ground is weak by design (noise times 3).
Worse with the baro: the attitude NEES of the gentle takeoff without near features, 13 [12-14] to
20 [20-22].

Scale drift, 5 km at 12 m/s and 40 m (420 s), sparse ground points (0.004 per m^2), 8 seeds, baro
on unless stated. Along-track error and the along-track sigma the filter reports, in % of the path
at 5 km, and their ratio; median (range):

| Flight | along-track error | along-track sigma | error / sigma | without baro: error |
|---|---|---|---|---|
| straight, constant speed | 31 % (26-41) | 2.3 % | 14 (12-17) | 33 % (26-41) |
| straight, speed +-20 % every 40 s | 12 % (11-13) | 1.1 % | 12 (9.9-13) | 7.6 % (6.0-9.2) |
| straight, S-turns 30 m every 40 s | 2.2 % (0.77-3.1) | 0.56 % | 3.9 (1.4-5.5) | 2.1 % (0.83-3.1) |
| loop R 800 m, constant speed (misleading, see below) | 0.63 % (0.07-4.4) | 0.79 % | 0.82 (0.08-5.5) | 0.57 % (0.26-4.0) |
| loop, speed changes (misleading, see below) | 0.76 % (0.18-1.9) | 0.59 % | 1.3 (0.30-3.3) | 0.49 % (0.19-1.1) |
| loop, S-turns (misleading, see below) | 0.42 % (0.15-0.98) | 0.42 % | 1.0 (0.36-2.3) | 0.39 % (0.15-1.0) |

(The loop rows are not a scale measure: at 5 km the loop of R 800 m is almost closed. Scale on
loops: `tools/scale.py`, and the 20-seed table in DATASETS.md section 7.)

- Error growth: on a straight leg at constant speed the error grows faster than the distance (2.0 %
  at 1 km, 16 % at 3 km, 33 % at 5 km): the along-track velocity drifts, as section 4 expects when
  nothing accelerates, but much faster than "a few percent". On the loop the along-track error at
  5 km is under 1 %, but that says nothing there: 5 km is almost one full lap, and a circle that is
  too large comes back near its start. Its horizontal error (7 % of the path at 5 km, 8 % at 3 km)
  is the same scale error, seen across the track (corrected 2026-11-26, review).
- The covariance does not show the growing scale uncertainty on straight legs: the along-track
  sigma stays at 1-2.3 % of the path while the error reaches 12-33 % (12-14 sigma at 5 km). Position
  NEES 56 on the straight constant-speed leg. With S-turns 3.9 sigma. The "about 1 sigma on loops"
  measured here was wrong for the same reason (one lap at 5 km): the loop at constant speed has the
  same speed error (1.40 times the true speed at the end) and position NEES 33. So the filter claims
  scale information it does not have whenever the flight does not accelerate. The cause is found
  (next stage, "Straight-leg scale bias"): it is a bias of the visual update, not an accel bias
  problem. The loop rows above hide the same bias: at 5 km the loop of R 800 m is almost closed, and
  a circle that is too large comes back near the start (see there).
- The baro gives no scale on a level leg, as intended (33 % to 31 %). With speed changes it makes the
  scale worse (7.6 % to 12 % on the straight leg, 0.49 % to 0.76 % on the loop). Diagnostic: the same
  flights with the baro's speed term removed (`gen_nospeedterm.py`, a copy) end at 6.4 %. So the
  speed-squared term of the model, a fake height change of up to 1 m that follows the speed, is
  what leaks into the scale. A real baro has such a term (dynamic pressure at the port); our
  recordings must measure it before the baro is trusted in fast flight.
- Noise bands (ATE and NEES of each set with pixel noise 0.4995 / 0.5005 px): every baro difference
  in both tables is outside the bands, except the straight leg with S-turns (no change); the loop
  with S-turns is slightly better with the baro (ATE 0.51 to 0.49 %).

CPU (laptop, Ryzen 7 7435HS, one thread, single runs): filter 0.5-1.7 ms per update frame median
(0.5 on the 5 km line with baro, 0.7 on renders, 1.7 on EuRoC), up to 3-4 ms at the 99th percentile
with landmarks or at the liftoff, 6.6 ms worst (EuRoC). Tracker 6.5-8.2 ms per camera frame
including PNG decoding. The Pi 5 will be slower; measured at point A.

Step 3 report: DATASETS.md section 7 (Monte Carlo summary, CPU, what rests on synthetic data, what
is left before step 4).

**Straight-leg scale bias (2026-11-26).** Why is the speed biased high on straight legs at constant
speed? Runs, scripts and a throwaway copy of the code with diagnostic switches:
`~/datasets/analysis/scale_bias_2026-11-26/` (`results.md` lists them). No code in the repo changed:
the cause is a property of the estimator, not a coding error. Synthetic data only (generator tracks
without drift, flat ground, no wind).

Set: `--scenario line --duration 420 --speed 12 --height 40 --density 0.004 --manoeuvre none
--baro`, baro update off, `config/vio.yaml` + `config/replay_synthetic.yaml`. Speed ratio =
estimated / true horizontal speed at 95 % of the flight; along-track error and sigma at 5 km in % of
the path; median (range).

| Run | seeds | speed ratio | along-track error | sigma | NEES pos |
|---|---|---|---|---|---|
| today's code | 20 | 1.52 (1.31-1.67), 20 of 20 above 1 | 33 % (26-41) | 2.3 % | 60 |
| no noise at all | 8 | 1.00 | 0.0 % | 3.4 % | 0.0 |
| exact pixels, filter told 0.5 px | 8 | 1.26 (1.22-1.29) | 18 % | 2.7 % | 13 |
| exact pixels, filter told 0.05 px (added in the midpoint stage) | 8 | 1.01 | 1.0 % | 0.35 % | |
| pixel noise only (IMU noise 0) | 8 | 1.33 (1.19-1.41) | 22 % | 2.4 % | |
| td frozen | 8 | 1.52 | 34 % | 2.3 % | |
| propagation Jacobians at the truth | 8 | 1.52 | 34 % | 2.3 % | 63 |
| visual Jacobians at the truth | 8 | 1.01 (0.85-1.13) | 0.5 % (-5.4-7.6) | 1.8 % | 5.0 |
| ... only the clone attitudes true | 8 | 1.49 | 32 % | 2.3 % | 59 |
| ... only the point true | 8 | 1.31 | 21 % | 2.0 % | 38 |
| ... only the clone positions true | 8 | 1.02 | 0.8 % | 1.8 % | 4.9 |
| ... estimated positions, spacing scaled to the true one | 8 | 1.31 | 21 % | 2.0 % | 38 |
| ... true shape of the window at the estimated scale | 20 | 1.01 (0.83-1.14) | signed +1.1 % (-9.9-8.6); absolute median 3.8 %, rms 4.8 % | 1.8 % | 4.8 |
| ... true shape, only along the track | 8 | 1.41 | 27 % | 2.3 % | |
| ... true shape, only vertical / only sideways | 8 | 1.30 / 1.32 | 20 % / 21 % | 2.3 / 1.9 % | |
| ... true shape, vertical and sideways | 8 | 1.13 | 8.6 % | 1.9 % | |

"At the truth" means: the ground truth aligned to the estimate in position and yaw only (the
directions VIO cannot observe, and in which the estimate drifts by hundreds of metres); tilt,
shape and scale stay true. The residuals are always computed from the estimate. Without that
alignment the Jacobians and the covariance refer to different frames and the filter diverges.

What is proven (on this synthetic set):

1. The speed gain comes from the visual updates. Summed over the flight, the updates add +4.5 to
   +9.6 m/s along the velocity per seed; propagation adds -5 to +2 m/s at random. Each update's
   along-track correction is small (rms 0.04 m/s) but its mean is positive, about +2.5 mm/s per
   update at the true speed, growing with the speed error (+3.8 mm/s per unit of speed ratio).
   Turning the velocity (|v + dv| > |v| for a sideways correction) contributes only +0.07 m/s.
2. The source is where the visual update is linearized: at the estimated clone positions, whose
   shape (positions relative to each other within the window) is off. With the same code and the
   same residuals but the Jacobians taken at the true shape, at the estimate's own (wrong) scale,
   the bias is gone: signed median +1.1 % over 20 seeds. It is not honest though (corrected after
   the review): absolute median 3.8 %, rms 4.8 %, sigma 1.8 %, error/sigma rms 2.7, 5 of 20 seeds
   beyond 3 sigma, position NEES 4.8. The scale error itself does not feed the bias
   (correcting only the scale leaves 21 %); the clone attitudes do not matter. Most of it comes from
   the window bending away from a straight line sideways and vertically (8.6 % with those two parts
   true). The bend is smooth: a quadratic or cubic fit through the window as the linearization
   point changes nothing (30-32 %).
3. Not a coding error found: the update formulas give an unbiased filter when evaluated at the
   true shape (still overconfident along the track, see 2); the propagation Jacobians at the truth change nothing; the IMU
   propagation alone is honest on this trajectory (60 seeds, 30 s, mean NEES of position 2.4-2.8,
   attitude 2.7-3.0, velocity 2.7-3.2 against the 95 % band 2.41-3.65). td is not involved
   (frozen: same bias).
4. It is not specific to the plain EKF: an iterated EKF (residual and Jacobians at the updated
   estimate, 1 or 3 iterations) biases the scale low by the same amount (speed ratio 0.54, -32 %,
   NEES 135). Linearizing at the prior pushes the scale up, linearizing at the converged estimate of
   the same update pushes it down; the true shape lies between.
5. Why it shows on straight legs: at constant speed nothing accelerates, so the IMU gives no scale
   and nothing pulls the speed back; the small mean push of every update adds up over 2100 updates.
   With accelerations the real scale information holds it (table below).

Guesses (not proven): why the sign is positive at the prior (the update straightens a bent window;
in the linear model that lengthens it, which the filter books as speed); why the window baseline
matters so much (13 m at stride 3: 104 %; 26 m today: 33 %; 52 m: 5.5-7.8 %), probably because the
shape error relative to the baseline is what counts. Lead 5 of the review is plausible but not
re-tested (the FEJ code is gone): FEJ linearizes every clone at its first estimate, before any
update, which is the noisiest choice, and it gave the same 30 % high speed.

The same mechanism on the other sets (today's code / true window shape; 8 seeds, renders 20 starts,
ATE in % of the path, position NEES):

| Set | today | true shape |
|---|---|---|
| straight, constant speed (20 seeds) | 13.6 % / 60 | 2.0 % / 4.8 |
| straight, speed +-20 % | 3.9 % / 19 | 0.51 % / 2.3 |
| straight, S-turns | 1.4 % / 11 | 0.57 % / 2.7 |
| loop R 800 m, constant speed | 4.7 % / 33 | 0.91 % / 3.4 |
| loop, speed changes | 1.5 % / 12 | 0.68 % / 3.5 |
| loop, S-turns | 0.51 % / 4.7 | 0.46 % / 3.4 |
| figure8, tracks | 0.154 % / 2.9 | 0.129 % / 3.0 |
| render fig8_tilt_img | 0.172 % / 3.0 | 0.165 % / 3.0 |

- The loop at constant speed has the same bias (speed ratio 1.40 at 95 %, horizontal error 8.1 % of
  the path at 3 km); stage 4 called it honest because its along-track error at 5 km is small, but at
  5 km the loop is almost closed and a circle that is too large comes back near the start.
- Speed changes and S-turns reduce it, and the figure-eight and the render (tighter turns, 13-43 deg
  tilt) show almost nothing. Weaker manoeuvres do not remove it (added after the review; baro off,
  speed ratio at 95 % of the flight, today's code, tools/scale.py): S-turns every 40 / 80 / 160 s
  1.02 / 1.28 / 1.51, speed changes every 40 / 80 / 160 s 1.11 / 1.32 / 1.44 (40 s: 20 seeds, 80 and
  160 s: 8 seeds, the review's sessions). One manoeuvre every minute or two is not enough. So the more the flight accelerates, the less it matters. A real flight
  has wind and corrections, which helps; drifting KLT tracks may hurt (not tested).

Options (none implemented; straight leg, 8 seeds unless stated, filter CPU per update frame on the
laptop, today 0.81 ms):

| Option | along-track error / sigma | NEES pos | cost | risk |
|---|---|---|---|---|
| A. 22 clones, track age 144 (52 m window) | 5.5 % / 0.90 % | 12 | 1.64 ms per update frame (x2) | still 6 sigma; changes a default |
| B. 11 clones, stride 12, track age 144 (52 m) | 7.8 % / 1.06 % | 16 | 1.34 ms per update frame, half as many frames | tracks of 4.8 s (KLT drift on real images); slow flight and hover need short strides, so only as a motion-based clone rule (open item 4) |
| C. iterated EKF (1 or 3 iterations) | -32 % / 1.3 % | 135 | 2-4 times the update (not measured) | wrong sign, no gain |
| D. smooth linearization point (polynomial fit through the window) | 30-32 % | 51-57 | small | no gain |
| E. scale from outside: ground plane with the baro height above the takeoff point | not measured | | a plane or a per-feature constraint in the filter, a formulation change | flat-ground assumption; wrong over hills, trees, buildings; needs a check that the points lie on a plane |
| F. honest covariance on unaccelerated flight (scale uncertainty that grows when nothing accelerates) | bias stays | not measured | small | noise inflation: your decision; the quality module would see the problem, the output stays wrong |
| G. fly with turns or speed changes | S-turns: 1.4 % ATE today | 11 | none | flight planning, not a fix; still overconfident |
| (ideal: true window shape, cannot be built) | signed +1.1 %, rms 4.8 % / 1.8 % (20 seeds) | 4.8 | | not an upper bound (corrected after the review): the midpoint linearization of the next stage does as well on the same 20 seeds |

My view at the time: the estimator alone will not remove it. That was wrong: a linearization point
halfway between prior and update (next stage) removes most of it. Also wrong: recommending B. The
review ran A and B on the other sets (20 starts each): on EuRoC their equivalents diverged in some
starts (A, 22 clones at stride 3 with age 72: worst run 721 %, median ATE 0.85 %; B, stride 6 with
age 72: worst run 1562 %, median 1.45 %; today 1.08 % worst, 0.72 % median); on the renders they are
mixed (normal render ATE 0.172 % today, A 0.224 %, B 0.199 %; low contrast 0.446 / 0.367 / 0.406 %;
hover 0.568 / 0.687 / 0.522 %). Longer windows therefore stay an open item of the motion-based clone
rule, not a fix of the scale bias. Not done: an analytic model of the bias, a test with drifting
tracks (renders) on a straight leg.

Deviations of that stage (not listed then, review): the line and loop runs had the baro update off
(`nobaro.yaml`), and most diagnostic sets had 8 seeds; only the today / true-shape comparison of the
straight leg had 20.

**Midpoint linearization (2026-11-26).** Proposed by the review, built behind a switch, off by
default: `filter.midpoint_linearization` (`config/vio.yaml`, `msckf.h`). The stacked MSCKF update is
done once to get the updated clone poses; the prior is restored (`Filter::restore`) and the update is
done again with the Jacobians (Hx, Hf) taken at the clone poses halfway between prior and update
(slerp and mean), the point triangulated at those poses. Residuals, covariance and state stay at the
prior. The second pass uses the tracks that passed their chi2 test in the first pass and does not
test them again (keep-gate, below; as first built the per-track tests ran again in the second pass);
the joint gate runs in both passes. If the first pass is rejected by the joint gate nothing changed
and there is no second pass. Landmark updates are not
affected. Switch off: identical to the code before, every column but timing, 56 of 56 replays
(figure8, line, loop, approach_hover, liftoff with baro and landed flag, the three renders, EuRoC);
also the baro-on line set reproduces the stage 4 numbers exactly. Runs, scripts and the status of the
stage: `~/datasets/analysis/midpoint_2026-11-26/` (`STATUS.md`, `run_eval.sh`, `run_sweep.sh`,
`summarize.py`, `results_so_far.txt`).

Off, midpoint as first built and midpoint with keep-gate (the repo version) over every set of the
step 3 report: DATASETS.md section 7 (20 seeds or starts, noise band of 13.8 for each version; the
24 km set keeps its 4 seeds). Keep-gate and the first build agree within the noise bands on 20 of the
22 sets with a band, so the short version holds for both:

- Flights without (much) acceleration: the bias is much smaller, not gone (corrected after the
  review). Straight leg at constant speed, 20 seeds, baro off: ATE 13.6 % to 1.81 %, position NEES 60
  to 5.5, scale (tools/scale.py, estimated / true distance) over the flight 1.33 to 1.03, over the
  last 10 % of the path 1.76 to 1.12 (0.98-1.27), speed ratio at 95 % of the flight 1.52 to 1.07
  (0.94-1.22); with the baro on 1.51 to 1.055 (0.92-1.21), last 10 % 1.74 to 1.107 (0.96-1.26). The
  speed is still too high at the end in most seeds and the scale late in the flight is above the
  mean: what is left still grows; loop at constant speed 5.1 % to
  0.90 %, NEES 37 to 3.0. Weaker manoeuvres (8 seeds, baro off; speed ratio at 95 % of the flight):
  S-turns every 80 / 160 s 1.28 / 1.51 to 1.00 / 1.07, speed changes every 80 / 160 s 1.32 / 1.44 to
  1.01 / 1.02 (ATE 8.8-14 % to 1.0-2.8 %).
- Everything else is the same or better outside the noise bands: figure8 0.148 % to 0.129 %, long
  flight 9 km 0.44 % to 0.33 % (NEES 4.2 to 2.6), 24 km NEES 4.9 to 2.3, normal render 0.172 % to
  0.163 %, low-contrast render 0.446 % to 0.359 % (NEES 10.5 to 8.4), gentle takeoff 89 % to 35 %.
- Worse: the hover render (ATE 0.569 % to 0.609 %, NEES 4.6 to 5.5, below), the EuRoC tail (below),
  the attitude NEES of the sharp takeoff (7.8 to 9.0).
- Not honest: on the straight leg (20 seeds, baro off) the along-track error at 5 km is +2.9 % signed,
  2.9 % absolute median, rms 5.8 %, 13.7 % worst, sigma 2.1 %, error/sigma rms 2.7, 7 of 20 seeds
  beyond 3 sigma. The true window shape (same 20 sessions) does about as well, not better: ATE 1.95 %
  against 1.81 %, NEES 4.8 against 5.5, along-track rms 4.8 % against 5.8 %.
- With the baro on, speed changes still leak through the generator's baro speed term (straight leg
  with speed changes: 6.4 % to 3.3 %, NEES 57 to 17; baro off 4.1 % to 0.66 %, NEES 21 to 2.3).

How large the along-track sigma should be on a straight leg (corrected after the review): not known.
It has not been derived; that would need the information the measurements carry about the scale on
such a flight (a Cramer-Rao bound for this sensor setup), which was not done. What is known:
- At constant velocity the scale cannot be observed (section 4: monocular camera, no acceleration),
  so its uncertainty must not shrink below what the start and the noise put into it. The filter's
  along-track sigma stays at a constant share of the path (1.5-1.9 % from 1 to 5 km with the true
  window shape, 2.1-2.3 % today), which is what a filter does that thinks it measures the speed.
- The spread of the errors of one estimator: with the true window shape the rms along-track error
  grows from 1.9 % at 1 km to 4.8 % at 5 km (20 seeds), with the midpoint linearization 5.8 % at
  5 km. That is the spread of these estimators, not the sigma an honest filter should report; a
  better estimator could have less spread.
- Taking the propagation Jacobians from the truth as well does not change the overconfidence
  (error/sigma rms 2.6 to 2.7, 8 seeds); its cause is not established.

Why it helps, as far as it is shown. Blend sweep: Jacobians at (1 - b) prior + b update, straight
leg, baro off, 8 seeds, signed along-track error at 5 km (`run_sweep.sh`, diagnostic copy):

| Set | b = 0 (today) | 0.25 | 0.5 | 0.75 | 1 (iterated EKF) | zero crossing |
|---|---|---|---|---|---|---|
| 0.5 px and IMU noise (the usual set) | +33 % | +19 % | +3.5 % | -14.5 % | -32 % | about 0.55 |
| 0.1 px and IMU noise, filter 0.1 px | +6.0 % | +3.6 % | +0.9 % | -2.0 % | -5.1 % | about 0.57 |
| 1.0 px and IMU noise, filter 1.0 px | +59 % | +28 % | -8.8 % | -45 % | -54 % | about 0.43 |
| 0.5 px, no IMU noise | +22 % | +7.2 % | -11 % | -35 % | -50 % | about 0.33 |
| exact pixels, filter told 0.05 px | +1.0 % | +1.0 % | +0.9 % | +0.9 % | +0.8 % | no bias to remove |
| exact pixels, filter told 0.5 px | +18 % | +18 % | +17 % | +16 % | +16 % | none |

- Shown: the bias is close to linear in b and changes sign between prior and update.
- Shown: where it crosses zero depends on the mix of pixel and IMU noise (0.33-0.57). With 0.5 px
  and our generator's IMU noise it crosses near 0.55, which is why b = 0.5 works so well on these
  sets; that is partly luck of the noise mix, not a property of the method.
- Likely explanation, not proven (corrected after the review: it does not explain why the crossing
  moves): for a measurement with constant curvature the slope taken halfway between two points is
  exactly the slope of the line joining them, so a linear model taken there predicts the change from
  the prior to the update, and taken at either end it errs in opposite directions. The update stands
  in for where the state should go; how well it does that depends on the noise, which may be why the
  crossing moves.
- Exact pixels: with the filter told 0.05 px the bias is only 1 % at any b: the bias comes from the
  pixel noise, not from the IMU noise. With the filter told 0.5 px for exact pixels (the "IMU noise
  only" set of the last stage) the update under-trusts the pixels, leaves the shape errors of the
  window, and moves the clones little; blending between two nearly equal poses changes nothing (18 %
  at every b). So "no effect with exact pixels" is a noise-model mismatch, not a failure of the idea.
- 1 px: the error changes fast with b (about 150 % per unit of b near 0.5), the crossing moves to
  0.43, and b = 0.5 over-corrects (speed 0.88 times the true one, -8.8 %).
- Guess: two parts, one that this update's own noise makes and that changes sign along prior to
  update, one from shape errors the update leaves in the window, which no b removes.
- What it means for real images: the filter uses 1 px on real tracks, and the real mix of track noise
  (white plus drift, 0.17-1.0 px white part on the renders) and IMU noise is not known. On a straight
  leg b = 0.5 may over-correct, so the speed may come out too low instead of too high. The renders
  (real KLT tracks, 1 px) do not show it, but they turn all the time (scale 1.003 off and on); a
  straight-leg render was not made (about 1 h of rendering per 2 min of flight). Our recordings or
  such a render have to show it before the switch is trusted on straight legs.

Unobservable directions. In invariant errors the yaw direction (dtheta along the vertical on the IMU
and every clone) and the three translations (dp on the IMU, every clone and landmark) do not depend
on the state. Diagnostic check on every stacked update, both passes, switch off and on (straight leg,
normal and hover render, EuRoC start 22): max |H N| / (|H| |N|) below 1.4e-16. Position and yaw still
gain no information.

EuRoC (121 starts): two runs above 1 m aligned in both versions, but different ones: off starts 25.45
(14 %) and 22.35 (2.6 %), on 24.3 (diverges, 1378 %) and 25.25 (3.7 %); the 20-start medians do not
change (0.718 % / 0.711 %, NEES 66 / 66). In 24.3 the second pass rejects almost every track at the
per-track chi2 test after the liftoff (30 rejected and 0 used in a frame where the switch off uses 5),
so the filter runs on the IMU alone and diverges. At the EuRoC liftoff the first pass moves the clones
a lot (7 s on the IMU alone before), and the tests repeated with the midpoint Jacobians reject tracks
that passed at the prior. With keep-gate (now the repo version) EuRoC 121 starts: one run above 1 m
aligned (25.25, 1.58 m aligned, 3.7 %) against two with the switch off (worst 3.79 m) and two as first
built (worst 728 m); median ATE 0.728 %, NEES 67 / 40, as before.

Keep-gate (2026-11-26, in the repo since the stage after the first build). The second pass keeps the
first pass's per-track chi2 decisions; the joint gate stays in both passes. Results (DATASETS.md
section 7, third column): within the noise bands of the first build on 20 of the 22 sets with a band;
slightly worse than it on two (loop with speed changes 1.357 % to 1.371 %, straight leg with S-turns
and baro off 0.587 % to 0.606 %); against the switch off on EuRoC 20 starts ATE 0.718 % to 0.729 %
(outside the band; NEES 66 to 64). The tails are better: EuRoC 121 starts worst run 1378 % (first
build) to 3.7 % (off: 14.2 %); sharp takeoff worst run 80 % to 31 % (off: 74 %).

What it changes statistically. The per-track test is the usual EKF innovation test, chi2 = r^T S^-1 r
with S = Ho P Ho^T + R, and keep-gate takes Ho from the prior Jacobians; the update then uses the
same residual with Ho from the midpoint Jacobians, whose S is different. So the test and the update
use two different linear models of the same measurement. In the first build both used the midpoint
model, but the midpoint depends on the first pass, which these same residuals produced: the test
judged each residual partly against a point it had moved itself. With keep-gate the decision is the
plain test at the prior and does not depend on the first pass.
What it could hide:
- A track that fits the prior model but not the midpoint model now enters the update; only the joint
  gate can still stop it. Measured with a counter in the diagnostic copy (14 runs: figure8, straight
  leg, three render starts, hover render, two sharp takeoffs, four EuRoC starts): 0-11 of 1300-6400
  kept tracks per run would fail the test with the midpoint Jacobians, at most 0.2 %. Rare, but it is
  the case after a big first-pass correction, as at the EuRoC liftoff, where a wrong prior can let a
  bad track through that the midpoint model would have caught.
- An outlier that passed at the prior also moved the first pass and so the midpoint: it has a say in
  the model it is then used with.
- The rejection share in the output (n_rejected) describes the prior test only.
Side observation from the counter: the mean chi2 of kept tracks is 0.59 of the 95 % limit on the
synthetic sets (where the noise model is right) but 0.08-0.17 on the renders and EuRoC: the 1 px
pixel noise used on real tracks is far larger than their white noise (0.17-1.0 px, section 13), so
the per-track test there only catches gross outliers.

Hover render: ATE 0.569 % to 0.609 %, NEES 4.6 to 5.5, both outside the noise band. The difference is a
few centimetres during the hover itself (median position error at 30 s 0.44 m to 0.50 m, at 68 s
0.38 m to 0.36 m) and depends on the window: hover drift 12-60 s 0.222 to 0.260 m/min, 12-69 s 0.397
to 0.374 m/min. Guess: in hover the clones barely move between prior and update, so the midpoint
changes the Jacobians of features with very little parallax by noise only. The keep-gate variant gives the same hover render numbers. The synthetic approach_hover
(tracks) shows the same small effect: ATE 0.258 % to 0.254 %, hover drift (12-60 s) 0.111 to 0.138
m/min, outside the band.

CPU (laptop, one thread, quiet machine, median of 3 runs; the whole visual update per update frame,
printed by vio_replay): straight leg 0.36 ms to 0.71 ms, normal render 1.18 ms to 2.28 ms, EuRoC
3.14 ms to 4.43 ms; so 1.4-2 times, plus 1.1 ms on the render's update frames, which already carry
the tracker (5-6 ms). On the Pi 5 (2-3 times slower per core, section 8) that is 2-3 ms more on every
update frame, inside a budget that is already tight. Keep-gate costs the same (single runs: 0.71 ms,
2.28 ms, 4.35 ms against 0.36, 1.21, 3.18 ms off).

Recommendation about the default (updated with keep-gate): keep it off. Keep-gate removed the EuRoC
divergence and makes the tails better than the switch off, so the remaining reasons are: b = 0.5 fits
our generator's noise mix and over-corrects at 1 px, the value we use on real tracks, and no
straight-leg test with real tracks exists; the straight-leg bias is smaller, not gone, and the filter
stays overconfident along the track; the hover gets slightly worse; it doubles the visual update
time. What I would do next, if you agree: render one straight leg at constant speed (about 1 h per
2 min of flight; needs space on C: first) and check the speed ratio with real KLT tracks at 1 px, and
decide the default on our recordings. Not done: tuning b (not allowed: one value for every set).

Open after the review fixes, the moving origin, part a, the EuRoC liftoff check and stage 4:

1. The filter is overconfident on the low-contrast render (position NEES 10, range 2.0-28 over 20
   starts; 4.9 with part a on) and on EuRoC (69, 7.4-281 with td 0; part a does not change it);
   reasons: section 13.3 and DATASETS.md section 6. Parts b and c of section 13 are not decided. No
   scaling or noise inflation meanwhile. Part a: one q does not fit hover and flight (13.10,
   question 6); the hover render gets worse with the flight q.
2. Distance from the origin: moving origin implemented (above); results in DATASETS.md section 6.
3. td: online with the 30 ms window, to be chosen on our own recordings (your decision). Known:
   false confidence on gentle motion; -3.3 ms on the normal-contrast render (true 0, sigma 1.9 ms),
   not investigated for now.
4. The fixed update stride should become a motion-based clone rule; stride 6 and age cap 72 are
   provisional.
5. Done (stage 4 of step 3): baro, km-scale scale-drift sessions. Found: on straight legs the
   scale error grows to 12-33 % in 5 km and the covariance does not show it (12-14 sigma); a baro
   error that follows the speed leaks into the scale; the takeoff stays poor with the baro.
   Cause of the scale error found (2026-11-26, "Straight-leg scale bias" above): the visual update
   linearized at the estimated clone positions pushes the speed up a little on every update, and on
   flights without accelerations nothing pulls it back; also on loops at constant speed. Midpoint
   linearization (above, switch off by default) removes most of it on synthetic data; still
   overconfident along the track; the keep-gate rule (in the repo) removed the EuRoC divergence of
   the first build; untested on straight legs with real tracks. Off by default; waiting for your
   decision.
6. EuRoC liftoff: B and C implemented (above); 2 of 121 starts still above 1 m, the missing
   recovery (D) goes to step 4. The landed-flag rate (2 or 10 Hz) and the real vibration on our
   drone wait for our recordings. C is off by default: with a ground-truth flag it makes the EuRoC
   tail worse (cause not established) and it fires while moving in a gentle takeoff (review).
7. td on a straight leg: unobservable there (no rotation), yet its sigma shrinks from 5 ms to
   0.74 ms while the error reaches -8 to +9 ms (up to 12 sigma), with or without the scale bias. Probably
   the same kind of false information (gyro noise looks like rotation); not investigated. Freezing
   td does not change the scale bias.

Stage 1b, the Pi recorder and our own data (RECORDING.md). Then camera calibration and the choice of
the camera model, then Stage 2 (quality on real data).

Stage 2b, ArduPilot SITL + Gazebo (SIMULATION.md): closed-loop flights over km with perfect ground
truth, ODOMETRY into EKF3, LOST and recovery, failsafes. Before anything goes to the real FC.

Stage 3 (MAVLink output on the real FC, bench), Stage 4 (field).

Which results need which data. Nothing in the right column may be claimed from synthetic data alone:

| Result | Synthetic is enough | Needs real data (EuRoC, our recordings) or the simulation |
|---|---|---|
| Metrics tool computes ATE / RPE / loop closure correctly | yes | |
| Filter math, covariance consistency under the generator's noise model | yes | |
| Time offset estimate, camera model math, scale behaviour in principle | yes | |
| Front end quality (track length, inlier ratio, robustness to blur, low texture) | | EuRoC, then our recordings |
| Consistency with real, imperfect IMU and images | | EuRoC |
| Accuracy numbers to compare with other VIO systems | | EuRoC |
| Rolling-shutter correction, sky mask on real images, our FC IMU path, vibration | | our recordings |
| Drift over 2-5 km, hover drift, loop closure on our routes | | simulation, then our flights with ground truth |
| EKF3 accepting our data, LOST / recovery, failsafes | | simulation, then bench |
| Latency and CPU on the Pi | | the Pi |

When the Pi and the drone are needed:

| Point | What is needed | What for |
|---|---|---|
| A, after step 3 of 1a (the filter runs) | the Pi alone, no camera or FC: SSH, build packages | build on aarch64 and measure CPU per frame with replay, early enough to change course if the Pi is too slow |
| B, start of 1b | Pi + camera + FC on USB + fast SD card (A2 / V30, 64 GB or more) | recorder, bench recordings, IMU noise recording (2-3 h static) |
| C, when the final mount is built | the drone with the mount, Aprilgrid printed | camera intrinsics, model choice, camera-IMU calibration |
| D, first flights | the drone, ideally the GNSS logger | real recordings for replay and metrics |
| E, Stage 3 | Pi + camera + FC, laptop with SITL | end-to-end latency, MAVLink output, bench tests without props |

Until point A nothing on the Pi is needed. Point B is the one to prepare camera, FC and SD card for.

## 12. Open questions

1. Upgrade to 4.7.3 or not: your decision, the relevant changes are in ARDUPILOT_SETUP.md section 12.
2. MAVLink logging (`LOG_BACKEND_TYPE = 2`): your decision, assessment in ARDUPILOT_SETUP.md
   section 13.
3. When to add Loiter to the mode switch: your decision, after the simulation stage.
4. The path of the parameter backup file `stellar_h7v2_before_vio` (2026-11-17), to complete the
   "before" column in ARDUPILOT_SETUP.md section 10. It will not be committed.
5. Dynamic covariance: the questions in section 13.10.

## 13. Dynamic covariance (proposal)

Status: proposal, 2026-11-25. Part a (drift-aware feature noise, 13.4) is implemented since
2026-11-26 behind a switch that is off by default (`filter.track_drift.enabled`); results in 13.11.
Parts b and c are on hold (2026-11-26); the LOST thresholds and sigma_z are not decided. No config
default changed. Your decision (2026-11-25): the covariance must be dynamic, growing and shrinking with the
real conditions. A single fixed output scale factor (the earlier proposal in section 11) is dropped.

All measurements below use the current binaries and throwaway scripts outside the repo, in
`~/datasets/analysis/dynamic_cov_2026-11-25/` (scripts, raw outputs in `runs/`). From the repo root,
`~/datasets/analysis/dynamic_cov_2026-11-25/run_all.sh` repeats all of it (about 3 minutes on 16
threads). Renders use the 1 px pixel noise override (`px1.yaml` there, review problem 6).
The baseline and the filter-dependent signals of 13.3 were measured again on 2026-11-26 with the
code after the review fixes and the moving origin, over 20 start times, and part a was evaluated:
`~/datasets/analysis/drift_noise_2026-11-26/` (`run_baseline.sh`, `run_synthetic.sh`,
`run_chaos.sh`, `viewfit.py`, results in `runs/` and `synth/`).

### 13.1 What is constant today, what varies, and what each constant causes

What varies: the filter covariance P. It grows with IMU propagation and shrinks with every update,
so it already reacts to "no features" and "many features". But how fast it grows and how much each
feature is trusted come only from the constants below.

| Constant | Value | How it is used | What the data says (13.3) |
|---|---|---|---|
| `filter.pixel_noise_px` | 1 px (generator sessions 0.5) | every MSCKF and landmark pixel, as white noise: independent between frames and between features | The real track error is mostly a slow drift, not white noise. Its white part is 0.17-1.0 px RMS, below the 1 px. On the normal render the margin covers one window of drift (honest), on the low-contrast render there is no margin left (overconfident). |
| Same pixel noise for landmarks | 1 px, every update for up to 60 s | each update treats the landmark's pixel error as new and independent | The same drift is counted again at each update. Landmark count follows the EuRoC attitude NEES (Spearman +0.55). |
| IMU noise densities and random walks | `imu:` or the session's `sensor.yaml` | process noise, white, the same in hover and in a 45 deg bank | On EuRoC 33-38 % of the tracks fail the 95 % chi2 gate (5 % expected), although their white pixel error is 0.88 px RMS: the pose part of the innovation covariance is too small. EuRoC velocity and attitude NEES rise with speed, rotation rate and acceleration (+0.24 to +0.31). Not on the renders, where the IMU model is exact. |
| ZUPT velocity noise | 0.01 m/s | only when still | No evidence either way: still only at the EuRoC start. On the drone it needs the landed check with motors running (Stage 1b). |
| `filter.chi2_probability` | 0.95 | the gate | Fine as a probability; it is only as good as the covariance it tests against. |
| `output.degraded_inflation_per_s` | 0.5 | planned growth in DEGRADED | Not used yet, no basis in data; replaced by 13.4 and 13.6. |

### 13.2 Signals available at run time

- Tracker, per frame (`vio_track` stats): forward-backward error median and 90th percentile, share
  lost in the forward-backward check, KLT success ratio, number of features, grid cells occupied,
  largest share of features in one cell, usable image fraction (sky mask), brightness, contrast,
  blur (variance of the Laplacian, high = sharp), saturated share, motion the gyro did not explain.
- Tracker, per feature: track length, image path length, view change since the first observation.
- Filter, per update: features used, chi2 rejection share, features without parallax, landmarks,
  normalized innovations (NIS; not written by the current binaries, so not measured here).
- IMU: rotation rate, acceleration, high-pass acceleration (vibration proxy), gaps. On the drone
  also the FC `VIBRATION` message with clipping counts, and later ESC RPM and throttle.

### 13.3 Which signals predict the error

Targets per update frame:

- true pixel error: tracks from `vio_track` (same age cap as the filter), every stride-th
  observation, cut into pieces of 12, triangulated from the ground truth poses; median
  reprojection error of the observations made in the last update interval;
- normalized drift: growth of the position error over the last 2 s divided by the growth of the
  reported position sigma. Above about 1 the filter was overconfident at that moment. This is the
  target that matters: the absolute error is accumulated history and no current signal can
  predict it;
- velocity and attitude NEES (EuRoC column).

Spearman rank correlation per run, median over the start times. The series are autocorrelated and
the start times overlap, so anything under about 0.3 counts as "does not predict".

Commands (repo root, `A=~/datasets/analysis/dynamic_cov_2026-11-25`, `R=$A/runs`):

```
# filter, six start times: fig8 renders 0 2 4 6 8 10, hover 0 4 8 12 16 20, EuRoC 21-25
build/vio_replay ~/datasets/synthetic/fig8_tilt_img --mode vio --start 0 --config config/vio.yaml \
  --config config/replay_synthetic.yaml --config $A/px1.yaml --out $R/tilt_s0.csv
build/vio_replay ~/datasets/synthetic/fig8_tilt_low --mode vio --start 0 ...same... --out $R/low_s0.csv
build/vio_replay ~/datasets/synthetic/approach_hover_img --mode vio --start 0 ...same... --out $R/hover_s0.csv
build/vio_replay ~/datasets/euroc/MH_01_easy --mode vio --start 22 --config config/vio.yaml \
  --config config/replay_euroc.yaml --config $A/tdfix.yaml --out $R/euroctd0_s22.csv
# tracker statistics
build/vio_track ~/datasets/synthetic/fig8_tilt_img --out $R/tilt_feat.csv --stats $R/tilt_stats.csv
build/vio_track ~/datasets/euroc/MH_01_easy --no-sky-mask --start 20 --max-track-age 36 \
  --out $R/euroc_feat.csv --stats $R/euroc_stats.csv
# signals against errors
python3 $A/signals.py . $R tilt ~/datasets/synthetic/fig8_tilt_img 6 0 2 4 6 8 10
python3 $A/signals.py . $R euroctd0 ~/datasets/euroc/MH_01_easy 3 21 22 23 24 25
# drift: residual against track length, fits, run-time estimates (long tracks: --max-track-age 0)
python3 $A/drift.py . $R/tilt_long.csv ~/datasets/synthetic/fig8_tilt_img 6
python3 $A/driftfit.py . $R/tilt_long.csv ~/datasets/synthetic/fig8_tilt_img 6
python3 $A/qest2.py . $R/tilt_feat.csv ~/datasets/synthetic/fig8_tilt_img 6 $R/tilt_s0.csv $R/tilt_s4.csv
python3 $A/anchor.py . $R/tilt_long.csv ~/datasets/synthetic/fig8_tilt_img
```

EuRoC rows use td fixed at 0 (`tdfix.yaml`, td prior 1e-6 ms) to keep review problems 2 and 3 out;
with online td the picture is the same (13.9).

Baseline, re-measured on 2026-11-26 (review fixes and moving origin included; the first
measurement of 2026-11-25 was made before both, its medians over six starts in brackets). Per-run
values as median (min-max) over 20 start times: `fig8_tilt_img` and `fig8_tilt_low` 0, 0.5, ..., 9.5
s; `approach_hover_img` 0, 0.2, ..., 3.8 s (every run includes the approach, so the hover window is
the same for all); EuRoC 21, 21.25, ..., 25.75 s with td fixed at 0 (21 to 21.5 give the same run).
Renders with `config/replay_render.yaml` (1 px).

| Data | NEES pos | NEES att | pos inside 3 sigma | att inside 3 sigma | chi2 rejected | white pixel error, median / RMS / > 3 px |
|---|---|---|---|---|---|---|
| `fig8_tilt_img` | 3.0 (0.80-11) [3.3] | 2.4 (1.3-4.7) [1.8] | 100 % (88-100) | 100 % (99.5-100) | 0.1 % (0-0.2) | 0.25 / 0.38 px / 0 % |
| `fig8_tilt_low` | 10 (2.0-28) [8.2] | 7.9 (3.8-16) [9.9] | 91 % (57-100) | 94 % (76-100) | 6.6 % (5.7-7.2) | 0.83 / 1.00 px / 1.3 % |
| `approach_hover_img` | 4.6 (1.5-13) [15] | 3.5 (1.5-7.3) [5.2] | 99.9 % (74-100) | 99.6 % (92-100) | 0.1 % (0-0.3) | 0.09 / 0.17 px / 0.4 % |
| EuRoC `MH_01_easy`, td 0 | 69 (7.4-281) [77] | 42 (33-69) [40] | 86 % (42-95) | 67 % (63-69) | 34 % (32-38) | 0.43 / 0.88 px / 5.9 % |

(White pixel error: RMS of the inner 95 %, from the tracker alone, so it did not change. On EuRoC it
includes the error of the reference attitude, 0.1-0.2 deg, which is 0.8-1.6 px. The hover value in
brackets came from starts 0-20 s, several of them already in the hover, and from the code before
the review fixes, so the change from 15 to 4.6 cannot be pinned on one fix.)

Signals against the true pixel error ("pix") and the normalized drift ("drift"), median Spearman
over the 20 start times (re-measured 2026-11-26; `signals.py` from the 2026-11-25 folder):

| Signal | tilt pix | tilt drift | low pix | low drift | hover pix | hover drift | EuRoC pix | EuRoC drift | EuRoC att NEES |
|---|---|---|---|---|---|---|---|---|---|
| forward-backward error, median | +0.72 | +0.53 | +0.00 | +0.44 | +0.46 | -0.02 | +0.27 | +0.08 | +0.11 |
| forward-backward error, p90 | +0.68 | +0.45 | +0.05 | +0.41 | +0.50 | -0.01 | +0.31 | +0.10 | +0.08 |
| share lost in forward-backward | -0.16 | +0.29 | +0.04 | +0.36 | -0.13 | -0.05 | +0.14 | +0.18 | +0.21 |
| KLT success ratio | +0.24 | +0.07 | +0.11 | -0.21 | -0.14 | -0.02 | -0.02 | -0.00 | -0.21 |
| number of features | +0.26 | +0.22 | +0.07 | +0.04 | -0.39 | +0.17 | -0.02 | +0.13 | -0.14 |
| grid cells occupied | -0.66 | -0.57 | -0.14 | -0.47 | -0.18 | +0.07 | -0.03 | +0.14 | -0.16 |
| largest cell share | +0.13 | +0.51 | -0.09 | +0.45 | +0.42 | -0.15 | +0.09 | +0.15 | +0.27 |
| usable image fraction | -0.92 | -0.25 | -0.15 | -0.41 | -0.03 | +0.16 | -0.22 | +0.26 | +0.35 |
| contrast | -0.37 | -0.29 | -0.20 | -0.24 | -0.30 | +0.04 | +0.20 | +0.26 | -0.11 |
| blur (high = sharp) | +0.40 | -0.14 | -0.00 | -0.17 | -0.34 | -0.09 | -0.33 | +0.39 | +0.37 |
| brightness | +0.56 | -0.12 | -0.00 | -0.08 | -0.31 | -0.10 | +0.36 | +0.22 | -0.27 |
| chi2 rejection share | +0.15 | +0.68 | +0.02 | +0.19 | -0.45 | +0.35 | +0.37 | +0.14 | -0.23 |
| landmarks |  |  |  |  | -0.18 | +0.36 | -0.51 | +0.10 | +0.52 |
| speed | -0.69 | +0.22 | -0.31 | +0.26 | +0.11 | -0.32 | +0.01 | -0.04 | +0.26 |
| rotation rate | +0.79 | +0.28 | +0.10 | +0.34 | +0.65 | -0.15 | -0.05 | +0.03 | +0.29 |
| high-pass acceleration | +0.09 | +0.08 | -0.09 | +0.08 | +0.04 | -0.04 | -0.20 | +0.04 | +0.29 |

Findings:

1. The track error is a drift, not white noise. RMS residual of a track piece triangulated from
   ground truth poses, for 4 / 8 / 12 / 24 observations: tilt 0.12 / 0.24 / 0.42 / 0.81 px, low
   0.67 / 0.97 / 1.16 / 1.44, hover 0.07 / 0.12 / 0.16 / 0.24, EuRoC 0.29 / 0.47 / 0.63 / 0.93. White
   noise would give nearly the same value for every length (0.79 to 0.97 of sigma). A point fixed
   from the first 12 observations and reprojected later has an error whose variance grows linearly,
   a random walk. Fit sigma^2 = sigma0^2 + q x per image axis:

   | Data | per deg of view change | per px of image path | per second |
   |---|---|---|---|
   | tilt | sigma0 0.0, q 0.094 px^2/deg, R^2 0.98 | 0.43, 0.013, 0.97 | 0.0, 1.75, 0.99 |
   | low | 1.33, 0.139, 0.87 | 1.27, 0.026, 0.95 | 1.24, 2.60, 0.93 |
   | hover | 0.48, 0.012, 0.79 | 0.27, 0.0005, 0.83 | 0.38, 0.005, 0.78 |
   | EuRoC (incl. reference error) | 0.0, 6.0, 0.94 | 0.0, 0.11, 0.99 | 0.0, 6.8, 0.94 |

   The random walk fits everywhere, but q differs a lot between data sets. View change is the most
   uniform driver on the renders (factor 12 between hover and low contrast, against 50 per px and
   500 per second) and it is the physical reason KLT drifts: the patch changes its appearance.
2. Per-frame signals. On both fig8 renders the same signals predict the normalized drift:
   forward-backward error (+0.41 to +0.53) and feature spread (cells occupied -0.47 / -0.57, largest
   cell share +0.45 / +0.51). Rotation rate (+0.28 / +0.34), contrast (-0.29 / -0.24) and the chi2
   rejection share (+0.68 / +0.19) no longer pass 0.3 on both; with six starts on the older code
   they did (+0.36 / +0.40, -0.47 / -0.46, +0.55 / +0.30). On hover the chi2 rejection share
   (+0.35), landmarks (+0.36) and speed (-0.32) now reach about 0.3, none of the tracker signals
   does. On EuRoC no signal reaches 0.3 for the drift, and contrast and blur change sign. The EuRoC
   attitude NEES follows landmarks (+0.52), MSCKF features (+0.41), blur (+0.37), usable fraction
   (+0.35) and motion (speed +0.26, rotation +0.29, acceleration +0.29).
3. Do not predict anything useful: KLT success ratio, number of features, brightness, blur (sign
   changes), saturation (none on the renders, mixed on EuRoC), the high-pass acceleration on the
   renders (the generated IMU has no vibration), speed (for the drift), the filter's own sigma.
4. The drift rate cannot be measured at run time from what we have today, two attempts:
   - Residuals of the tracks with the filter's own poses are 4 to 8 times those with ground truth
     poses (mean squared residual tilt 1.7 against 0.2 px^2, low 6.7 against 1.5; EuRoC slope 3x).
     They are dominated by the relative pose error inside the window, about the RPE: 0.4 % of 26 m
     is 11 cm, about 1 px at 25 m height. Only the residual normalized by its full covariance
     (NIS) can be used, which is what the monitor in 13.4 c does.
   - KLT straight from the birth image to the current image against the chained track: matches the
     real drift after 12 frames on the low-contrast render (rank correlation +0.67, same RMS), but
     after 72 frames the direct KLT lands 25-28 px away on both fig8 renders. Useless beyond about
     one second of flight.
5. The white part is not the problem: below 1 px everywhere. Normal render 0.38 px against 1 px:
   the margin covers the drift of one window, so it is honest. Low contrast 1.0 px against 1 px:
   no margin, the drift comes on top, overconfident.

### 13.4 Where the adaptation acts, and the recommendation

| | a. Measurement noise in the filter | b. Process noise in the filter | c. Output covariance only |
|---|---|---|---|
| Changes the estimate | yes, drifted observations get less weight | yes | no |
| Matches a measured cause | track drift, measured on all four data sets | motion-dependent IMU error, only a hint on EuRoC | anything not modelled, without saying what |
| Feedback risk | none when driven by geometry | none when driven by the IMU | none, nothing goes back into the filter |
| Testable on synthetic data | yes, generator with known drift | yes, generator with scale factor errors | partly |
| Parameters from | our recordings | datasheet, then our recordings | statistics, not fitted |
| CPU | one small Cholesky per track (at most 21 x 21) | nothing | nothing |

Recommendation: a as the main mechanism, c as a guard on the output, b built but switched off
(k = 0) until our recordings show it is needed.

**a. Drift-aware feature noise (in the filter).** For an MSCKF track with observations i = 1..m,
s_i is the view change of the feature accumulated along the track: the sum of the angles between
the rays from consecutive clone positions to the triangulated point (s_1 = 0). Per image axis:

```
R_track = sigma_w^2 * I  +  q * B (x) I_2,      B_ij = min(s_i, s_j)
```

the covariance of a random walk in s with white noise on top. The first version of this proposal
used v_i, the view change since the first observation (the measure of the landmark guard). Review
point 4 (2026-11-26): if the view goes away and comes back, v returns to 0 and the modelled drift
with it, but chained KLT does not undo its drift. Compared on the true pixel error of the renders
(the ground is the plane z = 0, so the true point of a track is where its first ray meets the
ground; error = pixel minus the projection of that point with the true pose; `viewfit.py`, per-axis
Gaussian fit sigma0^2 + q x, every 6th frame, tracks with an error above 5 px left out as wrong):

| Render, tracks | share of observations with s > 1.5 v | -log likelihood, v since first / s accumulated | q, px^2/deg, v / s |
|---|---|---|---|
| normal, age cap 72 | 0 % | 9581 / 9473 | 0.043 / 0.043 |
| normal, no age cap | 1 % | 14696 / 14419 | 0.053 / 0.049 |
| low contrast, age cap 72 | 0 % | 18199 / 18189 | 0.063 / 0.062 |
| low contrast, no age cap | 0 % | 17142 / 17143 | 0.061 / 0.059 |
| hover, age cap 72 | 6 % | -35885 / -35956 | 0.011 / 0.012 |
| hover, no age cap | 58 % | 18008 / 17154 | 0.011 / 0.011 |

The accumulated view change fits as well or better on every set, clearly where the two differ
(hover, long tracks: 854 better). In flight the two are nearly the same (s/v about 1). Neither
predicts the error well where the view came back the most (hover, s/v above 4: measured sigma 0.48
px, predicted 0.64 and 0.67), so the drift there grows more slowly than either model says.
Implemented: s (accumulated).

Whitening: the 2m x 2m track covariance is factored once (Cholesky, L L^T = R_track) and the
residuals and both Jacobians are multiplied by L^-1 before the point is removed. Removing the point
from the whitened system with an orthonormal nullspace basis keeps the noise at the identity; this
is the same information as projecting first and whitening N^T R_track N, and it serves the landmark
birth (same split of the track) without a second code path. From there on stacking, QR compression
and the chi2 gate stay as they are, with unit noise.

Landmarks: update noise sigma_w^2 as now, plus process noise on the landmark position before each
update:

```
Q_f = q * dv * J+ J+^T
```

J is the 2 x 3 Jacobian of the pixel by the landmark position (already computed in `project_point`),
J+ its pseudo-inverse, dv the view change since the landmark's previous update. The drift then acts
as the landmark sliding over the ground: correlated over time, which is what drift is, and carried
by the state. In hover (dv near 0) a landmark keeps its weight; in flight it loses it as fast as its
view changes. The existing guards (5 deg, 60 s, two chi2 failures) stay.

Parameters and bounds:

- sigma_w, white KLT noise of our camera: 0.2-2 px.
- q_0 per deg of view change, from our recordings (13.7).
- sigma_max = 4 px: observations whose modelled sigma sqrt(sigma_w^2 + q s_i) would exceed it are
  left out of the update (the point is triangulated again without them). This is how far the chi2
  gate can ever open. The tracker is not told; its age cap re-detects anyway.
- Optional frame multiplier: q = q_0 * clamp(fb_p90 / fb_p90_ref, 1/4, 4). The forward-backward
  error is the only frame signal that predicted the drift on both fig8 renders with a plausible
  mechanism (KLT's own consistency). Evidence is moderate and absent on hover and EuRoC: off unless
  our recordings confirm it.

Expected: the conflict between hover (long tracks) and low contrast (short tracks) goes away in
principle, because long tracks keep their old observations at the weight their drift allows. Not
expected to fix the EuRoC attitude (IMU side) or the hover NEES (cause still open: white error
0.17 px, drift q tiny, yet NEES 4-20).

**b. Motion-dependent IMU noise (in the filter, built, default off).**

```
n_g_eff^2 = n_g^2 + (k_g * |w|)^2        n_a_eff^2 = n_a^2 + (k_a * |a|)^2
```

w is the gyro rate minus bias, a = R f + g the kinematic acceleration. Per IMU sample in the
propagation, open loop. k_g = k_a = 0 by default. A vibration term from the FC `VIBRATION` message is
left out until hover recordings show accel noise above the static value.

**c. Innovation monitor (output only).** For every track that reaches the update, before the chi2
gate (rejected tracks count too): u_j = NIS_j / m_k, NIS_j = r'^T r' after whitening, k = 2m - 3
degrees of freedom, m_k = k (1 - 2/(9k))^3 the median of chi2 with k degrees of freedom. Per update
with at least 5 tracks: u = median of the u_j; a median does not move while fewer than half of the
tracks are outliers.

```
lambda_target = clamp(u, 1, 10)
lambda += (lambda_target - lambda) * dt / tau,   tau = 2 s rising, 20 s falling
C_out = lambda * C_filter                          (position and velocity block)
```

Without tracks lambda holds its value. lambda is never below 1: the monitor can only make the output
more cautious than the filter, never more confident. It also feeds the quality value and the
DEGRADED test. It would react to the EuRoC case (33-38 % rejections where 5 % are expected); how much
of that NEES it removes can only be measured once NIS is written out (implementation step).

### 13.5 Stability

- No loop inside the filter from residuals to noise: a is driven by geometry (view change), b by
  the IMU. Neither reads residuals or rejections. Only c reads residuals, and it acts on the output.
- Bigger noise cannot let outliers in: the widest the gate can open is sigma_max = 4 px, set by
  geometry. Gross outliers are removed before that by the forward-backward check and the chi2 gate.
- Rejections inflate nothing: the monitor counts rejected tracks but takes the median, and its value
  never goes back into the gate.
- No fast jumps: R grows smoothly with v; lambda rises with 2 s and falls with 20 s; b is smooth in
  the IMU.
- Zero features: no visual update. P grows by IMU propagation (honest and fast), lambda holds, the
  quality module counts bad time: DEGRADED, then LOST after 2 s (section 7). When features return,
  lambda falls only slowly, so the output stays cautious for a while.
- Numerics: R_o is positive definite by construction (sigma_w > 0), lambda is finite and clamped. Any
  non-finite value in P or C_out makes the output LOST (NaN position), never a NaN covariance (13.6).

### 13.6 What ArduPilot 4.6.3 receives, and the quality states

- Position: posErr = cbrt(c0^2 + c6^2 + c11^2) from our variances, floor `VISO_POS_M_NSE` (0.3 m
  planned), EKF cap 10 m. Equal sigma on all axes: 0.3 m gives 0.29 m, 1 m gives 1.44 m, 2 m gives
  3.6 m, and the 10 m cap is reached at 4.3 m (or at sigma_z = 5.6 m alone). Beyond that our growing
  covariance changes nothing for the EKF. Below about 0.5 m the mapping gives a little less than our
  sigma (sigma_xy 0.4 m with sigma_z near 0: 0.37 m, 7 % optimistic); the 0.3 m floor covers most of
  that range. ARDUPILOT_SETUP.md section 3 used to call the mapping "never more confident than the
  truth"; corrected there on 2026-11-25.
- Velocity: our velocity covariance is ignored, `VISO_VEL_M_NSE` (0.3 m/s) is always used. The
  dynamic velocity covariance therefore acts only through the NaN switch: the whole velocity becomes
  NaN once sqrt(lambda) * sigma_v (with vz) exceeds 3 x 0.3 = 0.9 m/s.
- A NaN position covariance with a valid position means maximum trust (the floor) in 4.6.3. Rule:
  the covariance we send is always finite and positive. If it cannot be, the position is NaN too
  (LOST). The MAVLink convention "NaN in the first element = unknown" is never used.
- z: our sigma_z enters posErr although the EKF takes height from the baro. A covariance that grows
  mostly in z (weak scale, low parallax) also lowers the weight of our horizontal position.
  Conservative, kept for now, checked in SITL (open question 3).
- Quality states (section 7):
  - GOOD: C_out = lambda * C_filter, lambda at most 2, horizontal sigma at most 1.5 m, quality 50-100
    from the horizontal sigma and lambda.
  - DEGRADED: lambda above 2, or horizontal sigma above 1.5 m, or a quality test of section 7 fails.
    We still send C_out; there is no extra time-based inflation, because the filter's propagation
    already grows P when updates are missing (`output.degraded_inflation_per_s` would go). Quality
    1-49.
  - LOST: as in section 7 (no good frames for 2 s), and also when the horizontal sigma exceeds 3 m:
    there the 4.6.3 posErr nears its 10 m cap and our number no longer tells the EKF anything. NaN
    position and velocity, quality -1.
  - The thresholds 1.5 m, 3 m and lambda 2 are proposals, to be set in SITL (Stage 2b). Recovery is
    unchanged (section 7); the slowly falling lambda keeps the output cautious after a gap.

### 13.7 Calibration, and on which data

Not on EuRoC: its IMU and reference are not ours (DATASETS.md section 6). EuRoC only shows that
nothing gets worse.

| Parameter | Data | When |
|---|---|---|
| sigma_w | static bench recording, camera over a textured target, motors off and on: KLT jitter without any view change | point B |
| q_0 per deg of view change | a printed planar texture with Aprilgrid tags around it, moved by hand in front of the camera together with the calibration: the tags give the true homography, so every tracked point has a known true position; drift against view change as in 13.3 | point C |
| frame multiplier on/off | the same recordings: does the forward-backward error predict q there? | point C, D |
| k_g, k_a | datasheet start (ICM-42688-P sensitivity and cross-axis errors); Kalibr with the scale-misalignment IMU model on the calibration recordings; hover and flight recordings | point C, D |
| vibration term | hover recordings with motors: FC `VIBRATION` and our IMU stream | point D |
| monitor constants (1, 10, 2 s, 20 s) | statistics, not fitted | |
| DEGRADED / LOST thresholds | SITL closed-loop flights: what the EKF does with our posErr | Stage 2b |

- Before any real data, the mechanism is checked on synthetic data: a generator option for KLT-like
  drift (random walk per degree of view change with a known q) on feature-only sessions, and
  generator scale-factor errors for b. The filter with the true q must be honest there.
- Recordings are split: parameters from one half, acceptance on the other half.
- Waits for our recordings: sigma_w, q_0, the frame multiplier decision, k_g, k_a, vibration.
- Waits for the fixes of review problems 1-3 (13.9): anything judged by position NEES over km, and
  anything measured with td not equal to 0.

### 13.8 Acceptance test

Revised 2026-11-26 (review): one chaotic run must not decide the test. On the low-contrast render a
pixel noise of 0.999 / 1.000 / 1.001 px moves the six-start median position NEES from 20.5 to 18 to
11 and one start from 47 to 8.5. So: more runs, quantiles instead of the worst run, and a noise band
per data set.

Sets. Every number is reported as median and range (min-max) over:

- synthetic feature-only sessions with injected drift (`gen_synthetic.py --track-age 72 --drift-q
  q`), the filter given the true q: 20 seeds per q;
- `fig8_tilt_img`, `fig8_tilt_low`: 20 start times 0, 0.5, ..., 9.5 s; `approach_hover_img`: 0, 0.2,
  ..., 3.8 s (all include the approach);
- EuRoC `MH_01_easy`: 20 start times 21, 21.25, ..., 25.75 s (21 to 21.5 give the same run), with td
  fixed at 0 and online; reported, not graded, its IMU is not ours;
- long flights far from the start (moving origin): 8 seeds at R = 1500 m, 4 at R = 4000 m;
- our recordings: every flight with the GNSS logger, several start points per flight, horizontal
  position only.

Noise band. Each render set is also run with `pixel_noise_px` 0.999 and 1.001 (with part a on, the
same plus the switch), a change that should change nothing. The lowest and highest of the three
medians form the band of that set. A difference between two versions that stays inside the band
of both is reported as "no change", whatever its sign.

Pass, per data set:

- per-run mean NEES of position and of attitude: median 2-4.5 (honest: 3); at most 2 of 20 runs
  above 8 (the 90th percentile below 8; one run above 8 does not fail the set);
- share of per-axis errors inside 3 sigma: median at least 97 %, at most 2 of 20 runs below 95 %;
  inside 1 sigma: median at most 85 % (too cautious also fails, it wastes the EKF's trust);
- accuracy: median ATE at most 10 % worse than today, unless inside the noise band;
- output path (unit test): lambda >= 1, posErr monotonic in our sigma, never a NaN covariance with a
  valid position.

Today's baseline is the first table in 13.3 (2026-11-26).

### 13.9 What depends on the known problems from the review

| Problem | Reproduced here | What in this design depends on it |
|---|---|---|
| 1. Results depend on the distance from the world origin | yes: figure8, 8 seeds, 0 / 1 / 3 / 5 km: median ATE 1.5 / 2.2 / 4.9 / 9.1 m, position NEES 3.3 / 6.1 / 27 / 81, attitude NEES 2.8 / 2.9 / 3.6 / 5.6. With an attitude prior of 0.01 deg instead of 0.1 the 3 km NEES is still 26, so the prior is not the cause. Renders and EuRoC stay within about 100 m of the origin, so 13.3 is not affected. | Everything at km scale: checking q_0 on flights, acceptance on flights, the LOST threshold. The monitor c does not see it (innovations are local). Must be fixed before any calibration on flights. |
| 2. Holding the last IMU sample to reach a frame time | not reproduced (the +5 ms experiment). With td fixed at 0 EuRoC frames land on IMU samples, which is why 13.3 uses that. | b and c: with td not 0 (our drone: about 30 ms, not synchronized) it adds residuals that grow with rotation rate, which b would read as IMU noise and c as overconfidence. k_g and the monitor wait for the fix. |
| 3. Online td unreliable | partly: online td ends at -3.3 / -1.5 / -7.5 ms for starts 20 / 22 / 24 on EuRoC (true about 0). | Same as 2. EuRoC with online td gives the same picture as td fixed (position NEES 14-143 against 6.8-284, rejections 33-36 % against 33-38 %): td is not why EuRoC is overconfident. |
| 4. Large NEES spread between nearly identical runs | yes, first table in 13.3. New: hover 4.3-20 position NEES. | The acceptance test and all correlations use medians over start times. |

Status 2026-11-26: problems 1 and 2 are fixed (moving origin, IMU interpolated at the frame time,
DESIGN.md section 11), the 13.3 baseline is re-measured on that code; 4 is handled by 20 starts and
the noise band (13.8); 3 stays (online td, your decision).
| 5. The two causes in DATASETS.md section 6 not established | Cause 1 (track drift) is now measured directly, without the filter (13.3, finding 1). Whether it explains the NEES needs the implementation. Cause 2 still cannot be separated on EuRoC; td is ruled out. | a rests on cause 1 (measured). b rests on cause 2 (not established), so it is off by default. |
| 6. Render results need a 1 px override not in `config/` | used here (`px1.yaml`). True white error of the renders 0.17-1.0 px. | sigma_w becomes a measured value per camera. Until `config/` has a render setting, every render command must name the override. |

### 13.10 Open questions for you

1. The split as recommended: a in the filter, c at the output, b built with k = 0?
2. LOST also when the horizontal sigma exceeds 3 m (below the 4.6.3 posErr cap), on top of the 2 s
   rule?
3. sigma_z in posErr: honest as today, or capped to the horizontal sigma because the EKF takes height
   from the baro? Decide in SITL.
4. Order: fix review problems 1-3 before implementing this? Recommended: at least 1 and 2 first.
5. A printed planar texture with Aprilgrid tags for the drift measurement at point C (a few extra
   minutes in CALIBRATION.md step 2)?

Answered: 1 in part (a approved behind a switch, 2026-11-26; b and c not decided), 4 (problems 1
and 2 were fixed first). New after part a (13.11):

6. One q does not fit hover and flight (renders: 0.011 against 0.05-0.06 px^2/deg). Options: a q per
   flight phase (speed), the frame multiplier of 13.4 a, landmarks with a smaller q than MSCKF
   tracks, or leave it to our recordings. Which, if any, should be tried on synthetic data next?
7. The hover NEES tail (7 of 20 runs above 8 with the switch on, 3 with it off) and the hover drift
   cause: investigate now or after our hover recordings?

Answered 2026-11-26: 6 and 7, nothing until our own recordings; q stays 0.05 as a placeholder, the
switch stays off. Parts b and c are on hold.

### 13.11 Part a, implemented and checked (2026-11-26)

Approved on 2026-11-26 for implementation behind a switch that is off by default; parts b and c,
the LOST thresholds and sigma_z are not decided.

What is built:

- `config/vio.yaml`: `filter.track_drift` with `enabled: false`, `q_px2_per_deg: 0.05` (placeholder,
  see below), `sigma_max_px: 4`. sigma_w is the existing `filter.pixel_noise_px`; no new white noise
  value.
- `src/msckf.cpp`: `track_noise()` and `accumulated_view_change()`; with the switch on, `linearize()`
  leaves out observations above sigma_max, whitens the track and everything after it uses unit
  noise (MSCKF tracks and landmark birth). Landmarks get the process noise q * dv * J+ J+^T before
  each update (`Filter::add_landmark_noise`), dv from the viewing direction at the previous update.
- `tools/gen_synthetic.py`: `--track-age N` (re-detect after N frames, new id) and `--drift-q Q`
  (random walk per degree of view change, accumulated, own random stream). Without them the output
  is byte for byte the same as before. A generated session with q 0.1 gives back q 0.0995 and a white
  part of 0.51 px (0.5 set).
- `tests/test_filter.cpp`: drift made as the generator makes it, whitened with `track_noise()`, has
  unit covariance (max deviation 0.026 over 20000 tracks) on a track whose view comes back; with the
  "since first" model the same check is off by a factor of 49.
- `tools/eval_runs.py`: new columns position inside 1 sigma, attitude inside 3 sigma, chi2 rejected.

Switch off: the output is identical to the code before this stage, file for file (all columns but the
three wall-clock columns `ms_*`): 100 replays, three renders and EuRoC (td 0 and online), 20 starts
each. Time per update frame with the switch on: within the noise of the parallel runs (renders 1.7
to 1.9 ms, EuRoC 5.1 to 5.3 ms, laptop).

Synthetic tracks with known drift (part B of the task). Figure8 with the generator defaults (915 m,
92 s), 20 seeds, `--track-age 72 --drift-q q`, white pixel noise 0.5 px; `config/vio.yaml` +
`config/replay_synthetic.yaml`, switch on with the true q. A track's accumulated view change at
the age cap is 19 deg (median), so the drift at its end is 0.75 / 1.4 / 2.4 px for q 0.03 / 0.1 /
0.3. Median (range):

| true q, px^2/deg | switch | NEES pos | NEES att | pos inside 1 / 3 sigma | chi2 rejected | ATE |
|---|---|---|---|---|---|---|
| 0 | off | 2.5 (1.2-5.9) | 2.7 (1.6-4.5) | 68 / 100 % | 5.1 % | 0.12 % (0.08-0.36) |
| 0 | on | the same medians and ranges | | | | |
| 0.03 | off | 9.4 (2.1-21) | 10 (5.9-26) | 42 / 91 % | 27 % | 0.27 % (0.09-0.62) |
| 0.03 | on | 3.1 (0.57-6.8) | 2.7 (1.7-4.3) | 71 / 100 % | 5.0 % | 0.20 % (0.08-0.42) |
| 0.1 | off | 59 (26-794) | 54 (33-107) | 16 / 49 % | 64 % | 1.4 % (0.63-15) |
| 0.1 | on | 3.0 (0.75-6.0) | 2.8 (1.8-4.6) | 68 / 100 % | 5.0 % | 0.31 % (0.11-0.47) |
| 0.3 | off | 64 (31-643) | 45 (18-86) | 16 / 45 % | 87 % | 4.2 % (1.8-115) |
| 0.3 | on | 3.1 (0.72-5.7) | 2.8 (1.8-5.8) | 66 / 100 % | 4.9 % | 0.41 % (0.17-0.73) |

These figure8 sessions fly at about 10 m/s, above the 4 m/s limit for landmarks, so they test only
the MSCKF part (review, 2026-11-26). Added for the landmark part: approach and hover
(`--scenario approach_hover --speed 8 --height 25 --duration 60 --drift-q q`, no `--track-age`, so
landmark features keep drifting for as long as they are seen), 20 seeds, hover window 12-60 s:

| true q, px^2/deg | switch | NEES pos | NEES att | pos inside 1 / 3 sigma | chi2 rejected | hover drift | landmarks in the hover |
|---|---|---|---|---|---|---|---|
| 0 | off | 1.5 (0.50-6.2) | 1.9 (0.99-3.7) | 82 / 100 % | 6.9 % | 0.12 m/min (0.03-0.64) | 12 |
| 0 | on | the same medians and ranges | | | | | |
| 0.05 | off | 12 (2.2-44) | 10 (3.4-26) | 32 / 86 % | 24 % | 0.81 m/min (0.19-7.0) | 4 |
| 0.05 | on | 2.7 (0.76-6.6) | 2.5 (1.2-6.6) | 68 / 100 % | 6.5 % | 0.35 m/min (0.04-0.78) | 11 |

With q 0.05 and the switch off, 15 of 20 runs are above 8 and far fewer landmarks survive (4
instead of 11-12; which guard drops them was not checked); with the true q no run is above 8 and
the landmarks stay. The landmark process noise works when q
is right. On the hover render it is not (0.011 measured there, 0.05 used; below).

Today's filter is overconfident on drifting tracks already at q 0.03 and falls apart at 0.1. With the
true q it is honest at every q: NEES about 3, no run above 8, 5 % rejections at the 95 % gate, two
thirds inside 1 sigma; all four sets pass 13.8. The ATE is also much smaller, because drifted
observations no longer pull the estimate. This shows the mechanism and its math under the model it
assumes; it does not show that real KLT drift follows that model (13.4 a: it does not in hover).

Renders and EuRoC, switch on, not tuned. q = 0.05 px^2/deg for all of them: the accumulated-view fit
on the true pixel error of the normal render (`fig8_tilt_img`, tracks without age cap, 0.049,
13.4 a), measured from the tracker output before any filter run with the switch on. Nothing was
fitted to NEES. The other sets are out of sample: their own fits are 0.06 (low contrast) and 0.011
(hover); EuRoC has no clean value (its reference attitude error dominates, 13.3). sigma_w stays at
1 px. Median (range) over 20 starts:

| Data | switch | NEES pos | NEES att | pos inside 1 / 3 sigma | chi2 rejected | ATE | other |
|---|---|---|---|---|---|---|---|
| `fig8_tilt_img` | off | 3.0 (0.80-11) | 2.4 (1.3-4.7) | 66 / 100 % | 0.1 % | 0.17 % (0.11-0.27) | RPE 0.49 % |
| | on | 2.2 (0.81-4.2) | 1.7 (1.0-2.9) | 75 / 100 % | 0.0 % | 0.18 % (0.11-0.28) | RPE 0.48 % |
| `fig8_tilt_low` | off | 10 (2.0-28) | 7.9 (3.8-16) | 42 / 91 % | 6.6 % | 0.45 % (0.14-0.80) | RPE 1.03 % |
| | on | 4.9 (1.2-8.5) | 3.8 (2.1-12) | 52 / 99 % | 1.4 % | 0.38 % (0.16-0.47) | RPE 0.79 % |
| `approach_hover_img` | off | 4.6 (1.5-13) | 3.5 (1.5-7.3) | 61 / 100 % | 0.1 % | 0.57 % (0.28-1.25) | hover 0.22 m/min (0.05-0.66) |
| | on | 5.4 (3.0-13) | 1.8 (0.95-2.5) | 60 / 96 % | 0.1 % | 0.85 % (0.47-1.47) | hover 0.67 m/min (0.14-1.19) |
| EuRoC, td 0 | off | 69 (7.4-281) | 42 (33-69) | 45 / 86 % | 34 % | 0.20 m aligned (0.11-0.37) | |
| | on | 70 (6.4-362) | 37 (33-92) | 42 / 85 % | 29 % | 0.21 m (0.13-253) | start 22.25 diverges |
| EuRoC, td online | off | 71 (8.1-1654) | 40 (33-443) | 38 / 78 % | 35 % | 0.21 m (0.11-1754) | start 25.75 diverges |
| | on | 59 (7.5-1150) | 37 (32-234) | 39 / 83 % | 29 % | 0.20 m (0.13-1521) | start 25.75 diverges |

Noise band (13.8), medians with pixel noise 0.999 / 1.000 / 1.001 px:

| Data | off: NEES pos, att | on: NEES pos, att | off / on: ATE or hover drift |
|---|---|---|---|
| `fig8_tilt_img` | 3.0-3.1, 2.4 | 2.2, 1.6-1.7 | ATE 0.172 / 0.177 % |
| `fig8_tilt_low` | 8.9-10.5, 7.9-9.1 | 4.8-4.9, 3.8-4.4 | ATE 0.39-0.45 / 0.36-0.39 % |
| `approach_hover_img` | 4.6-5.0, 3.3-3.6 | 5.3-5.4, 1.8-1.9 | hover 0.22 / 0.66-0.68 m/min |

Every change between off and on is larger than the band. The ATE of the normal render moves by 3 %,
inside the 10 % that 13.8 allows.

Reading:

- Low-contrast render: position NEES 10 to 4.9, attitude 7.9 to 3.8, chi2 rejections 6.6 to 1.4 %,
  ATE and RPE a little better, with a q from another render. This is the first direct evidence that
  track drift is a large part of the overconfidence there (cause 1 in DATASETS.md section 6). It
  still fails 13.8: position median 4.9 (limit 4.5), 4 of 20 runs above 8 in attitude.
- Normal render: more cautious, attitude NEES 1.7 is below the honest range (13.8 wants 2-4.5),
  accuracy the same. q = 0.05 is a little high for it in flight with the 1 px white noise on top.
- Hover render, worse: hover drift three times larger, ATE 50 % larger, position NEES tail 3 to 7
  runs above 8. The drone in this render drifts at about 0.3 m/s during the hover, so a landmark's
  view changes by about 0.1 deg per update (measured), which q = 0.05 turns into landmark process
  noise five times the hover's own fitted drift. Diagnostics, not results: with the landmark process
  noise switched off the hover drift is 0.28 m/min, with the hover's own q 0.011 it is 0.29 m/min.
  Both still have 7-8 runs above 8, so that tail comes with the MSCKF side, not from the landmarks.
- EuRoC: medians about the same (td 0: 69 to 70; online: 71 to 59), attitude NEES 42 to 37,
  rejections 34 to 29 %. Not graded. One more diverging run with td 0 (start 22.25); with online td
  start 25.75 diverges with the switch off as well.
- One q does not fit hover and flight. The same drift model gives 0.011 in hover and 0.05-0.06 in
  flight on the renders, and in hover where the view comes back neither view-change model predicts
  the error (13.4 a). This is the open point of part a (13.10, question 6).

