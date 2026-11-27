# Handoff (2026-11-26)

Where the project stands, what was decided, and what comes next. Read this first when picking the
work up again, then `docs/DESIGN.md`.

## 1. State

Stage 0 (research and design) is done and approved. Stage 1a steps 1 (replay, metrics) and 2 (front
end) are done and reviewed. Step 3 (the filter) is complete with stage 4 (2026-11-26, checked);
the step 3 report is DATASETS.md section 7. The straight-leg scale bias stage (2026-11-26) is
reported and waiting for your answer. Since the review of 2026-11-25 (Sonnet,
`docs/HANDOFF_SONNET.md`) these stages were done and reported; 1-6 were also checked by the
reviewer:

1. Review fixes (2026-11-25): repeatable evaluation, IMU interpolated at the frame time, frames at
   their IMU time, a less noisy td Jacobian.
2. Moving origin (2026-11-25, accepted, 100 m): positions inside the filter relative to an origin
   that follows the drone. 4 km out: ATE 224 m to 85 m, NEES 15 to 4.5.
3. Part a of the dynamic covariance (2026-11-26, accepted, stays off): drift-aware feature noise
   behind `filter.track_drift.enabled`, q 0.05 as a placeholder. Honest on synthetic drift with the
   true q; low-contrast render NEES 10 to 4.9; hover render worse with the flight q.
4. EuRoC divergence (2026-11-26): about 7 s on the IMU alone while the rotors spin on the floor,
   then tracks 0.2 m away at liftoff that pass the per-track gate and wreck the accel bias.
5. Takeoff robustness (2026-11-26, checked): B (joint gate on the stacked MSCKF update, always on)
   and C (zero-velocity update from the FC landed flag with the motors running). EuRoC: 2 of 121
   starts above 1 m instead of 18. C is off by default since the review (it worsens the EuRoC tail
   with a ground-truth flag and fires while moving in a gentle takeoff); the code stays.
6. Stage 4 of step 3 (2026-11-26, checked by the reviewer): baro update (weak, bias in the state,
   chi2 gated), takeoff climb options, km-scale scale-drift sessions, Monte Carlo summary, CPU, step
   3 report. Found: on straight legs at constant speed the scale error reaches 31-33 % in 5 km and
   the covariance does not show it (14 sigma); a baro error that follows the speed leaks into the
   scale; the baro halves the takeoff error but the takeoff stays poor.
7. Straight-leg scale bias (2026-11-26, reported, reviewed): cause found, no code changed. The
   visual update, linearized at the estimated clone positions (their shape within the window is
   off), pushes the speed up by about 2.5 mm/s per update on average; with no acceleration nothing
   pulls it back. Jacobians at the true window shape remove it (2.0 % instead of 13.6 % ATE, NEES
   4.8 instead of 60, 20 seeds). Not a coding error; an iterated EKF biases it the other way (-32 %).
   Also on loops at constant speed (speed 1.40x), which stage 4 had called honest. Options A-G in
   DESIGN.md section 11, "Straight-leg scale bias".
8. Midpoint linearization (2026-11-26, reported, reviewed): switch `filter.midpoint_linearization`,
   off by default; off identical to before (56 of 56). Straight leg at constant speed 13.1 % to 1.9 %
   ATE, NEES 55 to 4.5, bias smaller but not gone (speed ratio at 95 % 1.51 to 1.055, scale over the
   last 10 % 1.107); loop 4.9 % to 0.84 %; better or the same everywhere except the hover render
   (0.57 % to 0.61 %, NEES 4.6 to 5.5) and the EuRoC tail of the first build.
9. Keep-gate (2026-11-26, reported, waiting): the second pass keeps the first pass's per-track chi2
   decisions (in the repo); switch still off by default, off identical (56 of 56). Same as the first
   build within the bands on 20 of 22 sets; EuRoC 121 starts worst run 1378 % to 3.7 % (off 14.2 %),
   sharp takeoff worst 80 % to 31 %; the test now uses the prior Jacobians, the update the midpoint
   ones (DESIGN.md section 11). Accepted; the switch stays off.
10. Pixel noise study (2026-11-26, reported, reviewed; corrected in stage A of the next prompt): no code or default changed. Real track noise
   measured (white-equivalent per filter window 0.24-0.91 px on the renders, at most 0.67 px on EuRoC;
   drift q 0.004-0.076 px^2/deg). On synthetic straight legs with that drift and 1 px told the speed
   is 2.24 times too high (midpoint off) or 0.85 (on); telling the measured value instead makes the
   renders overconfident (NEES 3 to 20) and the hover diverge; with drift-aware noise (part a) and the
   right white value the midpoint works (0.95, NEES 3.6) at low noise only. Options A-E in DESIGN.md
   section 11, "Pixel noise study".
11. Stages A and B of the step 4 prompt (2026-11-26): A, doc fixes of the noise study (accepted). B,
   `sim/` reviewed against the ardupilot_gazebo f116b03, Gazebo Harmonic and Copter-4.6.3 sources
   (read on the web, nothing installed or run); fixed: SITL noise parameters, `sim/vio-iris.parm`
   instead of `gazebo-iris.parm`, route `straight`, `sim/replay.sh <session> off|on`,
   `config/midpoint_on.yaml`, setup packages; `sim/README.md` rewritten as step by step instructions.
   Accepted with the tee / buffering fix (done). C: step 4 plan written in DESIGN.md section 11,
   "Step 4 plan" (proposal, nothing implemented), revised after your review: position sigma drives
   neither DEGRADED nor LOST (states from the frame signals and the velocity sigma; C_filter sent
   unchanged, the EKF caps posErr at 10 m); the straight-leg speed bias is a known blind spot of the
   monitor (options E1-E5); recovery D3 and D2 tested, D1 only after asking you, G1 with a
   covariance test, no G2; time-based growth in DEGRADED dropped; the accel delay compensation is its
   own item (3a). Sections 7, 13.6 and 13.10 updated to match. Approved with changes (E1 log
   only, E3 not now, E5 flight rule in ARDUPILOT_SETUP.md section 8, limits 0.9 / 2 m/s, K = 5).
12. Step 4 piece 1, quality monitor (2026-11-26): `quality.h/.cpp`, `test_quality`, 14 new columns
   at the end of the vio_replay rows (`quality.enabled`, on by default; off gives the old file).
   332 replays: old columns identical in all. Results and the proposed score formula in DESIGN.md
   section 11, "Piece 1". Accepted (2026-11-27) with report fixes: all sets, DEGRADED score floor
   from `output.viso_qual_min` (only LOST is not fused), EuRoC DEGRADED is a false alarm.
13. Step 4 piece 2, fault injection (2026-11-27): `inject.h/.cpp`, `test_inject`, `vio_replay
   --inject`. 480 fault replays and 641 monitor replays (old columns identical in all). Results, the
   LOST exit alternatives L0-L3 and the open findings (frozen and outliers with few tracks, overconfidence
   after a 0.5 s IMU gap, td step not followed) in DESIGN.md section 11, "Piece 2". Waiting for your
   decisions and OK before piece 3.

Your standing decisions: no constant output scale factor, the covariance becomes dynamic step by
step (parts b and c on hold); update stride 6 and age cap 72 provisional; td online with the 30 ms
window, chosen on our own recordings; no q or hover NEES work until our recordings; option A (depth
guard) not wanted, D (recovery) in step 4, E (motion-dependent IMU noise) on hold; C off by default
until our takeoff recordings; `EXTENDED_SYS_STATE` at 10 Hz; stage 4 of step 3 off hold; the
straight-leg scale error is investigated before step 4 (step 4 waits); the baro stays on by
default, decided again on our recordings. Nothing has been committed: the repo still has only the
initial commit, everything else is untracked.

Simulation (2026-11-26, on your request, before Stage 2): `sim/` records open loop flights from
ArduPilot SITL + Gazebo (`sim/README.md`, `config/replay_sim.yaml`). The flight script was run against
SITL with a stand-in for Gazebo; the Gazebo part waits for its first run on your Linux machine.

## 2. Rules for this project (yours, keep them)

- No commit or push without your explicit OK. No co-author lines, signatures or watermarks.
- Docs, comments and README in plain language, no long dashes (use "-"). Check before finishing:
  `grep -rnP "\x{2014}|\x{2013}" docs config tools src include tests README.md`.
- Code simple and readable, few tests, only the key ones.
- Never change anything on the flight controller and never put files on it. You apply parameter
  changes by hand from `docs/ARDUPILOT_SETUP.md` section 10.
- Stage 1b (the Pi recorder) is passive: the only message it may send to the FC is
  `SET_MESSAGE_INTERVAL`.
- Do not set up SSH to the Pi or install anything on it until point A of the plan (DESIGN.md
  section 11).
- Before copying any GPL code (OpenVINS, VINS), stop and ask. Permissive code (Basalt, Kimera) may be
  borrowed in pieces with attribution, listed in `docs/THIRD_PARTY.md`. No LICENSE file yet.
- Real site coordinates only in `config/local.yaml` (in `.gitignore`). The parameter backup file
  `stellar_h7v2_before_vio` must never be committed.

## 3. Hardware and firmware, short

- FC: Stingbee Stellar H7V2, ArduCopter V4.6.3 (92b0cd78): the official 4.6.3 tag plus locally
  added board files (vendor build). One ICM-42688 IMU, DPS310 baro, external compass, no SD card,
  no FC logs (`LOG_BACKEND_TYPE = 0`).
- Drone: no GPS at all, no rangefinder, no telemetry radio. Mode switch today: Acro / Stabilize /
  AltHold, no Loiter.
- Pi 5 (aarch64, host `zlp.local`, later) on the FC USB port. Camera IMX219 ~120 deg, looking
  straight down, mount not built yet.
- The 4.6.3 quirks we design around, the filter delays (gyro 4.8-9.9 ms, accel 12.8-13.4 ms), the
  failsafe values and their meaning: all in `docs/ARDUPILOT_SETUP.md`.

## 4. Plan and where we are

| Stage | Content | Status |
|---|---|---|
| 0 | Research, design, ArduPilot setup | done, approved |
| 1a step 1 | synthetic generator, replay, metrics | done, reviewed |
| 1a step 2 | front end: camera models (radtan, Kannala-Brandt), KLT, forward-backward check, grid, gyro prediction, sky mask, rendered ground sequences | done, reviewed |
| 1a step 3 | filter: IMU propagation with covariance, MSCKF update, landmarks, time offset, static init | done (2026-11-26): stages 1-4, review fixes, moving origin, part a (off), EuRoC divergence, takeoff robustness; report in DATASETS.md section 7. Straight-leg scale bias: cause found; midpoint linearization built behind a switch (off), evaluated, decision waiting |
| 1a step 4 | quality module, fault injection, "FC IMU" degradation, recovery when every track is rejected (option D) | |
| 1a step 5 | results on synthetic data and one EuRoC sequence | |
| 1b | Pi recorder (spec in `docs/RECORDING.md`), own recordings, camera calibration | needs the Pi (point B) |
| 2 | quality on real data | |
| 2b | ArduPilot SITL + Gazebo (`docs/SIMULATION.md`, assessment only, do not start early) | |
| 3 | MAVLink output on the real FC, bench | |
| 4 | field | |

DESIGN.md section 11 says which results need real data and when the Pi is needed (points A-E).

## 5. Waiting on you

1. Decided (2026-11-26): the midpoint switch stays off until there is real data; no more noise or
   linearization studies; Gazebo / SITL code only (you run it on the Linux machine); next is step 4,
   in stages A (doc fixes), B (finish sim/), C (step 4 plan), D (step 4).
2. Dynamic covariance: parts b and c on hold; LOST thresholds, sigma_z not decided.
3. The path of the parameter backup file, to fill the "?" entries in ARDUPILOT_SETUP.md section 10.
4. Decisions: firmware upgrade to 4.7.3 or not; MAVLink logging (`LOG_BACKEND_TYPE = 2`, decide before
   Stage 3); when to add Loiter to the mode switch (after the simulation stage); GNSS logger for
   ground truth.

## 6. Code

### Code from step 1

| File | What |
|---|---|
| `CMakeLists.txt` | library `vio_core`, `vio_replay`, tests |
| `include/vio/types.h` | shared types; times in ns, quaternions w first, body = IMU frame |
| `include/vio/session.h`, `src/session.cpp` | reads a session in the EuRoC layout (images or `features.csv`), replays it in time order, interpolates ground truth |
| `include/vio/imu_propagation.h`, `src/imu_propagation.cpp` | midpoint IMU integration of the mean state |
| `src/tools/vio_replay.cpp` | `--mode gt` (ground truth) or `--mode imu` (IMU only); the filter becomes a third mode in step 3 |
| `tools/gen_synthetic.py` | synthetic sessions: NED world, FRD body, figure8 or hover, IMU with noise and bias drift, down-looking camera, features without images |
| `tools/metrics.py` | ATE (navigation and aligned), RPE on 100 m, loop closure, hover drift, graded against the spec |
| `tests/test_imu_propagation.cpp`, `tools/test_metrics.py` | the two tests, both run by `ctest` |
| `tools/prepare_euroc.sh` | unpacks or checks a hand-downloaded EuRoC sequence |

Build and run:

```
cmake -S . -B build -G Ninja && cmake --build build && ctest --test-dir build
tools/gen_synthetic.py ~/datasets/synthetic/fig8
build/vio_replay ~/datasets/synthetic/fig8 --mode imu --out /tmp/fig8_imu.csv
tools/metrics.py /tmp/fig8_imu.csv ~/datasets/synthetic/fig8
```

Results so far: ground-truth replay gives exactly 0 on all metrics. Noise-free IMU-only replay stays
within 0.5 mm over 915 m (4 mm over 245 m at up to 50 deg tilt), so generator and propagation agree on
frames, gravity, rates and time stamps. With noise, IMU-only drifts by hundreds of metres in 90 s, as
expected.

Generated sessions: `~/datasets/synthetic/fig8`, `fig8_clean`, `hover` (outside the repo).

### Code from step 2

| File | What |
|---|---|
| `include/vio/camera_model.h`, `src/camera_model.cpp` | radial-tangential and Kannala-Brandt: project (with Jacobian), unproject to unit bearings |
| `include/vio/frontend.h`, `src/frontend.cpp` | gyro rotation between frames, gyro prediction, rolling-shutter correction, sky mask (ray table), image metrics, the tracker |
| `src/tools/vio_track.cpp` | runs the tracker on a session's images, writes the tracks (`--out`, required) and per-frame stats (`--stats`, optional) |
| `src/session.cpp`, `src/tools/vio_replay.cpp` | `--features` (read tracks from a file), `--start` (skip the hand-carried part of EuRoC MH) |
| `tools/eval_tracks.py` | track statistics and track correctness against the ground truth |
| `tools/render_ground.py` | renders ground images for a synthetic session (down-looking fisheye over flat textured ground) |
| `tools/prepare_euroc.sh` | now extracts only cam0, imu0, ground truth and body.yaml |
| `tests/test_camera_models.cpp`, `tests/test_frontend.cpp` | the two new tests |

```
build/vio_track ~/datasets/euroc/MH_01_easy --no-sky-mask --start 22 --out /tmp/mh01_features.csv --stats /tmp/mh01_stats.csv
tools/eval_tracks.py /tmp/mh01_features.csv /tmp/mh01_stats.csv ~/datasets/euroc/MH_01_easy
tools/gen_synthetic.py ~/datasets/synthetic/fig8_tilt --speed 12 --radius 50 --height 25
tools/render_ground.py ~/datasets/synthetic/fig8_tilt ~/datasets/synthetic/fig8_tilt_img
build/vio_track ~/datasets/synthetic/fig8_tilt_img --out /tmp/tilt_features.csv --stats /tmp/tilt_stats.csv
```

Results: DATASETS.md section 2 (EuRoC) and section 5 (rendered ground). Open items from step 2:
DESIGN.md section 11.

### Code from the midpoint linearization stage (2026-11-26)

| File | What |
|---|---|
| `include/vio/msckf.h`, `src/msckf.cpp` | `filter.midpoint_linearization` (off): `ClonePose(s)`, `linearize(..., lin)` (Jacobians at given clone poses, point triangulated there, residual at the estimate), `stacked_update()` (per-track chi2, stacking, QR, joint gate as before, now a helper; `passed` / `only` for keep-gate), `process()` runs it once, or twice with the switch on; the second pass keeps the first pass's per-track decisions (keep-gate stage) |
| `include/vio/filter.h`, `src/filter.cpp` | `Filter::restore(saved)`: back to a copy, time counters kept |
| `src/config.cpp`, `config/vio.yaml` | `filter.midpoint_linearization: false` |
| `src/tools/vio_replay.cpp` | stdout summary also gives the whole visual update time per update frame (CSV unchanged) |
| `tools/scale.py` | signed scale: estimated / true distance on 100 m pieces of the path (works on closed loops), speed ratio at 95 % |
| `tools/test_metrics.py` | new test: scale.py on a 3 % too large circle in another frame |

```
tools/eval_runs.py --seeds $(seq 1 20) --gen "--scenario line --duration 420 --speed 12 --height 40 --radius 800 --density 0.004 --manoeuvre none --baro" --configs config/vio.yaml config/replay_synthetic.yaml on.yaml --out /tmp/eval/line_on
tools/scale.py /tmp/eval/line_on
tools/scale.py /tmp/eval/tilt --session ~/datasets/synthetic/fig8_tilt_img
```

(`on.yaml`: `filter: {midpoint_linearization: true}`.) All runs of the stage, `run_eval.sh` (every
set of the step 3 report, off and on, with the noise band), `run_sweep.sh` (blend factor, needs the
diagnostic binary with `DIAG_BLEND`), `summarize.py`, `table_md.py`, `check_sessions.py`, the
diagnostic copy `diag_src` (`DIAG_BLEND`, `DIAG_KEEP_GATE`, the H N check, the would-fail counter),
the keep-gate stage (`run_identity2.sh`, `run_kg2.sh` with `guard_kg2.sh`, `table3_md.py`) and
`STATUS.md`: `~/datasets/analysis/midpoint_2026-11-26/`. Three-column table again:
`python3 ~/datasets/analysis/midpoint_2026-11-26/table3_md.py` (from the repo root).

### Straight-leg scale bias stage (2026-11-26)

No repo code changed. All runs, the throwaway diagnostic copy of the code (`diag_src`, switches
listed in `results.md` and `diag_src/include/vio/diag.h`), both binaries and the scripts
(`run_*.sh`, `speed.py`, `account.py`, `imu_check.py`, `gen_pixonly.py`) are in
`~/datasets/analysis/scale_bias_2026-11-26/`. Main commands (repo root):

```
W=~/datasets/analysis/scale_bias_2026-11-26
tools/eval_runs.py --seeds $(seq 1 20) --gen "--scenario line --duration 420 --speed 12 --height 40 --radius 800 --density 0.004 --manoeuvre none --baro" --configs config/vio.yaml config/replay_synthetic.yaml $W/cfg/nobaro.yaml --out /tmp/eval/line20
python3 $W/speed.py /tmp/eval/line20
DIAG_GT_VIS=1 DIAG_GT_MODE=shape tools/eval_runs.py --replay $W/bin/vio_replay_diag --seeds $(seq 1 20) --reuse --configs config/vio.yaml config/replay_synthetic.yaml $W/cfg/nobaro.yaml --out <copy of /tmp/eval/line20>
```

### Code from stage 4 of step 3 (2026-11-26)

| File | What |
|---|---|
| `include/vio/filter.h`, `src/filter.cpp` | `baro_update(altitude)`: weak vertical update with the baro bias in the state, noise inflated near the takeoff height and in fast climbs, chi2 gated; takeoff height from a start at rest; `zupt_landed` off by default |
| `include/vio/session.h`, `src/session.cpp` | `Session::baro` from `mav0/baro0/data.csv` |
| `src/tools/vio_replay.cpp` | newest baro sample at each update frame, each used once; summary counts baro updates |
| `src/config.cpp`, `config/vio.yaml` | `filter.baro.*` read (new: `inflate_factor: 3`); `filter.zupt.landed.enabled: false` |
| `tools/gen_synthetic.py` | `--baro` (model is a guess), `--climb-speed`, `--climb-step`, scenarios `line` and `loop` with `--manoeuvre none / speed / sturn` |
| `tests/test_filter.cpp` | `test_baro`: keeps the height, finds the offset, horizontal sigma unchanged |

```
tools/eval_runs.py --seeds $(seq 1 8) --gen "--scenario line --duration 420 --speed 12 --height 40 --density 0.004 --baro" --configs config/vio.yaml config/replay_synthetic.yaml --out /tmp/eval/line
tools/eval_runs.py --seeds $(seq 1 20) --gen "--scenario liftoff --duration 20 --speed 8 --height 6 --climb-speed 0.3 --climb-step 2 --blind-below 1.0 --landed-rate 10 --baro" --configs config/vio.yaml config/replay_synthetic.yaml --out /tmp/eval/gentle
```

Runs, scripts (`run_stage.sh`, `scale_metrics.py`, `liftoff_metrics.py`, `bands.py`, the diagnostic
generator without the baro speed term) and `montecarlo.md`: `~/datasets/analysis/step3_stage4_2026-11-26/`.

### Code from the takeoff stage (2026-11-26)

| File | What |
|---|---|
| `src/msckf.cpp`, `include/vio/msckf.h` | B: the stacked MSCKF update is chi2 gated as a whole; `MsckfStats::update_rejected` |
| `include/vio/filter.h`, `src/filter.cpp` | C: `zero_velocity_update(image_still, image_moving, landed)`: the still update as before, or a landed one (v only, not gated) when the flag is fresh, the image does not move and the mean accel is gravity |
| `include/vio/session.h`, `src/session.cpp` | `Session::landed` from `mav0/landed0/data.csv`; feature rows with id -1 mark frames without features |
| `src/tools/vio_replay.cpp` | landed flag at each update frame (`max_flag_age_s`), output columns `static` (0 / 1 still / 2 landed) and `update_rejected`; the tracker runs only for sessions with images |
| `src/config.cpp`, `config/vio.yaml` | `filter.zupt.landed`: `enabled` (true then, false since the review), `max_flag_age_s: 0.1`, `accel_max: 0.3` |
| `tools/gen_synthetic.py` | `--scenario liftoff` with vibration, near-ground points, `--blind-below`, landed flag (`--landed-rate`, `--landed-clear`) |
| `tests/test_filter.cpp` | `test_landed_zupt`: applied on the ground with the motors running, not with the flag off, a moving image or a climb |

```
tools/eval_runs.py --seeds $(seq 1 20) --gen "--scenario liftoff --duration 20 --speed 8 --height 25" --configs config/vio.yaml config/replay_synthetic.yaml --out /tmp/eval/liftoff
tools/eval_runs.py ~/datasets/euroc/MH_01_easy --starts $(seq 21 0.05 27) --configs config/vio.yaml config/replay_euroc.yaml --rpe-length 10 --out /tmp/eval/euroc121
```

Runs, scripts and the 4.6.3 source files read: `~/datasets/analysis/takeoff_2026-11-26/` and
`~/datasets/analysis/ardupilot_src_4.6.3/`.

### Code from the EuRoC divergence stage (2026-11-26)

| File | What |
|---|---|
| `src/tools/vio_replay.cpp`, `src/tools/vio_track.cpp` | `--out` is required, `vio_track --stats` is optional: no default files in the current folder (a stray `stats.csv` had ended up in the repo root) |
| `README.md`, `docs/HANDOFF.md` | example commands write to `/tmp` |

No filter change. The investigation (61-start runs, the diagnostic copy of the code in `diag_src`
with track and update printouts and the two option switches, `first_update.py`, `gtcheck.py`,
`run_hover_drift.sh`) is in `~/datasets/analysis/euroc_divergence_2026-11-26/`.

```
tools/eval_runs.py ~/datasets/euroc/MH_01_easy --starts $(seq 24 0.05 27) --configs config/vio.yaml config/replay_euroc.yaml --rpe-length 10 --out /tmp/eval/euroc61
```

### Code from the drift-aware noise stage (2026-11-26, part a of DESIGN.md section 13)

| File | What |
|---|---|
| `include/vio/msckf.h`, `src/msckf.cpp` | `track_noise()`, `accumulated_view_change()`; with `filter.track_drift.enabled` the track is whitened in `linearize()` (unit noise afterwards), observations above `sigma_max_px` left out, landmarks get process noise from their view change |
| `include/vio/filter.h`, `src/filter.cpp` | `add_landmark_noise(j, Q)` |
| `src/config.cpp`, `config/vio.yaml` | `filter.track_drift`: `enabled: false`, `q_px2_per_deg: 0.05` (placeholder from the normal render), `sigma_max_px: 4` |
| `tools/gen_synthetic.py` | `--track-age N` (re-detect, new id) and `--drift-q Q` (KLT-like drift); without them the same output as before |
| `tools/eval_runs.py` | new columns: position inside 1 sigma, attitude inside 3 sigma, chi2 rejected |
| `tests/test_filter.cpp` | `test_track_noise`: whitened drifting track has unit covariance, also when the view comes back |

```
tools/eval_runs.py --seeds $(seq 1 20) --gen "--track-age 72 --drift-q 0.1" --configs config/vio.yaml config/replay_synthetic.yaml on_q0.1.yaml --out /tmp/eval/drift01
tools/eval_runs.py ~/datasets/synthetic/fig8_tilt_low --starts $(seq 0 0.5 9.5) --configs config/vio.yaml config/replay_synthetic.yaml config/replay_render.yaml drift_on.yaml --out /tmp/eval/low_on
```

(`on_q0.1.yaml`: `filter: {track_drift: {enabled: true, q_px2_per_deg: 0.1}}`; `drift_on.yaml`:
`filter: {track_drift: {enabled: true}}`.) All runs of this stage, the scripts (`run_baseline.sh`,
`run_synthetic.sh`, `run_chaos.sh`, `viewfit.py`) and the binaries are in
`~/datasets/analysis/drift_noise_2026-11-26/`.

### Code from the moving-origin stage (2026-11-25)

| File | What |
|---|---|
| `include/vio/filter.h`, `src/filter.cpp` | `origin()`, `world_state()`, `shift_origin(d)`, `shift_origin_if_far()`; initialization puts the origin at the start |
| `src/tools/vio_replay.cpp` | shifts after each update frame, writes world positions, reports the number of shifts |
| `config/vio.yaml` | `filter.origin_shift_distance_m: 100` |
| `tests/test_filter.cpp` | `test_origin_shift`: world positions and ordinary covariance unchanged, also when shifting to the current position |
| `tools/eval_runs.py` | keeps the generator arguments in each seed folder; a mismatch stops with an error; `--reuse` for sessions made elsewhere |
| `tools/gen_synthetic.py` | looks only at landmarks within 250 m in x (same output byte for byte, about 6 times faster on km flights) |

The runs of this stage (sessions, both binaries, `run_all.sh`, `results.txt`) are in
`~/datasets/analysis/moving_origin_2026-11-25/` (outside the repo, about 2 GB).

### Code from the review-fix stage (2026-11-25)

| File | What |
|---|---|
| `tools/eval_runs.py` | the same replay over several start times or seeds, median and range of ATE, RPE, hover drift, NEES, td |
| `config/replay_render.yaml` | 1 px pixel noise for rendered sessions (real tracks) |
| `src/session.cpp` | `replay_in_imu_time`: frames at frame time + td, for both signs of td, with the next IMU sample |
| `src/filter.cpp` | `propagate_to(t, next)` interpolates the IMU at the frame time; `clone_td_jacobian`; `augment_clone` takes a centred gyro mean |
| `src/frontend.cpp` | `average_gyro` |
| `src/tools/vio_replay.cpp` | uses the above; counts late frames; output column `td_sigma_ms` |
| `tools/gen_synthetic.py` | `--td MS` (camera stamps written td earlier) |
| `tests/test_filter.cpp` | new: clone td Jacobian, frame time between two IMU samples, frame order for td -4 / +4 / -2 ms |

```
tools/eval_runs.py ~/datasets/synthetic/fig8_tilt_img --starts 0 0.034 0.067 0.1 0.134 0.167 --configs config/vio.yaml config/replay_synthetic.yaml config/replay_render.yaml --out /tmp/eval/tilt
tools/eval_runs.py ~/datasets/euroc/MH_01_easy --starts 21.5 22 22.05 22.1 22.5 23 --configs config/vio.yaml config/replay_euroc.yaml --rpe-length 10 --out /tmp/eval/euroc
tools/eval_runs.py --seeds 1 2 3 4 5 6 7 8 --gen "--speed 12 --radius 50 --height 25 --laps 3 --td 10" --configs config/vio.yaml config/replay_synthetic.yaml td30.yaml --out /tmp/eval/td10
```

(`td30.yaml`: `imu: {td_initial_ms: 0.0, td_prior_sigma_ms: 30.0}`. Use `--gen=--no-noise` when the
generator argument starts with a dash.)

### Code from step 3 (stages 1-3)

| File | What |
|---|---|
| `include/vio/filter.h`, `src/filter.cpp` | error-state EKF with right-invariant errors: IMU state, td, baro bias, landmarks, clones; covariance propagation, still detector, static start, zero-velocity updates, generic chi2-gated update |
| `include/vio/msckf.h`, `src/msckf.cpp` | visual update: tracks, triangulation, nullspace projection, QR compression, "use once", landmarks (birth from a track, update, drift guard) |
| `include/vio/config.h`, `src/config.cpp` | layered config (`vio.yaml` plus override files), filter, MSCKF and tracker settings |
| `config/replay_euroc.yaml`, `config/replay_synthetic.yaml` | overrides for dataset replays (priors, IMU noise from the session, stride) |
| `src/tools/vio_replay.cpp` | `--mode vio`; frames processed at their IMU time; with images and no `--features` the tracker runs inside and gets attitude, gyro bias and landmark ids from the filter |
| `src/frontend.cpp` | `max_track_age` (re-detect old features) and the keep list for landmarks |
| `tools/consistency.py` | NEES, share inside 1 and 3 sigma, error/sigma, Monte Carlo band |
| `tools/gen_synthetic.py` | new scenario `approach_hover` |
| `tests/test_filter.cpp` | transition matrix and feature Jacobians against numeric ones, static start |

```
tools/gen_synthetic.py ~/datasets/synthetic/approach_hover --scenario approach_hover --speed 8 --height 25 --duration 60
tools/render_ground.py ~/datasets/synthetic/approach_hover ~/datasets/synthetic/approach_hover_img
build/vio_replay ~/datasets/synthetic/approach_hover_img --mode vio --config config/vio.yaml --config config/replay_synthetic.yaml --out /tmp/hover_vio.csv
tools/metrics.py /tmp/hover_vio.csv ~/datasets/synthetic/approach_hover_img --hover 12 69
tools/consistency.py /tmp/hover_vio.csv ~/datasets/synthetic/approach_hover_img
build/vio_replay ~/datasets/euroc/MH_01_easy --mode vio --start 22 --config config/vio.yaml --config config/replay_euroc.yaml --out /tmp/mh01_vio.csv
```

Results: DATASETS.md section 6, as median and range over 20 start times or seeds (the single-run
numbers that stood here are not comparable). Short version, today's code with default settings
(joint gate included; it changes nothing on the renders and very little on the synthetic sets):
honest on synthetic tracks (figure-eight, 8 seeds: ATE 0.15 %, NEES 2.9 / 2.5) and on the
normal-contrast render (ATE 0.17 %, NEES 3.0 / 2.4); hover render drift 0.22 m/min, NEES 4.6 / 3.5;
overconfident on the low-contrast render (NEES 10 / 7.7) and on EuRoC (online td, 121 starts:
aligned ATE 0.21 m, NEES 67 / 40). Why: not established. Measured so far: track drift explains a
large part on the low-contrast render (part a of DESIGN.md section 13 brings it to 4.9 / 3.8) but
not on EuRoC; the IMU side of EuRoC cannot be separated from its reference (DATASETS.md section 6,
"Why the filter is overconfident"). Km-scale legs and takeoff (stage 4, corrected 2026-11-26 after
the review, 20 seeds): flights that do not accelerate often are inaccurate and overconfident,
straight legs and loops alike (constant speed: straight 13 % ATE, NEES 55; loop 4.9 %, NEES 35;
speed changes every 40 s: 6.4 % and 2.5 %, NEES 57 and 31); S-turns every 40 s bring both near 1 %
(straight 0.99 %, NEES 6; loop 0.49 %, NEES 4.2); manoeuvres every 80 s or more do not; takeoffs are
poor. The earlier "loops and flights with turns are fine" was wrong (a loop of R 800 m closes at
5 km and hides its scale error). Full table: DATASETS.md section 7. Deviations from the plan and open items:
DESIGN.md section 11.

## 7. Things to remember

- EuRoC: z-up world, gravity (0, 0, -9.81), confirmed on MH_01_easy. Our sessions use NED and say so
  in `session.yaml`. MH_01: hand-carried 1-20 s, still 20-38 s, takeoff at about 44 s; runs use
  `--start 22`. The zip is no longer needed, the unpacked sequence is in `~/datasets/euroc/MH_01_easy`.
- EuRoC cam0 looks forward: always `vio_track --no-sky-mask` there.
- CLAHE is off by default since step 2 (no gain on EuRoC or rendered ground, 2 ms per frame);
  `vio_track --clahe` turns it on for comparisons.
- The default figure-eight is gentle (about 13 deg tilt). For tilt up to 43 deg:
  `--speed 12 --radius 50 --height 25` (`fig8_tilt`). Rendered: `fig8_tilt_img` (normal contrast),
  `fig8_tilt_low` (low contrast), `approach_hover_img` (hover test). Rendering takes about 1 s per
  frame. Also in `~/datasets/synthetic`: `approach_hover`, `approach_hover_clean` (no noise).
- Filter replays always take two configs: `--config config/vio.yaml --config config/replay_euroc.yaml`
  (or `replay_synthetic.yaml`). Synthetic sessions start from the ground truth state (they start in
  flight), EuRoC with a static start. For the renders add `--config config/replay_render.yaml` (1 px
  pixel noise); the synthetic default of 0.5 px fits the generator's feature files, not real tracks.
- Filter decisions you approved: right-invariant errors instead of first-estimate Jacobians; no
  noise inflation; "use once" plus the tracker age cap; landmarks only for hover. EuRoC cannot
  validate IMU noise for our drone (DATASETS.md section 6).
- Synthetic sessions with features only (no images) cannot show track drift; use the renders with
  the tracker inside `vio_replay` for anything about real tracks.
- Laptop: WSL2 Ubuntu 24.04, AMD Ryzen 7 7435HS. Eigen 3.4, OpenCV 4.6, yaml-cpp 0.8 are installed.
  No `unzip`. `sudo` needs your password, so new apt packages have to be installed by you. Two numpy
  versions exist (system 1.26, user 2.5); the tools work with both. Python cv2 exists only in the
  user site, the tools do not use it.
- From WSL the Pi is only reachable through Windows `ssh.exe` (WSL cannot resolve `.local`), and no
  SSH keys exist yet. Not needed before point A.
- Generator options added in step 3 that change what a session contains: `--baro` (writes
  `mav0/baro0/data.csv`; off by default so older sessions stay byte-identical), `--scenario liftoff`
  (writes `mav0/landed0/data.csv`), `--scenario line` / `loop` with `--manoeuvre`. The baro and the
  vibration are guessed models. A session with a baro file gets baro updates (`filter.baro.enabled`
  is true by default); a config override `filter: {baro: {enabled: false}}` switches them off.
- The temporary scratch folder of the session gets cleared now and then; nothing important lives
  there. Use absolute paths, a failed `cd` once dropped downloads into the repo.
- Long runs (EuRoC 121 starts, regressions with noise bands: 10-40 min) must be started detached
  (`setsid nohup ... &`): sessions have ended in the middle of a run twice, and runs started from
  the session die with it.
- `vio_replay` and `vio_track` need `--out`; nothing is written to the current folder by default.
- Output column `static`: 0 none, 1 still update, 2 landed update; `update_rejected`: the joint
  gate skipped the stacked update.
- The generator's IMU noise (the drone placeholder, 0.57 m/s^2 per sample) is above the still
  detector's limit: generated sessions never get still updates. The liftoff scenario therefore
  starts armed.
- EuRoC MH_01: rotors run on the floor from 38.8 s, liftoff (ground truth up 1 cm) at 45.21 s. A copy
  with a landed flag taken from the ground truth: `~/datasets/analysis/takeoff_2026-11-26/euroc_with_flag`
  (diagnostic only).
- ArduPilot 4.6.3 source files that were read are in `~/datasets/analysis/ardupilot_src_4.6.3/`
  (downloaded from the public tag, treated as data). The landed flag: ARDUPILOT_SETUP.md section 2.

## 8. What comes next

First your answer on the stage 4 report (DESIGN.md section 11, "Stage 4 of step 3"; DATASETS.md
section 7). Open items are listed in DESIGN.md section 11 ("Open after the review fixes, the moving
origin, part a, the EuRoC liftoff check and stage 4"). Runs and scripts, all outside the repo in
`~/datasets/analysis/`: `dynamic_cov_2026-11-25/` and `drift_noise_2026-11-26/` (section 13),
`moving_origin_2026-11-25/`, `euroc_divergence_2026-11-26/` (with `diag_src`, a throwaway copy of
the code that prints every track and update), `takeoff_2026-11-26/`, `step3_stage4_2026-11-26/`,
`scale_bias_2026-11-26/` (with `diag_src`, a throwaway copy with ground-truth Jacobian switches).
The session scratch folder gets cleared, sometimes during a stage: keep anything needed later
there.

Findings to keep in mind:

- Single runs mislead: NEES varies by up to 40x between start times only seconds apart, and six
  start times are not enough either (pixel noise 0.999 / 1.001 px moved a six-start median from 18
  to 20.5 and 11). Use 20 start times or seeds, report median and range, and judge a change against
  the noise band of DESIGN.md section 13.8 (`tools/eval_runs.py`).
- The track error is a random walk in the view change of the feature, not white noise. Its white
  part is 0.17-1.0 px, below the configured 1 px. The view change accumulated along the track fits
  better than the change since the first observation (2026-11-26). Fitted q on the renders: 0.05-0.06
  px^2/deg in flight, 0.011 in hover.
- Part a (switch `filter.track_drift.enabled`): honest on synthetic drift with the true q; with one
  q from the normal render the low-contrast render improves a lot and the hover render gets worse.
- EuRoC diverged after liftoff in about 15 % of the start times (18 of 121); with the joint gate 2
  (22.35, 25.45). Those two go wrong through updates that pass every gate; only recovery (D) would
  catch them.
- The landed zero-velocity update (C, off by default) removes the 7 s on the IMU alone in the sharp
  synthetic liftoff, but in a gentle takeoff it fires while the drone climbs (review: 8 of 10 runs
  with the flag 1 s late), and on EuRoC with a ground-truth flag more runs end above 1 m (11 of
  121, not explained by its velocity noise). The vibration, the flag timing on our FC and C's real
  effect wait for our recordings (motors running on the ground, takeoffs in Stabilize and AltHold).
- Takeoff without C stays poor even with the baro (aligned ATE 4.5-51 m, NEES 55-137 in the
  synthetic sets): the velocity error at liftoff is 0.8-1.6 m/s.
- 4.6.3 landed flag: Stabilize and Acro clear it at throttle above zero after the spool-up (a fast
  push can lift off before that), AltHold after the liftoff. The flag alone is never enough; in a
  gentle takeoff the image and accelerometer checks do not stop the update either (review), so C is
  off.
- Scale drift (stage 4, synthetic): straight legs at constant speed drift 31-33 % along the track in
  5 km with the covariance claiming 2.3 % (14 sigma); speed changes 7.6-12 %, S-turns 2.2 %. The
  loops are not "under 1 % and honest": at constant speed the speed ends 1.40x too high, hidden
  because the 5 km point is almost a full lap. The baro gives no scale on level legs, but a baro
  error that follows the speed leaks into the scale during speed changes (diagnostic without the
  speed term: 6.4 % instead of 12 %).
- Cause of that scale drift (2026-11-26): a bias of the visual update from linearizing at the
  estimated window shape, adding up when nothing accelerates. Test tools for such questions: Jacobians
  from the ground truth must be aligned to the estimate in position and yaw first (invariant errors
  contain absolute positions), otherwise the filter diverges for the wrong reason. Linearizing at the
  updated estimate (iterated EKF) flips the bias, it does not remove it. IMU propagation alone is
  honest (60 seeds).
- EuRoC rejects 33-38 % of the tracks at the 95 % chi2 gate, with td online or fixed at 0, and still
  29 % with part a on. Why is not known; track drift alone does not explain it.
- The filter's own residuals cannot measure the drift (relative pose error dominates), and KLT
  straight from the birth image fails after about one second of flight.
- Midpoint linearization (2026-11-26, switch off by default): removes most of the scale bias on
  synthetic flights without accelerations; the bias is nearly linear in where between prior and
  update the Jacobians are taken, and where it crosses zero depends on the noise mix (b = 0.5 fits
  0.5 px with our generator's IMU noise, over-corrects at 1 px). Still overconfident along the track
  (error/sigma rms 2.7). One EuRoC start of 121 diverges with it (second-pass chi2 tests reject the
  tracks after the liftoff). DESIGN.md section 11.
- Noise-band runs can write to RAM (`/dev/shm`) and keep only their `.txt` summary (`run_kg2.sh`): the
  WSL disk on C: grows with every write and does not give back deleted space (deleting 1.6 GB of band
  run files on 2026-11-26 did not slow the fall of free space on C:).
- Since the evening of 2026-11-26 the WSL disk is on D: (about 185 GB free) and C: has about 50 GB;
  the WSL swap is still on C:. Before long runs check `df -m /mnt/d /mnt/c`, stop below 10 GB on either
  (`guard_study.sh` in the noise study folder). The notes about C: below are history.
- Scripts that use SciPy (`noise_fit.py`, `viewfit.py`) need `python3 -s`: the user site has NumPy 2.5,
  which the system SciPy cannot load; `-s` uses the system NumPy 1.26.
- Real track noise: `~/datasets/analysis/noise_study_2026-11-26/noise_fit.py <repo> <vio_track
  features> <session> --stride N [--plane]` (white-equivalent per window, white part and drift q
  against the ground truth). It needs true camera poses with attitude: works on renders, EuRoC and
  Gazebo / SITL, not on our recordings with the planned GNSS-only ground truth (GROUND_TRUTH.md).
  Values measured in Gazebo are those of the simulated camera, not of ours, so they cannot replace
  our camera's white part and q (option D in DESIGN.md section 11).
- WSL restarted three times on 2026-11-26. The journal of the last one shows read and write errors
  on the swap device; memory inside WSL was never short. C: (holds the WSL disk and its swap) was
  94 % full (about 9 GB free). Watch `df -m /mnt/c`, stop long runs below about 8 GB free (the
  stage folder has `guard_rest.sh`); measured RAM per job: replays 66-360 MB, a 420 s generator 120 MB.
  Check generated sessions after a restart (`check_sessions.py`): files written at the moment of
  the restart were empty or cut.
- `eval_runs.py --hover A B` must come after the config files: a config after it is taken as the
  unused session argument and silently dropped. `pkill -f <name>` from a shell whose command line
  contains that name kills the shell itself; kill by PID.
- Results depended on the distance from the world origin (invariant errors); the moving origin
  handles it. Any km-scale test must still get far from its start; a moved start alone proves
  nothing.

Stage 4 of step 3 is done and checked. The straight-leg scale problem is understood (2026-11-26,
DESIGN.md section 11, "Straight-leg scale bias") and the midpoint linearization is built and
evaluated (switch off by default); before step 4, waiting for your decision on its default, the
keep-gate rule and a straight-leg render.

Not in step 3: the quality module and recovery (step 4), the rest of the dynamic covariance
(DESIGN.md section 13, parts b and c, if approved), the FC IMU delay model (a later replay option),
live output to the latest IMU sample (Stage 3), a motion-based clone rule instead of the fixed
stride (open item), the tlog converter that writes `mav0/landed0/data.csv` for our recordings
(Stage 1b, RECORDING.md section 6).
