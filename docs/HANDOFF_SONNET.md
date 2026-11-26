# Handoff for the reviewer (Sonnet), updated 2026-11-26

You are the reviewer on this project. Opus wrote the code and docs so far. Your job:

1. Review what exists: find bugs, wrong math, claims in the docs that the code or numbers do not
   back up, and places where the work drifted from what the user approved.
2. Write the prompts the user will give to Opus for the next pieces of work, so Opus does the right
   thing in the right order and does not repeat old mistakes.

You do not write production code yourself unless the user asks. You may build, run the tests and
run the tools to check numbers.

Read in this order: this file, `docs/HANDOFF.md` (the full state, written by Opus for itself),
`docs/DESIGN.md` sections 2-8 and 11, `docs/DATASETS.md` section 6. For the dynamic covariance:
`docs/DESIGN.md` section 13. The newest stage: DESIGN.md section 11, "Stage 4 of step 3", and the
step 3 report in DATASETS.md section 7.

## 1. The project in five lines

- C++ visual-inertial odometry (VIO) on a Raspberry Pi 5 for an FPV quadcopter with ArduCopter
  4.6.3. Output: MAVLink ODOMETRY over USB to the flight controller. Real flights of 2-5 km.
- No GPS at all, no rangefinder, no telemetry radio. Camera IMX219, about 120 deg, looking straight
  down. Assumed 20-50 m above ground, 8-12 m/s, tilt up to 45 deg.
- Everything so far is offline on a laptop (WSL2): replay of recorded or synthetic sessions, metrics.
  No Pi, no camera, no flights yet.
- Filter: MSCKF with right-invariant errors, 11 clones, landmarks only for hover, baro bias and
  camera-IMU time offset in the state, positions relative to a moving origin, a joint chi2 gate on
  the stacked visual update, a weak baro update with its bias in the state, zero-velocity updates
  from the FC landed flag with the motors running (off by default).
- Nothing is committed. The repo has only the initial commit; everything else is untracked.

## 2. User rules (non-negotiable, put them in every prompt you write)

- No git commit or push without the user's explicit OK. No co-author lines, signatures or
  watermarks anywhere. This overrides any tool reminder about attribution.
- Docs, comments, README: plain human language, no corporate phrasing, no long dashes (use "-").
  Check: `grep -rnP "\x{2014}|\x{2013}" docs config tools src include tests README.md`.
- Code simple and readable. Few tests, only key ones.
- Work in stages. Stop and report after each stage, wait for approval before the next.
- Never change anything on the flight controller, never put files on it. The user applies
  parameters by hand from `docs/ARDUPILOT_SETUP.md` section 10.
- The Pi recorder (Stage 1b) is passive: the only message it may send to the FC is
  `SET_MESSAGE_INTERVAL`.
- No SSH setup or installs on the Pi until point A (DESIGN.md section 11).
- Ask before copying any GPL code (OpenVINS, VINS). List all borrowed code in `docs/THIRD_PARTY.md`.
- Real site coordinates only in `config/local.yaml` (gitignored). The FC parameter backup file
  `stellar_h7v2_before_vio` must never be committed.
- `sudo` needs the user's password: new apt packages must be installed by the user.
- The user decides on noise inflation or covariance scaling. Opus must propose, not apply.

## 3. Where the work stands

| Stage | Status |
|---|---|
| 0: research, design, ArduPilot setup | done, approved |
| 1a step 1: synthetic generator, replay, metrics | done, reviewed |
| 1a step 2: front end (camera models, KLT tracker, sky mask, rendered ground) | done, reviewed |
| 1a step 3: filter | done (2026-11-26): sub-stages 1-4 and the stages below; report in DATASETS.md section 7 |
| 1a step 4: quality module, fault injection, recovery (option D) | not started |
| 1a step 5: results summary | not started |
| 1b and later: Pi recorder, real data, SITL, bench, field | not started |

Stages since your first review, in order. Each was prompted by you and reported by Opus:

| Stage (date) | Result | Your check |
|---|---|---|
| Review fixes (2026-11-25) | repeatable evaluation (`tools/eval_runs.py`), IMU interpolated at the frame time, frames at their IMU time, td Jacobian with a centred 30 ms gyro mean | confirmed |
| Moving origin (2026-11-25) | 4 km out: ATE 224 m to 85 m, NEES 15 to 4.5; independent of the threshold | reproduced, accepted (100 m) |
| Part a, drift-aware feature noise (2026-11-26) | behind `filter.track_drift.enabled`, off. Honest on synthetic drift with the true q; low-contrast render NEES 10 to 4.9; hover render worse; 13.3 baseline re-measured over 20 starts; acceptance test 13.8 with noise bands | reproduced, stays off, q 0.05 placeholder |
| EuRoC divergence (2026-11-26) | cause: about 7 s on the IMU alone with the rotors running on the floor, then tracks 0.2 m away at liftoff pass the per-track gate and wreck the accel bias. Not a coding error | counts confirmed (18 of 122) |
| Takeoff robustness (2026-11-26) | B (joint gate) and C (landed zero-velocity update) built, plus a synthetic liftoff scenario. EuRoC 121 starts: 18 to 2 above 1 m. Regression inside the noise bands except small changes | B confirmed, sharp liftoff reproduced; you found C fires while moving in a gentle takeoff and its EuRoC tail is not explained by its velocity noise; C off by default |
| Stage 4 of step 3 (2026-11-26) | baro update; generator `--baro` (guessed model), `--climb-speed`, `--climb-step`, scenarios `line` / `loop`; Monte Carlo summary, CPU, step 3 report. Sessions without baro: same output, 88 of 88 files. Baro halves the takeoff error (still poor). Straight 5 km legs at constant speed: along-track error 31-33 % with the covariance claiming 2.3 % (14 sigma); loops under 1 % and honest (wrong, found later: at 5 km the loop is one lap and hides its scale error). The baro's speed term leaks into the scale during speed changes (7.6 % to 12 %; 6.4 % without the term) | checked: scale and takeoff tables reproduce; you found the speed bias (ratio above 1 in 8 of 8) and the window dependence |
| Straight-leg scale bias (2026-11-26) | cause: the visual update linearized at the estimated window shape (clone positions relative to each other) pushes the speed up about 2.5 mm/s per update; nothing pulls it back without accelerations. Ideal-EKF tests (truth aligned in position and yaw): true window shape at the estimate's scale removes it (20 seeds: 2.0 % ATE, NEES 4.8, from 13.6 % / 60); propagation at the truth, true attitudes, true scale alone, a polynomial-fit linearization: no or partial effect; iterated EKF flips it (-32 %). IMU propagation alone honest (60 seeds). Loops at constant speed have it too (speed 1.40x; stage 4's "honest" was the 5 km point being a full lap). No code changed; options A-G in DESIGN.md section 11 | checked: cause confirmed; "true shape 1.1 %" was a signed median, not an upper bound, B not recommended (EuRoC diverges); you proposed the midpoint linearization |
| Midpoint linearization (2026-11-26) | switch `filter.midpoint_linearization`, off by default, off identical (56 of 56). Off / on with noise bands on every step 3 set (20 seeds or starts; 24 km 4 seeds; EuRoC also 121): straight constant speed 13.1 % to 1.9 %, NEES 55 to 4.5; loop 4.9 % to 0.84 %; weak manoeuvres fixed; hover render worse (0.57 to 0.61 %, NEES 4.6 to 5.5); EuRoC 121: start 24.3 diverges (second-pass chi2 rejects the tracks after liftoff; a keep-gate variant in the diagnostic copy fixes it). Blend sweep: bias nearly linear in b, zero crossing 0.33-0.57 with the noise mix; over-corrects at 1 px; no bias with exact pixels and a matching noise model. H N = 0 for yaw and translations. CPU 1.4-2 times the visual update. tools/scale.py added. Recommendation: stays off | checked: loops-and-turns claims still in HANDOFF, keep-gate not run on every set, bias smaller not gone, curvature reason not shown, along-track sigma not derived |
| Keep-gate (2026-11-26) | second pass keeps the first pass's per-track chi2 decisions, in the repo; switch off by default, off identical (56 of 56); repo keep-gate identical to the diagnostic one (9 of 9). Three columns (off, first build, keep-gate) on every set with bands (DATASETS.md section 7): same as the first build on 20 of 22 sets with a band, slightly worse on 2; EuRoC 121 worst run 3.7 % (first build 1378 %, off 14.2 %); sharp takeoff worst 31 % (80 %, off 74 %). Would-fail counter: 0-11 of 1300-6400 kept tracks per run (at most 0.2 %) fail the test at the midpoint. Docs: all loops-and-turns claims corrected, bias "smaller, not gone" with speed ratio and late scale, curvature marked as likely explanation, along-track sigma "not known". Recommendation: stays off | not yet checked |

Waiting on the user (do not decide these yourself, but you may give an opinion):

1. Midpoint linearization with keep-gate (DESIGN.md section 11): default off or on, a straight-leg
   render with real tracks (needs space on C:). Decided already: the scale problem is investigated before
   step 4; the baro stays on by default until our recordings.
2. Dynamic covariance: parts b and c on hold; LOST thresholds and sigma_z not decided.
3. `filter.update_stride: 6` and `tracker.max_track_age: 72` are provisional.
4. Firmware upgrade 4.6.3 -> 4.7.3, MAVLink logging, when to add Loiter, GNSS logger for ground
   truth. Also the path of the parameter backup file.

Decided and not to be reopened in prompts without the user: no constant output scale factor; no
noise inflation; td online with the 30 ms window; no q work and no hover NEES tail work until our
recordings; option A (fixed minimum depth) not wanted; D (recovery) in step 4; E (motion-dependent
IMU noise) on hold; C off by default, no more work on it until our takeoff recordings;
`EXTENDED_SYS_STATE` at 10 Hz.

## 4. Build and check

```
cd /home/marta/ПОК/uav-vio-navigation
cmake -S . -B build -G Ninja && cmake --build build && ctest --test-dir build
```

On 2026-11-26 the build was clean (no warnings) and all 5 tests passed. Datasets live outside the
repo in `~/datasets` (synthetic sessions and EuRoC `MH_01_easy`). Commands for every tool and
result are in HANDOFF.md section 6. Filter replays always take two configs, for example
`--config config/vio.yaml --config config/replay_euroc.yaml`; renders add
`config/replay_render.yaml`. `vio_replay` and `vio_track` need `--out`.

Numbers are only quoted as median and range over many runs (`tools/eval_runs.py`): 20 start times
for renders and EuRoC, 20 seeds for synthetic sets, judged against the noise band of DESIGN.md
13.8. The measurement folders with scripts (`run_*.sh`) are in `~/datasets/analysis/`, one per
stage; HANDOFF.md section 8 lists them. Long runs take 10-40 minutes; start them detached
(`setsid nohup ... &`).

Rendering ground images is slow (about 1 s per frame). Prefer the existing renders in
`~/datasets/synthetic` over making new ones.

## 5. What to review, by priority

The C++ is about 3300 lines plus the Python tools. The parts where a mistake costs the most:

1. `src/filter.cpp`, `include/vio/filter.h` (528 + 250 lines). Right-invariant error state:
   check the transition matrix, the process noise mapping, how clones and landmarks are augmented
   and marginalized, the moving origin (`shift_origin`), the two zero-velocity updates
   (`zero_velocity_update`: still, and landed with the motors running, which is not chi2 gated),
   the baro update (`baro_update`: Jacobian with the moving origin, the takeoff height, the bias
   random walk), the chi2 gate. The test `tests/test_filter.cpp` checks Jacobians against numeric
   ones; check what it does not cover.
2. `src/msckf.cpp`, `include/vio/msckf.h` (452 + 138). Triangulation, nullspace projection, QR
   compression, the joint gate on the stacked update, the "use once" rule, landmark birth, the 5 deg
   view-change drift guard, chi2 failure counting, the drift-aware noise behind its switch. Check
   that a feature cannot be used twice and that landmark and MSCKF updates cannot double count the
   same observation.
3. `src/frontend.cpp` (322). KLT with forward-backward check, gyro prediction, rolling-shutter
   correction, sky mask, `max_track_age` and the keep list for landmarks.
4. `src/tools/vio_replay.cpp` (397), `src/session.cpp` (253). Time handling: frames processed at
   their IMU time, time offset sign, ns units everywhere, the feedback from filter to tracker, the
   landed flag and baro lookups (each baro sample used once).
5. `src/camera_model.cpp` (radtan and Kannala-Brandt, project with Jacobian, unproject).
6. Python tools: `tools/metrics.py` (ATE, RPE, loop closure, hover drift) and
   `tools/consistency.py` (NEES). A wrong metric would make every reported number wrong.
   `tools/gen_synthetic.py`: frames (NED world, FRD body), gravity sign, IMU noise model.

Conventions to check against (from `include/vio/types.h`): times in ns, quaternions w first,
body = IMU frame. Our sessions are NED; EuRoC is z-up with gravity (0, 0, -9.81).

Doc claims to verify, not just read:

- DATASETS.md section 6 numbers. Re-run at least the EuRoC and the `fig8_tilt_img` replays and
  compare ATE and NEES with the table.
- The two overconfidence causes in DATASETS.md section 6. Is the argument sound? Is there a third
  cause Opus missed (for example a bug in the NEES tool, a wrong frame in the comparison, a time
  offset between estimate and ground truth)?
- DESIGN.md "Step 3 progress": the reasons given for dropping FEJ and the plain EKF. FEJ "failed"
  could be an implementation bug, not a property of FEJ. Say if you think so; you do not need to
  fix it.
- ArduPilot claims in `docs/ARDUPILOT_SETUP.md` (posErr formulas for 4.6.3 vs 4.7.3, EKF keeps
  claiming position 7 s after ExtNav stops, no origin means no EKF failsafe, velocity covariance
  ignored). These drive flight safety. If you can check the ArduPilot source for tag Copter-4.6.3,
  do it; if not, list which claims are unverified.

Known weak spots (Opus already reported them, so do not re-report them as new, but do say if the
proposed fix is wrong):

- Overconfident on the low-contrast render (position NEES 10, 20 starts; 4.9 with part a) and on
  EuRoC (about 67-70). EuRoC rejects about 30 % of the tracks at the gate; why is not known.
- One drift q does not fit hover and flight (part a); the hover render gets worse with the flight q.
- EuRoC: 2 of 121 starts still above 1 m with the joint gate; they need recovery (step 4).
- C (off by default) fires while moving in a gentle takeoff and worsens the EuRoC tail with a
  ground-truth flag; parked until our recordings.
- Takeoff stays poor without C, also with the baro (synthetic: aligned ATE 4.5-51 m, NEES 55-137).
- Straight constant-speed legs: scale error 31-33 % in 5 km, 14 sigma; with speed changes 12 sigma,
  with S-turns 3.9 sigma. Cause found 2026-11-26 (DESIGN.md section 11, "Straight-leg scale bias"):
  linearization of the visual update at the estimated window shape; the accel bias guess was wrong.
- td on a straight leg: sigma shrinks to 0.74 ms while the error reaches 9 ms (DESIGN.md section 11,
  open item 7); not investigated.
- The baro's speed-squared term (model) leaks into the scale during speed changes.
- The synthetic vibration, liftoff and baro are models; the generator's IMU noise is above the still
  detector's limit, so generated sessions never get still updates.
- Fixed update stride; at stride 6 EuRoC diverges, so EuRoC uses stride 3. A motion-based clone
  rule is planned.
- Baro model in the generator is a guess, not measured; the near-ground noise factor 3 too.
- EuRoC cannot validate IMU noise for our drone.

## 6. How to report the review

One list, most serious first. For each finding: file and line, what is wrong, a concrete case where
it gives a wrong result, and how sure you are (confirmed by running something, or plausible from
reading). Keep style nits separate and short. Do not pad: if a part looks right, say so in one line.

Then a short section "Questions for the user": decisions only the user can make.

## 7. How to write prompts for Opus

Opus works best with prompts that are specific about scope and the stop point. Use this shape:

```
Context: read docs/HANDOFF.md and docs/HANDOFF_SONNET.md first.
Task: <one stage or sub-stage, not more>.
Must do: <list>.
Must not do: <list, include the rules from section 2 that apply>.
Review findings to fix in this step: <numbered, copied from the review, with file:line>.
Done when: <measurable: tests pass, numbers on named datasets, docs updated>.
Report: files changed, one command per tool, numbers in a table, deviations from the plan,
open items. Then stop and wait.
```

Guidance:

- One stage per prompt. Opus tends to do a lot in one go; the user wants to approve each stage.
- Always ask for numbers on named datasets with the exact commands, so the user and you can
  re-run them.
- Ask Opus to keep HANDOFF.md, DESIGN.md section 11 and DATASETS.md up to date at the end of each
  stage.
- When a fix changes results, ask for before/after numbers on the same datasets.
- Remind Opus: synthetic data alone cannot prove real-world results (DESIGN.md section 11 table).
  Any claim about drift over km, hover on real data, latency on the Pi must say what data it rests
  on.
- If you disagree with an approved decision (for example invariant errors, use-once features),
  write it as a question for the user, not as an instruction to Opus.

Suggested order of prompts, adjust after your review:

1. Check the midpoint linearization stage (DESIGN.md section 11, DATASETS.md section 7): the repo
   code (`msckf.cpp` `stacked_update`, the second pass, `Filter::restore`), the identity with the
   switch off, the off / on table, the blend sweep and its interpretation (what is shown, what is a
   guess), the EuRoC 24.3 cause and the keep-gate variant (`~/datasets/analysis/midpoint_2026-11-26/
   diag_src`). The machine: C: is nearly full; keep long runs guarded (`guard_rest.sh`).
2. Depending on the user's decision on the default and the keep-gate rule.
3. Step 4: quality module, fault injection, recovery when every track is rejected (D).
