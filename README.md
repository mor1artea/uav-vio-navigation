# uav-vio-navigation

Visual-inertial odometry for a down-looking camera on a Raspberry Pi 5, feeding position and
velocity to ArduCopter over MAVLink on a drone without GNSS. Design and plan: `docs/DESIGN.md`.

Current state: Stage 1a step 3 (the filter), stages 1-3 of 4 done. Replay, metrics, the front end
and the filter work on synthetic data, rendered images and EuRoC.

## Build (laptop or Pi)

Needs CMake, a C++17 compiler, Eigen 3, yaml-cpp and OpenCV 4. On Ubuntu:
`sudo apt install cmake g++ ninja-build pkg-config libeigen3-dev libyaml-cpp-dev libopencv-dev`.

```
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build
```

The Python tools need Python 3 with numpy and PyYAML.

## Generate, replay, measure

```
tools/gen_synthetic.py ~/datasets/synthetic/fig8
build/vio_replay ~/datasets/synthetic/fig8 --mode imu --out /tmp/fig8_imu.csv
tools/metrics.py /tmp/fig8_imu.csv ~/datasets/synthetic/fig8
```

- `gen_synthetic.py` writes a session in the EuRoC folder layout with exact ground truth
  (`--scenario figure8|hover`, `--no-noise`, `--camera equidistant|radial-tangential`, see `--help`).
  `--track-age 72 --drift-q 0.1` makes the tracks behave like KLT tracks: re-detected after 72
  frames and drifting with the view change (DESIGN.md section 13.4). `--scenario liftoff` stands on
  the ground with the motors running, then climbs, and writes the FC landed flag (DESIGN.md
  section 11, "Takeoff robustness"). `--scenario line` / `loop` fly km-long legs for the scale drift,
  `--baro` adds a simulated baro (a guess, DATASETS.md section 1). `--scenario liftoff` stands on
  the ground with the motors running, then climbs, and writes the FC landed flag
  (`mav0/landed0/data.csv`), which `vio_replay` uses for zero-velocity updates on the ground.
- `vio_replay` reads a session (synthetic or EuRoC) in time order and writes a trajectory.
  `--mode gt` writes the ground truth (checks the chain), `--mode imu` integrates the IMU only.
  `--start S` skips the first seconds (EuRoC MH: the drone is carried by hand at first, use 22),
  `--features file` reads tracks from `vio_track` instead of the images.
  `--mode vio` runs the filter. On a session with images and no `--features`, the tracker runs
  inside the replay and gets the filter's attitude, gyro bias and landmarks, as in the live system:
  `build/vio_replay ~/datasets/euroc/MH_01_easy --mode vio --start 22 --config config/vio.yaml --config config/replay_euroc.yaml --out /tmp/mh01_vio.csv`
  then `tools/consistency.py /tmp/mh01_vio.csv ~/datasets/euroc/MH_01_easy` checks the covariance.
  Rendered sessions also take `--config config/replay_render.yaml` (1 px pixel noise for real tracks).
- `eval_runs.py` repeats a replay over several start times or synthetic seeds and prints median and
  range. Single runs differ a lot, so results are quoted this way, over 20 runs:
  `tools/eval_runs.py ~/datasets/euroc/MH_01_easy --starts $(seq 21 0.25 25.75) --configs config/vio.yaml config/replay_euroc.yaml --rpe-length 10 --out /tmp/eval_euroc`
- `metrics.py` prints ATE (navigation and aligned), RPE on 100 m segments, loop closure error and,
  with `--hover START END`, hover drift, graded against the spec thresholds.

## Front end (feature tracking on images)

```
build/vio_track ~/datasets/euroc/MH_01_easy --no-sky-mask --start 22 --out /tmp/mh01_features.csv --stats /tmp/mh01_stats.csv
tools/eval_tracks.py /tmp/mh01_features.csv /tmp/mh01_stats.csv ~/datasets/euroc/MH_01_easy
build/vio_replay ~/datasets/euroc/MH_01_easy --features /tmp/mh01_features.csv --mode gt --out /tmp/mh01_gt.csv
```

- `vio_track` runs the tracker on the images of a session and writes the tracks (same format as the
  synthetic `features.csv`) and per-frame quality numbers. Settings come from `config/vio.yaml`;
  `--no-gyro`, `--clahe` / `--no-clahe`, `--frame-step N` for comparisons; `--no-sky-mask` for
  EuRoC, whose camera looks forward.
- `eval_tracks.py` prints track lengths, success rate, forward-backward error, grid coverage, time
  per frame and, with the session, the share of wrong tracks against the ground truth.
- `render_ground.py` renders ground images for a synthetic session (down-looking camera over flat
  textured ground), so the tracker can be checked in our own geometry:

```
tools/gen_synthetic.py ~/datasets/synthetic/fig8_tilt --speed 12 --radius 50 --height 25
tools/render_ground.py ~/datasets/synthetic/fig8_tilt ~/datasets/synthetic/fig8_tilt_img
```

Datasets and generated sessions live outside the repo, in `~/datasets`.
