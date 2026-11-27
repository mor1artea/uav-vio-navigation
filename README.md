# uav-vio-navigation

Visual-inertial odometry (VIO) for a small drone without GNSS. A camera looks straight down, a
Raspberry Pi 5 tracks features and runs a filter together with the flight controller's IMU, and
the goal is to send position and velocity to ArduCopter over MAVLink, so the drone can navigate
for 2-5 km with no GPS at all.

**Warning: this cannot run on a drone.** There is no camera capture, no MAVLink input or output,
no recorder and no live program. Nothing has been flown or tested on the hardware. Everything in
this repo runs offline, on recorded or generated data, on a Linux PC.

## Hardware (target)

- Flight controller: Stingbee Stellar H7V2, ArduCopter 4.6.3, one ICM-42688-P IMU, DPS310 baro,
  external compass. No GPS, no rangefinder, no SD card logs.
- Companion: Raspberry Pi 5 on the FC's USB port. Camera: IMX219 with a ~120 deg lens, looking
  straight down (the mount is not built yet).

## What works today (offline only)

- `vio_replay`: replays a session (EuRoC folder layout) and runs the filter: IMU propagation, MSCKF
  visual update, landmarks for hover, online camera-IMU time offset, weak baro update, quality
  monitor (GOOD / DEGRADED / LOST, logged only), fault injection (`--inject`). Writes a CSV.
- `vio_track`: the feature tracker (KLT with gyro prediction, sky mask) on a session's images.
- Python tools: a synthetic session generator, a ground renderer, metrics (ATE, RPE, hover drift),
  covariance consistency (NEES), repeated runs with median and range.
- Tested on synthetic data, rendered ground images and one real public sequence (EuRoC MH_01_easy).
  Results and their limits: `docs/DATASETS.md` sections 6-7 and `docs/HANDOFF.md`.
- `sim/`: scripts for open-loop flights in ArduPilot SITL + Gazebo. **Never run with Gazebo**, and
  `sim/setup.sh` was never run either.

## What does not exist yet

Camera capture, MAVLink I/O, the live module on the Pi, the Pi recorder and the converter from
recordings to sessions, calibration of the real camera, any real recording from our drone, closed
loop in simulation. `config/vio.yaml` already has sections for some of these (`camera`, `mavlink`,
`origin`, `recording`, `logging`); no code reads them yet.

## Build and test

Linux (Ubuntu 22.04 / 24.04, native or WSL2). Needs CMake, a C++17 compiler, Eigen 3, yaml-cpp,
OpenCV 4; the Python tools need Python 3 with numpy and PyYAML.

```
sudo apt install cmake g++ ninja-build pkg-config libeigen3-dev libyaml-cpp-dev libopencv-dev python3-numpy python3-yaml
cmake -S . -B build -G Ninja
cmake --build build
ctest --test-dir build
```

## Quick run (synthetic, about 5 s)

```
tools/gen_synthetic.py /tmp/fig8 --baro
build/vio_replay /tmp/fig8 --mode vio --config config/vio.yaml --config config/replay_synthetic.yaml --out /tmp/fig8_vio.csv
tools/metrics.py /tmp/fig8_vio.csv /tmp/fig8
tools/consistency.py /tmp/fig8_vio.csv /tmp/fig8
```

Expect about 0.15 % ATE and a position NEES of about 3-4 (honest is 3). Add `--inject blackout:40:3` to `vio_replay` to
see the quality states react to a 3 s camera blackout. More commands (EuRoC, rendered images,
repeated runs): `docs/DATASETS.md`. Data and generated sessions live outside the repo (default
`~/datasets`); nothing large goes into git.

## Documents

Reading order: this file, then `docs/HANDOFF.md`, then the first part of `docs/DESIGN.md`.

| Document | What it is |
|---|---|
| `docs/HANDOFF.md` | Current state, what is left, known problems, rules. Start here |
| `docs/DESIGN.md` | Design (sections 1-10) and the work plan with a journal of every experiment (section 11 on) |
| `docs/DATASETS.md` | Data sources, what each can show, how to run them, filter results |
| `docs/ARDUPILOT_SETUP.md` | What ArduPilot 4.6.3 does with our data, FC parameters to apply by hand, pilot procedure |
| `docs/RECORDING.md` | Specification of the Pi recorder (not built) |
| `docs/CALIBRATION.md` | How to calibrate camera, camera-IMU and IMU noise (not done yet) |
| `docs/GROUND_TRUTH.md` | How flights will get ground truth (GNSS logger, not chosen yet) |
| `docs/SIMULATION.md` | SITL + Gazebo assessment; `sim/README.md` has the step-by-step scripts |
| `docs/THIRD_PARTY.md` | Licenses, borrowed code (none so far), rules for copying code |
