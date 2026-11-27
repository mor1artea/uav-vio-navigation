# Simulation: ArduPilot SITL + Gazebo, recorded flights

Flies a simulated drone and records the camera, the FC IMU, baro, landed flag and the ground truth
as a session that `vio_replay` reads. Open loop: our module does not steer the drone and sends
nothing to SITL. Closed loop (ODOMETRY back into EKF3) comes later, see `docs/SIMULATION.md`.

**State (2026-11-26): never run.** Neither `sim/setup.sh` nor any flight with Gazebo or SITL has been
run with these scripts. What was checked without running it is at the end of this file. The first run will show what is still wrong; the steps below say what you should see and what
to send back when it is not that.

## Needed

Native Linux, Ubuntu 22.04 or 24.04, a graphics card with a working OpenGL / EGL driver, about 15 GB
of disk for the tools, and about 3 GB per 5 km flight (camera images; an estimate).

## Step 1: install (once)

```
sim/setup.sh                      # asks for your sudo password; 30-60 min
```

Installs Gazebo Harmonic (apt), ArduPilot SITL from the `Copter-4.6.3` tag and the ardupilot_gazebo
plugin (commit f116b03) into `~/sim`, nothing into this repository. ArduPilot's own script adds lines
to `~/.profile` (on 24.04 it activates the Python venv `~/venv-ardupilot`, made with the system
packages, so the Gazebo Python modules stay visible).

You should see at the end: `Done. Log out and in again ...`. Log out and in again.
If it stops with an error: send the last 50 lines of the terminal.

## Step 2: build and make the world (once)

```
cmake -S . -B build -G Ninja && cmake --build build && ctest --test-dir build
sim/test_make_session.py
sim/make_world.py ~/sim/world_line
```

You should see: `100% tests passed`; then `ok` from the session test; then
`wrote .../world_line: 116 ground tiles (about 0.6 GB of graphics memory)` (a few minutes).

## Step 3: start the simulator (terminal 1, before every flight)

```
sim/run_sim.sh ~/sim/world_line            # add --gui to watch it
```

You should see: `Gazebo and SITL are running. SITL waits for sim/fly_record.py ...`. With `--gui` the
drone stands on grey textured ground. `~/sim/world_line/sitl/sitl.log` should end with
`Waiting for connection`. If the script says `one of them stopped`: send `~/sim/world_line/gazebo.log`
and `~/sim/world_line/sitl/sitl.log`.

Stop it with Ctrl-C after each flight and start it again before the next one (SITL starts with fresh
parameters every time).

## Step 4: a short first flight (terminal 2)

```
mkdir -p ~/datasets/sim
sim/fly_record.py ~/datasets/sim/first --route straight --leg 200 2>&1 | tee -i ~/datasets/sim/first.log
```

You should see, in this order: `waiting for SITL`; `arming (waits for the EKF and the simulated GPS)`
(up to a minute or two); `takeoff`; `turning (clock alignment)`; `flying to north 200 m, east 0 m`;
`landing`; `recorded N images, 0 dropped by this script`; then the checks, every line starting with
`ok`. Lines starting with `FC:` are messages from SITL, for information. The lines appear as they
happen (the script writes line by line; `tee -i` keeps the log complete if you stop it with Ctrl-C).

## Step 5: check the camera (once, on the first flight)

```
build/vio_track ~/datasets/sim/first --out ~/datasets/sim/first_tracks.csv --stats ~/datasets/sim/first_stats.csv
tools/eval_tracks.py ~/datasets/sim/first_tracks.csv ~/datasets/sim/first_stats.csv ~/datasets/sim/first
```

You should see in the correctness part: almost no tracks above 2 px (our own rendered ground: 0.1 %
over 10 frames). Many wrong tracks mean the lens, the mounting or the clock is wrong. Also open a few
images in `~/datasets/sim/first/mav0/cam0/data`: ground only, no parts of the drone, and the ground
moving from the top of the image to the bottom while flying forward.

## Step 6: the scale test flights

A straight 2.5 km leg north at constant speed, 12 m/s at 40 m, once without and once with wind (4 m/s
from the west, across the track, with gusts). Restart terminal 1 (step 3) before each.

```
sim/fly_record.py ~/datasets/sim/straight_calm --route straight --leg 2500 2>&1 | tee -i ~/datasets/sim/straight_calm.log
sim/fly_record.py ~/datasets/sim/straight_wind --route straight --leg 2500 --wind 4 --wind-from 270 --gust 1.5 \
    2>&1 | tee -i ~/datasets/sim/straight_wind.log
```

Each takes about 5 min of simulated time (longer if the simulation runs slower than real time) and
ends like step 4. For more flights of the same kind add `--seed 2`, `--seed 3` (other gusts) and a new
folder name; one flight is one number.

These 2.5 km legs cannot be compared directly with the 5 km synthetic numbers of DESIGN.md section 11
and DATASETS.md section 7: the error on a straight leg grows faster than the distance (so a 2.5 km
number is not half of a 5 km one), the start is different (takeoff and turns here, a start in flight
there), the autopilot's corrections on the simulated GPS add small accelerations, and the camera,
the ground texture and the noise are Gazebo's. Compare the midpoint off and on on the same flight,
and flights with each other.

## Step 7: replay with our filter, midpoint off and on

```
sim/replay.sh ~/datasets/sim/straight_calm off
sim/replay.sh ~/datasets/sim/straight_calm on
```

(the same for `straight_wind`). Each prints, and saves in `<session>/vio_off/summary.txt` or
`vio_on/summary.txt`: the filter log, ATE and RPE, the consistency (NEES), and the signed scale
(estimated / true distance; above 1 means the speed came out too high). The trajectory is in
`trajectory.csv` next to it.

## What to send back

If anything fails, or with the results of steps 5-7:

```
tar czf ~/sim_report.tgz ~/datasets/sim/*.log ~/sim/world_line/gazebo.log ~/sim/world_line/sitl/sitl.log \
    ~/sim/world_line/camera.yaml ~/datasets/sim/*/vio_*/summary.txt ~/datasets/sim/*/raw/camera.yaml
```

That is small (logs and summaries, no images). If the camera check of step 5 looks wrong, add three
images from `mav0/cam0/data` and `first_tracks.csv`.

## Options and problems

| Problem or wish | What to do |
|---|---|
| The check says "largest camera gap" FAIL, or images are dropped | slow the simulation: `sim/make_world.py ~/sim/world_line --rtf 0.5` |
| Gazebo crashes or the image is black | the wide-angle camera may not work on your driver: `sim/make_world.py ~/sim/world_line --lens pinhole` (100 deg instead of the fisheye) |
| Nothing arrives, or it hangs at the start | `~/sim/world_line/gazebo.log` and `~/sim/world_line/sitl/sitl.log` |
| Arming never succeeds | the `FC:` lines in the log say why; usually the EKF still waits for the simulated GPS: wait, or send the log |
| A square route | `--route square --leg 1250` and a world that covers it: `sim/make_world.py ~/sim/world_square --north -200 1450 --east -200 1450` |
| Wind | `--wind` mean speed, `--wind-from` direction (270 = from the west), `--gust` spread. Avoid wind from the east (`--wind-from` near 90): Gazebo low-pass filters the wind direction as a plain angle, which jumps at 180 deg |

## What these flights can and cannot say

- The autopilot holds the track on the simulated GPS (GUIDED). Our real drone flies in AltHold with a
  pilot and no GPS, so its corrections and speed changes will differ.
- The camera is Gazebo's: its image noise and track drift are not those of our camera. Noise values
  measured here cannot replace our camera's (DESIGN.md section 11, option D).
- Guesses, not measured: air drag (`make_world.py --drag`, sets the tilt in cruise), the gust model,
  the gyro noise with the motors running (`--gyro-noise`, SIM_GYR1_RND). The accelerometer noise of
  4.6.3 SITL is fixed at 0.01 m/s^2 per sample (SIM_ACC1_RND only scales simulated motor vibration,
  which is off). The filter uses the noise placeholders of `config/vio.yaml`, larger than this.
- Flat ground, no 3D structure, no motion blur, no rolling shutter, no vibration of our airframe.
- The FC filter settings match ours (`INS_GYRO_FILTER` 80, `INS_ACCEL_FILTER` 20); the harmonic notch
  is not simulated, and the accel/gyro delay difference of `config/vio.yaml` is not applied.
- `fly_record.py` arms and flies. It is for SITL only: it connects to this computer only and stops if
  the other side has no SITL parameters. Never point it at a real flight controller.

## What was checked without running it (2026-11-26)

Against the sources (read on GitHub, nothing downloaded): ardupilot_gazebo f116b03 (the Iris models,
the plugin block, the world and build instructions), Gazebo Harmonic (gz-sim 8 odometry publisher,
wind effects and wind mode, gz-sensors 8 wide-angle camera and sensor topic, gz-rendering 8 lens and
its shader, sdformat lens element, gz-msgs 10 wind message), ArduPilot Copter-4.6.3 (HIGHRES_IMU,
SITL noise parameters and the IMU simulation, the JSON interface, the SITL command line and serial
ports, `gazebo-iris` default parameters, the 24.04 install script).

- Correct as written: the model edits (names, quotes, `iris_with_standoffs::` references), the
  camera mounting and its `T_BS` (top of the image towards the nose), the equidistant lens
  (isotropic, fx = fy = image width / horizontal field of view = 340 px, centre in the middle), the
  topics, wind force and wind topic, the ground truth (the model pose, at the IMU, simulation time),
  the clocks (HIGHRES_IMU in FC time, images and truth in simulation time, aligned from the turns), the
  ENU / FLU to NED / FRD conversion.
- Fixed: SIM_ACC1_RND was set but has no effect without simulated vibration (option removed);
  SIM_GYR1_RND described correctly; `gazebo-iris.parm` replaced by `sim/vio-iris.parm` (it adds a sonar
  rangefinder and precision landing our drone does not have); the route `straight` and
  `sim/replay.sh` added; `gstreamer1.0-plugins-ugly` added to the setup as in the plugin README.
- Tested offline: `make_world.py` on stand-in model files built from the quoted source lines
  (well-formed output, all edits applied), `sim/test_make_session.py`, `sim/replay.sh` up to the
  filter start.
- Not checked: anything at run time (rendering on your driver, the speed of the simulation, arming,
  the flight itself); whether SITL's first serial port really waits for the connection (the default
  path was not found in the source; usual SITL behaviour); the IMU sample rates of SITL; the exact
  pixel centre convention of Gazebo images (at most 0.5 px); the sdformat tag name for wind (taken as
  `<enable_wind>`); possible NumPy clashes in the 24.04 venv (OpenCV from apt with a newer pip NumPy,
  as happened on the development laptop).
