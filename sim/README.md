# Simulation: ArduPilot SITL + Gazebo, recorded flights

Flies a simulated drone and records the camera, the FC IMU, baro, landed flag and the ground truth
as a session that `vio_replay` reads. Open loop: our module does not steer the drone and sends
nothing to SITL. Closed loop (ODOMETRY back into EKF3) comes later, see `docs/SIMULATION.md`.

**State (2026-11-26): the Gazebo part has never been run.** Checked on the development laptop:
`fly_record.py` flies the whole route against a `Copter-4.6.3` SITL (with a stand-in for Gazebo), the
session conversion has a test, `make_world.py` writes well-formed files and `vio_replay` loads the
converted layout. The world, the camera, the wind and the Gazebo topics are written from the Gazebo
Harmonic sources and need their first run. `setup.sh` has not been run either.

## Needed

Native Linux, Ubuntu 22.04 or 24.04, a graphics card with a working OpenGL driver, several GB of
disk for the tools and about 3 GB per 5 km flight (estimate).

## Once

```
sim/setup.sh                      # asks for your sudo password, takes a while
```

Log out and in again, so that the changes to `~/.profile` take effect. Then build this repository and
make the world (a few minutes):

```
cmake -S . -B build -G Ninja && cmake --build build && ctest --test-dir build
sim/test_make_session.py
sim/make_world.py ~/sim/world_line
```

## One flight

Terminal 1:

```
sim/run_sim.sh ~/sim/world_line            # add --gui to see it
```

Terminal 2:

```
sim/fly_record.py ~/datasets/sim/line1 --world ~/sim/world_line --speed 12 --height 40 --wind 4 --gust 1.5
```

It takes off, turns twice, flies 2.5 km north and back, lands, then converts and checks the data.
Every line of the check must say `ok`. Stop terminal 1 with Ctrl-C and start it again before the
next flight.

For a first try use a short leg: add `--leg 200`.

## First run: check the camera before trusting any number

The lens and the mounting are what could be wrong without being visible. Track features on a piece
of the flight and compare them with the ground truth:

```
build/vio_track ~/datasets/sim/line1 --start 60 --duration 60 --out /tmp/sim_f.csv --stats /tmp/sim_s.csv
tools/eval_tracks.py /tmp/sim_f.csv /tmp/sim_s.csv ~/datasets/sim/line1
```

On flat textured ground almost no track should be above 2 px reprojection error (our own rendered
ground: 0.1 % over 10 frames). Large errors mean the camera model, the mounting or the clock is wrong.
Also look at a few images in `mav0/cam0/data`: ground only, no drone parts in the middle, the image
moving from top to bottom when flying forward.

## Replay

```
build/vio_replay ~/datasets/sim/line1 --mode vio --config config/vio.yaml --config config/replay_sim.yaml --out /tmp/line1.csv
tools/metrics.py /tmp/line1.csv ~/datasets/sim/line1
tools/consistency.py /tmp/line1.csv ~/datasets/sim/line1
```

One flight is one number. Fly several with different `--seed` (gusts) before comparing anything.

## Options and problems

| Problem or wish | What to do |
|---|---|
| The check says "largest camera gap" FAIL, or frames are dropped | slow the simulation: `sim/make_world.py ... --rtf 0.5` |
| Gazebo crashes or the image is black | the wide angle camera may not work on your driver: `sim/make_world.py ... --lens pinhole` (100 deg instead of the fisheye) |
| Nothing arrives, or it hangs at the start | `<world>/gazebo.log` and `<world>/sitl/sitl.log` |
| A square route | `--route square --leg 1250`, and a world that covers it: `sim/make_world.py ~/sim/world_square --north -200 1450 --east -200 1450` |
| Wind | `--wind` mean speed, `--wind-from` direction (270 = from the west), `--gust` spread. Avoid `--wind-from 90`: Gazebo averages the direction angle, which breaks at 180 deg |

## What these flights can and cannot say

- The autopilot holds the track with the simulated GPS. Our real drone flies in AltHold with a
  pilot and no GPS, so its corrections and speed changes will differ.
- Guesses, not measured: air drag (`--drag`, sets the tilt in cruise), the gust model, the IMU noise
  (`--gyro-noise`, `--accel-noise`). The filter uses the noise placeholders of `config/vio.yaml`.
- Flat ground, no 3D structure, no motion blur, no rolling shutter, no vibration of our airframe.
- The FC filter settings match ours (`INS_GYRO_FILTER` 80, `INS_ACCEL_FILTER` 20); the harmonic notch
  is not simulated, and the accel/gyro delay difference of `config/vio.yaml` is not applied.
- `fly_record.py` arms and flies. It is for SITL only: it connects to this computer only and stops if
  the other side has no SITL parameters. Never point it at a real flight controller.
