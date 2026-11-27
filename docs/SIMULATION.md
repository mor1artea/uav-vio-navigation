# ArduPilot SITL + Gazebo (stage between Stage 2 and Stage 3)

Assessment; only the open loop scripts of section 5 exist. Goal: fly the whole system in closed loop over kilometres with
perfect ground truth, before anything goes to the real FC.

## 1. Pieces

| Piece | What | Notes |
|---|---|---|
| ArduPilot SITL | built from the `Copter-4.6.3` tag, the same flight code as on the FC | `./waf configure --board sitl && ./waf copter` |
| Gazebo | Gazebo Harmonic (the version the ArduPilot plugin recommends) | needs OpenGL; under WSL2 this goes through WSLg, which may be slow |
| ardupilot_gazebo plugin | connects SITL and Gazebo through its JSON interface, no ROS | SITL runs in lock-step with the Gazebo physics |
| World | a flat or gently hilly ground several km across, textured with aerial images at about 0.1-0.3 m/px | split into tiles (one 4 km texture at 0.2 m/px would be 20000 x 20000 px); avoid repeating patterns, they confuse feature tracking |
| Drone model | the plugin's quad model with an added down-facing camera | Gazebo's wide-angle camera sensor supports an equidistant lens, so our ~120 deg lens can be imitated |
| Camera to our module | a small Gazebo transport subscriber, built only in a simulation target | image time stamp = simulation time |
| IMU and FC state | MAVLink from SITL (UDP), exactly like on the drone | `HIGHRES_IMU` etc. |
| Our output | ODOMETRY back to SITL | EKF3 in SITL fuses it like the real FC would |

Time: in lock-step SITL's clock follows the simulation time, and the camera images carry simulation
time too. The two differ only by a constant, which our normal clock sync removes. The simulation may
run slower than real time; that is fine for correctness, but nothing measured there says anything
about latency.

Rough work estimate, part-time: SITL + Gazebo + plugin on WSL2 1-2 days (more if WSLg rendering is a
problem; a native Linux machine is the fallback), world with textured ground 1-3 days, camera and
transport adapter 1-2 days, scenarios (long missions, vision loss, RC loss) 2-3 days. About 1-2 weeks.

## 2. What the simulation can validate

- Closed-loop flights over several km in Loiter and Auto, with vision as the only horizontal source.
- That EKF3 4.6.3 accepts our ODOMETRY: frames, covariance mapping, gating, `VISO_QUAL_MIN`, NaN
  handling, `reset_counter`, `VISO_DELAY_MS`.
- LOST and recovery as the EKF and the controller see them: the 7 s position timeout, the EKF
  failsafe, the position reset on recovery.
- Failsafe reactions: RC loss (`SIM_RC_FAIL`), EKF failsafe, with our real failsafe parameters.
  SITL also writes FC logs, so VISP/VISV/XKF are visible here even though the real FC has none.
- Perfect ground truth: drift in percent over 2-5 km, loop closure, hover drift.
- Down-facing specifics that the public datasets cannot show: flat ground far away, tilt up to 45
  deg with the horizon and sky in view (sky mask), scale on long smooth legs.
- The pilot procedure, rehearsed with a joystick or the RC through the GCS.

## 3. What it cannot validate

- Rolling shutter: Gazebo cameras are global shutter.
- Real motion blur, sensor noise, exposure and lighting (dusk, water, glare).
- Real vibration and its aliasing into our IMU stream (SITL vibration is a simple model).
- The exact FC IMU path: the ICM-42688 internal filters and its fast sampling are not simulated, and
  the USB link is replaced by local UDP.
- Real textures: an aerial photo draped on a plane has no real 3D structure, no moving leaves or
  water, and no parallax from trees and buildings.
- Latency and CPU on the Pi.
- Compass disturbance from the motors.

## 4. Injecting the FC IMU delay and filtering

SITL runs ArduPilot's own inertial sensor code on the simulated IMU samples. So the important part
comes for free once the SITL parameters match the FC:

- `INS_GYRO_FILTER = 80`, `INS_ACCEL_FILTER = 20` and the harmonic notch (`INS_HNTCH_*` with
  `MODE = 3`) run on the simulated IMU exactly as on the FC. With `SIM_ESC_TELEM = 1` SITL provides
  ESC RPM, so the per-motor notches track like on the drone. `SIM_VIB_MOT_MAX` / `SIM_VIB_FREQ` add
  motor vibration for the notches to work on.
- `HIGHRES_IMU` from SITL is produced by the same code and stamped at send time like on the FC.
- What is missing would be added in our MAVLink input adapter (not written yet), as a
  simulation-only option: the sensor's internal filter delay (about 1 ms gyro, 1.5 ms accel), a
  random transport delay and jitter similar to USB, and occasional gaps. For synthetic sessions the
  generator already does this (`tools/gen_synthetic.py --fc-imu`, DESIGN.md section 11, "Piece 3").
- IMU noise (checked in 4.6.3): the accelerometer gets a fixed uniform 0.01 m/s^2 per sample;
  `SIM_ACC1_RND` only scales simulated motor vibration (`SIM_VIB_*`); `SIM_GYR1_RND` adds gyro noise
  in deg/s times the throttle with the motors running. Matching our FC IMU (Allan variance,
  CALIBRATION.md step 4) would need the vibration model, not these two parameters alone.

## 5. Open loop scripts (2026-11-26)

`sim/` holds the first part, recording only: setup, a textured world with wind, a flight script for
SITL and the conversion into a session for `vio_replay` (`sim/README.md`). Moved before Stage 2 on
your request. Never run: neither `sim/setup.sh` nor any flight with Gazebo or SITL has been run
with these scripts; only the offline parts were tested (end of `sim/README.md`). Reviewed
against the ardupilot_gazebo, Gazebo Harmonic and Copter-4.6.3 sources on 2026-11-26 (what was
checked and fixed: end of `sim/README.md`). The scale test (straight leg at constant speed, with and
without wind) and its replay with the midpoint off and on are one command each (`sim/README.md`
steps 6 and 7). Closed loop (sections 1 and 2) is not started: it needs the MAVLink output (Stage 3
code). The quality module exists, offline only (step 4).
