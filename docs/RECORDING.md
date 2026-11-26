# Recording and replay (Stage 1b)

Status: specification only. The recorder is built after the offline VIO work on public datasets
(Stage 1a, DESIGN.md section 11). The replay part is built first, in Stage 1a, for the datasets.

The recorder runs on the Pi during every bench session and flight. Replay runs the same recordings
on any Linux machine (x86 or aarch64). Most debugging happens on recordings, not in the air.

## 1. Stage 1 is passive

The recorder only listens. The one thing it sends to the FC is `MAV_CMD_SET_MESSAGE_INTERVAL` (as
`COMMAND_LONG`, target the FC) to ask for the streams below. It does not send:

- HEARTBEAT, TIMESYNC, PARAM_* requests, REQUEST_MESSAGE;
- an EKF origin, vision data, parameter changes, mode or source set commands.

One possible later exception, only with your OK: receiving the FC log over MAVLink
(`LOG_BACKEND_TYPE = 2`) needs the recorder to start the log and ACK its blocks (ARDUPILOT_SETUP.md
section 13).

`SET_MESSAGE_INTERVAL` is not saved on the FC: after a reboot of the FC the rates are gone, so the
recorder sends the requests again whenever it sees the FC reboot (HEARTBEAT gap, `SYSTEM_TIME`
going back) or the USB link reconnect.

## 2. There are no FC logs (`LOG_BACKEND_TYPE = 0`, no SD card)

Everything that would normally be read from the FC dataflash log has to come from MAVLink messages
recorded on the Pi. By default (`SR0_*` = 0) the FC sends nothing on USB except HEARTBEAT and
STATUSTEXT, so every stream below is requested by the recorder. 4.6.3 accepts any of them up to 250 Hz
(interval of at least 4 ms at a 400 Hz loop).

| Message | Rate | Replaces FC log | Used for |
|---|---|---|---|
| `HIGHRES_IMU` | 200 Hz | IMU, BARO, MAG | VIO input: gyro, accel, mag direction, baro |
| `ATTITUDE_QUATERNION` | 50 Hz | ATT | FC attitude; also checks our gyro integration (aliasing) against the EKF, which integrates full-rate delta angles |
| `LOCAL_POSITION_NED` | 10 Hz | XKF1 position / velocity | FC EKF state, comparison with VIO |
| `EKF_STATUS_REPORT` | 5 Hz | XKF4 test ratios, flags | `velocity_variance`, `pos_horiz_variance` etc. are the square roots of the EKF innovation test ratios (1.0 = gate), plus the EKF status flags |
| `VIBRATION` | 5 Hz | VIBE | vibration levels and accel clipping counts at the full sensor rate (our saturation check) |
| `ESC_TELEMETRY_1_TO_4` | 10 Hz | ESC | motor RPM: notch positions, gyro delay model, events |
| `VFR_HUD` | 5 Hz | CTUN (part) | throttle, climb rate, baro altitude |
| `SERVO_OUTPUT_RAW` | 10 Hz | RCOU | motor commands |
| `RC_CHANNELS` | 5 Hz | RCIN | the pilot's mode switch and EKF source switch positions |
| `HEARTBEAT` | 2 Hz | MODE, EV | flight mode, armed state |
| `SYS_STATUS` | 2 Hz | POWR (part), sensor health | sensor present/healthy bits, FC load, battery |
| `EXTENDED_SYS_STATE` | 10 Hz | | landed state: zero-velocity updates on the ground, also with the motors running. When 4.6.3 sets and clears it: ARDUPILOT_SETUP.md section 2 |
| `POWER_STATUS` | 1 Hz | POWR | FC 5 V rail, USB connected flag |
| `SYSTEM_TIME` | 2 Hz | | FC boot time, for the passive clock sync |
| `STATUSTEXT` | always | MSG | FC messages: pre-arm failures, "Using EKF Source Set N", EKF aiding changes, failsafes |

Total on USB is about 20 kB/s, no problem for USB.

What we cannot see this way:

- IMU data above 200 Hz and before filtering (no batch sampler or FFT). The aliasing question is
  answered indirectly by comparing integrated gyro with `ATTITUDE_QUATERNION`.
- Per-axis, per-source EKF innovations and EKF internal states (biases, wind, covariance). Only the
  combined test ratios at 5 Hz.
- In Stage 3: whether each ODOMETRY sample was fused, and the posErr the FC really used (VISP/VISV
  in the FC log). We only see what we sent, our quality versus `VISO_QUAL_MIN`, and the effect on
  `EKF_STATUS_REPORT` and `LOCAL_POSITION_NED`. The bench check of the posErr formula therefore
  moves to SITL on the 4.6.3 tag (which logs), plus an indirect test on the real FC: with a known
  position offset and covariance, the expected `pos_horiz_variance` differs between the 4.6.3 and the
  4.7 formula.
- Anything that happens while the Pi is off, booting or crashed, and messages lost on the USB link.
  Lost messages are visible as gaps in the MAVLink sequence numbers.
- FC events between two 2 Hz HEARTBEATs are only known to that resolution, unless a STATUSTEXT names
  them.

## 3. Clock sync without sending anything

The FC time in `HIGHRES_IMU.time_usec` and `SYSTEM_TIME.time_boot_ms` is compared with the Pi arrival
time (CLOCK_BOOTTIME) of the same message. The offset is the minimum of `arrival - fc_time` over a
sliding window (like ArduPilot's own jitter correction), plus a linear drift fit. The error is the
minimum USB transport delay, well under 1 ms and constant, which ends up in the camera-IMU offset
anyway. `TIMESYNC` would measure it slightly better, but it is not needed and stays out of Stage 1.

## 4. Session layout

One folder per session on the Pi, by default `/var/lib/vio/recordings/<YYYYmmdd_HHMMSS>/`:

| File | Content |
|---|---|
| `meta.yaml` | config used (vio.yaml merged with local.yaml, without the origin coordinates), recorder version, git hash, Pi hostname, camera mode and controls, FC version from HEARTBEAT/STATUSTEXT |
| `frames.bin` | raw 8-bit grayscale frames, 640x480, back to back |
| `frames.csv` | per frame: index, SensorTimestamp (ns, CLOCK_BOOTTIME), arrival time, exposure (us), analogue gain, digital gain, frame duration, offset in `frames.bin` |
| `mavlink.tlog` | every MAVLink packet from the FC, each prefixed with the Pi arrival time (CLOCK_BOOTTIME, us). Same layout as a QGC/MAVProxy tlog, so pymavlink can read it; the dates just look like 1970 |
| `clock.csv` | once per second: CLOCK_BOOTTIME, CLOCK_REALTIME, chrony/RTC status. Links the session to UTC for the external GNSS log |
| `system.csv` | once per second: Pi CPU load, temperature, throttling flags, free disk |
| `gnss/` | empty at recording time. External GNSS logs are copied in after the flight |

Frames are stored raw because compression costs CPU on the Pi and changes the image. 640x480 at
30 fps is 9.2 MB/s, about 5.5 GB for a 10 minute flight. The SD card needs a sustained write speed
above that (an A2 / V30 card); the recorder checks the write rate and free space and reports in
`system.csv`. The recording folder is rotated: oldest sessions are deleted when the configured limit
is reached.

## 5. External GNSS log

The recorder does not talk to the GNSS logger. After the flight its log is copied into `gnss/`, and
`tools/align_gnss.py` aligns it in time:

1. Coarse: GNSS UTC time against the `clock.csv` pairs. This needs a correct Pi wall clock: either the
   Pi 5 RTC with its backup battery, or a time sync before the flight (phone hotspot and chrony).
   `clock.csv` records whether the clock was synced.
2. Fine: cross-correlation of the horizontal acceleration from GNSS velocity with the accelerometer
   rotated by the FC attitude, over the whole flight. This does not need VIO and gives about 20-50 ms.
3. The result (offset, quality of the correlation peak) is written next to the log and used by the
   metrics tool.

Formats planned: NMEA (`GGA` + `RMC`, most loggers and phone apps), GPX, and u-blox UBX (for a PPK
receiver later).

## 6. Replay

`vio_replay <session>` reads a session and feeds frames and MAVLink messages to the same code paths
as the live system, in time order, either as fast as possible (deterministic, for tests and metrics)
or at the original speed. It builds and runs on x86 Linux. Camera capture and everything with
latency or CPU numbers only make sense on the Pi.

EuRoC and TUM-VI are read by a small adapter that turns their folders into the same internal
messages. Datasets live outside the repo (default `~/datasets`), see DATASETS.md.

Today `vio_replay` reads the EuRoC folder layout. For our recordings a converter (not written yet)
will write the same layout from `frames.bin` and `mavlink.tlog`, including the FC landed state as
`mav0/landed0/data.csv`: `#timestamp [ns],landed_state`, one row per `EXTENDED_SYS_STATE`, the
FC send time mapped to the IMU clock (section 3), `landed_state` as in MAV_LANDED_STATE (1 on the
ground, 2 in the air, 3 taking off, 4 landing). The synthetic liftoff sessions already write it.

## 7. Recordings needed for the IMU noise values

The filter's IMU noise values (`imu.*_noise_density`, `imu.*_random_walk` in `config/vio.yaml`) must
come from our own filtered FC stream, not from the ICM-42688 datasheet. On EuRoC the datasheet
values of its IMU made the IMU-only stretch after takeoff clearly overconfident (DESIGN.md section 11,
step 3): motor vibration and the FC filters change the noise a lot. Stage 1b therefore records:

- A still stretch for the Allan variance (CALIBRATION.md step 4): drone on a firm surface, motors
  off, nobody touching it, 2-3 hours of `HIGHRES_IMU` at 200 Hz at a stable temperature. This gives
  the white noise and the bias random walk of the filtered stream.
- In-flight IMU data: hover and slow forward flight, each at least 2-3 minutes, plus the motors
  spinning on the ground at idle for a minute. Vibration and the aliasing described in
  ARDUPILOT_SETUP.md section 2 raise the noise far above the still values; these recordings show by
  how much. The flight values are the ones the filter uses, the still values are the floor.
- Both together with `VIBRATION` and `ESC_TELEMETRY_1_TO_4` (section 2), so the noise can be related
  to motor speed and vibration level.

The noise values are then set from these recordings and checked by replaying flights with ground
truth: the consistency numbers (`tools/consistency.py`) must be honest before the module may send
data to the FC.
