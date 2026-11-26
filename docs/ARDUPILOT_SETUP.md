# ArduPilot setup for the VIO module

For: Stingbee Stellar H7V2, ArduCopter V4.6.3 (92b0cd78), ChibiOS 88b84600, frame QUAD/X_REV,
no GPS, external compass, baro only, Raspberry Pi 5 on the FC USB port.

Everything here is written against `Copter-4.6.3` and was checked in its source. Places where
`Copter-4.7.3` behaves differently are marked **4.7.3:**. File references are relative to the
ArduPilot repo. Nothing here has been applied to the FC; section 10 is the list for you to apply
by hand.

## 1. Board and firmware

From `libraries/AP_HAL_ChibiOS/hwdef/StellarH7V2/hwdef.dat`:

| Item | Value | What it means for us |
|---|---|---|
| MCU / flash | STM32H743, 2048 KB | VISO, EKF3 ExternalNav, `HIGHRES_IMU` and Lua are compiled in (they need more than 1 MB) |
| IMU | One ICM-42688-P on SPI1, `ROTATION_PITCH_180`, fast sampling on (`INS_FAST_SAMPLE=1`), external clock | One IMU only: `SCALED_IMU2/3` are empty, EKF3 runs one core, no IMU redundancy |
| Baro | DPS310 (or BMP280) on I2C2 | Only height sensor |
| Compass | None on the board, external compass probed on I2C | Your external compass is the yaw source |
| USB | OTG1 = `SERIAL0` | Our link |
| UART6 | Default protocol GPS | Unused |
| Storage | No SD card, `LOG_BACKEND_TYPE = 0` (from your parameter backup) | No FC logs and no Lua scripts. Everything is recorded over MAVLink on the Pi (RECORDING.md). MAVLink logging to the Pi: section 13 |

### What the firmware hash tells us

- `92b0cd78` is exactly the commit of the official tag `Copter-4.6.3` ("Copter: version to 4.6.3
  official fix", full hash `92b0cd788ec2...`). ChibiOS `88b84600` is exactly the ChibiOS submodule
  of that tag.
- But the tag does not contain a StellarH7V2 board definition, and neither does the `ArduPilot-4.6`
  branch. The board only arrived in release 4.7.0.
- The firmware build writes `git rev-parse --short=8 HEAD` as the version hash
  (`Tools/ardupilotwaf/git_submodule.py`), with no marker for local changes. So this binary was built
  from the 4.6.3 tag with the board files added locally, outside git: a vendor or custom build, not
  one from the ArduPilot firmware server.
- For our quirks this means: the EKF3, AP_VisualOdom and MAVLink code is the 4.6.3 tag code, as long
  as the builder only added the board files. That is the normal way to do it, but the hash cannot
  prove that nothing else was edited. So all 4.6.3 quirks below are expected to hold, and we confirm
  the important ones on the bench instead of trusting the hash:
  - posErr formula (Stage 3): send ODOMETRY with variances 1 / 1 / 1 m^2. The 4.6.3 formula gives
    1.44 m, the 4.7 one 1.73 m (both above the `VISO_POS_M_NSE` floor). If the FC logs, read posErr
    in VISP. Without FC logs: check the formula in SITL on the 4.6.3 tag, and on the real FC compare
    `EKF_STATUS_REPORT.pos_horiz_variance` after a known position offset with the value expected for
    each formula;
  - `AHRS_ORIGIN_LAT` must not exist and `EK3_SRC_OPTIONS` defaults to 1 (both are 4.7 changes);
  - SITL against the real 4.6.3 tag is used for everything else.
- ArduPilot is GPL-3.0, so the vendor has to give you the exact source of that binary on request.
  Asking Stingbee for it (or at least their hwdef) would remove the remaining doubt.

## 2. IMU from the FC over MAVLink

| Message | Content | Time stamp | Notes |
|---|---|---|---|
| `HIGHRES_IMU` (105) | float accel m/s^2, gyro rad/s, calibrated mag, baro | `micros64()` at send time | Our choice |
| `RAW_IMU` (27) | int16 accel mG, gyro mrad/s, mag | `micros64()` at send time | Gyro clips at 1877 deg/s, below the 2000 deg/s sensor range |
| `SCALED_IMU2/3` | 2nd/3rd IMU | `millis()` | Empty on this board |

- The values are `ins.get_accel()` / `ins.get_gyro()`: the latest filtered sample, rotated into the
  vehicle body frame. Sending at 200 Hz just picks that value. There is no extra anti-alias filter
  before the pick, so whatever the FC filters let through above 100 Hz folds into our data.
- Stamped at send time, not at sample time. The sample is taken at the start of the FC loop, the
  message goes out later in the same loop: 0-2.5 ms later at 400 Hz, about 1.3 ms on average. This
  part is the same for gyro and accel.

### Filter chain on this FC and its delay

Your settings: `INS_GYRO_FILTER = 80`, `INS_ACCEL_FILTER = 20`, harmonic notch 1 on
(`MODE = 3` ESC telemetry, `FREQ = 40`, `BW = 12`, `ATT = 20`, `REF = 1`, `HMNCS = 5`, `OPTS = 6`,
`FM_RAT = 1`), notch 2 off.

Decoded from the 4.6.3 source (`libraries/Filter/HarmonicNotchFilter.cpp`,
`libraries/AP_Vehicle/AP_Vehicle.cpp`):

- `HMNCS = 5` = bits 0 and 2: notches on the 1st and the 3rd harmonic of the motor frequency.
- `OPTS = 6` = bits 1 and 2: bit 1 "Multi-Source" (for ESC telemetry: a separate notch set for each
  motor's own RPM instead of one set on the average), bit 2 "Update at loop rate" (notch centers
  updated at 400 Hz instead of 200 Hz). No double or triple notch.
- So the gyro runs through 8 notches in series: 4 motors x (1x and 3x the motor frequency).
- `FREQ = 40` in ESC mode is the minimum: a notch never goes below 40 Hz (per harmonic), it fades
  out below that and is disabled under 10 Hz (motors stopped). `FM_RAT` only applies to throttle
  mode. In Multi-Source mode `REF` only switches tracking on. If ESC data is missing for 1 s, the FC
  falls back to throttle-based notches.
- `BW = 12` with `FREQ = 40` gives Q = 3.06, used for every notch. `ATT = 20` gives a 20 dB notch.
- The notches are on the gyro only. The accel gets only its 20 Hz low-pass.
- The ICM-42688 is fast sampled (`INS_GYRO_RATE` default 1 on H7, so 2 kHz) with the internal
  filters set by the driver: UI filter at ODR/4 and anti-alias filter about 536 Hz (gyro) / 213 Hz
  (accel) (`AP_InertialSensor_Invensensev3.cpp`).

Group delay in the band that matters for VIO (0.5-5 Hz), computed from the exact filter
coefficient formulas of the 4.6.3 code (script in the Stage 1 tools):

| Stage | Gyro | Accel |
|---|---|---|
| Sensor UI filter + anti-alias filter (2-pole approximation) | 0.9 ms | 1.5 ms |
| 8 harmonic notches, motor at 40 / 60 / 80 / 100 / 150 / 200 Hz | 6.2 / 4.1 / 3.1 / 2.4 / 1.6 / 1.1 ms | none |
| Low-pass (80 Hz gyro, 20 Hz accel), 2 kHz rate | 2.8 ms | 11.3-11.9 ms |
| Sample to send inside the FC loop (same for both) | about 1.3 ms, jitter 0-2.5 ms | same |
| Total | 4.8-9.9 ms + send, depends on motor RPM | 12.8-13.4 ms + send, constant |

The numbers barely change with a 4 or 8 kHz backend rate (under 0.1 ms).

What this means for the time offset model (used in DESIGN.md and `config/vio.yaml`):

- Gyro and accel get separate delays. Accel is behind the gyro by about 3-8 ms depending on RPM.
  Before integration, accel samples are shifted in time by `accel_delay - gyro_delay` so both line up.
- The gyro delay is not constant: it moves by up to 5 ms between low and high RPM because of the
  notches. The camera-IMU offset `td` is estimated online against the gyro, so it follows the
  average. The RPM-dependent part is left to the online estimate for now (see ESC telemetry below).
- The common send lag and USB transport do not matter here: our IMU time comes from the FC
  timestamp, mapped to the Pi clock passively from the message arrival times (minimum delay plus a
  slow drift fit, RECORDING.md section 3, no TIMESYNC sent), and the rest is in `td`.

Aliasing with `INS_GYRO_FILTER = 80`: the 80 Hz low-pass only attenuates 5.4 dB at 100 Hz, 11.5 dB at
150 Hz and 16.5 dB at 200 Hz. The motor peaks are notched, but broadband vibration between 100 and
200 Hz folds into our 200 Hz gyro stream. The accel at 20 Hz is fine (-28 dB at 100 Hz). We do not
ask you to change the flight tune. Instead, the first hover recordings compare our gyro stream with
the FC's own IMU log, and the gyro noise in the filter config is set from that. If aliasing turns out
to be bad, the fallback is to take the rotation between frames from `ATTITUDE_QUATERNION`, which the
EKF builds from un-aliased full-rate delta angles.

### ESC telemetry

What it gives us, without adding anything to the filter now:

- Indirectly, the most important thing: per-motor RPM is what makes the notches track the motors,
  which keeps our gyro data clean.
- Motor frequencies tell us where the notches are, so the gyro delay above can be computed for every
  moment. The module will log `ESC_TELEMETRY_1_TO_4` (10 Hz) in recordings. If the online `td` estimate
  shows the RPM-dependent swing in replays, we can add the delay model later from the logged RPM.
- RPM itself is not a good vibration indicator. The FC already sends a direct one: `VIBRATION`
  (vibration levels and accel clipping counts). The quality module uses that at 5 Hz, and its
  clipping counters are our IMU saturation check, because saturation happens at 2 kHz inside the FC
  and is invisible in the filtered 200 Hz data.
- A dropout of ESC telemetry makes the FC switch to throttle notches, which changes the gyro delay.
  Logging RPM makes such events visible in replays.
- The `HIGHRES_IMU` magnetometer is `compass.get_field() * 1000`, but `get_field()` is already in
  milligauss. We only use its direction.
- Maximum rate: `MAV_CMD_SET_MESSAGE_INTERVAL` is refused if `interval_ms * 800 < loop period in
  us` (`GCS_MAVLINK::cap_message_interval`). At `SCHED_LOOP_RATE = 400` the shortest accepted
  interval is 4 ms, so 250 Hz. We use 200 Hz.
- `SET_MESSAGE_INTERVAL` is not saved to parameters, so the module requests its rates at startup and
  after every reconnect: `HIGHRES_IMU` 200 Hz, `ATTITUDE_QUATERNION` 50 Hz, `LOCAL_POSITION_NED` 10 Hz,
  `ESC_TELEMETRY_1_TO_4` 10 Hz (logged only), `EKF_STATUS_REPORT` 5 Hz, `VIBRATION` 5 Hz,
  `EXTENDED_SYS_STATE` 10 Hz, `GPS_GLOBAL_ORIGIN` once.
- `TIMESYNC` is answered with the FC time in ns (`handle_timesync`).

### Landed flag (`EXTENDED_SYS_STATE.landed_state`), read from the 4.6.3 source (2026-11-26)

We use the flag for zero-velocity updates on the ground with the motors running (DESIGN.md
section 11, "Takeoff"). When it is set and when it clears, from the `Copter-4.6.3` tag:

- `landed_state` is `MAV_LANDED_STATE_ON_GROUND` exactly when `ap.land_complete` is set; otherwise
  `TAKEOFF` or `LANDING` if the mode says so, else `IN_AIR` (`GCS_Mavlink.cpp`
  `GCS_MAVLINK_Copter::landed_state`). Disarmed means always landed (`land_detector.cpp`).
- Stabilize and Acro (`mode_stabilize.cpp`, `mode_acro.cpp`): the flag clears in the first loop with
  the motors at `THROTTLE_UNLIMITED` and the throttle not at its lower limit, that is as soon as the
  throttle stick is above zero once the motors have spooled up. Before the drone leaves the ground,
  as the review read. One exception: the spool-up (`AP_MotorsMulticopter.cpp`, `SPOOLING_UP`)
  ramps the thrust limit over `MOT_SPOOL_TIME` (0.5 s by default) and ends only when the ramp meets
  the commanded throttle. If the pilot pushes the stick from zero to well above hover in one go,
  the ramp passes hover thrust before it meets the stick, and the drone can lift off up to a few
  tenths of a second before the flag clears.
- AltHold (`mode_althold.cpp`, `mode.cpp` `get_alt_hold_state`, `takeoff.cpp`
  `do_pilot_takeoff`): with a climb request the takeoff starts and the throttle ramps up over
  `TKOFF_SLEW_TIME`. The flag clears only when the throttle reaches `TKOFF_THR_MAX` (default 0.9),
  or the vertical controller (reset to the current state every loop while landed, so in effect the
  measured motion) reaches half its maximum acceleration or the pilot's climb request (clipped to
  10-50 % of `PILOT_SPEED_UP`), or half the takeoff altitude. The drone leaves the ground when the
  thrust passes its weight, which is before most of these: in AltHold the flag normally clears after
  the liftoff, by up to the rest of the throttle ramp.
- Also in every mode (`land_detector.cpp`): a landed flag with the throttle output above half the
  hover throttle and the motors unlimited is cleared as an internal error.

So the flag alone can never guarantee that no zero-velocity update happens after the liftoff: it
can be late in all three modes. The rate only sets how often an update is possible on the ground;
the hold-off (`filter.zupt.landed.max_flag_age_s`, 0.1 s) only bounds how long after the flag
clears an update can still happen. The image and the accelerometer have to stop it after the
liftoff in the sharp synthetic liftoff, also with a flag 0.5 s late; in a gentle takeoff they do
not (DESIGN.md section 11). The rate is 10 Hz (your decision, 2026-11-26): an update is possible on
nearly every update frame on the ground instead of one in three, and it costs nothing on USB. The
landed update itself is off by default until our own takeoff recordings.

## 3. What ArduPilot 4.6.3 does with our messages

| Topic | Behavior | Where |
|---|---|---|
| ODOMETRY frames | Only `frame_id = MAV_FRAME_LOCAL_FRD (20)`, `child_frame_id = MAV_FRAME_BODY_FRD (12)`, anything else is silently dropped | `GCS_Common.cpp` `handle_odometry` |
| Position | NED meters from the EKF origin, no rotation, only `VISO_SCALE` | `AP_VisualOdom_MAV.cpp` |
| posErr | `cbrt(c0^2 + c6^2 + c11^2)` from `pose_covariance` (not a standard deviation), clamped to `[VISO_POS_M_NSE, 100]`, then EKF clamps to 0.01-10 m. **4.7.3:** `sqrt(c0 + c6 + c11)`, EKF clamp 0.01-100 m | `handle_odometry`, `AP_NavEKF3_PosVelFusion.cpp` |
| Velocity | Body FRD in the message, rotated to NED with the message `q` | `handle_odometry` |
| Velocity noise | Message covariance ignored, `VISO_VEL_M_NSE` (clamped 0.05-5 m/s). **4.7.3:** same, clamp 0.05-50 | `AP_VisualOdom_MAV.cpp` |
| Vertical velocity | Fused whenever ExtNav horizontal velocity is fused, even with `EK3_SRCx_VELZ = 0` (fixed in 4.7 by PR 29213, which says it "can cause a spectacular crash"). So our vz must always be as good as vx/vy. **4.7.3:** follows `EK3_SRCx_VELZ` | `PosVelFusion.cpp` `fuseData[2]` |
| quality | Fused only if `quality >= VISO_QUAL_MIN`. `-1` failed, `0` unknown. Rejected data is still written to the FC log (VISP/VISV), if the FC logs | `AP_VisualOdom_MAV.cpp` |
| NaN | NaN position or velocity (any component) is dropped quietly | `writeExtNavData`, `writeExtNavVelData` |
| reset_counter | Any change resets the EKF horizontal position to ours | `AP_VisualOdom_Backend.cpp`, `PosVelFusion.cpp` |
| Rate | At most one sample per 20 ms (50 Hz) | `extNavIntervalMin_ms` |
| Health / pre-arm | VISO healthy if a message came in the last 300 ms, else arming fails (vision arming check) | `AP_VisualOdom_Backend.cpp`, `AP_Arming.cpp` |
| Gating | ExtNav uses `EK3_POS_I_GATE`, `EK3_VEL_I_GATE`. Long rejection or variance above `EK3_GLITCH_RAD` resets the EKF to our position | `PosVelFusion.cpp` |
| Yaw in our `q` | Used only if `EK3_SRCx_YAW = 6`. Not used by us (compass yaw) | `writeExtNavData` |

What this means for our output:

- Covariance: we send the real covariance, but the 4.6.3 mapping does not keep it honest. posErr =
  cbrt(c0^2 + c6^2 + c11^2) takes the variances, squares them and takes a cube root, so it mixes
  units and depends on all three axes. With equal sigmas it is 3^(1/3) sigma^(4/3), which is smaller
  than sigma whenever sigma is below about 0.33 m: the EKF then trusts us more than we said. Example:
  sigma_xy = 0.40 m, sigma_z = 0.05 m gives posErr 0.37 m. In the other direction our z variance
  inflates it although the EKF takes height from the baro: sigma_xy = 0.3 m, sigma_z = 0.7 m gives
  0.64 m. What we send therefore has to be chosen with this formula in mind (DESIGN.md section 13),
  and checked in SITL.
- Never send a NaN covariance with a valid position. According to the review of the 4.6.3 source,
  posErr then ends at the `VISO_POS_M_NSE` floor, the highest trust the EKF gives. Not checked in
  SITL yet.
- LOST: ODOMETRY with `quality = -1` and NaN position and velocity. Not fused, still logged.
  "Very large covariance" is not used, because the EKF clamps it to 10 m and would still fuse it.
- DEGRADED: position covariance grows, quality drops. The whole velocity is sent as NaN once its
  sigma (including vz) is more than about 3x `VISO_VEL_M_NSE`.

## 4. Timing and VISO_DELAY_MS

- `time_usec` goes through a jitter correction (`AP_RTC/JitterCorrection.cpp`). It learns the
  offset as the minimum of `arrival - time_usec`, so constant latency disappears and only the
  variable part is kept. `VISO_DELAY_MS` is the only way to tell the EKF the constant delay.
- Our `time_usec` = Pi time (CLOCK_BOOTTIME, us) when the estimate is valid. We propagate to the
  latest IMU sample before sending. Start with `VISO_DELAY_MS = 10`, then tune from our latency logs
  and the EKF ExtNav innovations.

## 5. Frames

- World: NED, origin = EKF origin = takeoff point, north from the compass.
- Body: FRD at the FC IMU. We output the IMU position, so `VISO_POS_XYZ = INS_POS1_XYZ` (usually 0).
- Camera: OpenCV convention. Nominal down-looking rotation with the image top towards the nose:
  `R_body_cam = [[0,-1,0],[1,0,0],[0,0,1]]`. The real one comes from calibration.
- ODOMETRY: `x, y, z` IMU position NED. `q` body FRD -> NED, order `w, x, y, z`. `vx, vy, vz` body FRD.
  Covariances: upper triangle of the 6x6, row major. `estimator_type = MAV_ESTIMATOR_TYPE_VIO (3)`.
  Our component id `MAV_COMP_ID_VISUAL_INERTIAL_ODOMETRY (197)`. Message 331 needs MAVLink 2.

## 6. Running with no GPS at all

### EKF origin

- Nothing sets the origin without GPS. And Copter's `ekf_check()` returns at its first line when
  there is no origin, so without one the EKF failsafe never fires. The origin is a safety item.
- 4.6.3: the module reads `GPS_GLOBAL_ORIGIN`. If none is set, it sends `SET_GPS_GLOBAL_ORIGIN` with
  the coordinates from `config/local.yaml` and retries until the FC confirms. The FC applies it only
  if no origin is set yet (`GCS_MAVLINK::set_ekf_origin`). **4.7.3:** the FC can also do it alone
  with `AHRS_ORIGIN_LAT/LON/ALT` and `AHRS_OPTIONS` bit 4.

How much the exact origin matters for us (no GPS, local frame):

| Used for | Effect of an origin rounded to the nearest city (10-50 km off) |
|---|---|
| Enabling the EKF failsafe check | None. Any valid origin works. It must not be 0, 0 |
| Compass declination (`COMPASS_AUTODEC=1`, computed once from the location, WMM table on a 10 deg grid with bilinear interpolation) | Usually a few tenths of a degree. The table's own error is of a similar size. It turns "north" for the FC and for us together (we take our initial yaw from the FC), so the VIO itself is not affected |
| Earth field strength for compass checks | None at that scale |
| Altitude | None. Heights are relative to the baro at boot; the origin altitude only shifts reported AMSL values |
| Lat/lon shown in the GCS, and anything given in lat/lon (Auto waypoints, polygon fences) | Off by the rounding distance. Do not plan Auto missions or polygon fences on a map. Relative things (home, RTL, circle fence) are fine |
| Route metrics against a GNSS logger | The heading difference from declination is removed in the metrics tool, which computes the true declination from the logger's own position |

So yes: coordinates rounded to the nearest city are enough. If you prefer, you can also set
`COMPASS_AUTODEC = 0` and enter `COMPASS_DEC` by hand.

### EKF sources from boot

- Set 1 is active at boot, so it must be the vision set (section 10).
- EKF3 starts using ExternalNav once tilt alignment is done and our data arrives
  (`readyToUseExtNav`). With compass yaw it also needs its yaw aligned from the compass before it
  reports itself healthy.
- Important default: `EK3_SRC2_YAW` and `EK3_SRC3_YAW` default to 0 (none). The vision-off sets must
  get `YAW = 1`, or switching to them leaves the EKF without a yaw source.

### Arming checks

| Check | What happens | What we do |
|---|---|---|
| GPS | No GPS checks with `GPS1_TYPE = 0` and no source set using GPS | `GPS1_TYPE = 0` |
| Compass | Normal checks | Calibrate the external compass, run compass-motor calibration |
| VisOdom | Fails if no VISO message for 300 ms | The module must run. On the ground it sends ODOMETRY with quality -1 until initialized |
| "Need Position Estimate" | Arming in Loiter / PosHold needs an EKF position | Arm in Stabilize or AltHold |

## 7. MAVLink links and telemetry

Current state: USB (`SERIAL0`) is the Pi link. `SERIAL5` is set to MAVLink2 at 57600, but no radio is
confirmed, so the design treats the drone as having no telemetry link and does not depend on one.
While nothing is connected to `SERIAL5`, the FC sends its own `SR5_*` streams into the void and
forwards nothing there, because forwarding only happens to links where a GCS has been heard.

If a radio is added later, this is how it interacts with our USB link:

- Stream rates are per link. Our `SET_MESSAGE_INTERVAL` on USB only changes what the FC sends on USB.
  The radio's rates come from `SR5_*` or from the GCS on the radio, and only affect the radio. Neither
  side can change the other's rates.
- Forwarding is the problem. ODOMETRY, NAMED_VALUE_FLOAT and STATUSTEXT have no target, so ArduPilot
  treats them as broadcast and forwards them to every link where it has seen a GCS
  (`MAVLink_routing.cpp` `check_and_forward`). ODOMETRY at 30 Hz is about 7 kB/s, more than a 57600 baud
  radio carries (about 5.7 kB/s), so the radio link would be flooded. Broadcasts from the GCS would also
  reach the Pi.
- Fix: set bit 10 ("Don't forward mavlink to/from", value 1024) in `SERIAL0_OPTIONS`. In 4.6.3 this makes
  the USB channel private (`GCS_MAVLINK::set_channel_private`): nothing from the Pi is forwarded, and the
  Pi only gets messages addressed to it. The FC's own messages still reach the GCS. Our STATUSTEXT then
  stays on the Pi side and in our own recording.
- FC load: all links share the FC's MAVLink send time. A radio with normal rates (a few messages at
  2-10 Hz) is no problem, but very high radio stream rates could delay our 200 Hz IMU stream. The module
  watches the IMU rate and gaps anyway, so this would show up as a quality drop.

## 8. Pilot procedure

The pilot is the protection against a vision loss. There is no FC script (no SD card), the FC itself
reacts only after about 8 s (section 9), and the pilot cannot see the module's state in the goggles:
ArduPilot only writes a companion's STATUSTEXT to its log (`GCS_MAVLINK::handle_statustext`), it
never shows it on the OSD. The OSD does show the FC's own messages.

### Current mode switch (parameter backup 2026-11-17)

`FLTMODE_CH = 7`, `FLTMODE1..6 = 1, 0, 2, 0, 1, 2`. Checked against the 4.6.3 source (mode numbers in
`ArduCopter/mode.h`, PWM bands in the `FLTMODE` parameter docs):

| Slot | PWM on channel 7 | Value | Mode |
|---|---|---|---|
| 1 | up to 1230 | 1 | Acro |
| 2 | 1231-1360 | 0 | Stabilize |
| 3 | 1361-1490 | 2 | AltHold |
| 4 | 1491-1620 | 0 | Stabilize |
| 5 | 1621-1749 | 1 | Acro |
| 6 | 1750 and up | 2 | AltHold |

Your reading is right. With a 3-position switch at about 1000 / 1500 / 2000 us the positions give
Acro / Stabilize / AltHold. There is no Loiter and no other mode that uses a horizontal position.

### Three kinds of flights

1. Recording only: the module records, the FC does not fuse vision (`VISO_TYPE = 0` as now, or a
   source set without ExternalNav). Vision has no influence at all. Stage 1b and Stage 2 flights are
   this, with the current parameters unchanged.
2. Vision fused, flown in Acro / Stabilize / AltHold: vision cannot steer the drone horizontally, but
   it is not completely without effect. It feeds the EKF tilt estimate, and in 4.6.3 our vertical
   velocity is always fused (section 3), so it also feeds the climb rate AltHold uses, and it can
   trigger the vibration failsafe's altitude compensation (section 9). A bad vision velocity can make
   AltHold lean slightly or hold height worse. The source switch "vision off" removes that.
3. Vision controls the drone: needs Loiter (or PosHold) on the mode switch. Only when you decide to
   add it, after the simulation stage.

### Switches needed for flights of kind 2 and 3

1. Flight mode switch: AltHold (and Stabilize / Acro) are the takeover modes, they do not depend on
   vision. Loiter only when added.
2. EKF source switch (`RCx_OPTION = 90`, not set up yet): low = set 1 (vision on), middle / high =
   set 2 / 3 (vision off). Every switch change shows "Using EKF Source Set N" on the OSD. From the 4.6.3
   source (`RC_Channel::init_aux_function`): the switch is not read at boot, only when it moves. After
   power-up set 1 is always active, whatever the switch position.

Before the flight (kind 2 and 3):

1. Pi booted and the module running. Arming fails with "PreArm: VisOdom: not healthy" otherwise.
2. Source switch to low. If it was left in middle or high at power-up, move it to middle (OSD: "Using
   EKF Source Set 2") and back to low ("... Set 1").
3. Arm in Stabilize or AltHold, never in Loiter. Take off in AltHold and climb above 3-5 m.

In flight (kind 3, once Loiter exists):

- Loiter only for planned tests: low, close, over textured ground, hand on the mode switch.
- Signs of trouble: the drone drifts or speeds up without stick input; OSD messages such as "EKF3 IMU0
  stopped aiding", "EKF variance", "EKF Failsafe".
- Reaction: mode switch to AltHold at once. If it still leans or drifts, source switch to vision off.
  Fly back by hand. No return to Loiter in the same flight during the first test series.

The module never sends source set or mode commands, so it cannot fight the pilot.

## 9. Failsafes

### Your current values, decoded from the 4.6.3 source

| Parameter | Value | Meaning in 4.6.3 | What it does on this drone now |
|---|---|---|---|
| `FS_THR_ENABLE` | 3 | RC loss: always Land | On the ground: disarm. In the air: Land. Without a position the landing has no position hold, the drone descends and drifts with the wind, and the pilot has no RC to correct it |
| `FS_OPTIONS` | 16 | bit 4: continue in pilot-controlled modes on GCS failsafe | Only concerns a GCS link failsafe. There is no GCS link, so no effect. Bit 3 (continue an ongoing landing on any failsafe) is not set |
| `FS_EKF_ACTION` | 1 | EKF failsafe: Land, only in modes that need a position | No effect now: none of Acro / Stabilize / AltHold needs a position, and the EKF check does not even run without an EKF origin (section 6) |
| `FS_EKF_THRESH` | 0.8 | default variance threshold | |
| `FS_VIBE_ENABLE` | 1 | vibration failsafe | When the EKF velocity test ratio is above 1 and the vertical innovations are positive, altitude control switches to "Vibration compensation" (OSD message). Only in modes with automatic throttle (AltHold), no mode change. With vision fused in 4.6.3, a bad vision vz can also trigger it |
| `FS_CRASH_CHECK` | 1 | crash check | Disarms after 2 s of: lean over 15 deg, attitude error over 30 deg, low acceleration, speed under 10 m/s. Not active in Acro (and Flip): `ModeAcro::crash_check_enabled()` returns false |
| `FS_DR_ENABLE` | 2 | dead-reckoning failsafe: RTL | Only when the EKF navigates on wind estimates alone (drag or airspeed fusion) and only in position modes. Neither applies, so no effect |
| `BATT_FS_LOW_ACT`, `BATT_FS_CRT_ACT` | 0, 0 | no action | Below `BATT_LOW_VOLT = 18.6` / `BATT_CRT_VOLT = 18.0` the FC only warns (OSD message). The pilot has to land |

In short, today:

- RC loss: the drone lands where it is, drifting with the wind.
- EKF failure: no automatic action. The EKF failsafe needs a position mode and an origin, and there is
  neither. The pilot flies on in Acro / Stabilize / AltHold.
- Low battery: warnings only.

### Vision loss once vision is fused (4.6.3 code, `AP_NavEKF3_Control.cpp`, `ekf_check.cpp`)

| Time after our data stops being fused | What happens |
|---|---|
| 0 s | EKF has only IMU + baro + compass and dead-reckons horizontally |
| until 7 s | EKF still reports a valid position (`posRetryTimeNoVel_ms = 7000`). In Loiter the drone would hold a position that is drifting by meters |
| ~7 s | `posTimeout`: no position any more |
| ~8 s | Copter has no position for 1 s (10 checks at 10 Hz) -> EKF failsafe -> `FS_EKF_ACTION`, only in modes that need position, and only if an EKF origin is set |
| 15 s | EKF goes to constant position mode |

A crashed module, a Pi brownout or a pulled USB cable look exactly the same to the FC. When vision
comes back, EKF3 resets its position to ours and aids again. With `FS_EKF_ACTION = 1` a vision loss in
Loiter ends in a landing without position hold; with 2 (AltHold) the pilot keeps control, except
when the RC link is lost at the same time (then it lands). Proposal: change it to 2 when Loiter is
added.

### RC loss with vision fused, for later

With vision fused, an EKF origin and a home, RTL becomes possible (it needs a position, not GPS) and
then depends completely on vision. With `FS_THR_ENABLE = 3` the drone lands instead, which is the
predictable choice until VIO has proven itself. Keep 3.

## 10. Parameters before and after

"Before" comes from your parameter backup `stellar_h7v2_before_vio` (2026-11-17) and the values you
read earlier. "?" means not read yet; the column will be completed from the backup file once I have
its path (the file is not committed). Defaults are the 4.6.3 defaults from the source.

When: "now" = nothing changes on the FC. Stage 1b and Stage 2 are recording only and need no
parameter change at all (the recorder only sends `SET_MESSAGE_INTERVAL`, which is not saved).
"Stage 2b" = these values are first used in SITL. "Stage 3" = applied by you on the real FC.

| Parameter | Before | After | When | Reason |
|---|---|---|---|---|
| `AHRS_EKF_TYPE` | 3 | 3 | | EKF3 |
| `EK3_ENABLE` | 1 | 1 | | |
| `GPS1_TYPE` | ? (default 1, auto) | 0 | Stage 3 | No GPS on the drone, removes GPS checks |
| `EK3_SRC1_POSXY` | 3 (GPS) | 6 (ExternalNav) | Stage 3 | Vision for horizontal position |
| `EK3_SRC1_VELXY` | 3 (GPS) | 6 | Stage 3 | Vision for horizontal velocity |
| `EK3_SRC1_POSZ` | 1 (Baro) | 1 | | |
| `EK3_SRC1_VELZ` | 3 (GPS) | 6 | Stage 3 | 4.6.3 fuses our vz anyway (section 3) |
| `EK3_SRC1_YAW` | 1 (Compass) | 1 | | |
| `EK3_SRC2_POSXY`, `VELXY` | no horizontal source (0, 0) | 0, 0 | | Vision off |
| `EK3_SRC2_POSZ`, `VELZ` | ? (defaults 1, 0) | 1, 0 | | |
| `EK3_SRC2_YAW` | ? (default 0) | 1 | Stage 3 | Without it, "vision off" also means no yaw source |
| `EK3_SRC3_*` | no horizontal source, rest ? | same as set 2 | Stage 3 | High switch position |
| `EK3_SRC_OPTIONS` | ? (default 1) | 0 | Stage 3 | Do not fuse all velocity sources at once |
| `VISO_TYPE` | 0 | 1 | Stage 3 | MAVLink visual odometry. The other `VISO_*` parameters only appear after setting it and rebooting |
| `VISO_POS_X/Y/Z` | appear with `VISO_TYPE` (default 0) | = `INS_POS1_X/Y/Z` (normally 0) | Stage 3 | We output the IMU point |
| `VISO_ORIENT`, `VISO_SCALE` | default 0, 1 | 0, 1 | | We rotate ourselves |
| `VISO_DELAY_MS` | default 10 | 10, then tuned in SITL and on the bench | Stage 3 | Section 4 |
| `VISO_VEL_M_NSE` | default 0.1 | 0.3 | Stage 3 | The only velocity noise ArduPilot uses |
| `VISO_POS_M_NSE` | default 0.2 | 0.3 | Stage 3 | Floor for posErr |
| `VISO_YAW_M_NSE` | default 0.2 | 0.2 | | Unused with compass yaw |
| `VISO_QUAL_MIN` | default 0 | 10 | Stage 3 | FC also rejects LOST (-1) and very low quality data |
| `RCx_OPTION` (free 3-position switch) | ? | 90 | Stage 3 | EKF source switch, vision on / off |
| `FLTMODE_CH`, `FLTMODE1..6` | 7; 1, 0, 2, 0, 1, 2 (Acro, Stabilize, AltHold) | no change now. Loiter in one slot when you decide | your decision | Section 8 |
| `FS_THR_ENABLE` | 3 (Land) | 3 | | Predictable without a position |
| `FS_EKF_ACTION` | 1 (Land) | 1 now, 2 (AltHold) when Loiter is added | with Loiter | Pilot keeps control after a vision loss |
| `FS_EKF_THRESH` | 0.8 | 0.8 | | |
| `FS_VIBE_ENABLE`, `FS_CRASH_CHECK`, `FS_OPTIONS`, `FS_DR_ENABLE` | 1, 1, 16, 2 | no change | | Section 9 |
| `BATT_FS_LOW_ACT`, `BATT_FS_CRT_ACT` | 0, 0 | your decision | | Today warnings only. 1 = Land (without position: a drifting landing) |
| `BATT_LOW_VOLT`, `BATT_CRT_VOLT` | 18.6, 18.0 | no change | | |
| `LOG_BACKEND_TYPE` | 0 | 0, or 2 if you choose MAVLink logging | your decision | Section 13 |
| `SERIAL0_PROTOCOL` | ? (default 2) | 2 | check | MAVLink 2 on USB |
| `SERIAL0_OPTIONS` | ? (default 0) | 0 | check | 1024 the day a telemetry radio is added (section 7) |
| `SERIAL5_PROTOCOL`, `SERIAL5_BAUD` | 2, 57 | no change | | No radio |
| `SR0_*` | ? (default 0) | 0 | check | The module sets its own rates; a GCS on USB saves its stream requests into `SR0_*` in 4.6.3 |
| `SCHED_LOOP_RATE` | ? (default 400) | 400 | check | 200 Hz IMU stream |
| `INS_GYRO_FILTER`, `INS_ACCEL_FILTER` | 80, 20 | no change | | Never after calibration, they set the IMU delays |
| `INS_HNTCH_*` | 1, 3, 40, 12, 20, 1, 5, 6, 1 | no change | | Section 2 |
| `INS_HNTC2_ENABLE` | 0 | 0 | | |
| `INS_GYRO_RATE`, `INS_FAST_SAMPLE` | ? (H7 defaults 1, 1) | no change | | Our delay numbers change by less than 0.1 ms |
| `AHRS_ORIENTATION` | ? | no change, never after calibration | | Body frame |
| `ANGLE_MAX` | ? (default 3000) | 4500 only if not set already | check | Your 45 deg limit |
| `COMPASS_AUTODEC` | ? (default 1) | 1 | check | Declination from the origin, section 6 |
| `COMPASS_MOT_*` | ? | compass-motor calibration done | before Stage 3 | Our heading starts from the compass |
| `ARMING_CHECK` | ? (default 1) | 1 | check | |
| `EK3_POS_I_GATE`, `EK3_VEL_I_GATE`, `EK3_GLITCH_RAD` | ? (defaults 500, 500, 25) | no change | | |

## 11. Lua scripting

Not usable on this hardware: the FC has no SD card, so there is nowhere to put scripts. The earlier
idea of a Lua guard that switches to AltHold within a second of a vision loss is dropped. The
protection is the pilot procedure in section 8.

## 12. What changes in 4.7.3 for us

From `ArduCopter/ReleaseNotes.txt` at `Copter-4.7.3` (4.7.0 to 4.7.3) and the code diff:

| Area | Change | Matters for us? |
|---|---|---|
| Board | Stellar H7V2 support added officially (PR 28926) | Yes: an official build instead of a custom or vendor one |
| ExtNav vertical velocity | ExtNav vz no longer fused when `EK3_SRCx_VELZ = 0` (PR 29213) | Yes: in 4.6.3 our vz is always fused |
| ExtNav noise | Correct ExtNav variances used, limits raised to the GPS limits: position 100 m, velocity 50 m/s (PR 29420). ODOMETRY posErr becomes `sqrt(c0 + c6 + c11)` | Yes: in 4.6.3 the posErr formula is odd and capped at 10 m |
| EKF origin | `AHRS_ORIGIN_LAT/LON/ALT` + `AHRS_OPTIONS` bits 3/4 (PR 31177), origin setting delayed until the filter is ready (PR 32505), protection against uninitialized origin (PR 29636) | Yes: the FC can set its own origin without the companion |
| EKF stability | Delta velocity bias covariance fix for on-ground instability (PR 32069), covariance corruption fix (PR 32177), position controller EKF reset handling (PR 31357) | Yes: general robustness, and smoother reaction when the EKF resets to our position after a loss |
| EK3_SRC_OPTIONS | Default changed to 0 (PR 30584) | Minor, we set it anyway |
| EKF failsafe | New "Report only" action, pilot notified in all cases (PR 30189) | Minor |
| EKF cores | Cores can use different source sets (PR 29750), manual lane switching (PR 29623) | No, one IMU, one core |
| VISO | MAVLink backend can align to GPS (PR 30650) | No, no GPS |
| MAVLink | Stream rate requests no longer saved to parameters (PR 28894); serial port blocking write bug fixed (PR 33437) | Yes: fewer surprises after bench sessions, and a link robustness fix |
| IMU | Direct DMA for Invensense v3 IMUs incl. ICM-42688 (PR 28201), up to 3 harmonic notches (PR 30246), fast rate loop option (PR 27029), more detailed gyro logging (4.7.3) | Some: lower FC load and better vibration filtering. Nothing changes in `HIGHRES_IMU` itself |
| Lua | New bindings such as `vehicle:is_crashed()`, custom modes, duplicate scripting parameter names checked (PR 30523) | No, scripting is not usable without an SD card |

Nothing found that changes `HIGHRES_IMU`, `SET_MESSAGE_INTERVAL` limits, the jitter correction or the
ODOMETRY frame rules.

## 13. MAVLink logging to the Pi (`LOG_BACKEND_TYPE = 2`), assessment only

What it is (4.6.3, `libraries/AP_Logger/AP_Logger_MAVLink.cpp`): the FC writes its normal binary log,
but instead of a file it sends it over MAVLink in `REMOTE_LOG_DATA_BLOCK` messages of 200 bytes. The
backend is compiled in whenever logging and MAVLink are (true for this board). `LOG_BACKEND_TYPE` is a
bitmask: 0 none, 1 file, 2 MAVLink, 4 block.

How it works:

- The receiver on the Pi starts it by sending `REMOTE_LOG_BLOCK_STATUS` with the special sequence
  number START. The FC then sends the log to that link only.
- The receiver must ACK every block (or NACK missing ones). The FC keeps unacknowledged blocks in a
  buffer of `LOG_MAV_BUFSIZE` kB (default 8) and resends them. If the buffer is full, log data is lost.
- If the receiver does not answer for 10 s, the FC stops sending.
- `LOG_MAV_RATEMAX` can limit the logging rate of the streaming messages.

Feasible in 4.6.3: yes. What it costs:

| Cost | Size |
|---|---|
| USB bandwidth | a typical Copter log is tens of kB/s, more with fast IMU logging. USB carries about 1 MB/s, our own traffic is about 20 kB/s. No problem |
| FC send time | the log blocks share the FC's MAVLink send time with our 200 Hz IMU stream. Risk of IMU jitter or gaps if the log rate is high; to be measured, and `LOG_BITMASK` / `LOG_MAV_RATEMAX` can be reduced |
| FC memory | 8 kB by default, can be raised (the H743 has enough RAM) |
| Pi software | a small receiver in the recorder (start, ACK, NACK, write `.bin`), about 100-200 lines. mavlink-router is said to support this for ArduPilot; to be checked |
| Passive rule | the receiver has to send START and ACKs to the FC. That breaks the "only SET_MESSAGE_INTERVAL" rule of Stage 1 and needs your OK |
| Coverage | logs only exist while the Pi runs and answers. If the Pi crashes or the USB cable comes off, the FC log stops exactly when it would be most useful |

What it gives: the FC-internal data we otherwise cannot see (RECORDING.md section 2): VISP / VISV
(was each vision sample fused, which posErr), per-axis EKF innovations, full IMU logs, events, mode
changes with exact times. That is most valuable in Stage 3 (bench and first vision-fused flights);
in Stage 1b it would mainly give a reference IMU log for the aliasing question.

Suggestion: not now. Decide before Stage 3; if yes, add the receiver to the recorder then.
