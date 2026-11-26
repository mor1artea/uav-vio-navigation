# Ground truth for the metrics

The drone has no GNSS, so the route metrics (ATE, RPE on 100 m, loop closure) and the hover test
need an independent reference that the VIO and the FC never see.

## What accuracy we need

| Metric | Spec threshold | Error the reference may have |
|---|---|---|
| ATE on a 2-5 km route | 1-5% of distance = 20-250 m | a few meters is fine |
| Loop closure on a 2-5 km loop | 1-3% = 20-150 m | a few meters is fine |
| RPE on 100 m segments | 1.5-5% = 1.5-5 m | must be clearly below 1.5 m for the "excellent" grade |
| Hover 1 min | 0.5-2 m/min | about 0.1 m |

So a normal GNSS receiver (1.5-3 m) is enough for ATE and loop closure, borderline for RPE, and not
enough for the hover test.

## Recommended setup

1. A u-blox GNSS receiver on the drone, connected to the Pi by USB, not to the FC. Our recorder logs
   its raw output. The VIO and the FC never use it. Time sync is free, because it is logged with the
   same Pi clock, and the receiver's own GNSS time is in the log too.
2. For trustworthy RPE and hover numbers: a receiver that outputs raw measurements (for example a
   u-blox ZED-F9P), and post-processing (PPK) with RTKLIB against a reference station. The reference
   is either a nearby permanent station from a national network (RINEX files, often free) or a second
   receiver on a tripod at the takeoff point. PPK gives 2-5 cm, works after the flight, and needs no
   radio link.
3. Measure the antenna offset from the FC IMU (like the camera offset) and put it in the metrics
   tool. Put the antenna on top, away from the Pi, the camera cable and the VTX, which all make RF
   noise.
4. Tests must be done in a place where GNSS works for the logger.

A phone app is the backup: 3-5 m accuracy and an unknown time offset. Usable for a rough check of
the route shape, not for the spec metrics.

## Until the logger is chosen

The recording does not depend on the logger. Any separate GNSS logger (or a phone app as a rough
backup) records on its own; after the flight its file is copied into the session's `gnss/` folder,
and `tools/align_gnss.py` aligns it in time: coarse by UTC against the Pi clock pairs in
`clock.csv`, fine by correlating accelerations (details in RECORDING.md section 5). For the coarse
step the Pi needs a correct wall clock on the day: the Pi 5 RTC with its backup battery, or a time
sync before takeoff.

## Hover without RTK or PPK

Place a large printed AprilTag (about 1 m) flat on the ground under the hover point and hover at
10-20 m. A separate tool computes the camera pose from the tag in every frame (single-frame PnP,
independent of the filter state). That gives the true position relative to the tag to a few
centimeters. Limits: same camera, same calibration and same time stamps as the VIO, so calibration
errors affect both. Good enough to tell 0.5 m/min from 2 m/min.

## How the metrics will compare frames

- Our frame: origin at takeoff, north from the compass. GNSS: converted to a local NED frame at the
  takeoff point.
- Two numbers for each metric:
  - "navigation": only the takeoff point is matched, no rotation fitting. This includes the
    heading error, which is what the drone really experiences.
  - "aligned": the usual best-fit alignment (as in evo). This removes the constant heading offset and
    shows the pure drift.
- The spec numbers will be checked against the navigation version, with the aligned version next to
  it for comparison. Details in the metrics tool in Stage 1.
