# Calibration

What has to be calibrated before the module may send data to the FC, and how. The results go into
`config/vio.yaml`. Redo steps 1-3 after any change to the camera mount, the lens or the FC mounting.

Tools: Kalibr (BSD-3) in its Docker image, on a laptop. Kalibr uses ROS internally, but only inside
the container on the laptop. Nothing of it runs on the drone. OpenCV `cv::fisheye::calibrate` is a
fallback for step 2.

## 0. Mount first

- Mount the camera rigidly to the same frame part as the FC, with no damper of its own. A soft
  camera mount makes the camera-IMU transform change in flight, and the filter assumes it is fixed.
  Short exposure deals with vibration blur instead.
- If the FC sits on soft grommets, that is normal and acceptable (small relative motion), but do
  not add more softness between the two.
- Lock the lens focus (glue) after focusing at about 20 m. A lens that turns invalidates the
  intrinsics.

## 1. Measure the camera offset (lever arm)

1. Body frame = FC IMU, FRD: x towards the nose, y to the right, z down. The ICM-42688 sits near the
   center of the Stellar H7V2 board; check the board photo and use the chip position, not the board
   corner.
2. Camera point = the center of the lens front glass (close enough to the optical center for a
   small lens).
3. Measure the three distances from the IMU chip to the lens center along the body axes with a ruler
   or caliper, in cm, to about 5 mm. Example: camera 3 cm behind the FC and 4 cm below it ->
   `t_body_cam: [-0.03, 0.00, 0.04]`.
4. Write it to `extrinsics.t_body_cam`. Kalibr will also estimate it in step 3, but for offsets of a
   few cm its estimate is noisy. If Kalibr differs from the ruler by more than 1-2 cm, trust the ruler.

## 2. Camera intrinsics and the choice of the camera model

1. Print an Aprilgrid (6x6 tags, A3 or larger) and glue it flat on a rigid board. Measure the real
   tag size after printing.
2. Record with our recorder in exactly the flight camera settings: same sensor mode, bit depth,
   output size and exposure limit. Intrinsics depend on the mode and the scaling.
3. Move the board slowly in front of the camera for 60-90 s. Cover the image corners and edges,
   which matter most for a 120 deg lens. Tilt the board in all directions. Good light, so the
   exposure stays short.
4. Convert the recording to a Kalibr bag (`kalibr_bagcreater` reads a folder of images; our replay
   tool will export that layout) and run `kalibr_calibrate_cameras` twice on the same data: with
   `pinhole-radtan` (radial-tangential) and with `pinhole-equi` (Kannala-Brandt).
5. Choose the model by the result:
   - compare the reprojection error overall and per image region (center, edges, corners);
   - radial-tangential if it fits as well as Kannala-Brandt everywhere, including the corners (moderate
     distortion). It is the simpler, more common model;
   - Kannala-Brandt if radial-tangential leaves clearly larger errors at the edges and corners (strong
     distortion, which is likely at about 120 deg).
6. Accept if the reprojection error of the chosen model is under about 0.5 px and not much larger in
   the corners. Fill `camera.model`, `camera.intrinsics`, `camera.distortion`, set
   `camera.calibrated: true`.
7. The same calibration also gives the rectified pinhole crop of about 80-90 deg that DESIGN.md
   section 6 compares against the main pipeline. No extra calibration is needed for it.

## 3. Camera-IMU extrinsics and time offset

1. Put the drone on the bench, propellers off, FC powered, the Pi running the recorder. The recorder
   stores frames and `HIGHRES_IMU` at 200 Hz with synchronized time stamps.
2. Keep the Aprilgrid static. Hold the drone and move it in front of the grid for 60-90 s: rotate
   about all three axes, and also translate along all three. Smooth but lively motion, keep the grid
   mostly in view.
3. Export the recording to a Kalibr bag (images + `imu0.csv`) and run
   `kalibr_calibrate_imu_camera` with the intrinsics from step 2 and the IMU noise from step 4.
   Kalibr assumes gyro and accel have the same timing, but on this FC the accel is about 9 ms behind
   the gyro when the motors are off (20 Hz vs 80 Hz low-pass, no notches). The export tool shifts the
   accel by `accel_delay_ms - gyro_delay_no_notch_ms` first, so Kalibr's time shift refers to the gyro.
4. Kalibr gives `T_cam_imu` (the inverse of what we need) and `timeshift_cam_imu`. Invert to get
   `R_body_cam` and `t_body_cam`. For `imu.td_initial_ms`, take Kalibr's time shift and add the notch
   delay that exists only in flight (`gyro_delay_ms - gyro_delay_no_notch_ms`): on the bench the
   motors are stopped, so the notches are off and the gyro is about 1-6 ms faster than in flight. The
   online estimate takes it from there.
5. Without Kalibr, or as a check of it: the start value of `td` from a recording, offline. The
   rotation between consecutive frames is computed from the feature tracks alone (homography or
   essential matrix on the undistorted bearings), turned into a rotation rate, and cross-correlated
   with the gyro rate magnitude over a recording with lively rotations. The shift with the highest
   correlation is `td`, to about 1-2 ms. It goes to `imu.td_initial_ms`; `imu.td_prior_sigma_ms`
   (30 ms by default, because the camera, USB and FC time stamping may add tens of ms) can then be
   lowered to about 5 ms. The filter refines `td` online.
6. Sanity checks:
   - the rotation should be the nominal down-looking rotation plus only a few degrees;
   - the time shift should be in the range of the FC gyro delay without notches (about 4-6 ms) plus
     the camera timing, so roughly 5-30 ms;
   - gyro and accel residuals in the Kalibr report should look like white noise.
7. Set `extrinsics.calibrated: true`.

The IMU data is already in the FC body frame (board rotation and `AHRS_ORIENTATION` applied by the
FC), so the result is directly camera -> body. Do not change `AHRS_ORIENTATION`, `INS_GYRO_FILTER`,
`INS_ACCEL_FILTER` or the `INS_HNTCH_*` notch settings after calibration: they change the frame or the
IMU delays.

## 4. IMU noise

1. Record `HIGHRES_IMU` at 200 Hz with the drone completely still for 2-3 hours, at a stable
   temperature.
2. Compute the Allan deviation (a small script in `tools/` will do it) and read the white noise
   density and the random walk for gyro and accel.
3. The FC already low-pass filters the signal at 20 Hz, so the white noise from Allan comes out too
   small. Multiply all four values by 3-5 for the filter config. Tune further on recorded flights.

## 5. Rolling shutter line time

- From the Pi kernel driver: line length 3448 pixel clocks at 182.4 MHz = 18.9 us per sensor row in
  the 10-bit binned mode, and half of that (9.45 us) in the 8-bit special binning mode. After
  downscaling 1232 rows to 480, one output row = 9.45 * 1232 / 480 = 24.3 us (8-bit) or 48.5 us
  (10-bit).
- Check: Kalibr can estimate the line delay (`kalibr_calibrate_rs_cameras`). A simpler check is a
  recording of fast rotations over a textured surface: with the right line time, the reprojection
  error of tracked features drops clearly when the RS correction is on.
- The module logs the frame duration and exposure from the camera metadata. If libcamera ever
  changes the line length (horizontal blanking), the line time changes too, so the module compares
  the measured readout with the config and lowers the quality if they disagree.

## 6. Exposure

- Fixed exposure cap `camera.exposure_max_us` (2000 us by default), gain free up to
  `analogue_gain_max`. Exposure and gain of every frame come from the libcamera metadata and are
  logged and used in the quality score.
- If daylight images are too dark at 2 ms, raise the gain first, then the exposure, and check blur
  in fast maneuvers on a recording.
