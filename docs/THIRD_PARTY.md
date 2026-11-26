# Third-party code

Rules for this repo:

- Borrow only the pieces we need, adapt them to our design and style. No whole frameworks, no
  dependency trees.
- Check the license of the project before copying. Keep the original copyright notice in the
  file, and add a short comment with the source project and file.
- List every borrowed piece in the table below.
- No code from GPL projects until the repo license is decided. Ask first.
- No LICENSE file for the repo yet.

## Borrowed code

Nothing has been copied yet.

| Our file | What was taken | Source project and file | License | What we changed |
|---|---|---|---|---|
| | | | | |

## Dependencies (linked, not copied)

| Library | License | Notes |
|---|---|---|
| Eigen | MPL-2.0 | Header-only |
| OpenCV | Apache-2.0 | core, imgproc, video (KLT) in the library; imgcodecs (PNG) in `vio_track`; calib3d only in the camera model test |
| libcamera | LGPL-2.1 | Linked dynamically |
| MAVLink c_library_v2 | MIT for the generated headers | The generator is LGPL-3.0, but the mavlink repo grants an explicit MIT exception for generated code |
| yaml-cpp | MIT | Config parsing |

## Simulation tools (run beside the repo, not copied into it)

| Tool | License | Notes |
|---|---|---|
| ArduPilot SITL (Copter-4.6.3) | GPL-3.0 | `sim/setup.sh` clones it into `~/sim` |
| ardupilot_gazebo | LGPL-3.0 | cloned into `~/sim`; `sim/make_world.py` makes changed copies of its Iris models in the world folder, outside the repo |
| Gazebo Harmonic | Apache-2.0 | installed with apt |
| pymavlink | LGPL-3.0 | imported by `sim/fly_record.py` |

## Candidate sources, license checked

| Project | License (from its repo) | What we may take | What the license requires |
|---|---|---|---|
| Basalt (basalt-headers, basalt) | BSD-3-Clause | Kannala-Brandt camera model, patch KLT tracking ideas | Keep the copyright notice and license text in the file, no use of their names for promotion |
| Kimera-VIO | BSD-2-Clause | Gyro-predicted feature tracking | Keep the copyright notice and license text |
| ROVIO | BSD-3-Clause | EKF and patch handling ideas | Same as Basalt |
| OKVIS / OKVIS2 | BSD-3-Clause | IMU error model ideas | Same as Basalt |
| Kalibr | BSD-3-Clause | Used as a tool only, not linked | Nothing for us |
| OpenVINS | GPL-3.0 | MSCKF update, nullspace projection, time offset, ZUPT. Ideas only for now | Copyleft: the whole repo would have to be GPL-3.0 |
| VINS-Mono / VINS-Fusion | GPL-3.0 | Ideas only for now | Copyleft |
| ORB-SLAM3, SVO Pro | GPL-3.0 | Ideas only | Copyleft |
| ArduPilot Lua applets | GPL-3.0 | Not used (no FC scripting on this hardware) | Copyleft |
| msckf_vio (Penn) | Non-commercial research license | Ideas only, never copy | Use restricted to non-profit research |
