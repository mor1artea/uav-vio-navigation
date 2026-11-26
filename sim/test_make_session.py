#!/usr/bin/env python3
"""Test of sim/make_session.py without a simulator: a synthetic flight is written the way Gazebo and
SITL would deliver it (ENU and FLU truth, FC clock with an offset), converted back, and compared.

  sim/test_make_session.py            # prints ok or fails
"""

import os
import shutil
import subprocess
import sys
import tempfile

import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, HERE)
sys.path.insert(0, os.path.join(HERE, "..", "tools"))
from make_session import ENU_TO_NED, FRD_TO_FLU, convert, quat_to_rot, rot_to_quat  # noqa: E402
from metrics import load_trajectory  # noqa: E402

OFFSET_NS = 12_345_678_901  # sim time minus FC time


def main():
    tmp = tempfile.mkdtemp()
    try:
        src, out = os.path.join(tmp, "src"), os.path.join(tmp, "out")
        # Strong turns: the clocks are matched on the rotation rates.
        subprocess.run([sys.executable, os.path.join(HERE, "..", "tools", "gen_synthetic.py"), src,
                        "--duration", "40", "--speed", "12", "--radius", "50", "--height", "25"],
                       check=True, capture_output=True)
        t, p, q = load_trajectory(src)
        t = t + 100_000_000_000  # sim time
        R = quat_to_rot(q)  # NED world, FRD body
        R_enu_flu = ENU_TO_NED.T @ R @ FRD_TO_FLU.T
        p_enu = p @ ENU_TO_NED
        raw = os.path.join(out, "raw")
        os.makedirs(raw)
        with open(os.path.join(raw, "truth.csv"), "w") as f:
            for ti, pi, qi in zip(t, p_enu, rot_to_quat(R_enu_flu)):
                f.write(f"{ti}," + ",".join(f"{x:.9f}" for x in np.r_[pi, qi]) + "\n")
        imu = np.loadtxt(os.path.join(src, "mav0", "imu0", "data.csv"), delimiter=",", comments="#")
        t_fc = (t - OFFSET_NS) // 1000
        rng = np.random.default_rng(1)
        with open(os.path.join(raw, "imu.csv"), "w") as f, open(os.path.join(raw, "clock.csv"), "w") as c:
            for ti, tf, r in zip(t, t_fc, imu):
                f.write(f"{tf}," + ",".join(f"{x:.9f}" for x in r[1:]) + "\n")
                c.write(f"{tf},{ti - int(rng.uniform(0, 50e6))}\n")  # truth arrives late
        for name, value in (("baro", 100.0), ("landed", 2)):
            with open(os.path.join(raw, name + ".csv"), "w") as f:
                f.writelines(f"{tf},{value}\n" for tf in t_fc[::20])
        with open(os.path.join(raw, "images.csv"), "w") as f:
            f.writelines(f"{ti},{ti}.png\n" for ti in t[::7])
        shutil.copy(os.path.join(src, "mav0", "cam0", "sensor.yaml"), os.path.join(raw, "camera.yaml"))

        ok = convert(out)
        data = np.loadtxt(os.path.join(out, "mav0", "imu0", "data.csv"), delimiter=",",
                          comments="#", usecols=0, dtype=np.int64)
        clock_error_ms = abs(data[0] - t[0]) * 1e-6
        t2, p2, q2 = load_trajectory(out)
        pos_error = np.abs(p2 - p).max()
        rot_error = np.abs(quat_to_rot(q2) - R).max()
        print(f"clock error {clock_error_ms:.3f} ms, position {pos_error:.2e} m, rotation {rot_error:.2e}")

        # A wrong frame must be caught: flip the sign of one gyro axis.
        imu_file = os.path.join(raw, "imu.csv")
        rows = [l.split(",") for l in open(imu_file).read().splitlines()]
        for r in rows:
            r[2] = str(-float(r[2]))
            r[3] = str(-float(r[3]))
        open(imu_file, "w").write("\n".join(",".join(r) for r in rows) + "\n")
        caught = not convert(out)

        passed = ok and clock_error_ms < 5 and pos_error < 1e-6 and rot_error < 1e-6 and caught
        print("ok" if passed else "FAILED")
        sys.exit(0 if passed else 1)
    finally:
        shutil.rmtree(tmp)


if __name__ == "__main__":
    main()
