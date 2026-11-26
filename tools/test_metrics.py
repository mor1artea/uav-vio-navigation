#!/usr/bin/env python3
"""Key tests of tools/metrics.py on trajectories with known errors."""

import math
import os
import sys
import unittest

import numpy as np

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
import metrics  # noqa: E402


def circle(n=2000, radius=100.0, laps=1.0):
    """Closed circle at constant height, heading along the path. Returns (t, p, q)."""
    a = np.linspace(0, 2 * math.pi * laps, n)
    p = np.stack([radius * np.cos(a), radius * np.sin(a), np.full(n, -30.0)], axis=1)
    yaw = a + math.pi / 2
    q = np.stack([np.cos(yaw / 2), np.zeros(n), np.zeros(n), np.sin(yaw / 2)], axis=1)
    t = (np.arange(n) * 50_000_000).astype(np.int64)  # 20 Hz
    return t, p, q


class MetricsTest(unittest.TestCase):
    def test_identical_trajectories_give_zero(self):
        gt = circle()
        r = metrics.evaluate(gt, gt)
        self.assertAlmostEqual(r["ate_rmse_m"], 0.0, places=9)
        self.assertAlmostEqual(r["rpe_mean_pct"], 0.0, places=6)
        self.assertAlmostEqual(r["loop_closure_pct"], 0.0, places=9)

    def test_scale_error_shows_in_rpe_not_in_loop_closure(self):
        t, p, q = circle()
        scaled = (t, p[0] + 1.01 * (p - p[0]), q)  # 1 % too long everywhere
        r = metrics.evaluate(scaled, (t, p, q))
        # The error is 1 % of the straight distance between the segment ends. A 100 m arc on a
        # 100 m radius circle has a chord of 2 * 100 * sin(0.5) m, so 0.959 % of the arc length.
        self.assertAlmostEqual(r["rpe_mean_pct"], 2 * math.sin(0.5), delta=0.005)
        self.assertAlmostEqual(r["loop_closure_pct"], 0.0, places=6)  # a scaled loop still closes

    def test_drift_in_loop_closure_and_heading_in_navigation_ate(self):
        t, p, q = circle()
        dist = np.concatenate([[0], np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))])
        drifted = p.copy()
        drifted[:, 0] += 0.01 * dist  # 1 % of the distance travelled, along x
        r = metrics.evaluate((t, drifted, q), (t, p, q))
        self.assertAlmostEqual(r["loop_closure_pct"], 1.0, delta=1e-6)

        # A 2 deg heading error about the start point: large navigation ATE, near zero aligned ATE.
        c, s = math.cos(math.radians(2)), math.sin(math.radians(2))
        rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        rotated = p[0] + (p - p[0]) @ rz.T
        r = metrics.evaluate((t, rotated, q), (t, p, q))
        expected_max = 2 * 100.0 * 2 * math.sin(math.radians(1))  # far side of the circle, 200 m away
        self.assertAlmostEqual(r["ate_max_m"], expected_max, delta=0.05)
        self.assertLess(r["ate_aligned_rmse_m"], 1e-6)

    def test_scale_tool_sees_a_scaled_closed_loop(self):
        # tools/scale.py: a circle 3 % too large in a rotated, shifted frame closes like the true one,
        # but every 100 m piece is 3 % too long. Speed ratio from the velocity columns.
        import tempfile
        import scale
        t, p, _ = circle(n=4000, radius=300.0)
        dt = 0.05
        v = np.gradient(p, dt, axis=0)
        c, s = math.cos(0.7), math.sin(0.7)
        rz = np.array([[c, -s, 0], [s, c, 0], [0, 0, 1]])
        p_est = (1.03 * (p - p[0])) @ rz.T + np.array([50.0, -20.0, 0.0])
        v_est = 1.03 * v @ rz.T
        with tempfile.TemporaryDirectory() as d:
            gt_dir = os.path.join(d, "mav0", "state_groundtruth_estimate0")
            os.makedirs(gt_dir)
            with open(os.path.join(gt_dir, "data.csv"), "w") as f:
                f.write("#t,px,py,pz,qw,qx,qy,qz,vx,vy,vz\n")
                for i in range(len(t)):
                    f.write(f"{t[i]},{p[i,0]},{p[i,1]},{p[i,2]},1,0,0,0,{v[i,0]},{v[i,1]},{v[i,2]}\n")
            est = os.path.join(d, "est.csv")
            with open(est, "w") as f:
                f.write("#timestamp [ns],p_x,p_y,p_z,q_w,q_x,q_y,q_z,v_x,v_y,v_z\n")
                for i in range(len(t)):
                    f.write(f"{t[i]},{p_est[i,0]},{p_est[i,1]},{p_est[i,2]},1,0,0,0,{v_est[i,0]},{v_est[i,1]},{v_est[i,2]}\n")
            r = scale.scale_of_run(est, d)
        self.assertAlmostEqual(r["mean"], 1.03, delta=1e-6)
        self.assertAlmostEqual(r["end"], 1.03, delta=1e-6)
        self.assertAlmostEqual(r["speed95"], 1.03, delta=1e-3)


if __name__ == "__main__":
    unittest.main(verbosity=2)
