#!/usr/bin/env python3
"""Flies the simulated drone (SITL + Gazebo, started with sim/run_sim.sh) and records a session.

SITL ONLY. This script arms and flies the drone and changes parameters. It connects to SITL on this
computer and nowhere else, and stops if the other side does not have the SITL parameters. Never
change it to connect to a real flight controller.

The flight: wait on the ground, take off, two turns on the spot (they line up the two clocks), fly
the route with the autopilot in GUIDED, land. ArduPilot flies on the simulated GPS; the recording has
no GPS in it.

  --route straight --leg metres north at constant speed, land at the end (the scale test)
  --route line     --leg metres north, and back
  --route square   a square with sides of --leg metres (make the world big enough: --east)

IMU noise in 4.6.3 SITL (read in AP_InertialSensor_SITL.cpp): the accelerometer always gets a uniform
noise of 0.01 m/s^2 per sample; SIM_ACC1_RND only scales simulated motor vibration (SIM_VIB_*), so it
is not set here. SIM_GYR1_RND (--gyro-noise) adds uniform gyro noise in deg/s times the throttle,
only with the motors running.

Example (second terminal, after sim/run_sim.sh):
  sim/fly_record.py ~/datasets/sim/line1 --world ~/sim/world_line --speed 12 --height 40 --wind 4 --gust 1.5
"""

import argparse
import math
import os
import queue
import shutil
import sys
import threading
import time

import cv2
import numpy as np

os.environ.setdefault("MAVLINK20", "1")
from pymavlink import mavutil  # noqa: E402

try:
    from gz.msgs10.image_pb2 import Image
    from gz.msgs10.odometry_pb2 import Odometry
    from gz.msgs10.wind_pb2 import Wind
    from gz.transport13 import Node
except ImportError as e:
    sys.exit(f"error: Gazebo Harmonic Python packages not found ({e}). Run sim/setup.sh")

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from make_session import convert  # noqa: E402

SITL_ADDRESS = "tcp:127.0.0.1:5760"  # fixed on purpose, see above


class Recorder:
    """Writes the raw files. Gazebo callbacks run in their own threads."""

    def __init__(self, out):
        self.raw = os.path.join(out, "raw")
        self.images = os.path.join(out, "mav0", "cam0", "data")
        os.makedirs(self.raw)
        os.makedirs(self.images)
        self.files = {n: open(os.path.join(self.raw, n + ".csv"), "w")
                      for n in ("truth", "imu", "baro", "landed", "clock", "images")}
        self.sim_ns = 0  # latest ground truth time
        self.fc_us = 0   # latest FC time, from HIGHRES_IMU
        self.n_images = 0
        self.dropped = 0
        self.queue = queue.Queue(maxsize=200)
        self.writer = threading.Thread(target=self._write_images, daemon=True)
        self.writer.start()
        self.lock = threading.Lock()

    def on_truth(self, m):
        t = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nsec
        p, q = m.pose.position, m.pose.orientation
        with self.lock:
            self.sim_ns = t
            if not self.files["truth"].closed:
                self.files["truth"].write(f"{t},{p.x},{p.y},{p.z},{q.w},{q.x},{q.y},{q.z}\n")

    def on_image(self, m):
        t = m.header.stamp.sec * 1_000_000_000 + m.header.stamp.nsec
        try:
            self.queue.put_nowait((t, m.height, m.width, bytes(m.data)))
        except queue.Full:
            self.dropped += 1

    def _write_images(self):
        while True:
            item = self.queue.get()
            if item is None:
                return
            t, h, w, data = item
            rgb = np.frombuffer(data, np.uint8).reshape(h, w, -1)
            gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
            cv2.imwrite(os.path.join(self.images, f"{t}.png"), gray, [cv2.IMWRITE_PNG_COMPRESSION, 1])
            self.files["images"].write(f"{t},{t}.png\n")
            self.n_images += 1

    def on_mavlink(self, m):
        kind = m.get_type()
        if kind == "HIGHRES_IMU":
            self.files["imu"].write(f"{m.time_usec},{m.xgyro},{m.ygyro},{m.zgyro},{m.xacc},{m.yacc},{m.zacc}\n")
            self.files["baro"].write(f"{m.time_usec},{m.pressure_alt}\n")
            if self.sim_ns:
                self.files["clock"].write(f"{m.time_usec},{self.sim_ns}\n")
            self.fc_us = m.time_usec
        elif kind == "EXTENDED_SYS_STATE" and self.fc_us:
            self.files["landed"].write(f"{self.fc_us},{m.landed_state}\n")

    def close(self):
        self.queue.put(None)
        self.writer.join()
        with self.lock:
            for f in self.files.values():
                f.close()


class Drone:
    def __init__(self, link, recorder, wind):
        self.link = link
        self.rec = recorder
        self.wind = wind
        self.position = None  # NED from the EKF, for the route only
        self.armed = False
        self.landed_state = 0
        self.last_heartbeat = 0.0

    def poll(self):
        """Handles messages for a moment; every wait goes through here, so recording never stops."""
        m = self.link.recv_match(blocking=True, timeout=0.05)
        while m is not None:
            kind = m.get_type()
            self.rec.on_mavlink(m)
            if kind == "LOCAL_POSITION_NED":
                self.position = (m.x, m.y, m.z)
            elif kind == "HEARTBEAT" and m.get_srcComponent() == 1:
                self.armed = bool(m.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED)
            elif kind == "EXTENDED_SYS_STATE":
                self.landed_state = m.landed_state
            elif kind == "STATUSTEXT":
                print("  FC:", m.text)
            m = self.link.recv_match(blocking=False)
        now = time.monotonic()
        if now - self.last_heartbeat > 1.0:  # or the GCS failsafe stops the flight
            self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS,
                                         mavutil.mavlink.MAV_AUTOPILOT_INVALID, 0, 0, 0)
            self.last_heartbeat = now
        self.wind.step(self.rec.fc_us * 1e-6)

    def wait(self, condition, seconds, what):
        """Waits for condition(), at most this many seconds of FC (simulated) time."""
        start, wall = self.rec.fc_us, time.monotonic()
        while not condition():
            self.poll()
            if (self.rec.fc_us - start) * 1e-6 > seconds or time.monotonic() - wall > 20 * seconds + 60:
                raise RuntimeError(f"timeout: {what}")

    def sleep(self, seconds):
        start = self.rec.fc_us
        self.wait(lambda: (self.rec.fc_us - start) * 1e-6 >= seconds, seconds + 5, "sleep")

    def command(self, cmd, *params):
        p = list(params) + [0] * (7 - len(params))
        self.link.mav.command_long_send(self.link.target_system, self.link.target_component, cmd, 0, *p)

    def set_param(self, name, value):
        self.link.mav.param_set_send(self.link.target_system, self.link.target_component,
                                     name.encode(), float(value), mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        end = time.monotonic() + 3
        while time.monotonic() < end:
            m = self.link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if m is not None and m.param_id == name:
                return
        raise RuntimeError(f"parameter {name} not confirmed")

    def check_sitl(self):
        self.link.mav.param_request_read_send(self.link.target_system, self.link.target_component,
                                              b"SIM_SPEEDUP", -1)
        end = time.monotonic() + 5
        while time.monotonic() < end:
            m = self.link.recv_match(type="PARAM_VALUE", blocking=True, timeout=0.5)
            if m is not None and m.param_id == "SIM_SPEEDUP":
                return
        sys.exit("error: no SIM_SPEEDUP parameter. This is not SITL; stopping without changing anything")

    def arm_and_takeoff(self, height):
        self.link.set_mode("GUIDED")
        print("arming (waits for the EKF and the simulated GPS)")
        start = time.monotonic()
        while not self.armed:
            self.command(mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 1)
            t = time.monotonic()
            while time.monotonic() - t < 2 and not self.armed:
                self.poll()
            if time.monotonic() - start > 180:
                raise RuntimeError("could not arm (see the FC messages above)")
        print("takeoff")
        self.command(mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, height)
        self.wait(lambda: self.position and self.position[2] < -0.95 * height, 60, "takeoff")

    def yaw_to(self, degrees):
        self.command(mavutil.mavlink.MAV_CMD_CONDITION_YAW, degrees % 360, 30, 0, 0)
        self.sleep(5)

    def go_to(self, north, east, down, speed):
        heading = math.atan2(east - self.position[1], north - self.position[0])
        distance = math.hypot(north - self.position[0], east - self.position[1])
        mask = 0x09F8  # position and yaw only
        sent = -1e9

        def there():
            nonlocal sent
            if self.rec.fc_us * 1e-6 - sent > 5:
                self.link.mav.set_position_target_local_ned_send(
                    0, self.link.target_system, self.link.target_component,
                    mavutil.mavlink.MAV_FRAME_LOCAL_NED, mask, north, east, down,
                    0, 0, 0, 0, 0, 0, heading, 0)
                sent = self.rec.fc_us * 1e-6
            return math.hypot(north - self.position[0], east - self.position[1]) < 5

        self.wait(there, distance / speed * 2 + 60, f"waypoint north {north}, east {east}")


class WindSource:
    """Mean wind plus gusts (a random walk that returns to zero), sent to Gazebo once a second."""

    def __init__(self, node, speed, from_deg, gust, seed):
        self.pub = node.advertise("/world/vio/wind", Wind)
        to = math.radians(from_deg + 180)
        self.mean = np.array([speed * math.sin(to), speed * math.cos(to)])  # east, north
        self.gust = np.zeros(2)
        self.sigma = gust
        self.rng = np.random.default_rng(seed)
        self.next = 0.0

    def step(self, t):
        if t < self.next:
            return
        self.next = t + 1.0
        a = math.exp(-1.0 / 4.0)  # gusts change over about 4 s
        self.gust = a * self.gust + math.sqrt(1 - a * a) * self.sigma * self.rng.normal(0, 1, 2)
        m = Wind()
        m.enable_wind = True
        m.linear_velocity.x, m.linear_velocity.y = self.mean + self.gust
        self.pub.publish(m)


def main():
    ap = argparse.ArgumentParser(description="Fly SITL + Gazebo and record a session (SITL only).")
    ap.add_argument("out", help="new session folder")
    ap.add_argument("--world", default="~/sim/world_line", help="world folder (for camera.yaml)")
    ap.add_argument("--route", choices=["straight", "line", "square"], default="line")
    ap.add_argument("--leg", type=float, default=2500.0, help="leg length, m")
    ap.add_argument("--speed", type=float, default=12.0, help="m/s")
    ap.add_argument("--height", type=float, default=40.0, help="m above the start")
    ap.add_argument("--wind", type=float, default=0.0, help="mean wind, m/s")
    ap.add_argument("--wind-from", type=float, default=270.0, help="direction the wind comes from, deg")
    ap.add_argument("--gust", type=float, default=0.0, help="gust spread per axis, m/s")
    ap.add_argument("--ground-time", type=float, default=10.0, help="still on the ground first, s")
    ap.add_argument("--gyro-noise", type=float, default=0.5,
                    help="SIM_GYR1_RND: gyro noise with the motors running, deg/s times the throttle (a guess)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--force", action="store_true", help="overwrite the session folder")
    args = ap.parse_args()

    out = os.path.expanduser(args.out)
    camera = os.path.join(os.path.expanduser(args.world), "camera.yaml")
    if not os.path.exists(camera):
        sys.exit(f"error: {camera} not found, run sim/make_world.py first")
    if os.path.exists(out):
        if not args.force:
            sys.exit(f"error: {out} exists, use --force to overwrite")
        shutil.rmtree(out)

    link = mavutil.mavlink_connection(SITL_ADDRESS, source_system=255)
    print("waiting for SITL")
    if link.wait_heartbeat(timeout=30) is None:
        sys.exit("error: no heartbeat from SITL, is sim/run_sim.sh running?")
    rec = Recorder(out)
    shutil.copy(camera, os.path.join(rec.raw, "camera.yaml"))
    node = Node()
    wind = WindSource(node, args.wind, args.wind_from, args.gust, args.seed)
    drone = Drone(link, rec, wind)
    drone.check_sitl()

    # SITL parameters. The filters are the ones of the real FC (docs/ARDUPILOT_SETUP.md).
    for name, value in (("INS_GYRO_FILTER", 80), ("INS_ACCEL_FILTER", 20),
                        ("SIM_GYR1_RND", args.gyro_noise),
                        ("WPNAV_SPEED", args.speed * 100), ("ANGLE_MAX", 4500)):
        drone.set_param(name, value)

    for message, hz in ((mavutil.mavlink.MAVLINK_MSG_ID_HIGHRES_IMU, 200),
                        (mavutil.mavlink.MAVLINK_MSG_ID_EXTENDED_SYS_STATE, 10),
                        (mavutil.mavlink.MAVLINK_MSG_ID_LOCAL_POSITION_NED, 10)):
        drone.command(mavutil.mavlink.MAV_CMD_SET_MESSAGE_INTERVAL, message, 1e6 / hz)

    if not node.subscribe(Odometry, "/vio/truth", rec.on_truth) or \
            not node.subscribe(Image, "/vio/camera", rec.on_image):
        sys.exit("error: cannot subscribe to the Gazebo topics")

    ok = False
    try:
        t = time.monotonic()
        while rec.sim_ns == 0 or rec.fc_us == 0:
            drone.poll()
            if time.monotonic() - t > 30:
                missing = "ground truth from Gazebo" if rec.sim_ns == 0 else "HIGHRES_IMU from SITL"
                raise RuntimeError(f"no {missing}")
        drone.sleep(args.ground_time)
        drone.arm_and_takeoff(args.height)
        print("turning (clock alignment)")
        for heading in (90, 270, 0):
            drone.yaw_to(heading)
        leg, down = args.leg, -args.height
        route = {"straight": [(leg, 0)], "line": [(leg, 0), (0, 0)],
                 "square": [(leg, 0), (leg, leg), (0, leg), (0, 0)]}[args.route]
        for north, east in route:
            print(f"flying to north {north:.0f} m, east {east:.0f} m")
            drone.go_to(north, east, down, args.speed)
        print("landing")
        drone.link.set_mode("LAND")
        drone.wait(lambda: not drone.armed, 120, "landing")
        drone.sleep(3)
        ok = True
    except (RuntimeError, KeyboardInterrupt) as e:
        print(f"stopped: {e!r}")
    finally:
        rec.close()
        print(f"recorded {rec.n_images} images, {rec.dropped} dropped by this script")

    if not convert(out):
        sys.exit("some checks failed: do not trust numbers from this session")
    if not ok:
        sys.exit("the flight did not finish; the session holds what was recorded")


if __name__ == "__main__":
    main()
