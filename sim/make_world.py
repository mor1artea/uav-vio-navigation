#!/usr/bin/env python3
"""Writes the Gazebo world for the simulated flights: flat textured ground, wind, and the Iris quad of
ardupilot_gazebo with a camera looking straight down.

The ground texture is the procedural one of tools/render_ground.py (no repeating patterns), in tiles.
Area in NED metres around the start: --north MIN MAX, --east MIN MAX. The default covers a 2.5 km
leg to the north and back.

Camera: 640x480, 30 fps, equidistant fisheye with fx = fy = 340 (the guess in config/vio.yaml), mounted
3 cm behind and 5 cm below the IMU, top of the image towards the nose. --lens pinhole is the fallback
if the wide angle camera of Gazebo does not work on your graphics driver.

Output: <out>/world.sdf, <out>/models/, <out>/camera.yaml

Example:
  sim/make_world.py ~/sim/world_line
"""

import argparse
import math
import os
import re
import shutil
import sys

import cv2
import numpy as np
import yaml

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "tools"))
from render_ground import ground_texture  # noqa: E402

WIDTH, HEIGHT, FPS, FX = 640, 480, 30, 340.0
CAM_FLU = (-0.03, 0.0, -0.05)  # camera position in the model frame (x forward, y left, z up)


def make_tiles(out, north, east, tile, px, contrast, seed):
    """Texture images and one SDF visual per tile. Gazebo x is east, y is north."""
    tex_dir = os.path.join(out, "models", "vio_ground", "textures")
    os.makedirs(tex_dir, exist_ok=True)
    ne = int(math.ceil((east[1] - east[0]) / tile))
    nn = int(math.ceil((north[1] - north[0]) / tile))
    metres = tile / px
    visuals = []
    for j in range(nn):
        for i in range(ne):
            x0, y0 = east[0] + i * tile, north[0] + j * tile
            img = np.zeros((px, px), np.uint8)
            xs = x0 + (np.arange(px) + 0.5) * metres
            for r0 in range(0, px, 256):  # in strips: less memory
                rows = np.arange(r0, min(r0 + 256, px))
                ys = y0 + tile - (rows + 0.5) * metres  # image row 0 is the north edge
                x, y = np.meshgrid(xs, ys)
                tex = ground_texture(x, y, np.full(x.shape, metres), seed)
                img[rows] = np.clip(115 + contrast * tex, 0, 255).astype(np.uint8)
            name = f"tile_{i}_{j}.png"
            cv2.imwrite(os.path.join(tex_dir, name), cv2.cvtColor(img, cv2.COLOR_GRAY2BGR))
            visuals.append(f"""      <visual name="t_{i}_{j}">
        <pose>{x0 + tile / 2} {y0 + tile / 2} 0 0 0 0</pose>
        <cast_shadows>false</cast_shadows>
        <geometry><plane><normal>0 0 1</normal><size>{tile} {tile}</size></plane></geometry>
        <material>
          <ambient>1 1 1 1</ambient><diffuse>1 1 1 1</diffuse>
          <pbr><metal><albedo_map>model://vio_ground/textures/{name}</albedo_map>
          <metalness>0.0</metalness><roughness>1.0</roughness></metal></pbr>
        </material>
      </visual>""")
        print(f"  ground row {j + 1}/{nn}", flush=True)
    size = 2 * max(abs(v) for v in north + east) + 100
    model = f"""<?xml version="1.0"?>
<sdf version="1.9">
  <model name="vio_ground">
    <static>true</static>
    <link name="link">
      <collision name="ground">
        <geometry><plane><normal>0 0 1</normal><size>{size} {size}</size></plane></geometry>
      </collision>
{chr(10).join(visuals)}
    </link>
  </model>
</sdf>
"""
    write_model(out, "vio_ground", model)
    return ne * nn


def write_model(out, name, sdf):
    d = os.path.join(out, "models", name)
    os.makedirs(d, exist_ok=True)
    with open(os.path.join(d, "model.sdf"), "w") as f:
        f.write(sdf)
    with open(os.path.join(d, "model.config"), "w") as f:
        f.write(f"""<?xml version="1.0"?>
<model><name>{name}</name><version>1.0</version><sdf version="1.9">model.sdf</sdf></model>
""")


def camera_sensor(lens):
    """The camera link, and the matching description for vio_replay."""
    if lens == "equidistant":
        hfov, fx = WIDTH / FX, FX  # equidistant, scaled to the horizontal fov: r = fx * theta
        kind = "wideanglecamera"
        extra = """<lens><type>equidistant</type><scale_to_hfov>true</scale_to_hfov>
            <cutoff_angle>1.5707</cutoff_angle><env_texture_size>512</env_texture_size></lens>"""
        model, coeffs = "equidistant", [0.0, 0.0, 0.0, 0.0]
    else:
        hfov = math.radians(100.0)
        fx = WIDTH / 2 / math.tan(hfov / 2)
        kind, extra = "camera", ""
        model, coeffs = "radtan", [0.0, 0.0, 0.0, 0.0]
    x, y, z = CAM_FLU
    sdf = f"""    <link name="vio_camera_link">
      <pose>{x} {y} {z} 0 {math.pi / 2} 0</pose>
      <inertial><mass>0.001</mass>
        <inertia><ixx>1e-6</ixx><iyy>1e-6</iyy><izz>1e-6</izz></inertia></inertial>
      <sensor name="vio_camera" type="{kind}">
        <camera>
          <horizontal_fov>{hfov}</horizontal_fov>
          <image><width>{WIDTH}</width><height>{HEIGHT}</height><format>R8G8B8</format></image>
          <clip><near>0.1</near><far>3000</far></clip>
          {extra}
        </camera>
        <always_on>1</always_on>
        <update_rate>{FPS}</update_rate>
        <topic>/vio/camera</topic>
      </sensor>
    </link>
    <joint name="vio_camera_joint" type="fixed">
      <parent>base_link</parent><child>vio_camera_link</child>
    </joint>
"""
    # Body FRD from camera (x image right, y image down, z optical axis), top of the image to the nose.
    T = [0.0, -1.0, 0.0, CAM_FLU[0],
         1.0, 0.0, 0.0, -CAM_FLU[1],
         0.0, 0.0, 1.0, -CAM_FLU[2],
         0.0, 0.0, 0.0, 1.0]
    info = dict(sensor_type="camera", comment=f"Gazebo {kind}", rate_hz=FPS,
                resolution=[WIDTH, HEIGHT], camera_model="pinhole",
                intrinsics=[fx, fx, (WIDTH - 1) / 2, (HEIGHT - 1) / 2],
                distortion_model=model, distortion_coefficients=coeffs,
                T_BS=dict(cols=4, rows=4, data=T))
    return sdf, info


def make_drone(out, plugin, lens):
    """Copies the two Iris models of ardupilot_gazebo and adds the camera, wind and ground truth."""
    def read(name):
        path = os.path.join(plugin, "models", name, "model.sdf")
        if not os.path.exists(path):
            sys.exit(f"error: {path} not found, run sim/setup.sh or give --plugin")
        with open(path) as f:
            return f.read()

    def insert(text, marker, extra, before=False):
        if marker not in text:
            sys.exit(f"error: '{marker}' not found in the ardupilot_gazebo model (another version?)")
        return text.replace(marker, extra + marker if before else marker + extra, 1)

    camera, info = camera_sensor(lens)
    body = read("iris_with_standoffs")
    body = re.sub(r"<model name=['\"]iris_with_standoffs['\"]>", "<model name='vio_iris_body'>", body)
    body, n = re.subn(r"(<link name=['\"]base_link['\"]>)", r"\1\n      <enable_wind>true</enable_wind>", body)
    if n != 1:
        sys.exit("error: base_link not found in the ardupilot_gazebo model (another version?)")
    body = insert(body, "</model>", camera, before=True)
    write_model(out, "vio_iris_body", body)

    top = read("iris_with_ardupilot")
    if "model://iris_with_standoffs" not in top:
        sys.exit("error: iris_with_ardupilot does not include iris_with_standoffs (another version?)")
    top = top.replace("model://iris_with_standoffs", "model://vio_iris_body")
    top = top.replace("iris_with_standoffs::", "vio_iris_body::")
    top = top.replace('<model name="iris_with_ardupilot">', '<model name="vio_iris">')
    truth = """    <plugin filename="gz-sim-odometry-publisher-system"
        name="gz::sim::systems::OdometryPublisher">
      <odom_frame>world</odom_frame>
      <robot_base_frame>base_link</robot_base_frame>
      <dimensions>3</dimensions>
      <odom_publish_frequency>200</odom_publish_frequency>
      <odom_topic>/vio/truth</odom_topic>
    </plugin>
"""
    top = insert(top, "</model>", truth, before=True)
    write_model(out, "vio_iris", top)
    return info


WORLD = """<?xml version="1.0"?>
<sdf version="1.9">
  <world name="vio">
    <physics name="1ms" type="ignore">
      <max_step_size>0.001</max_step_size>
      <real_time_factor>{rtf}</real_time_factor>
    </physics>
    <gravity>0 0 -9.8</gravity>
    <plugin filename="gz-sim-physics-system" name="gz::sim::systems::Physics"/>
    <plugin filename="gz-sim-sensors-system" name="gz::sim::systems::Sensors">
      <render_engine>ogre2</render_engine>
    </plugin>
    <plugin filename="gz-sim-user-commands-system" name="gz::sim::systems::UserCommands"/>
    <plugin filename="gz-sim-scene-broadcaster-system" name="gz::sim::systems::SceneBroadcaster"/>
    <plugin filename="gz-sim-imu-system" name="gz::sim::systems::Imu"/>
    <!-- Wind and air drag: force = mass * k * (wind - velocity). The speed and the gusts are set by
         sim/fly_record.py while it flies. -->
    <wind><linear_velocity>0 0 0</linear_velocity></wind>
    <plugin filename="gz-sim-wind-effects-system" name="gz::sim::systems::WindEffects">
      <force_approximation_scaling_factor>{drag}</force_approximation_scaling_factor>
      <horizontal>
        <magnitude><time_for_rise>1</time_for_rise></magnitude>
        <direction><time_for_rise>1</time_for_rise></direction>
      </horizontal>
      <vertical><time_for_rise>1</time_for_rise></vertical>
    </plugin>
    <spherical_coordinates>
      <latitude_deg>-35.363262</latitude_deg>
      <longitude_deg>149.165237</longitude_deg>
      <elevation>584</elevation>
      <heading_deg>0</heading_deg>
      <surface_model>EARTH_WGS84</surface_model>
    </spherical_coordinates>
    <scene><ambient>0.8 0.8 0.8</ambient><background>0.75 0.8 0.9</background></scene>
    <light type="directional" name="sun">
      <cast_shadows>false</cast_shadows>
      <pose>0 0 100 0 0 0</pose>
      <diffuse>0.6 0.6 0.6 1</diffuse>
      <direction>-0.3 0.2 -0.9</direction>
    </light>
    <include><uri>model://vio_ground</uri></include>
    <include>
      <uri>model://vio_iris</uri>
      <pose degrees="true">0 0 0.195 0 0 90</pose>
    </include>
  </world>
</sdf>
"""


def main():
    ap = argparse.ArgumentParser(description="Write the Gazebo world for simulated flights.")
    ap.add_argument("out", help="world folder")
    ap.add_argument("--north", type=float, nargs=2, default=[-200.0, 2700.0], help="area, m")
    ap.add_argument("--east", type=float, nargs=2, default=[-200.0, 200.0], help="area, m")
    ap.add_argument("--tile", type=float, default=100.0, help="tile size, m")
    ap.add_argument("--tile-px", type=int, default=1024, help="pixels per tile side (1024: 0.1 m/px)")
    ap.add_argument("--contrast", type=float, default=35.0, help="texture spread, grey levels")
    ap.add_argument("--drag", type=float, default=0.25,
                    help="air drag, 1/s. 0.25 gives about 17 deg of tilt at 12 m/s (a guess)")
    ap.add_argument("--rtf", type=float, default=1.0, help="real time factor; below 1 if frames drop")
    ap.add_argument("--lens", choices=["equidistant", "pinhole"], default="equidistant")
    ap.add_argument("--plugin", default="~/sim/ardupilot_gazebo")
    ap.add_argument("--seed", type=int, default=1)
    args = ap.parse_args()

    out = os.path.abspath(os.path.expanduser(args.out))
    if os.path.exists(os.path.join(out, "models")):
        shutil.rmtree(os.path.join(out, "models"))
    os.makedirs(out, exist_ok=True)
    info = make_drone(out, os.path.expanduser(args.plugin), args.lens)
    n = make_tiles(out, args.north, args.east, args.tile, args.tile_px, args.contrast, args.seed)
    with open(os.path.join(out, "world.sdf"), "w") as f:
        f.write(WORLD.format(rtf=args.rtf, drag=args.drag))
    with open(os.path.join(out, "camera.yaml"), "w") as f:
        yaml.safe_dump(info, f, sort_keys=False, default_flow_style=None)
    mem = n * args.tile_px ** 2 * 4 * 4 / 3 / 1e9
    print(f"wrote {out}: {n} ground tiles (about {mem:.1f} GB of graphics memory)")


if __name__ == "__main__":
    main()
