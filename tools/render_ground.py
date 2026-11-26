#!/usr/bin/env python3
"""Renders camera images of a flat textured ground for a synthetic session, so the front end can be
checked in our own geometry: a down-looking fisheye camera over flat ground, 20-50 m high, tilting
up to about 45 deg (EuRoC looks forward into rooms).

Takes a session from gen_synthetic.py (trajectory, IMU, ground truth, camera) and writes a new
session with the same IMU and ground truth and real images instead of feature tracks:
  mav0/cam0/data.csv, data/<timestamp>.png, sensor.yaml

How a frame is made: every pixel is unprojected through the session's camera model, the ray is
intersected with the ground plane z = 0 (NED), and the texture is evaluated at that point. Rays that
miss the ground (above the horizon) show a bright sky. The texture is procedural: fields with sharp
borders and several octaves of value noise from 64 m down to 0.25 m. Octaves finer than the ground
footprint of a pixel are faded out, so far ground is smooth instead of aliased.
Not modelled: motion blur, rolling shutter, exposure changes, 3D structure, vignetting.

Example:
  tools/gen_synthetic.py ~/datasets/synthetic/fig8_tilt --speed 12 --radius 50 --height 25
  tools/render_ground.py ~/datasets/synthetic/fig8_tilt ~/datasets/synthetic/fig8_tilt_img
  tools/render_ground.py ~/datasets/synthetic/fig8_tilt ~/datasets/synthetic/fig8_tilt_low --contrast 10
"""

import argparse
import os
import shutil
import struct
import sys
import zlib

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from eval_tracks import load_camera, unproject  # noqa: E402
from metrics import associate, load_trajectory  # noqa: E402

WAVELENGTHS = [64.0, 32.0, 16.0, 8.0, 4.0, 2.0, 1.0, 0.5, 0.25]  # m
TEXTURE_STD = 0.77  # measured standard deviation of the raw texture sum, so --contrast is in grey levels


def lattice(ix, iy, seed):
    """Pseudo-random value in [0, 1) for integer lattice points (hash, no tables)."""
    with np.errstate(over="ignore"):
        h = (ix.astype(np.uint64) * np.uint64(0x9E3779B97F4A7C15)) ^ \
            (iy.astype(np.uint64) * np.uint64(0xC2B2AE3D27D4EB4F)) ^ np.uint64(seed * 0x165667B1)
        h ^= h >> np.uint64(29)
        h *= np.uint64(0xBF58476D1CE4E5B9)
        h ^= h >> np.uint64(32)
    return (h >> np.uint64(40)).astype(np.float32) / np.float32(2 ** 24)


def value_noise(x, y, wavelength, seed):
    u, v = x / wavelength, y / wavelength
    iu, iv = np.floor(u), np.floor(v)
    fu, fv = u - iu, v - iv
    fu, fv = fu * fu * (3 - 2 * fu), fv * fv * (3 - 2 * fv)
    iu, iv = iu.astype(np.int64), iv.astype(np.int64)
    a, b = lattice(iu, iv, seed), lattice(iu + 1, iv, seed)
    c, d = lattice(iu, iv + 1, seed), lattice(iu + 1, iv + 1, seed)
    return (a * (1 - fu) + b * fu) * (1 - fv) + (c * (1 - fu) + d * fu) * fv


def ground_texture(x, y, footprint, seed):
    """Ground brightness around 0 with a standard deviation of about 1, at ground points (x, y) seen
    with a pixel footprint (m)."""
    # Fields: the coarsest octave quantized to a few levels gives patches with sharp borders.
    fields = np.floor(value_noise(x, y, 40.0, seed) * 5) / 4 - 0.5
    tex = 2.0 * fields
    for k, wl in enumerate(WAVELENGTHS):
        fade = np.clip(wl / footprint / 2 - 1, 0, 1)  # gone when the wavelength is under 2 pixels
        tex += fade * (wl / 8) ** 0.15 * (value_noise(x, y, wl, seed + k + 1) - 0.5)
    return tex / TEXTURE_STD


def write_png(path, img):
    """8-bit greyscale PNG without extra libraries."""
    h, w = img.shape
    raw = b"".join(b"\x00" + img[r].tobytes() for r in range(h))

    def chunk(kind, data):
        return struct.pack(">I", len(data)) + kind + data + struct.pack(">I", zlib.crc32(kind + data))

    with open(path, "wb") as f:
        f.write(b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", struct.pack(">IIBBBBB", w, h, 8, 0, 0, 0, 0)) +
                chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))


def main():
    ap = argparse.ArgumentParser(description="Render ground images for a synthetic session.")
    ap.add_argument("source", help="session from gen_synthetic.py")
    ap.add_argument("out", help="new session folder with images")
    ap.add_argument("--contrast", type=float, default=35.0, help="texture standard deviation, grey levels")
    ap.add_argument("--brightness", type=float, default=110.0, help="mean grey level of the ground")
    ap.add_argument("--noise", type=float, default=2.0, help="sensor noise, grey levels (1 sigma)")
    ap.add_argument("--duration", type=float, default=0, help="render only this many seconds (0 = all)")
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--force", action="store_true", help="overwrite the output folder")
    args = ap.parse_args()

    src, out = os.path.expanduser(args.source), os.path.expanduser(args.out)
    if os.path.exists(out):
        if not args.force:
            sys.exit(f"error: {out} exists, use --force to overwrite")
        shutil.rmtree(out)
    with open(os.path.join(src, "session.yaml")) as f:
        if yaml.safe_load(f).get("world_frame") != "NED":
            sys.exit("error: the source must be a gen_synthetic.py session (NED world)")

    cam, T_bc = load_camera(src)
    w, h = cam["width"], cam["height"]
    uu, vv = np.meshgrid(np.arange(w, dtype=float), np.arange(h, dtype=float))
    rays_cam = unproject(np.stack([uu.ravel(), vv.ravel()], axis=1), cam)

    # Frame times: the ones of the source feature observations.
    with open(os.path.join(src, "mav0", "cam0", "features.csv")) as f:
        stamps = np.unique([int(l.split(",", 1)[0]) for l in f if l[0] != "#"])
    if args.duration > 0:
        stamps = stamps[stamps <= stamps[0] + int(args.duration * 1e9)]
    dummy = (stamps, np.zeros((len(stamps), 3)), np.tile([1.0, 0, 0, 0], (len(stamps), 1)))
    stamps, _, _, p_wb, R_wb = associate(dummy, load_trajectory(src))

    for d in ("mav0/imu0", "mav0/state_groundtruth_estimate0"):
        shutil.copytree(os.path.join(src, d), os.path.join(out, d))
    shutil.copy(os.path.join(src, "session.yaml"), out)
    os.makedirs(os.path.join(out, "mav0", "cam0", "data"))
    with open(os.path.join(src, "mav0", "cam0", "sensor.yaml")) as f:
        sensor = yaml.safe_load(f)
    sensor["comment"] = f"rendered ground, contrast {args.contrast}, noise {args.noise}"
    with open(os.path.join(out, "mav0", "cam0", "sensor.yaml"), "w") as f:
        yaml.safe_dump(sensor, f, sort_keys=False, default_flow_style=None)

    rng = np.random.default_rng(args.seed)
    rows = []
    for k, t in enumerate(stamps):
        R_wc = R_wb[k] @ T_bc[:3, :3]
        c = p_wb[k] + R_wb[k] @ T_bc[:3, 3]
        d = rays_cam @ R_wc.T                       # rays in the world frame (NED)
        down = d[:, 2]
        ground = down > 0.02                        # rays that hit the ground in front of the horizon
        s = np.where(ground, -c[2] / np.where(ground, down, 1), 0)  # distance along the ray
        x, y = c[0] + s * d[:, 0], c[1] + s * d[:, 1]
        footprint = np.where(ground, s / (cam["fx"] * np.maximum(down, 0.02)), 1e3)
        tex = ground_texture(x, y, footprint, args.seed)
        haze = np.exp(-s / 2000.0)                  # far ground fades towards the sky colour
        img = args.brightness + args.contrast * tex
        img = haze * img + (1 - haze) * 200
        img = np.where(ground, img, 215 + 20 * np.clip(-down, 0, 1))  # sky, brighter higher up
        img = img + rng.normal(0, args.noise, img.shape)
        img = np.clip(np.round(img), 0, 255).astype(np.uint8).reshape(h, w)
        write_png(os.path.join(out, "mav0", "cam0", "data", f"{t}.png"), img)
        rows.append(f"{t},{t}.png")
        if k % 100 == 0:
            print(f"  frame {k}/{len(stamps)}", flush=True)

    with open(os.path.join(out, "mav0", "cam0", "data.csv"), "w") as f:
        f.write("#timestamp [ns],filename\n" + "\n".join(rows) + "\n")
    print(f"wrote {out}: {len(rows)} frames, {w}x{h}")


if __name__ == "__main__":
    main()
