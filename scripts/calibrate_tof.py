#!/usr/bin/env python3
"""Calibrate the ToF against the floor.

Put the robot on the floor facing at least 1.5 m of clear, flat floor, head
centred (pan 0, tilt 0), and run inside the robot container:

    python3 scripts/calibrate_tof.py

It averages a few seconds of /tof/depth/image_raw, fits a plane to the floor
and reports:
  - the field of view that makes the floor flattest (vs the spec values)
  - the ToF's real height above the floor and its pitch/roll error relative
    to the URDF, i.e. the corrections to put in rainbow_description
  - the floor noise by distance after correction, i.e. the smallest obstacle
    that can be told apart from the floor
Nothing is changed on the robot.
"""

import sys
import time

import numpy as np
import rclpy
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from tf2_ros import Buffer, TransformListener

SPEC_HFOV, SPEC_VFOV = 64.3, 50.4  # degrees, Arducam spec (tof_node defaults)
MEASURED_HEIGHT = 0.13  # m: camera height above the floor, measured with a ruler
CORRIDOR_WIDTH = 0.94  # m: optional cross-check, wall-to-wall width in front of the robot
CAPTURE_S = 5.0
FLOOR_RANGE = (0.25, 1.6)  # m: depth used to find the floor


def rays(w, h, hfov, vfov):
    fx = w / (2 * np.tan(np.radians(hfov) / 2))
    fy = h / (2 * np.tan(np.radians(vfov) / 2))
    u, v = np.meshgrid(np.arange(w), np.arange(h))
    return (u - w / 2) / fx, (v - h / 2) / fy


def to_points(depth, rx, ry):
    z = depth
    return np.stack((rx * z, ry * z, z), axis=-1)


def fit_plane(p, iters=300, tol=0.015, rng=np.random.default_rng(0)):
    """RANSAC plane, refined on inliers. Returns (normal pointing up, d, inliers)."""
    best = None
    for _ in range(iters):
        a, b, c = p[rng.choice(len(p), 3, replace=False)]
        n = np.cross(b - a, c - a)
        if np.linalg.norm(n) < 1e-9:
            continue
        n /= np.linalg.norm(n)
        inl = np.abs((p - a) @ n) < tol
        if best is None or inl.sum() > best.sum():
            best = inl
    q = p[best]
    c = q.mean(axis=0)
    n = np.linalg.svd(q - c)[2][-1]
    if n[1] > 0:  # optical frame: y is down, so the floor's upward normal has y < 0
        n = -n
    d = -n @ c
    return n, d, np.abs(p @ n + d) < tol


def main():
    rclpy.init()
    node = rclpy.create_node('tof_calibration')
    buf = Buffer()
    TransformListener(buf, node)
    frames = []
    node.create_subscription(
        Image, '/tof/depth/image_raw',
        lambda m: frames.append(np.frombuffer(m.data, np.float32).reshape(m.height, m.width)),
        qos_profile_sensor_data)
    print(f'Capturing {CAPTURE_S:.0f} s of ToF depth; keep the robot still...')
    end = time.time() + CAPTURE_S
    while time.time() < end:
        rclpy.spin_once(node, timeout_sec=0.05)
    if len(frames) < 10:
        sys.exit(f'Only {len(frames)} depth frames received; is the ToF node running?')

    stack = np.stack(frames).astype(np.float64)
    stack[stack <= 0] = np.nan
    depth = np.nanmedian(stack, axis=0)  # per-pixel median over time
    h, w = depth.shape
    floor_band = (depth > FLOOR_RANGE[0]) & (depth < FLOOR_RANGE[1])
    floor_band[: h // 2] = False  # the floor is in the lower half of the image
    print(f'{len(frames)} frames, {w}x{h}; {floor_band.sum()} candidate floor pixels')

    # Field of view: flatness alone cannot pick it (shrinking the FOV squeezes
    # every surface flat), so choose the scale whose fitted floor sits at the
    # measured camera height. Fit on a fixed random sample of floor pixels so
    # this stays quick on the Pi.
    idx = np.flatnonzero(floor_band)
    idx = np.random.default_rng(1).choice(idx, min(3000, len(idx)), replace=False)

    def evaluate(scale):
        rx, ry = rays(w, h, SPEC_HFOV * scale, SPEC_VFOV * scale)
        p = to_points(depth, rx, ry).reshape(-1, 3)[idx]
        n, d, inl = fit_plane(p, iters=150)
        rms = np.sqrt(np.mean((p[inl] @ n + d) ** 2))
        return scale, n, d, rms, inl.mean()

    results = [evaluate(sc) for sc in np.arange(0.60, 1.301, 0.02)]
    print('\n  scale   FOV (deg)     floor height   RMS')
    for sc, n_, d_, rms_, _ in results[::3]:
        print(f'  {sc:.2f}   {SPEC_HFOV * sc:4.1f} x {SPEC_VFOV * sc:4.1f}   '
              f'{abs(d_) * 100:5.1f} cm     {rms_ * 100:.1f} cm')
    s, n, d, rms, frac = min(results, key=lambda r: abs(abs(r[2]) - MEASURED_HEIGHT))
    hfov, vfov = SPEC_HFOV * s, SPEC_VFOV * s

    # Pose relative to the floor (optical frame: x right, y down, z forward)
    height = abs(d)
    pitch_down = np.degrees(np.arctan2(-n[2], -n[1]))  # + = looking down
    roll = np.degrees(np.arctan2(-n[0], -n[1]))         # + = right side low

    try:
        t = buf.lookup_transform('base_footprint', 'tof_optical_frame', rclpy.time.Time())
        urdf_height = t.transform.translation.z
    except Exception:
        urdf_height = float('nan')

    print()
    print(f'Field of view: {hfov:.1f} x {vfov:.1f} deg (spec {SPEC_HFOV} x {SPEC_VFOV}, '
          f'scale {s:.3f})')
    print(f'Floor fit: {frac * 100:.0f}% of candidate pixels on the plane, RMS {rms * 100:.1f} cm')
    print(f'ToF height above floor: {height * 100:.1f} cm (URDF says {urdf_height * 100:.1f} cm)')
    print(f'ToF pitch: {pitch_down:+.1f} deg (+ = looking down; URDF assumes 0)')
    print(f'ToF roll:  {roll:+.1f} deg (+ = right side low; URDF assumes 0)')

    # Cross-check with the corridor: with the right FOV the side walls should
    # sit CORRIDOR_WIDTH apart. Points 3-40 cm above the floor, 0.6-1.4 m ahead.
    rx, ry = rays(w, h, hfov, vfov)
    p = to_points(depth, rx, ry).reshape(-1, 3)
    height_above = p @ n + d  # n points up, so this is positive above the floor
    ahead = p[:, 2]
    side = p[:, 0]
    wall = (height_above > 0.03) & (height_above < 0.40) & (ahead > 0.6) & (ahead < 1.4)
    if wall.sum() > 50:
        left, right = side[wall & (side < 0)], side[wall & (side > 0)]
        if len(left) > 20 and len(right) > 20:
            width = np.median(right) - np.median(left)
            print(f'Corridor width seen: {width * 100:.0f} cm '
                  f'(measured {CORRIDOR_WIDTH * 100:.0f} cm)')

    # Floor noise by distance after correction: how far floor pixels are from the plane
    rx, ry = rays(w, h, hfov, vfov)
    p = to_points(depth, rx, ry)
    dist = p @ n + d  # height above the fitted floor
    single = to_points(np.where(np.isfinite(stack[-1]), stack[-1], np.nan), rx, ry) @ n + d
    print('\nHeight of floor pixels above the fitted floor, lower half of the image:')
    print('  depth range    median   p5..p95 (averaged)   p5..p95 (single frame)')
    lower = np.zeros_like(floor_band)
    lower[h // 2:] = True
    for lo, hi in [(0.25, 0.5), (0.5, 0.75), (0.75, 1.0), (1.0, 1.5), (1.5, 2.0)]:
        m = lower & (depth >= lo) & (depth < hi) & (np.abs(dist) < 0.10)
        if m.sum() < 30:
            continue
        a, sgl = dist[m], single[m & np.isfinite(single)]
        print(f'  {lo:.2f}-{hi:.2f} m   {np.median(a) * 100:5.1f} cm  '
              f'{np.percentile(a, 5) * 100:5.1f}..{np.percentile(a, 95) * 100:5.1f} cm         '
              f'{np.percentile(sgl, 5) * 100:5.1f}..{np.percentile(sgl, 95) * 100:5.1f} cm')
    rclpy.shutdown()


if __name__ == '__main__':
    main()
