"""
Lidar scan check (headless-friendly)
====================================
Collects N scans from /scan, prints quality stats, and saves a top-down PNG of the scan.
Keep the lidar still: the per-beam noise numbers assume a static scene.

Usage (with the driver running, workspace sourced):
    python3 scripts/scan_check.py [--scans 50] [--out scan.png] [--extent 6]
"""

import argparse
import math
import time
import warnings

import numpy as np
import rclpy
from PIL import Image, ImageDraw
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class Collector(Node):
    def __init__(self, n):
        super().__init__('scan_check')
        self.n = n
        self.scans = []
        self.stamps = []
        self.create_subscription(LaserScan, '/scan', self.cb, qos_profile_sensor_data)

    def cb(self, msg):
        if len(self.scans) < self.n:
            self.scans.append(msg)
            self.stamps.append(time.monotonic())


def render(xs, ys, extent, out, px=800):
    img = Image.new('RGB', (px, px), 'white')
    d = ImageDraw.Draw(img)
    s = px / (2 * extent)
    c = px / 2
    for r in range(1, int(extent) + 1):  # 1 m range rings
        d.ellipse([c - r * s, c - r * s, c + r * s, c + r * s], outline=(220, 220, 220))
    d.line([c, c, c + 0.5 * s, c], fill=(220, 0, 0), width=3)  # +x (forward) in red
    d.line([c, c, c, c - 0.5 * s], fill=(0, 160, 0), width=3)  # +y (left) in green
    for x, y in zip(xs, ys):
        u, v = c + x * s, c - y * s
        d.rectangle([u - 1, v - 1, u + 1, v + 1], fill=(0, 0, 0))
    d.text((8, 8), f'1 m rings, extent +/-{extent} m, red=+x green=+y', fill=(80, 80, 80))
    img.save(out)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--scans', type=int, default=50)
    ap.add_argument('--out', default='scan.png')
    ap.add_argument('--extent', type=float, default=6.0, help='half-width of the image in metres')
    args = ap.parse_args()

    rclpy.init()
    node = Collector(args.scans)
    t0 = time.monotonic()
    while rclpy.ok() and len(node.scans) < args.scans and time.monotonic() - t0 < 30:
        rclpy.spin_once(node, timeout_sec=0.5)
    scans = node.scans
    node.destroy_node()
    rclpy.shutdown()
    if not scans:
        raise SystemExit('No scans received on /scan within 30 s; is the driver running?')

    m = scans[0]
    sizes = {len(s.ranges) for s in scans}
    rate = (len(scans) - 1) / (node.stamps[-1] - node.stamps[0]) if len(scans) > 1 else float('nan')
    R = np.array([s.ranges for s in scans if len(s.ranges) == len(m.ranges)], dtype=float)
    valid = (R >= m.range_min) & (R <= m.range_max) & np.isfinite(R)

    print(f'frame={m.header.frame_id} scans={len(scans)} rate={rate:.2f} Hz beams/scan={sorted(sizes)}')
    print(f'angle {math.degrees(m.angle_min):.1f}..{math.degrees(m.angle_max):.1f} deg, '
          f'increment {math.degrees(m.angle_increment):.3f} deg, '
          f'range limits {m.range_min:.2f}..{m.range_max:.2f} m')
    print(f'valid returns: {valid.mean() * 100:.1f}% of beams')

    Rv = np.where(valid, R, np.nan)
    warnings.simplefilter('ignore', RuntimeWarning)  # beams that never return give all-NaN columns
    with np.errstate(all='ignore'):
        med = np.nanmedian(Rv, axis=0)
        std = np.nanstd(Rv, axis=0)
        hit_frac = valid.mean(axis=0)
    stable = hit_frac > 0.8
    if stable.any():
        print(f'range of stable beams: {np.nanmin(med[stable]):.3f} .. {np.nanmax(med[stable]):.3f} m '
              f'(median {np.nanmedian(med[stable]):.3f} m)')
        print(f'per-beam noise (std over scans, static scene): median {np.nanmedian(std[stable]) * 1000:.1f} mm, '
              f'95th pct {np.nanpercentile(std[stable], 95) * 1000:.1f} mm')

    # Cardinal directions, handy for tape-measure checks.
    ang = m.angle_min + np.arange(R.shape[1]) * m.angle_increment
    for name, a in (('front +x', 0.0), ('left +y', math.pi / 2), ('back -x', math.pi), ('right -y', -math.pi / 2)):
        i = int(np.argmin(np.abs(np.angle(np.exp(1j * (ang - a))))))
        print(f'  {name:9s} ({math.degrees(ang[i]):7.2f} deg): {med[i]:.3f} m')

    ok = np.isfinite(med)
    render(med[ok] * np.cos(ang[ok]), med[ok] * np.sin(ang[ok]), args.extent, args.out)
    print(f'saved {args.out}')


if __name__ == '__main__':
    main()
