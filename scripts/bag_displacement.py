#!/usr/bin/env python3
"""Net displacement and heading change of the lidar over a slither run, by 2D ICP between the first and
the last scan in a bag (the body is straight at both ends, so the lidar pose stands for the body pose).

    python3 scripts/bag_displacement.py ~/bags/slither_<time>

Prints dx (forward, lidar x), dy (left) and dyaw (+ = turned left / CCW) of the end pose in the start frame.
"""
import math
import sys

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import LaserScan


def read_scans(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id='mcap'), rosbag2_py.ConverterOptions('', ''))
    reader.set_filter(rosbag2_py.StorageFilter(topics=['/scan']))
    scans = []
    while reader.has_next():
        _, data, _ = reader.read_next()
        scans.append(deserialize_message(data, LaserScan))
    return scans


def points(msg, max_range=6.0):
    r = np.asarray(msg.ranges, dtype=float)
    a = msg.angle_min + msg.angle_increment * np.arange(len(r))
    ok = np.isfinite(r) & (r > msg.range_min) & (r < min(msg.range_max, max_range))
    return np.stack([r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])], axis=1)


def icp(src, dst, iters=80, gates=(0.5, 0.25, 0.1)):
    """Rigid (R, t) taking src onto dst, point-to-point, with brute-force nearest neighbours. The outlier
    gate shrinks coarse to fine: a tight gate from the start stalls on turns of 10+ deg."""
    R, t = np.eye(2), np.zeros(2)
    for k in range(iters):
        reject = gates[min(k * len(gates) // iters, len(gates) - 1)]
        moved = src @ R.T + t
        d2 = ((moved[:, None, :] - dst[None, :, :]) ** 2).sum(-1)
        j = d2.argmin(1)
        keep = d2[np.arange(len(src)), j] < reject ** 2
        if keep.sum() < 20:
            break
        a, b = moved[keep], dst[j[keep]]
        ma, mb = a.mean(0), b.mean(0)
        U, _, Vt = np.linalg.svd((a - ma).T @ (b - mb))
        dR = (U @ Vt).T
        if np.linalg.det(dR) < 0:
            Vt[1] *= -1
            dR = (U @ Vt).T
        R, t = dR @ R, dR @ t + mb - dR @ ma
    return R, t, int(keep.sum()), len(src)


def main(path):
    scans = read_scans(path)
    first, last = scans[0], scans[-1]
    dur = (last.header.stamp.sec - first.header.stamp.sec) + (last.header.stamp.nanosec - first.header.stamp.nanosec) * 1e-9
    # end pose in the start frame: points seen at the end, moved by it, land on the start scan
    R, t, n_in, n = icp(points(last), points(first))
    yaw = math.degrees(math.atan2(R[1, 0], R[0, 0]))
    print(f'{len(scans)} scans over {dur:.1f} s; ICP inliers {n_in}/{n}')
    print(f'dx {t[0] * 100:+.1f} cm  dy {t[1] * 100:+.1f} cm  |d| {np.hypot(*t) * 100:.1f} cm  dyaw {yaw:+.1f} deg')


if __name__ == '__main__':
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    main(sys.argv[1])
