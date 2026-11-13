#!/usr/bin/env python3
"""Pick training frames from camera bags (dead-chicken detector, plan 8.1).

Each bag is cut into --count equal slots of "novelty":
    novelty = driven distance / --move + turned angle / --turn + time / --still
(distance from /odom/odom_raw, angle from |gyro z| of /imu/data_raw), so a
standing robot yields few frames and a moving / turning one many. From each
slot the sharpest frame is kept. Frames are skipped while turning faster than
--max-turn-rate (the main blur source; images lag the gyro by --lag) or if
less sharp than --min-sharp x the bag median. Sharpness is measured on the
floor (lower part) only: the ceiling lights score high even when blurred.
The original JPEG bytes are written, not re-encoded.

Usage (source ROS 2 Humble first):
  extract_bag_frames.py --out <dir> [--count 100] <bag_dir> [<bag_dir> ...]
Output: <out>/<bag_name>/<bag_name>_<seconds from bag start>.jpg
"""
import argparse
import os
import sys

import cv2
import numpy as np
import rosbag2_py
import yaml
from rclpy.serialization import deserialize_message
from sensor_msgs.msg import CompressedImage, Imu
from std_msgs.msg import Float32MultiArray

IMAGE_TOPIC = '/oak/rgb/image_raw/compressed'
ODOM_TOPIC = '/odom/odom_raw'  # [left_m, right_m], cumulative
IMU_TOPIC = '/imu/data_raw'
FLOOR_FROM = 0.45  # image height fraction where the floor starts
EXPOSURE_WINDOW = (-0.15, 0.1)  # s around the lag-corrected frame time for the turn rate


def open_reader(bag, topics):
    with open(os.path.join(bag, 'metadata.yaml')) as f:
        storage = yaml.safe_load(f)['rosbag2_bagfile_information']['storage_identifier']
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=bag, storage_id=storage),
                rosbag2_py.ConverterOptions('cdr', 'cdr'))
    reader.set_filter(rosbag2_py.StorageFilter(topics=topics))
    return reader


def sharpness(jpeg):
    gray = cv2.imdecode(np.frombuffer(jpeg, np.uint8), cv2.IMREAD_REDUCED_GRAYSCALE_4)
    if gray is None:
        return 0.0
    return cv2.Laplacian(gray[int(FLOOR_FROM * gray.shape[0]):], cv2.CV_64F).var()


def scan(bag):
    """Times [s from bag start]: images + sharpness, odometry distance, gyro |wz|."""
    reader = open_reader(bag, [IMAGE_TOPIC, ODOM_TOPIC, IMU_TOPIC])
    dist = 0.0
    last_odom = t0 = None
    images, odom, imu = [], [], []
    while reader.has_next():
        topic, data, t = reader.read_next()
        t0 = t if t0 is None else t0
        ts = (t - t0) * 1e-9
        if topic == ODOM_TOPIC:
            lr = deserialize_message(data, Float32MultiArray).data
            if last_odom is not None:
                dist += abs((lr[0] - last_odom[0]) + (lr[1] - last_odom[1])) / 2.0
            last_odom = lr
            odom.append((ts, dist))
        elif topic == IMU_TOPIC:
            imu.append((ts, abs(deserialize_message(data, Imu).angular_velocity.z)))
        else:
            images.append((t, ts, sharpness(bytes(deserialize_message(data, CompressedImage).data))))
    return t0, images, np.array(odom).reshape(-1, 2), np.array(imu).reshape(-1, 2)


def frame_quality(ts, sharp, imu, args):
    """(steady, ok) masks: steady = not turning faster than max_turn_rate, ok = steady and sharp."""
    rate = np.zeros(len(ts))
    if len(imu):
        gyro_t = ts - args.lag
        lo = np.searchsorted(imu[:, 0], gyro_t + EXPOSURE_WINDOW[0])
        hi = np.searchsorted(imu[:, 0], gyro_t + EXPOSURE_WINDOW[1], side='right')
        rate = np.array([imu[a:b, 1].max() if b > a else 0.0 for a, b in zip(lo, hi)])
    steady = rate <= args.max_turn_rate
    if not steady.any():
        return steady, steady
    return steady, steady & (sharp >= args.min_sharp * np.median(sharp[steady]))


def select(images, odom, imu, args):
    stamp = np.array([i[0] for i in images])
    ts = np.array([i[1] for i in images])
    sharp = np.array([i[2] for i in images])
    gyro_t = ts - args.lag

    turned = np.concatenate([[0.0], np.cumsum(imu[1:, 1] * np.diff(imu[:, 0]))]) if len(imu) else None
    dist = np.interp(gyro_t, odom[:, 0], odom[:, 1]) if len(odom) else 0.0
    turn = np.interp(gyro_t, imu[:, 0], turned) if len(imu) else 0.0
    novelty = dist / args.move + turn / np.radians(args.turn) + ts / args.still

    steady, ok = frame_quality(ts, sharp, imu, args)
    if not ok.any():
        return set(), int((~steady).sum()), int((steady & ~ok).sum())

    edges = np.linspace(novelty[0], novelty[-1], args.count + 1)
    slot = np.clip(np.searchsorted(edges, novelty, side='right') - 1, 0, args.count - 1)
    chosen, last = set(), -np.inf
    for s in range(args.count):
        idx = np.flatnonzero((slot == s) & ok & (ts >= last + args.min_gap))
        if idx.size:
            best = idx[np.argmax(sharp[idx])]
            chosen.add(stamp[best])
            last = ts[best]
    return chosen, int((~steady).sum()), int((steady & ~ok).sum())


def frame_name(name, t, t0):
    return f'{name}_{(t - t0) * 1e-9:07.2f}.jpg'


def write(bag, name, t0, chosen, out_dir):
    reader = open_reader(bag, [IMAGE_TOPIC])
    while reader.has_next():
        _, data, t = reader.read_next()
        if t in chosen:
            path = os.path.join(out_dir, frame_name(name, t, t0))
            with open(path, 'wb') as f:
                f.write(bytes(deserialize_message(data, CompressedImage).data))


def add_quality_args(ap):
    ap.add_argument('--min-sharp', type=float, default=0.5,
                    help='x median floor sharpness of the steady frames')
    ap.add_argument('--max-turn-rate', type=float, default=0.1, help='rad/s')
    ap.add_argument('--lag', type=float, default=0.3,
                    help='s the image stamps lag the gyro (measured 0.25-0.3 on the 2026-11-11 bags)')


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--out', required=True)
    ap.add_argument('--count', type=int, default=100, help='frames per bag (at most)')
    ap.add_argument('--move', type=float, default=0.10, help='m driven = 1 novelty unit')
    ap.add_argument('--turn', type=float, default=5.0, help='deg turned = 1 novelty unit')
    ap.add_argument('--still', type=float, default=5.0, help='s elapsed = 1 novelty unit')
    ap.add_argument('--min-gap', type=float, default=1.0, help='s between written frames')
    add_quality_args(ap)
    args = ap.parse_args()

    for bag in args.bags:
        bag = bag.rstrip('/')
        name = os.path.basename(bag)
        t0, images, odom, imu = scan(bag)
        if not images:
            print(f'{name}: no {IMAGE_TOPIC}', file=sys.stderr)
            continue
        chosen, turning, blurred = select(images, odom, imu, args)
        out_dir = os.path.join(args.out, name)
        os.makedirs(out_dir, exist_ok=True)
        write(bag, name, t0, chosen, out_dir)
        print(f'{name}: {len(images)} images ({turning} turning, {blurred} blurred) -> '
              f'{len(chosen)} written to {out_dir}')


if __name__ == '__main__':
    main()
