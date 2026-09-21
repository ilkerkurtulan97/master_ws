#!/usr/bin/env python3
"""
Capture a static reference PCD from the live lidar topic.
Run with Isaac Sim playing and NO dynamic objects in the scene.
The robot should stay still (or move slowly while recording accumulates).

Usage:
  source /home/ilker/master_ws/install/setup.bash
  python3 capture_static_pcd.py --duration 15 --out /path/to/output.pcd
"""
import argparse
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
import numpy as np
import struct
import time


def write_pcd(path: str, points: np.ndarray):
    n = len(points)
    header = (
        f"# .PCD v0.7\nFIELDS x y z\nSIZE 4 4 4\nTYPE F F F\nCOUNT 1 1 1\n"
        f"WIDTH {n}\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS {n}\nDATA binary\n"
    )
    with open(path, 'wb') as f:
        f.write(header.encode())
        f.write(points.astype(np.float32).tobytes())
    print(f"Wrote {n:,} points → {path}")


class PcdCapture(Node):
    def __init__(self, topic: str, duration: float, out: str):
        super().__init__('pcd_capture')
        self.duration = duration
        self.out = out
        self.clouds = []
        self.start = time.time()
        self.done = False
        self.sub = self.create_subscription(PointCloud2, topic, self.cb, 10)
        self.get_logger().info(f"Recording {topic} for {duration}s → {out}")

    def cb(self, msg: PointCloud2):
        if self.done:
            return
        elapsed = time.time() - self.start
        pts = np.array([[p[0], p[1], p[2]]
                        for p in pc2.read_points(msg, field_names=('x', 'y', 'z'),
                                                 skip_nans=True)], dtype=np.float32)
        if len(pts):
            self.clouds.append(pts)
        self.get_logger().info(f"  {elapsed:.1f}s — {len(pts)} pts this scan, "
                               f"total scans: {len(self.clouds)}", throttle_duration_sec=2.0)
        if elapsed >= self.duration:
            self.done = True
            merged = np.concatenate(self.clouds, axis=0)
            # Voxel downsample to 0.05m to remove duplicates while keeping density
            from collections import defaultdict
            vox = 0.05
            grid = {}
            for p in merged:
                key = (int(p[0]/vox), int(p[1]/vox), int(p[2]/vox))
                grid[key] = p
            deduped = np.array(list(grid.values()), dtype=np.float32)
            self.get_logger().info(f"Merged {len(merged):,} → {len(deduped):,} pts after dedup")
            write_pcd(self.out, deduped)
            raise SystemExit(0)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument('--topic', default='/front_3d_lidar/lidar_points')
    parser.add_argument('--duration', type=float, default=15.0,
                        help='Seconds to record (default 15)')
    parser.add_argument('--out', default='/home/ilker/master_ws/src/navmesh_planner/data/blender.pcd')
    args = parser.parse_args()

    rclpy.init()
    node = PcdCapture(args.topic, args.duration, args.out)
    try:
        rclpy.spin(node)
    except SystemExit:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
