#!/usr/bin/env python3
"""Subscribe to a PointCloud2 topic, grab one message, and print per-field stats.

Usage:
  ros2 run inclio_ros2 inspect_cloud.py              # defaults to /velodyne_points
  ros2 run inclio_ros2 inspect_cloud.py /lidar/points # any PointCloud2 topic

Or simply:
  python3 test/inspect_cloud.py [topic]
"""

import sys
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py.point_cloud2 import read_points


class CloudInspector(Node):
    def __init__(self, topic):
        super().__init__("cloud_inspector")
        self.sub = self.create_subscription(PointCloud2, topic, self.cb, 1)
        self.get_logger().info(f"Waiting for one message on '{topic}' ...")

    def cb(self, msg):
        fields = [f.name for f in msg.fields]
        self.get_logger().info(
            f"stamp={msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}  "
            f"frame={msg.header.frame_id}  "
            f"points={msg.width * msg.height}  "
            f"point_step={msg.point_step}  "
            f"fields={fields}"
        )

        pts = np.array(list(read_points(msg, field_names=fields, skip_nans=False)),
                        dtype=[(f, np.float64) for f in fields])

        for name in fields:
            col = pts[name]
            self.get_logger().info(
                f"  {name:>12s}  min={col.min():12.4f}  max={col.max():12.4f}  "
                f"mean={col.mean():12.4f}  std={col.std():12.4f}"
            )

        raise SystemExit(0)


def main():
    rclpy.init()
    topic = sys.argv[1] if len(sys.argv) > 1 else "/velodyne_points"
    node = CloudInspector(topic)
    rclpy.spin(node)


if __name__ == "__main__":
    main()
