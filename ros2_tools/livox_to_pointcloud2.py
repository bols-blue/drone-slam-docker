#!/usr/bin/env python3
"""livox_ros_driver2/CustomMsg を sensor_msgs/PointCloud2 (x y z intensity) に変換する (RViz2 表示用)。

  python3 livox_to_pointcloud2.py <入力トピック> <出力トピック>
"""
import sys

import numpy as np
import rclpy
from livox_ros_driver2.msg import CustomMsg
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2, PointField

FIELDS = [PointField(name=n, offset=4 * i, datatype=PointField.FLOAT32, count=1)
          for i, n in enumerate(("x", "y", "z", "intensity"))]


class LivoxToPointCloud2(Node):
    def __init__(self, src, dst):
        super().__init__("livox_to_pointcloud2")
        self.pub = self.create_publisher(PointCloud2, dst, 10)
        self.create_subscription(CustomMsg, src, self.cb, 10)
        self.get_logger().info("%s -> %s" % (src, dst))

    def cb(self, msg):
        n = len(msg.points)
        if n == 0:
            return
        a = np.array([(p.x, p.y, p.z, p.reflectivity) for p in msg.points], dtype=np.float32)
        a = a[np.abs(a[:, :3]).sum(1) > 0]  # 測距できなかった (0,0,0) の点を除く
        out = PointCloud2()
        out.header = msg.header
        out.height, out.width = 1, len(a)
        out.fields = FIELDS
        out.is_bigendian = False
        out.point_step = 16
        out.row_step = 16 * len(a)
        out.is_dense = True
        out.data = a.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(LivoxToPointCloud2(sys.argv[1], sys.argv[2]))


if __name__ == "__main__":
    main()
