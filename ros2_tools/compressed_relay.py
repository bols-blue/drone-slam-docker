#!/usr/bin/env python3
"""CompressedImage を自前で展開して bgr8 の Image として出し直す (RViz2 表示用)。

usb_cam を pixel_format: yuyv で使って記録した圧縮画像は format が
"yuv422_yuy2; jpeg compressed bgr8" になり、compressed_image_transport が展開後の画像に
yuv422_yuy2 のエンコーディングを付けてしまうため RViz2 で表示できない。中身は普通の JPEG なので
OpenCV で展開して bgr8 として publish し直す。
"""
import sys

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CompressedImage, Image


class Relay(Node):
    def __init__(self, src, dst):
        super().__init__("compressed_relay")
        self.pub = self.create_publisher(Image, dst, 5)
        self.create_subscription(CompressedImage, src, self.cb, 5)
        self.get_logger().info("%s -> %s (bgr8)" % (src, dst))

    def cb(self, m):
        img = cv2.imdecode(np.frombuffer(bytes(m.data), np.uint8), cv2.IMREAD_COLOR)
        if img is None:
            self.get_logger().warn("画像を展開できませんでした (format=%s)" % m.format)
            return
        out = Image()
        out.header = m.header
        out.height, out.width = img.shape[:2]
        out.encoding = "bgr8"
        out.step = out.width * 3
        out.data = img.tobytes()
        self.pub.publish(out)


def main():
    rclpy.init()
    rclpy.spin(Relay(sys.argv[1], sys.argv[2]))


if __name__ == "__main__":
    main()
