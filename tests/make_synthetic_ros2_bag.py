#!/usr/bin/env python3
"""変換ツールのテスト用に、drone_slam (ROS2 Jazzy) と同じトピック構成の小さな mcap bag を生成する。"""
import sys
from pathlib import Path

import numpy as np
from rosbags.rosbag2 import StoragePlugin, Writer
from rosbags.typesys import Stores, get_typestore

from importlib.machinery import SourceFileLoader
from importlib.util import module_from_spec, spec_from_loader

_loader = SourceFileLoader("conv", "/usr/local/bin/ros2bag-to-ros1")
conv = module_from_spec(spec_from_loader("conv", _loader))
_loader.exec_module(conv)

ts = get_typestore(Stores.ROS2_JAZZY)
ts.register(conv.livox_types("livox_ros_driver2"))
T = ts.types
Header, Time = T["std_msgs/msg/Header"], T["builtin_interfaces/msg/Time"]


def stamp(ns):
    return Time(sec=ns // 10**9, nanosec=ns % 10**9)


out = Path(sys.argv[1])
host0 = 1_790_000_000 * 10**9  # PC 時刻
lidar0 = 100 * 10**9            # Mid-360S 起動からの経過時間 (PTP 無し)
with Writer(out, version=8, storage_plugin=StoragePlugin.MCAP) as w:
    c_l = w.add_connection("/livox/lidar", "livox_ros_driver2/msg/CustomMsg", typestore=ts)
    c_i = w.add_connection("/livox/imu", "sensor_msgs/msg/Imu", typestore=ts)
    c_c = w.add_connection("/image_raw", "sensor_msgs/msg/Image", typestore=ts)
    c_f = w.add_connection("/mavros/imu/data", "sensor_msgs/msg/Imu", typestore=ts)
    Pt, Msg, Imu, Img = (T["livox_ros_driver2/msg/CustomPoint"], T["livox_ros_driver2/msg/CustomMsg"],
                         T["sensor_msgs/msg/Imu"], T["sensor_msgs/msg/Image"])
    Q, V3 = T["geometry_msgs/msg/Quaternion"], T["geometry_msgs/msg/Vector3"]
    cov = np.zeros(9)
    for k in range(20):  # 2 秒分
        t = k * 100_000_000
        pts = [Pt(offset_time=i * 1000, x=1.0 + i, y=0.0, z=0.5, reflectivity=10, tag=0, line=i % 4) for i in range(8)]
        m = Msg(header=Header(stamp=stamp(lidar0 + t), frame_id="livox_frame"), timebase=lidar0 + t,
                point_num=len(pts), lidar_id=0, rsvd=np.zeros(3, np.uint8), points=pts)
        w.write(c_l, host0 + t + 5_000_000, ts.serialize_cdr(m, Msg.__msgtype__))
        imu = Imu(header=Header(stamp=stamp(lidar0 + t), frame_id="livox_frame"), orientation=Q(x=0, y=0, z=0, w=1),
                  orientation_covariance=cov, angular_velocity=V3(x=0, y=0, z=0), angular_velocity_covariance=cov,
                  linear_acceleration=V3(x=0, y=0, z=1.0), linear_acceleration_covariance=cov)
        w.write(c_i, host0 + t + 1_000_000, ts.serialize_cdr(imu, Imu.__msgtype__))
        imu.header.stamp = stamp(host0 + t)
        w.write(c_f, host0 + t + 1_000_000, ts.serialize_cdr(imu, Imu.__msgtype__))
        yuyv = np.full((48, 64 * 2), 128, np.uint8)
        img = Img(header=Header(stamp=stamp(host0 + t), frame_id="camera_link"), height=48, width=64,
                  encoding="yuv422_yuy2", is_bigendian=0, step=128, data=yuyv.reshape(-1))
        w.write(c_c, host0 + t + 2_000_000, ts.serialize_cdr(img, Img.__msgtype__))
print("wrote", out)
