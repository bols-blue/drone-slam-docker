#!/usr/bin/env python3
"""texture_pipeline.py のテスト用に、合成シーン (市松模様の壁 + 床) の入力一式を生成する。

出力 (OUT_DIR):
  scans.pcd          壁と床の点群 (world 座標系)
  slam_output.bag    /Odometry (IMU 位置姿勢, y 方向に 4 m 移動)
  input.bag          /image_raw (解析的にレンダリングした bgr8 画像)
calib.yaml は同梱テンプレート (/opt/drone_slam/config/calib.yaml) の外部パラメータを使い、
画像サイズだけ小さくしたものを OUT_DIR/calib.yaml に書く。
"""
import os
import sys

import numpy as np
import open3d as o3d
import rosbag
import rospy
import yaml
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image

import calib as C

out = sys.argv[1]
os.makedirs(out, exist_ok=True)
with open("/opt/drone_slam/config/calib.yaml") as f:
    cy = yaml.safe_load(f)
W, H = 320, 180
cy["camera"].update({"width": W, "height": H, "fx": 225.0, "fy": 225.0, "cx": 160.0, "cy": 90.0,
                     "distortion": [0, 0, 0, 0, 0], "time_offset": 0.0})
with open(os.path.join(out, "calib.yaml"), "w") as f:
    yaml.safe_dump(cy, f)
c = C.load(os.path.join(out, "calib.yaml"))
K = c["camera"]["K"]
T_imu_cam = c["T_imu_cam"]

WALL_X, FLOOR_Z = 5.0, -1.5


def wall_color(p):
    """壁: y-z 0.5m 市松 (青/赤, RGB)"""
    k = (np.floor(p[1] / 0.5) + np.floor(p[2] / 0.5)) % 2
    return np.where(k[:, None] == 0, [0, 0, 255], [255, 0, 0])


# 点群
ys, zs = np.meshgrid(np.arange(-4, 8, 0.02), np.arange(FLOOR_Z, 3, 0.02))
wall = np.stack([np.full(ys.size, WALL_X), ys.ravel(), zs.ravel()], 1)
xs, ys2 = np.meshgrid(np.arange(0, WALL_X, 0.02), np.arange(-4, 8, 0.02))
floor = np.stack([xs.ravel(), ys2.ravel(), np.full(xs.size, FLOOR_Z)], 1)
pcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(np.vstack([wall, floor])))
o3d.io.write_point_cloud(os.path.join(out, "scans.pcd"), pcd)

# 軌跡と画像
t0 = 1_790_000_000.0
uu, vv = np.meshgrid(np.arange(W) + 0.5, np.arange(H) + 0.5)
rays_c = np.linalg.inv(K).dot(np.stack([uu.ravel(), vv.ravel(), np.ones(uu.size)]))
with rosbag.Bag(os.path.join(out, "slam_output.bag"), "w") as ob, rosbag.Bag(os.path.join(out, "input.bag"), "w") as ib:
    for k in range(101):
        t = t0 + k * 0.1
        y = 0.04 * k
        yaw = np.radians(10 * np.sin(k / 15.0))
        T_wi = np.eye(4)
        T_wi[:3, :3] = [[np.cos(yaw), -np.sin(yaw), 0], [np.sin(yaw), np.cos(yaw), 0], [0, 0, 1]]
        T_wi[:3, 3] = [0.0, y, 0.0]
        od = Odometry()
        od.header.stamp = rospy.Time.from_sec(t)
        od.header.frame_id = "camera_init"
        od.child_frame_id = "body"
        od.pose.pose.position.x, od.pose.pose.position.y, od.pose.pose.position.z = T_wi[:3, 3]
        od.pose.pose.orientation.z, od.pose.pose.orientation.w = np.sin(yaw / 2), np.cos(yaw / 2)
        ob.write("/Odometry", od, od.header.stamp)
        if k % 2:
            continue
        T_wc = T_wi.dot(T_imu_cam)
        d = T_wc[:3, :3].dot(rays_c)
        o = T_wc[:3, 3:4]
        tw = np.where(d[0] > 1e-6, (WALL_X - o[0]) / np.where(d[0] > 1e-6, d[0], 1), np.inf)
        tf = np.where(d[2] < -1e-6, (FLOOR_Z - o[2]) / np.where(d[2] < -1e-6, d[2], 1), np.inf)
        tt = np.minimum(tw, tf)
        p = o + d * tt
        img = np.zeros((uu.size, 3))
        onwall = (tw <= tf) & np.isfinite(tw)
        onfloor = (tf < tw) & np.isfinite(tf)
        img[onwall] = wall_color(p[:, onwall])
        pf = p[:, onfloor]
        kf = (np.floor(pf[0] / 1.0) + np.floor(pf[1] / 1.0)) % 2
        img[onfloor] = np.where(kf[:, None] == 0, [0, 200, 0], [255, 255, 255])
        im = Image()
        im.header.stamp = rospy.Time.from_sec(t)
        im.header.frame_id = "camera"
        im.height, im.width, im.encoding, im.step = H, W, "bgr8", W * 3
        im.data = img.reshape(H, W, 3)[:, :, ::-1].astype(np.uint8).tobytes()  # RGB->BGR
        ib.write("/image_raw", im, im.header.stamp)
print("wrote", out)
