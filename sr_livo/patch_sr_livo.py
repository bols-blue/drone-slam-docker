#!/usr/bin/env python3
"""SR-LIVO のソースに bag リプレイ用の最小パッチを当てる (Docker ビルド時に実行)。

1. imu_parameter/acc_scale: IMU 加速度に掛ける係数。Mid-360S 内蔵IMUは g 単位なので 9.81 を指定する
   (SR-LIVO は加速度を m/s^2 前提で扱い、スケール補正をしない)
2. camera_parameter/time_offset: 画像時刻を LiDAR 時刻に合わせる補正 [s]
   (t_image_in_lidar_clock = t_image - time_offset)
3. 色付き点が 0 点のとき savePCDFileBinary が例外で abort するのを回避
4. 可視化スレッドが while(1) で終了しないため、SIGINT 後に地図保存しても join でハングする問題を修正
"""
import sys

path = sys.argv[1]
src = open(path).read()


def replace(old, new, count=1):
    global src
    n = src.count(old)
    if n != count:
        sys.exit(f"patch failed: expected {count} occurrence(s), found {n}: {old!r}")
    src = src.replace(old, new)


replace(
    "    sensor_msgs::Imu::Ptr msg_temp(new sensor_msgs::Imu(*msg));\n",
    "    sensor_msgs::Imu::Ptr msg_temp(new sensor_msgs::Imu(*msg));\n"
    "    static const double acc_scale = ros::param::param<double>(\"imu_parameter/acc_scale\", 1.0);\n"
    "    msg_temp->linear_acceleration.x *= acc_scale;\n"
    "    msg_temp->linear_acceleration.y *= acc_scale;\n"
    "    msg_temp->linear_acceleration.z *= acc_scale;\n",
)
replace(
    "    time_img_buffer.push(msg->header.stamp.toSec());\n",
    "    static const double cam_time_offset = ros::param::param<double>(\"camera_parameter/time_offset\", 0.0);\n"
    "    time_img_buffer.push(msg->header.stamp.toSec() - cam_time_offset);\n",
    count=2,
)
replace(
    "    pcl::io::savePCDFileBinary(pcd_path, pcd_rgb);\n",
    "    if (point_count == 0) { std::cout << \"No colored points to save.\" << std::endl; return; }\n"
    "    pcl::io::savePCDFileBinary(pcd_path, pcd_rgb);\n",
)
head, sep, tail = src.partition("void lioOptimization::threadPubColorPoints()")
if not sep:
    sys.exit("patch failed: threadPubColorPoints not found")
tail = tail.replace("    while (1)\n", "    while (ros::ok())\n", 1)
src = head + sep + tail
open(path, "w").write(src)
print("patched", path)
