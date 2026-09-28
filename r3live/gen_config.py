#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""共通 calib.yaml から R3LIVE 用の設定 (rosparam yaml) を生成する。

R3LIVE の外部パラメータの意味 (r3live_vio.cpp set_image_pose を参照):
  camera_ext_R / camera_ext_t = カメラの IMU 座標系での姿勢 (T_imu_cam)。p_imu = R * p_cam + t
LiDAR-IMU: R3LIVE は回転を単位行列と仮定し、並進 (Lidar_offset_to_IMU) のみ扱う。
  p_imu = p_lidar + lidar_offset_to_imu  (= T_imu_lidar の t)
calib.yaml に `r3live:` セクションがあれば、生成結果に再帰的に上書きマージする
(例: r3live: {r3live_common: {estimate_intrinsic: 0}})。
"""
import argparse
import sys

import numpy as np

sys.path.insert(0, "/opt/drone_slam/lib")
import calib as C  # noqa: E402


def merge(dst, src):
    for k, v in src.items():
        if isinstance(v, dict) and isinstance(dst.get(k), dict):
            merge(dst[k], v)
        else:
            dst[k] = v
    return dst


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--map-dir", required=True)
    args = ap.parse_args()

    c = C.load(args.calib)
    cam = c["camera"]
    T_imu_cam = c["T_imu_cam"]
    T_imu_lidar = c["T_imu_lidar"]
    if not np.allclose(T_imu_lidar[:3, :3], np.eye(3), atol=1e-6):
        print("[r3live] WARNING: R3LIVE は LiDAR-IMU 間の回転を扱えません (単位行列と仮定)。"
              "T_imu_lidar.R は無視されます", file=sys.stderr)
    lidar = c.get("lidar", {})
    imu = c.get("imu", {})

    cfg = {
        "Lidar_front_end": {
            "lidar_type": 1,        # 1: livox_ros_driver/CustomMsg (horizon_handler, Avia/Mid-360(S) 共通)
            "N_SCANS": int(lidar.get("scan_lines", 4)),  # Mid-360S は 4 ライン (Avia は 6)
            "using_raw_point": 1,
            "point_step": 1,
            "blind": float(lidar.get("blind", 0.5)),
        },
        "r3live_common": {
            "map_output_dir": args.map_dir,
            "if_dump_log": 0,
            "record_offline_map": 1,          # r3live_meshing (テクスチャメッシュ) に必要
            "pub_pt_minimum_views": 3,
            "minimum_pts_size": 0.01,
            "image_downsample_ratio": 1,
            "estimate_i2c_extrinsic": 1,      # カメラ-IMU 外部パラメータのオンライン推定
            "estimate_intrinsic": 1,          # 内部パラメータのオンライン推定
            "maximum_vio_tracked_pts": 600,
            "append_global_map_point_step": 4,
        },
        "r3live_vio": {
            "image_width": int(cam["width"]),
            "image_height": int(cam["height"]),
            "camera_intrinsic": [float(v) for v in cam["K"].reshape(-1)],
            "camera_dist_coeffs": [float(v) for v in cam["D"][:5]] + [0.0] * (5 - len(cam["D"][:5])),
            "camera_ext_R": C.R_list(T_imu_cam),
            "camera_ext_t": C.t_list(T_imu_cam),
            # R3LIVE: t_lidar = t_camera + td。calib の time_offset は t_camera = t_lidar + offset
            "camera_time_offset_td": -float(cam.get("time_offset", 0.0)),
        },
        "r3live_lio": {
            "lio_update_point_step": 4,
            "max_iteration": 2,
            "lidar_time_delay": 0,
            "filter_size_corner": 0.30,
            "filter_size_surf": 0.30,
            "filter_size_surf_z": 0.30,
            "filter_size_map": 0.30,
            "fov_degree": 360.0,
            "long_rang_pt_dis": float(lidar.get("det_range", 100.0)),
            "acc_mul_G": 1 if imu.get("acc_in_g", True) else 0,
            "gravity_align": True,
            "lidar_offset_to_imu": C.t_list(T_imu_lidar),
        },
    }
    if isinstance(c.get("r3live"), dict):
        merge(cfg, c["r3live"])
    C.dump_yaml(cfg, args.out)
    print("[r3live] generated", args.out)


if __name__ == "__main__":
    main()
