#!/usr/bin/env python3
"""calib.yaml (共通) から FAST-LIO2 のパラメータ yaml を生成する。

FAST-LIO の mapping/extrinsic_T, extrinsic_R は「IMU 座標系から見た LiDAR の位置姿勢」
(laserMapping.cpp の Lidar_T_wrt_IMU / Lidar_R_wrt_IMU、p_imu = R * p_lidar + t) なので
calib.yaml の T_imu_lidar をそのまま使う。
Mid-360S の IMU 加速度は g 単位だが、FAST-LIO は初期化時に加速度ノルムで正規化するので追加設定は不要。
"""
import argparse

import calib as C

ap = argparse.ArgumentParser()
ap.add_argument("--calib", required=True)
ap.add_argument("--out", required=True)
args = ap.parse_args()

c = C.load(args.calib)
T = c["T_imu_lidar"]
imu = c.get("imu", {})
lidar = c.get("lidar", {})
cfg = {
    "common": {
        "lid_topic": c["topics"]["lidar"],
        "imu_topic": c["topics"]["imu"],
        "time_sync_en": False,
        "time_offset_lidar_to_imu": 0.0,
    },
    "preprocess": {
        "lidar_type": 1,  # Livox CustomMsg
        "scan_line": int(lidar.get("scan_lines", lidar.get("scan_line", 4))),  # Mid-360S は 4 ライン (Avia は 6)
        "blind": float(lidar.get("blind", 0.5)),
        "timestamp_unit": 3,  # Livox では未使用 (offset_time は ns 固定)
    },
    "mapping": {
        "acc_cov": float(imu.get("acc_noise", 0.1)),
        "gyr_cov": float(imu.get("gyr_noise", 0.1)),
        "b_acc_cov": float(imu.get("acc_bias_noise", 0.0001)),
        "b_gyr_cov": float(imu.get("gyr_bias_noise", 0.0001)),
        "fov_degree": 360.0,
        "det_range": float(lidar.get("det_range", 100.0)),
        "extrinsic_est_en": False,
        "extrinsic_T": C.t_list(T),
        "extrinsic_R": C.R_list(T),
    },
    "publish": {
        "path_en": True,
        "scan_publish_en": True,
        "dense_publish_en": True,
        "scan_bodyframe_pub_en": False,
    },
    "pcd_save": {"pcd_save_en": True, "interval": -1},
    # mapping_mid360.launch と同じ値
    "feature_extract_enable": False,
    "point_filter_num": 3,
    "max_iteration": 3,
    "filter_size_surf": 0.5,
    "filter_size_map": 0.5,
    "cube_side_length": 1000.0,
    "runtime_pos_log_enable": False,
}
C.dump_yaml(cfg, args.out)
print("[gen_config] wrote", args.out)
