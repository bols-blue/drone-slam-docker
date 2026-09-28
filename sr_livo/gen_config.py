#!/usr/bin/env python3
"""共通 calib.yaml から SR-LIVO の rosparam yaml を生成する。

SR-LIVO の外部パラメータ規約 (src/lioOptimization.cpp / rgbMapTracker.cpp で確認):
  extrinsic_R/t_imu_lidar  : p_imu = R * p_lidar + t            -> calib の T_imu_lidar
  extrinsic_R/t_imu_camera : カメラ姿勢の IMU 座標系表現 (q_world_camera = R_world_imu * R_imu_camera,
                             t_world_camera = R_world_imu * t_imu_camera + t_world_imu) -> T_imu_cam
calib.yaml に `sr_livo:` セクションがあれば、生成結果に上書きマージする (任意のパラメータ調整用)。
"""
import argparse
import sys

sys.path.insert(0, "/opt/drone_slam/lib")
import calib as C  # noqa: E402


def deep_merge(base, over):
    for k, v in over.items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_merge(base[k], v)
        else:
            base[k] = v
    return base


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--output-path", required=True)
    a = ap.parse_args()

    c = C.load(a.calib)
    cam, imu, lidar, topics = c["camera"], c["imu"], c["lidar"], c["topics"]
    K = cam["K"]
    cfg = {
        "debug_output": False,
        "output_path": a.output_path,
        "common": {
            "lidar_topic": topics["lidar"],
            "imu_topic": topics["imu"],
            "image_topic": topics["image"],
            "image_type": "RGB8",  # sensor_msgs/Image (tools で bgr8 に変換済み)。CompressedImage なら COMPRESSED
            "point_filter_num": 3,
            "time_sync_en": False,
            "gravity_acc": [0.0, 0.0, 9.81],
        },
        "lidar_parameter": {
            "lidar_type": 1,  # 1: Livox CustomMsg
            "N_SCANS": 4,     # Mid-360S は 4 ライン
            "SCAN_RATE": 10,
            "time_unit": 3,
            "blind": float(lidar.get("blind", 0.5)),
            "fov_degree": 360,
            "det_range": float(lidar.get("det_range", 100.0)),
        },
        "imu_parameter": {
            "acc_cov": float(imu.get("acc_noise", 0.1)),
            "gyr_cov": float(imu.get("gyr_noise", 0.1)),
            "b_acc_cov": float(imu.get("acc_bias_noise", 0.0001)),
            "b_gyr_cov": float(imu.get("gyr_bias_noise", 0.0001)),
            "time_diff_enable": False,
            "acc_scale": 9.81 if imu.get("acc_in_g", True) else 1.0,  # パッチで追加したパラメータ
        },
        "camera_parameter": {
            "image_width": int(cam["width"]),
            "image_height": int(cam["height"]),
            "camera_intrinsic": [float(v) for v in K.reshape(-1)],
            "camera_dist_coeffs": [float(v) for v in cam["D"]],
            "time_offset": float(cam.get("time_offset", 0.0)),  # パッチで追加したパラメータ
        },
        "extrinsic_parameter": {
            "extrinsic_enable": False,
            "extrinsic_t_imu_lidar": C.t_list(c["T_imu_lidar"]),
            "extrinsic_R_imu_lidar": C.R_list(c["T_imu_lidar"]),
            "extrinsic_t_imu_camera": C.t_list(c["T_imu_cam"]),
            "extrinsic_R_imu_camera": C.R_list(c["T_imu_cam"]),
        },
        # 以下は SR-LIVO 同梱 config/r3live.yaml (Livox Avia) の値
        "odometry_options": {
            "voxel_size": 0.1,
            "sample_voxel_size": 1.5,
            "max_distance": 2000.0,
            "max_num_points_in_voxel": 20,
            "init_num_frames": 20,
            "min_distance_points": 0.15,
            "distance_error_threshold": 100.0,
            "motion_compensation": "CONSTANT_VELOCITY",
            "initialization": "INIT_IMU",
        },
        "icp_options": {
            "size_voxel_map": 1.0,
            "num_iters_icp": 5,
            "min_number_neighbors": 20,
            "voxel_neighborhood": 1,
            "max_number_neighbors": 20,
            "max_dist_to_plane_ct_icp": 0.3,
            "threshold_orientation_norm": 0.1,
            "threshold_translation_norm": 0.01,
            "debug_print": False,
            "num_closest_neighbors": 1,
            "min_num_residuals": 200,
            "max_num_residuals": 600,
        },
        "map_options": {
            "size_voxel_map": 0.1,
            "max_num_points_in_voxel": 50,
            "min_distance_points": 0.01,
            "add_point_step": 1,
            "pub_point_minimum_views": 1,
        },
    }
    deep_merge(cfg, c.get("sr_livo") or {})
    C.dump_yaml(cfg, a.out)
    print("[sr_livo] generated", a.out)


if __name__ == "__main__":
    main()
