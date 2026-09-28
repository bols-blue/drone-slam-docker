#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""共通 calib.yaml から LVI-SAM (LVI-SAM-Easyused) 用の設定ファイルを生成する。

出力:
  <out>/params_lidar.yaml   : rosparam 形式 (LIO 側 + 外参 T_imu_lidar)
  <out>/params_camera.yaml  : OpenCV FileStorage 形式 (VINS 側 + 外参 T_imu_cam)

外参の規約 (LVI-SAM-Easyused):
  extrinsicRotation / extrinsicTranslation (params_lidar)  = T_imu_lidar  (p_imu = R p_lidar + t)
  extrinsicRotation / extrinsicTranslation (params_camera) = T_imu_cam    (p_imu = R p_cam + t)
ここでの "imu" は LVI-SAM に入力する 9 軸 IMU (既定: FC の /mavros/imu/data)。
"""
import argparse
import os
import sys

sys.path.insert(0, "/opt/drone_slam/lib")
import calib as C  # noqa: E402


def warn(msg):
    sys.stderr.write("\033[1;33m[lvi_sam gen_config] WARNING: %s\033[0m\n" % msg)


def fmt_list(vals):
    return "[" + ", ".join("%.10g" % v for v in vals) + "]"


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--map-dir", required=True, help="地図 (PCD) の保存先ディレクトリ (絶対パス)")
    args = ap.parse_args()

    c = C.load(args.calib)
    lv = c.get("lvi_sam", {}) or {}  # calib.yaml 内の任意の上書きブロック
    topics = c["topics"]
    cam = c["camera"]
    lidar = c.get("lidar", {})

    # ---- 入力 IMU の選択 ----
    imu_source = os.environ.get("LVI_IMU_SOURCE", lv.get("imu_source", "fc"))
    if imu_source == "fc" and topics.get("fc_imu") == topics["imu"]:
        warn("topics.fc_imu が LiDAR 内蔵IMU (%s) と同じなので livox モードで動かします" % topics["imu"])
        imu_source = "livox"
    has_orientation = True
    if imu_source == "fc":
        imu_topic = topics.get("fc_imu", "/mavros/imu/data")
        if "T_fcimu_lidar" in c:
            T_imu_lidar = C._to_T(c["T_fcimu_lidar"])
        else:
            warn("calib.yaml に T_fcimu_lidar がありません。Mid-360S 内蔵IMUの T_imu_lidar で代用します。"
                 "FC と LiDAR の取り付け姿勢が違う場合、LVI-SAM は発散します。必ず FC IMU-LiDAR 外参を設定してください。")
            T_imu_lidar = c["T_imu_lidar"]
        imu_noise = c.get("fc_imu_noise", {}) or {}
    elif imu_source == "livox":
        imu_topic = topics["imu"]
        T_imu_lidar = c["T_imu_lidar"]
        imu_noise = {}
        has_orientation = False
        warn("LiDAR 内蔵の 6 軸 IMU (姿勢なし) を入力します。姿勢は単位クォータニオン扱いとし、"
             "IMU 姿勢による roll/pitch 拘束と初期ヨー合わせを無効化します (本来の LVI-SAM の前提外・精度低下)。")
        if (c.get("imu", {}) or {}).get("acc_in_g", False):
            warn("imu.acc_in_g=true ですが LVI-SAM は加速度の単位変換に非対応です (m/s^2 前提)。"
                 "Mid-360S 内蔵IMUでは重力が 1/9.8 に見えて発散します。FC IMU を使ってください。")
    else:
        sys.exit("LVI_IMU_SOURCE は fc または livox を指定してください: %s" % imu_source)

    T_imu_cam = T_imu_lidar.dot(c["T_lidar_cam"])

    # ---- params_lidar.yaml ----
    map_dir = args.map_dir.rstrip("/") + "/"
    p = {
        "pointCloudTopic": os.environ.get("LVI_POINTS_TOPIC", "/livox/points"),
        "imuTopic": imu_topic,
        "odomTopic": "odometry/imu",
        "gpsTopic": "odometry/gpsz",
        "lidarFrame": "base_link",
        "baselinkFrame": "base_link",
        "odometryFrame": "odom",
        "mapFrame": "map",
        # FC の姿勢は ENU 基準なので、初期ヨーも IMU に合わせる (地図が東北上に揃う)
        "useImuHeadingInitialization": has_orientation,
        "useGpsElevation": False,
        "gpsCovThreshold": 2.0,
        "poseCovThreshold": 25.0,
        "savePCD": False,  # 保存は slam-replay 終了時に /lvi_sam/save_map サービスで行う
        "savePCDDirectory": map_dir,
        # --- Livox (Mid-360S: 非反復走査・4ライン / Avia: 6ライン) ---
        "sensor": "livox",
        "N_SCAN": int(lidar.get("scan_lines", 4)),
        # 1ライン・1フレームあたりの最大点数 (200k pts/s / 10Hz / 4 lines = 5000) より大きくする
        "Horizon_SCAN": 6000,
        "timeField": "time",
        "downsampleRate": 1,
        "lidarMinRange": float(lidar.get("blind", 0.5)),
        "lidarMaxRange": float(lidar.get("det_range", 100.0)),
        "imuAccNoise": float(imu_noise.get("acc_noise", 3.9939570888238808e-03)),
        "imuGyrNoise": float(imu_noise.get("gyr_noise", 1.5636343949698187e-03)),
        "imuAccBiasN": float(imu_noise.get("acc_bias_noise", 6.4356659353532566e-05)),
        "imuGyrBiasN": float(imu_noise.get("gyr_bias_noise", 3.5640318696367613e-05)),
        "imuGravity": 9.80511,
        "imuRPYWeight": 0.01 if has_orientation else 0.0,
        "imuHasOrientation": has_orientation,  # patches/0002
        "extrinsicTranslation": C.t_list(T_imu_lidar),
        "extrinsicRotation": C.R_list(T_imu_lidar),
        # MAVROS の姿勢は加速度・角速度と同じ機体座標系 (FLU) で定義される
        "yawAxis": "+z",
        "pitchAxis": "+y",
        "rollAxis": "+x",
        "edgeThreshold": 1.0,
        "surfThreshold": 0.1,
        "edgeFeatureMinValidNum": 10,
        "surfFeatureMinValidNum": 100,
        "odometrySurfLeafSize": 0.4,
        "mappingCornerLeafSize": 0.2,
        "mappingSurfLeafSize": 0.4,
        "z_tollerance": 1000,
        "rotation_tollerance": 1000,
        "numberOfCores": 4,
        "mappingProcessInterval": 0.15,
        "surroundingkeyframeAddingDistThreshold": 1.0,
        "surroundingkeyframeAddingAngleThreshold": 0.2,
        "surroundingKeyframeDensity": 2.0,
        "surroundingKeyframeSearchRadius": 50.0,
        "loopClosureEnableFlag": True,
        "loopClosureFrequency": 1.0,
        "surroundingKeyframeSize": 50,
        "historyKeyframeSearchRadius": 15.0,
        "historyKeyframeSearchTimeDiff": 30.0,
        "historyKeyframeSearchNum": 25,
        "historyKeyframeFitnessScore": 0.3,
        "globalMapVisualizationSearchRadius": 1000.0,
        "globalMapVisualizationPoseDensity": 10.0,
        "globalMapVisualizationLeafSize": 1.0,
    }
    p.update(lv.get("lidar_params", {}) or {})
    C.dump_yaml({"PROJECT_NAME": "lvi_sam", "lvi_sam": p}, os.path.join(args.out, "params_lidar.yaml"))

    # ---- params_camera.yaml (OpenCV FileStorage) ----
    D = [float(v) for v in cam.get("distortion", [0, 0, 0, 0, 0])] + [0.0] * 5
    if abs(D[4]) > 1e-9:
        warn("LVI-SAM の PINHOLE モデルは k3 に非対応です (k3=%g は無視されます)" % D[4])
    # calib: カメラ時刻 = LiDAR時刻 + time_offset / VINS: 画像時刻 + td = IMU時刻
    td = -float(cam.get("time_offset", 0.0))
    vc = {
        "estimate_td": 1,  # USB カメラ等ハードウェア同期なしを想定してオンライン推定
        "rolling_shutter": int(cam.get("rolling_shutter", 0)),
        "rolling_shutter_tr": float(cam.get("rolling_shutter_tr", 0.0)),
        "estimate_extrinsic": 0,
        "max_cnt": 150,
        "min_dist": 20,
        "freq": 20,
        "lidar_skip": 3,
        "loop_closure": 1,
    }
    vc.update(lv.get("camera_params", {}) or {})
    Ric = C.R_list(T_imu_cam)
    tic = C.t_list(T_imu_cam)
    lines = [
        "%YAML:1.0",
        "# drone-slam-docker: calib.yaml から自動生成",
        'project_name: "lvi_sam"',
        'imu_topic: "%s"' % imu_topic,
        'image_topic: "%s"' % topics["image"],
        'point_cloud_topic: "lvi_sam/lidar/deskew/cloud_deskewed"',
        "use_lidar: 1",
        "lidar_skip: %d" % vc["lidar_skip"],
        "align_camera_lidar_estimation: 1",
        # Easyused では未使用 (IF_OFFICIAL=0) だが読み込まれるので 0 を置く
        "lidar_to_cam_tx: 0.0", "lidar_to_cam_ty: 0.0", "lidar_to_cam_tz: 0.0",
        "lidar_to_cam_rx: 0.0", "lidar_to_cam_ry: 0.0", "lidar_to_cam_rz: 0.0",
        "model_type: PINHOLE",
        "camera_name: camera",
        "image_width: %d" % int(cam["width"]),
        "image_height: %d" % int(cam["height"]),
        "distortion_parameters:",
        "   k1: %.10g" % D[0],
        "   k2: %.10g" % D[1],
        "   p1: %.10g" % D[2],
        "   p2: %.10g" % D[3],
        "projection_parameters:",
        "   fx: %.10g" % float(cam["fx"]),
        "   fy: %.10g" % float(cam["fy"]),
        "   cx: %.10g" % float(cam["cx"]),
        "   cy: %.10g" % float(cam["cy"]),
        'fisheye_mask: "/config/fisheye_mask_720x540.jpg"',
        "acc_n: %.10g" % float(imu_noise.get("vins_acc_n", 0.02)),
        "gyr_n: %.10g" % float(imu_noise.get("vins_gyr_n", 0.01)),
        "acc_w: %.10g" % float(imu_noise.get("vins_acc_w", 0.002)),
        "gyr_w: %.10g" % float(imu_noise.get("vins_gyr_w", 4.0e-5)),
        "g_norm: 9.805",
        "estimate_extrinsic: %d" % vc["estimate_extrinsic"],
        "extrinsicRotation: !!opencv-matrix",
        "   rows: 3", "   cols: 3", "   dt: d",
        "   data: " + fmt_list(Ric),
        "extrinsicTranslation: !!opencv-matrix",
        "   rows: 3", "   cols: 1", "   dt: d",
        "   data: " + fmt_list(tic),
        "max_cnt: %d" % vc["max_cnt"],
        "min_dist: %d" % vc["min_dist"],
        "freq: %d" % vc["freq"],
        "F_threshold: 1.0",
        "show_track: 1",
        "equalize: 1",
        "fisheye: 0",
        "max_solver_time: 0.035",
        "max_num_iterations: 10",
        "keyframe_parallax: 10.0",
        "estimate_td: %d" % vc["estimate_td"],
        "td: %.10g" % td,
        "rolling_shutter: %d" % vc["rolling_shutter"],
        "rolling_shutter_tr: %.10g" % vc["rolling_shutter_tr"],
        "loop_closure: %d" % vc["loop_closure"],
        "skip_time: 0.0",
        "skip_dist: 0.0",
        "debug_image: 0",
        "match_image_scale: 0.5",
        # パッケージパスからの相対 (上流の仕様)
        'vocabulary_file: "/config/brief_k10L6.bin"',
        'brief_pattern_file: "/config/brief_pattern.yml"',
    ]
    with open(os.path.join(args.out, "params_camera.yaml"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("[lvi_sam gen_config] imu_source=%s imu_topic=%s" % (imu_source, imu_topic))


if __name__ == "__main__":
    main()
