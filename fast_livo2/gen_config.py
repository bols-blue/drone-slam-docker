#!/usr/bin/env python3
"""共通 calib.yaml から FAST-LIVO2 用の設定 (fast_livo2.yaml / camera.yaml) を生成する。

FAST-LIVO2 の外部パラメータの意味 (上流ソース src/LIVMapper.cpp, src/vio.cpp で確認):
  extrinsic_R / extrinsic_T : LiDAR -> IMU  (p_imu = R * p_lidar + T)   = calib の T_imu_lidar
  Rcl / Pcl                 : LiDAR -> カメラ (p_cam = Rcl * p_lidar + Pcl) = calib の T_cam_lidar
  img_time_offset           : 画像時刻に加算される (t = header + offset)。calib の camera.time_offset は
                              「カメラ時刻 = LiDAR時刻 + offset」なので符号を反転して渡す
  IMU 加速度は初期化時の平均ノルムで 9.81 に正規化されるため、Mid-360S の g 単位のままでよい
カメラは vikit の Pinhole (歪み d0..d3 = k1, k2, p1, p2。k3 は使われない)。
"""
import argparse
import os
import sys

import yaml

sys.path.insert(0, "/opt/drone_slam/lib")
import calib as C  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))


def deep_merge(base, override):
    for k, v in (override or {}).items():
        if isinstance(v, dict) and isinstance(base.get(k), dict):
            deep_merge(base[k], v)
        else:
            base[k] = v
    return base


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--calib", required=True)
    ap.add_argument("--out", required=True)
    ap.add_argument("--template", default=os.path.join(HERE, "fast_livo2_template.yaml"))
    a = ap.parse_args()

    c = C.load(a.calib)
    with open(a.template) as f:
        p = yaml.safe_load(f)
    with open(a.calib) as f:
        raw = yaml.safe_load(f)

    topics = c.get("topics", {})
    p["common"]["lid_topic"] = topics.get("lidar", "/livox/lidar")
    p["common"]["imu_topic"] = topics.get("imu", "/livox/imu")
    p["common"]["img_topic"] = topics.get("image", "/image_raw")

    p["extrin_calib"]["extrinsic_R"] = C.R_list(c["T_imu_lidar"])
    p["extrin_calib"]["extrinsic_T"] = C.t_list(c["T_imu_lidar"])
    p["extrin_calib"]["Rcl"] = C.R_list(c["T_cam_lidar"])
    p["extrin_calib"]["Pcl"] = C.t_list(c["T_cam_lidar"])

    cam = c["camera"]
    p["time_offset"]["img_time_offset"] = -float(cam.get("time_offset", 0.0))

    lidar = c.get("lidar", {})
    p["preprocess"]["scan_line"] = int(lidar.get("scan_lines", 4))  # Mid-360S は 4
    if "blind" in lidar:
        p["preprocess"]["blind"] = float(lidar["blind"])
    imu = c.get("imu", {})
    if "acc_noise" in imu:
        p["imu"]["acc_cov"] = float(imu["acc_noise"])
    if "gyr_noise" in imu:
        p["imu"]["gyr_cov"] = float(imu["gyr_noise"])

    camp = p.pop("camera")
    d = [float(v) for v in cam.get("distortion", [0, 0, 0, 0, 0])] + [0.0] * 5
    if abs(d[4]) > 1e-9:
        print("[gen_config] WARNING: FAST-LIVO2 の Pinhole モデルは k3 を使いません (k3=%g は無視)" % d[4],
              file=sys.stderr)
    camp.update({
        "cam_width": int(cam["width"]), "cam_height": int(cam["height"]),
        "cam_fx": float(cam["fx"]), "cam_fy": float(cam["fy"]),
        "cam_cx": float(cam["cx"]), "cam_cy": float(cam["cy"]),
        "cam_d0": d[0], "cam_d1": d[1], "cam_d2": d[2], "cam_d3": d[3],
    })

    # calib.yaml の fast_livo2: セクションで任意パラメータを上書き (camera: はカメラ側)
    ov = dict(raw.get("fast_livo2") or {})
    deep_merge(camp, ov.pop("camera", None))
    deep_merge(p, ov)

    os.makedirs(a.out, exist_ok=True)
    C.dump_yaml(p, os.path.join(a.out, "fast_livo2.yaml"))
    C.dump_yaml(camp, os.path.join(a.out, "camera.yaml"))
    print("[gen_config] wrote %s/fast_livo2.yaml, camera.yaml" % a.out)


if __name__ == "__main__":
    main()
