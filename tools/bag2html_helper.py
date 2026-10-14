#!/usr/bin/env python3
"""bag2html.py から SLAM イメージ (ROS1 Noetic) の中で呼ばれる補助スクリプト。

  probe   : ROS1 bag の中身を調べ、実行する手法を決め、手法ごとの calib.yaml を書き出す
  odom2tum: nav_msgs/Odometry のトピックを TUM 形式の軌跡に変換する

Python 3.8 / rosbag / PyYAML / numpy が必要 (drone-slam:fast-livo2 イメージに入っている)。
"""
import argparse
import copy
import json
import sys

import numpy as np
import rosbag
import yaml

ALL_ALGOS = ["fast-livo2", "fast-lio2-openmvs", "r3live", "sr-livo", "lvi-sam"]
IMAGE_TYPES = ("sensor_msgs/Image", "sensor_msgs/CompressedImage")


def deep_set(d, keys, value):
    for k in keys[:-1]:
        if not isinstance(d.get(k), dict):
            d[k] = {}
        d = d[k]
    d[keys[-1]] = value


def imu_stats(bag, topic, n=400):
    acc = []
    for _, m, _ in bag.read_messages(topics=[topic]):
        a = m.linear_acceleration
        acc.append((a.x, a.y, a.z))
        if len(acc) >= n:
            break
    if not acc:
        return None
    mean = np.mean(np.array(acc), axis=0)
    return {"mean": [round(float(v), 4) for v in mean], "norm": round(float(np.linalg.norm(mean)), 4)}


def probe(args):
    with open(args.calib) as f:
        base = yaml.safe_load(f)
    topics_cfg = base.setdefault("topics", {})
    bag = rosbag.Bag(args.bag)
    info = bag.get_type_and_topic_info().topics
    types = {t: v.msg_type for t, v in info.items()}
    counts = {t: v.message_count for t, v in info.items()}
    warnings = []
    notes = []

    duration = max(bag.get_end_time() - bag.get_start_time(), 1e-3)
    lidar = topics_cfg.get("lidar", "/livox/lidar")
    imu = topics_cfg.get("imu", "/livox/imu")
    if counts.get(lidar, 0) == 0:
        # Livox (Mid-360S) の /livox/lidar、または tools が Hesai (XT16) から作った <topic>/livox
        cands = sorted((t for t, ty in types.items() if ty == "livox_ros_driver/CustomMsg" and counts[t] > 0),
                       key=lambda t: -counts[t])
        if not cands:
            empty = [t for t, ty in types.items()
                     if ty in ("livox_ros_driver/CustomMsg", "sensor_msgs/PointCloud2") and counts[t] == 0]
            if empty:
                sys.exit("[probe] LiDAR のデータが記録されていません (%s が 0 件)。記録時に LiDAR ドライバが"
                         "動いていたか確認してください" % ", ".join(empty))
            others = [t for t, ty in types.items() if ty == "sensor_msgs/PointCloud2"]
            sys.exit("[probe] LiDAR トピック %s が bag にありません%s" % (
                lidar, " (PointCloud2: %s。Hesai 形式 (x y z intensity ring timestamp) なら tools が自動で "
                       "<topic>/livox に変換します)" % ", ".join(others) if others else ""))
        if lidar in types:
            warnings.append("LiDAR トピック %s は 0 件なので %s を使います" % (lidar, cands[0]))
        else:
            warnings.append("LiDAR トピック %s が無いので %s を使います" % (lidar, cands[0]))
        lidar = topics_cfg["lidar"] = cands[0]
    if counts.get(imu, 0) == 0:
        cands = sorted((t for t, ty in types.items() if ty == "sensor_msgs/Imu" and not t.endswith("_ms2")
                        and counts[t] > 0), key=lambda t: -counts[t])
        if not cands:
            sys.exit("[probe] IMU トピック %s が bag にありません" % imu)
        warnings.append("IMU トピック %s が無いので %s を使います" % (imu, cands[0]))
        imu = topics_cfg["imu"] = cands[0]
    imu_hz = counts[imu] / duration
    if imu_hz < 50:
        sys.exit("[probe] IMU %s が %.1f Hz しかありません。SLAM には 100 Hz 以上必要です"
                 " (ArduPilot なら RAW_IMU の送信レートを上げる。drone_slam の docs/time_sync.md 参照)" % (imu, imu_hz))
    notes.append("LiDAR %s %.1f Hz / IMU %s %.1f Hz" % (lidar, counts[lidar] / duration, imu, imu_hz))

    # 回転式 LiDAR (XT16 等、16 ライン以上) はライン番号が大きいほど上向きのレーザー。
    # ライン番号と点の高さが逆相関なら LiDAR が天地逆に付いている
    lidar_inverted = None
    if int((base.get("lidar") or {}).get("scan_lines", 4)) >= 16:
        mid = counts[lidar] // 2
        for k, (_, m, _) in enumerate(bag.read_messages(topics=[lidar])):
            if k < mid:
                continue
            pts = np.array([(p.z, p.line, p.x * p.x + p.y * p.y + p.z * p.z) for p in m.points])
            pts = pts[pts[:, 2] > 1.0]
            if len(pts) > 100:
                lines = np.unique(pts[:, 1])
                mean_z = [pts[pts[:, 1] == ln, 0].mean() for ln in lines]
                if len(lines) >= 4:
                    lidar_inverted = bool(np.corrcoef(lines, mean_z)[0, 1] < 0)
            break
        if lidar_inverted:
            T = np.array((base.get("T_imu_lidar") or {}).get("R", [1, 0, 0, 0, 1, 0, 0, 0, 1]), dtype=float).reshape(3, 3)
            msg = "LiDAR が天地逆に取り付けられていると判定しました (ライン番号が大きいほど点が低い)"
            if T[2, 2] > 0:
                warnings.append(msg + "。calib.yaml の T_imu_lidar の回転が天地逆になっていません (例: R = diag(1, -1, -1))。"
                                "回転が違うと SLAM は発散します")
            else:
                notes.append(msg)

    # 画像
    image = topics_cfg.get("image", "/image_raw")
    has_image = image in types or (image + "/compressed") in types
    if not has_image:
        cands = [t for t, ty in types.items() if ty in IMAGE_TYPES]
        if cands:
            t = cands[0][: -len("/compressed")] if cands[0].endswith("/compressed") else cands[0]
            warnings.append("画像トピック %s が無いので %s を使います" % (image, t))
            image = topics_cfg["image"] = t
            has_image = True

    # IMU の単位と向き (静止している最初の約 2 秒の平均加速度で判定)
    st = imu_stats(bag, imu)
    acc_in_g = bool((base.get("imu") or {}).get("acc_in_g", True))
    inverted = False
    if st:
        measured_g = st["norm"] < 3.0
        if measured_g != acc_in_g:
            warnings.append("IMU 加速度の大きさが %.2f なので imu.acc_in_g を %s に直します" % (st["norm"], measured_g))
            deep_set(base, ["imu", "acc_in_g"], measured_g)
            acc_in_g = measured_g
        inverted = st["mean"][2] < 0
        if inverted:
            notes.append("IMU の z 軸が下向き (LiDAR が天地逆に取り付けられている) と判定しました")

    fc_imu = topics_cfg.get("fc_imu", "/mavros/imu/data")
    fc_hz = counts.get(fc_imu, 0) / duration
    has_fc_imu = fc_imu in types and fc_imu != imu and fc_hz >= 50
    fc_slow = fc_imu in types and fc_imu != imu and 0 < fc_hz < 50
    has_ms2 = (imu + "_ms2") in types

    main = copy.deepcopy(base)
    if not has_image:
        deep_set(main, ["fast_livo2", "common", "img_en"], 0)

    # 実行する手法と、実行しない理由
    skip = {}
    if not has_image:
        skip["r3live"] = "画像が無いと色付き地図を保存しない (軌跡のみになる)"
        skip["sr-livo"] = "画像が無いと処理が始まらない"
    lvi = None
    if has_fc_imu:
        lvi = copy.deepcopy(base)
        deep_set(lvi, ["lvi_sam", "imu_source"], "fc")
        if "T_fcimu_lidar" not in base:
            warnings.append("calib.yaml に T_fcimu_lidar が無いので LVI-SAM の結果は信頼できません")
    elif has_ms2 and not inverted:
        lvi = copy.deepcopy(base)
        lvi["topics"]["imu"] = lvi["topics"]["fc_imu"] = imu + "_ms2"
        deep_set(lvi, ["imu", "acc_in_g"], False)
    elif has_ms2 and inverted:
        skip["lvi-sam"] = "FC の IMU (%s) が bag に無く、内蔵 6 軸 IMU が天地逆のため発散する" % fc_imu
    elif fc_slow:
        skip["lvi-sam"] = "姿勢付き IMU (%s) が %.1f Hz しかない (100 Hz 以上必要。ArduPilot なら ATTITUDE を上げる)" % (
            fc_imu, fc_hz)
    else:
        skip["lvi-sam"] = "姿勢付き IMU (%s) が bag に無い" % fc_imu

    algos = [a for a in ALL_ALGOS if a not in skip]
    with open(args.out_dir + "/calib_main.yaml", "w") as f:
        yaml.safe_dump(main, f, sort_keys=False, allow_unicode=True)
    if lvi is not None:
        with open(args.out_dir + "/calib_lvi.yaml", "w") as f:
            yaml.safe_dump(lvi, f, sort_keys=False, allow_unicode=True)

    result = {
        "duration_s": round(bag.get_end_time() - bag.get_start_time(), 1),
        "topics": {t: {"type": types[t], "count": counts[t]} for t in sorted(types)},
        "lidar": lidar, "imu": imu, "image": image if has_image else None,
        "has_image": has_image, "has_fc_imu": has_fc_imu, "imu_stats": st, "acc_in_g": acc_in_g,
        "inverted": inverted, "algos": algos, "skip": skip, "warnings": warnings, "notes": notes,
    }
    with open(args.out_dir + "/probe.json", "w") as f:
        json.dump(result, f, ensure_ascii=False, indent=1)
    print(json.dumps(result, ensure_ascii=False))


def odom2tum(args):
    bag = rosbag.Bag(args.bag)
    n = 0
    with open(args.out, "w") as f:
        for _, m, _ in bag.read_messages(topics=[args.topic]):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            f.write("%.6f %f %f %f %f %f %f %f\n" % (m.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w))
            n += 1
    print("[odom2tum] %d poses -> %s" % (n, args.out))


def main():
    ap = argparse.ArgumentParser()
    sub = ap.add_subparsers(dest="cmd")
    p = sub.add_parser("probe")
    p.add_argument("--bag", required=True)
    p.add_argument("--calib", required=True)
    p.add_argument("--out-dir", required=True)
    o = sub.add_parser("odom2tum")
    o.add_argument("--bag", required=True)
    o.add_argument("--topic", default="/Odometry")
    o.add_argument("--out", required=True)
    args = ap.parse_args()
    if args.cmd == "probe":
        probe(args)
    elif args.cmd == "odom2tum":
        odom2tum(args)
    else:
        ap.print_help()


if __name__ == "__main__":
    main()
