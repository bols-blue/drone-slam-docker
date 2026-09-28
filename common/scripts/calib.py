# -*- coding: utf-8 -*-
"""共通キャリブレーションファイル (calib.yaml) の読み込みユーティリティ。

各イメージの gen_config.py から使う。Python 3.6 (melodic) でも動くように書くこと。
座標変換は 4x4 同次変換行列 T_a_b (b 座標系の点を a 座標系に写す: p_a = T_a_b * p_b) で扱う。
"""
import copy

import numpy as np
import yaml


def _to_T(d):
    T = np.eye(4)
    T[:3, :3] = np.array(d["R"], dtype=float).reshape(3, 3)
    T[:3, 3] = np.array(d["t"], dtype=float).reshape(3)
    return T


def inv(T):
    Ti = np.eye(4)
    Ti[:3, :3] = T[:3, :3].T
    Ti[:3, 3] = -T[:3, :3].T.dot(T[:3, 3])
    return Ti


def load(path):
    with open(path) as f:
        c = yaml.safe_load(f)
    c = copy.deepcopy(c)
    c["T_imu_lidar"] = _to_T(c["T_imu_lidar"])
    if "T_cam_lidar" in c:
        c["T_cam_lidar"] = _to_T(c["T_cam_lidar"])
    elif "T_lidar_cam" in c:
        c["T_cam_lidar"] = inv(_to_T(c["T_lidar_cam"]))
    else:
        raise ValueError("calib.yaml に T_cam_lidar (または T_lidar_cam) がありません")
    # 派生量
    c["T_lidar_imu"] = inv(c["T_imu_lidar"])
    c["T_lidar_cam"] = inv(c["T_cam_lidar"])
    c["T_cam_imu"] = c["T_cam_lidar"].dot(c["T_lidar_imu"])
    c["T_imu_cam"] = inv(c["T_cam_imu"])
    cam = c["camera"]
    cam["K"] = np.array([[cam["fx"], 0, cam["cx"]], [0, cam["fy"], cam["cy"]], [0, 0, 1]], dtype=float)
    cam["D"] = np.array(cam.get("distortion", [0, 0, 0, 0, 0]), dtype=float)
    return c


def R_list(T):
    """3x3 回転を行優先のフラットリストで返す (yaml 出力用)"""
    return [float(v) for v in T[:3, :3].reshape(-1)]


def t_list(T):
    return [float(v) for v in T[:3, 3]]


def dump_yaml(data, path):
    """flow style 付きで yaml を書き出す (リストは [a, b, c] 形式)"""
    class _Dumper(yaml.SafeDumper):
        pass

    def _list_rep(dumper, value):
        return dumper.represent_sequence("tag:yaml.org,2002:seq", value, flow_style=True)

    _Dumper.add_representer(list, _list_rep)
    with open(path, "w") as f:
        yaml.dump(data, f, Dumper=_Dumper, default_flow_style=False, sort_keys=False, allow_unicode=True)
