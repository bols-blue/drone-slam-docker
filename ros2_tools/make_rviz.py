#!/usr/bin/env python3
"""view_bag_images.sh 用の RViz2 設定を作る。

  make_rviz.py --fixed base_link [--image /bag_view/image] [--livox /bag_view/livox_points] [--cloud /lidar_points ...]

3D ビューに LiDAR 点群 (Mid-360S は反射強度で着色、XT16 などその他の PointCloud2 は単色) と
base_link の座標軸を、左側のパネルにカメラ画像を表示する。
"""
import argparse

import yaml

CLOUD_COLORS = ["255; 140; 0", "0; 200; 255", "255; 80; 200", "120; 255; 120"]


def qos(topic):
    return {"Depth": 5, "Durability Policy": "Volatile", "History Policy": "Keep Last",
            "Reliability Policy": "Reliable", "Value": topic}


def cloud(name, topic, decay, color=None):
    d = {
        "Class": "rviz_default_plugins/PointCloud2", "Enabled": True, "Name": name, "Topic": qos(topic),
        "Style": "Points", "Size (Pixels)": 2, "Decay Time": decay, "Position Transformer": "XYZ",
        "Selectable": False, "Use Fixed Frame": True,
    }
    if color:
        d.update({"Color Transformer": "FlatColor", "Color": color})
    else:
        d.update({"Color Transformer": "Intensity", "Channel Name": "intensity", "Use rainbow": True,
                  "Autocompute Intensity Bounds": True})
    return d


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fixed", default="base_link")
    ap.add_argument("--image")
    ap.add_argument("--livox")
    ap.add_argument("--cloud", action="append", default=[])
    ap.add_argument("--decay", type=float, default=0.5, help="点群を残す秒数 (Mid-360S は 0.5〜1 秒で見やすい)")
    a = ap.parse_args()

    displays = [
        {"Class": "rviz_default_plugins/Grid", "Enabled": True, "Name": "Grid", "Cell Size": 1, "Plane Cell Count": 40,
         "Reference Frame": "<Fixed Frame>", "Color": "100; 100; 100"},
        {"Class": "rviz_default_plugins/TF", "Enabled": True, "Name": "TF", "Marker Scale": 0.5, "Show Names": True,
         "Show Axes": True, "Show Arrows": False},
    ]
    if a.livox:
        displays.append(cloud("Mid-360S", a.livox, a.decay))
    for i, t in enumerate(a.cloud):
        # LiDAR が 1 台だけなら反射強度で着色、Mid-360S と並べるときは見分けやすいよう単色
        color = CLOUD_COLORS[i % len(CLOUD_COLORS)] if (a.livox or i > 0) else None
        displays.append(cloud(("XT16 " if "lidar_points" in t or "xt16" in t else "Cloud ") + t, t, 0.0, color))
    if a.image:
        displays.append({"Class": "rviz_default_plugins/Image", "Enabled": True, "Name": "Camera",
                         "Normalize Range": True, "Topic": qos(a.image)})

    cfg = {
        "Panels": [{"Class": "rviz_common/Displays", "Name": "Displays"}],
        "Visualization Manager": {
            "Class": "", "Enabled": True, "Name": "root", "Displays": displays,
            "Global Options": {"Background Color": "30; 30; 30", "Fixed Frame": a.fixed, "Frame Rate": 30},
            "Tools": [{"Class": "rviz_default_plugins/MoveCamera"}, {"Class": "rviz_default_plugins/Select"}],
            "Value": True,
            "Views": {"Current": {"Class": "rviz_default_plugins/Orbit", "Distance": 25, "Pitch": 0.6,
                                  "Yaw": 3.6, "Focal Point": {"X": 0, "Y": 0, "Z": 0}}, "Saved": None},
        },
        "Window Geometry": {"Height": 1000, "Width": 1600, "Camera": {"collapsed": False},
                            "Displays": {"collapsed": False}},
    }
    print(yaml.safe_dump(cfg, sort_keys=False, allow_unicode=True))


if __name__ == "__main__":
    main()
