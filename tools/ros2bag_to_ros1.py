#!/usr/bin/env python3
"""ROS2 bag (mcap / sqlite3) を ROS1 bag に変換する。(ROS1 bag を入力して画像展開だけ行うことも可)

drone_slam (ROS2 Jazzy) で記録した bag を、ROS1 実装の SLAM (FAST-LIVO2 / FAST-LIO2 /
R3LIVE / SR-LIVO / LVI-SAM) でリプレイできるようにするためのツール。

主な処理:
  * livox_ros_driver2/msg/CustomMsg -> livox_ros_driver/CustomMsg (フィールドは同一、型名のみ違う)
  * 標準メッセージ (sensor_msgs/Imu 等) は ROS1 形式 (Header.seq 付き) に変換
  * 画像: yuv422 (usb_cam の yuyv) / rgb8 / CompressedImage を bgr8 の sensor_msgs/Image に変換
  * Mid-360 の時刻補正: PTP 同期をしていない Mid-360 はヘッダ時刻が LiDAR 起動からの経過時間になり、
    カメラ (PC 時刻) と数十年ずれる。受信時刻との差の中央値でオフセットを推定し補正する (--livox-time-align)

使用例:
  ros2bag-to-ros1 /data/texture_20260928_101010 /data/texture_20260928_101010.bag
"""
import argparse
import dataclasses
import statistics
import sys
from pathlib import Path

import cv2
import numpy as np
from rosbags.highlevel import AnyReader
from rosbags.rosbag1 import Writer
from rosbags.typesys import Stores, get_types_from_msg, get_typestore

LIVOX_POINT = """
uint32 offset_time
float32 x
float32 y
float32 z
uint8 reflectivity
uint8 tag
uint8 line
"""
LIVOX_MSG = """
std_msgs/Header header
uint64 timebase
uint32 point_num
uint8 lidar_id
uint8[3] rsvd
{pkg}/CustomPoint[] points
"""


def livox_types(pkg):
    types = {}
    types.update(get_types_from_msg(LIVOX_POINT, f"{pkg}/msg/CustomPoint"))
    types.update(get_types_from_msg(LIVOX_MSG.format(pkg=pkg), f"{pkg}/msg/CustomMsg"))
    return types


TYPE_RENAME = {
    "livox_ros_driver2/msg/CustomMsg": "livox_ros_driver/msg/CustomMsg",
    "livox_ros_driver2/msg/CustomPoint": "livox_ros_driver/msg/CustomPoint",
}
LIVOX_ROS1_TYPES = {"livox_ros_driver/msg/CustomMsg"}

YUV_CODES = {
    "yuv422_yuy2": cv2.COLOR_YUV2BGR_YUY2,
    "yuyv": cv2.COLOR_YUV2BGR_YUY2,
    "yuv422": cv2.COLOR_YUV2BGR_UYVY,  # ROS の yuv422 は UYVY
    "uyvy": cv2.COLOR_YUV2BGR_UYVY,
}


def convert_obj(src, dst_cls, ts_out):
    """rosbags の dataclass メッセージを別 typestore のクラスへ再帰的に写す。"""
    kwargs = {}
    for f in dataclasses.fields(dst_cls):
        name = f.name
        if not hasattr(src, name):
            if name == "seq":
                kwargs[name] = 0
                continue
            raise KeyError(f"{dst_cls.__msgtype__}.{name} に対応するフィールドがありません")
        val = getattr(src, name)
        kwargs[name] = convert_value(val, f.type, ts_out)
    return dst_cls(**kwargs)


def _field_msgtype(ftype):
    # rosbags のフィールド型注釈は文字列 (例: "std_msgs__msg__Header", "list[...]") のことがある
    return ftype if isinstance(ftype, str) else getattr(ftype, "__name__", str(ftype))


def convert_value(val, ftype, ts_out):
    if dataclasses.is_dataclass(val):
        name = TYPE_RENAME.get(val.__msgtype__, val.__msgtype__)
        return convert_obj(val, ts_out.types[name], ts_out)
    if isinstance(val, list) and val and dataclasses.is_dataclass(val[0]):
        name = TYPE_RENAME.get(val[0].__msgtype__, val[0].__msgtype__)
        cls = ts_out.types[name]
        return [convert_obj(v, cls, ts_out) for v in val]
    return val


def image_to_bgr8(msg, msgtype):
    """sensor_msgs/Image or CompressedImage -> (bgr ndarray) / None (そのまま通す場合)"""
    if msgtype == "sensor_msgs/msg/CompressedImage":
        img = cv2.imdecode(np.frombuffer(bytes(msg.data), np.uint8), cv2.IMREAD_COLOR)
        return img
    enc = msg.encoding.lower()
    data = np.frombuffer(bytes(msg.data), np.uint8)
    if enc in YUV_CODES:
        yuv = data[: msg.height * msg.step].reshape(msg.height, msg.step)[:, : msg.width * 2]
        return cv2.cvtColor(yuv.reshape(msg.height, msg.width, 2), YUV_CODES[enc])
    if enc == "rgb8":
        rgb = data.reshape(msg.height, msg.step)[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
        return cv2.cvtColor(rgb, cv2.COLOR_RGB2BGR)
    return None  # bgr8 / mono8 等はそのまま


def estimate_livox_offset(reader, conns, n=200):
    diffs = []
    for conn, logtime, raw in reader.messages(connections=conns):
        msg = reader.deserialize(raw, conn.msgtype)
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        diffs.append(logtime - stamp)
        if len(diffs) >= n:
            break
    return int(statistics.median(diffs)) if diffs else 0


def shift_stamp(stamp, offset_ns):
    t = stamp.sec * 1_000_000_000 + stamp.nanosec + offset_ns
    return type(stamp)(sec=t // 1_000_000_000, nanosec=t % 1_000_000_000)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src", help="ROS2 bag ディレクトリ (metadata.yaml を含む) / .mcap / .db3、または ROS1 .bag")
    ap.add_argument("dst", help="出力する ROS1 .bag")
    ap.add_argument("--topics", nargs="*", help="変換するトピック (省略時は変換可能な全トピック)")
    ap.add_argument("--exclude", nargs="*", default=[], help="除外するトピック")
    ap.add_argument("--livox-topics", nargs="*", default=["/livox/lidar", "/livox/imu"],
                    help="Mid-360 の時計で打刻されたトピック (時刻補正の対象)")
    ap.add_argument("--livox-time-align", choices=["auto", "on", "off"], default="auto",
                    help="auto: 受信時刻とのずれが --align-threshold 秒を超えたら補正")
    ap.add_argument("--align-threshold", type=float, default=1.0)
    ap.add_argument("--no-image-convert", action="store_true", help="画像を bgr8 に変換しない")
    ap.add_argument("--image-every", type=int, default=1, help="画像を N 枚に 1 枚だけ残す (間引き)")
    ap.add_argument("--header-time", action="store_true",
                    help="bag の記録時刻ではなくヘッダ時刻を ROS1 bag の時刻に使う (再生順がヘッダ時刻順になる)")
    ap.add_argument("--compress", choices=["none", "bz2", "lz4"], default="none")
    args = ap.parse_args()

    src = Path(args.src)
    dst = Path(args.dst)
    if dst.exists():
        sys.exit(f"出力先が既に存在します: {dst}")

    ts_in = get_typestore(Stores.ROS2_JAZZY)
    ts_in.register(livox_types("livox_ros_driver2"))
    ts_out = get_typestore(Stores.ROS1_NOETIC)
    ts_out.register(livox_types("livox_ros_driver"))

    paths = [src]
    with AnyReader(paths, default_typestore=ts_in) as reader:
        conns = []
        for c in reader.connections:
            if args.topics and c.topic not in args.topics:
                continue
            if c.topic in args.exclude:
                continue
            out_type = TYPE_RENAME.get(c.msgtype, c.msgtype)
            if c.msgtype == "sensor_msgs/msg/CompressedImage" and not args.no_image_convert:
                out_type = "sensor_msgs/msg/Image"
            if out_type not in ts_out.types:
                print(f"[skip] {c.topic} ({c.msgtype}): ROS1 に対応する型がありません", file=sys.stderr)
                continue
            conns.append(c)
        if not conns:
            sys.exit("変換対象のトピックがありません")

        # --- Mid-360 時刻オフセット推定 ---
        offset_ns = 0
        livox_conns = [c for c in conns if c.topic in args.livox_topics]
        lidar_conns = [c for c in livox_conns if "CustomMsg" in c.msgtype or "PointCloud2" in c.msgtype]
        if args.livox_time_align != "off" and lidar_conns:
            est = estimate_livox_offset(reader, lidar_conns)
            print(f"[info] Livox ヘッダ時刻と記録時刻の差 (中央値): {est / 1e9:.3f} s")
            if args.livox_time_align == "on" or abs(est) > args.align_threshold * 1e9:
                offset_ns = est
                print(f"[info] {args.livox_topics} のヘッダ時刻を {offset_ns / 1e9:+.3f} s 補正します "
                      "(PTP/PPS 同期していないと見なす)")

        writer = Writer(dst)
        if args.compress != "none":
            writer.set_compression(getattr(Writer.CompressionFormat, args.compress.upper()))
        counts = {}
        img_counter = {}
        with writer:
            out_conns = {}
            for c in conns:
                out_topic = c.topic
                out_type = TYPE_RENAME.get(c.msgtype, c.msgtype)
                if c.msgtype == "sensor_msgs/msg/CompressedImage" and not args.no_image_convert:
                    out_type = "sensor_msgs/msg/Image"
                    if out_topic.endswith("/compressed"):
                        out_topic = out_topic[: -len("/compressed")]
                out_conns[c.id] = (writer.add_connection(out_topic, out_type, typestore=ts_out), out_type)

            for c, logtime, raw in reader.messages(connections=conns):
                wconn, out_type = out_conns[c.id]
                msg = reader.deserialize(raw, c.msgtype)
                is_livox = c.topic in args.livox_topics

                if c.msgtype in ("sensor_msgs/msg/Image", "sensor_msgs/msg/CompressedImage"):
                    k = img_counter.get(c.topic, 0)
                    img_counter[c.topic] = k + 1
                    if k % args.image_every:
                        continue
                    bgr = None if args.no_image_convert else image_to_bgr8(msg, c.msgtype)
                    if bgr is not None:
                        Image = ts_out.types["sensor_msgs/msg/Image"]
                        Header = ts_out.types["std_msgs/msg/Header"]
                        h, w = bgr.shape[:2]
                        out = Image(
                            header=Header(seq=0, stamp=msg.header.stamp, frame_id=msg.header.frame_id),
                            height=h, width=w, encoding="bgr8", is_bigendian=0, step=w * 3,
                            data=np.ascontiguousarray(bgr).reshape(-1),
                        )
                    else:
                        out = convert_obj(msg, ts_out.types[out_type], ts_out)
                else:
                    out = convert_obj(msg, ts_out.types[out_type], ts_out)

                if is_livox and offset_ns and hasattr(out, "header"):
                    out.header.stamp = shift_stamp(out.header.stamp, offset_ns)
                    if hasattr(out, "timebase"):
                        out.timebase = out.timebase + offset_ns

                t = logtime
                if args.header_time and hasattr(out, "header"):
                    t = out.header.stamp.sec * 1_000_000_000 + out.header.stamp.nanosec
                writer.write(wconn, t, ts_out.serialize_ros1(out, out_type))
                counts[wconn.topic] = counts.get(wconn.topic, 0) + 1

    print("[done]", dst)
    for topic, n in sorted(counts.items()):
        print(f"  {topic}: {n} msgs")


if __name__ == "__main__":
    main()
