#!/usr/bin/env python3
"""ROS2 bag (mcap / sqlite3) を ROS1 bag に変換する。(ROS1 bag を入力して画像展開だけ行うことも可)

drone_slam (ROS2 Jazzy) で記録した bag を、ROS1 実装の SLAM (FAST-LIVO2 / FAST-LIO2 /
R3LIVE / SR-LIVO / LVI-SAM) でリプレイできるようにするためのツール。

主な処理:
  * livox_ros_driver2/msg/CustomMsg -> livox_ros_driver/CustomMsg (フィールドは同一、型名のみ違う)
  * 標準メッセージ (sensor_msgs/Imu 等) は ROS1 形式 (Header.seq 付き) に変換
  * 画像: yuv422 (usb_cam の yuyv) / rgb8 / CompressedImage を bgr8 の sensor_msgs/Image に変換
  * Hesai (XT16 等) の PointCloud2 (x y z intensity ring timestamp) -> livox_ros_driver/CustomMsg (<topic>/livox)。
    各 SLAM 手法は Livox 形式を共通に読めるため。時刻は記録時刻との関係を見て付け直す (--lidar-time-align)
  * IMU の外れ値 (起動直後に MAVROS が単位換算前の値を出す等) を除去
  * Mid-360 の時刻補正: PTP 同期をしていない Mid-360 はヘッダ時刻が LiDAR 起動からの経過時間になり、
    カメラ (PC 時刻) と数十年ずれる。受信時刻との差の中央値でオフセットを推定し補正する (--livox-time-align)

使用例:
  ros2bag-to-ros1 /data/texture_20260928_101010 /data/texture_20260928_101010.bag
"""
import argparse
import dataclasses
import statistics
import struct
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


# ---------------------------------------------------------------------------
# Hesai (XT16 など) の PointCloud2 -> livox_ros_driver/CustomMsg
#   各 SLAM 手法は Livox の CustomMsg (点ごとの相対時刻 offset_time) を共通に読めるので、
#   回転式 LiDAR の点群もこの形式にそろえる (R3LIVE は Velodyne 入力が未実装のため)。
# ---------------------------------------------------------------------------
PF_DTYPES = {1: "i1", 2: "u1", 3: "<i2", 4: "<u2", 5: "<i4", 6: "<u4", 7: "<f4", 8: "<f8"}
HESAI_FIELDS = {"x", "y", "z", "intensity", "ring", "timestamp"}
# CustomPoint (ROS1 シリアライズ時は詰めて並ぶ): uint32 offset_time, float32 x,y,z, uint8 reflectivity, tag, line
LIVOX_POINT_DTYPE = np.dtype([("offset_time", "<u4"), ("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                              ("reflectivity", "u1"), ("tag", "u1"), ("line", "u1")])


def pc2_array(msg):
    """PointCloud2 を numpy の構造化配列にする (必要なフィールドだけ)"""
    fields = [(f.name, PF_DTYPES[f.datatype], f.offset) for f in msg.fields if f.datatype in PF_DTYPES]
    dt = np.dtype({"names": [f[0] for f in fields], "formats": [f[1] for f in fields],
                   "offsets": [f[2] for f in fields], "itemsize": msg.point_step})
    n = msg.width * msg.height
    return np.frombuffer(bytes(msg.data), dtype=dt, count=n)


def is_hesai(msg):
    names = {f.name for f in msg.fields}
    ts = [f for f in msg.fields if f.name == "timestamp"]
    return HESAI_FIELDS <= names and ts and ts[0].datatype == 8


def analyze_hesai(reader, conn, threshold_s):
    """点群のヘッダ時刻と記録時刻の関係から、時刻の付け直し方を決める。

    keep  : PTP 等で同期済み (ずれが小さく一定) -> そのまま
    offset: 一定のずれ (PTP 未設定で LiDAR 内蔵時計) -> 中央値で一律に補正
    record: ずれが時間とともに変わる (LiDAR の時計の進み方がずれている) -> 記録時刻から付け直し、
            スキャン内の相対時刻もスキャン周期に合わせて伸縮する
    """
    rows = []
    for _, logtime, raw in reader.messages(connections=[conn]):
        m = reader.deserialize(raw, conn.msgtype)
        a = pc2_array(m)
        h = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        t = a["timestamp"] if len(a) else np.array([h])
        rows.append((logtime * 1e-9, h, float(t.min()), float(t.max())))
    if len(rows) < 3:
        return {"mode": "keep", "offset": 0.0, "scale": 1.0, "period": 0.1}
    r = np.array(rows)
    lag = r[:, 0] - r[:, 1]                       # 記録時刻 - ヘッダ時刻
    med = float(np.median(lag))
    spread = float(np.percentile(lag, 90) - np.percentile(lag, 10))
    period = float(np.median(np.diff(r[:, 0])))   # PC の時計で見たスキャン周期
    span = float(np.median(r[:, 3] - r[:, 2]))    # LiDAR の時計で見たスキャン 1 回分
    info = {"offset": med, "spread": spread, "period": period, "span": span, "scale": 1.0}
    if spread > 0.05 or abs(span / period - 1.0) > 0.03:
        scale = period / span if span > 0 else 1.0
        # スキャン同士の間隔は LiDAR の時計の差 (進み方を補正) で決め、記録時刻には全体として合わせる。
        # 記録時刻をそのまま使うと、大きなメッセージがまとめて届いたときにスキャンの時刻が重なるため。
        # LiDAR の時刻が跳んだ (PTP の補正など) ところだけ記録時刻に合わせ直す。
        rec, t0 = r[:, 0], r[:, 2]
        starts = np.empty(len(r))
        starts[0] = rec[0] - period
        for k in range(1, len(r)):
            dh = (t0[k] - t0[k - 1]) * scale
            drec = rec[k] - rec[k - 1]
            if dh > 0 and abs(dh - drec) < 0.5:
                starts[k] = starts[k - 1] + dh
            else:
                starts[k] = rec[k] - period
        starts += np.median(rec - period - starts)
        info.update(mode="record", scale=scale, starts=starts, idx=0)
    elif abs(med) > threshold_s:
        info.update(mode="offset")
    else:
        info.update(mode="keep")
    return info


def hesai_to_livox_ros1(msg, logtime_ns, info, frame_id=None):
    """Hesai の PointCloud2 を ROS1 の livox_ros_driver/CustomMsg のバイト列にする"""
    a = pc2_array(msg)
    keep = np.isfinite(a["x"]) & ((a["x"] != 0) | (a["y"] != 0) | (a["z"] != 0))
    a = a[keep]
    ts = a["timestamp"]
    t0 = float(ts.min()) if len(a) else 0.0
    rel = (ts - t0) * info["scale"]
    if info["mode"] == "record":
        if "starts" in info and info["idx"] < len(info["starts"]):
            start = float(info["starts"][info["idx"]])
            info["idx"] += 1
        else:
            # スキャンの最後の点を受信した時刻 ≒ 記録時刻 とみなし、先頭は 1 周期前
            start = logtime_ns * 1e-9 - info["period"]
    elif info["mode"] == "offset":
        start = t0 + info["offset"]
    else:
        start = t0
    order = np.argsort(rel, kind="stable")
    pts = np.empty(len(a), dtype=LIVOX_POINT_DTYPE)
    pts["offset_time"] = np.clip(np.round(rel[order] * 1e9), 0, 2**32 - 1).astype("<u4")
    for k in ("x", "y", "z"):
        pts[k] = a[k][order]
    pts["reflectivity"] = np.clip(a["intensity"][order], 0, 255).astype("u1")
    pts["tag"] = 0x10            # Livox の「通常の点」(FAST-LIO 等のタグフィルタを通す)
    pts["line"] = np.clip(a["ring"][order], 0, 255).astype("u1")
    start_ns = int(round(start * 1e9))
    fid = (frame_id or msg.header.frame_id).encode()
    head = struct.pack("<III", 0, start_ns // 1_000_000_000, start_ns % 1_000_000_000)
    head += struct.pack("<I", len(fid)) + fid
    body = struct.pack("<QIB3B", start_ns, len(pts), 0, 0, 0, 0) + struct.pack("<I", len(pts)) + pts.tobytes()
    return head + body, start_ns


def imu_norm_median(reader, conn, n=400):
    v = []
    for _, _, raw in reader.messages(connections=[conn]):
        a = reader.deserialize(raw, conn.msgtype).linear_acceleration
        v.append((a.x * a.x + a.y * a.y + a.z * a.z) ** 0.5)
        if len(v) >= n:
            break
    return float(np.median(v)) if v else 0.0


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("src", help="ROS2 bag ディレクトリ (metadata.yaml を含む) / .mcap / .db3、または ROS1 .bag")
    ap.add_argument("dst", help="出力する ROS1 .bag")
    ap.add_argument("--topics", nargs="*", help="変換するトピック (省略時は変換可能な全トピック)")
    ap.add_argument("--exclude", nargs="*", default=["/xt16/points_time_ring"],
                    help="除外するトピック (既定: drone_slam の XT16 再配信トピック。/lidar_points から作り直すため)")
    ap.add_argument("--livox-topics", nargs="*", default=["/livox/lidar", "/livox/imu"],
                    help="Mid-360 の時計で打刻されたトピック (時刻補正の対象)")
    ap.add_argument("--livox-time-align", choices=["auto", "on", "off"], default="auto",
                    help="auto: 受信時刻とのずれが --align-threshold 秒を超えたら補正")
    ap.add_argument("--align-threshold", type=float, default=1.0)
    ap.add_argument("--no-image-convert", action="store_true", help="画像を bgr8 に変換しない")
    ap.add_argument("--image-every", type=int, default=1, help="画像を N 枚に 1 枚だけ残す (間引き)")
    ap.add_argument("--header-time", action="store_true",
                    help="bag の記録時刻ではなくヘッダ時刻を ROS1 bag の時刻に使う (再生順がヘッダ時刻順になる)")
    ap.add_argument("--imu-ms2-copy", nargs="*", default=[], metavar="TOPIC",
                    help="加速度が g 単位の IMU トピックについて、9.80665 倍した複製を <TOPIC>_ms2 として追加する "
                         "(m/s^2 前提の LVI-SAM に Mid-360S 内蔵IMU を入力する場合など)")
    ap.add_argument("--hesai-topics", nargs="*", default=None,
                    help="Livox 形式に変換する Hesai の点群トピック (既定: timestamp/ring を持つ PointCloud2 を自動検出)")
    ap.add_argument("--keep-hesai-source", action="store_true", help="変換元の Hesai の PointCloud2 も残す")
    ap.add_argument("--lidar-time-align", choices=["auto", "keep", "offset", "record"], default="auto",
                    help="Hesai 点群の時刻: auto=自動判定 / keep=そのまま / offset=一律補正 / record=記録時刻から付け直す")
    ap.add_argument("--imu-outlier-ratio", type=float, default=8.0,
                    help="|加速度| が中央値のこの倍数を超える IMU メッセージを捨てる (0 で無効)")
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

        # --- Hesai 点群の検出と時刻の付け直し方の決定 ---
        hesai = {}
        for c in conns:
            if c.msgtype != "sensor_msgs/msg/PointCloud2":
                continue
            if args.hesai_topics is not None and c.topic not in args.hesai_topics:
                continue
            first = next(reader.messages(connections=[c]), None)
            if first is None or not is_hesai(reader.deserialize(first[2], c.msgtype)):
                continue
            info = analyze_hesai(reader, c, args.align_threshold)
            if args.lidar_time_align != "auto":
                info["mode"] = args.lidar_time_align
                if info["mode"] != "record":
                    info["scale"] = 1.0
            hesai[c.id] = info
            print(f"[info] {c.topic} (Hesai) -> {c.topic}/livox: 時刻の扱い={info['mode']} "
                  f"(記録-ヘッダ 中央値 {info.get('offset', 0):+.3f} s, ばらつき {info.get('spread', 0):.3f} s, "
                  f"スキャン周期 PC {info['period']:.4f} s / LiDAR {info.get('span', 0):.4f} s)")
            if info["mode"] == "record":
                print("[info]   LiDAR の時計の進み方が PC とずれているため、記録時刻からスキャン時刻を付け直します "
                      "(PTP が Locked になっていない可能性。精度は受信遅延のばらつき程度)")

        # --- IMU の |加速度| の中央値 (外れ値除去と単位判定に使う) ---
        imu_med = {c.id: imu_norm_median(reader, c) for c in conns if c.msgtype == "sensor_msgs/msg/Imu"}

        writer = Writer(dst)
        if args.compress != "none":
            writer.set_compression(getattr(Writer.CompressionFormat, args.compress.upper()))
        counts = {}
        img_counter = {}
        with writer:
            out_conns = {}
            ms2_conns = {}
            hesai_conns = {}
            for c in conns:
                if c.topic in args.imu_ms2_copy and c.msgtype == "sensor_msgs/msg/Imu":
                    if imu_med.get(c.id, 0) < 3.0:
                        ms2_conns[c.id] = writer.add_connection(c.topic + "_ms2", "sensor_msgs/msg/Imu", typestore=ts_out)
                    else:
                        print(f"[info] {c.topic} は既に m/s^2 なので _ms2 の複製は作りません")
                if c.id in hesai:
                    hesai_conns[c.id] = writer.add_connection(c.topic + "/livox", "livox_ros_driver/msg/CustomMsg",
                                                              typestore=ts_out)
            for c in conns:
                if c.id in hesai and not args.keep_hesai_source:
                    continue
                out_topic = c.topic
                out_type = TYPE_RENAME.get(c.msgtype, c.msgtype)
                if c.msgtype == "sensor_msgs/msg/CompressedImage" and not args.no_image_convert:
                    out_type = "sensor_msgs/msg/Image"
                    if out_topic.endswith("/compressed"):
                        out_topic = out_topic[: -len("/compressed")]
                out_conns[c.id] = (writer.add_connection(out_topic, out_type, typestore=ts_out), out_type)

            dropped = {}
            for c, logtime, raw in reader.messages(connections=conns):
                msg = reader.deserialize(raw, c.msgtype)
                if c.id in hesai:
                    data, _ = hesai_to_livox_ros1(msg, logtime, hesai[c.id])
                    hconn = hesai_conns[c.id]
                    writer.write(hconn, logtime, data)
                    counts[hconn.topic] = counts.get(hconn.topic, 0) + 1
                    if c.id not in out_conns:
                        continue
                if c.id in imu_med and args.imu_outlier_ratio > 0 and imu_med[c.id] > 0:
                    a = msg.linear_acceleration
                    if (a.x * a.x + a.y * a.y + a.z * a.z) ** 0.5 > args.imu_outlier_ratio * imu_med[c.id]:
                        dropped[c.topic] = dropped.get(c.topic, 0) + 1
                        continue
                wconn, out_type = out_conns[c.id]
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
                if c.id in ms2_conns:
                    a = out.linear_acceleration
                    out.linear_acceleration = type(a)(x=a.x * 9.80665, y=a.y * 9.80665, z=a.z * 9.80665)
                    mconn = ms2_conns[c.id]
                    writer.write(mconn, t, ts_out.serialize_ros1(out, out_type))
                    counts[mconn.topic] = counts.get(mconn.topic, 0) + 1

    for topic, n in dropped.items():
        print(f"[info] {topic}: 外れ値の IMU メッセージを {n} 件除去しました")
    print("[done]", dst)
    for topic, n in sorted(counts.items()):
        print(f"  {topic}: {n} msgs")


if __name__ == "__main__":
    main()
