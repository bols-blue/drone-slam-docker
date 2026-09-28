#!/usr/bin/env python3
"""slam-replay の出力 (色付き点群 + 軌跡) を Web ビューア用の軽量バイナリに変換する。

  python3 export_viewer_data.py --out viewer/data \
      --run fast-livo2:FAST-LIVO2:/output/fast_livo2_xxx/map_rgb.pcd:/output/fast_livo2_xxx/trajectory_tum.txt \
      --run sr-livo:SR-LIVO:/output/sr_livo_xxx/rgb_map.pcd:/output/sr_livo_xxx/pose.txt

出力: data/<id>.bin (int16 xyz[cm 相当に量子化] + uint8 rgb) と data/manifest.json
対応入力: PCD (ascii / binary, フィールド x y z [rgb|rgba]) / PLY (ascii / binary_little_endian, x y z [red green blue])
軌跡: TUM 形式 (t x y z qx qy qz qw)
"""
import argparse
import json
from pathlib import Path

import numpy as np

PCD_TYPES = {("F", 4): "<f4", ("F", 8): "<f8", ("U", 1): "u1", ("U", 2): "<u2", ("U", 4): "<u4",
             ("I", 1): "i1", ("I", 2): "<i2", ("I", 4): "<i4"}
PLY_TYPES = {"float": "<f4", "float32": "<f4", "double": "<f8", "float64": "<f8", "uchar": "u1", "uint8": "u1",
             "char": "i1", "int8": "i1", "ushort": "<u2", "uint16": "<u2", "short": "<i2", "int16": "<i2",
             "uint": "<u4", "uint32": "<u4", "int": "<i4", "int32": "<i4"}


def read_pcd(path):
    with open(path, "rb") as f:
        hdr = {}
        while True:
            line = f.readline().decode("ascii", "ignore").strip()
            if not line or line.startswith("#"):
                continue
            k, *v = line.split()
            hdr[k] = v
            if k == "DATA":
                break
        fields, sizes, types = hdr["FIELDS"], [int(s) for s in hdr["SIZE"]], hdr["TYPE"]
        counts = [int(c) for c in hdr.get("COUNT", ["1"] * len(fields))]
        n = int(hdr["POINTS"][0])
        dt = np.dtype([(fn if c == 1 else fn, PCD_TYPES[(t, s)], (c,) if c > 1 else ())
                       for fn, s, t, c in zip(fields, sizes, types, counts)])
        mode = hdr["DATA"][0]
        if mode == "binary":
            arr = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
        elif mode == "ascii":
            raw = np.loadtxt(f, dtype=np.float64, ndmin=2)
            arr = np.zeros(len(raw), dtype=dt)
            for i, fn in enumerate(fields):
                if fn in ("rgb", "rgba"):
                    arr[fn] = raw[:, i].astype(np.float32).view(np.uint32) if dt[fn].kind == "f" else raw[:, i]
                else:
                    arr[fn] = raw[:, i]
        else:
            raise SystemExit(f"{path}: DATA {mode} は未対応 (pcl_convert_pcd_ascii_binary で binary に変換してください)")
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float32)
    rgb = None
    for key in ("rgb", "rgba"):
        if key in arr.dtype.names:
            v = arr[key]
            v = v.view(np.uint32) if v.dtype.kind == "f" else v.astype(np.uint32)
            rgb = np.stack([(v >> 16) & 255, (v >> 8) & 255, v & 255], 1).astype(np.uint8)
    return xyz, rgb


def read_ply(path):
    with open(path, "rb") as f:
        fmt, props, n, in_vertex = None, [], 0, False
        while True:
            line = f.readline().decode("ascii", "ignore").strip()
            if line.startswith("format"):
                fmt = line.split()[1]
            elif line.startswith("element"):
                _, name, cnt = line.split()
                in_vertex = name == "vertex"
                if in_vertex:
                    n = int(cnt)
            elif line.startswith("property") and in_vertex:
                p = line.split()
                if p[1] == "list":
                    raise SystemExit("vertex に list プロパティがある PLY は未対応")
                props.append((p[2], PLY_TYPES[p[1]]))
            elif line == "end_header":
                break
        dt = np.dtype(props)
        if fmt == "binary_little_endian":
            arr = np.frombuffer(f.read(n * dt.itemsize), dtype=dt, count=n)
        elif fmt == "ascii":
            raw = np.loadtxt(f, max_rows=n, ndmin=2)
            arr = np.zeros(n, dtype=dt)
            for i, (name, _) in enumerate(props):
                arr[name] = raw[:, i]
        else:
            raise SystemExit(f"PLY format {fmt} は未対応")
    xyz = np.stack([arr["x"], arr["y"], arr["z"]], 1).astype(np.float32)
    rgb = None
    names = arr.dtype.names
    for r, g, b in (("red", "green", "blue"), ("r", "g", "b")):
        if r in names:
            rgb = np.stack([arr[r], arr[g], arr[b]], 1).astype(np.uint8)
    return xyz, rgb


def voxel_downsample(xyz, rgb, voxel, cap, seed=0):
    keys = np.floor(xyz / voxel).astype(np.int64)
    _, idx = np.unique(keys, axis=0, return_index=True)
    if len(idx) > cap:
        idx = np.random.default_rng(seed).choice(idx, cap, replace=False)
    idx.sort()
    return xyz[idx], (rgb[idx] if rgb is not None else None)


def read_traj(path):
    if not path:
        return None
    rows = []
    for line in Path(path).read_text().splitlines():
        p = line.replace(",", " ").split()
        if len(p) >= 4 and not line.startswith("#"):
            try:
                rows.append([float(v) for v in p[:4]])
            except ValueError:
                pass
    return np.array(rows) if rows else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", required=True)
    ap.add_argument("--run", action="append", required=True,
                    help="id:表示名:点群ファイル[:軌跡ファイル[:メモ]]")
    ap.add_argument("--voxel", type=float, default=0.15)
    ap.add_argument("--cap", type=int, default=450000)
    ap.add_argument("--dataset", default="")
    args = ap.parse_args()
    out = Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    runs = []
    for spec in args.run:
        parts = spec.split(":")
        rid, name, cloud = parts[0], parts[1], parts[2]
        traj_path = parts[3] if len(parts) > 3 else ""
        note = ":".join(parts[4:]) if len(parts) > 4 else ""
        xyz, rgb = (read_ply if cloud.lower().endswith(".ply") else read_pcd)(cloud)
        ok = np.isfinite(xyz).all(1)
        xyz, rgb = xyz[ok], (rgb[ok] if rgb is not None else None)
        n_full = len(xyz)
        colored = 0.0 if rgb is None else float((rgb.astype(int).sum(1) > 0).mean())
        xyz, rgb = voxel_downsample(xyz, rgb, args.voxel, args.cap)
        center = np.round(np.median(xyz, 0), 2)
        rel = xyz - center
        scale = float(max(np.abs(rel).max() / 32000.0, 0.001))
        q = np.round(rel / scale).astype("<i2")
        if rgb is None:
            rgb = np.full((len(xyz), 3), 180, np.uint8)
        with open(out / f"{rid}.bin", "wb") as f:
            f.write(q.tobytes())
            f.write(np.ascontiguousarray(rgb).tobytes())
        traj = read_traj(traj_path)
        tinfo = {}
        if traj is not None and len(traj) > 1:
            step = max(1, len(traj) // 1500)
            pts = traj[::step, 1:4]
            if not np.array_equal(pts[-1], traj[-1, 1:4]):
                pts = np.vstack([pts, traj[-1, 1:4]])
            seg = np.linalg.norm(np.diff(traj[:, 1:4], axis=0), axis=1)
            tinfo = {
                "points": np.round(pts - center, 3).tolist(),
                "length_m": round(float(seg.sum()), 2),
                "duration_s": round(float(traj[-1, 0] - traj[0, 0]), 1),
                "end_to_start_m": round(float(np.linalg.norm(traj[-1, 1:4] - traj[0, 1:4])), 3),
                "poses": int(len(traj)),
            }
        lo, hi = xyz.min(0), xyz.max(0)
        runs.append({
            "id": rid, "name": name, "note": note, "file": f"{rid}.bin", "count": int(len(xyz)),
            "count_full": int(n_full), "colored_ratio": round(colored, 4), "scale": scale,
            "center": center.tolist(), "extent_m": np.round(hi - lo, 1).tolist(), "trajectory": tinfo,
        })
        print(f"{rid}: {n_full} -> {len(xyz)} pts, file {(out / (rid + '.bin')).stat().st_size / 1e6:.1f} MB")
    (out / "manifest.json").write_text(json.dumps({"dataset": args.dataset, "voxel_m": args.voxel, "runs": runs},
                                                  ensure_ascii=False))


if __name__ == "__main__":
    main()
