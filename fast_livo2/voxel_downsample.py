#!/usr/bin/env python3
"""バイナリ PCD をボクセルグリッドで間引く (ボクセル内の点の平均位置・平均色 / 平均反射強度)。

入力: x y z rgb (img_en: 1 の色付き地図) または x y z intensity ... (img_en: 0 の LIO 地図。
      法線などの余分なフィールドは捨てる)。出力は x y z rgb または x y z intensity。

FAST-LIVO2 内蔵の pcl::VoxelGrid は地図範囲 / leaf^3 が int の範囲を超えると
"Leaf size is too small" で間引きをスキップするため、広い地図でも動くようにこちらで行う。
"""
import argparse

import numpy as np

PCD_TYPES = {("F", "4"): "<f4", ("F", "8"): "<f8", ("U", "1"): "u1", ("U", "2"): "<u2", ("U", "4"): "<u4",
             ("I", "1"): "i1", ("I", "2"): "<i2", ("I", "4"): "<i4"}


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
        if hdr["DATA"][0] != "binary":
            raise SystemExit("binary PCD のみ対応: %s" % path)
        counts = hdr.get("COUNT", ["1"] * len(hdr["FIELDS"]))
        if any(c != "1" for c in counts):
            raise SystemExit("COUNT が 1 以外のフィールドには未対応: %s" % path)
        dt = np.dtype([(n, PCD_TYPES[(t, s)]) for n, s, t in zip(hdr["FIELDS"], hdr["SIZE"], hdr["TYPE"])])
        n = int(hdr["POINTS"][0])
        a = np.fromfile(f, dtype=dt, count=n)
    if not {"x", "y", "z"} <= set(a.dtype.names):
        raise SystemExit("x y z が無い PCD です: %s" % path)
    return a


def write_pcd(path, out, attr):
    n = len(out)
    if attr == "rgb":
        fields, types = "x y z rgb", "F F F U"
    else:
        fields, types = "x y z intensity", "F F F F"
    hdr = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS %s\nSIZE 4 4 4 4\n"
           "TYPE %s\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n"
           % (fields, types, n, n))
    with open(path, "wb") as f:
        f.write(hdr.encode())
        f.write(out.tobytes())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("dst")
    ap.add_argument("--leaf", type=float, default=0.1)
    args = ap.parse_args()

    a = read_pcd(args.src)
    xyz = np.stack([a["x"], a["y"], a["z"]], 1).astype(np.float64)
    ok = np.isfinite(xyz).all(1)
    a, xyz = a[ok], xyz[ok]
    k = np.floor((xyz - xyz.min(0)) / args.leaf).astype(np.int64)
    dims = k.max(0) + 1
    key = k[:, 0] + dims[0] * (k[:, 1] + dims[1] * k[:, 2])
    _, inv, cnt = np.unique(key, return_inverse=True, return_counts=True)
    m = len(cnt)
    names = a.dtype.names
    attr = "rgb" if "rgb" in names else ("intensity" if "intensity" in names else None)
    out = np.zeros(m, dtype=np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"),
                                      ("rgb", "<u4") if attr == "rgb" else ("intensity", "<f4")]))
    for i, c in enumerate("xyz"):
        out[c] = np.bincount(inv, weights=xyz[:, i], minlength=m) / cnt
    if attr == "rgb":
        rgb = a["rgb"].view(np.uint32) if a["rgb"].dtype.kind == "f" else a["rgb"].astype(np.uint32)
        ch = []
        for shift in (16, 8, 0):
            v = np.bincount(inv, weights=(rgb >> shift) & 255, minlength=m) / cnt
            ch.append(np.clip(np.rint(v), 0, 255).astype(np.uint32))
        out["rgb"] = (ch[0] << 16) | (ch[1] << 8) | ch[2]
    elif attr == "intensity":
        out["intensity"] = np.bincount(inv, weights=a["intensity"].astype(np.float64), minlength=m) / cnt
    write_pcd(args.dst, out, attr)
    print("[voxel_downsample] %d -> %d points (leaf %.3f m): %s" % (len(a), m, args.leaf, args.dst))


if __name__ == "__main__":
    main()
