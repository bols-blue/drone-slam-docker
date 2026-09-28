#!/usr/bin/env python3
"""バイナリ PCD (x y z rgb) をボクセルグリッドで間引く (ボクセル内の点の平均位置・平均色)。

FAST-LIVO2 内蔵の pcl::VoxelGrid は地図範囲 / leaf^3 が int の範囲を超えると
"Leaf size is too small" で間引きをスキップするため、広い地図でも動くようにこちらで行う。
"""
import argparse

import numpy as np

DT = np.dtype([("x", "<f4"), ("y", "<f4"), ("z", "<f4"), ("rgb", "<u4")])


def read_pcd(path):
    with open(path, "rb") as f:
        header = []
        while True:
            line = f.readline()
            header.append(line)
            if line.startswith(b"DATA"):
                break
        fields = [l for l in header if l.startswith(b"FIELDS")][0].split()[1:]
        if fields != [b"x", b"y", b"z", b"rgb"] or b"binary" not in header[-1]:
            raise SystemExit("x y z rgb の binary PCD のみ対応: %s" % path)
        return np.frombuffer(f.read(), dtype=DT)


def write_pcd(path, a):
    n = len(a)
    hdr = ("# .PCD v0.7 - Point Cloud Data file format\nVERSION 0.7\nFIELDS x y z rgb\nSIZE 4 4 4 4\n"
           "TYPE F F F U\nCOUNT 1 1 1 1\nWIDTH %d\nHEIGHT 1\nVIEWPOINT 0 0 0 1 0 0 0\nPOINTS %d\nDATA binary\n" % (n, n))
    with open(path, "wb") as f:
        f.write(hdr.encode())
        f.write(a.tobytes())


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
    out = np.empty(m, dtype=DT)
    for i, c in enumerate("xyz"):
        out[c] = np.bincount(inv, weights=xyz[:, i], minlength=m) / cnt
    rgb = a["rgb"]
    ch = []
    for shift in (16, 8, 0):
        v = np.bincount(inv, weights=(rgb >> shift) & 255, minlength=m) / cnt
        ch.append(np.clip(np.rint(v), 0, 255).astype(np.uint32))
    out["rgb"] = (ch[0] << 16) | (ch[1] << 8) | ch[2]
    write_pcd(args.dst, out)
    print("[voxel_downsample] %d -> %d points (leaf %.3f m): %s" % (len(a), m, args.leaf, args.dst))


if __name__ == "__main__":
    main()
