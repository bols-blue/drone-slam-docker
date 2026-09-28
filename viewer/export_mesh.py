#!/usr/bin/env python3
"""テクスチャ付き / 頂点色付きメッシュを Web ビューア用に軽量化して manifest.json に追加する。

open3d が必要 (fast-lio2-openmvs イメージに入っている):
  docker run --rm -v $PWD:/w --entrypoint python3 ghcr.io/bols-blue/drone-slam:fast-lio2-openmvs \
      /w/viewer/export_mesh.py --data /w/viewer/data --id r3live --mesh /w/output/r3live_xxx/textured_mesh.ply

テクスチャ (OBJ + 画像) は各三角形の頂点で色をサンプルして頂点色に焼き込み、
その後 quadric decimation で --faces 枚まで減らす。
出力: data/<id>.mesh.bin (非インデックス三角形: int16 xyz ×3頂点 + uint8 rgb ×3頂点)
"""
import argparse
import json
from pathlib import Path

import numpy as np
import open3d as o3d


def bake_texture_to_vertex_colors(mesh):
    """UV テクスチャを頂点色に焼き込む (頂点を共有する三角形の色を平均)"""
    tex = [np.asarray(t) for t in mesh.textures]
    uv = np.asarray(mesh.triangle_uvs).reshape(-1, 3, 2)
    tri = np.asarray(mesh.triangles)
    mat = np.asarray(mesh.triangle_material_ids) if len(mesh.triangle_material_ids) else np.zeros(len(tri), int)
    acc = np.zeros((len(mesh.vertices), 3))
    cnt = np.zeros(len(mesh.vertices))
    for m, img in enumerate(tex):
        if img.size == 0:
            continue
        sel = np.where(mat == m)[0]
        h, w = img.shape[:2]
        u = np.clip((uv[sel, :, 0] * (w - 1)).round().astype(int), 0, w - 1)
        v = np.clip(((1 - uv[sel, :, 1]) * (h - 1)).round().astype(int), 0, h - 1)
        col = img[v, u, :3].astype(float)
        # 未テクスチャ (黒) の角は平均に入れない
        valid = col.sum(-1) > 0
        for k in range(3):
            idx = tri[sel, k]
            np.add.at(acc, idx[valid[:, k]], col[valid[:, k], k])
            np.add.at(cnt, idx[valid[:, k]], 1)
    colors = np.where(cnt[:, None] > 0, acc / np.maximum(cnt, 1)[:, None], 90.0) / 255.0
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
    return mesh


def load_textured_obj(path):
    """OBJ (v / vt / f v/vt) + mtl の map_Kd を読み、頂点色付き open3d メッシュを返す。
    open3d の画像読み込みが失敗する大きな JPEG テクスチャ (OpenMVS 出力) 用。"""
    import cv2

    path = Path(path)
    vs, vts, faces, mats, cur = [], [], [], [], 0
    mtl_file, mat_names = None, {}
    for line in path.read_text(errors="ignore").splitlines():
        if line.startswith("v "):
            vs.append(line.split()[1:4])
        elif line.startswith("vt "):
            vts.append(line.split()[1:3])
        elif line.startswith("f "):
            corners = [c.split("/") for c in line.split()[1:4]]
            faces.append([int(c[0]) - 1 for c in corners] + [int(c[1]) - 1 if len(c) > 1 and c[1] else -1 for c in corners])
            mats.append(cur)
        elif line.startswith("mtllib "):
            mtl_file = path.parent / line.split(None, 1)[1].strip()
        elif line.startswith("usemtl "):
            cur = mat_names.setdefault(line.split(None, 1)[1].strip(), len(mat_names))
    textures = {}
    if mtl_file and mtl_file.exists():
        name = None
        for line in mtl_file.read_text().splitlines():
            if line.startswith("newmtl "):
                name = line.split(None, 1)[1].strip()
            elif line.strip().startswith("map_Kd") and name in mat_names:
                img = cv2.imread(str(mtl_file.parent / line.split(None, 1)[1].strip()), cv2.IMREAD_COLOR)
                textures[mat_names[name]] = img[:, :, ::-1]
    v = np.array(vs, float)
    f = np.array(faces, int)
    vt = np.array(vts, float) if vts else np.zeros((1, 2))
    mats = np.array(mats)
    acc = np.zeros((len(v), 3))
    cnt = np.zeros(len(v))
    for m, img in textures.items():
        sel = np.where((mats == m) & (f[:, 3:] >= 0).all(1))[0]
        h, w = img.shape[:2]
        uv = vt[f[sel, 3:]]
        u = np.clip((uv[..., 0] * (w - 1)).round().astype(int), 0, w - 1)
        vv = np.clip(((1 - uv[..., 1]) * (h - 1)).round().astype(int), 0, h - 1)
        col = img[vv, u].astype(float)
        valid = col.sum(-1) > 0
        for k in range(3):
            np.add.at(acc, f[sel, k][valid[:, k]], col[valid[:, k], k])
            np.add.at(cnt, f[sel, k][valid[:, k]], 1)
    mesh = o3d.geometry.TriangleMesh(o3d.utility.Vector3dVector(v), o3d.utility.Vector3iVector(f[:, :3]))
    colors = np.where(cnt[:, None] > 0, acc / np.maximum(cnt, 1)[:, None], 90.0) / 255.0
    mesh.vertex_colors = o3d.utility.Vector3dVector(colors)
    return mesh


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--data", required=True, help="export_viewer_data.py の出力ディレクトリ")
    ap.add_argument("--id", required=True, help="manifest.json 内の run id")
    ap.add_argument("--mesh", required=True)
    ap.add_argument("--faces", type=int, default=220000)
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    data = Path(args.data)
    manifest = json.loads((data / "manifest.json").read_text())
    run = next(r for r in manifest["runs"] if r["id"] == args.id)

    if args.mesh.lower().endswith(".obj"):
        mesh = load_textured_obj(args.mesh)
    else:
        mesh = o3d.io.read_triangle_mesh(args.mesh, enable_post_processing=False)
    n_faces_full = len(mesh.triangles)
    if mesh.has_triangle_uvs() and len(mesh.textures):
        mesh = bake_texture_to_vertex_colors(mesh)
        mesh.triangle_uvs = o3d.utility.Vector2dVector()
        mesh.textures = []
    if not mesh.has_vertex_colors():
        mesh.paint_uniform_color([0.7, 0.7, 0.7])
    mesh.remove_duplicated_vertices()
    mesh.remove_degenerate_triangles()
    if len(mesh.triangles) > args.faces:
        mesh = mesh.simplify_quadric_decimation(args.faces)
    v = np.asarray(mesh.vertices)
    c = (np.clip(np.asarray(mesh.vertex_colors), 0, 1) * 255).round().astype(np.uint8)
    t = np.asarray(mesh.triangles)
    center = np.array(run["center"])
    rel = v[t].reshape(-1, 3) - center
    scale = float(max(np.abs(rel).max() / 32000.0, 0.001))
    q = np.round(rel / scale).astype("<i2")
    col = c[t].reshape(-1, 3)
    out = data / f"{args.id}.mesh.bin"
    with open(out, "wb") as f:
        f.write(q.tobytes())
        f.write(np.ascontiguousarray(col).tobytes())
    run["mesh"] = {"file": out.name, "faces": int(len(t)), "faces_full": int(n_faces_full), "scale": scale,
                   "label": args.label}
    (data / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False))
    print(f"{args.id}: mesh {n_faces_full} -> {len(t)} faces, {out.stat().st_size / 1e6:.1f} MB")


if __name__ == "__main__":
    main()
