#!/usr/bin/env python3
"""FAST-LIO2 の出力 (地図 + 軌跡) とカメラ画像から、テクスチャ付きメッシュと色付き点群を作る後処理。

  1. scans.pcd (world=camera_init 座標系) -> ダウンサンプル・法線推定 -> Poisson 再構成 -> mesh.ply
  2. slam_output.bag の /Odometry (IMU の位置姿勢) を画像時刻に補間し、
     T_world_cam = T_world_imu * T_imu_cam でカメラ姿勢を求める。キーフレームを選んで歪み補正し、
     COLMAP テキストモデル (cameras.txt / images.txt / 空の points3D.txt) を書き出す
  3. OpenMVS: InterfaceCOLMAP -> scene.mvs, TextureMesh --mesh-file mesh.ply -> textured.obj (+ png)
  4. 地図点をキーフレームに投影して (簡易デプステスト付き) 色を平均 -> colored_map.ply / .pcd

slam-replay から自動実行されるほか、既存の出力ディレクトリに対して単体でも実行できる:
  docker run --rm -v $PWD:/work --entrypoint texture-pipeline ghcr.io/bols-blue/drone-slam:fast-lio2-openmvs \\
      --out-dir /work/output/fast_lio2_openmvs_XXXX --bag /work/flight.bag --calib /work/calib.yaml
"""
import argparse
import json
import os
import shutil
import subprocess
import sys
import time

import cv2
import numpy as np
import open3d as o3d
import rosbag
from scipy.spatial import cKDTree
from scipy.spatial.transform import Rotation, Slerp

import calib as C


def log(*a):
    print("[texture]", *a, flush=True)


# ----------------------------------------------------------------------------- 軌跡
def load_odometry(bag_path, topic):
    ts, pos, quat = [], [], []
    with rosbag.Bag(bag_path) as b:
        for _, m, _ in b.read_messages(topics=[topic]):
            p, q = m.pose.pose.position, m.pose.pose.orientation
            ts.append(m.header.stamp.to_sec())
            pos.append([p.x, p.y, p.z])
            quat.append([q.x, q.y, q.z, q.w])
    if len(ts) < 2:
        raise RuntimeError("{} に {} が 2 件以上ありません".format(bag_path, topic))
    ts = np.array(ts)
    order = np.argsort(ts)
    ts, pos, quat = ts[order], np.array(pos)[order], np.array(quat)[order]
    keep = np.concatenate([[True], np.diff(ts) > 1e-6])
    return ts[keep], pos[keep], quat[keep]


class PoseInterp:
    def __init__(self, ts, pos, quat):
        self.ts, self.pos = ts, pos
        self.slerp = Slerp(ts, Rotation.from_quat(quat))

    def __call__(self, t):
        """t における T_world_imu (4x4)。範囲外なら None"""
        if t < self.ts[0] or t > self.ts[-1]:
            return None
        T = np.eye(4)
        T[:3, :3] = self.slerp([t]).as_matrix()[0]
        T[:3, 3] = [np.interp(t, self.ts, self.pos[:, k]) for k in range(3)]
        return T


# ----------------------------------------------------------------------------- 画像
def decode_image(m):
    enc = m.encoding.lower()
    buf = np.frombuffer(m.data, np.uint8)
    if enc in ("bgr8", "rgb8"):
        img = buf.reshape(m.height, m.step)[:, : m.width * 3].reshape(m.height, m.width, 3)
        return img if enc == "bgr8" else cv2.cvtColor(img, cv2.COLOR_RGB2BGR)
    if enc in ("mono8", "8uc1"):
        img = buf.reshape(m.height, m.step)[:, : m.width]
        return cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    if enc in ("bgra8", "rgba8"):
        img = buf.reshape(m.height, m.step)[:, : m.width * 4].reshape(m.height, m.width, 4)
        return cv2.cvtColor(img, cv2.COLOR_BGRA2BGR if enc == "bgra8" else cv2.COLOR_RGBA2BGR)
    if enc in ("yuv422", "uyvy", "yuv422_yuy2", "yuyv"):
        img = buf.reshape(m.height, m.step)[:, : m.width * 2].reshape(m.height, m.width, 2)
        code = cv2.COLOR_YUV2BGR_UYVY if enc in ("yuv422", "uyvy") else cv2.COLOR_YUV2BGR_YUY2
        return cv2.cvtColor(img, code)
    raise ValueError("未対応の画像エンコーディング: " + m.encoding)


def select_keyframes(bag_path, topic, interp, T_imu_cam, time_offset, args):
    """画像を走査し、移動量/回転量でキーフレームを選ぶ。戻り値: [(stamp, T_world_cam, bgr)]"""
    kfs = []
    last_T = None
    n_total = 0
    compressed = topic.endswith("/compressed")
    with rosbag.Bag(bag_path) as b:
        if topic not in b.get_type_and_topic_info().topics:
            raise RuntimeError("入力 bag に画像トピック {} がありません".format(topic))
        for _, m, _ in b.read_messages(topics=[topic]):
            n_total += 1
            # calib.yaml: カメラ時刻 = LiDAR時刻 + time_offset
            t = m.header.stamp.to_sec() - time_offset
            T_wi = interp(t)
            if T_wi is None:
                continue
            T_wc = T_wi.dot(T_imu_cam)
            if last_T is not None:
                d = np.linalg.norm(T_wc[:3, 3] - last_T[:3, 3])
                ang = np.degrees(Rotation.from_matrix(last_T[:3, :3].T.dot(T_wc[:3, :3])).magnitude())
                if d < args.kf_dist and ang < args.kf_angle:
                    continue
            if compressed:
                img = cv2.imdecode(np.frombuffer(m.data, np.uint8), cv2.IMREAD_COLOR)
            else:
                img = decode_image(m)
            if args.min_sharpness > 0:
                sharp = cv2.Laplacian(cv2.cvtColor(img, cv2.COLOR_BGR2GRAY), cv2.CV_64F).var()
                if sharp < args.min_sharpness:
                    continue
            kfs.append((t, T_wc, img))
            last_T = T_wc
    log("画像 {} 枚中 {} 枚をキーフレームに選択 (軌跡の時間範囲内・移動量条件)".format(n_total, len(kfs)))
    if len(kfs) > args.max_keyframes:
        idx = np.linspace(0, len(kfs) - 1, args.max_keyframes).round().astype(int)
        kfs = [kfs[i] for i in idx]
        log("上限 --max-keyframes により {} 枚に間引き".format(len(kfs)))
    return kfs


# ----------------------------------------------------------------------------- メッシュ
def build_mesh(pcd_path, traj_pos, out_path, args):
    pcd = o3d.io.read_point_cloud(pcd_path)
    log("地図点群 {} 点を読み込み".format(len(pcd.points)))
    if len(pcd.points) == 0:
        raise RuntimeError("scans.pcd が空です")
    pcd = pcd.voxel_down_sample(args.voxel)
    pcd, _ = pcd.remove_statistical_outlier(nb_neighbors=20, std_ratio=2.0)
    log("ダウンサンプル後 {} 点 (voxel={} m)".format(len(pcd.points), args.voxel))
    pcd.estimate_normals(o3d.geometry.KDTreeSearchParamHybrid(radius=args.voxel * 4, max_nn=30))
    # 法線を最寄りの軌跡位置 (=センサ位置) の方向に向ける
    pts = np.asarray(pcd.points)
    nrm = np.asarray(pcd.normals)
    _, idx = cKDTree(traj_pos).query(pts)
    flip = np.einsum("ij,ij->i", nrm, traj_pos[idx] - pts) < 0
    nrm[flip] *= -1
    pcd.normals = o3d.utility.Vector3dVector(nrm)

    mesh, dens = o3d.geometry.TriangleMesh.create_from_point_cloud_poisson(pcd, depth=args.poisson_depth)
    dens = np.asarray(dens)
    mesh.remove_vertices_by_mask(dens < np.quantile(dens, args.density_quantile))
    # Poisson が点の無い所に張った膜を除去 (元の点から離れた頂点を削除)
    d, _ = cKDTree(pts).query(np.asarray(mesh.vertices))
    mesh.remove_vertices_by_mask(d > args.voxel * args.trim_factor)
    if len(mesh.triangles) > args.max_faces:
        mesh = mesh.simplify_quadric_decimation(args.max_faces)
    # OpenMVS TextureMesh は非多様体メッシュで std::out_of_range により異常終了するため除去しておく
    mesh.remove_duplicated_vertices()
    mesh.remove_duplicated_triangles()
    mesh.remove_degenerate_triangles()
    mesh.remove_non_manifold_edges()
    mesh.remove_unreferenced_vertices()
    mesh.compute_vertex_normals()
    o3d.io.write_triangle_mesh(out_path, mesh, write_ascii=False)
    log("メッシュ: 頂点 {} / 面 {} -> {}".format(len(mesh.vertices), len(mesh.triangles), out_path))
    return pcd


# ----------------------------------------------------------------------------- COLMAP
def write_colmap(kfs, K, D, size, colmap_dir, scale):
    img_dir = os.path.join(colmap_dir, "images")
    sparse = os.path.join(colmap_dir, "sparse")
    os.makedirs(img_dir, exist_ok=True)
    os.makedirs(sparse, exist_ok=True)
    w, h = size
    Ks = K.copy()
    if scale != 1.0:
        Ks[:2] *= scale
        w, h = int(round(w * scale)), int(round(h * scale))
    with open(os.path.join(sparse, "cameras.txt"), "w") as f:
        f.write("# CAMERA_ID MODEL WIDTH HEIGHT PARAMS[]\n")
        f.write("1 PINHOLE {} {} {:.6f} {:.6f} {:.6f} {:.6f}\n".format(w, h, Ks[0, 0], Ks[1, 1], Ks[0, 2], Ks[1, 2]))
    undist = []
    with open(os.path.join(sparse, "images.txt"), "w") as f:
        f.write("# IMAGE_ID QW QX QY QZ TX TY TZ CAMERA_ID NAME\n# POINTS2D[] (empty)\n")
        for i, (t, T_wc, img) in enumerate(kfs):
            if img.shape[1] != size[0] or img.shape[0] != size[1]:
                raise RuntimeError("画像サイズ {}x{} が calib.yaml の camera.width/height {}x{} と一致しません"
                                   .format(img.shape[1], img.shape[0], size[0], size[1]))
            u = cv2.undistort(img, K, D, None, K) if np.any(D) else img
            if scale != 1.0:
                u = cv2.resize(u, (w, h), interpolation=cv2.INTER_AREA)
            name = "{:05d}_{:.6f}.jpg".format(i, t)
            cv2.imwrite(os.path.join(img_dir, name), u, [cv2.IMWRITE_JPEG_QUALITY, 95])
            T_cw = C.inv(T_wc)
            qx, qy, qz, qw = Rotation.from_matrix(T_cw[:3, :3]).as_quat()
            tx, ty, tz = T_cw[:3, 3]
            f.write("{} {:.9f} {:.9f} {:.9f} {:.9f} {:.6f} {:.6f} {:.6f} 1 {}\n\n".format(
                i + 1, qw, qx, qy, qz, tx, ty, tz, name))
            undist.append((T_cw, u))
    open(os.path.join(sparse, "points3D.txt"), "w").write("# empty (mesh is given to TextureMesh directly)\n")
    return Ks, (w, h), undist


def run(cmd, cwd):
    log("$ " + " ".join(cmd))
    subprocess.run(cmd, cwd=cwd, check=True)


def openmvs_texture(colmap_dir, mesh_path, mvs_dir, args):
    os.makedirs(mvs_dir, exist_ok=True)
    threads = ["--max-threads", str(args.threads)] if args.threads else []
    run(["InterfaceCOLMAP", "-i", colmap_dir, "-o", os.path.join(mvs_dir, "scene.mvs"),
         "--image-folder", os.path.join(colmap_dir, "images")] + threads, cwd=mvs_dir)
    base = ["TextureMesh", os.path.join(mvs_dir, "scene.mvs"), "--mesh-file", mesh_path,
            "-o", os.path.join(mvs_dir, "textured.mvs"), "--export-type", args.export_type,
            "--resolution-level", str(args.texture_resolution_level),
            "--cost-smoothness-ratio", str(args.cost_smoothness_ratio)] + threads
    try:
        run(base, cwd=mvs_dir)
    except subprocess.CalledProcessError as e:
        # シーム平滑化で落ちることがあるので、無効にして再試行する
        log("TextureMesh が失敗 ({})。シーム平滑化を無効にして再試行します".format(e.returncode))
        run(base + ["--global-seam-leveling", "0", "--local-seam-leveling", "0"], cwd=mvs_dir)


# ----------------------------------------------------------------------------- 色付き点群
def colorize(points, undist, Ks, size, args):
    w, h = size
    acc = np.zeros((len(points), 3))
    wsum = np.zeros(len(points))
    cell = max(1, int(args.zbuf_cell))
    gw, gh = (w + cell - 1) // cell, (h + cell - 1) // cell
    for T_cw, img in undist:
        pc = points.dot(T_cw[:3, :3].T) + T_cw[:3, 3]
        z = pc[:, 2]
        valid = (z > 0.3) & (z < args.color_max_dist)
        idx = np.nonzero(valid)[0]
        if idx.size == 0:
            continue
        u = Ks[0, 0] * pc[idx, 0] / z[idx] + Ks[0, 2]
        v = Ks[1, 1] * pc[idx, 1] / z[idx] + Ks[1, 2]
        inb = (u >= 0) & (u < w - 1) & (v >= 0) & (v < h - 1)
        idx, u, v = idx[inb], u[inb], v[inb]
        if idx.size == 0:
            continue
        # 簡易デプステスト: 粗いグリッドごとの最小深度より十分奥の点は遮蔽とみなす
        ci = (v.astype(int) // cell) * gw + (u.astype(int) // cell)
        zb = np.full(gw * gh, np.inf)
        np.minimum.at(zb, ci, z[idx])
        vis = z[idx] <= zb[ci] * (1.0 + args.occlusion_ratio) + 0.05
        idx, u, v = idx[vis], u[vis], v[vis]
        col = img[v.round().astype(int), u.round().astype(int)].astype(float)  # BGR
        # 画像中心に近く、近距離の観測ほど重くする
        r = np.hypot((u - w / 2) / (w / 2), (v - h / 2) / (h / 2))
        wt = (1.0 / np.maximum(z[idx], 0.5)) * np.clip(1.2 - r, 0.1, 1.0)
        np.add.at(acc, idx, col * wt[:, None])
        np.add.at(wsum, idx, wt)
    has = wsum > 0
    rgb = np.zeros((len(points), 3))
    rgb[has] = acc[has] / wsum[has, None]
    return np.clip(rgb[:, ::-1] / 255.0, 0.0, 1.0), has


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out-dir", required=True, help="slam-replay の出力ディレクトリ (scans.pcd, slam_output.bag)")
    ap.add_argument("--bag", required=True, help="入力 bag (カメラ画像を含む ROS1 bag)")
    ap.add_argument("--calib", required=True)
    ap.add_argument("--map", help="地図 PCD (既定: <out-dir>/scans.pcd)")
    ap.add_argument("--odom-bag", help="/Odometry を含む bag (既定: <out-dir>/slam_output.bag)")
    ap.add_argument("--odom-topic", default="/Odometry")
    ap.add_argument("--image-topic", help="既定: calib.yaml の topics.image")
    ap.add_argument("--voxel", type=float, default=0.05, help="メッシュ化前のダウンサンプル [m]")
    ap.add_argument("--poisson-depth", type=int, default=10)
    ap.add_argument("--density-quantile", type=float, default=0.05, help="低密度頂点の除去割合")
    ap.add_argument("--trim-factor", type=float, default=3.0, help="元点群から voxel*この倍率以上離れた頂点を除去")
    ap.add_argument("--max-faces", type=int, default=2_000_000)
    ap.add_argument("--kf-dist", type=float, default=0.3, help="キーフレーム間隔 (移動量) [m]")
    ap.add_argument("--kf-angle", type=float, default=10.0, help="キーフレーム間隔 (回転量) [deg]")
    ap.add_argument("--max-keyframes", type=int, default=300)
    ap.add_argument("--min-sharpness", type=float, default=0.0, help="ラプラシアン分散がこれ未満のブレ画像を除外 (0=無効)")
    ap.add_argument("--image-scale", type=float, default=1.0, help="テクスチャ用画像の縮小率")
    ap.add_argument("--export-type", default="obj", choices=["obj", "ply", "glb", "gltf"])
    ap.add_argument("--texture-resolution-level", type=int, default=0)
    ap.add_argument("--cost-smoothness-ratio", type=float, default=0.1)
    ap.add_argument("--color-voxel", type=float, default=0.02, help="色付き点群の点間隔 [m]")
    ap.add_argument("--color-max-dist", type=float, default=30.0, help="色付けに使う最大距離 [m]")
    ap.add_argument("--zbuf-cell", type=int, default=8, help="デプステストのグリッド [px]")
    ap.add_argument("--occlusion-ratio", type=float, default=0.05)
    ap.add_argument("--threads", type=int, default=0)
    ap.add_argument("--skip-mesh", action="store_true", help="メッシュ化と OpenMVS を行わない (色付き点群のみ)")
    args = ap.parse_args()

    t0 = time.time()
    out = os.path.abspath(args.out_dir)
    map_path = args.map or os.path.join(out, "scans.pcd")
    odom_bag = args.odom_bag or os.path.join(out, "slam_output.bag")
    c = C.load(args.calib)
    cam = c["camera"]
    topic = args.image_topic or c["topics"]["image"]
    K, D = cam["K"], cam["D"]
    size = (int(cam["width"]), int(cam["height"]))
    tex_dir = os.path.join(out, "texture")
    if os.path.isdir(tex_dir):
        shutil.rmtree(tex_dir)
    os.makedirs(tex_dir)
    summary = {"map": map_path, "odom_bag": odom_bag, "image_topic": topic}

    ts, pos, quat = load_odometry(odom_bag, args.odom_topic)
    log("軌跡 {} 姿勢 ({:.1f} s, 全長 {:.1f} m)".format(
        len(ts), ts[-1] - ts[0], np.linalg.norm(np.diff(pos, axis=0), axis=1).sum()))
    interp = PoseInterp(ts, pos, quat)

    kfs = select_keyframes(args.bag, topic, interp, c["T_imu_cam"], float(cam.get("time_offset", 0.0)), args)
    if not kfs:
        raise RuntimeError("キーフレームがありません (画像時刻と軌跡の時刻範囲が重なっているか確認)")
    colmap_dir = os.path.join(tex_dir, "colmap")
    Ks, ssize, undist = write_colmap(kfs, K, D, size, colmap_dir, args.image_scale)
    summary["keyframes"] = len(kfs)

    mesh_path = os.path.join(out, "mesh.ply")
    if not args.skip_mesh:
        build_mesh(map_path, pos, mesh_path, args)
        mvs_dir = os.path.join(tex_dir, "mvs")
        openmvs_texture(colmap_dir, mesh_path, mvs_dir, args)
        # 成果物を出力ディレクトリ直下に集める
        tdir = os.path.join(out, "textured_mesh")
        os.makedirs(tdir, exist_ok=True)
        for f in os.listdir(mvs_dir):
            if f.startswith("textured") and not f.endswith(".mvs") and not f.endswith(".log"):
                shutil.copy2(os.path.join(mvs_dir, f), tdir)
        summary["textured_mesh"] = sorted(os.listdir(tdir))

    pcd = o3d.io.read_point_cloud(map_path).voxel_down_sample(args.color_voxel)
    pts = np.asarray(pcd.points)
    rgb, has = colorize(pts, undist, Ks, ssize, args)
    cpcd = o3d.geometry.PointCloud(o3d.utility.Vector3dVector(pts[has]))
    cpcd.colors = o3d.utility.Vector3dVector(rgb[has])
    o3d.io.write_point_cloud(os.path.join(out, "colored_map.ply"), cpcd)
    o3d.io.write_point_cloud(os.path.join(out, "colored_map.pcd"), cpcd)
    log("色付き点群: {} / {} 点に色が付きました -> colored_map.ply/.pcd".format(int(has.sum()), len(pts)))
    summary.update({"colored_points": int(has.sum()), "total_points": len(pts), "elapsed_s": round(time.time() - t0, 1)})
    with open(os.path.join(out, "texture_summary.json"), "w") as f:
        json.dump(summary, f, indent=2, ensure_ascii=False)
    log("完了 ({:.0f} s)".format(time.time() - t0))


if __name__ == "__main__":
    try:
        main()
    except Exception as e:  # noqa: BLE001
        log("ERROR:", e)
        raise
