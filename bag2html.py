#!/usr/bin/env python3
"""rosbag を渡すと、変換 → 各 SLAM 手法でリプレイ → 比較ビューア (HTML) 作成までを一括で行う。

  python3 bag2html.py ~/drone_slam_bags/raw_20260925_004138 --calib configs/my_calib.yaml

出力 (既定: ./output/<bag名>/):
  report.html       データを埋め込んだ 1 ファイルのビューア (ダブルクリックで開ける)
  viewer/           Web サーバに置く版 (index.html + data/)
  summary.md        手法ごとの結果一覧
  runs/<手法>_<日時>/  各手法の生の出力 (地図 PCD / メッシュ / 軌跡 / ログ)
  logs/             各ステップのログ

必要なもの: Docker と Python 3.8 以上 (標準ライブラリのみ)。イメージは初回に自動で pull される。
"""
import argparse
import base64
import datetime
import json
import os
import re
import shutil
import subprocess
import sys
import time
import urllib.request
from pathlib import Path

REPO = Path(__file__).resolve().parent
ALL_ALGOS = ["fast-livo2", "fast-lio2-openmvs", "r3live", "sr-livo", "lvi-sam"]
NAMES = {"fast-livo2": "FAST-LIVO2", "fast-lio2-openmvs": "FAST-LIO2 + OpenMVS", "r3live": "R3LIVE",
         "sr-livo": "SR-LIVO", "lvi-sam": "LVI-SAM"}
OUT_PREFIX = {"fast-livo2": "fast_livo2_", "fast-lio2-openmvs": "fast_lio2_openmvs_", "r3live": "r3live_",
              "sr-livo": "sr_livo_", "lvi-sam": "lvi_sam_"}
DESC = {
    "fast-livo2": "LiDAR・IMU・画像を ESIKF で 1 つのボクセル地図に統合。",
    "fast-lio2-openmvs": "LiDAR-慣性 SLAM の後に、キーフレーム画像を投影して色付け・テクスチャ付け (OpenMVS)。",
    "r3live": "LIO で形状、VIO で色を付ける 2 系統構成。カメラの時刻ずれと内部・外部パラメータをオンライン推定。",
    "sr-livo": "R3LIVE 系をスイープ再構成で高速化。ループ閉じ込みなし。",
    "lvi-sam": "因子グラフ + ループ閉じ込み。地図に色は付かない。",
}
CDN_LIBS = [
    "https://cdn.jsdelivr.net/npm/three@0.147.0/build/three.min.js",
    "https://cdn.jsdelivr.net/npm/three@0.147.0/examples/js/controls/OrbitControls.js",
]


def log(msg):
    print("[bag2html] " + msg, flush=True)


def run(cmd, logfile=None, check=True):
    """コマンドを実行。logfile があれば出力をそこに書く"""
    if logfile:
        with open(logfile, "w") as f:
            r = subprocess.run(cmd, stdout=f, stderr=subprocess.STDOUT)
    else:
        r = subprocess.run(cmd)
    if check and r.returncode != 0:
        tail = ""
        if logfile:
            tail = "\n".join(Path(logfile).read_text(errors="ignore").splitlines()[-15:])
        raise RuntimeError("コマンドが失敗しました (exit %d): %s\n%s" % (r.returncode, " ".join(map(str, cmd)), tail))
    return r.returncode


def uid_gid():
    return "%d:%d" % (os.getuid(), os.getgid())


def calib_topic(calib_path, key, default):
    """PyYAML を使わずに calib.yaml の topics.<key> を読む (最初に見つかったもの)"""
    m = re.search(r"^\s+%s:\s*([^\s#]+)" % re.escape(key), Path(calib_path).read_text(), re.M)
    return m.group(1) if m else default


def newest_dir(root, prefix, since):
    cands = [p for p in Path(root).glob(prefix + "*") if p.is_dir() and p.stat().st_mtime >= since - 5]
    return max(cands, key=lambda p: p.stat().st_mtime) if cands else None


def find_outputs(algo, d):
    """手法の出力ディレクトリから (地図, 軌跡, メッシュ) を探す。無ければ None"""
    def first(*names):
        for n in names:
            p = d / n
            if p.exists() and p.stat().st_size > 0:
                return p
        return None
    if algo == "fast-livo2":
        return first("map_rgb.pcd", "map_rgb_raw.pcd"), first("trajectory_tum.txt"), None
    if algo == "fast-lio2-openmvs":
        return (first("colored_map.pcd", "scans.pcd"), first("trajectory_tum.txt"),
                first("textured_mesh/textured.obj", "mesh.ply"))
    if algo == "r3live":
        return first("map/rgb_pt.pcd"), first("trajectory_imu_tum.txt"), first("map/textured_mesh.ply")
    if algo == "sr-livo":
        return first("rgb_map.pcd"), first("pose.txt"), None
    if algo == "lvi-sam":
        return first("GlobalMap.pcd"), first("trajectory_lio_tum.txt"), None
    return None, None, None


def traj_stats(path):
    rows = []
    for line in Path(path).read_text().splitlines():
        p = line.replace(",", " ").split()
        if len(p) >= 4 and not line.startswith("#"):
            try:
                rows.append([float(v) for v in p[:4]])
            except ValueError:
                pass
    if len(rows) < 2:
        return None
    length, maxstep, maxabs = 0.0, 0.0, 0.0
    for a, b in zip(rows, rows[1:]):
        s = sum((b[i] - a[i]) ** 2 for i in (1, 2, 3)) ** 0.5
        length += s
        maxstep = max(maxstep, s)
    for r in rows:
        maxabs = max(maxabs, max(abs(r[1]), abs(r[2]), abs(r[3])))
    return {"poses": len(rows), "length_m": round(length, 1), "duration_s": round(rows[-1][0] - rows[0][0], 1),
            "max_step_m": round(maxstep, 2), "max_abs_m": round(maxabs, 1)}


def inline_libs(html, cache_dir):
    """three.js を HTML に埋め込む (オフラインでも開けるように)。取得できなければ CDN 参照のまま"""
    cache_dir.mkdir(parents=True, exist_ok=True)
    for url in CDN_LIBS:
        tag = '<script src="%s"></script>' % url
        if tag not in html:
            continue
        f = cache_dir / url.split("/npm/")[1].replace("/", "_")
        try:
            if not f.exists():
                with urllib.request.urlopen(url, timeout=30) as r:
                    f.write_bytes(r.read())
            code = f.read_text().replace("</script", "<\\/script")
            html = html.replace(tag, "<script>\n%s\n</script>" % code, 1)
        except Exception as e:  # noqa: BLE001
            log("WARNING: %s を埋め込めませんでした (表示にはインターネット接続が必要): %s" % (url, e))
    return html


def build_report(viewer_dir, out_html, title, cache_dir):
    """viewer/ (index.html + data/) からデータ埋め込みの 1 ファイル HTML を作る"""
    html = (REPO / "viewer" / "index.html").read_text()
    emb = {}
    for p in sorted((viewer_dir / "data").rglob("*")):
        if p.is_file():
            key = "data/" + p.relative_to(viewer_dir / "data").as_posix()
            emb[key] = json.loads(p.read_text()) if p.suffix == ".json" else base64.b64encode(p.read_bytes()).decode()
    html = re.sub(r"<title>.*?</title>", "<title>%s</title>" % title, html, count=1)
    html = html.replace("<h1>SLAM Map Bench<small>drone-slam-docker</small></h1>",
                        "<h1>SLAM Map Bench<small>%s</small></h1>" % title, 1)
    payload = json.dumps(emb, ensure_ascii=False).replace("</", "<\\/")
    marker = '<script src="https://cdn.jsdelivr.net/npm/three@'
    i = html.index(marker)
    html = html[:i] + "<script>window.EMBEDDED_DATA = %s;</script>\n" % payload + html[i:]
    html = inline_libs(html, cache_dir)
    out_html.write_text(html)


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("bag", help="ROS2 bag ディレクトリ / .mcap / .db3、または ROS1 .bag")
    ap.add_argument("--calib", help="共通キャリブレーション (既定: common/config/calib.yaml の仮値)")
    ap.add_argument("--out", help="出力ディレクトリ (既定: ./output/<bag名>)")
    ap.add_argument("--algos", default="auto",
                    help="実行する手法をカンマ区切りで (%s)。auto は bag の中身から選ぶ" % ",".join(ALL_ALGOS))
    ap.add_argument("--rate", type=float, default=1.0, help="再生速度 (処理が追いつかなければ 0.5 など)")
    ap.add_argument("--start", type=float, help="再生開始オフセット [s]")
    ap.add_argument("--duration", type=float, help="再生する秒数")
    ap.add_argument("--image", default=os.environ.get("DRONE_SLAM_IMAGE", "ghcr.io/bols-blue/drone-slam"),
                    help="Docker イメージ名 (タグを除く)")
    ap.add_argument("--title", help="ビューアのタイトル (既定: bag 名)")
    ap.add_argument("--no-pull", action="store_true", help="開始時に Docker イメージを最新に更新しない")
    ap.add_argument("--timeout", type=int, default=7200, help="1 手法あたりの制限時間 [s]")
    args = ap.parse_args()

    if shutil.which("docker") is None:
        sys.exit("docker が見つかりません")
    src = Path(args.bag).expanduser().resolve()
    if not src.exists():
        sys.exit("bag が見つかりません: %s" % src)
    name = src.name
    for ext in (".mcap", ".db3", ".bag"):
        if name.endswith(ext):
            name = name[: -len(ext)]
    out = Path(args.out).expanduser().resolve() if args.out else (Path.cwd() / "output" / name)
    calib = Path(args.calib).expanduser().resolve() if args.calib else REPO / "common" / "config" / "calib.yaml"
    if not args.calib:
        log("WARNING: --calib が無いので仮値の calib.yaml を使います (色付け・テクスチャは正しくなりません)")
    for d in ("runs", "logs", "work"):
        (out / d).mkdir(parents=True, exist_ok=True)
    img = args.image
    t_start = time.time()
    log("出力先: %s" % out)

    pulled = set()

    def pull(tag):
        if args.no_pull or tag in pulled:
            return
        pulled.add(tag)
        log("イメージを確認中: %s:%s" % (img, tag))
        run(["docker", "pull", "-q", "%s:%s" % (img, tag)], logfile=out / "logs" / ("pull_%s.log" % tag), check=False)

    # ---- 1. ROS1 bag に変換 ----
    is_ros1 = src.is_file() and src.suffix == ".bag"
    if is_ros1:
        bag = src
    else:
        bag = out / "work" / (name + ".ros1.bag")
        if bag.exists():
            log("変換済みの bag を使います: %s" % bag)
        else:
            imu_topic = calib_topic(calib, "imu", "/livox/imu")
            pull("tools")
            log("ROS2 bag を ROS1 bag に変換中 (数分かかります)")
            tmp = bag.with_suffix(".tmp.bag")
            if tmp.exists():
                tmp.unlink()
            run(["docker", "run", "--rm", "-u", uid_gid(), "-v", "%s:/src:ro" % src.parent, "-v", "%s:/work" % bag.parent,
                 "%s:tools" % img, "/src/" + src.name, "/work/" + tmp.name, "--imu-ms2-copy", imu_topic],
                logfile=out / "logs" / "convert.log")
            tmp.rename(bag)
    bag_dir, bag_name = bag.parent, bag.name

    # ---- 2. bag の中身を調べて手法と calib を決める ----
    pull("fast-livo2")
    log("bag の中身を確認中")
    shutil.copy(calib, out / "work" / "calib_input.yaml")
    helper = REPO / "tools" / "bag2html_helper.py"
    run(["docker", "run", "--rm", "-u", uid_gid(), "-v", "%s:/bag:ro" % bag_dir, "-v", "%s:/work" % (out / "work"),
         "-v", "%s:/opt/bag2html_helper.py:ro" % helper, "--entrypoint", "bash", "%s:fast-livo2" % img, "-c",
         "source /opt/ros/noetic/setup.bash && python3 /opt/bag2html_helper.py probe --bag /bag/%s "
         "--calib /work/calib_input.yaml --out-dir /work" % bag_name],
        logfile=out / "logs" / "probe.log")
    probe = json.loads((out / "work" / "probe.json").read_text())
    for w in probe["warnings"]:
        log("WARNING: " + w)
    for n in probe["notes"]:
        log(n)
    log("bag: %.0f 秒, 画像 %s, FC IMU %s" % (probe["duration_s"], probe["image"] or "なし",
                                            "あり" if probe["has_fc_imu"] else "なし"))
    if args.algos == "auto":
        algos = probe["algos"]
        for a, why in probe["skip"].items():
            log("%s は実行しません: %s" % (NAMES[a], why))
    else:
        algos = [a.strip() for a in args.algos.split(",") if a.strip()]
        bad = [a for a in algos if a not in ALL_ALGOS]
        if bad:
            sys.exit("不明な手法: %s" % ", ".join(bad))
        if "lvi-sam" in algos and not (out / "work" / "calib_lvi.yaml").exists():
            log("WARNING: LVI-SAM 用の IMU が無いため、通常の calib で実行します")

    # ---- 3. 各手法でリプレイ ----
    results = []
    replay_opts = ["--rate", str(args.rate)]
    if args.start is not None:
        replay_opts += ["--start", str(args.start)]
    if args.duration is not None:
        replay_opts += ["--duration", str(args.duration)]
    for i, algo in enumerate(algos, 1):
        c = out / "work" / ("calib_lvi.yaml" if algo == "lvi-sam" and (out / "work" / "calib_lvi.yaml").exists()
                            else "calib_main.yaml")
        pull(algo)
        log("(%d/%d) %s を実行中 ..." % (i, len(algos), NAMES[algo]))
        t0 = time.time()
        rc = None
        try:
            rc = subprocess.run(
                ["docker", "run", "--rm", "-v", "%s:/data:ro" % bag_dir, "-v", "%s:/config/calib.yaml:ro" % c,
                 "-v", "%s:/output" % (out / "runs"), "%s:%s" % (img, algo), "--bag", "/data/" + bag_name] + replay_opts,
                stdout=open(out / "logs" / ("%s.log" % algo), "w"), stderr=subprocess.STDOUT,
                timeout=args.timeout).returncode
        except subprocess.TimeoutExpired:
            log("WARNING: %s が制限時間 (%d 秒) を超えました" % (NAMES[algo], args.timeout))
        d = newest_dir(out / "runs", OUT_PREFIX[algo], t0)
        res = {"algo": algo, "name": NAMES[algo], "exit": rc, "dir": str(d) if d else None,
               "elapsed_s": round(time.time() - t0), "status": "失敗", "notes": []}
        if d:
            if algo == "fast-lio2-openmvs" and not (d / "trajectory_tum.txt").exists() and (d / "slam_output.bag").exists():
                run(["docker", "run", "--rm", "-u", uid_gid(), "-v", "%s:/d" % d, "-v",
                     "%s:/opt/bag2html_helper.py:ro" % helper, "--entrypoint", "bash", "%s:fast-livo2" % img, "-c",
                     "source /opt/ros/noetic/setup.bash && python3 /opt/bag2html_helper.py odom2tum "
                     "--bag /d/slam_output.bag --out /d/trajectory_tum.txt"], logfile=out / "logs" / "odom2tum.log",
                    check=False)
            cloud, traj, mesh = find_outputs(algo, d)
            res.update({"cloud": str(cloud) if cloud else None, "traj": str(traj) if traj else None,
                        "mesh": str(mesh) if mesh else None})
            st = traj_stats(traj) if traj else None
            res["traj_stats"] = st
            if cloud and st:
                res["status"] = "成功"
                if st["max_abs_m"] > 500 or st["max_step_m"] > 5:
                    res["status"] = "発散"
                    res["notes"].append("軌跡が発散しています (最大 %.0f m、1 ステップ最大 %.1f m)" % (st["max_abs_m"], st["max_step_m"]))
            elif st:
                res["status"] = "軌跡のみ"
            if algo == "fast-lio2-openmvs" and probe["has_image"] and not (d / "textured_mesh").exists():
                res["notes"].append("テクスチャ付けに失敗 (texture.log 参照)")
        log("    -> %s (%d 秒)" % (res["status"], res["elapsed_s"]))
        results.append(res)

    # ---- 4. ビューア用データ ----
    viewer = out / "viewer"
    if viewer.exists():
        shutil.rmtree(viewer)
    (viewer / "data").mkdir(parents=True)
    shows = [r for r in results if r["status"] == "成功"]
    if shows:
        pull("tools")
        log("ビューア用データを書き出し中")
        no_img = "" if probe["has_image"] else " カメラ画像なし (LiDAR+IMU のみ) のため色は高さです。"
        specs, flips = [], []
        for r in shows:
            st = r["traj_stats"]
            note = DESC[r["algo"]] + no_img + " 軌跡長 %.1f m。" % st["length_m"]
            if r["algo"] == "fast-lio2-openmvs" and probe["inverted"]:
                flips.append(r["algo"])
                note += " 重力整列しないため表示用に上下反転。"
            specs += ["--run", "%s:%s:%s:%s:%s" % (r["algo"], r["name"], "/work/" + str(Path(r["cloud"]).relative_to(out)),
                                                  "/work/" + str(Path(r["traj"]).relative_to(out)), note)]
        ds = "入力: <b>%s</b> · %.0f s · %s" % (src.name, probe["duration_s"],
                                               "画像 " + probe["image"] if probe["has_image"] else "カメラ画像なし")
        notes = ["同じ bag を各手法の Docker イメージで %.1f 倍速リプレイして作った地図です (%s)。" %
                 (args.rate, datetime.date.today().isoformat())]
        for r in results:
            if r["status"] != "成功":
                notes.append("%s: %s%s" % (r["name"], r["status"], "。" + " ".join(r["notes"]) if r["notes"] else ""))
        for a, why in (probe["skip"].items() if args.algos == "auto" else []):
            notes.append("%s は実行していません: %s" % (NAMES[a], why))
        notes += probe["notes"]
        cmd = ["docker", "run", "--rm", "-u", uid_gid(), "-v", "%s:/work" % out, "-v", "%s:/v:ro" % (REPO / "viewer"),
               "--entrypoint", "python", "%s:tools" % img, "/v/export_viewer_data.py", "--out", "/work/viewer/data",
               "--dataset", ds] + sum([["--note", n] for n in notes], []) + specs
        if flips:
            cmd += ["--flip-ids"] + flips
        run(cmd, logfile=out / "logs" / "export.log")
        for r in shows:
            if r.get("mesh"):
                pull("fast-lio2-openmvs")
                log("メッシュを軽量化中: %s" % r["name"])
                run(["docker", "run", "--rm", "-u", uid_gid(), "-v", "%s:/work" % out, "-v",
                     "%s:/v:ro" % (REPO / "viewer"), "--entrypoint", "python3", "%s:fast-lio2-openmvs" % img,
                     "/v/export_mesh.py", "--data", "/work/viewer/data", "--id", r["algo"],
                     "--mesh", "/work/" + str(Path(r["mesh"]).relative_to(out))],
                    logfile=out / "logs" / ("mesh_%s.log" % r["algo"]), check=False)
        shutil.copy(REPO / "viewer" / "index.html", viewer / "index.html")
        title = args.title or name
        log("report.html を作成中")
        build_report(viewer, out / "report.html", title, Path.home() / ".cache" / "drone-slam-docker")
    else:
        log("WARNING: 地図を出力できた手法が無いため、ビューアは作りません")

    # ---- 5. まとめ ----
    lines = ["# %s" % name, "", "| 手法 | 結果 | 地図 | 軌跡長 | 処理時間 | 備考 |", "| --- | --- | --- | --- | --- | --- |"]
    for r in results:
        st = r.get("traj_stats") or {}
        lines.append("| %s | %s | %s | %s | %d 秒 | %s |" % (
            r["name"], r["status"], Path(r["cloud"]).name if r.get("cloud") else "—",
            "%.1f m" % st["length_m"] if st else "—", r["elapsed_s"], " ".join(r["notes"])))
    for a, why in (probe["skip"].items() if args.algos == "auto" else []):
        lines.append("| %s | 未実行 | — | — | — | %s |" % (NAMES[a], why))
    if probe["warnings"]:
        lines += ["", "注意:"] + ["- " + w for w in probe["warnings"]]
    (out / "summary.md").write_text("\n".join(lines) + "\n")
    (out / "summary.json").write_text(json.dumps({"probe": probe, "results": results}, ensure_ascii=False, indent=1))
    print()
    print("\n".join(lines))
    print()
    log("完了 (%d 分)。" % round((time.time() - t_start) / 60))
    if shows:
        log("ビューア: %s" % (out / "report.html"))


if __name__ == "__main__":
    main()
