# drone-slam-docker

ドローン (Livox Mid-360S + カメラ + FC) で記録した rosbag をリプレイしながら、LiDAR SLAM で 3D マップ (色付き点群・テクスチャ付きメッシュ) を作るための Docker イメージ集です。
同じ bag と同じキャリブレーションファイルを 5 つの手法に流し、結果をブラウザで並べて比較できます。

- 比較ビューア (公開版): <https://bols-blue.github.io/drone-slam-docker/>
- イメージ: `ghcr.io/bols-blue/drone-slam:<タグ>` (認証なしで pull できます)

## インストール

Linux で Docker (sudo なしで実行できること) と Python 3.8 以上が使える環境が前提です。

```bash
curl -fsSL https://raw.githubusercontent.com/bols-blue/drone-slam-docker/main/install.sh | bash
```

リポジトリを `~/.local/share/drone-slam-docker` に取得し、`~/.local/bin` に次のコマンドを置きます (clone 済みなら `./install.sh` でも可)。Docker イメージは初回実行時に必要な分だけ自動で取得されます (`--pull` で全部を先に取得。合計 約 25 GB)。

| コマンド | 内容 |
|---|---|
| `drone-slam-bag2html <bag> --calib my_calib.yaml` | bag から比較ビューア (`./output/<bag名>/report.html`) まで一括で作る |
| `drone-slam-view <bag>` | bag のカメラ画像と LiDAR 点群 (Mid-360S / XT16) を RViz2 で見る |
| `drone-slam-run <手法\|all> <bag> --calib my_calib.yaml` | 手法を指定して bag をリプレイする |
| `drone-slam-update` | リポジトリと取得済みイメージを最新にする |

アンインストールは `install.sh --uninstall` です (コマンドだけを削除します)。

## クイックスタート

```bash
cp ~/.local/share/drone-slam-docker/common/config/calib.yaml ./my_calib.yaml   # 実機の値に書き換える (後述)
drone-slam-bag2html ~/drone_slam_bags/raw_20260925_004138 --calib ./my_calib.yaml
# → ./output/raw_20260925_004138/report.html をブラウザで開く
```

`bag2html.py` は次の順に処理します。

1. ROS2 bag (mcap / db3) を ROS1 bag に変換する (2 回目以降は変換済みを再利用)
2. bag の中身を調べる: カメラ画像・FC の IMU の有無、IMU の単位 (g / m/s²)、LiDAR の天地逆取り付け
3. 意味のある結果が出る手法だけを選んで、順にリプレイする (選ばなかった理由はログと `summary.md` に出る)
4. 地図・メッシュ・軌跡をビューア用に軽量化し、`report.html` (データと three.js を埋め込んだ 1 ファイル。オフラインでも開ける) と `summary.md` を書き出す

所要時間の目安は「bag の長さ × 手法数 + 変換の数分」です (243 秒の bag で 2 手法: 約 19 分)。`--duration 60` で先頭だけ試せます。主なオプション: `--algos fast-livo2,r3live` `--out` `--rate` `--start` `--duration` `--title` `--no-pull` (`python3 bag2html.py --help`)。

## イメージ一覧

| タグ | 手法 | 系統 | ROS | 主な出力 | 画像なしの bag |
|---|---|---|---|---|---|
| `fast-livo2` | [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) | ① LIV 密結合 (ESIKF) | Noetic | 色付き点群、軌跡、(任意) COLMAP 形式 | LIO のみで動く (`img_en: 0`、地図は反射強度付き) |
| `r3live` | [R3LIVE](https://github.com/hku-mars/r3live) | ① LIV 密結合 | Melodic | 色付き点群、テクスチャ付きメッシュ、軌跡 | 軌跡のみ (地図の保存に画像が必要) |
| `sr-livo` | [SR-LIVO](https://github.com/ZikangYuan/sr_livo) | ① LIV 密結合 | Noetic | 色付き点群、軌跡 | 出力なし (画像が無いと処理が始まらない) |
| `lvi-sam` | [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM) ([Easyused 版](https://github.com/Cc19245/LVI-SAM-Easyused)) | ① LIV 因子グラフ (ループ閉じ込みあり) | Melodic | 点群 (色なし)、軌跡 | 動く (姿勢付き 9 軸 IMU が必須) |
| `fast-lio2-openmvs` | [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) + [OpenMVS](https://github.com/cdcseacave/openMVS) | ② LIO + 後処理テクスチャ | Noetic | 点群、メッシュ、テクスチャ付きメッシュ、色付き点群 | 点群と軌跡まで (テクスチャ付けは画像が必要) |
| `tools` | ROS2 bag → ROS1 bag 変換 | — | Python | ROS1 bag | — |
| `ros2-viewer` | RViz2 での bag 表示 (`drone-slam-view`) | — | Jazzy | — | — |

各手法の出力ファイル、チューニング項目、注意点は各ディレクトリ (`fast_livo2/` `r3live/` `sr_livo/` `lvi_sam/` `fast_lio2_openmvs/`) の README にあります。

## キャリブレーションファイル (calib.yaml)

[`common/config/calib.yaml`](common/config/calib.yaml) をコピーして実機の値に書き換えます。全手法がこの 1 ファイルから自分の設定ファイルを自動生成します (生成結果は出力の `generated_config/` に残ります)。座標変換は `T_a_b` (b 座標系の点を a 座標系に写す: p_a = R · p_b + t、R は行優先 9 要素) で書きます。

| キー | 内容 |
|---|---|
| `topics.lidar` / `imu` / `image` / `fc_imu` | 入力トピック |
| `lidar.scan_lines` / `blind` / `det_range` | ライン数 (Mid-360S は 4)、除外する近距離 / 最大距離 [m] |
| `imu.acc_in_g` | IMU 加速度が g 単位か (Mid-360S 内蔵 IMU は true) |
| `T_imu_lidar` | LiDAR 点 → 内蔵 IMU 座標 (Mid-360 の公式値が入っている) |
| `camera` | 解像度、内部パラメータ、歪み (plumb_bob)、`time_offset` (カメラ時刻 − LiDAR 時刻 [s]) |
| `T_cam_lidar` | LiDAR 点 → カメラ座標 (OpenCV 流: x 右 / y 下 / z 光軸)。逆向きの値は `T_lidar_cam:` と書けば自動で反転する |
| `T_fcimu_lidar` | LiDAR 点 → FC の IMU 座標 (LVI-SAM のみ) |
| `fast_livo2:` `r3live:` `sr_livo:` `lvi_sam:` | 手法固有パラメータの上書き |

テンプレートのカメラ関係の値は仮値です。カメラ内部パラメータ (チェッカーボード) と LiDAR-カメラ外部パラメータ ([direct_visual_lidar_calibration](https://koide3.github.io/direct_visual_lidar_calibration/) の `T_lidar_camera` を `T_lidar_cam:` に貼る) を実測するまで、色付け・テクスチャは正しくなりません。

LiDAR を天地逆に取り付けた場合は `T_cam_lidar` / `T_fcimu_lidar` の回転にその向きを含めます (`T_imu_lidar` は LiDAR 内部の関係なので変わりません)。重力の向きは各手法が起動時に IMU から合わせますが、FAST-LIO2 は世界座標が起動時の IMU の向きのままなので、地図が上下逆さまに出ます (`bag2html.py` は表示時に自動で反転します)。

## Hesai XT16 で使う

XT16 は IMU を内蔵しないため、FC (ArduPilot) の IMU (`/mavros/imu/data_raw`) と組み合わせます。テンプレートは [`common/config/calib_xt16.yaml`](common/config/calib_xt16.yaml) です。

```bash
cp ~/.local/share/drone-slam-docker/common/config/calib_xt16.yaml ./my_calib_xt16.yaml   # FC と XT16 の位置関係を書き換える
drone-slam-bag2html <XT16 の bag> --calib ./my_calib_xt16.yaml
```

- **点群の変換**: `tools` が Hesai の点群 (`/lidar_points`、x y z intensity ring timestamp) を自動で検出し、Livox 形式 (`/lidar_points/livox`) に変換します。5 手法とも Livox 形式を共通に読めるためです (R3LIVE は Velodyne 形式の入力が未実装)
- **時刻**: XT16 の時刻を記録時刻と比べて自動で扱いを決めます (`--lidar-time-align`)。PTP で同期済みならそのまま、一定のずれなら一律に補正、時計の進み方がずれていれば (PTP が Locked になっていない等) 記録時刻を基準に付け直します。PTP の設定は drone_slam の `docs/time_sync.md`
- **IMU**: 100 Hz 以上必要です (ArduPilot は RAW_IMU の送信レートを上げる)。50 Hz 未満の bag は理由を表示して止めます。起動直後に MAVROS が出す単位換算前の値 (mg) は外れ値として除きます (`--imu-outlier-ratio`)
- **取り付けの向き**: `T_imu_lidar` は FC の機体座標 (x 前 / y 左 / z 上) から見た XT16 の向きと位置です。`bag2html.py` は XT16 のライン番号と点の高さから天地逆の取り付けを判定し、`T_imu_lidar` と合っていなければ警告します (天地逆なら R = `[1, 0, 0, 0, -1, 0, 0, 0, -1]`)
- **LVI-SAM**: 姿勢付きの `/mavros/imu/data` も 100 Hz 以上必要です (ArduPilot は ATTITUDE を上げる)。足りなければ自動で省きます
- 2026-10-14 の試験記録 (静止 26 秒) では、FAST-LIVO2 / FAST-LIO2 / R3LIVE が静止を正しく推定しました (始点からのずれ 0〜3 cm)。SR-LIVO はカメラの外部パラメータが仮値のままでは発散するので、カメラのキャリブレーション後に評価してください

## 個別に実行する

```bash
# 1 手法だけ / 全手法 (ROS2 bag は自動で ROS1 に変換。出力は ./output/<手法>_<日時>/)
./run.sh fast-livo2 ~/drone_slam_bags/texture_20261010_101010 --calib ./my_calib.yaml
./run.sh all ~/drone_slam_bags/texture_20261010_101010 --calib ./my_calib.yaml

# docker run を直接使う場合
docker run --rm -u $(id -u) -v ~/bags:/data ghcr.io/bols-blue/drone-slam:tools \
  /data/texture_20261010_101010 /data/flight.bag [--imu-ms2-copy /livox/imu]
docker run --rm -v ~/bags:/data:ro -v $PWD/my_calib.yaml:/config/calib.yaml:ro -v $PWD/output:/output \
  ghcr.io/bols-blue/drone-slam:fast-livo2 --bag /data/flight.bag
```

- 各イメージの ENTRYPOINT (`slam-replay`) のオプション: `--rate` `--start` `--duration` `--tail` (再生後の待ち秒数) `--rviz` (X11 が必要) `--no-record`
- `tools` の主なオプション: `--imu-ms2-copy <topic>` (g 単位の IMU を m/s² に換算した `<topic>_ms2` を追加。LVI-SAM に内蔵 IMU を入れる場合。既に m/s² なら作らない)、`--livox-time-align` (PTP 同期していない Mid-360S の時刻を補正。既定は自動判定)、`--hesai-topics` / `--keep-hesai-source` / `--lidar-time-align` (Hesai 点群の Livox 形式への変換と時刻の扱い)、`--imu-outlier-ratio` (IMU の外れ値除去)、`--image-every N` (画像の間引き)
- 点群・メッシュは CloudCompare や MeshLab でも開けます

## 比較ビューア

[`viewer/`](viewer/) はブラウザで複数手法の地図 (点群・メッシュ・軌跡) を並べて比較するビューアです。`bag2html.py` を使わずに手で作る場合:

```bash
# 点群 + 軌跡 (tools イメージで動く)
docker run --rm -u $(id -u) -v $PWD:/w -w /w --entrypoint python ghcr.io/bols-blue/drone-slam:tools \
  viewer/export_viewer_data.py --out viewer/data \
  --run fast-livo2:FAST-LIVO2:output/fast_livo2_xxx/map_rgb.pcd:output/fast_livo2_xxx/trajectory_tum.txt
# メッシュ (open3d が必要。fast-lio2-openmvs イメージで動く)
docker run --rm -u $(id -u) -v $PWD:/w -w /w --entrypoint python3 ghcr.io/bols-blue/drone-slam:fast-lio2-openmvs \
  viewer/export_mesh.py --data viewer/data --id r3live --mesh output/r3live_xxx/textured_mesh.ply
python3 -m http.server -d viewer 8000   # http://localhost:8000
```

- 表示用に 1 手法あたり最大 45 万点へ間引き、メッシュはテクスチャを頂点色に焼き込んで約 22 万面以下にしています。細部は元のファイルで確認してください
- 公開版 (GitHub Pages) は `viewer/**` を main に push すると [`pages.yml`](.github/workflows/pages.yml) が再公開します。データセットは `viewer/data/<id>/` に置き、`viewer/data/datasets.json` に登録します (`#<id>` で直接開けます)
- 公開版のうち R3LIVE dataset 由来の地図は元データと同じ CC BY-NC-SA 4.0 (非商用) です

## bag の画像と点群を RViz2 で見る

```bash
ros2_tools/view_bag_images.sh ~/drone_slam_bags/raw_20260928_064026          # Mid-360S + カメラ
ros2_tools/view_bag_images.sh ~/claud_code/drone_slam_比較/raw_20261014_014135  # XT16 + カメラ
ros2_tools/view_bag_images.sh <bag> --rate 0.5 --xt16-tf "0 0 -0.1 0 0 0"
```

ROS2 bag をループ再生しながら、RViz2 の 3D ビューに LiDAR 点群 (反射強度で着色) を、左のパネルにカメラ画像を表示します。終了はウィンドウを閉じるか Ctrl+C です。

- LiDAR は bag に入っている方を自動で選びます。Mid-360S (`livox_ros_driver2/CustomMsg`) は `livox_to_pointcloud2.py` で PointCloud2 に変換し、XT16 は `/lidar_points` (無ければ最初の PointCloud2) をそのまま表示します。`--cloud <topic>` で表示する点群を追加できます
- 表示の基準は `base_link` です。`--livox-tf` / `--xt16-tf` に `"x y z roll pitch yaw"` (m, rad) で LiDAR の取り付けを渡します (既定: Mid-360S は天地逆 `"0 0 0 3.14159 0 0"`、XT16 は `"0 0 0 0 0 0"`)
- その他のオプション: `--image <topic>` `--rate <倍速>` `--decay <秒>` (Mid-360S の点群を残す時間。既定 0.5)
- 必要なもの: X11 ディスプレイ (Wayland でも XWayland があれば可)。表示には `ghcr.io/bols-blue/drone-slam:ros2-viewer` (RViz2・image_transport_plugins・livox_ros_driver2 メッセージ入りの ROS2 Jazzy イメージ) を使います (`DRONE_SLAM_ROS2_IMAGE` で変更可)
- コンテナはネットワークから切り離して動かすので、同じ PC で動いている ROS2 (実機の記録など) とは混線しません
- usb_cam を `pixel_format: yuyv` で使って記録した圧縮画像は format が `yuv422_yuy2; jpeg compressed bgr8` になり、RViz2 の標準の展開では表示できません。`compressed_relay.py` で JPEG を展開し直して表示します

## 記録側で必要なトピック

| トピック | 型 | 用途 |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` (`xfer_format: 1`) | 全手法。点ごとの時刻が必要なので PointCloud2 ではなく CustomMsg で記録する |
| `/livox/imu` | `sensor_msgs/Imu` | 全手法 (LVI-SAM は FC の IMU が無い場合のみ) |
| `/lidar_points` (XT16) | `sensor_msgs/PointCloud2` (x y z intensity ring timestamp) | XT16 構成の全手法 (tools が Livox 形式に変換)。記録時に 0 件になっていないか `ros2 bag info` で確認する |
| `/mavros/imu/data_raw` (XT16) | `sensor_msgs/Imu` | XT16 構成の全手法。100 Hz 以上 (ArduPilot の RAW_IMU) |
| `/image_raw` (または `/image_raw/compressed`) | `Image` / `CompressedImage` | 色付け・テクスチャ。10 Hz 以上を推奨 |
| `/mavros/imu/data` | `sensor_msgs/Imu` | LVI-SAM (姿勢付き 9 軸 IMU、100 Hz 以上) |

## 実機データで分かったこと (2026-09 の記録)

| 項目 | 結果 | 対応 |
|---|---|---|
| LiDAR-PC 時刻同期 | Livox のヘッダ時刻は記録時刻と 0.11 s (1 スキャン分) の差で揃っている | 補正不要 |
| LiDAR の欠落 | 2026-09-25 04:25 以降の 4 本は 1.4〜2.2 Hz しか記録されていない (本来 10 Hz)。正常な bag にも 1.4 s の欠落が 1 か所 | 記録側のネットワーク・CPU 負荷を確認する |
| 取り付け | IMU の重力が (0, 0, −1 g) で、Mid-360S は天地逆 | calib の回転に反映。`bag2html.py` は自動判定 |
| カメラ | `raw_20260928_064026` の画像は `/image_raw/compressed` が 1 Hz のみ。format に `yuv422_yuy2` が付く | 10 Hz 以上で記録する。usb_cam は `pixel_format: yuyv2rgb` (または mjpeg) にすると format の問題も解消する |
| FC の IMU | 同じ bag で `/mavros/imu/data` が 3.9 Hz | LVI-SAM には 100 Hz 以上が必要。MAVLink のストリームレートを上げる |
| SLAM 結果 (`raw_20260925_004138`, 243 s, 画像なし) | FAST-LIVO2 と FAST-LIO2 の軌跡の差は RMS 2.0 cm、R3LIVE とも 5〜6 cm。離陸後に高度約 4.2 m で最大約 20 m 移動 | [公開版ビューア](https://bols-blue.github.io/drone-slam-docker/) で確認できる |

公開データセット [R3LIVE dataset](https://github.com/ziv-lin/r3live_dataset) の `hku_campus_seq_00` (Livox Avia + カメラ、202 s) では、5 手法すべてでリプレイと地図出力 (色付き点群、R3LIVE と OpenMVS はテクスチャ付きメッシュ) を確認しています。

## ハードウェア要件

| 項目 | 必要なもの | 備考 |
|---|---|---|
| カメラ同期 | 外部トリガ (LiDAR の PPS 等と同期) が理想 | USB カメラは時刻が PC 受信時刻になる。FAST-LIVO2 / SR-LIVO は時刻ずれを推定しないので色ずれしやすく、R3LIVE と LVI-SAM はオンライン推定がある |
| シャッター | グローバルシャッター | どの手法もローリングシャッターをモデル化していない |
| 露光 | 固定露光 | FAST-LIVO2 には露光推定があるが、固定の方が安定 |
| IMU | 内蔵 6 軸 IMU (LVI-SAM 以外) / 姿勢付き 9 軸 IMU 100 Hz 以上 (LVI-SAM) | |
| 計算機 | 4 コア以上、RAM 16 GB 以上、ディスクは bag の 3〜5 倍。GPU 不要 | R3LIVE の中間ファイルと FAST-LIVO2 の全点 PCD は数百 MB〜GB になる |

## ディレクトリ構成

| パス | 内容 |
|---|---|
| `bag2html.py`, `tools/bag2html_helper.py` | bag から比較ビューアまでの一括処理 |
| `run.sh` | 手法を指定してリプレイするホスト側ラッパー |
| `common/` | 全イメージ共通: `calib.yaml` テンプレート、`slam-replay` (ENTRYPOINT)、`calib.py`、`livox_ros_driver` メッセージ定義、CustomMsg → PointCloud2 変換ノード |
| `tools/` | ROS2 → ROS1 bag 変換 (`tools` イメージ) |
| `fast_livo2/` `r3live/` `sr_livo/` `lvi_sam/` `fast_lio2_openmvs/` | 手法ごとの Dockerfile、設定生成 (`gen_config.py`)、起動・保存手順 (`profile.sh`)、上流へのパッチ、README |
| `viewer/` | 比較ビューア (GitHub Pages で公開) と書き出しスクリプト |
| `ros2_tools/` | ROS2 bag の画像と点群 (Mid-360S / XT16) を RViz2 で見るスクリプトと、その表示用イメージ (`ros2-viewer`) |
| `install.sh` | インストーラ (コマンドを `~/.local/bin` に置く) |
| `.github/workflows/` | イメージのビルド・公開 (`build.yml`)、ビューアの公開 (`pages.yml`) |

## ビルド

main への push で GitHub Actions ([`build.yml`](.github/workflows/build.yml)) が全イメージをビルドし、`ghcr.io/<owner>/drone-slam:<tag>` と `:<tag>-<commit>` に push します (`viewer/` と `*.md` だけの変更ではビルドしません)。ローカルでビルドする場合:

```bash
docker build -t drone-slam:fast-livo2 -f fast_livo2/Dockerfile --build-arg MAKE_JOBS=8 .
```

上流リポジトリはすべてコミット SHA (OpenMVS は v2.2.0) で固定しています。

## ライセンス

このリポジトリのスクリプト類は MIT です。イメージに含まれる各ソフトウェアはそれぞれのライセンスに従います: FAST-LIVO2 / R3LIVE / SR-LIVO / FAST-LIO2 は GPL-2.0、LVI-SAM は BSD-3-Clause、OpenMVS は AGPL-3.0。FAST-LIVO2 と R3LIVE の作者は商用利用について別途連絡を求めており、R3LIVE は無償利用を個人・学術に限定しています。業務で使う前に各上流の README を確認してください。
