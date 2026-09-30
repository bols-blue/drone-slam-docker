# drone-slam-docker

ドローン (Livox Mid-360S + カメラ + FC) で記録した rosbag をリプレイしながら、LiDAR SLAM でテクスチャ付き 3D マップを作るための Docker イメージ集です。
同じ bag と同じキャリブレーションファイルを複数の手法に流して、結果を比較できます。

| イメージ | 手法 | 系統 | ROS | 主な出力 |
|---|---|---|---|---|
| `ghcr.io/bols-blue/drone-slam:fast-livo2` | [FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) | ① LIV 密結合 (ESIKF) | Noetic | 色付き点群、軌跡、(任意) COLMAP 形式 |
| `ghcr.io/bols-blue/drone-slam:r3live` | [R3LIVE](https://github.com/hku-mars/r3live) | ① LIV 密結合 | Melodic | 色付き点群、テクスチャ付きメッシュ、軌跡 |
| `ghcr.io/bols-blue/drone-slam:sr-livo` | [SR-LIVO](https://github.com/ZikangYuan/sr_livo) | ① LIV 密結合 | Noetic | 色付き点群、軌跡 |
| `ghcr.io/bols-blue/drone-slam:lvi-sam` | [LVI-SAM](https://github.com/TixiaoShan/LVI-SAM) ([LVI-SAM-Easyused](https://github.com/Cc19245/LVI-SAM-Easyused) 版) | ① LIV 因子グラフ (ループ閉じ込みあり) | Melodic | 点群 (色なし)、軌跡 |
| `ghcr.io/bols-blue/drone-slam:fast-lio2-openmvs` | [FAST-LIO2](https://github.com/hku-mars/FAST_LIO) + [OpenMVS](https://github.com/cdcseacave/openMVS) | ② LIO + 後処理テクスチャ | Noetic | 点群、メッシュ、テクスチャ付きメッシュ、色付き点群 |
| `ghcr.io/bols-blue/drone-slam:tools` | ROS2 bag → ROS1 bag 変換 | — | (Python) | ROS1 bag |

各手法の詳細 (出力ファイル、チューニング項目、注意点) は各ディレクトリの README を見てください。

## 使い方

### 1. キャリブレーションファイルを用意する

[`common/config/calib.yaml`](common/config/calib.yaml) をコピーして、実機の値に書き換えます。全手法がこの 1 ファイルから各自の設定ファイルを自動生成します (生成結果は出力の `generated_config/` に残ります)。

- `T_cam_lidar`: LiDAR 点 → カメラ座標 (`p_cam = R p_lidar + t`)。direct_visual_lidar_calibration の `T_lidar_camera` は逆向きなので、その場合は `T_lidar_cam:` として書けば自動で反転します
- `T_imu_lidar`: LiDAR 点 → 内蔵 IMU 座標。Mid-360 の公式値が入っています
- `camera`: 内部パラメータと歪み (plumb_bob)、`time_offset` (カメラ時刻 − LiDAR 時刻)
- `T_fcimu_lidar`: LVI-SAM で FC の IMU を使う場合のみ
- 手法固有のパラメータは `fast_livo2:` `r3live:` `sr_livo:` などのセクションで上書きできます

### 2. 実行する

`drone_slam` (ROS2 Jazzy) で記録した bag (mcap) をそのまま渡せます。ROS1 bag への変換は自動で行われ、`<bag名>.ros1.bag` として bag の隣に保存されます。

```bash
# 1 手法だけ
./run.sh fast-livo2 ~/drone_slam_bags/texture_20261010_101010 --calib ./my_calib.yaml

# 全手法を順に実行 (出力は ./output/<手法>_<日時>/)
./run.sh all ~/drone_slam_bags/texture_20261010_101010 --calib ./my_calib.yaml

# 処理が追いつかない場合は再生を遅くする / 一部区間だけ
./run.sh r3live flight.ros1.bag --calib ./my_calib.yaml --rate 0.5 --start 30 --duration 120
```

`docker run` を直接使う場合:

```bash
# ROS2 bag → ROS1 bag
docker run --rm -u $(id -u) -v ~/bags:/data ghcr.io/bols-blue/drone-slam:tools \
  /data/texture_20261010_101010 /data/flight.bag

# リプレイ + 地図作成
docker run --rm -v ~/bags:/data:ro -v $PWD/my_calib.yaml:/config/calib.yaml:ro -v $PWD/output:/output \
  ghcr.io/bols-blue/drone-slam:fast-livo2 --bag /data/flight.bag
```

`slam-replay` (各イメージの ENTRYPOINT) のオプション: `--rate` `--start` `--duration` `--tail` (再生後の待ち秒数) `--rviz` (X11 が必要) `--no-record`。

### 3. 結果を見る

- 点群・メッシュは CloudCompare や MeshLab で開けます
- [`viewer/`](viewer/) はブラウザで複数手法の地図 (点群・メッシュ・軌跡) を並べて比較するビューアです。公開版: <https://bols-blue.github.io/drone-slam-docker/> (実飛行データと公開データセットの比較。`viewer/**` を main に push すると `.github/workflows/pages.yml` が再公開します)
- 公開版の公開データセット (R3LIVE dataset) 由来の地図は CC BY-NC-SA 4.0 (非商用) です。データセットを追加するときは `viewer/data/<id>/` に書き出し、`viewer/data/datasets.json` に登録します

```bash
# 点群 + 軌跡 (numpy のみ。tools イメージで動く)
docker run --rm -u $(id -u) -v $PWD:/w -w /w --entrypoint python ghcr.io/bols-blue/drone-slam:tools \
  viewer/export_viewer_data.py --out viewer/data \
  --run fast-livo2:FAST-LIVO2:output/fast_livo2_xxx/map_rgb.pcd:output/fast_livo2_xxx/trajectory_tum.txt \
  --run r3live:R3LIVE:output/r3live_xxx/map/rgb_pt.pcd:output/r3live_xxx/trajectory_imu_tum.txt
# メッシュ (open3d が必要。fast-lio2-openmvs イメージで動く)
docker run --rm -u $(id -u) -v $PWD:/w -w /w --entrypoint python3 ghcr.io/bols-blue/drone-slam:fast-lio2-openmvs \
  viewer/export_mesh.py --data viewer/data --id r3live --mesh output/r3live_xxx/textured_mesh.ply
python3 -m http.server -d viewer 8000   # http://localhost:8000 (#<データセットid> で切り替え)
```

  メッシュのテクスチャは頂点色に焼き込み、表示用に約 22 万面へ間引いています。細部は元の OBJ / PLY を MeshLab 等で確認してください

## 記録側で必要なトピック

`drone_slam_bringup` の `record_for_texture.launch.py` で録ったもので足ります。

| トピック | 型 | 用途 |
|---|---|---|
| `/livox/lidar` | `livox_ros_driver2/CustomMsg` (`xfer_format: 1`) | 全手法。点ごとの時刻が必要なので PointCloud2 ではなく CustomMsg で記録する |
| `/livox/imu` | `sensor_msgs/Imu` | 全手法 (LVI-SAM 以外) |
| `/image_raw` (または `.../compressed`) | `sensor_msgs/Image` / `CompressedImage` | 色付け・テクスチャ |
| `/mavros/imu/data` | `sensor_msgs/Imu` | LVI-SAM のみ (姿勢付き 9 軸 IMU が必須) |

## ハードウェア要件と、現在の構成で足りない点

現在の構成 (Mid-360S + USB カメラ (usb_cam, 1280×720) + FC) と照らし合わせた結果です。

| 項目 | 必要なもの | 現状 | 影響 |
|---|---|---|---|
| LiDAR-PC 時刻同期 | PTP (gPTP) または GPS PPS で Mid-360S を PC 時刻に同期 | 未確認 (同期していなければ LiDAR 時刻は起動からの経過時間) | **不足の可能性大**。`tools` が記録時刻との差で補正するが、数 ms〜数十 ms の誤差が残り、色ずれの原因になる。PC で `ptp4l` をマスタとして動かすのが最も簡単 |
| カメラ-LiDAR 時刻同期 | カメラの外部トリガ (LiDAR の PPS 等と同期) | USB (UVC) カメラ。時刻は PC 受信時刻 | **不足**。FAST-LIVO2 / SR-LIVO は時刻ずれを推定しないので色ずれ・VIO 発散が起きやすい。R3LIVE と LVI-SAM はオンライン推定があるので比較的強い |
| シャッター | グローバルシャッター | USB カメラ (ローリングシャッターの可能性) | どの手法もローリングシャッターをモデル化していない。機体が速く動くと画像側の誤差になる |
| 露光 | 固定露光 (または露光時間の記録) | 自動露光 (想定) | FAST-LIVO2 は露光推定があるが、固定露光の方が安定 |
| カメラ内部・外部パラメータ | チェッカーボードでの内部キャリブ + LiDAR-カメラ外部キャリブ | テンプレート値 | **必須**。未実施だと色付け・テクスチャは正しくならない (`direct_visual_lidar_calibration` 推奨) |
| IMU | 内蔵 IMU (6 軸) | Mid-360S 内蔵 ICM40609 | FAST-LIVO2 / R3LIVE / SR-LIVO / FAST-LIO2 は OK |
| 9 軸 IMU (LVI-SAM) | 姿勢付き IMU を 100 Hz 以上 + LiDAR との外部パラメータ + 時刻同期 | FC の `/mavros/imu/data` (既定レート 50 Hz 程度) | **不足**。LVI-SAM を使うなら MAVLink のストリームレートを上げ、FC-LiDAR 外部パラメータを求める必要がある。FC と Mid-360S の時刻も揃っている必要がある |
| 計算機 (後処理) | 4 コア以上、RAM 16 GB 以上、ディスクは bag の 3〜5 倍 | 開発 PC: 20 コア / 61 GB | 足りている。今回の範囲 (①②) は GPU 不要。R3LIVE の中間ファイル (`test.r3live`) と FAST-LIVO2 の全点 PCD は数百 MB〜GB になる |

## テスト状況

公開データセット [R3LIVE dataset](https://github.com/ziv-lin/r3live_dataset) の `hku_campus_seq_00` (Livox Avia + カメラ、ハードウェア同期なし、202 秒) で、全イメージのリプレイと地図出力を確認しています。このデータセットは CC BY-NC-SA 4.0 のため、リポジトリには含めていません。
Mid-360S の実機データではまだ検証していません。

## ビルド

GitHub Actions ([`.github/workflows/build.yml`](.github/workflows/build.yml)) が main への push で全イメージをビルドし、`ghcr.io/<owner>/drone-slam:<tag>` と `:<tag>-<commit>` に push します。ローカルでビルドする場合:

```bash
docker build -t drone-slam:fast-livo2 -f fast_livo2/Dockerfile --build-arg MAKE_JOBS=8 .
```

上流リポジトリはすべてコミット SHA で固定しています。各 Dockerfile を参照してください。

## ライセンス

このリポジトリのスクリプト類は MIT です。イメージに含まれる各ソフトウェアはそれぞれのライセンスに従います: FAST-LIVO2 / R3LIVE / SR-LIVO / FAST-LIO2 は GPL-2.0、LVI-SAM は BSD-3-Clause、OpenMVS は AGPL-3.0。R3LIVE と FAST-LIVO2 の作者は商用利用について別途連絡を求めているので、業務で使う前に各上流の README を確認してください。
