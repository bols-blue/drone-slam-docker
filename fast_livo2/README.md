# FAST-LIVO2 (`drone-slam:fast-livo2`)

[hku-mars/FAST-LIVO2](https://github.com/hku-mars/FAST-LIVO2) (ROS1 Noetic) を使います。LiDAR・IMU・カメラを密結合した LIV-SLAM で、色付き点群地図を作ります。

## 実行例

```bash
docker run --rm \
  -v /path/to/bags:/data:ro \
  -v /path/to/config:/config:ro \
  -v /path/to/output:/output \
  ghcr.io/bols-blue/drone-slam:fast-livo2 --bag /data/flight.bag
```

`/config/calib.yaml` には共通フォーマットのキャリブレーションファイルを置きます(`common/config/calib.yaml` 参照)。そこから FAST-LIVO2 用の `fast_livo2.yaml` と `camera.yaml` が自動生成され、`generated_config/` に保存されます。

## 出力 (`/output/fast_livo2_<日時>/`)

| ファイル | 内容 |
|---|---|
| `map_rgb.pcd` | 色付き点群地図(ボクセル間引き済み。既定 0.1 m) |
| `map_rgb_raw.pcd` | 間引き前の全点。大きくなります(200 秒で約 700 MB) |
| `trajectory_tum.txt` | IMU の軌跡(TUM 形式: `t x y z qx qy qz qw`) |
| `lidar_poses.txt` | LiDAR フレームごとの IMU 姿勢 |
| `slam_output.bag` | `/aft_mapped_to_init` (nav_msgs/Odometry) |
| `colmap/` | `-e FAST_LIVO2_COLMAP=1` を付けたときだけ出力。COLMAP 形式(画像・姿勢・点)で、OpenMVS などでメッシュ化・テクスチャ付けに使えます |
| `slam.log`, `generated_config/` | ログと、実際に使われた設定 |

## 主なチューニング項目

`calib.yaml` に `fast_livo2:` セクションを書くと、任意のパラメータを上書きできます。

```yaml
fast_livo2:
  lio:
    voxel_size: 1.0          # 広い屋外では 1.0〜2.0 (上流の UAV 設定 MARS_LVIG は 2.0)
  preprocess:
    filter_size_surf: 0.2    # 入力点の間引き。処理が追いつかないときは大きくする
    point_filter_num: 2
  pcd_save:
    filter_size_pcd: 0.05    # map_rgb.pcd の解像度
  vio:
    img_point_cov: 1000      # 画像の信頼度 (大きいほど画像の重みが小さい)
  time_offset:
    exposure_time_init: 0.0
  camera:
    scale: 0.5               # 画像処理の解像度倍率
```

- 画像トピックに `/compressed` しか無い bag の場合は、自動で raw に展開します。
- `lidar.scan_lines` の既定値は 4 (Mid-360S) です。Avia の場合は 6 にします。
- 処理が実時間に追いつかない場合は `--rate 0.5` を指定します。

## Mid-360S + USB カメラでの注意点

- **時刻同期が最重要です。** FAST-LIVO2 には画像とLiDARの時刻ずれをオンラインで推定する機能がありません。`camera.time_offset` に固定値を入れるだけです。作者の機材は、LiDAR(PPS)とカメラ(トリガ)をハードウェアで同期しています。USB (UVC) カメラの時刻は PC が受信した時刻なので、数十 ms の遅れや揺らぎが出ます。この場合、色ずれや VIO の発散につながります。
  - Mid-360S は PTP(gPTP)で PC 時刻に同期させてください。同期しない場合、LiDAR の時刻は Mid-360S 起動からの経過時間になります(`tools` の変換で補正できますが、精度は落ちます)。
  - カメラは、外部トリガ入力を持つグローバルシャッタ機が推奨です。
- **ローリングシャッタは考慮されていません。** 高速な機体の動きでは、画像側の誤差になります。
- **露光:** 自動露光でも `exposure_estimate_en: true` である程度対応できます。ただし、固定露光のほうが安定します。
- **FC の IMU は使いません。** Mid-360S の内蔵 IMU を使います(加速度が g 単位でも、初期化時にノルムで正規化されるので問題ありません)。
- **カメラの歪み:** Pinhole モデルは k1, k2, p1, p2 のみを使い、k3 は無視されます。
- **ループクロージャはありません。** 長距離の飛行ではドリフトが蓄積します。

## 実装メモ

- 上流の `-march=native` は `-march=nehalem -mtune=generic` に置き換えています(他の CPU で Illegal instruction にならないようにするため)。変更するには `--build-arg CPU_ARCH_FLAGS=...` を指定します。
- 上流はビルド時のソースディレクトリの `Log/` に書き込みます。そのため実行時には、`Log/` を出力ディレクトリへのシンボリックリンクに差し替えています。
- 地図は、ノードが SIGINT で終了するときに保存されます。`slam-replay` はノードに直接 SIGINT を送り、保存が終わるまで待ちます。
- 上流の `pcl::VoxelGrid` は、地図が広いとインデックスがオーバーフローして間引きをスキップします。そのため `voxel_downsample.py` で間引き直しています。
