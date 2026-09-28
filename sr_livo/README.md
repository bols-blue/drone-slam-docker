# SR-LIVO イメージ (`ghcr.io/bols-blue/drone-slam:sr-livo`)

[SR-LIVO](https://github.com/ZikangYuan/sr_livo)（R3LIVE ベースの LiDAR-慣性-視覚オドメトリ、Sweep Reconstruction）を
ROS Noetic 上でビルドし、rosbag リプレイで色付き点群地図を作るイメージです。

- upstream: `ZikangYuan/sr_livo` @ `97a26660ca0d3723e1556b447d3b7d34f2866555`
- ベース: `ros:noetic-perception`（Ubuntu 20.04 / PCL 1.10 / OpenCV 4.2 / Ceres 1.14）

## 実行

```bash
docker run --rm -v /path/to/bags:/data -v $PWD/calib.yaml:/config/calib.yaml:ro -v $PWD/output:/output \
  ghcr.io/bols-blue/drone-slam:sr-livo --bag /data/flight.bag
```

入力トピックは共通 `calib.yaml` の `topics.lidar`（`livox_ros_driver/CustomMsg`）、`topics.imu`、`topics.image`（`sensor_msgs/Image`）です。
ROS2 bag は先に `tools` イメージで ROS1 bag に変換してください。

## 出力 (`/output/sr_livo_<日時>/`)

| ファイル | 内容 |
|---|---|
| `rgb_map.pcd` | 色付き点群地図（PointXYZRGB、終了時に保存） |
| `pose.txt` | 軌跡、TUM 形式（`time tx ty tz qx qy qz qw`）。IMU の世界座標系での姿勢 |
| `velocity.txt`, `bias.txt` | 速度と IMU バイアスの推定値 |
| `slam_output.bag` | `/Odometry_after_opt`, `/path` の記録 |
| `generated_config/sr_livo.yaml` | calib.yaml から生成した SR-LIVO のパラメータ |

## パラメータ

`calib.yaml` から自動生成します。外部パラメータの対応は次のとおりです（ソースで確認済み）。

- `extrinsic_R/t_imu_lidar` ← `T_imu_lidar`（p_imu = R·p_lidar + t）
- `extrinsic_R/t_imu_camera` ← `T_imu_cam`（IMU 座標系でのカメラ姿勢）

調整したいパラメータは、`calib.yaml` に `sr_livo:` セクションを追加すると上書きできます。例:

```yaml
sr_livo:
  common: {point_filter_num: 2}
  map_options: {pub_point_minimum_views: 3}
```

## ビルド時に当てているパッチ (`patch_sr_livo.py`)

1. `imu_parameter/acc_scale` を追加。SR-LIVO は加速度を m/s² 前提で扱い、スケール補正をしません。一方 Mid-360S の内蔵IMUは g 単位なので、既定で 9.81 倍します（`imu.acc_in_g`）。
2. `camera_parameter/time_offset` を追加（`camera.time_offset`）。画像時刻から差し引いて LiDAR の時計に合わせます。
3. 色付き点が 0 点のときに PCD 保存が例外で abort しないようにしました。
4. 可視化スレッドが `while(1)` のままだと SIGINT 後に終了しないため、`ros::ok()` で抜けるようにしました。

## 注意点

- **ループクロージャはありません**（論文でも今後の課題とされています）。長距離の飛行ではドリフトが蓄積します。
- LiDAR・カメラ・IMU 間の時間オフセットはオンライン推定しません。カメラとLiDARのハードウェア同期（または正確な `time_offset`）が必要です。
- メッシュ化やテクスチャリングのユーティリティはありません（出力は色付き点群のみ）。メッシュが必要な場合は R3LIVE か FAST-LIO2+OpenMVS のイメージを使ってください。
- LIO のパラメータ（voxel / ICP）は同梱 `config/r3live.yaml`（Livox Avia 用）の値をそのまま使っています。Mid-360S 用には未調整です。
