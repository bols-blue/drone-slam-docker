# LVI-SAM イメージ (`drone-slam:lvi-sam`)

LiDAR・カメラ・9軸IMUをファクターグラフ(GTSAM)で融合する LVI-SAM を、rosbag リプレイで実行するイメージです。

- ベース: [Cc19245/LVI-SAM-Easyused](https://github.com/Cc19245/LVI-SAM-Easyused) の `new` ブランチ(commit `29a21b3`)
  - 本家 [TixiaoShan/LVI-SAM](https://github.com/TixiaoShan/LVI-SAM) は LiDAR・IMU・カメラ間の座標関係が作者のセンサ構成向けにハードコードされていて、他の構成ではそのまま動きません。Easyused 版はこの外参を `T_imu_lidar` と `T_imu_cam` の2つで設定できるように直したもので、LIO 部分も Livox(`sensor: livox`)に対応した新しい LIO-SAM に更新されています。
  - 本イメージでの追加パッチ(`patches/`)
    - `0001`: `/lvi_sam/save_map` サービスを有効化し、保存先を絶対パスで指定できるようにする
    - `0002`: `imuHasOrientation: false` のとき、IMU の姿勢を単位クォータニオンとして扱う(6軸 IMU での検証用)
- 環境: Ubuntu 18.04 / ROS Melodic / GTSAM 4.0.2 / Ceres 1.14

## 実行

```bash
docker run --rm -v /path/to/bags:/data -v /path/to/calib.yaml:/config/calib.yaml:ro -v $PWD/output:/output \
  ghcr.io/bols-blue/drone-slam:lvi-sam --bag /data/flight.bag
```

入力トピック(`calib.yaml` の `topics`): `lidar`(livox CustomMsg)、`image`、`fc_imu`(既定 `/mavros/imu/data`)。
`/livox/lidar` はコンテナ内で `livox_to_pointcloud2` によって ring・time 付きの PointCloud2(`/livox/points`)に変換してから入力します。

## 出力 (`/output/lvi_sam_<日時>/`)

| ファイル | 内容 |
|---|---|
| `GlobalMap.pcd` | 地図点群(Corner＋Surf の特徴点。色なし) |
| `SurfMap.pcd` / `CornerMap.pcd` | 特徴点の種類別地図 |
| `trajectory.pcd` / `transformations.pcd` | キーフレームの位置・姿勢 |
| `trajectory_lio_tum.txt` / `trajectory_vio_tum.txt` | LIO/VIO の軌跡(TUM形式) |
| `slam_output.bag` | オドメトリ・パス(`LVI_RECORD_DENSE=1` を指定すると、地図座標に変換した全スキャン点群も記録) |

LVI-SAM は地図に**色を付けません**。出力されるのは形状と軌跡だけです。テクスチャを付ける場合は、軌跡と画像を fast-lio2-openmvs イメージの後処理(テクスチャマッピング)に渡してください。

環境変数:
- `LVI_IMU_SOURCE=fc|livox`(既定 `fc`)。`livox` では LiDAR 内蔵 6 軸 IMU を使い、IMU 姿勢による拘束をオフにします(本来の前提外。`topics.fc_imu` が `topics.imu` と同じ場合も自動でこのモードになります)。加速度が m/s² のデータに限ります。g 単位の Mid-360S 内蔵 IMU は不可です。
- `LVI_SAVE_RESOLUTION`: 保存時のボクセルサイズ [m](既定 0 = 間引きなし)
- `LVI_RECORD_DENSE=1`

## 必要なハードウェア・キャリブレーション(不足しやすい点)

1. **9軸IMU(姿勢出力付き)が必須です。** LVI-SAM は IMU の orientation を使うため、Mid-360S 内蔵 IMU(ICM40609、6軸で姿勢なし)では起動直後に `Invalid quaternion, please use a 9-axis IMU!` と表示されて停止します。そこで既定では FC の `/mavros/imu/data` を使います。
   - **レート**: LVI-SAM は 100〜200Hz 以上の IMU を前提にしています。MAVROS の既定(数十Hz)のままだと精度が落ちるため、FC 側で ATTITUDE_QUATERNION / HIGHRES_IMU のメッセージレートを上げてください(`mavros/set_message_interval` や PX4 の `MAV_x_RATE` 等)。
2. **FC IMU と LiDAR の外参 `T_fcimu_lidar`** を `calib.yaml` に追加してください(書式は `T_imu_lidar` と同じで、`p_fcimu = R * p_lidar + t`)。
   ```yaml
   T_fcimu_lidar:
     R: [1, 0, 0,  0, 1, 0,  0, 0, 1]
     t: [0.0, 0.0, -0.10]
   ```
   未設定の場合は Mid-360S 内蔵 IMU の `T_imu_lidar` で代用し、警告を出します。FC と LiDAR の取り付け向きが違うと LVI-SAM は発散します。カメラの外参は `T_fcimu_cam = T_fcimu_lidar · T_lidar_cam` として自動計算します。
3. **時刻同期**: FC IMU(MAVROS が PC 時刻に変換)、Mid-360S(PTP なしだと LiDAR 起動からの経過時間)、カメラ(PC 時刻)の時計が揃っている必要があります。tools イメージの変換で Mid-360S の時刻は受信時刻に合わせて補正されますが、転送遅延(数ms〜数十ms)は残ります。本来は PC を PTP マスターにして Mid-360S を同期(gPTP)させ、カメラもハードウェアトリガにするのが理想です。カメラと IMU の時刻差はオンライン推定(`estimate_td: 1`)を有効にしています。
4. **カメラ**: PINHOLE モデル(k1, k2, p1, p2)のみ対応で、k3 は無視されます。ローリングシャッターのカメラでは `camera.rolling_shutter: 1` と `rolling_shutter_tr` の設定を推奨します。

## 動作確認

R3LIVE dataset `hku_campus_seq_00`(Livox Avia・6ライン、6軸 IMU、1280×1024 カメラ、202 s)を `LVI_IMU_SOURCE=livox` 相当で最後まで処理できました。LIO の軌跡は 173.6 m で、開始点と終了点の差は 0.02 m(ループクロージャで閉合)、`GlobalMap.pcd` は 83 万点です。`lidar.scan_lines: 6` のように、ライン数は calib で変更できます。

## Mid-360S を使う場合の注意

- LVI-SAM のレンジ画像投影は回転式 LiDAR(N_SCAN × Horizon_SCAN)を前提にしています。本イメージでは Livox モード(`N_SCAN: 4`、`Horizon_SCAN: 6000`。列番号にはライン内の時系列順を使う)で動かしますが、非反復スキャンの Mid-360S では特徴点の抽出が回転式 LiDAR ほど安定しません。FAST-LIO2 や FAST-LIVO2 のような直接法と比べると、精度・ロバスト性は劣る可能性があります。
- 垂直 FOV は -7〜52° と上向きです。ドローン下方の船体を狙うなら、取り付け向きを考慮してください。
- 細かいパラメータは `calib.yaml` の `lvi_sam:` ブロックで上書きできます。
  ```yaml
  lvi_sam:
    lidar_params: {mappingSurfLeafSize: 0.2, loopClosureEnableFlag: false}
    camera_params: {estimate_td: 0}
  fc_imu_noise: {acc_noise: 0.01, gyr_noise: 0.002}
  ```
