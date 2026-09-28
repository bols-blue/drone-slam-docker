# R3LIVE イメージ (`drone-slam:r3live`)

[hku-mars/r3live](https://github.com/hku-mars/r3live)（commit `6143a38`、ROS Melodic）を使い、
ROS1 bag をリプレイしながら **色付き点群** を作り、終了後にオフラインユーティリティで **テクスチャ付きメッシュ** まで一括生成します。

## 実行

```bash
docker run --rm -v /path/to/bags:/data -v /path/to/calib.yaml:/config/calib.yaml:ro -v $PWD/output:/output \
  ghcr.io/bols-blue/drone-slam:r3live --bag /data/flight.bag
```

ROS2 bag は、先に `tools` イメージで ROS1 bag に変換してください。

## 出力 (`/output/r3live_<日時>/`)

| ファイル | 内容 |
|---|---|
| `colored_map.pcd` → `map/rgb_pt.pcd` | RGB 色付き点群 (3 視点以上で色が付いた点) |
| `textured_mesh.ply` → `map/textured_mesh.ply` | 頂点色付きメッシュ (`r3live_meshing`) |
| `map/reconstructed_mesh.{ply,obj}` | 色なしメッシュ |
| `map/test.r3live` | オフライン地図 (画像と姿勢を含むため大きい)。メッシュ化の再実行に使う |
| `trajectory_imu_tum.txt` / `trajectory_camera_tum.txt` | 軌跡 (TUM 形式) |
| `slam_output.bag` | `/aft_mapped_to_init` `/camera_odom` `/path` `/camera_path` |
| `generated_config/r3live_config.yaml` | calib.yaml から生成した R3LIVE 設定 |
| `slam.log` / `mesh.log` | ログ |

## 設定

- 共通の `calib.yaml` から自動生成します。外部パラメータは R3LIVE の `camera_ext_R/t` = **T_imu_cam**（IMU 座標系でのカメラ姿勢）に変換して渡します。
- `calib.yaml` に `r3live:` セクションを書くと、生成された設定に上書きできます。
  ```yaml
  r3live:
    r3live_common: {estimate_intrinsic: 0, estimate_i2c_extrinsic: 1}
    r3live_lio: {filter_size_map: 0.2}
  ```
- 環境変数（`docker run -e`）
  - `R3LIVE_MESH=0`: メッシュ化をスキップする
  - `R3LIVE_MESH_ARGS="decimate_mesh:=0.5 texturing_smooth_factor:=5"`: メッシュ化パラメータを変更する
  - `R3LIVE_SAVE_TIMEOUT` / `R3LIVE_MESH_TIMEOUT`: 地図保存とメッシュ化のタイムアウト（秒）

## Mid-360S 向けに当てているパッチ (`patch_r3live.py`)

1. 元実装の「x > 0.7 m（前方）の点のみ使う」制限（Avia 前提）を距離判定に変更しました。これで 360° の点をすべて使えます。
2. LiDAR-IMU 並進は元実装では Avia の固定値でした。`T_imu_lidar.t` から設定できるようにしています（回転は単位行列を仮定）。
3. IMU 加速度を g → m/s² に変換します（Mid-360S の内蔵 IMU は g 単位。元実装は m/s² 入力が前提）。
4. 初期化時に重力方向へ姿勢を合わせます。元実装は、起動時に IMU が水平であることを前提にしていました。
5. `camera.time_offset` を、カメラ-IMU 時刻オフセット `td` の初期値として渡します。
6. 画面なし（headless）で動くようにしました。地図はトリガーファイルで自動保存します（キー入力の `s` や pcl_viewer は不要）。

## 注意点

- **時刻同期**: R3LIVE はカメラ-IMU 間の時刻オフセット `td` と外部パラメータをオンラインで推定するため、USB カメラのようにハードウェア同期されていない構成でも比較的動きます。ただしオフセットが数十 ms を超えると推定が破綻しやすいので、`calib.yaml` の `camera.time_offset` におおよその値を入れてください。Mid-360S を PTP 同期していない場合は、tools の変換時に時刻補正（`--livox-time-align`）が必要です。
- **ローリングシャッター**: モデル化していないので、高速に回転するとテクスチャがずれます。グローバルシャッターのカメラを推奨します。
- **ループクロージャなし**: 長距離の飛行ではドリフトがそのまま地図に残ります。
- **再生速度**: VIO が重いため、`--rate 0.5` 程度に下げると安定することがあります。
- **初期静止**: IMU の初期化に、bag の冒頭 1〜2 秒の静止区間が必要です。
- `test.r3live` には全キーフレーム画像が含まれるため、飛行時間に比例して大きくなります（数 GB 程度）。
