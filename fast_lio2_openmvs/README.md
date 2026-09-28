# fast-lio2-openmvs

FAST-LIO2（LiDAR-慣性SLAM）で地図と軌跡を作り、その後でカメラ画像を貼り付けてテクスチャ付きメッシュと色付き点群を作るイメージです（調査資料の方式②）。
SLAM と色付けを別々の工程に分けているので、どちらかだけをやり直すこともできます。

- イメージ: `ghcr.io/bols-blue/drone-slam:fast-lio2-openmvs`
- 上流: [hku-mars/FAST_LIO](https://github.com/hku-mars/FAST_LIO) `7cc4175`、[cdcseacave/openMVS](https://github.com/cdcseacave/openMVS) `v2.2.0`（CPU ビルド。v2.3 以降は focal の gcc9/OpenCV4.2 でビルドできない）、Open3D 0.18（CPU 版）

## 実行

```bash
docker run --rm -v $PWD/data:/data -v $PWD/config:/config -v $PWD/output:/output \
  ghcr.io/bols-blue/drone-slam:fast-lio2-openmvs --bag /data/flight.bag
```

`/config/calib.yaml`（共通のキャリブレーションファイル）から FAST-LIO の設定を自動で作ります。
環境変数 `TEXTURE=false` を付けると、後処理（テクスチャリング）を行わず SLAM だけを実行します。
後処理のオプションは `TEXTURE_ARGS="--kf-dist 0.5 --image-scale 0.5"` のように渡せます。

後処理だけをやり直す場合は、既存の出力ディレクトリを指定します。

```bash
docker run --rm -v $PWD:/work --entrypoint texture-pipeline ghcr.io/bols-blue/drone-slam:fast-lio2-openmvs \
  --out-dir /work/output/fast_lio2_openmvs_YYYYmmdd_HHMMSS --bag /work/data/flight.bag --calib /work/config/calib.yaml
```

## 出力 (`/output/fast_lio2_openmvs_<日時>/`)

| ファイル | 内容 |
|---|---|
| `scans.pcd` | FAST-LIO の地図（world = `camera_init` 座標系、全スキャンの歪み補正後点群） |
| `slam_output.bag` | `/Odometry`（IMU の位置姿勢）と `/path` |
| `mesh.ply` | Poisson 再構成メッシュ（テクスチャなし） |
| `textured_mesh/textured.obj` + `.mtl` + `*.png` | OpenMVS TextureMesh によるテクスチャ付きメッシュ |
| `colored_map.ply` / `.pcd` | 地図点をキーフレーム画像に投影して色を付けた点群 |
| `texture/colmap/` | 歪み補正済みのキーフレーム画像と COLMAP 形式のカメラ姿勢 |
| `texture/mvs/` | OpenMVS の作業ファイルとログ |
| `texture_summary.json`, `texture.log` | 後処理の概要とログ |

## 後処理の流れ（`texture_pipeline.py`）

1. `scans.pcd` を voxel でダウンサンプル（`--voxel` 0.05 m）し、外れ値を除去して法線を推定します。法線は、最も近い軌跡位置（センサ位置）の側へ向けます。
2. Open3D で Poisson 再構成します（`--poisson-depth` 10）。低密度の頂点（`--density-quantile`）と、元の点から離れた膜状の面（`--trim-factor`）は取り除きます。
3. `/Odometry` を画像時刻に補間し、`T_world_cam = T_world_imu · T_imu_cam` でカメラ姿勢を求めます。画像時刻はカメラ時計なので、`calib.yaml` の `camera.time_offset` を引いて LiDAR 時計に合わせます。
4. 移動量 `--kf-dist` 0.3 m または回転量 `--kf-angle` 10° ごとにキーフレームを選びます（上限は `--max-keyframes` 300）。歪み補正した画像を COLMAP の PINHOLE モデルとして書き出します。
5. OpenMVS で `InterfaceCOLMAP` → `TextureMesh --mesh-file mesh.ply` を実行します。
6. 点群の色付けでは、`--color-voxel` 0.02 m の点を各キーフレームに投影し、粗い Z バッファで遮蔽を判定します。色は距離と画像中心からの近さで重み付けして平均します。

## 注意点

- 色付けの品質は、**カメラ内部パラメータ**、**LiDAR-カメラ外部パラメータ**、**時刻同期**で決まります。USB カメラ（ハードウェアトリガなし、ローリングシャッタ）では、機体が速く回転すると色ずれが出ます。`camera.time_offset` を調整するか、`--min-sharpness` でブレた画像を除外してください。
- Mid-360S は 4 ラインの非繰り返しスキャンなので、静止時間が長いと点の密度が偏ります。Poisson の穴や膜が目立つ場合は、`--voxel`、`--poisson-depth`、`--trim-factor` を調整してください。
- FAST-LIO にはループクロージャがありません。長い飛行で累積誤差が出ると、テクスチャが二重に写ります。
- OpenMVS は CPU 版です（`DensifyPointCloud` は使いません）。大きな地図では `TextureMesh` に数分～数十分かかります。
- OpenMVS v2.2.0 の TextureMesh は、シーム平滑化の処理中に `std::out_of_range` で異常終了することがあります。その場合は `--global-seam-leveling 0 --local-seam-leveling 0` を付けて自動で再試行します。このときパッチ境界の色の段差は平滑化されません。
- bag の再生が終わった後は、`clock_keeper.py` が `/clock` を進め続けます。`/clock` が止まったままだと、FAST-LIO が終了処理で待ち続けてしまい `scans.pcd` を保存できないためです。
- `test/make_synthetic_texture_inputs.py` は後処理の単体テスト用で、市松模様の壁と床からなる合成シーンの入力を作ります。
