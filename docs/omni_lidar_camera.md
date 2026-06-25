# 全天球カメラ + LiDAR 色付き点群メモ

## 目的

- ロボット上部に全天球カメラを追加する。
- 3D LiDAR と全天球カメラの外部パラメータを扱えるようにし、色付き点群を出す。
- LiDAR perception が検出/追跡した物体について、全天球画像から歪み補正済みクロップを取得する。

## 実装方針

主対象は Webots。Webots の `Camera` は `projection "spherical"` / `projection "cylindrical"` を持ち、広角カメラや正距円筒画像のシミュレーションに使える。公式ドキュメントでは spherical projection は魚眼系、cylindrical projection は equirectangular 画像生成に使えると案内されている。今回の360度カメラは後段の点群投影と合わせやすい `projection "cylindrical"` + `fieldOfView 2π` を使う。

- 参考: Webots Camera reference: https://cyberbotics.com/doc/reference/camera
- 参考: Webots spherical camera sample: `/usr/local/webots/projects/samples/devices/worlds/spherical_camera.wbt`

Gazebo Classic 11 は通常カメラの ROS 2 plugin は使えるが、全天球/魚眼を ROS 2 の `Image` + `CameraInfo` + 後段投影まで安定して扱う経路が弱い。公式の Gazebo ROS plugin は通常センサ連携が中心で、広角カメラ固有の扱いは Webots の方が素直だった。

- 参考: Gazebo Classic ROS plugins: https://classic.gazebosim.org/tutorials?tut=ros_gzplugins
- 参考: gazebo_plugins package: https://index.ros.org/p/gazebo_plugins/

そのため、Webots では真の球面投影カメラを使い、Gazebo 側は将来確認用の6面カメラ合成フォールバックを残す。

## 追加トピック

Webots:

| トピック | 型 | 内容 |
|---|---|---|
| `/omni_camera/image_raw/image_color` | `sensor_msgs/Image` | Webots cylindrical camera の全天球画像 |
| `/omni_camera/image_raw/compressed` | `sensor_msgs/CompressedImage` | キャリブレーション/記録向け JPEG 圧縮画像 |
| `/omni_camera/image_raw/camera_info` | `sensor_msgs/CameraInfo` | Webots driver が出す camera_info |
| `/omni_camera/equirect/camera_info` | `sensor_msgs/CameraInfo` | equirectangular 扱いを明示する補助 CameraInfo |
| `/lidar/points_intensity` | `sensor_msgs/PointCloud2` | direct_visual_lidar_calibration 用の疑似 intensity 付き点群 |
| `/perception/colorized_points` | `sensor_msgs/PointCloud2` | `/lidar/points`（Webots は `/lidar/points/point_cloud`）にRGBを付与した点群 |
| `/perception/object_crops/image_rect` | `sensor_msgs/Image` | 追跡物体ごとの透視投影クロップを並べた画像 |
| `/slam/colorized_points_map` | `sensor_msgs/PointCloud2` | SLAM/odom 座標に蓄積した色付き点群地図 |
| `/slam/glim_colorized_points_map` | `sensor_msgs/PointCloud2` | GLIM の補正済み `glim_map` 座標に蓄積した色付き点群地図 |

Services:

| サービス | 型 | 内容 |
|---|---|---|
| `/slam/save_colorized_map` | `std_srvs/Trigger` | 蓄積済みカラー点群を PLY に保存 |

TF:

| 親 | 子 | 初期値 |
|---|---|---|
| `base_link` | `lidar_link` | `xyz=(0,0,0.20), rpy=(0,0,0)` |
| `base_link` | `omni_camera_link` | `xyz=(0,0,0.75), rpy=(0,0,0)` |

この `lidar_link -> omni_camera_link` が外部キャリブレーションの初期値になる。実機や厳密検証では、このTFをキャリブレーション結果で置き換える。

## キャリブレーション手法の整理

### 1. 既知ターゲット方式

チェッカーボード、AprilTag、反射板、穴あき板、平面板などをLiDARとカメラの両方で観測し、対応する平面・辺・角・点を使って外部パラメータを推定する。

長所:

- シミュレーションで検証しやすい。
- 対応関係が明確で、初期導入の再現性が高い。
- Zhou et al. のようにチェッカーボードの平面と境界線を使う手法は、少ない姿勢数でも解ける。

短所:

- ターゲット準備が必要。
- VLP-16相当のような低層LiDARでは、近距離や角度によってターゲット点が疎になりやすい。
- 全天球画像では、ターゲットを画像の端/上下に置くと投影歪みが大きいので、複数方位に分散して観測する方がよい。

参考:

- Zhou, Li, Kaess, “Automatic Extrinsic Calibration of a Camera and a 3D LiDAR using Line and Plane Correspondences”, IROS 2018: https://www.ri.cmu.edu/app/uploads/2018/09/Zhou18iros.pdf

### 2. 手動対応点 + 最適化

点群上の3D点と画像上の2D点を人が選び、PnP/非線形最小二乗で初期外部パラメータを推定する。`direct_visual_lidar_calibration` は手動初期値を作ってから投影を見ながら調整する導線を持つ。

長所:

- ターゲット無しでも開始できる。
- シミュレーションの既知形状（壁の角、机の角、箱など）を使える。

短所:

- 対応点選びの品質に強く依存する。
- 全天球では投影モデルを間違えると中心付近だけ合って周辺がずれる。

参考:

- direct_visual_lidar_calibration program docs: https://koide3.github.io/direct_visual_lidar_calibration/programs/

### 3. ターゲットレス方式

点群の構造と画像のテクスチャ/エッジを直接合わせる。`direct_visual_lidar_calibration` はROS 1/ROS 2対応で、pinhole/fisheye/omnidirectional camera と spinning LiDAR を扱えると説明されている。

長所:

- キャリブレーションターゲット不要。
- 実運用環境のテクスチャや構造を使える。
- 全天球カメラにも向く。

短所:

- 初期値が悪いと局所解に落ちやすい。
- テクスチャの少ない環境、動体が多い環境、LiDAR点が疎な環境では不安定。
- シミュレーションでは材質/照明が単調だと画像エッジが弱い。

参考:

- direct_visual_lidar_calibration: https://github.com/koide3/direct_visual_lidar_calibration

## 実装方針サマリ

| 領域 | 現行方針 |
|---|---|
| 全天球カメラ | Webots は `projection "cylindrical"` を主対象にする。Gazebo Classic は 6 面カメラ合成をフォールバックとして残す |
| 投影モデル | `omni_projection.py` に集約し、色付き点群、物体クロップ、分類クロップ、信号認識ビュー、検証スクリプトで同じ式を使う |
| 初期TF | `base_link -> lidar_link = z 0.20m`、`base_link -> omni_camera_link = z 0.75m`。ロボット天面の写り込みを避けるためカメラはマスト搭載相当 |
| キャリブレーション記録 | `/omni_camera/equirect/camera_info`、`/omni_camera/image_raw/compressed`、`/lidar/points_intensity` を出す。圧縮画像を bag 記録の既定にする |
| 色付き点群地図 | 2D SLAM/odom 姿勢へ `/perception/colorized_points` を蓄積し、`/slam/colorized_points_map` として publish/save する |
| GLIM 色付き地図 | GLIM は `glim_*` の独立 TF tree で動かし、`/slam/glim_colorized_points_map` へ蓄積する。`config/glim_webots/` を MID-360/VLP-16 共通の既定にする |
| 既知の注意 | GLIM だけ `LD_LIBRARY_PATH` を launch 側で補正する。`config/glim_webots_vlp16/` は参照されていない未使用コピー |

## 検証コマンド

```bash
cd ~/ros2_ws
colcon build --packages-select susumu_object_perception --symlink-install

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash

ros2 launch susumu_object_perception webots_outdoor.launch.py nav:=False rviz:=True

# キャリブレーション用ターゲット world
ros2 launch susumu_object_perception webots_calibration.launch.py

ros2 topic hz /omni_camera/image_raw/image_color
ros2 topic hz /omni_camera/image_raw/compressed
ros2 topic hz /lidar/points/point_cloud
ros2 topic hz /lidar/points_intensity
ros2 topic hz /perception/colorized_points
ros2 topic hz /perception/object_crops/image_rect
ros2 topic hz /slam/colorized_points_map
```

RVizでは `/perception/colorized_points` を `PointCloud2`、`/perception/object_crops/image_rect` を `Image` display で確認する。
`slam:=True` 時は `/slam/colorized_points_map` を `PointCloud2` display で `map` frame に表示する。

キャリブレーションbag:

```bash
ros2 run susumu_object_perception record_omni_calibration_bag.sh
ros2 run susumu_object_perception run_direct_visual_lidar_calibration_docker.sh \
  ~/ros2_ws/omni_calibration_bags/<bag_dir> \
  ~/ros2_ws/omni_calibration_preprocessed/<case_name> \
  preprocess
```

`record_omni_calibration_bag.sh` は全天球画像が大きいため、既定で `--compression-mode file --compression-format zstd` を使う。
`run_direct_visual_lidar_calibration_docker.sh` は `/omni_camera/image_raw/compressed` と `--camera_model equirectangular` を既定にしている。

色付き点群SLAM:

```bash
ros2 launch susumu_object_perception webots_colored_slam.launch.py

ros2 topic echo --once /slam/colorized_points_map --field header
ros2 topic echo --once /slam/colorized_points_map --field width
ros2 service call /slam/save_colorized_map std_srvs/srv/Trigger {}
```

`webots_colored_slam.launch.py` は `webots_simulation.launch.py` を `nav:=True slam:=True omni_perception:=True colored_slam:=True` で呼ぶショートカット。既定 world は `calibration.wbt`、既定では Autoware perception を起動しない。

GLIM 3D loop-closure 色付き点群SLAM:

```bash
ros2 launch susumu_object_perception webots_glim_colored_slam.launch.py \
  rviz:=False mode:=fast perception:=False

ros2 topic echo --once /slam/glim_colorized_points_map --field header
ros2 topic echo --once /slam/glim_colorized_points_map --field width
ros2 run tf2_ros tf2_echo glim_map glim_lidar
ros2 service call /slam/save_colorized_map std_srvs/srv/Trigger {}
```

GLIM 終了時は `/tmp/dump/graph.bin`, `/tmp/dump/graph.txt`, `/tmp/dump/traj_lidar.txt`, `/tmp/dump/odom_lidar.txt` が生成される。
`/tmp/dump` は GLIM の固定出力先なので、設定ファイルまで厳密に再検証したい場合は起動前に消す。

## 色付き点群の検査結果サマリ

`/omni_camera/image_raw/image_color`、圧縮画像、equirect camera_info、`/lidar/points_intensity`、
`/perception/colorized_points`、`/slam/colorized_points_map` は publish される。点群は
`x/y/z/rgb` を持ち、主要ターゲットに非黒 RGB が入る。

現行検査は `validate_omni_colorization.py` に集約する。評価側も `omni_projection.py` を使い、
`--out-prefix` で JSON/CSV/Markdown を保存できる。

```bash
ros2 run susumu_object_perception validate_omni_colorization.py \
  --yaws 0,90,180,270 --startup-sec 35 --grab-timeout-sec 15 --require-pass \
  --min-large-target-score 0.40 --mode realtime
```

代表値:

| 項目 | 結果 |
|---|---|
| 大ターゲット色一致 | 赤/黄/緑/マゼンタの 4 方位でおおむね pass |
| 軽量 1 方位 fast 検証 | `validation_passed=true`, `color_score mean≈0.77`, `large_image_projection_error_deg max≈8.2` |
| 1度未満の精密投影評価 | 現検査 world と点密度では未達。大きい高コントラストターゲットか外部キャリブレーションが必要 |

色付き点群SLAMは、`odom` から開始し `slam_toolbox` の `map` が出たら map 蓄積へ切り替わる。
`/slam/save_colorized_map` で PLY 保存でき、短時間走行で点数が増えることを確認済み。

GLIM 色付き地図は `glim_map` frame で publish され、MID-360 経路でも `/lidar/points_intensity`
と `/perception/colorized_points` が sensor QoS で流れる。`ros2 topic echo` で確認する場合は
`--qos-reliability best_effort` を付ける。pose-graph backend は起動するが、短い検証だけでは
実ループ制約の成立までは確認していない。`/tmp/dump` は GLIM の固定出力先なので、設定込みで
再検証する前に消す。

次の改善候補は `direct_visual_lidar_calibration` 等で `lidar_link -> omni_camera_link` を最適化し、
色付き点群で再評価すること。1度未満を定量評価するなら、大きい AprilTag/ChArUco 板や複数の
垂直エッジ板を LiDAR 点が十分乗る距離と高さに置く。

## 追加調査: 誤差をキャリブレーションで吸収する方法

ユーザー指摘どおり、固定の `yaw_offset` / `pitch_offset` だけで吸収するのは不十分。必要なのは
LiDAR→カメラの6DoF外部パラメータと、全天球画像の投影モデルを含めたキャリブレーション。

調査した候補:

| 方法 | 内容 | 今回の適性 |
|---|---|---|
| `direct_visual_lidar_calibration` | ROS 1/ROS 2対応。pinhole/fisheye/omnidirectional/equirectangular、spinning LiDARを扱える。ターゲットレスで1ペアから開始可能 | **第一候補**。今回の全天球/equirectangular + VLP-16相当と合う |
| ターゲット付きチェッカーボード/ChArUco | 複数姿勢のターゲットを画像と点群で検出し、外部パラメータを推定 | シミュレーションでは可能。ただしVLP-16は垂直点密度が低く、ターゲット検出が不安定になりやすい |
| Webots投影モデルを自前同定 | 既知ターゲット色/位置から Webots cylindrical projection の写像を推定 | シミュレーション専用なら可能。ただし実機移行性が低い |
| 6面rectilinear cubemap | 6台の通常カメラで正確な投影モデルを作り、合成画像から色付け | 1度未満を狙いやすい。Webots cylindrical cameraの実投影差を避けられるが、「単一全天球カメラ」とは内部実装が異なる |

参照:

- `direct_visual_lidar_calibration`: https://github.com/koide3/direct_visual_lidar_calibration
- Koide et al., “General, Single-shot, Target-less, and Automatic LiDAR-Camera Extrinsic Calibration Toolbox”, ICRA 2023: https://arxiv.org/abs/2302.05094
- Program details (`omnidirectional/equirectangular`、`--rotate_camera` など): https://koide3.github.io/direct_visual_lidar_calibration/programs/
- Webots Camera reference (`spherical` / `cylindrical` projection): https://www.cyberbotics.com/doc/reference/camera
- Webots camera imaging model discussion: https://github.com/cyberbotics/webots/discussions/2614
- Webots wideangle discussion: https://hl.forum.robocup.org/t/modeling-wideangle-lenses-in-webots/495

実装側の対応:

- `colorized_pointcloud_node.py` と `object_image_crop_node.py` に `calibration_rpy_deg` を追加した。
- これにより、キャリブレーションで得た回転補正を点群色付けと物体クロップへ同時に反映できる。
- `equirect_camera_info_node.py` と `pointcloud_intensity_node.py` を追加し、bag記録から外部キャリブレーション前処理へ渡しやすくした。
- `omni_image_compress_node.py` を追加し、bag容量を大幅に下げた。
- `omni_sensor_tf_node.py` を追加し、初期TFまたは `direct_visual_lidar_calibration` の `calib.json` から LiDAR-camera TF をpublishできるようにした。
- `colorized_pointcloud_mapper_node.py` を追加し、SLAM/odom座標に色付き点群を蓄積できるようにした。
- `record_omni_calibration_bag.sh` と `run_direct_visual_lidar_calibration_docker.sh` を追加した。
- 本命は補正パラメータ直書きではなく、最終的に `lidar_link -> omni_camera_link` のTFをキャリブレーション結果へ置換すること。

`direct_visual_lidar_calibration` の調査結果:

- ローカルに `direct_visual_lidar_calibration` をcloneし、`docs/programs.md` と実装を確認した。
- `--camera_model equirectangular` はサポートされ、preprocess 実装では画像サイズから intrinsic `{width, height}` を作る。
- 公式Docker image は `koide3/direct_visual_lidar_calibration:humble`。
- ネイティブビルドは Jammy の `libceres-dev 2.0.0` と現行 submodule の組み合わせで `fatal error: ceres/manifold.h: No such file or directory` になった。したがって、この環境ではDocker経由が現実的。
- `calib.json` の `T_lidar_camera` は `p_lidar = T_lidar_camera * p_camera` の向きで保存されるため、ROS TF に入れる場合は向きの反転確認が必要。
- 11秒の無圧縮bagは約2.9GiBになった。Docker preprocess は `equirectangular`、画像サイズ、`/lidar/points_intensity`、`intensity` channel を正しく認識したが、180秒では完了しなかった。長めに取る場合は圧縮、画像解像度低下、短い複数bag、または高速ストレージが必要。
- 11秒の圧縮bagは約135MiBになった。`ros2 run susumu_object_perception run_direct_visual_lidar_calibration_docker.sh ... preprocess` は 10秒台で完走し、`calib.json` と `.ply/.png` を出力した。

現実的な誤差目標:

- Webots cylindrical cameraをそのまま使う場合: Webots固有の投影差があり、現状の簡易モデルでは全方位1度未満は現実的ではない。
- `direct_visual_lidar_calibration` の equirectangular/omnidirectional モデルで最適化する場合: 目標は1度未満。ただしVLP-16相当の疎な点群では、十分なテクスチャ・複数姿勢・点群densificationが必要。
- シミュレーションで確実に1度未満を狙う場合: 6面rectilinear cubemap方式が最も堅い。

## 今後のキャリブレーション環境案

- `webots_worlds/calibration.wbt` に色付き箱と高コントラスト板を複数方位に配置した。
- ターゲットはカメラの上下端に寄せすぎず、水平周り360度に分散する。
- VLP-16相当は垂直解像度が低いので、板をLiDARから2-5m程度、LiDAR水平面を横切る高さに置く。
- 最初は既知TFで投影確認し、次に `direct_visual_lidar_calibration` の手動初期値/ターゲットレス最適化を試す。

## AprilTag 既知ターゲット方式（全天球 + LiDAR 外部キャリブ、2026-06-24 実装）

ターゲットレスの `direct_visual_lidar_calibration` とは別に、**AprilTag 板を既知ターゲットにした
外部キャリブ**を実装する。`apriltag_ros` はピンホール rectified 画像前提で全天球(equirect)に直接
使えないため、本リポでは **OpenCV(`cv2.aruco` の AprilTag 36h11 辞書)で検出する自前パイプライン**に
する（OpenCV 4.12 で `DICT_APRILTAG_36h11` / `ArucoDetector` / `solvePnP` が使えることを確認済み。
`apriltag_ros` パッケージ依存も独自 `.msg` も増やさない）。

### 方式

| 段 | 内容 |
|---|---|
| ターゲット | `calibration.wbt` の方位パネル表面に AprilTag 36h11 を `ImageTexture` で貼る。**既知サイズ・既知 ID**。外部キャリブは LiDAR↔カメラの相対姿勢だけなので、タグの `map` 座標は不要 |
| カメラ側 | `/omni_camera/image_raw/image_color`(equirect) を、各タグ方位へ `omni_projection.perspective_directions` で**透視ビュー展開**（信号認識と同じ作法）→ 仮想ピンホール intrinsic を与え `cv2.aruco` で検出 → `cv2.solvePnP` でタグ4隅の**カメラ座標系姿勢 `T_cam_tag`**（中心・法線・コーナー3D）を得る |
| LiDAR側 | `/lidar/points` からタグ板付近の点をクラスタリング → 平面フィット（RANSAC）→ タグ板の**中心・法線を LiDAR 座標系**で得る。タグの並進はパネル幾何（既知）で板中心に合わせる |
| 対応付け | 各タグについて「カメラ座標の板中心・法線」↔「LiDAR座標の板中心・法線」を対応。複数方位タグ × 複数フレームを集め、点対応は SVD（Kabsch/Umeyama）、法線対応も加えて **6DoF `T_lidar_camera` を最小二乗推定** |
| 出力 | `T_lidar_camera`（`[x,y,z,qx,qy,qz,qw]` 7値）を **`calib.json` 形式**で書く。既存 `omni_sensor_tf_node.py`（`results.T_lidar_camera` を読む）と `scripts/direct_calib_to_tf.py` がそのまま流用でき、`direct_visual_lidar_calibration` と同じ TF 置換導線に乗る |

`T_lidar_camera` は `p_lidar = T_lidar_camera * p_camera` の向き（vlcal と同一規約）で保存する。

### なぜ全天球で透視ビュー展開が要るか

全天球は Webots cylindrical projection（正距円筒）で、ピンホール intrinsic と画素の対応が成立しない。
`cv2.solvePnP` はピンホール前提なので、**タグが写っている方位だけ仮想ピンホール透視ビューに展開**して
から検出・PnP する。これは `traffic_light_detector_node.py` が信号灯で実証済みの手法と同型。

### 実装物

| ファイル | 役割 |
|---|---|
| `webots_worlds/calibration.wbt` | 4 方位パネルに AprilTag 36h11 テクスチャ板を追加（`apriltag_textures/` の生成 PNG を貼る） |
| `susumu_object_perception/apriltag_extrinsic_calib_node.py` | 全天球画像 + LiDAR 点群から上記方式で `T_lidar_camera` を推定し `calib.json` を出力。標準型のみ使用 |
| `scripts/generate_apriltag_textures.py` | `cv2.aruco.generateImageMarker` で 36h11 タグ PNG を生成（Webots テクスチャ用） |
| `launch/webots_calibration.launch.py` | `apriltag_calib:=True` で上記ノードを opt-in 起動 |

### 合格条件

- `calibration.wbt` でライブ起動し、複数方位の AprilTag を検出・PnP・LiDAR 平面抽出できる。
- 推定した `T_lidar_camera` が既知初期 TF（`base_link->lidar_link z0.20` と `->omni_camera_link z0.75`
  から導く `lidar_link->omni_camera_link = z0.55`）に対し、並進数 cm / 回転数度のオーダーで一致する。
- 得た TF を `omni_sensor_tf_node.py` に渡し、`validate_omni_colorization.py` の投影誤差が既知 TF と
  同等以下になる（色付け品質が劣化しない）。

### 実行

```bash
cd ~/ros2_ws
colcon build --packages-select susumu_object_perception --symlink-install
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# タグテクスチャ生成（初回のみ。webots_worlds/apriltag_textures/ に出る）
ros2 run susumu_object_perception generate_apriltag_textures.py --ids 0,1,2,3

# AprilTag 外部キャリブを実行（calib.json を出力）
ros2 launch susumu_object_perception webots_calibration.launch.py \
  mode:=realtime rviz:=False perception:=False colored_slam:=False \
  apriltag_calib:=True

# 得た TF で色付け（既存導線。omni_sensor_tf_node が calib.json を読む）
ros2 launch susumu_object_perception webots_calibration.launch.py \
  omni_calibration_json:=~/ros2_ws/src/susumu_object_perception/outputs/extrinsic_calibration/calib.json
```

### ライブ検証結果（2026-06-24、calibration.wbt, MID-360）

4 方位パネル（tag id 0/1/2/3）を全て検出し、`T_lidar_camera` を推定できた（20 フレーム平均）。

検証時の全天球画像と各方位の透視ビューは `docs/images/apriltag_calib_*.png`
（`omni_pano` = 元の正距円筒画像、`view_{front,left,back,right}_id{0,1,2,3}` = タグ検出に使った透視ビュー）。
FRONT/BACK パネルの `rotation` を外す前はこの front/back ビューでタグが白飛びして未検出だった（後述の落とし穴）。

**精度（板厚補正 `board_thickness=0.04` 適用後）:**
- 推定値: translation `[-0.0230, 0.0072, 0.5477] m`、quat(xyzw) `[-0.0007, -0.0023, 0.0013, 1.0000]`（ほぼ無回転）。
- 真値 `lidar_link->omni_camera_link = z 0.55 / 無回転` に対し **回転 0.32°・対応点 RMS 9.6mm**
  （板厚補正前は RMS 20.6mm。補正で半減した）。z 単独は 0.5477（誤差 2.3mm）。
- ただし **並進絶対誤差は 24mm 残る**。主因は x の -23mm 系統オフセット。これは板厚では説明できず
  （補正量を 0〜±20mm 変えても -23mm 不変と実測で確認）、**LiDAR がタグ板の下半分にしか点を返さず
  点群重心が板物理中心より下に偏る**こと（cam が見るタグ中心 z≈-0.15 に対し LiDAR 板中心 z≈+0.39。
  本来は両者の z 差が 0.55 になるべきだが、LiDAR 側が板中心を捉えきれていない）に由来する。
  これは MID-360 の上向き FOV とパネル高さ（中心 0.75m）の組み合わせによるシミュ固有の幾何で、
  探索アルゴリズムやキャリブ式の誤りではない。**回転 0.32°・RMS 9.6mm は色付け用途には十分**だが、
  「並進 1cm 未満」の精密目標には未達。1cm 未満を狙うならパネルを LiDAR 水平面（z0.20）中心へ下げるか、
  LiDAR 点群の z 分布上端を板上端とみなす重心補正が要る（今回は未実施）。

**活用実証（得た TF を色付けに適用、初期 TF と比較）:**
- calib.json を `omni_calibration_json:=...` で渡すと、`omni_sensor_tf_node` が
  `lidar_link->omni_camera_link` を **初期値 `[0,0,0.55]` からキャリブ実測値 `[-0.023,0.007,0.548]` に置換**
  し（`tf2_echo` で確認）、`colorized_pointcloud_node` がそのキャリブ TF で `/perception/colorized_points`
  を色付けすることを確認した。
- 色付け品質（カラー物体の点に正しい色が乗るかのスコア、1.0 が完全一致）:

  | ターゲット | 初期 TF | キャリブ TF |
  |---|---|---|
  | green box | 0.970 | 0.976 |
  | magenta cylinder | 0.985 | 0.982 |

  **キャリブ TF で色付け品質は初期 TF と同等（差 ±0.6%）= 劣化させない**。シミュ真値が
  ほぼ z0.55/無回転なので両 TF とも良好になるのは想定どおりで、ここでの実証点は「キャリブ結果が
  TF として系に正しく注入され、実際に色付けへ使われ、品質を壊さない」こと。
- 出力 calib.json は `scripts/direct_calib_to_tf.py` / `omni_sensor_tf_node.py` がそのまま読め、
  `direct_visual_lidar_calibration` と同じ TF 置換導線（`omni_calibration_json:=...`）に乗る。

### 実装上の落とし穴（再現する人向け）

- **LiDAR 点群トピックは `/lidar/points/point_cloud`**（Webots driver の `/point_cloud` サフィックス）。
  calib ノードの `input_cloud` を launch で合わせる（`/lidar/points` のままだと点群が来ず
  「not enough tag correspondences」になる）。
- **Webots の `Box` テクスチャは回転（`rotation`）の有無で貼られ方が変わる**。FRONT/BACK パネルに
  `rotation 0 0 1 1.5708` が付いていた頃はタグ面が白飛びして検出できず、回転を外して 4 パネルを
  「回転なし Box」に統一したら全タグ検出できた。テクスチャ板は回転で配置せず、Box 寸法と translation で
  正対させる。
