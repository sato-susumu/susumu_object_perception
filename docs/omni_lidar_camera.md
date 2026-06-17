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
| `/velodyne_points/point_cloud_intensity` | `sensor_msgs/PointCloud2` | direct_visual_lidar_calibration 用の疑似 intensity 付き点群 |
| `/perception/colorized_points` | `sensor_msgs/PointCloud2` | `/velodyne_points/point_cloud` にRGBを付与した点群 |
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
| `base_link` | `velodyne_link` | `xyz=(0,0,0.20), rpy=(0,0,0)` |
| `base_link` | `omni_camera_link` | `xyz=(0,0,0.75), rpy=(0,0,0)` |

この `velodyne_link -> omni_camera_link` が外部キャリブレーションの初期値になる。実機や厳密検証では、このTFをキャリブレーション結果で置き換える。

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

## 今回の試行錯誤

1. 最初に Gazebo Classic 11 で全天球相当を考えた。
   - `gazebo_ros_camera` は通常カメラなら素直に動く。
   - ただし全天球/魚眼で歪みモデル、`CameraInfo`、後段の点群投影を一貫して扱う確実な経路が弱い。
   - Gazebo用には6方向90度カメラを合成するフォールバックを入れたが、ユーザー要望に合わせて主対象はWebotsの球面投影へ切り替えた。

2. Webotsを調べた。
   - `/usr/local/webots/projects/samples/devices/worlds/spherical_camera.wbt` に `Camera { projection "spherical" }` の実例があった。
   - Webots公式ドキュメントの説明では、360度の正距円筒画像には `projection "cylindrical"` が適しているため、最終実装は `projection "cylindrical"` にした。
   - `webots_ros2_driver` からは通常の `Camera` device として扱えるため、`resource/turtlebot_webots_3d.urdf` に `reference="omni_camera"` を追加した。

3. 全天球画像の補正方針を決めた。
   - Webots cylindrical camera の出力は全天球の正距円筒画像として扱う。
   - 点群色付けは正距円筒モデルで `yaw/pitch -> pixel` に投影する。
   - 物体画像は、物体中心方向の局所透視投影に再サンプリングし、通常カメラ風のクロップにする。

4. キャリブレーション初期値をTFで明示した。
   - `base_link -> velodyne_link = z 0.20m`
   - `base_link -> omni_camera_link = z 0.75m`
   - よって初期外部パラメータは同軸で、カメラがLiDARより55cm上。0.32m では全天球画像の大部分にロボット天面が写り込み、色付き点群がロボット表面色を拾ったため、マスト搭載相当の高さに上げた。
   - 厳密には、ターゲットレス/ターゲット方式で `velodyne_link -> omni_camera_link` を推定して置き換える。

5. Webots の実投影を shader から確認した。
   - `/usr/local/webots/resources/wren/shaders/merge_spherical.frag` を読むと、cylindrical では `xCurrentAngle = (0.5 - texUv.x) * fovX`、`yCurrentAngle = (texUv.y - 0.5) * fovY / fovYCorrectionCoefficient + pi/2` を使っていた。
   - これに合わせて `colorized_pointcloud_node.py` / `object_image_crop_node.py` の既定投影を `webots_cylindrical` にし、Webots camera の取り付け姿勢を含む回転 `WEBOTS_CYLINDRICAL_ROT` を適用した。
   - これで単純な `yaw/pitch` 実装より、緑箱・マゼンタ円柱を含む全方位の色一致が大きく改善した。

6. キャリブレーション用データの出力を追加した。
   - `/omni_camera/equirect/camera_info` は ROS 標準に無い `distortion_model: equirectangular` を明示する補助情報。`direct_visual_lidar_calibration` では `--camera_model equirectangular` を指定すると画像サイズから intrinsic を作るため、これは主に記録・確認用。
   - Webots の `/velodyne_points/point_cloud` は intensity を持たないため、`/velodyne_points/point_cloud_intensity` を追加した。疑似 intensity は距離から作る。キャリブレーションの主情報は幾何+画像なので、まずは前処理を通すための補助値として扱う。
   - `/omni_camera/image_raw/compressed` を追加し、bag記録は圧縮画像を既定にした。全天球 2048x1024 raw をそのまま記録すると bag が膨れすぎるため、キャリブレーションでは JPEG 圧縮画像を使う。

7. 色付き点群SLAM地図を追加した。
   - `colorized_pointcloud_mapper_node.py` が `/perception/colorized_points` を TF で `map` に変換し、voxel downsample しながら `/slam/colorized_points_map` として蓄積する。
   - `slam_toolbox` がまだ `map` を出していない場合は `odom` にフォールバックし、`map` が使えるようになったら蓄積をクリアして `map` へ切り替える。
   - `/slam/save_colorized_map` で `~/ros2_ws/colorized_slam_maps/colorized_map_<timestamp>.ply` に保存できる。
   - これは「2D LiDAR SLAMで姿勢を安定化し、その姿勢に全天球RGB付き3D LiDAR点群を積む」方式。完全な3D loop-closure付きカラー点群SLAMではないが、今の構成で動く現実的な第一段階。

8. GLIM で 3D loop-closure 付き色付き点群SLAMを追加した。
   - この環境には `glim_ros` / `glim` の CUDA 版が入っていた。
   - 初回起動では `/usr/local/lib/libgtsam_points*.so` と `/opt/ros/humble/lib/x86_64-linux-gnu/libgtsam.so.4` が混在し、`undefined symbol: gtsam::NonlinearFactor::rekey(...)` で落ちた。
   - `LD_LIBRARY_PATH=/usr/local/lib:/opt/ros/humble/lib:/opt/ros/humble/lib/x86_64-linux-gnu:...` を GLIM ノードだけに適用すると起動できたので、`webots_glim_colored_slam.launch.py` の `additional_env` に閉じ込めた。
   - `config/glim_webots/` に Webots VLP-16 相当向け GLIM 設定を追加し、`config_global_mapping_pose_graph.json` を使って明示的な pose graph loop closure 構成にした。
   - GLIM は ROS topic として `/glim_ros/pose_corrected` を出すが、この環境では `glim_map -> glim_imu` TF を publish しなかったため、`pose_stamped_tf_bridge_node.py` で PoseStamped を TF に変換した。
   - GLIM が出す `glim_imu -> glim_lidar` と bridge の `glim_map -> glim_imu` をつなぎ、`colorized_pointcloud_mapper_node.py` は `source_frame_override:=glim_lidar` で `/perception/colorized_points` を `glim_map` へ積む。
   - 既存 Webots/Nav2 の `odom -> base_link` TF と衝突しないよう、GLIM は `glim_*` の独立フレームツリーにした。

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
ros2 topic hz /velodyne_points/point_cloud
ros2 topic hz /velodyne_points/point_cloud_intensity
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

## 色付き点群の検査結果

2026-06-17 に `webots_calibration.launch.py nav:=False rviz:=False perception:=False omni_perception:=True mode:=fast` で実データ検査した。

確認できたこと:

- `/omni_camera/image_raw/image_color` は publish される。
- `/omni_camera/image_raw/compressed` は JPEG として publish される。
- `/perception/colorized_points` は publish される。
- `/omni_camera/equirect/camera_info` は `width=2048`, `height=1024`, `distortion_model=equirectangular` で publish される。
- `/velodyne_points/point_cloud_intensity` は `x/y/z/intensity` フィールドを持つ。
- `PointCloud2` は `x/y/z/rgb` フィールドを持ち、全点に非黒RGBが入る。
- `/slam/colorized_points_map` は `x/y/z/rgb` フィールドを持ち、`slam:=True` では `frame_id=map` になる。
- 初期実装では水平投影の左右が逆だったため、`atan2(y, x)` から `atan2(-y, x)` に修正した。
- 初期実装では縦方向が逆で床/ロボット天面色を拾っていたため、`v=(pi/2 + pitch)/pi*height` に修正した。
- 0.32m搭載では全天球画像の大部分にロボット天面が写り、色付き点群が自己投影色を拾ったため、Webotsの全天球カメラを `z=0.75m` に上げた。
- その後、Webots shader 互換の `webots_cylindrical` 投影に置き換えた。

shader 互換投影への修正後の代表統計:

| 検査領域 | 期待 | 結果 |
|---|---|---|
| 赤パネル | 赤 | `mean RGB ≈ [223, 54, 54]`, score `0.99` |
| 黄パネル | 黄 | `mean RGB ≈ [129, 121, 37]`, score `0.97` |
| 緑箱 | 緑 | `mean RGB ≈ [45, 176, 38]`, score `0.50` |
| マゼンタ円柱 | マゼンタ | `mean RGB ≈ [162, 46, 156]`, score `0.87` |

4方向回転検査:

```bash
ros2 run susumu_object_perception validate_omni_colorization.py \
  --yaws 0,90,180,270 --startup-sec 35 --grab-timeout-sec 15 --require-pass \
  --min-large-target-score 0.40
```

| yaw | 赤panel | 黄panel | 緑box | マゼンタcylinder |
|---:|---:|---:|---:|---:|
| 0 deg | 0.99 | 0.97 | 0.50 | 0.87 |
| 90 deg | 0.93 | 0.95 | 0.52 | 0.87 |
| 180 deg | 0.97 | 0.99 | 0.56 | 0.86 |
| 270 deg | 0.91 | 0.96 | 0.56 | 0.92 |

小球マーカー込みの summary は `color_score mean=0.685 min=0.133`, `image_projection_error_deg mean=8.270 max=19.863`。小球マーカーは VLP-16相当では点数が7-15点程度で、画像側の同色ピクセル抽出にも引っ張られるため、1度未満評価用のターゲットとしては不十分だった。

2026-06-18 に同じ4方向を `--require-pass` 付きで再実行し、`validation_passed=true` を確認した。
合否は大ターゲット（赤/黄パネル、緑箱、マゼンタ円柱）の色一致率 `0.40` 以上と、画像投影健全性 `20deg` 以内で判定した。
`8deg` しきい値では赤パネルが最大 `9.20deg`、小型の orange/white marker が `16-20deg` になり失敗したため、画像 centroid は1度未満評価ではなく、投影が大きく破綻していないことの補助指標に下げた。

結論:

`/perception/colorized_points` は、主要な色付き物体についてロボットyawを変えても色が入れ替わらず、Webots上で実用確認できる段階になった。ただし、現状の検査worldとVLP-16相当の点密度では、画像投影誤差を1度未満と断言できる評価にはなっていない。1度未満を狙うなら、`direct_visual_lidar_calibration` による外部パラメータ最適化、または点密度の高いLiDAR/大きい高コントラストターゲットが必要。

色付き点群SLAMの検査:

- `webots_simulation.launch.py world:=calibration.wbt nav:=True slam:=True rviz:=False perception:=False omni_perception:=True colored_slam:=True mode:=fast` で起動した。
- 同じ構成を短く起動するため、`webots_colored_slam.launch.py` を追加した。
- 起動直後は `odom` に蓄積し、`slam_toolbox` が `map` を出した時点で `odom -> map` に切り替わることをログで確認した。
- `/slam/colorized_points_map` は `frame_id: map` で publish された。
- 静止時は `width=1549`。`/cmd_vel` で約8秒動かした後は `width=13458` まで増えた。
- `/slam/save_colorized_map` で `/home/taro/ros2_ws/colorized_slam_maps/colorized_map_1781722681.ply` を保存できた。PLYヘッダは `element vertex 13458`。

GLIM 3D loop-closure 色付き点群SLAMの検査:

- `webots_glim_colored_slam.launch.py rviz:=False mode:=fast perception:=False` で GLIM が `libglobal_mapping_pose_graph.so` を読み込むことを確認した。
- 起動直後に `/slam/glim_colorized_points_map` が `frame_id: glim_map`, `width=1601` で publish された。
- `tf2_echo glim_map glim_lidar` は `glim_map -> glim_imu -> glim_lidar` の接続を確認できた。初期静止時の yaw は約 `-1.02deg` で、VLP-16相当 + Webots IMU の初期推定としては現実的な範囲。
- 小さな四角ループ走行後、`/slam/glim_colorized_points_map` は `points=16867`, `colored_ratio=1.0000`, `rgb_std=[52.4,47.8,50.5]` になり、走行で地図が増えることを確認した。
- `/slam/save_colorized_map` で `/home/taro/ros2_ws/colorized_slam_maps/colorized_map_1781729933.ply` を保存できた。PLY検査は `points=16867`, `header_vertices=16867`, `colored_ratio=1.0000`, `validation_passed=true`。
- GLIM 正常終了時に `/tmp/dump/graph.bin`, `/tmp/dump/graph.txt`, `/tmp/dump/traj_lidar.txt`, `/tmp/dump/odom_lidar.txt` が生成された。今回の実行では `graph.txt` が `num_submaps: 6`, `num_all_frames: 2691` を示した。
- ただし、短い四角ループでは `num_matching_cost_factors: 0` だった。pose-graph loop-closure backend は起動済みだが、この検証だけでは実ループ制約の成立までは確認できていない。

既知の課題:

- GLIM viewer extension は `glim_imu -> glim_lidar` の時刻外挿警告を多く出す。mapper は最新TFを使うため `/slam/glim_colorized_points_map` は出ているが、ログはまだうるさい。
- Webots fast mode では LiDAR/IMU 間のサンプル数が少なく、GLIM が `insufficient number of IMU data between LiDAR scans` を警告する。より厳密なループ閉じ検証では `mode:=realtime` または LiDAR/IMU publish rate 調整を試す。
- 実ループ制約を成立させるには、より特徴の多い world、長めの周回、`min_travel_dist` / `max_neighbor_dist` / `min_inliear_fraction` の追加調整が必要。
- `config_global_mapping_gpu.json` も追加して試したが、短い calibration world 周回ではカラー map は `points=17540` まで増えた一方、終了時に GTSAM `IndeterminantLinearSystemException` が連続して `/tmp/dump` が残らなかった。既定は安定して保存できた `config_global_mapping_pose_graph.json` に戻した。
- `/tmp/dump` は GLIM の固定出力先で前回内容が残りやすい。設定ファイルの保存確認まで含めるときは起動前に削除する。

次の改善候補:

- `direct_visual_lidar_calibration` 等で初期TFを最適化し、色付き点群で検証する。
- 1度未満の定量検査には、大きいAprilTag/ChArUco板や複数の垂直エッジ板を使い、LiDAR点が十分乗る距離と高さに置く。
- 実用優先なら、全天球画像は Webots cylindrical camera、色付けは6面rectilinear補助カメラのcubemapから行う構成も候補。

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
- Webots wideangle discussion: https://hl.forum.robocup.org/t/modeling-wideangle-lenses-in-webots/495

実装側の対応:

- `colorized_pointcloud_node.py` と `object_image_crop_node.py` に `calibration_rpy_deg` を追加した。
- これにより、キャリブレーションで得た回転補正を点群色付けと物体クロップへ同時に反映できる。
- `equirect_camera_info_node.py` と `pointcloud_intensity_node.py` を追加し、bag記録から外部キャリブレーション前処理へ渡しやすくした。
- `omni_image_compress_node.py` を追加し、bag容量を大幅に下げた。
- `omni_sensor_tf_node.py` を追加し、初期TFまたは `direct_visual_lidar_calibration` の `calib.json` から LiDAR-camera TF をpublishできるようにした。
- `colorized_pointcloud_mapper_node.py` を追加し、SLAM/odom座標に色付き点群を蓄積できるようにした。
- `record_omni_calibration_bag.sh` と `run_direct_visual_lidar_calibration_docker.sh` を追加した。
- 本命は補正パラメータ直書きではなく、最終的に `velodyne_link -> omni_camera_link` のTFをキャリブレーション結果へ置換すること。

`direct_visual_lidar_calibration` の調査結果:

- ローカルに `direct_visual_lidar_calibration` をcloneし、`docs/programs.md` と実装を確認した。
- `--camera_model equirectangular` はサポートされ、preprocess 実装では画像サイズから intrinsic `{width, height}` を作る。
- 公式Docker image は `koide3/direct_visual_lidar_calibration:humble`。
- ネイティブビルドは Jammy の `libceres-dev 2.0.0` と現行 submodule の組み合わせで `fatal error: ceres/manifold.h: No such file or directory` になった。したがって、この環境ではDocker経由が現実的。
- `calib.json` の `T_lidar_camera` は `p_lidar = T_lidar_camera * p_camera` の向きで保存されるため、ROS TF に入れる場合は向きの反転確認が必要。
- 11秒の無圧縮bagは約2.9GiBになった。Docker preprocess は `equirectangular`、画像サイズ、`/velodyne_points/point_cloud_intensity`、`intensity` channel を正しく認識したが、180秒では完了しなかった。長めに取る場合は圧縮、画像解像度低下、短い複数bag、または高速ストレージが必要。
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
