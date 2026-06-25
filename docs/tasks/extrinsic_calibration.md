# 外部キャリブレーションタスク — 全天球カメラ + 3D LiDAR

このページは README / AGENTS のタスク一覧「外部キャリブレーション」の詳細ページ。
シミュレータ上で **全天球カメラと 3D LiDAR の外部パラメータ（`lidar_link -> omni_camera_link`）**
を推定し、色付き点群・物体クロップなど LiDAR×カメラ連携の TF をキャリブ結果で置き換えるところまでを扱う。

手法の詳細・投影モデル・実測値・落とし穴の正本は [`../omni_lidar_camera.md`](../omni_lidar_camera.md)。
ここはタスクとしての目的・入出力・実行・合格基準・制約に絞る（重複定義しない）。

## 入出力

| 項目 | 内容 |
|---|---|
| 入力 | `webots_worlds/calibration.wbt`（4 方位に AprilTag 36h11 パネル）、`/omni_camera/image_raw/image_color`（equirect）、`/lidar/points/point_cloud` |
| 実行 | `launch/webots_calibration.launch.py apriltag_calib:=True` |
| 出力（最終） | `~/ros2_ws/apriltag_calib/calib.json`（`results.T_lidar_camera` = `[x,y,z,qx,qy,qz,qw]`、`p_lidar = T * p_camera`） |
| 出力（中間） | `experiments/extrinsic_calibration/<YYYY-MM-DD>_<label>/`（試行版 calib、PnP/平面フィットの中間ログ、複数回測定。gitignore） |
| 利用 | calib.json を `omni_calibration_json:=...` で渡すと `omni_sensor_tf_node` が `lidar_link -> omni_camera_link` TF を置換。色付き点群 / 物体クロップ / 色付き SLAM 地図が同じ TF を使う |

## 方式（2 系統）

| 方式 | 内容 | 位置づけ |
|---|---|---|
| **AprilTag 既知ターゲット** | calibration.wbt の 4 方位パネルに AprilTag 36h11 を貼り、全天球を透視ビュー展開 → `cv2.aruco` 検出 → `solvePnP` でタグのカメラ座標、LiDAR は方位切出し+平面 RANSAC で板中心、Umeyama で 6DoF 推定。`apriltag_extrinsic_calib_node.py` | 本タスクの主対象。`apriltag_ros` 非依存・独自 msg 無し |
| **ターゲットレス** | `direct_visual_lidar_calibration`（equirectangular 対応）を Docker 経由で実行 | 既存導線。`run_direct_visual_lidar_calibration_docker.sh` |

どちらも出力は同じ `calib.json` 形式で、`omni_sensor_tf_node.py` / `scripts/direct_calib_to_tf.py` が読める。

## 実行（AprilTag 方式）

```bash
cd ~/ros2_ws
colcon build --packages-select susumu_object_perception --symlink-install
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# タグテクスチャ生成（初回のみ。webots_worlds/apriltag_textures/ に出る）
ros2 run susumu_object_perception generate_apriltag_textures.py --ids 0,1,2,3

# キャリブ実行 → calib.json を出力
ros2 launch susumu_object_perception webots_calibration.launch.py \
  mode:=realtime rviz:=False perception:=False colored_slam:=False \
  apriltag_calib:=True

# 得た TF で色付け（活用。omni_sensor_tf_node が calib.json を読む）
ros2 launch susumu_object_perception webots_calibration.launch.py \
  colored_slam:=True \
  omni_calibration_json:=~/ros2_ws/apriltag_calib/calib.json
# 色付き点群を PLY 保存
ros2 service call /slam/save_colorized_map std_srvs/srv/Trigger {}
```

## 合格基準

1. **検出と推定**
   calibration.wbt で 4 方位の AprilTag を全て検出し、`calib.json` に `T_lidar_camera` を出力できる。
2. **精度（色付け用途の基準）**
   真値 `lidar_link -> omni_camera_link = z 0.55 / 無回転` に対し **回転 1°未満・対応点 RMS 1cm 程度**。
   現状 realtime で回転 0.32°・RMS 9.6mm を達成。
3. **活用**
   calib.json を `omni_calibration_json:=...` で渡すと TF が初期値 `[0,0,0.55]` からキャリブ実測値へ置換され、
   `colorized_pointcloud_node` がそのキャリブ TF で色付けする。色付け品質（カラー物体の色一致スコア）が
   初期 TF と同等以下に劣化しない（実測: green 0.970→0.976 / magenta 0.985→0.982）。

## 制約・注意

- **並進絶対誤差は現状 24mm 残り、1cm 未満は未達**（x の -23mm 系統オフセット）。主因は LiDAR がタグ板の
  下半分にしか点を返さず点群重心が板中心より下に偏ること（MID-360 上向き FOV + パネル高 0.75m のシミュ
  固有幾何）。板厚補正では消えない。1cm 未満を狙うならパネルを LiDAR 水平面（z0.20）中心へ下げる等が要る。
  色付け用途には回転 0.32°・RMS 9.6mm で十分なので、精密化は必要になってから。
- **LiDAR 点群トピックは `/lidar/points/point_cloud`**（Webots driver の `/point_cloud` サフィックス）。
- **Webots の `Box` テクスチャは回転（`rotation`）の有無で貼られ方が変わる**。タグ板は回転で配置せず
  Box 寸法 + translation で正対させる（回転ありだとタグが白飛びして未検出になる）。
- **CycloneDDS 推奨**（全天球画像が大きく、FastRTPS SHM の罠を避ける）。
- 評価モードは `mode:=realtime`。
- 独自 `.msg` は作らない。検出は OpenCV `cv2.aruco`、出力は vlcal 互換 calib.json。

## 関連

- [全天球カメラ + LiDAR 色付き点群メモ](../omni_lidar_camera.md)（手法・実測・落とし穴の正本）
- [カラー点群出力タスク](colorized_pointcloud.md)（キャリブ結果を使う下流タスク）
- 検証画像: `docs/images/apriltag_calib_*.png`（全天球画像と各方位の透視ビュー）
