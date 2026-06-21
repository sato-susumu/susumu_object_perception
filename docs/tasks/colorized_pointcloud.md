# カラー点群出力タスク — 全天球画像で LiDAR 点群に色を付ける

このページは README のタスク一覧「カラー点群出力」の詳細ページ。3D LiDAR 点群を全天球カメラ画像へ
投影して RGB を付け、必要に応じて SLAM/GLIM 座標に蓄積し、PLY として保存する。

## 入出力

| 項目 | 内容 |
|---|---|
| 入力 | `/lidar/points` または `/lidar/points/point_cloud`、`/omni_camera/image_raw/image_color`、LiDAR-camera TF |
| 実行 | `webots_simulation.launch.py omni_perception:=True colored_slam:=True`、`webots_colored_slam.launch.py`、`webots_glim_colored_slam.launch.py` |
| 点群出力 | `/perception/colorized_points` |
| 蓄積地図 | `/slam/colorized_points_map` または `/slam/glim_colorized_points_map` |
| 保存 | `/slam/save_colorized_map` (`std_srvs/Trigger`) |

## 実行

単発の色付き点群:

```bash
ros2 launch susumu_object_perception webots_simulation.launch.py \
  world:=calibration.wbt nav:=False rviz:=True \
  perception:=False omni_perception:=True image_recognition:=False colored_slam:=False
```

2D SLAM/odom 座標に蓄積する色付き点群地図:

```bash
ros2 launch susumu_object_perception webots_colored_slam.launch.py

ros2 topic echo --once /slam/colorized_points_map --field header \
  --qos-reliability best_effort
ros2 service call /slam/save_colorized_map std_srvs/srv/Trigger {}
```

GLIM の補正済み 3D 座標に蓄積する場合:

```bash
ros2 launch susumu_object_perception webots_glim_colored_slam.launch.py \
  rviz:=False mode:=realtime perception:=False image_recognition:=False

ros2 topic echo --once /slam/glim_colorized_points_map --field header \
  --qos-reliability best_effort
ros2 service call /slam/save_colorized_map std_srvs/srv/Trigger {}
```

`/perception/colorized_points`、`/slam/*colorized_points_map` は sensor QoS のため、`ros2 topic echo` では
`--qos-reliability best_effort` を付ける。

## 合格基準

1. **色付き点群が publish される**
   `/perception/colorized_points` が `x/y/z/rgb` フィールドを持つ `PointCloud2` として出る。
   frame は入力 LiDAR frame（通常 `lidar_link`）。

2. **主要な対象色が入れ替わらない**
   `calibration.wbt` の赤/黄パネル、緑箱、マゼンタ円柱など、方位を変えても明らかな色入れ替わりがない。
   定量確認には `validate_omni_colorization.py` を使う。summary の
   `large_image_projection_error_deg` は大ターゲットだけの投影誤差で、小球マーカー由来の不安定さを
   分けて見るための補助値。合格確認では大ターゲットの色一致率 `min_large_target_score` 以上、
   大ターゲット投影誤差 `max_large_image_error_deg` 以下を満たすこと。

3. **蓄積地図が増える**
   `colored_slam:=True` では `/slam/colorized_points_map` が `map` または `odom` frame で増える。
   GLIM では `/slam/glim_colorized_points_map` が `glim_map` frame で増える。

4. **保存できる**
   `/slam/save_colorized_map` が `success: true` を返し、`maps/colorized/` に PLY が保存される。
   PLY の vertex 数が topic の点数と大きく矛盾しない。

5. **キャリブレーション前提が明示されている**
   未キャリブレーション時は初期 TF を使う。厳密検証では `omni_calibration_json` で
   `direct_visual_lidar_calibration` の結果を入れる。

## 制約と注意

- Webots の全天球カメラは cylindrical projection。色付けは Webots shader に合わせた投影モデルを使う。
- 現状のカラー点群地図は 2D SLAM/odom または GLIM 姿勢に RGB 付き点群を積むもの。2D occupancy map の
  代替ではない。
- GLIM は独立した `glim_*` TF ツリーで動かす。Nav2 の `map/odom/base_link` と混ぜない。
- `mode:=fast` は軽い確認には使えるが、LiDAR/IMU のサンプル不足や時刻外挿が出やすい。厳密検証は
  `validate_omni_colorization.py --mode realtime`。
- 全点に色が入っていても、外部パラメータが正しいとは限らない。位置合わせ品質は
  `validate_omni_colorization.py` やキャリブレーション用 world で確認する。

## 確認コマンド

```bash
ros2 topic hz /omni_camera/image_raw/image_color
ros2 topic hz /perception/colorized_points
ros2 topic echo --once /perception/colorized_points --field fields \
  --qos-reliability best_effort

# 軽い健全性確認（1方位だけ）
ros2 run susumu_object_perception validate_omni_colorization.py \
  --yaws 0 --startup-sec 35 --grab-timeout-sec 15 --require-pass \
  --min-large-target-score 0.40 --max-large-image-error-deg 10.0 \
  --mode fast \
  --out-prefix /tmp/omni_colorization_fast_yaw0

# 厳密確認（時間はかかるが realtime で4方向）
ros2 run susumu_object_perception validate_omni_colorization.py \
  --yaws 0,90,180,270 --startup-sec 35 --grab-timeout-sec 15 --require-pass \
  --min-large-target-score 0.40 --max-large-image-error-deg 10.0 \
  --mode realtime \
  --out-prefix /tmp/omni_colorization_realtime_4yaw
```

`--out-prefix` を指定すると JSON / CSV / Markdown の評価レポートを保存する。標準出力だけだと
過去 run と比較しづらいため、採用/未採用判断ではレポートも成果物として残す。

### 直近結果（2026-06-21）

`validate_omni_colorization.py` に次を追加した。

- `--out-prefix`: JSON / CSV / Markdown の評価レポート保存。
- `--max-large-image-error-deg`: 小球マーカーを除いた大ターゲット専用の投影誤差ゲート。既定 10deg。

軽量 1 方位確認:

```bash
ros2 run susumu_object_perception validate_omni_colorization.py \
  --yaws 0 --startup-sec 35 --grab-timeout-sec 15 --require-pass \
  --min-large-target-score 0.40 --mode fast \
  --out-prefix /tmp/omni_colorization_fast_yaw0_v2
```

結果は `validation_passed=true`。レポートは
`/tmp/omni_colorization_fast_yaw0_v2.{json,csv,md}` に保存された。

| 指標 | 値 |
|---|---:|
| `color_score_mean` | 0.769 |
| `color_score_min` | 0.042 |
| `image_projection_error_deg_mean` | 8.124 |
| `image_projection_error_deg_max` | 19.835 |
| `large_image_projection_error_deg_mean` | 4.708 |
| `large_image_projection_error_deg_max` | 8.175 |

小球の orange / white marker は点数が少なく、score や centroid が不安定なまま。大ターゲット
（赤/黄パネル、緑箱、マゼンタ円柱）は色一致率が合格し、投影誤差も 10deg ゲート内に収まった。
このため現状の合格判定は「主要ターゲットの色入れ替わりが無いこと」を確認する実用ゲートであり、
1deg 未満の精密外部較正を証明するものではない。

方針参考:

- Webots Camera reference: cylindrical projection は equirectangular 画像を生成する。
  <https://www.cyberbotics.com/doc/reference/camera>
- direct_visual_lidar_calibration program details: `--camera_model equirectangular` と
  `calib.json` の `T_lidar_camera` を扱える。
  <https://koide3.github.io/direct_visual_lidar_calibration/programs/>

## 関連

- [全天球カメラ + LiDAR 色付き点群メモ](../omni_lidar_camera.md)
- [ロボット / LiDAR 構成](../robot_lidar.md)
- [launch 一覧](../launch.md)
