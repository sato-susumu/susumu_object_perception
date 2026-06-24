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

### キャリブ成果を使った実行（2026-06-24）

本タスク（カラー点群出力）を、[外部キャリブタスク](extrinsic_calibration.md)で得た
`apriltag_calib/calib.json` を `omni_calibration_json:=` で渡して実行し、全合格基準を満たすことを確認した
（合格基準 5「厳密検証ではキャリブ入力を入れる」を AprilTag 較正結果で満たす実行例）。

```bash
ros2 launch susumu_object_perception webots_colored_slam.launch.py \
  world:=calibration.wbt mode:=realtime rviz:=False \
  perception:=False image_recognition:=False \
  omni_calibration_json:=~/ros2_ws/apriltag_calib/calib.json
# calibration.wbt は静止 world のため、その場回転で全周を蓄積する
ros2 topic pub -r 5 /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.4}}"   # 約18-20秒で1周強
ros2 service call /slam/save_colorized_map std_srvs/srv/Trigger {}
```

結果（成果物 `maps/colorized/colorized_pointcloud_apriltag_calib.ply`、6412 点）— 合格基準 1〜5 を確認:

| 合格基準 | 結果 |
|---|---|
| 1. 色付き点群 publish | PLY に `x/y/z + red/green/blue`、6412 点全て色付き |
| 2. 主要ターゲットの色入れ替わりなし | green box `[64,150,45]`（緑, score 0.885）、magenta cyl `[176,64,160]`（マゼンタ, score 0.964） |
| 3. 蓄積地図が増える | その場回転で 2421 → 6412 点 |
| 4. 保存できる | `/slam/save_colorized_map` が `success: true`、`maps/colorized/` に PLY 保存 |
| 5. キャリブ前提の明示 | `omni_sensor_tf` が `lidar_link->omni_camera_link` を初期 TF でなく**キャリブ値 `[-0.023,0.007,0.548]`** に置換（`tf2_echo` 確認） |

注: `validate_omni_colorization.py` は素の world を自前 launch して `--lidar-z/--camera-z` 固定値で独自投影
するため、`omni_camera_link` TF（= calib.json）を反映しない。キャリブ TF の効果を見る色一致は、実際に
キャリブ TF で色付けした `/perception/colorized_points`（上表）で評価する。静止 world なので並進は増えず、
その場回転で全方位を埋める運用。広域の色付き地図が欲しい場合はロボットが移動する world で同手順を使う。

### 各種屋内環境でのカラー点群出力（キャリブ成果使用、2026-06-25）

外部キャリブの `calib.json` を使い、**屋内 world（indoor / break_room）でロボットを巡回させながら**
色付き点群を蓄積する実行。当初はブレ（壁の二重化・放射状の筋）が出て不合格だったが、3 段階で改善した。

```bash
ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
  world:=indoor.wbt waypoints:=indoor_waypoints.yaml \
  nav_params_file:=nav2_params_webots_explore.yaml \
  mode:=realtime rviz:=False loop:=False \
  perception:=False omni_perception:=True image_recognition:=False \
  colored_slam:=True \
  omni_calibration_json:=~/ros2_ws/apriltag_calib/calib.json
# 巡回完走後
ros2 service call /slam/save_colorized_map std_srvs/srv/Trigger {}
python3 scripts/check_colorized_cloud.py maps/colorized/<name>.ply --true-x 5 --true-y 10
```

**ブレ低減の 3 要素（重要）:**
1. **mapper フィルタ強化**（`webots_simulation.launch.py` の `colored_slam_max_range=7 / min_z=-0.1 /
   max_z=2.0 / voxel 0.05`）。LiDAR 遠距離点は姿勢の微小回転誤差が増幅されて放射状の筋になるので近距離だけ採る。
   床下・天井の散乱も切る。→ 床下点 10.2%→0%、天井外れ消失。
2. **調整済み slam_toolbox 設定**（`nav_params_file:=nav2_params_webots_explore.yaml`）。指定しないと
   TurtleBot3 標準 params の未調整 slam_toolbox になり姿勢がブレる。→ 主要部寸法が真値一致、占有セル
   28177→13574（壁が薄くシャープに）。
3. **キャリブ TF 適用**（`omni_calibration_json`）。`omni_sensor_tf` が初期 TF でなくキャリブ値で色付け。

**結果（`check_colorized_cloud.py`）:**

| world | 点数 | 主要部寸法 (真値) | 床下点 | 占有セル | 目視 |
|---|---|---|---|---|---|
| indoor | 223k | 5.19 x 10.03m (5x10) | 0% | 13574 | 矩形の部屋・什器・色が読み取れる |
| break_room | 336k | 12.84 x 7.92m (12.86x7.7) | 0% | 28375 | 長方形の部屋・仕切り・家具が読み取れる |

成果物: `maps/colorized/colorized_pointcloud_{indoor,break_room}_apriltag_calib_final.ply` と
確認画像 `_check.png`。

**GLIM は不向き（実測）:** 3D loop-closure の GLIM を試したが、Webots は `basicTimeStep 20ms`
（IMU 最大 50Hz）で、GLIM の LiDAR-inertial odometry に必要な高レート IMU を供給できず
`num_imu=0` で odometry が破綻、点群はむしろ悪化した（主要部 9m / 床下 26%）。GLIM は本環境の
カラー点群出力には使わない。

### 品質改善の取り組みと到達点（2026-06-25）

「リアルタイムで色付き点群を出している事例があるなら打開策があるはず」という観点で、世の中の
リアルタイム色付き点群（R3LIVE / FAST-LIO / OmniColor 等）を一次情報で調査し、適用できる打開策を
順に実装・実測した。OmniColor（全天球カメラ×LiDAR 色付け、本リポと同構成）は ghosting/blur の主因を
「不正確な pose と LiDAR-camera 時刻ミスマッチ」と指摘しており、これを軸に検証した。

各手段の効果（indoor、占有セル数=壁の厚みの代理。小さいほどシャープ＝ブレが少ない）:

| 手段 | 占有セル | 効果 |
|---|---|---|
| 初回（フィルタ無し・標準 slam） | — | ブレブレ・床下点 10%・不合格 |
| ① mapper フィルタ強化（max_range7/min_z/max_z/voxel0.05） | 28177 | **最大の効き**。外れ点・床下・天井・遠距離を除去 |
| ② + 調整 slam（nav2_params_webots_explore.yaml） | 13574 | 大きく改善（28177→13574, 52%減）。姿勢が安定 |
| ③ + 時刻同期色付け（点群スタンプに最も近い画像を選択） | 12990 | わずか（4.3%減）。Webots は画像/点群が同一 sim 時刻で元々ズレが小さい |
| ④ + slam 微調整（minimum_travel 0.3→0.15） | 12110 | わずか（6.8%減）かつ 2D 地図壁率は 9.2% に悪化。**未採用** |

**採用版 = ①+②+③**（フィルタ強化 + explore slam 設定 + 時刻同期色付け）。④は壁率が悪化したため不採用
（専用 params は削除）。時刻同期色付け（`colorized_pointcloud_node` の `image_buffer_len` /
`image_sync_max_dt`）は Webots では効果小だが、実機の非同期センサでは効くので採用して残す。

**到達点と残る限界:** 初回の「放射状にブレブレ」状態からは大幅改善し、矩形の部屋構造・什器・色が
読み取れる水準（主要部寸法が真値一致、床下/天井外れ 0、占有セル半減超）になった。ただし壁を完全な
単一線にはできておらず、放射状の筋が残る。真因は**巡回（移動しながら全フレーム蓄積）での SLAM 2D 姿勢
誤差**（巡回中の壁率 8%。止まりながら作るマッピングは 2.6%）で、これは色付けやキャリブでなく姿勢の
問題。Webots は `basicTimeStep 20ms`（IMU 最大 50Hz）で GLIM 等の LIO 系も使えず（[上記参照]）、
2D SLAM の姿勢精度がそのまま上限になる。**根本解決には「静止時のみ蓄積」（移動中の姿勢誤差を点群に
乗せない）が唯一未検証の有力策**として残る。検査は `scripts/check_colorized_cloud.py`（占有セル・
床下点率・主要部寸法・XY/XZ 投影図）で行う。

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

### 履歴サマリ（2026-06-21）

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

代表 run は `validation_passed=true`。レポートは JSON / CSV / Markdown に保存できる。

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
判断: 現状の合格判定は「主要ターゲットの色入れ替わりが無いこと」を確認する実用ゲート。
1deg 未満の精密外部較正を証明するものではないため、精密評価は realtime 4方向 validation と
キャリブレーション入力の整備を次に見る。

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
