# マッピングタスク（屋外） — 特徴が多い屋外 world の自律地図作成

このページは README のタスク一覧「マッピング（屋外）」の詳細ページ。屋内マッピングは別タスク
で[`mapping_indoor.md`](mapping_indoor.md) を参照。

## 現在の本線: GLIM で 3D 点群を作り、その点群から 2D 地図を作る

2026-06-21 の方針転換で、屋外の前提を「特徴の少ない開けた広場」から
**都市部・公園のように MID360 が常時複数特徴を捉えられる空間**へ変更した。
その後の slam_toolbox 2D マッピング評価では、段差・縁石・フェンス付近で yaw が崩れ、
2D LaserScan へ潰してから SLAM する方針自体が屋外 MID360 の情報量を捨てすぎることが分かった。
屋外本線は次の GLIM-first 方針へ切り替える。

1. 屋外専用 world で Webots + MID360 + GLIM を起動し、3D 点群地図を作る。
2. `/slam/glim_colorized_points_map` の PointCloud2 を `save_pointcloud2_to_ply.py` で PLY 保存する。
   GLIM dump/offline viewer からの Export Points は、loop closure 後の高品質出力や手動確認用の
   フォールバックとして残す。
3. `evaluate_glim_map_variants.py` で PLY と任意の `traj_lidar.txt` / topic pose trajectory を
   横並び評価し、選定候補を Nav2 用 2D map (`.pgm/.yaml`) としてプロモートする。
   個別変換が必要な場合だけ `glim_cloud_to_2d_map.py` を直接使う。
4. プロモートした 2D map から屋外 waypoint を作り、保存地図 + AMCL + Nav2 で巡回する。
5. 作成済み地図の妥当性評価に限り、world 由来の正解データと比較する。

重要: `maps/*_gt.yaml` は正解データであり、`map_file` や waypoint 生成の入力にしない。
Nav2 が読む地図は、GLIM のセンサ点群から `glim_cloud_to_2d_map.py` で作った
`maps/village_square_trimmed_glim2d.yaml` / `maps/village_park_trimmed_glim2d.yaml` のような
保存地図にする。world 由来地図は評価にだけ使う。

ただし、屋内の launch / params / world は変更しない。屋外の試行錯誤は次の屋外専用 entry point
と設定だけで行う。

| 役割 | 屋外専用成果物 |
|---|---|
| world | `webots_worlds/village_square_trimmed.wbt`, `webots_worlds/village_park_trimmed.wbt` |
| 3D マッピング | `launch/webots_outdoor_glim_mapping.launch.py` + `config/glim_webots/` |
| PLY 保存 | `scripts/save_pointcloud2_to_ply.py` |
| trajectory 保存 | `scripts/save_pose_trajectory_to_tum.py` |
| 2D map 生成 | `scripts/glim_cloud_to_2d_map.py` |
| 2D map 候補比較 | `scripts/evaluate_glim_map_variants.py` |
| waypoint 生成 | `scripts/generate_outdoor_waypoints.py` |
| 保存地図巡回 | `launch/webots_outdoor_waypoint_nav.launch.py` |
| 正解データ生成/照合 | `scripts/generate_webots_ground_truth_map.py` / `scripts/check_map_vs_world.py`（評価専用） |

2つの新 world は、公式 Webots `village_center.wbt` のロボット周辺にあるフェンス・植栽・ベンチ・
街灯・小建物の密度を小さな区画へ切り出した設計にした。外周は `road_closed_*` の Box バリアで
通行止めにし、frontier が開けた屋外へ伸び続けないようにする。低い床や見た目の舗装だけでなく、
LiDAR が確実に拾える高さ 0.3m 以上の箱・フェンス・植栽・建物を複数方向に置く。

参考にした一次情報:

- GLIM Home: https://koide3.github.io/glim/
- GLIM Getting started: https://koide3.github.io/glim/quickstart.html
- GLIM Installation: https://koide3.github.io/glim/installation.html
- GLIM Docker images: https://koide3.github.io/glim/docker.html
- GLIM Important parameters: https://koide3.github.io/glim/parameters.html
- GLIM Sensor setup guide: https://github.com/koide3/glim/wiki/Sensor-setup-guide
- TUM RGB-D dataset file formats: https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats
- ROS 2 PoseStamped message: https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html
- ROS 2 PointCloud2 message: https://docs.ros.org/en/ros2_packages/humble/api/sensor_msgs/msg/PointCloud2.html
- `sensor_msgs_py.point_cloud2`: https://docs.ros.org/en/iron/p/sensor_msgs_py/sensor_msgs_py.point_cloud2.html
- ROS 2 QoS policies: https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
- Webots WorldInfo: https://cyberbotics.com/doc/reference/worldinfo
- Webots Solid / boundingObject: https://cyberbotics.com/doc/reference/solid
- Nav2 Navigation Concepts: https://docs.nav2.org/concepts/index.html

### 実行手順（新しい屋外本線）

`village_square_trimmed.wbt`:

```bash
ros2 launch susumu_object_perception webots_outdoor_glim_mapping.launch.py \
  world:=village_square_trimmed.wbt \
  mode:=realtime \
  rviz:=True

# 別端末で、走行中の GLIM pose を TUM trajectory として保存する。
ros2 run susumu_object_perception save_pose_trajectory_to_tum.py \
  --topic /glim_ros/pose_corrected \
  --out maps/glim/village_square_trimmed_pose.tum \
  --duration-sec 600 \
  --timeout-sec 660 \
  --min-poses 100 \
  --qos reliable

# 走行で GLIM 点群が十分に育ったら、現在の colorized GLIM map topic を PLY として保存する。
ros2 run susumu_object_perception save_pointcloud2_to_ply.py \
  --topic /slam/glim_colorized_points_map \
  --out maps/glim/village_square_trimmed_points.ply \
  --timeout-sec 30 \
  --min-points 5000 \
  --qos sensor_data

# 走行後に GLIM を終了すると /tmp/dump に traj_lidar.txt 等が残る。
# loop closure 後の高品質出力を使う場合は、GLIM offline_viewer で /tmp/dump を開き、
# File -> Save -> Export Points から同じ PLY パスへ出力する。
# loop closure 後の軌跡を使う場合は `--trajectory /tmp/dump/traj_lidar.txt` に差し替える。
ros2 run glim_ros offline_viewer

# trajectory なし / topic pose / GLIM dump trajectory を同じPLYから横並び評価する。
# GLIM dump がある場合は `--trajectory dump=/tmp/dump/traj_lidar.txt` も追加する。
ros2 run susumu_object_perception evaluate_glim_map_variants.py \
  --cloud maps/glim/village_square_trimmed_points.ply \
  --wbt webots_worlds/village_square_trimmed.wbt \
  --out-prefix maps/village_square_trimmed_glim2d_eval \
  --trajectory topic_pose=maps/glim/village_square_trimmed_pose.tum \
  --adopt-prefix maps/village_square_trimmed_glim2d \
  --waypoints-out maps/village_square_trimmed_glim2d_waypoints.yaml \
  --waypoint-max-segment-length 4.0

# `--waypoint-route-clearance 0.75` は edge 安全余裕の実験用。
# cycle21 live で悪化したため、屋外既定にはしていない。

ros2 run susumu_object_perception generate_webots_ground_truth_map.py \
  --wbt webots_worlds/village_square_trimmed.wbt \
  --out maps/village_square_trimmed_gt.yaml \
  --preview maps/village_square_trimmed_gt.png

ros2 run susumu_object_perception check_map_vs_world.py \
  --wbt webots_worlds/village_square_trimmed.wbt \
  --map maps/village_square_trimmed_glim2d.yaml \
  --out maps/village_square_trimmed_glim2d_vs_world.png \
  --report maps/village_square_trimmed_glim2d_vs_world.json \
  --object-report maps/village_square_trimmed_glim2d_vs_world.csv

ros2 launch susumu_object_perception webots_outdoor_waypoint_nav.launch.py \
  world:=village_square_trimmed.wbt \
  map_file:=$HOME/ros2_ws/src/susumu_object_perception/maps/village_square_trimmed_glim2d.yaml \
  waypoints:=$HOME/ros2_ws/src/susumu_object_perception/maps/village_square_trimmed_glim2d_waypoints.yaml \
  mode:=realtime \
  loop:=False
```

`village_park_trimmed.wbt`:

```bash
ros2 launch susumu_object_perception webots_outdoor_glim_mapping.launch.py \
  world:=village_park_trimmed.wbt \
  mode:=realtime \
  rviz:=True

ros2 run susumu_object_perception save_pose_trajectory_to_tum.py \
  --topic /glim_ros/pose_corrected \
  --out maps/glim/village_park_trimmed_pose.tum \
  --duration-sec 600 \
  --timeout-sec 660 \
  --min-poses 100 \
  --qos reliable

ros2 run susumu_object_perception save_pointcloud2_to_ply.py \
  --topic /slam/glim_colorized_points_map \
  --out maps/glim/village_park_trimmed_points.ply \
  --timeout-sec 30 \
  --min-points 5000 \
  --qos sensor_data

ros2 run glim_ros offline_viewer

ros2 run susumu_object_perception evaluate_glim_map_variants.py \
  --cloud maps/glim/village_park_trimmed_points.ply \
  --wbt webots_worlds/village_park_trimmed.wbt \
  --out-prefix maps/village_park_trimmed_glim2d_eval \
  --trajectory topic_pose=maps/glim/village_park_trimmed_pose.tum \
  --adopt-prefix maps/village_park_trimmed_glim2d \
  --waypoints-out maps/village_park_trimmed_glim2d_waypoints.yaml \
  --waypoint-max-segment-length 4.0

ros2 run susumu_object_perception generate_webots_ground_truth_map.py \
  --wbt webots_worlds/village_park_trimmed.wbt \
  --out maps/village_park_trimmed_gt.yaml \
  --preview maps/village_park_trimmed_gt.png

ros2 run susumu_object_perception check_map_vs_world.py \
  --wbt webots_worlds/village_park_trimmed.wbt \
  --map maps/village_park_trimmed_glim2d.yaml \
  --out maps/village_park_trimmed_glim2d_vs_world.png \
  --report maps/village_park_trimmed_glim2d_vs_world.json \
  --object-report maps/village_park_trimmed_glim2d_vs_world.csv

ros2 launch susumu_object_perception webots_outdoor_waypoint_nav.launch.py \
  world:=village_park_trimmed.wbt \
  map_file:=$HOME/ros2_ws/src/susumu_object_perception/maps/village_park_trimmed_glim2d.yaml \
  waypoints:=$HOME/ros2_ws/src/susumu_object_perception/maps/village_park_trimmed_glim2d_waypoints.yaml \
  mode:=realtime \
  loop:=False
```

### 2026-06-21 方針転換: GLIM-first 屋外マッピング

採用:

- 屋外本線を `slam_toolbox` の `/scan` 2D SLAM から、GLIM の 3D 点群地図生成へ切り替える。
- `webots_outdoor_glim_mapping.launch.py`: `webots_glim_colored_slam.launch.py` を屋外 world 既定で包む
  entry point。RViz は既定 ON、teleop GUI も既定 ON。Nav2 / slam_toolbox は起動しない。
- `glim_cloud_to_2d_map.py`: GLIM/offline_viewer から出した PLY、または PCD を Nav2 map
  (`.pgm/.yaml`) へ変換する。地面帯を free、地面より上の点を occupied に投影し、GLIM の
  `traj_lidar.txt` があれば軌跡から障害物点への raycast で free を補強する。
- `save_pointcloud2_to_ply.py`: `/slam/glim_colorized_points_map` など任意の PointCloud2 topic を
  1サンプル PLY 保存する。改善ループでは手動 offline_viewer を挟まず、GLIM topic から
  `glim_cloud_to_2d_map.py` へ渡せることを優先する。
- `save_pose_trajectory_to_tum.py`: `/glim_ros/pose_corrected` の PoseStamped を TUM trajectory
  (`timestamp tx ty tz qx qy qz qw`) として保存する。`glim_cloud_to_2d_map.py --trajectory`
  に渡し、軌跡から障害物点への raycast free 補強を自動化する。
- `evaluate_glim_map_variants.py`: 同じ GLIM PLY/PCD を trajectory なし / topic pose /
  GLIM dump trajectory で横並び評価し、Nav2用2D地図候補の採用判断を JSON/CSV/Markdown に残す。

未採用:

- `webots_outdoor_mapping.launch.py` の staged frontier / yaw watchdog を屋外本線として続けること。
  watchdog は yaw drift を検出・中断できたが、2D map が崩れた後の回復まではできない。
- world 形状を変えて段差を無くすこと。ユーザー方針により world は変更しない。
- Webots GPS や world 由来真値を SLAM 入力に戻すこと。真値は評価専用のまま。

静的評価:

- `glim_cloud_to_2d_map.py` は既存 `maps/colorized/indoor_colored.ply` で smoke 済み。
  出力 `/tmp/indoor_colored_glim2d.yaml` は `171x284`, `0.05m/pix`、
  `points=80774`, `free_band=16589`, `occupied_band=60831`。
- launch smoke:
  `webots_outdoor_glim_mapping.launch.py world:=village_square_trimmed.wbt mode:=fast rviz:=True
  teleop_gui:=False` で Webots / RViz / GLIM が起動し、`libodometry_estimation_gpu.so`,
  `libsub_mapping.so`, `libglobal_mapping_pose_graph.so` のロードを確認。`/lidar/points_intensity`
  は `width=20160`、`/glim_ros/pose_corrected` は `frame_id=glim_map`、
  `/slam/glim_colorized_points_map` は `width=8023` を publish した。
  ただし `mode:=fast` では GLIM が `insufficient IMU data between LiDAR scans` を多く出すため、
  採用評価や地図作成は `mode:=realtime` で行う。

次の低成績箇所:

1. `village_square_trimmed.wbt` を `mode:=realtime` でフル走行し、
   `save_pose_trajectory_to_tum.py` と `save_pointcloud2_to_ply.py` を並行して記録する。
2. `evaluate_glim_map_variants.py` で topic pose と GLIM dump `traj_lidar.txt` を同じ PLY に対して比較し、
   loop closure 後 trajectory が unknown と GT coverage を改善するか確認する。
3. 採用候補地図から `generate_outdoor_waypoints.py` で waypoint を作り、屋外専用 Nav2 巡回で
   reached/missed を評価する。

### 2026-06-21 改善サイクル15: GLIM topic から PLY を自動保存

低成績箇所:

- GLIM-first 本線は妥当だが、点群 PLY の取り出しが `offline_viewer` 手作業だけだと
  改善ループの `調査 -> 実装 -> 評価` を自動で回せない。

採用:

- `save_pointcloud2_to_ply.py` を追加。PointCloud2 の `x/y/z` と `rgb` / `rgba` /
  `red,green,blue` / `r,g,b` を読み、ASCII PLY として保存する。
- 既定 topic は `/slam/glim_colorized_points_map`、既定 QoS は sensor data。GLIM/colorized
  系のような高頻度点群 topic に合わせ、`--min-points` で地図が十分育つまで待てるようにした。
- `CMakeLists.txt` に install 対象として追加し、`ros2 run susumu_object_perception
  save_pointcloud2_to_ply.py` で起動できるようにした。

未採用:

- `offline_viewer` 経路の削除。GLIM 公式手順では dump/offline viewer から PLY を export できるため、
  loop closure 後の高品質出力や手動レビュー用のフォールバックとして残す。
- 今回の fast-mode smoke map を採用地図にすること。`mode:=fast` では GLIM が
  `insufficient IMU data between LiDAR scans` を多く出すため、品質判断は `mode:=realtime` の
  フル走行で行う。

評価値:

- synthetic PointCloud2: `xyz + rgb(float32)` 3点を `/test_pointcloud2_save` から保存し、
  PLY の色が `255,0,0` / `0,255,0` / `0,0,255` になることを確認。
- build: `colcon build --packages-select susumu_object_perception --symlink-install` 成功。
- live smoke:
  `webots_outdoor_glim_mapping.launch.py world:=village_square_trimmed.wbt mode:=fast rviz:=True
  teleop_gui:=False` で `/slam/glim_colorized_points_map` から
  `maps/glim/village_square_trimmed_cycle15_topic.ply` を保存。`points=10075`, `skipped=0`,
  `frame=glim_map`。
- 2D 変換:
  `maps/village_square_trimmed_cycle15_topic_glim2d.yaml` は `827x756`, `0.05m/pix`,
  `points=10075`, `free=3218`, `occupied=5259`, `unknown=597726`, `rays=0`。
  `eval_map_quality.py` は `41.4x37.8m`, 壁率 `52.8%`, 最大連結成分 `100%`, 連結片数 `1`,
  判定 `OK`。
- GT 比較 smoke:
  `check_map_vs_world.py` は `samples=118`, `inside=118`, `near_occupied=94`,
  `near_ratio_inside=0.797`。fence は `objects_inside=4/4`, `mean_object_coverage=0.819`,
  `inside=102`, `near=81`, `cells={'occupied': 72, 'free': 0, 'unknown': 30}`。
  最低 coverage は `PicketFence` 1件の `0.276` で、未知セルが多い。

参考にした一次情報:

- GLIM Getting started: https://koide3.github.io/glim/quickstart.html
- ROS 2 PointCloud2 message: https://docs.ros.org/en/ros2_packages/humble/api/sensor_msgs/msg/PointCloud2.html
- `sensor_msgs_py.point_cloud2`: https://docs.ros.org/en/iron/p/sensor_msgs_py/sensor_msgs_py.point_cloud2.html
- ROS 2 QoS policies: https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html

次の低成績箇所:

1. `mode:=realtime` で `village_square_trimmed.wbt` をフル走行し、自動保存 PLY と
   GLIM dump/offline_viewer PLY を同じ `glim_cloud_to_2d_map.py` 条件で比較する。
2. `traj_lidar.txt` を使った raycast free 補強を有効にし、今回 `rays=0` だった未知領域の多さを減らす。
3. GT 比較で fence・streetlight・通行止めバリアの対応率を測り、`free_z_*` /
   `obstacle_z_*` / dilation を調整する。

### 2026-06-21 改善サイクル16: GLIM pose から TUM trajectory を自動保存

低成績箇所:

- サイクル15で PointCloud2 から PLY は自動保存できたが、軌跡が無いため
  `glim_cloud_to_2d_map.py` の raycast free 補強が `rays=0` になり、unknown が大きく残った。
  GLIM 終了後の `/tmp/dump/traj_lidar.txt` だけに依存すると、改善ループ中に
  「点群保存 -> 2D変換 -> 評価」を即時に回しにくい。

採用:

- `save_pose_trajectory_to_tum.py` を追加。`/glim_ros/pose_corrected` の PoseStamped を購読し、
  TUM trajectory 形式 `timestamp tx ty tz qx qy qz qw` で保存する。
- `CMakeLists.txt` に install 対象として追加し、`ros2 run susumu_object_perception
  save_pose_trajectory_to_tum.py` で起動できるようにした。
- 屋外手順を、走行中に `save_pose_trajectory_to_tum.py` を並行実行し、その TUM を
  `glim_cloud_to_2d_map.py --trajectory` に渡す形へ変更した。

未採用:

- Webots GPS / world 由来真値を trajectory として地図生成へ入れること。真値は評価専用のまま。
- `offline_viewer` / `/tmp/dump/traj_lidar.txt` の削除。GLIM 公式 dump は loop closure 後の
  高品質軌跡として有用なので、topic pose trajectory と比較するフォールバックに残す。
- 今回の fast-mode smoke map の採用。短時間・ほぼ静止の軌跡なので free ray が放射状に広がり、
  GT 近傍率は少し下がった。採用判断は `mode:=realtime` のフル走行で行う。

評価値:

- synthetic PoseStamped: `/test_pose_trajectory` から `8` pose を保存し、TUM 行が
  `timestamp tx ty tz qx qy qz qw` になっていることを確認。
- 既存 PLY + 原点TUM smoke:
  `maps/glim/village_square_trimmed_cycle15_topic.ply` に原点2 poseのTUMを渡すと
  `rays=137162`、hist は `unknown=597726 -> 415790`, `free=12973 -> 194909`。
  raycast 補強が `glim_cloud_to_2d_map.py` に実際に効くことを確認した。
- live smoke:
  `webots_outdoor_glim_mapping.launch.py world:=village_square_trimmed.wbt mode:=fast rviz:=True
  teleop_gui:=False` で `/glim_ros/pose_corrected` は `geometry_msgs/msg/PoseStamped`、
  QoS `RELIABLE`。`save_pose_trajectory_to_tum.py` は
  `maps/glim/village_square_trimmed_cycle16_pose.tum` に `215` pose を保存。
  同時に `maps/glim/village_square_trimmed_cycle16_topic.ply` は `7459` points。
- 2D 変換:
  `maps/village_square_trimmed_cycle16_glim2d.yaml` は `826x755`, `0.05m/pix`,
  `points=7459`, `free=2755`, `occupied=3581`, `raycast_carved_cells=122490`。
  hist は `occupied=12072`, `unknown=423206`, `free=188352`。
  サイクル15の `unknown=597726` から `174520` cell 減少（約 `29.2%`）。
- map quality:
  `41.3x37.8m`, 壁率 `6.0%`, 最大連結成分 `96%`, 連結片数 `7`, 判定 `OK(微小片あり)`。
- GT 比較 smoke:
  `samples=118`, `inside=118`, `near_occupied=92`, `near_ratio_inside=0.780`。
  fence は `objects_inside=4/4`, `mean_object_coverage=0.808`,
  `inside=102`, `near=80`, `cells={'occupied': 64, 'free': 8, 'unknown': 30}`。
  サイクル15の `near_ratio_inside=0.797`, fence mean `0.819` より少し下がったため、
  短時間静止軌跡のraycastは「unknown削減の動作確認」に留める。

参考にした一次情報:

- GLIM Getting started: https://koide3.github.io/glim/quickstart.html
- TUM RGB-D dataset file formats: https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats
- ROS 2 PoseStamped message: https://docs.ros2.org/foxy/api/geometry_msgs/msg/PoseStamped.html
- evo trajectory formats: https://github.com/MichaelGrupp/evo/wiki/Formats

次の低成績箇所:

1. `mode:=realtime` で `village_square_trimmed.wbt` をフル走行し、走行開始から終了まで
   `save_pose_trajectory_to_tum.py` を動かして面状の trajectory を取る。
2. `evaluate_glim_map_variants.py` で「trajectory なし / topic pose trajectory /
   `/tmp/dump/traj_lidar.txt`」を同じ PLY から比較し、unknown、free過多、GT coverage を見る。
3. `raycast-max-range`、`raycast-max-points`、`free_z_*`、`obstacle_z_*` を調整し、
   fence coverage を維持したまま unknown を減らす。

### 2026-06-21 改善サイクル17: GLIM 2D map variant 評価の自動化

低成績箇所:

- サイクル16で topic pose trajectory による raycast free 補強は動くようになったが、
  「trajectory なし / topic pose / GLIM dump `traj_lidar.txt`」の比較が手作業のままだった。
  屋外本線を進めるには、同じ PLY を複数条件で2D地図化し、unknown と world 評価を同じ基準で
  横並びにする必要がある。

採用:

- `evaluate_glim_map_variants.py` を追加。1つの GLIM PLY/PCD に対して、`none` baseline と
  `--trajectory LABEL=PATH` で指定した複数 trajectory 条件を一括評価する。
- 各 variant で `glim_cloud_to_2d_map.py`、`eval_map_quality.py`、
  `check_map_vs_world.py` を実行し、`*_summary.json/.csv/.md` と variant ごとの
  map / preview / world overlay / object CSV を保存する。
- 採用候補の選定規則は「no-trajectory baseline から `near_ratio_inside` と
  `fence_mean_coverage` を `0.02` 超えて落とさず、`unknown_cells` が最小の variant」とした。
  正解データは評価にだけ使い、2D地図生成には入れない。

未採用:

- 目視で PNG を見て採用候補を決める運用。評価値が残らず、次サイクルで同じ判断を再現できない。
- `none` trajectory の現行データ採用。fence coverage は同じだが、unknown が多すぎる。
- 今回の fast-mode 短時間地図を最終採用地図にすること。GLIM は fast で
  `insufficient IMU data between LiDAR scans` を多く出すため、採用品質の判断は realtime
  フル走行に限る。

評価値:

- 既存 live 成果物比較:
  `maps/glim/village_square_trimmed_cycle16_topic.ply` (`7459` points) と
  `maps/glim/village_square_trimmed_cycle16_pose.tum` (`215` pose) を入力に
  `maps/village_square_trimmed_cycle17_variants_summary.*` を生成した。
- `none`: `rays=0`, `unknown=599293`, `free=12265`, `occupied=12072`,
  `near_ratio_inside=0.779661`, `fence_mean_coverage=0.807602`,
  fence cells `free/unknown/occupied=0/38/64`。
- `topic_pose`: `rays=122490`, `unknown=423206`, `free=188352`, `occupied=12072`,
  `near_ratio_inside=0.779661`, `fence_mean_coverage=0.807602`,
  fence cells `free/unknown/occupied=8/30/64`。coverage を下げずに unknown を
  `176087` cell 減らしたため、このデータでは採用候補。
- cycle17 live smoke:
  `webots_outdoor_glim_mapping.launch.py world:=village_square_trimmed.wbt mode:=fast
  rviz:=True teleop_gui:=False` を起動し、RViz 起動、GLIM topic、`/slam/glim_colorized_points_map`
  を確認。`maps/glim/village_square_trimmed_cycle17_live_topic.ply` は `7440` points、
  `maps/glim/village_square_trimmed_cycle17_live_pose.tum` は `211` pose。
  `maps/village_square_trimmed_cycle17_live_variants_summary.*` でも `topic_pose` が採用候補:
  `none unknown=599289`、`topic_pose rays=122558`, `unknown=423207`,
  `near_ratio_inside=0.779661`, `fence_mean_coverage=0.807602`。

参考にした一次情報:

- GLIM Getting started: https://koide3.github.io/glim/quickstart.html
  (`/tmp/dump`、`traj_lidar.txt`、offline viewer の PLY export)
- TUM RGB-D dataset file formats: https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats
  (`timestamp tx ty tz qx qy qz qw`)
- Nav2 Map Server: https://docs.nav2.org/configuration/packages/configuring-map-server.html
  (Nav2 が読む grid map / metadata の扱い)

次の低成績箇所:

1. `mode:=realtime` のフル走行で `topic_pose` と GLIM dump `/tmp/dump/traj_lidar.txt` を
   `evaluate_glim_map_variants.py` に同時入力し、loop closure 後 trajectory が
   coverage を保ったまま unknown をさらに減らすか確認する。
2. 採用候補の2D地図から `generate_outdoor_waypoints.py` で屋外waypointを生成し、
   `webots_outdoor_waypoint_nav.launch.py` で保存地図 + Nav2 巡回へ進める。
3. `fence_free_samples=8` のように真値障害物上を free に削る副作用が増えたら、
   `raycast-max-range` / obstacle dilation / 高さ帯を調整して coverage を維持する。

### 2026-06-21 改善サイクル18: 採用候補地図のプロモートと waypoint 接続

低成績箇所:

- サイクル17で `topic_pose` variant は採用候補になったが、候補を安定した Nav2 map 名へ移し、
  そのまま waypoint 生成と巡回 launch に渡す部分が手作業だった。
- 生成直後の source 側 `maps/` にある地図や waypoint を basename で渡すと、launch が
  install/share 側を見に行くため、ビルド前の成果物をライブ検証に使えなかった。

採用:

- `evaluate_glim_map_variants.py` に `--adopt-prefix` を追加した。選定された variant の
  `.yaml/.pgm/.png/.json/_vs_world.*` を指定 prefix へコピーし、promoted YAML の `image:` は
  promoted PGM の basename に書き換える。Nav2 map_server が単体 YAML として読める形にするため。
- 同スクリプトに `--waypoints-out` と屋外 waypoint 生成パラメータを追加し、選定/プロモート後の
  map から `generate_outdoor_waypoints.py` を続けて実行できるようにした。
- `webots_waypoint_nav.launch.py` と `webots_outdoor_waypoint_nav.launch.py` は waypoint YAML に
  絶対パスを渡せるようにした。生成直後の source 側成果物をビルドや install 同期なしで
  `webots_outdoor_waypoint_nav.launch.py` へ渡せる。

未採用:

- `maps/village_square_trimmed_cycle18_promoted_glim2d.yaml` と
  `maps/village_square_trimmed_cycle18_promoted_glim2d_waypoints.yaml` を巡回用の最終採用地図に
  すること。地図候補としては読み込めたが、Nav2 巡回は `reached=1/35` で止まった。
- world 変更。今回も `village_square_trimmed.wbt` は変更していない。

評価値:

- 入力: `maps/glim/village_square_trimmed_cycle17_live_topic.ply` (`7440` points) と
  `maps/glim/village_square_trimmed_cycle17_live_pose.tum` (`211` pose)。
- variant 比較: `none` は `rays=0`, `unknown=599289`,
  `near_ratio_inside=0.779661`, `fence_mean_coverage=0.807602`。
  `topic_pose` は `rays=122558`, `unknown=423207`,
  `near_ratio_inside=0.779661`, `fence_mean_coverage=0.807602` で選定。
- promoted map:
  `maps/village_square_trimmed_cycle18_promoted_glim2d.yaml`。
  `826x755` cells, `41.3x37.8m`, `resolution=0.05`,
  `origin=[-20.470119, -17.094547, 0]`,
  occupied/free/unknown=`12078/188345/423207`。
  `eval_map_quality.py` は wall rate `6.0%`, max component `96%`, components `7`,
  verdict `OK(微小片あり)`。
- world 評価: samples `118`, inside `118`, near `92`,
  `near_ratio_inside=0.779661`。fence `mean_object_coverage_inside=0.807602`,
  fence cell counts occupied/free/unknown=`64/8/30`。obstacle coverage mean `0.75`。
- waypoint 生成:
  `maps/village_square_trimmed_cycle18_promoted_glim2d_waypoints.yaml` に `35` 点。
  coverage area `271m2`, total geodesic path `137.2m`,
  max geodesic jump `16.2m`。この最大ジャンプは巡回品質として低成績。
- ライブ smoke:
  `webots_outdoor_waypoint_nav.launch.py` を `rviz:=True`、source 側 map/waypoints の絶対パスで起動。
  map_server は source 側 `village_square_trimmed_cycle18_promoted_glim2d.yaml` と PGM を読み、
  RViz、AMCL、Nav2 lifecycle active、35 waypoint 読み込みを確認。
  #0 は到達したが、mission timeout `45.126s` で `reached=1/35`, `missed=34`。
  planner log には `Starting point in lethal space! Cannot create feasible plan.` が出た。

参考にした一次情報:

- GLIM Getting started: https://koide3.github.io/glim/quickstart.html
  (`/tmp/dump`、`traj_lidar.txt`、offline viewer の PLY export)
- Nav2 Map Server: https://docs.nav2.org/configuration/packages/configuring-map-server.html
  (Map Server は OccupancyGrid map の load/save/publish を扱う)
- Nav2 map_server README: https://github.com/ros-navigation/navigation2/blob/main/nav2_map_server/README.md
  (YAML の `image` / `resolution` / `origin` と `yaml_filename` パラメータ)
- TUM RGB-D file formats: https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats
  (`timestamp tx ty tz qx qy qz qw`)

次の低成績箇所:

1. promoted GLIM 2D map の Nav2 到達性。初期 pose、#0/#1、local/global costmap 上の
   lethal cell を保存地図と照合し、`Starting point in lethal space` の原因を静的地図、
   inflation、または動的 obstacle layer に分ける。
2. waypoint 生成の到達性前処理。最大測地ジャンプ `16.2m` を抑え、ComputePathToPose または
   inflation 済みグリッドで各 waypoint/edge を事前検査してから YAML を出す。
3. `glim_cloud_to_2d_map.py` の free carving と obstacle dilation の調整。unknown を減らしつつ、
   fence 上 free sample `8` と obstacle free/unknown を増やしすぎない条件を探す。

### 2026-06-21 改善サイクル19: 屋外 waypoint の長距離 edge 分割

低成績箇所:

- サイクル18の promoted GLIM 2D map は Nav2 map として読めたが、屋外巡回 smoke は
  `reached=1/35` で、最大測地ジャンプ `16.2m` の長い waypoint 区間が残っていた。
- planner log に `Starting point in lethal space! Cannot create feasible plan.` が出たため、
  waypoint を増やすだけで解けるとは断定せず、Nav2 の costmap/lethal 判定も確認した。

採用:

- `generate_waypoints.py` に `--max-segment-length` を追加した。巡回順を保ったまま、長い測地区間だけ
  passable グリッド上の最短経路で分割する。中間点は可能な限り通常 waypoint と同じ配置 clearance
  を満たすセルに置く。
- `generate_outdoor_waypoints.py` の屋外既定を `--max-segment-length 4.0` にした。屋内の既定は
  `0.0` のままなので、屋外の試行錯誤が屋内 waypoint 生成へ勝手に波及しない。
- `evaluate_glim_map_variants.py` に `--waypoint-max-segment-length` を追加し、promote と同時生成する
  屋外 waypoint へ同じ上限を渡せるようにした。

未採用:

- unsafe な厳密 4m 分割。最大測地ジャンプは `4.1m` まで下がったが、追加 waypoint の最小 clearance が
  `0.354m` になり、Nav2 の inflation / footprint 余裕として不十分だった。
- `spacing=2.0`, `max_waypoints=80` の密な経路。最大測地ジャンプは `9.5m` までしか下がらず、
  80点巡回は評価時間を伸ばす割に根本原因を分けにくい。
- cycle19 の経路を「巡回合格」として採用すること。ライブ smoke は改善したが未合格。

評価値:

- 入力 map は cycle18 と同じ `topic_pose` promoted GLIM 2D map。
  `topic_pose`: `rays=122558`, `unknown=423207`, `near_ratio_inside=0.779661`,
  `fence_mean_coverage=0.807602`。`none` baseline は `rays=0`, `unknown=599289`。
- waypoint 静的評価:
  `maps/village_square_trimmed_cycle19_promoted_glim2d_waypoints.yaml` に `53` 点。
  `inserted=18`, total geodesic path `137.2m`, max geodesic jump `7.9m`,
  max straight jump `5.7m`。全 waypoint が free かつ clearance `>=0.75m`。
  cycle18 の `35` 点、max geodesic jump `16.2m` から改善した。
- world 評価は地図内容が cycle18 と同じため据え置き:
  samples `118`, near `92`, `near_ratio_inside=0.779661`,
  fence `mean_object_coverage_inside=0.807602`, obstacle coverage mean `0.75`。
- ライブ smoke:
  `webots_outdoor_waypoint_nav.launch.py` を `rviz:=True`、source 側 map/waypoints の絶対パス、
  `goal_timeout_sec:=60.0`, `mission_timeout_sec:=150.0` で起動。
  RViz、AMCL、Nav2 lifecycle active、53 waypoint 読み込みを確認。
  結果は `maps/village_square_trimmed_cycle19_nav_smoke_segmented.*` に保存し、
  `mission_timeout`, `reached=9/53`, `missed=44`。
  #0-#6 は連続到達、#12 と #15 も到達したが、#7 以降で
  `Starting point in lethal space`、`None of the points of the global plan were in the local costmap`、
  `No valid trajectories` が多発した。

参考にした一次情報:

- Nav2 Costmap 2D: https://docs.nav2.org/configuration/packages/configuring-costmaps.html
  (`lethal_cost_threshold` は occupancy grid のどの値を lethal obstacle とみなすかを決める)
- Nav2 Inflation Layer: https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html
  (障害物周辺に cost を広げ、inscribed radius 内を lethal にする)
- Nav2 ComputePathToPose: https://docs.nav2.org/configuration/packages/bt-plugins/actions/ComputePathToPose.html
  (planner は現在 pose または明示 start pose から goal への path を生成する)
- navigation2 issue #3992: https://github.com/ros-navigation/navigation2/issues/3992
  (`Starting point in lethal space` と costmap/raytrace 境界問題の上流事例)

次の低成績箇所:

1. waypoint 生成ではなく、巡回中の pose / global costmap / local costmap のずれをリアルタイム記録する。
   #6 以降の現在 pose、AMCL particle、odom、global_costmap、local_costmap、scan を同じ時刻で保存し、
   lethal start が静的 map、AMCL drift、obstacle layer、local costmap window のどこで発生したかを分ける。
2. `webots_outdoor_waypoint_nav.launch.py` に評価用 monitor を追加し、`Starting point in lethal space` や
   `global plan not in local costmap` の直前直後を JSON/PNG で残す。
3. その結果が pose drift なら AMCL/odom/IMU 融合、costmap 側なら inflation/obstacle layer、
   地図側なら GLIM 2D map の free carving / dilation をそれぞれ屋外専用設定で見直す。

### 2026-06-21 改善サイクル20: Nav2 pose/costmap 診断の常設化

低成績箇所:

- サイクル19は waypoint の長距離 edge を減らしたが、ライブ巡回は `reached=9/53` で未合格。
  `Starting point in lethal space` と `None of the points of the global plan were in the local costmap`
  が同じ run に出ており、静的地図、global costmap、local costmap、pose drift のどれが主因か
  ログだけでは切り分けられなかった。

採用:

- `scripts/nav2_pose_costmap_monitor_node.py` を追加した。これは観測専用で、TF/goal/costmap へは
  何も書き戻さない。`/map`, `/global_costmap/costmap`, `/local_costmap/costmap`, `/plan`,
  `/scan`, `/waypoint_nav/status` を同時サンプルし、ロボット pose が各 grid 上で何値か、
  global plan が local costmap 内の free セルへ入っているかを JSON/CSV/Markdown/PNG に残す。
- `webots_outdoor_waypoint_nav.launch.py` に屋外専用引数 `costmap_monitor` と
  `costmap_monitor_prefix` を追加した。既定 OFF なので屋内巡回や通常屋外巡回には影響しない。
  評価時だけ `costmap_monitor:=True` で起動する。
- 診断分類は次を優先する:
  `pose_static_lethal`, `pose_global_lethal_static_free`,
  `pose_local_lethal_static_free`, `plan_not_in_local_costmap`,
  `plan_no_free_points_in_local_costmap`。

未採用:

- Nav2 params の即時変更。今回の目的は、どの層が start cell / global plan を壊しているかを
  実測で分けること。根拠なしに global/local inflation や planner を調整することはしない。
- world 変更、または world/GPS 真値を Nav2 入力へ戻すこと。今回も真値は使わず、保存地図と
  Nav2 の実 runtime topic だけで診断した。

評価値:

- 静的検証:
  `python3 -m py_compile scripts/nav2_pose_costmap_monitor_node.py
  launch/webots_outdoor_waypoint_nav.launch.py` 成功。
  `ros2 launch ... --show-args` で `costmap_monitor` / `costmap_monitor_prefix` が見えることを確認。
  `colcon build --packages-select susumu_object_perception --symlink-install` 成功。
- ライブ smoke:
  `webots_outdoor_waypoint_nav.launch.py` を `rviz:=True`, `costmap_monitor:=True`,
  source 側 promoted map/waypoints の絶対パス、`goal_timeout_sec:=60.0`,
  `mission_timeout_sec:=150.0` で起動。
  ナビ結果は `maps/village_square_trimmed_cycle20_nav_smoke_monitor.*` に保存し、
  `mission_timeout`, `reached=16/53`, `missed=37`。
  cycle19 の `reached=9/53` より進んだが、合格ではない。
- monitor 結果:
  `maps/village_square_trimmed_cycle20_pose_costmap_monitor.{json,csv,md,png}` を生成。
  samples `380`, events `51`。
  diagnosis counts は
  `ok=169`, `pose_global_lethal_static_free=94`, `pose_static_lethal=110`,
  `plan_not_in_local_costmap=3`, `pose_global_lethal=1`, `missing_tf=3`。
- 最初の有効 event は #6 で、map pose `(-0.326, 5.705)`、
  `static=0`, `global=99`, `local=0`, `local_plan=48/48`。
  保存地図上は free なのに global costmap が lethal 相当になり、その直後に
  planner の `Starting point in lethal space` が出た。
- #14 では `pose_global_lethal_static_free` が続いた後、map pose `(4.603, 1.682)` で
  `static=100`, `global=100`, `local=0`, `local_plan=0/0` へ遷移した。
  ここから recovery 後も `pose_static_lethal` が継続し、後続 waypoint が連続して失敗した。
- 末尾では map pose `(-2.623, 4.133)` で `static=0`, `global=99`, `local=0`,
  `local_plan=0/0`, scan front `0.75-0.85m` が継続した。
  これは「global plan が local costmap に入らない」症状も monitor で再現できたことを示す。

解釈:

- 低成績の主因は単純な waypoint 間隔ではない。global costmap inflation が保存地図 free 上の
  pose を高コスト化し、recovery 中または直後に robot pose が保存地図 occupied へ入り込む。
  その後は planner の start cell が静的地図上でも lethal になり、`Starting point in lethal space`
  が連鎖する。
- local costmap の robot pose は今回の代表 event では `local=0` のままなので、少なくとも最初の
  failure は local obstacle layer だけでは説明できない。

参考にした一次情報:

- Nav2 Costmap 2D: https://docs.nav2.org/configuration/packages/configuring-costmaps.html
  (costmap は planner/controller が collision や高コスト領域を判断する 2D grid。`lethal_cost_threshold`,
  `global_frame`, `robot_base_frame`, `rolling_window` の意味も確認)
- Nav2 Inflation Layer: https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html
  (障害物周辺へ cost を広げ、inscribed radius 内を lethal にする)
- Nav2 DWB `PathDistCritic` source:
  https://github.com/ros-navigation/navigation2/blob/main/nav2_dwb_controller/dwb_critics/src/path_dist.cpp
  (`worldToMap` で global plan 点が local costmap 内かつ known にならない場合に
  `None of the ... global plan ... local costmap and free` を出す)
- navigation2 issue #3567:
  https://github.com/ros-planning/navigation2/issues/3567
  (同じ DWB/local costmap エラーと TF/costmap が乱れる上流事例)

次の低成績箇所:

1. 屋外 GLIM saved-map patrol 専用 Nav2 params を分け、global costmap の inflation を弱める、または
   global planner 用の robot radius / cost threshold を見直す。local costmap は衝突回避用に維持し、
   global costmap が start cell を潰さない構成を試す。
2. `maps/village_square_trimmed_cycle20_pose_costmap_monitor.csv` の event 座標
   `(-0.326,5.705)`, `(4.603,1.682)`, `(-2.623,4.133)` を GLIM 2D map / world / waypoint path と
   重ね、map 由来の細い occupied や unknown 境界を通っていないか確認する。
3. waypoint 生成の edge 検査を「保存地図 free」だけでなく、global inflation 相当の高コスト領域を
   避ける基準へ拡張する。候補 edge が inflated cost を踏む場合は中間点をずらすか、その edge を
   避ける順序へ組み替える。

### 2026-06-21 改善サイクル21: route clearance 実験と未採用判定

低成績箇所:

- サイクル20の低成績は、保存地図上では free の現在 pose が global costmap で `99`
  (inscribed inflated obstacle 相当) になり、`Starting point in lethal space` と
  `None of the points of the global plan were in the local costmap` が連鎖することだった。
- 当初仮説は「waypoint の点自体は `clearance=0.75m` でも、点間の測地 edge は
  `connect-clearance=0.35m` 上で最短路を取るため、壁際や unknown 境界に寄りすぎる」だった。

採用:

- `scripts/generate_waypoints.py` に `--route-clearance` を追加した。未指定時は従来どおり
  `--connect-clearance` を使うため、屋内と既存屋外既定には影響しない。
- route clearance は測地距離行列と `--max-segment-length` の edge 分割に使う passable 領域を
  絞るための実験オプション。静的に「edge が inflation に食い込むか」を試す用途で残す。
- `scripts/generate_outdoor_waypoints.py` と `scripts/evaluate_glim_map_variants.py` からも
  明示指定時だけ route clearance を渡せるようにした。

未採用:

- `--route-clearance 0.75` を屋外 waypoint 生成の既定にすること。静的には安全側へ寄るが、
  下記 live smoke で `reached=9/49` へ悪化したため、既定値は従来互換の
  `route_clearance == connect_clearance` に戻した。
- Nav2 global inflation / robot radius の即時変更。サイクル21では waypoint edge 仮説の検証に
  限定し、Nav2 params は変更していない。
- world 変更、または world/GPS 真値を Nav2 入力へ戻すこと。今回も真値は評価にだけ使う。

評価値:

- 静的 waypoint 生成:
  `village_square_trimmed_cycle19_promoted_glim2d.yaml` に対し
  `--route-clearance 0.75 --max-segment-length 4.0` を明示して生成。
  `53` 点から `49` 点へ減り、route 対象面積は約 `271m2` から `192m2` へ縮小。
  total geodesic path は `137.2m` から `121.5m`、max geodesic jump は `7.9m` から `4.0m`、
  straight max は `5.7m` から `4.0m` へ改善し、waypoint の最小 clearance は `0.75m` を維持した。
- GLIM variant evaluator:
  `topic_pose` が採用候補で、map 指標はサイクル19/20と同じ
  `near_ratio_inside=0.779661`, `fence_mean_coverage=0.807602`。
  `--waypoints-out` に `--waypoint-route-clearance 0.75` を明示し、
  `maps/village_square_trimmed_cycle21_promoted_glim2d_waypoints.yaml` を生成した。
- ライブ smoke:
  `webots_outdoor_waypoint_nav.launch.py` を `rviz:=True`, `costmap_monitor:=True`,
  `goal_timeout_sec:=60.0`, `mission_timeout_sec:=150.0` で起動。
  ナビ結果は `maps/village_square_trimmed_cycle21_nav_smoke_routeclear.*` に保存し、
  `mission_timeout`, `reached=9/49`, `missed=40`。
  cycle20 の `reached=16/53` より悪化した。
- monitor 結果:
  `maps/village_square_trimmed_cycle21_pose_costmap_monitor.{json,csv,md,png}` を生成。
  samples `406`, events `28`。
  diagnosis counts は
  `ok=304`, `plan_not_in_local_costmap=17`,
  `pose_global_lethal_static_free=37`, `pose_global_lethal=35`,
  `pose_static_lethal=8`, `plan_no_free_points_in_local_costmap=4`,
  `local_missing=1`。
- 最初の有効 event は #8 で、map pose `(4.624, -11.583)`、
  `static=0`, `global=0`, `local=43`, `local_plan=0/0`。
  つまり開始時点では global inflation ではなく、DWB が global plan を local costmap 内の
  free セルとして読めないことが先に起きた。
- 直後に map pose `(6.830, -10.094)` へ移り、`static=-1`, `global=99`, `local=53`,
  `local_plan=0/0` となった。recovery 後も #10 で `(7.030, -8.180)`,
  `static=0`, `global=99`, `local=0` が続き、さらに `(7.008, -8.499)`,
  `static=100`, `global=100`, `local=0` へ入った。

解釈:

- route clearance は「静的 edge が細い場所を通る」問題を減らすが、今回の主因ではなかった。
  悪化の直接原因は、局所 controller / recovery 中に global plan が local costmap へ入らない状態、
  あるいは robot pose が inflated/unknown/static lethal 領域へ移る状態で、その後の waypoint が
  連鎖的に失敗すること。
- 次の改善は waypoint 生成をさらに局所的にいじるのではなく、屋外 saved-map patrol 専用の
  Nav2 behavior tree / recovery / controller / local costmap サイズ・更新周期・plan prune の
  整合を大きく見直す。

参考にした一次情報:

- Nav2 Inflation Layer:
  https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html
  (`inflation_radius` と robot inscribed radius 内の lethal cost を確認)
- Nav2 Smac 2D Planner:
  https://docs.nav2.org/configuration/packages/smac/configuring-smac-2d.html
  (`cost_travel_multiplier` は高コスト領域から離れた経路を促す)
- Nav2 Costmap 2D:
  https://docs.nav2.org/configuration/packages/configuring-costmaps.html
  (`robot_radius` と OccupancyGrid の `INSCRIBED_INFLATED_OBSTACLE=99` を確認)
- Nav2 DWB `PathDistCritic` source:
  https://github.com/ros-navigation/navigation2/blob/main/nav2_dwb_controller/dwb_critics/src/path_dist.cpp
  (global plan 点が local costmap 内かつ free として入らない場合のエラー条件を確認)

次の低成績箇所:

1. 屋外 saved-map patrol 専用の Nav2 recovery を見直す。特に失敗後の `spin` / `backup` が
   global plan と local costmap の整合を壊す場合、屋外専用 BT または recovery 無効化/縮小を試す。
2. `plan_not_in_local_costmap` を先に潰すため、local costmap サイズ、planner/controller の
   plan pruning、DWB の `prune_plan`/forward simulation、controller frequency の整合を調べる。
3. global inflation を弱める案はまだ候補だが、cycle21 の最初の event は `global=0` なので
   最初に触るべき主因ではない。次は controller/recovery と local costmap を優先する。

### 2026-06-21 改善サイクル22: no-recovery BT 実験と static lethal 主因化

低成績箇所:

- サイクル21の「次」は、屋外 saved-map patrol 専用の Nav2 recovery / controller /
  local costmap 整合だった。特に default BT の `Spin` / `BackUp` が失敗後に robot pose を
  inflated/static lethal へ動かしている可能性を疑った。

採用:

- `waypoint_nav_node.py` に `behavior_tree` パラメータを追加した。空なら従来どおり Nav2 側
  `default_bt_xml_filename` を使う。明示した場合だけ `NavigateToPose.Goal.behavior_tree` に渡す。
  屋内巡回の既定は変えない。
- `webots_waypoint_nav.launch.py` / `webots_outdoor_waypoint_nav.launch.py` に `behavior_tree`
  引数を追加した。屋外 wrapper でも既定は空なので、通常 run は Nav2 既定 recovery BT のまま。
- `behavior_trees/outdoor_patrol_replanning_no_recovery.xml` を追加した。これは
  `RateController(1Hz) -> ComputePathToPose -> FollowPath` だけの診断用 BT で、
  `Spin` / `Wait` / `BackUp` / costmap clear recovery を含めない。

未採用:

- no-recovery BT を屋外既定にすること。下記 live smoke で `reached=6/53` に悪化したため、
  `webots_outdoor_waypoint_nav.launch.py behavior_tree` の既定は空に戻した。
- recovery を主因とみなして調整を続けること。最初の破綻は #6 で `pose_static_lethal` だったため、
  recovery だけでは説明できない。

評価値:

- 静的検証:
  `python3 -m py_compile susumu_object_perception/waypoint_nav_node.py
  launch/webots_waypoint_nav.launch.py launch/webots_outdoor_waypoint_nav.launch.py` 成功。
  `xml.etree.ElementTree` で
  `behavior_trees/outdoor_patrol_replanning_no_recovery.xml` を parse 成功。
  `git diff --check` 成功。
  `colcon build --packages-select susumu_object_perception --symlink-install` 成功し、
  install/share に BT XML が入ることを確認。
- ライブ smoke:
  `webots_outdoor_waypoint_nav.launch.py` を `rviz:=True`, `costmap_monitor:=True`,
  `behavior_tree:=.../outdoor_patrol_replanning_no_recovery.xml`,
  cycle19 promoted map/waypoints、`goal_timeout_sec:=60.0`,
  `mission_timeout_sec:=150.0` で起動。
  ナビ結果は `maps/village_square_trimmed_cycle22_nav_smoke_no_recovery.*` に保存し、
  `complete`, `reached=6/53`, `missed=47`, `elapsed=28.199s`。
  cycle20 の `reached=16/53` より悪化した。
- monitor 結果:
  `maps/village_square_trimmed_cycle22_pose_costmap_monitor_no_recovery.{json,csv,md,png}` を生成。
  samples `125`, events `17`。
  diagnosis counts は
  `ok=48`, `pose_global_lethal_static_free=75`, `pose_static_lethal=1`, `missing_tf=1`。
- 最初の有効 event は #6 で、map pose `(-0.142, 5.671)`、
  `static=100`, `global=100`, `local=0`, `local_plan=52/52`, scan front `2.605m`。
  つまり、local plan は local costmap 上で free だが、robot pose は保存地図上の occupied に入った。
- no-recovery BT では failure 後に spin/backUp は走らなかったが、#6 で start cell が
  static/global lethal になった後、#7 以降は planner が即 `Starting point in lethal space` で失敗し、
  waypoint が高速に action failure になった。

解釈:

- no-recovery は「recovery が原因で lethal に入る」仮説を否定した。recovery を切っても #6 で
  static lethal に入るため、次の主因は「#5→#6 の実走行軌跡が保存地図 occupied / inflated cell を踏む」
  こととして扱う。
- monitor の `local_plan=52/52` は、local costmap だけを見ると経路が free であることを示す。
  しかし global/static 上では pose が lethal になるので、次は local controller が従う実軌跡と
  static/global costmap の不整合を明示的に検査する。

参考にした一次情報:

- Nav2 Detailed Behavior Tree Walkthrough:
  https://docs.nav2.org/behavior_trees/overview/detailed_behavior_tree_walkthrough.html
  (default `navigate_to_pose_w_replanning_and_recovery.xml` は 1Hz replanning と recovery actions を含む)
- Nav2 Behavior Server:
  https://docs.nav2.org/configuration/packages/configuring-behavior-server.html
  (`Spin`, `BackUp`, `Wait` などの behavior/recovery と `simulate_ahead_time` を確認)
- Nav2 DWB Controller:
  https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html
  (DWB は critic-based local planner で、`PathDist` / `GoalDist` 等の critic を使う)
- Nav2 Costmap 2D:
  https://docs.nav2.org/configuration/packages/configuring-costmaps.html
  (planner/controller は costmap を collision や高コスト領域判断に使う)
- navigation2 issue #3375:
  https://github.com/ros-navigation/navigation2/issues/3375
  (`Resulting plan has 0 poses in it` と DWB/local costmap/window の関係を確認)

次の低成績箇所:

1. `maps/village_square_trimmed_cycle22_pose_costmap_monitor_no_recovery.csv` の #6 周辺
   `(-0.142, 5.671)` と、#5 `(-0.995,4.680)` -> #6 `(1.105,6.880)` の global plan /
   local command 軌跡を地図に重ね、static occupied を踏む理由を可視化する。
2. waypoint edge 生成時、単に waypoint 点と測地最短路だけでなく、Nav2 inflation 相当の
   clearance と曲率/旋回余裕を持つ「実走行可能 corridor」を評価し、危険 edge は中間点を
   追加するか順序から外す。
3. local controller が静的地図上の occupied に入った時点で goal を即キャンセルし、近傍の
   safe pose へ戻す屋外専用 safety guard を検討する。ただし、まずは地図/edge 可視化で主因を確定する。

### 2026-06-21 改善サイクル23: local static layer 実験は既定未採用

低成績箇所:

- サイクル22の #6 で、robot pose は保存地図/static と global costmap 上では occupied
  (`static=100`, `global=100`) なのに、DWB が使う local costmap は free (`local=0`,
  `local_plan=52/52`) だった。局所 controller が保存地図の障害物を見ていない可能性を
  優先して検証した。

実装して試したこと:

- `config/nav2_params_webots_explore_outdoor.yaml` の `local_costmap.plugins` に
  `static_layer` を追加し、`map_topic: /map`, `map_subscribe_transient_local: True` で
  保存地図を local rolling costmap に入れる実験を行った。
- 1回目は `footprint_clearing_enabled: False`、2回目は Nav2 StaticLayer 公式仕様の
  「ロボット footprint 下の occupied を clear する」挙動を使うため
  `footprint_clearing_enabled: True` で評価した。
- どちらも live 評価後、既定の屋外設定は `local_costmap.plugins:
  ["obstacle_layer", "inflation_layer"]` に戻した。屋内設定は変更していない。

採用:

- 成果物として、2本の live report と costmap monitor report を残した。
  - `maps/village_square_trimmed_cycle23_nav_smoke_local_static.*`
  - `maps/village_square_trimmed_cycle23_pose_costmap_monitor_local_static.*`
  - `maps/village_square_trimmed_cycle23b_nav_smoke_local_static_footprint_clear.*`
  - `maps/village_square_trimmed_cycle23b_pose_costmap_monitor_local_static_footprint_clear.*`

未採用:

- local costmap に `static_layer` を常時入れること。#6 は越えられたが、cycle20 の既定
  `reached=16/53` より悪い `reached=14/53` で、既定採用する成績ではなかった。
- `footprint_clearing_enabled: True` 版。こちらも `reached=14/53` で改善せず、#14 以降の
  `Starting point in lethal space` を解決しなかった。

評価値:

- 静的検証:
  `python3` の YAML safe_load / 重複キー検査が成功。
  `colcon build --packages-select susumu_object_perception --symlink-install` 成功。
  `webots_outdoor_waypoint_nav.launch.py --show-args` と
  `webots_outdoor_mapping.launch.py --show-args` 成功。
- local static (`footprint_clearing_enabled: False`) live:
  `webots_outdoor_waypoint_nav.launch.py` を `rviz:=True`, `costmap_monitor:=True`,
  cycle19 promoted map/waypoints、`goal_timeout_sec:=60.0`,
  `mission_timeout_sec:=150.0` で起動。
  waypoint report は `mission_timeout`, `reached=14/53`, `missed=39`,
  `elapsed=150.045s`。
  monitor は samples `346`, events `52`、
  diagnosis counts は `ok=120`, `pose_static_lethal=220`,
  `pose_local_lethal_static_free=2`, `pose_global_lethal_static_free=1`,
  `plan_no_free_points_in_local_costmap=2`, `missing_tf=1`。
- 同 run の最初の event は #6、map `(-0.203, 5.376)`、
  `static=0`, `global=89`, `local=91`, `local_plan=51/47`。
  cycle22 の #6 で `static=100`, `global=100`, `local=0` だった状態とは変わり、
  local costmap が保存地図由来の高コストを見て停止/回復できた。
- しかし最初の `pose_static_lethal` は #14、map `(4.592, 1.488)`、
  `static=100`, `global=100`, `local=100`, `local_plan=0/0`, scan front `0.700m`。
  以降は static/global/local すべて lethal の pose から抜けられず、
  #15 以降も `Starting point in lethal space` が続いた。
- local static + footprint clearing live:
  waypoint report は `mission_timeout`, `reached=14/53`, `missed=39`,
  `elapsed=150.213s`。
  monitor は samples `410`, events `65`、
  diagnosis counts は `ok=122`, `pose_global_lethal_static_free=134`,
  `pose_global_lethal=115`, `pose_static_lethal=36`,
  `plan_no_free_points_in_local_costmap=2`, `local_missing=1`。
  最初の `pose_static_lethal` は #14、map `(4.437, 1.575)`,
  `static=100`, `global=100`, `local=100`, `local_plan=0/0`。

解釈:

- local static layer は「local controller が保存地図を見ていない」問題の切り分けには有効だった。
  #6 の早期破綻は避けられ、cycle22 の no-recovery `6/53` から `14/53` まで戻った。
- ただし既定 cycle20 の `16/53` より悪く、#14 で実軌跡そのものが保存地図の occupied /
  inflated 領域へ入り込む主因は残った。local costmap に static を入れるだけでは
  「危険 edge を選ばない」「実走行軌跡が corridor を踏み外さない」問題は解けない。
- 次は Nav2 パラメータの局所調整ではなく、waypoint edge / route graph / 実軌跡 corridor を
  地図上で評価する大きめの見直しを行う。

参考にした一次情報:

- Nav2 Static Layer Parameters:
  https://docs.nav2.org/configuration/packages/costmap-plugins/static.html
  (`StaticLayer` は map_server/SLAM の地図を costmap に置くこと、`footprint_clearing_enabled`
  の既定と意味、`map_subscribe_transient_local`, `map_topic` を確認)
- navigation2 `static_layer.cpp`:
  https://github.com/ros-navigation/navigation2/blob/main/nav2_costmap_2d/plugins/static_layer.cpp
  (rolling costmap で map frame と odom frame を変換して static cost を copy する実装、
  footprint clearing が costmap 更新時に footprint 領域を free にする実装を確認)
- Nav2 Controller Server:
  https://docs.nav2.org/configuration/packages/configuring-controller-server.html
  (controller server が local costmap を持つ構成を確認)
- Nav2 DWB BaseObstacle critic:
  https://docs.nav2.org/configuration/packages/trajectory_critics/base_obstacle.html
  (DWB が costmap 上の軌道コストを評価することを確認)

次の低成績箇所:

1. #13 -> #14 の edge を最優先で扱う。#14 失敗直前に `plan_no_free_points_in_local_costmap`
   が出ており、その後 robot pose が `(4.4〜4.6, 1.5〜1.7)` 付近の static/global lethal に入る。
2. `maps/village_square_trimmed_cycle23*_pose_costmap_monitor_*.csv` と waypoint YAML から、
   global plan / 実 pose / static map / local costmap を同じ画像へ重ねる。危険 edge を
   waypoint 生成時に除外または中間点へ分割できる形にする。
3. local static layer を再検討する場合は、単独ではなく「static lethal に入る前に goal を中断して
   safe pose へ戻す guard」または「危険 edge を出さない route graph」とセットで評価する。

## 改善サイクル24: waypoint edge clearance cost（未採用）

低成績箇所:

- cycle23 の次タスクだった #13 -> #14 edge / corridor を、個別 edge 禁止ではなく route graph 全体の
  clearance cost として扱った。Nav2 の costmap は障害物周辺を inflation し、controller は local costmap
  上で軌道を評価するため、保存地図上で「通行可能」なセル列でも inflation 近傍を長く通る edge は
  実走で破綻しやすい。

実装して試したこと:

- `scripts/generate_waypoints.py` に `--edge-clearance`、`--edge-clearance-weight`、
  `--edge-risk-report` を追加した。既定は `edge-clearance-weight=0.0` で従来互換。
- `--edge-clearance-weight > 0` のとき、距離場 `dist_cells` から `edge-clearance` 未満のセルに
  soft penalty を掛け、NN + 2-opt の距離行列を「測地距離」から「clearance weighted route cost」に
  置き換える。通行可否は変えず、狭い corridor を route graph の順序として選びにくくする。
- `generate_outdoor_waypoints.py` から上記 option を渡せるようにした。屋外 wrapper の既定値は変えていない。
- 比較用に、同一 map / spacing / clearance / max-waypoints / max-segment-length で baseline と
  weight `4.0`, `8.0`, `12.0` を生成した。

採用:

- edge risk の評価出力機能:
  - `maps/village_square_trimmed_cycle24_baseline_e075_risk.*`
  - `maps/village_square_trimmed_cycle24_edgecost_w4_risk.*`
  - `maps/village_square_trimmed_cycle24_edgecost_w8_risk.*`
  - `maps/village_square_trimmed_cycle24_edgecost_w12_risk.*`
- live 評価成果物:
  - `maps/village_square_trimmed_cycle24_nav_smoke_edgecost_w8.*`
  - `maps/village_square_trimmed_cycle24_pose_costmap_monitor_edgecost_w8.*`

未採用:

- `maps/village_square_trimmed_cycle24_edgecost_w8_waypoints.yaml` を採用 waypoint にすること。
  offline 指標は大きく改善したが、live では `reached=14/56` で cycle20 既定 `16/53` より悪化した。
- 屋外 wrapper の既定値を `--edge-clearance-weight` 付きへ変更すること。今回の live では実走行軌跡が
  global inflation / static occupied に入る問題を止められず、既定化する根拠がない。

評価値:

- offline baseline (`edge_clearance=0.75m`, weight `0.0`):
  total shortfall `12.893`, low-clearance edges `14/52`, worst edge `#12`
  `(3.305,9.63)->(3.805,3.93)`, min clearance `0.35m`, geodesic `7.904m`,
  shortfall `2.09628`。
- offline edge-cost w8:
  total shortfall `1.585`, low-clearance edges `12/55`, worst edge `#53`
  `(-5.545,-4.07)->(-7.145,-8.57)`, min clearance `0.391m`, geodesic `6.113m`,
  shortfall `0.83445`。
- 旧 #13 -> #14 相当:
  baseline edge `#13` は min clearance `0.35m`, shortfall `0.43335`。
  w8 では東側枝の連続 edge が min clearance `0.762m` 以上の区間を含む順序へ変わり、
  `#13 -> #14` の直結リスクは解消した。
- live smoke (`rviz:=True`, `mode:=fast`, cycle19 promoted map,
  `village_square_trimmed_cycle24_edgecost_w8_waypoints.yaml`,
  `goal_timeout_sec=60.0`, `mission_timeout_sec=150.0`,
  `costmap_monitor:=True`):
  `mission_timeout`, `reached=14/56`, `missed=42`。
- costmap monitor:
  samples `357`, events `41`。
  diagnosis counts は `ok=201`, `pose_static_lethal=69`,
  `pose_global_lethal_static_free=45`, `plan_not_in_local_costmap=31`,
  `pose_global_lethal=6`, `plan_no_free_points_in_local_costmap=3`,
  `local_missing=2`。
  最終的には map `(4.01, 2.368)` で `static=100`, `global=100`,
  `local=0` の `pose_static_lethal` になり、そこから抜けられなかった。

解釈:

- route graph の大局的な順序は改善できた。offline では low-clearance edge の総量が
  `12.893 -> 1.585` まで下がり、旧 #13 -> #14 直結も避けられた。
- それでも live では、DWB が辿った実軌跡が waypoint 間の理想経路より膨らみ、
  global costmap の inflation や static occupied に入った。したがって次の主因は
  waypoint の順序ではなく、**global plan / local controller の実軌跡 corridor が
  saved-map の安全帯内に収まらないこと**。
- `edge-clearance-weight` は診断・候補生成として残すが、現時点の屋外本線には採用しない。

参考にした一次情報:

- Nav2 Waypoint Follower:
  https://docs.nav2.org/configuration/packages/configuring-waypoint-follower.html
  (waypoint follower が `NavigateToPose` action server で ordered waypoints を順に実行し、
  到達不能 waypoint の扱いに `stop_on_failure` があることを確認)
- Nav2 Costmap 2D:
  https://docs.nav2.org/configuration/packages/configuring-costmaps.html
  (planner / controller server が costmap を collision / high cost area に使うこと、
  rolling window local costmap と footprint collision checking の位置づけを確認)
- Nav2 Inflation Layer:
  https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html
  (lethal obstacle 周辺に exponential decay の cost を作り、`inflation_radius` /
  `cost_scaling_factor` が clearance と直結することを確認)
- Nav2 Navigate Through Poses:
  https://docs.nav2.org/behavior_trees/trees/nav_through_poses_recovery.html
  (中間 pose を hard constraint として扱う設計があることを確認。今回は既存
  `NavigateToPose` 連続実行のまま評価)

次の低成績箇所:

1. waypoint 生成側だけではなく、global plan と local controller の実軌跡を同じ corridor として評価する。
   `nav2_pose_costmap_monitor_node.py` の PNG/CSV に、robot 走行軌跡と waypoint edge ID を残し、
   「計画は安全だが実軌跡が膨らむ」のか「計画自体が inflation に近い」のかを分ける。
2. saved-map 上で安全な centerline / skeleton を作り、waypoint 間を straight edge ではなく
   centerline-following の中間 goal 列に変換する。単なる長区間分割ではなく、controller が外側へ
   膨らみにくい曲率・間隔で goal を置く。
3. `pose_global_lethal_static_free` が多いので、global inflation 上の実 pose を live で検出したら
   次 waypoint へ進む前に safe pose へ戻す guard を屋外専用に検討する。

## 改善サイクル25: plan / actual corridor trace monitor（採用）

低成績箇所:

- cycle24 で `edge-clearance-weight` は offline shortfall を `12.893 -> 1.585` まで改善したが、
  live は `reached=14/56` へ悪化した。主因を waypoint 順序だけで扱うのではなく、
  Nav2 の global plan と local controller が実際に通した robot trajectory の corridor 差として測る。

実装して試したこと:

- `scripts/nav2_pose_costmap_monitor_node.py` に waypoint YAML 読み込みを追加し、
  各サンプルに `edge_from` / `edge_to` / target pose を記録するようにした。
- 最新 global plan から robot pose までの最短距離 `path_error_m` を記録し、
  `path_error_warn_m` 以上なら `path_tracking_error` として診断に入れるようにした。
- JSON summary に trajectory 集計を追加した。全体の `max_path_error_m` / `mean_path_error_m` と、
  waypoint ごとの diagnosis counts、`pose_static_lethal_samples`、
  `pose_global_lethal_static_free_samples` を出す。
- PNG report に robot trace（magenta）と event 点（red x）を static / global / local costmap 上へ重ねるようにした。
- `launch/webots_outdoor_waypoint_nav.launch.py` の costmap monitor 起動時に `waypoints_file` を渡すようにした。

採用:

- plan / actual corridor trace monitor の拡張。これは navigation そのものを改善する値ではなく、
  次の大きな改善で「計画が悪いのか、controller 実軌跡が膨らむのか、復帰不能 pose に落ちるのか」を
  waypoint/edge 単位で切り分けるための評価基盤として採用する。
- live 評価成果物:
  - `maps/village_square_trimmed_cycle25_nav_smoke_trace_baseline.*`
  - `maps/village_square_trimmed_cycle25_pose_costmap_monitor_trace_baseline.*`

未採用:

- controller を RPP へ切り替えること。Nav2 公式 docs では Regulated Pure Pursuit は path を追従し、
  curvature / obstacle proximity / collision check で速度を調整できるが、過去 cycle の GPS/RPP 実験は
  採用値に届いておらず、今回も controller 変更より先に plan / actual corridor の定量化を優先した。
- Nav2 パラメータ変更。今回の目的は既定構成の失敗を測ることで、`config/nav2_params*.yaml` は変更していない。

評価値:

- live smoke (`rviz:=True`, `mode:=fast`, cycle19 promoted GLIM 2D map,
  `village_square_trimmed_cycle19_promoted_glim2d_waypoints.yaml`,
  `goal_timeout_sec=60.0`, `mission_timeout_sec=150.0`,
  `costmap_monitor:=True`):
  `mission_timeout`, `reached=17/53`, `missed=36`、elapsed `150.118s`。
- costmap monitor:
  samples `461`, events `65`。
  diagnosis counts は `ok=190`, `pose_static_lethal=170`,
  `pose_global_lethal_static_free=54`, `pose_global_lethal=29`,
  `plan_not_in_local_costmap=13`, `plan_no_free_points_in_local_costmap=3`,
  `path_tracking_error=1`, `local_missing=1`。
- trajectory summary:
  valid pose samples `460`, `max_path_error_m=8.186`, `mean_path_error_m=2.248`。
  waypoint `18` は `max_path_error_m=8.186` で `path_tracking_error=1`、
  `pose_global_lethal=6`。waypoint `21` は `pose_global_lethal_static_free=15`,
  `pose_static_lethal=8`。waypoint `22` は `pose_static_lethal=156` で、
  mission timeout 後に static lethal cell 上から抜けられない状態を示した。
- PNG 目視:
  robot trace が static/global costmap の障害物近傍へ入り込み、赤い event 点が waypoint 18 以降へ集中した。
  失敗は waypoint 単発ではなく、東側 corridor で drift / recovery 後に復帰不能 pose へ落ちる系列として扱う。

解釈:

- cycle24 の route graph 改善だけでは、controller が実際に通る corridor を安全帯へ閉じ込められない。
- まず waypoint `18` 付近で path tracking error と global lethal が発生し、その後 waypoint `21` / `22`
  で static/global lethal に入って復帰不能化している。次は waypoint 順序ではなく、
  **centerline-following の中間 goal 列**または**static/global lethal に入る前に中断して safe pose へ戻す屋外専用 guard**
  を評価する。

参考にした一次情報:

- Nav2 DWB Controller:
  https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html
  (DWB が default controller で、trajectory critics によって候補軌道を評価することを確認)
- Nav2 Regulated Pure Pursuit Controller:
  https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html
  (path following、curvature / proximity regulation、active collision checking の位置づけを確認)
- Nav2 Smac Planner:
  https://docs.nav2.org/configuration/packages/configuring-smac-planner.html
  (2D A* planner が cost-aware planning を行うことを確認)
- Nav2 upstream issue #5037:
  https://github.com/ros-navigation/navigation2/issues/5037
  (robot pose と path の距離、controller feedback / tracking error を測る需要と議論を確認)
- Regulated Pure Pursuit README:
  https://github.com/ros-navigation/navigation2/blob/main/nav2_regulated_pure_pursuit_controller/README.md
  (global path pruning、lookahead、collision checking arc の実装意図を確認)

次の低成績箇所:

1. waypoint `18` 周辺の corridor を最優先に、saved-map の centerline / skeleton から中間 goal 列を生成し、
   straight edge ではなく controller が膨らみにくい曲率・間隔で `NavigateToPose` を送る。
2. waypoint `21` / `22` のように `pose_global_lethal_static_free` や `pose_static_lethal` が続く場合、
   次 waypoint へ進む前に直近の safe pose へ戻す屋外専用 guard を `waypoint_nav_node.py` 側で検討する。
3. controller 変更（RPP など）は、上記 trace monitor の `path_error_m` と lethal samples を比較指標にして、
   屋外専用 params に閉じて評価する。

## 改善サイクル26: centerline-following route expansion（waypoint候補は未採用）

低成績箇所:

- cycle25 の次タスクだった waypoint `18` 周辺の corridor を、個別 waypoint ではなく
  **保存地図上のrouteを中間goal列へ展開する全体設計**として扱った。Nav2 は中間 pose を通る
  task を扱えるが、現状ノードは `NavigateToPose` を逐次送る構成なので、まず既存 executor の入力
  YAML を centerline-following に寄せる。

実装して試したこと:

- `scripts/expand_waypoint_route.py` を追加した。`generate_outdoor_waypoints.py` が作った waypoint YAML を読み、
  保存地図の route passable 連結成分上で各 edge の最短経路を計算し、`--max-segment-length` 以下の
  中間goal列へ展開する後処理ツール。
- 元 waypoint 順序は維持する。長い pose-to-pose jump だけを地図上のgeodesic pathに沿って分割し、
  report CSV/JSON/Markdown と overlay PNG を残す。
- 比較候補:
  - `maps/village_square_trimmed_cycle26_centerline_follow_2m_waypoints.yaml`
  - `maps/village_square_trimmed_cycle26_centerline_follow_15m_waypoints.yaml`
- 2.0m版を RViz + costmap monitor 付きで live 評価した。

採用:

- `scripts/expand_waypoint_route.py` を診断・候補生成ツールとして採用する。既存 generator の巡回順を壊さず、
  「sparse route goal が原因で path error が膨らむのか」を切り分けられる。
- live 評価成果物:
  - `maps/village_square_trimmed_cycle26_nav_smoke_centerline_2m.*`
  - `maps/village_square_trimmed_cycle26_pose_costmap_monitor_centerline_2m.*`

未採用:

- `maps/village_square_trimmed_cycle26_centerline_follow_2m_waypoints.yaml` を屋外本線 waypoint にすること。
  path tracking error は改善したが、危険 corridor へ忠実に入ってしまい、global/local costmap の lethal 近傍で
  復帰不能になった。
- 1.5m版を live 評価へ進めること。2.0m版で「細分化するほど安全」ではないことが分かったため、
  さらに密にしても同じ危険 corridor を強く辿る可能性が高い。

評価値:

- offline expansion:
  - 2.0m版: input `53` -> output `96`, inserted `43`, `max_output_segment_m=2.597`,
    `mean_output_segment_m=1.444`。
  - 1.5m版: input `53` -> output `118`, inserted `65`, `max_output_segment_m=2.236`,
    `mean_output_segment_m=1.172`。
- live smoke (`rviz:=True`, `mode:=fast`, cycle19 promoted GLIM 2D map,
  `village_square_trimmed_cycle26_centerline_follow_2m_waypoints.yaml`,
  `goal_timeout_sec=60.0`, `mission_timeout_sec=240.0`,
  `costmap_monitor:=True`):
  `mission_timeout`, `reached=22/96`, `missed=74`, elapsed `240.159s`。
- costmap monitor:
  samples `535`, events `54`。
  diagnosis counts は `ok=320`, `pose_global_lethal_static_free=189`,
  `pose_static_lethal=16`, `plan_no_free_points_in_local_costmap=8`,
  `pose_local_lethal_static_free=1`, `local_missing=1`。
- trajectory summary:
  `max_path_error_m=1.23`, `mean_path_error_m=0.127`。cycle25 の
  `max_path_error_m=8.186`, `mean_path_error_m=2.248` からは大きく改善した。
  一方で waypoint `22` は `pose_static_lethal=16`,
  `pose_global_lethal_static_free=10`、waypoint `36` は
  `pose_global_lethal_static_free=58`。path をよく追えていても、path 自体が危険な corridor に入っている。
- PNG 目視:
  robot trace は global plan に近いが、赤い event 点が北側 corridor の global costmap high-cost 領域に集中した。

解釈:

- 中間goal列は「plan と実軌跡の乖離」を下げるには有効だった。ただし今回の保存地図では、
  saved-map geodesic path が Nav2 inflation / local obstacle に対して安全とは限らない。
- 次は route を細かくするのではなく、**実行中に high-cost / lethal に入る前に中断し、直近の safe pose へ戻す
  outdoor-only guard** が必要。cycle25/26 の両方で、miss 後に次 waypoint へ進む設計が危険姿勢を悪化させている。

参考にした一次情報:

- Nav2 overview:
  https://docs.nav2.org/
  (Nav2 が intermediate poses や behavior tree による複合 task を扱えることを確認)
- Nav2 Navigate Through Poses:
  https://docs.nav2.org/behavior_trees/trees/nav_through_poses_recovery.html
  (中間 pose を hard constraint として通る task と、`RemovePassedGoals` の考え方を確認)
- Nav2 Waypoint Follower:
  https://docs.nav2.org/configuration/packages/configuring-waypoint-follower.html
  (`NavigateToPose` action server で ordered waypoints を順に実行し、失敗時の継続/停止を扱うことを確認)
- Nav2 waypoint following design issue #803:
  https://github.com/ros-navigation/navigation2/issues/803
  (外部 waypoint navigator が `NavigateToPose` をpreemptしながら追従する設計選択肢を確認)
- Nav2 Regulated Pure Pursuit:
  https://docs.nav2.org/configuration/packages/configuring-regulated-pp.html
  (path following性能とcollision checkingの位置づけを確認。今回のlive悪化はcontroller切替前にroute安全性を扱うべきと判断)

次の低成績箇所:

1. `waypoint_nav_node.py` に outdoor-only safe-pose guard を追加する。現在poseが
   `pose_global_lethal_static_free` / `pose_static_lethal` 相当になった、または goal timeout した場合、
   直近の safe pose へ戻る recovery goal を挟み、危険姿勢のまま次 waypoint へ進まない。
2. safe pose は world 真値ではなく、巡回中に monitor できる map/costmap から得る。
   まずは waypoint 到達直後の robot pose を safe pose として保存し、miss 後にそこへ戻る。
3. その次に、offline route 生成側で `pose_global_lethal_static_free` が多い corridor を
   `expand_waypoint_route.py` の report/monitor結果からブラックリスト化し、候補routeから除外する。

## 改善サイクル27: safe-pose guard 評価（既定未採用）

低成績箇所:

- cycle26 の次タスクだった「危険姿勢のまま次 waypoint へ進む」問題を対象にした。
  saved-map geodesic path を細かくしても、controller が high-cost / lethal pose へ入ると
  Smac Planner が `Starting point in lethal space` となり、その後の waypoint も崩れる。

実装:

- `waypoint_nav_node.py` に optional な `safe_pose_guard` を追加した。既定は `False`。
- guard は `/amcl_pose` と `/global_costmap/costmap` だけを見る。world 由来の正解データは使わない。
- 現在 pose の global costmap 値が閾値以上で一定時間続いた場合、または goal failure / timeout 時に、
  最後に safe costmap 値だった pose へ `NavigateToPose` recovery goal を送る。
- recovery 成否は waypoint report の `safe_pose_recoveries` に残す。
- `webots_waypoint_nav.launch.py` / `webots_outdoor_waypoint_nav.launch.py` に
  `safe_pose_guard`, `safe_pose_cost_threshold`, `safe_pose_safe_threshold`,
  `safe_pose_hold_sec`, `safe_pose_recovery_timeout_sec` を追加した。既定OFFなので屋内や通常巡回には影響しない。

採用:

- `safe_pose_guard` の実装とレポート項目は、診断用・次の方式比較用として残す。
  「lethal pose に入ってから Nav2 goal で戻れるか」を定量的に否定できるようになったため。

未採用:

- `safe_pose_guard:=True` を屋外本線の既定にすること。
- 閾値を下げるだけで予防できるという方針。`safe_pose_cost_threshold=50` でも検出時には
  すでに planner の start pose が lethal 扱いになっており、復帰 goal も計画不能だった。

評価条件:

```bash
ros2 launch susumu_object_perception webots_outdoor_waypoint_nav.launch.py \
  world:=village_square_trimmed.wbt \
  map_file:=maps/village_square_trimmed_cycle19_promoted_glim2d.yaml \
  waypoints:=maps/village_square_trimmed_cycle26_centerline_follow_2m_waypoints.yaml \
  mode:=fast rviz:=True loop:=False \
  perception:=False omni_perception:=False image_recognition:=False colored_slam:=False slam:=False \
  goal_timeout_sec:=60.0 mission_timeout_sec:=240.0 \
  safe_pose_guard:=True \
  costmap_monitor:=True
```

評価値:

| 条件 | waypoint結果 | safe-pose recovery | monitor summary |
|---|---|---|---|
| `cost_threshold=80`, `safe_threshold=40`, `hold=1.0s` | `safe_pose_recovery_failed`, `reached=11/96`, `missed=[9,12]`, elapsed `34.395s` | #9 は成功、#12 は timeout | samples `220`, events `36`, `ok=56`, `pose_global_lethal_static_free=9`, `pose_static_lethal=152`, `max_path_error_m=3.035`, `mean_path_error_m=2.243` |
| `cost_threshold=50`, `safe_threshold=10`, `hold=0.2s` | `safe_pose_recovery_failed`, `reached=9/96`, `missed=[9]`, elapsed `29.274s` | #9 が timeout | samples `190`, events `30`, `ok=53`, `pose_global_lethal_static_free=1`, `pose_static_lethal=133`, `max_path_error_m=0.718`, `mean_path_error_m=0.529` |

成果物:

- `maps/village_square_trimmed_cycle27_nav_smoke_safe_guard_2m.*`
- `maps/village_square_trimmed_cycle27_pose_costmap_monitor_safe_guard_2m.*`
- `maps/village_square_trimmed_cycle27_nav_smoke_safe_guard_t50_2m.*`
- `maps/village_square_trimmed_cycle27_pose_costmap_monitor_safe_guard_t50_2m.*`

解釈:

- high-cost / lethal の発生検出はできたが、検出後に `NavigateToPose` で safe pose へ戻す方式は遅い。
  Smac Planner は現在 pose が lethal space にあると復帰先が安全でも path を作れず、recovery goal 自体が詰まる。
- `safe_pose_guard` は「次 waypoint へ進んで悪化する」問題を抑えるより先に mission を止める方向へ働いた。
  cycle26 の `reached=22/96` より悪化したため、既定採用しない。
- 次は Nav2 planner に再計画させる復帰ではなく、(a) offline/online で危険 corridor を避ける、
  (b) lethal に入る前に中断する、(c) lethal 入り直後は短時間の直接制御 escape で footprint を free へ戻す、
  のどれかを屋外専用で評価する。

参考にした一次情報:

- Nav2 Costmap 2D:
  https://docs.nav2.org/configuration/packages/configuring-costmaps.html
  (planner/controller が collision や high-cost area を costmap で判定することを確認)
- Nav2 NavigateToPose recovery tree:
  https://docs.nav2.org/behavior_trees/trees/nav_to_pose_recovery.html
  (通常 recovery は clear/spin/wait/backup などを BT で実行することを確認)
- Nav2 NavigateToPose consistent replanning tree:
  https://docs.nav2.org/behavior_trees/trees/nav_to_pose_with_consistent_replanning_and_if_path_becomes_invalid.html
  (path invalid 時の再計画/recovery の位置づけを確認)
- Nav2 Simple Commander:
  https://docs.nav2.org/commander_api/index.html
  (外部 executor が feedback を見ながら cancel / goal 再投入できることを確認)

次の低成績箇所:

1. `pose_static_lethal` が出た後に Nav2 goal で戻すのではなく、発生前の edge / corridor を
   monitor結果からブラックリスト化して route 生成側で避ける。
2. どうしても lethal pose に入った場合は、屋外専用の短時間 direct escape
   （低速後退/旋回、cmd_vel、局所costのみ監視）を検証する。Nav2 planner を通さないことが要点。
3. `safe_pose_guard` を使う場合も既定OFFの診断用に留め、採用評価では reached 数と
   `pose_static_lethal` サンプルを cycle26 baseline と比較する。

### 2026-06-21 方針転換サイクルの結果

採用:

- `village_square_trimmed.wbt`: village_center の街区風。小建物・街灯・ベンチ・植栽・フェンスを
  34m 四方に配置し、四辺を通行止めバリアで閉じた。
- `village_park_trimmed.wbt`: village_center の公園風。フェンス・Pergolas・ベンチ・植栽・低い
  planters を配置し、四辺を通行止めバリアで閉じた。
- `generate_outdoor_waypoints.py`: 屋外専用 waypoint 生成 wrapper。屋内 `generate_waypoints.py`
  の実装は共有するが、既定値は屋外用に `spacing=4.0m`, `clearance=0.75m`,
  `connect_clearance=0.35m`, `max_waypoints=40`, `limit_radius=14.0m` として分離した。
  `limit_radius` は `webots_outdoor_mapping.launch.py` の `explore_radius` と合わせ、LiDAR が
  通行止めの向こう側を free として観測したセルを巡回点にしないための制限。
- `webots_outdoor_waypoint_nav.launch.py`: 保存地図 + AMCL + Nav2 で屋外 waypoint を巡回する
  屋外専用 launch。屋内巡回 launch の既定値は変えない。
- `webots_outdoor_mapping.launch.py` / `webots_outdoor_waypoint_nav.launch.py` の屋外既定
  `goal_timeout_sec=120.0`。trimmed 屋外では 10m 級の goal が自然に出るため、屋内寄りの
  60s では到達前にブラックリスト化されることがあった。

未採用:

- 旧 `outdoor.wbt` / `city_robot.wbt` を屋外マッピング本線に戻すこと。特徴が少なく、今回の
  「MID360 が常時いくつも特徴を捉える」前提に合わない。
- 旧 GPS/IMU sparse outdoor 構成を本線にすること。過去実験として残すが、今回の屋外タスクの
  合格導線ではない。
- `generate_waypoints.py` で `max_waypoints` 超過時に全点を出力する旧挙動。`village_square`
  snapshot では候補 474 点になり、全点の測地距離計算が長時間化したため未採用。指定上限内に
  farthest-point sampling で間引く挙動へ変更した。

評価値:

- 静的 WBT 検査: 両 world とも `{` / `}` 数一致、`TurtleBot3Burger` と `road_closed_*` 4辺を確認。
- 軽量ライブ smoke（`mode:=fast`, `nav:=False`, `slam:=False`, 認識OFF）:
  - `village_square_trimmed.wbt`: Webots 起動、`TurtleBot3Burger` controller 接続、
    `/clock`、`/lidar/points/point_cloud`、`/scan` を確認。点群 width `20160`、`/scan` は有限 range を返す。
  - `village_park_trimmed.wbt`: Webots 起動、`TurtleBot3Burger` controller 接続、
    `/clock`、`/lidar/points/point_cloud`、`/scan` を確認。点群 width `20160`、`/scan` は有限 range を返す。
- wrapper smoke: `generate_outdoor_waypoints.py --map maps/outdoor.yaml --out /tmp/outdoor_wrapper_smoke_waypoints.yaml`
  は 12 waypoint を生成し、既存 `generate_waypoints.py` 呼び出しまで確認済み。
- `village_square_trimmed.wbt` realtime mapping snapshot（旧 `goal_timeout_sec=60.0` 条件）:
  `map_saver_cli` で `maps/village_square_trimmed_cycle6_snapshot.yaml/.pgm` を保存。地図は
  `904x1028`、`0.05m/pix`、約 `45.2 x 51.4m`。探索ログでは `60s` timeout が複数回出た一方、
  短い goal は到達でき、SLAM/map_server/Nav2 の導線は動作。
- `generate_outdoor_waypoints.py` 実地評価:
  - `limit_radius` 無しでは候補 `474` 点、80 点へ間引いても waypoint 範囲が
    `x=-19.175..25.975`, `y=-26.375..24.975` まで広がり、区画外 free を拾ったため未採用。
  - `limit_radius=14.0` 採用後は候補 `137` 点から 80 点へ間引き。waypoint 範囲
    `x=-13.925..13.825`, `y=-12.425..11.975`、最大半径 `13.998m`、半径外 `0/80`。
    カバー領域 `510m²`、測地経路長 `241.4m`、最大測地ジャンプ `12.4m`。
- 保存地図 + AMCL + Nav2 waypoint live:
  `webots_outdoor_waypoint_nav.launch.py world:=village_square_trimmed.wbt
  map_file:=village_square_trimmed_cycle6_snapshot.yaml
  waypoints:=village_square_trimmed_cycle6_snapshot_waypoints.yaml mode:=realtime rviz:=False
  loop:=False goal_timeout_sec:=120.0` で map_server/AMCL/Nav2 が active、80 waypoint 読み込み、
  `NavigateToPose` で #0〜#3 は連続到達、#4 走行中に検証終了。

次の低成績箇所:

1. `village_square_trimmed.wbt` を修正後の `goal_timeout_sec=120.0` 条件で完走させ、自動保存地図
   `maps/village_square_trimmed.yaml` を作る。
2. `village_square_trimmed_cycle6_snapshot_waypoints.yaml` の 80 点フル巡回を完走評価し、
   reached/missed と最大スキップ箇所を残す。
3. 同じ mapping → waypoint → saved-map Nav2 の流れを `village_park_trimmed.wbt` に展開し、
   2 world の差を比較する。

### 2026-06-21 改善サイクル7: saved-map 巡回評価の永続化

方針:

- Nav2 Waypoint Follower は ordered waypoints を `NavigateToPose` で順番に実行し、失敗時に継続可否を
  `stop_on_failure` で扱う。既存 `waypoint_nav_node.py` も同じ考え方で各点へ `NavigateToPose` を
  送っているため、経路実行方式は維持する。
- 低成績箇所は「長い屋外 saved-map 巡回がログ依存で、完走/途中終了の reached/missed が成果物として
  残らない」こと。長時間巡回を毎回最後まで待つのではなく、bounded live でも report を残せるようにする。

採用:

- `waypoint_nav_node.py` に `report_prefix` を追加。指定時は JSON/CSV/Markdown を waypoint ごとに更新する。
- `mission_timeout_sec` を追加。wall-clock で巡回評価全体を打ち切り、未実行 waypoint は
  `mission_timeout` として report に残す。sim time 初期ジャンプや長時間試験の停止忘れを避けるため、
  判定は wall-clock。
- `webots_waypoint_nav.launch.py` と `webots_outdoor_waypoint_nav.launch.py` に
  `report_prefix` / `mission_timeout_sec` 引数を追加。既定は report 無効、mission timeout 無効なので
  屋内既定動作は変えない。

未採用:

- `FollowWaypoints` へ丸ごと移行すること。Nav2 公式には適した標準機能だが、現状のノードは
  goal ごとの timeout / retry / partial report を細かく制御でき、屋外評価の失敗分析に必要な情報を
  残しやすいため、今回は維持した。
- timeout 後の古い action result callback を report に反映する挙動。初回 live で
  `mission_timeout` report が `in_progress` に上書きされたため未採用とし、finish 時に token を進めて
  以後の callback を無視するよう修正した。

評価値:

- 静的検証: `python3 -m py_compile susumu_object_perception/waypoint_nav_node.py` 成功。
- build: `colcon build --packages-select susumu_object_perception --symlink-install` 成功。
- launch parse: `webots_waypoint_nav.launch.py` / `webots_outdoor_waypoint_nav.launch.py` の
  `--show-args` で `report_prefix` と `mission_timeout_sec` を確認。
- live 評価:
  `webots_outdoor_waypoint_nav.launch.py world:=village_square_trimmed.wbt
  map_file:=village_square_trimmed_cycle6_snapshot.yaml
  waypoints:=village_square_trimmed_cycle6_snapshot_waypoints.yaml mode:=realtime rviz:=False
  loop:=False goal_timeout_sec:=120.0 mission_timeout_sec:=60.0
  report_prefix:=maps/village_square_trimmed_cycle7_waypoint_nav`。
  結果は `reason=mission_timeout`, reached `4/80`, missed `[4..79]`, elapsed `60.455s`。
  #0〜#3 は `succeeded`、#4 は走行中に `mission_timeout`。成果物:
  `maps/village_square_trimmed_cycle7_waypoint_nav.{json,csv,md}`。

参考にした一次情報:

- Nav2 Waypoint Follower: https://docs.nav2.org/configuration/packages/configuring-waypoint-follower.html
- Nav2 NavigateToPose: https://docs.nav2.org/configuration/packages/bt-plugins/actions/NavigateToPose.html
- ROS 2 Actions: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

次の低成績箇所:

1. `village_square_trimmed.wbt` を修正後の `goal_timeout_sec=120.0` 条件で完走させ、
   自動保存地図 `maps/village_square_trimmed.yaml` を作る。
2. `village_square_trimmed_cycle6_snapshot_waypoints.yaml` の 80 点フル巡回を、今回追加した
   `report_prefix` 付きで完走または長時間 bounded 評価し、失敗箇所を report から特定する。
3. `village_park_trimmed.wbt` に同じ report 付き mapping → waypoint → saved-map Nav2 の流れを展開する。

### 2026-06-21 改善サイクル8: 屋外巡回の粗粒度化と速度上限整合

方針:

- 低成績箇所は cycle7 の `60s` bounded 評価が `reached=4/80` に留まったこと。ただしこれは
  局所的な失敗 waypoint ではなく、屋外で 2.2m 間隔の密な点を全て `NavigateToPose` にして
  停止判定する設計と、屋外 DWB の速度上限が屋内標準より低いことの組み合わせとして扱う。
- Nav2 DWB の公式 kinematic parameters では `max_vel_x` が X 方向速度上限、`max_speed_xy` が
  並進速度上限。SimpleProgressChecker は `required_movement_radius` と
  `movement_time_allowance` で進捗失敗を判定する。今回は進捗判定を緩めるのではなく、屋外だけ
  巡回スループットを上げる。
- 屋外 waypoint は「密な面カバー点」ではなく「道路・広場スケールの route goal」として扱い、
  Nav2 が点間を連続走行する設計へ寄せる。

採用:

- `config/nav2_params_webots_explore_outdoor.yaml` の DWB `max_vel_x` / `max_speed_xy` を
  `0.22` から `0.26m/s` へ変更。屋内標準 `config/nav2_params.yaml` と同じ TurtleBot3 waffle
  上限に揃え、屋外専用 params のみに閉じた。
- `scripts/generate_outdoor_waypoints.py` の屋外既定を `spacing=4.0m`,
  `max_waypoints=40` に変更。屋内共通生成器の既定値は変えない。
- 新成果物 `maps/village_square_trimmed_cycle8_waypoints.yaml/.png` を生成。対象半径は従来通り
  `14.0m`、カバー領域は `510m²`。

未採用:

- 40 waypoint 化だけで十分とみなすこと。60秒内の到達数は cycle7 と同じ `4` 点で、問題は
  waypoint 数だけではなく長距離セグメントの回頭・再計画・停止時間にもある。
- 進捗判定をさらに緩めること。今回は `mission_timeout` での bounded 評価であり、
  `Failed to make progress` では止まっていないため、SimpleProgressChecker の緩和は原因に対して
  直接効く変更ではない。
- 屋内 `config/nav2_params.yaml` や屋内 waypoint 生成を変更すること。屋外試行錯誤は屋外専用
  params / wrapper に限定する。

評価値:

- waypoint 生成:
  - cycle7 既定: `80` 点、カバー領域 `510m²`、測地経路長 `241.4m`、最大測地ジャンプ `12.4m`。
  - cycle8 既定: `40` 点、カバー領域 `510m²`、測地経路長 `164.4m`、最大測地ジャンプ `11.8m`。
  - 参考 variant: `spacing=3.0/max=60` は `216.1m`、`spacing=3.5/max=48` は `185.7m`。
- 静的検証: `python3 -c "import yaml; yaml.safe_load(open('config/nav2_params_webots_explore_outdoor.yaml'))"` 成功。
- 静的検証: `python3 -m py_compile scripts/generate_outdoor_waypoints.py susumu_object_perception/waypoint_nav_node.py` 成功。
- build: `colcon build --packages-select susumu_object_perception --symlink-install` 成功。
- launch parse: `ros2 launch susumu_object_perception webots_outdoor_waypoint_nav.launch.py --show-args` 成功。
- live 評価:
  `webots_outdoor_waypoint_nav.launch.py world:=village_square_trimmed.wbt
  map_file:=village_square_trimmed_cycle6_snapshot.yaml
  waypoints:=village_square_trimmed_cycle8_waypoints.yaml mode:=realtime rviz:=False
  loop:=False goal_timeout_sec:=120.0 mission_timeout_sec:=60.0
  report_prefix:=maps/village_square_trimmed_cycle8_waypoint_nav`。
  結果は `reason=mission_timeout`, reached `4/40`, missed `[4..39]`, elapsed `60.471s`。
  #0〜#3 の duration は `7.804s`, `11.402s`, `13.503s`, `23.353s`。到達点間の直線距離は
  cycle7 の `6.6m/60s` から cycle8 の `10.2m/60s` へ増えたが、到達点数は増えなかった。
  成果物: `maps/village_square_trimmed_cycle8_waypoint_nav.{json,csv,md}`。
- 終了時に Webots ROS adapter / `bumper_plugin` の既知の shutdown ノイズが出たが、report 保存後で
  残プロセスは無し。

参考にした一次情報:

- Nav2 DWB kinematic parameters: https://docs.nav2.org/configuration/packages/dwb-params/kinematic.html
- Nav2 SimpleProgressChecker: https://docs.nav2.org/configuration/packages/nav2_controller-plugins/simple_progress_checker.html
- Nav2 Tuning Guide: https://docs.nav2.org/tuning/index.html

次の低成績箇所:

1. `village_square_trimmed` の長距離セグメントで `/cmd_vel` 実測、回頭時間、再計画周期、
   recovery 有無を report に足し、到達数ではなく「走行距離/停止時間/平均速度」で比較する。
2. `controller_frequency`、DWB critic、または Regulated Pure Pursuit など controller plugin の
   屋外専用比較を行う。変更は `config/nav2_params_webots_explore_outdoor.yaml` か派生ファイルに限定する。
3. `village_park_trimmed.wbt` でも同じ `40` waypoint + `60s` bounded 評価を取り、world 固有か
   屋外巡回設定全体の問題かを分ける。

### 2026-06-21 方針修正: world 由来地図は正解データ

ユーザー指摘により、方針を修正した。`village_square_trimmed_world` のような world 由来の地図は
正解データであり、Nav2 運用地図や waypoint 生成入力にしてはいけない。屋外マッピング本線は
`slam_toolbox` がセンサデータだけで作った保存地図を使い、正解データは作成後の評価にだけ使う。

採用:

- `scripts/generate_webots_ground_truth_map.py`: trimmed Webots world から評価用の
  `maps/<world>_gt.yaml/.pgm/.png` を作る。これは正解データであり、`map_file` には渡さない。
- `scripts/check_map_vs_world.py`: SLAM 保存地図を WBT の真値構造に重ね、外周・床・障害物との対応を
  評価する。SLAM 地図生成後の合否判定に使う。
- `launch/webots_outdoor_waypoint_nav.launch.py` の既定を
  `map_file:=village_square_trimmed.yaml`,
  `waypoints:=village_square_trimmed_waypoints.yaml` に戻した。
- `generate_outdoor_waypoints.py` の既定 `limit_radius` は `14.0m` に戻した。SLAM 地図が区画外を
  free として拾った場合に、巡回点が通行止め外へ漏れるのを避けるため。
- 両 trimmed world のロボット初期位置を `(0, 0)` にした。旧 `village_square_trimmed` の
  `(0, -12)` は南端寄りで、初期周辺の特徴量が少なく、地図原点も偏った。

未採用:

- world 由来地図を「運用 Nav2 map」として使うこと。
- world 由来地図から waypoint を生成すること。
- `maps/village_square_trimmed_cycle9_world_nav.*` を屋外巡回評価として扱うこと。GT 地図を使った
  無効な試行なので削除し、評価値には入れない。
- `maps/village_square_trimmed_cycle6_snapshot.pgm` を採用地図にすること。視覚確認では大きな
  raytrace disk と疎な黒線だけで、閉じた街区地図になっていなかった。

評価値:

- `village_square_trimmed_cycle6_snapshot.pgm`: `904x1028`, occupied `0.52%`,
  unknown `36.02%`, free `63.46%`。壁率が低すぎ、区画外 free も多いため不合格。
- `(0,0)` 初期位置で正解データを使わずに `slam_toolbox` を再実行し、途中地図を
  `maps/village_square_trimmed_slam_partial_20260621.yaml/.pgm` に保存した。探索は
  3 frontier 到達後、4つ目 `(-9.7, -4.7)` 付近で `Starting point in lethal space` が多発し、
  残り `307` frontier candidates が到達不能になったため、採用地図にはしない。
- `village_square_trimmed_slam_partial_20260621`: `713x789`, `35.6 x 39.5m`,
  occupied `6771`, unknown `337616`, free `218170`。壁率は `1.2%` で、GT の `14.2%` より
  かなり低い。
- `maps/village_square_trimmed_slam_partial_20260621_vs_world.{png,json,csv}`:
  WBT 真値との near ratio は全体 `0.720`、fence `0.706`、obstacle `0.813`。1本の
  `PicketFence` が coverage `0.000`、複数の `StreetLight` / `TrafficCone` も coverage `0.000`。
- 評価用 GT `maps/village_square_trimmed_gt.yaml`: `700x700`, `35.0 x 35.0m`,
  occupied `65627`, unknown `27600`, free `396773`。
- 評価用 GT `maps/village_park_trimmed_gt.yaml`: `700x700`, `35.0 x 35.0m`,
  occupied `44418`, unknown `27600`, free `417982`。
- `check_map_vs_world.py` による GT 生成地図の自己整合:
  - `village_square_trimmed`: fence `4/4`, obstacle `16/16`, near ratio `1.000`。
  - `village_park_trimmed`: fence `6/6`, obstacle `23/23`, near ratio `1.000`。
- MID360 で捉えられる特徴量:
  - `village_square_trimmed` 初期位置 `(0,0)`: WBT 由来障害物は 5m 以内 `2`、10m 以内 `18`,
    15m 以内 `27`, 25m 以内 `31`。四象限は `[9,8,7,7]`。
  - `village_park_trimmed` 初期位置 `(0,0)`: WBT 由来障害物は 5m 以内 `2`、10m 以内 `15`,
    15m 以内 `33`, 25m 以内 `37`。四象限は `[11,8,9,9]`。
  - Webots の MID360 近似 LiDAR は水平 `360deg`, vertical `~59deg`, max range `40m` なので、
    初期位置から複数方向に十分な静的特徴を観測できる。

参考にした一次情報:

- Nav2 Map Server: https://docs.nav2.org/configuration/packages/configuring-map-server.html
- Nav2 Mapping and Localization: https://docs.nav2.org/setup_guides/sensors/mapping_localization.html
- Nav2 Navigating while Mapping: https://docs.nav2.org/tutorials/docs/navigation2_with_slam.html
- slam_toolbox: https://github.com/SteveMacenski/slam_toolbox
- Webots Solid / boundingObject: https://cyberbotics.com/doc/reference/solid

次の低成績箇所:

1. `Starting point in lethal space` の原因を `/scan`、SLAM 地図、global/local costmap のどこで
   生んでいるか切り分ける。候補は低すぎる静的障害物の inflation、raytrace による free 漏れ、
   走行中の自己位置ずれ、または costmap obstacle/voxel 層の高さ帯。
2. `village_square_trimmed_slam_partial_20260621` の欠落した fence / streetlight を起点に、
   MID360 の `/scan` 投影高さ、slam_toolbox の obstacle 表現、frontier goal の setback を評価する。
3. 合格した SLAM 地図だけを `maps/village_square_trimmed.yaml` として置き、
   `generate_outdoor_waypoints.py` と
   `webots_outdoor_waypoint_nav.launch.py` の入力に使う。

### 2026-06-21 改善サイクル10: 屋外マッピング costmap の役割分担

方針:

- 低成績箇所は cycle9 の `Starting point in lethal space`。これは個別 frontier の問題ではなく、
  屋外マッピング中の `/scan`、SLAM 地図、global/local costmap、planner の役割分担が曖昧なこととして扱う。
- Nav2 公式 docs では Costmap2D は planner/controller が衝突や高コスト領域を扱うための環境表現、
  ObstacleLayer は 2D raycasting、VoxelLayer は 3D/depth 系の 3D raycasting を2Dへ潰す層。
  今回の入力は既に `pointcloud_to_laserscan` 後の 2D LaserScan なので、ObstacleLayer と
  VoxelLayer の両方へ同じ `/scan` を入れる構成は過剰と判断した。
- SLAM 中の大域計画は `/map` の `static_layer + inflation_layer` に限定し、生 `/scan` は
  local costmap の即時障害物回避と slam_toolbox の地図生成に使う。これにより、一時的なセンサ点が
  global costmap のロボット足元を lethal にして Smac の start cell を潰す経路を断つ。

採用:

- `config/nav2_params_webots_explore_outdoor.yaml` の `local_costmap.plugins` を
  `["obstacle_layer", "inflation_layer"]` に変更し、`voxel_layer` を外した。
- local `obstacle_layer` に `footprint_clearing_enabled: True`,
  `observation_persistence: 0.0`, `scan.inf_is_valid: True` を明示した。足元をセンサ由来障害物として
  残さず、`+inf` ray も local clearing に使うため。
- `global_costmap.plugins` を `["static_layer", "inflation_layer"]` に変更し、global の
  `obstacle_layer` / `voxel_layer` を外した。
- 変更は屋外専用 `config/nav2_params_webots_explore_outdoor.yaml` に限定し、屋内 params は変更しない。

未採用:

- LaserScan を `ObstacleLayer` と `VoxelLayer` の両方に入れ続けること。VoxelLayer は3D/depth系向けで、
  今回の 2D `/scan` 入力では二重 marking になりやすい。
- global costmap に生 `/scan` obstacle を入れ続けること。探索時の大域計画は SLAM 地図基準にし、
  近傍の衝突回避は local costmap に分担させる。
- cycle10 地図を `maps/village_square_trimmed.yaml` として採用すること。start lethal は消えたが、
  wall/feature coverage はまだ不十分。

評価値:

- 静的検証: `python3 -c "import yaml; yaml.safe_load(open('config/nav2_params_webots_explore_outdoor.yaml'))"` 成功。
- launch parse: `ros2 launch susumu_object_perception webots_outdoor_mapping.launch.py --show-args` 成功。
- live 起動ログで local costmap は `obstacle_layer` + `inflation_layer`、global costmap は
  `static_layer` + `inflation_layer` のみを読み込んだ。
- `village_square_trimmed.wbt` / `mode:=realtime` / `map_name:=village_square_trimmed_cycle10`:
  frontier #1 `(1.2,0.4)`, #2 `(2.0,-3.8)`, #3 `(-1.7,-12.2)` は到達。前回 start lethal が出た
  4つ目付近の frontier `(-9.7,-6.1)` は `path valid` で走行開始し、検証中に
  `Starting point in lethal space` は出なかった。
- 同じ #4 で次の問題として `Failed to make progress` が2回発生した。recovery 後に再走行したが、
  bounded 評価として地図を保存して終了した。
- `maps/village_square_trimmed_cycle10.yaml/.pgm`: `664x767`, `33.2 x 38.4m`,
  occupied `5219`, unknown `325927`, free `178142`。壁率 `1.0%`、free 最大連結成分 `100%`,
  連結片数 `4`。
- `maps/village_square_trimmed_cycle10_vs_world.{png,json,csv}`:
  WBT 真値との near ratio は全体 `0.729`、fence `0.716`、obstacle `0.813`。
  cycle9 partial の全体 `0.720` から微増、連結片数は `12 -> 4` に改善したが、GT 壁率 `14.2%`
  には届かない。

参考にした一次情報:

- Nav2 Costmap2D: https://docs.nav2.org/configuration/packages/configuring-costmaps.html
- Nav2 ObstacleLayer: https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html
- Nav2 VoxelLayer: https://docs.nav2.org/configuration/packages/costmap-plugins/voxel.html
- Nav2 Smac 2D Planner: https://docs.nav2.org/configuration/packages/smac/configuring-smac-2d.html
- Nav2 Tuning Guide: https://docs.nav2.org/tuning/index.html

次の低成績箇所:

1. `Failed to make progress` の原因を `/cmd_vel` 実測、local costmap、frontier goal の距離/姿勢、
   DWB critic のどこで起きているか切り分ける。
2. 欠落している `PicketFence` と細い `StreetLight` / `TrafficCone` を拾えるよう、SLAM 用 `/scan`
   の高さ帯・range・frontier の走行経路を見直す。これは地図品質の本体で、costmap role separation
   だけでは解決しない。
3. 合格地図の条件は、少なくとも `Starting point in lethal space` なし、`Failed to make progress`
   なし、GT near ratio の明確改善、外周 fence の coverage `0.0` 解消とする。

### 2026-06-21 改善サイクル11: 長距離 frontier を経路上の中間ゴールに分割

方針:

- 低成績箇所は cycle10 の `Failed to make progress`。cycle10 では start lethal は抑えられたが、
  4つ目 frontier で長いゴールへ向かう途中に DWB / progress checker 側で詰まった。
- Nav2 の `SimpleProgressChecker` は一定時間内に必要移動量を満たせないと失敗扱いにする。
  屋外探索 params は既に `required_movement_radius=0.2` / `movement_time_allowance=45.0` まで
  緩めているため、これ以上 controller 判定を鈍らせるのではなく、frontier 実行単位を短くする。
- `ComputePathToPose` の検証済み path を使い、経路上の `max_path_goal_distance` m 先を
  `NavigateToPose` に送る staged frontier にする。単純な直線クリップではなく planner 経路上で
  切るので、壁越しの中間点を作らない。
- 正解地図 `maps/*_gt.yaml` は実行中に使わない。保存した SLAM 地図の評価だけに使う。

採用:

- `frontier_explore_node.py` に `max_path_goal_distance` parameter を追加した。既定 `0.0` は無効なので、
  屋内 mapping の挙動は変えない。
- `webots_outdoor_mapping.launch.py` だけ `max_path_goal_distance:=4.0` を既定にした。屋外専用
  launch からのみ staged frontier を有効化し、屋内 launch / params へ波及させない。

未採用:

- cycle11 地図を `maps/village_square_trimmed.yaml` として採用すること。progress failure は消えたが、
  6本目の staged goal で start cell が lethal になり、地図品質も改善しなかった。
- `movement_time_allowance` をさらに伸ばすこと。今回の本質は、1 goal が長すぎることと、
  移動後に現在セルが static map 上で lethal になることであり、進捗判定をさらに遅らせても
  地図品質は上がらない。

評価値:

- 静的検証:
  - `python3 -m py_compile susumu_object_perception/frontier_explore_node.py` 成功。
  - `python3 -c "import yaml; yaml.safe_load(open('config/nav2_params_webots_explore_outdoor.yaml'))"` 成功。
  - `ros2 launch susumu_object_perception webots_outdoor_mapping.launch.py --show-args` 成功。
  - `colcon build --packages-select susumu_object_perception --symlink-install` 成功。
- live:
  - `village_square_trimmed.wbt` / `mode:=realtime` /
    `map_name:=village_square_trimmed_cycle11` /
    `max_path_goal_distance:=4.0`。
  - この run はユーザーの「次からシミュレーション動作時は rviz も起動」指示の前に
    `rviz:=False` で起動済みだった。次回以降のライブ検証は、明示的なヘッドレス検証を除き
    `rviz:=True` で起動する。
  - staged nav は `5` 回発生し、`Reached the goal!` は `5` 回。
  - `Failed to make progress` は `0` 回。cycle10 の低成績箇所は改善した。
  - 6本目の staged goal `(-6.4, -4.8)` 付近で `Starting point in lethal space` が発生し、
    その後 `364` candidates が全て unreachable になった。`Starting point in lethal space` は
    bounded log で `378` 回。
- `maps/village_square_trimmed_cycle11.yaml/.pgm`: `670x719`, `33.5 x 36.0m`,
  occupied `4998`, unknown `295133`, free `181599`。壁率 `1.0%`。
- `maps/village_square_trimmed_cycle11_vs_world.{png,json,csv}`:
  WBT 真値との near ratio は全体 `0.720`、fence `0.716`、obstacle `0.750`。
  cycle10 の全体 `0.729`、obstacle `0.813` より下がった。画像目視でも右上 fence は
  ほぼ unknown のままで、採用地図にはしない。

参考にした一次情報:

- Nav2 SimpleProgressChecker: https://docs.nav2.org/configuration/packages/nav2_controller-plugins/simple_progress_checker.html
- Nav2 Controller Server: https://docs.nav2.org/configuration/packages/configuring-controller-server.html
- Nav2 ComputePathToPose: https://docs.nav2.org/configuration/packages/bt-plugins/actions/ComputePathToPose.html
- Nav2 DWB Controller: https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html

次の低成績箇所:

1. 6本目の staged goal 後に、なぜロボットの start cell が SLAM static map / global costmap 上で
   lethal になったかを切り分ける。候補は、stage goal が未知/障害物に近すぎる、SLAM 更新で
   ロボット足元近傍が occupied 化する、または frontier 方向が狭い外周フェンスへ寄りすぎること。
2. `max_path_goal_distance` をさらに短くするだけでなく、経路上の中間ゴールに
   map clearance / unknown clearance を要求して、lethal 化しやすい点を避ける。
3. 次回 live は `rviz:=True` で local/global costmap、planned path、robot footprint を見ながら、
   start cell lethal 直前の位置を目視確認する。

## 旧方針: sparse outdoor は本線から外す

以下は、特徴の少ない `outdoor.wbt` / `city_robot.wbt` を対象にした過去実験の記録。
今回の屋外タスクは特徴豊富な都市部・公園を前提にするため、採用本線ではない。

2026-06-21 の改善サイクルで、フェンス欠落のような局所問題からいったん離れ、物が少ない
屋外ナビの全体方針を見直した。結論は、`outdoor.wbt` / `city_robot.wbt` のような特徴点が少ない
屋外では、2D LiDAR SLAM で安定した地図を作って AMCL/Nav2 へ渡す構成を本線にしない。
屋外の本線は次の分担にする。

- **global localization**: GPS/GNSS + IMU/odom。Nav2 公式の GPS navigation tutorial は
  GPS を global positioning source とし、`robot_localization` で sensor fusion して Nav2 の
  GPS waypoint follower を使う構成を示している。
- **heading**: GPS 単体では姿勢が出ないため、絶対 heading を IMU/magnetometer、dual GPS、
  または地図照合等で得る。Webots では `/imu` が使えるため、まずは IMU yaw を使う。
- **obstacle / traversability**: LiDAR/画像は地図生成の主役ではなく、局所障害物と走行可能領域の
  cost grid を作るために使う。Nav2 maintainer も、屋外では GPS/BYO localization と、
  環境に合わせた obstacle vs navigable preprocessing が必要だと述べている。
- **mapping**: 2D OccupancyGrid は「屋外ナビの前提」ではなく、レビュー・局所障害物確認・
  既知構造物の可視化に格下げする。広域 sparse outdoor で map 品質を上げるなら 3D LiDAR/visual/LIO 系を別系統で検討する。

参考にした一次情報:

- Nav2 GPS navigation tutorial: https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html
- Nav2 transform setup guide: https://docs.nav2.org/setup_guides/transformation/setup_transforms.html
- Nav2 concepts / state estimation: https://docs.nav2.org/concepts/index.html
- Nav2 Costmap 2D docs: https://docs.nav2.org/configuration/packages/configuring-costmaps.html
- Nav2 robot_localization setup guide: https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html
- Nav2 Waypoint Follower docs: https://docs.nav2.org/configuration/packages/configuring-waypoint-follower.html
- Nav2 DWB Controller docs: https://docs.nav2.org/configuration/packages/configuring-dwb-controller.html
- Nav2 SimpleProgressChecker docs: https://docs.nav2.org/configuration/packages/nav2_controller-plugins/simple_progress_checker.html
- Nav2 NavigateToPose BT action docs: https://docs.nav2.org/configuration/packages/bt-plugins/actions/NavigateToPose.html
- robot_localization navsat_transform_node docs: https://docs.ros.org/en/melodic/api/robot_localization/html/navsat_transform_node.html
- Open Robotics Discourse「How to Use NAV2 on Outdoor Vehicles」: https://discourse.openrobotics.org/t/how-to-use-nav2-on-outdoor-vehicles/38525

### GPS waypoint / localization prototype（2026-06-21）

上の方針をこのリポジトリへ取り込む第一歩として、地図・SLAM・AMCL・Nav2 を使わず、
GPS 位置と IMU/odom yaw だけで相対 waypoint を走る `scripts/outdoor_gps_waypoint_nav_node.py` を追加した。
これは最終構成ではなく、「物が少ない屋外は地図なし global positioning で走る」という前提を
この環境で検証するための足場。

次の段階として、GPS/IMU 由来の global pose を Nav2 互換の TF ツリーへ入れる
`scripts/outdoor_gps_localization_node.py` と `launch/webots_outdoor_gps_nav.launch.py` を追加した。
Nav2 公式 docs は `map->odom->base_link` 系の TF を要求し、GPS navigation tutorial では
GPS を global positioning source として `robot_localization` と waypoint follower へ渡す構成を
示している。この prototype は、Webots の GPS が現状 `PointStamped` であることを踏まえ、
まず `map->odom` を直接 publish して Nav2 接続前の座標系責務を検証する。

入力:

- GPS: `auto` 検出。`/gps`、`/TurtleBot3Burger/gps`、`*/gps` のうち
  `NavSatFix` / `PointStamped` / `Vector3Stamped` / `Odometry` を購読する。
- heading: `/imu` yaw を優先し、なければ `/odom` yaw、最後に GPS 移動方向を使う。
- obstacle: 任意で `/scan` 前方 cone の停止判定を使う。今回の smoke では純粋な GPS 制御を見るため
  `obstacle_stop_range_m:=0.0` とした。
- waypoint: `maps/outdoor_gps_smoke_waypoints.yaml`。初期 GPS 位置からの相対 `[x_m, y_m]`。

地図なし follower 単体の実行:

```bash
ros2 launch susumu_object_perception webots_simulation.launch.py \
  world:=outdoor.wbt nav:=False rviz:=False perception:=False \
  omni_perception:=False image_recognition:=False colored_slam:=False mode:=realtime

ros2 run susumu_object_perception outdoor_gps_waypoint_nav_node.py \
  --ros-args \
  -p waypoints_file:=$HOME/ros2_ws/src/susumu_object_perception/maps/outdoor_gps_smoke_waypoints.yaml \
  -p output_prefix:=/tmp/outdoor_gps_smoke \
  -p mission_timeout_sec:=70.0 \
  -p waypoint_timeout_sec:=22.0 \
  -p goal_tolerance_m:=0.35 \
  -p max_linear_mps:=0.18 \
  -p obstacle_stop_range_m:=0.0
```

GPS localization bridge と follower の実行:

```bash
ros2 launch susumu_object_perception webots_outdoor_gps_nav.launch.py \
  world:=outdoor.wbt mode:=realtime run_follower:=True \
  output_prefix:=/tmp/outdoor_gps_nav
```

`run_follower:=False` なら Webots + GPS localization だけを起動する。評価器や別の follower を
先に/後から組み合わせるときに使う。

ライブ結果（follower 単体 smoke）:

- GPS topic: `/TurtleBot3Burger/gps` (`geometry_msgs/msg/PointStamped`)
- result: `complete`
- reached: `4/4`
- missed: `[]`
- samples: `184`
- GPS path length: `2.078m`
- final distance: `0.346m`
- Nav2/SLAM/AMCL/map_server は未使用

ライブ結果（GPS localization bridge、`webots_outdoor_gps_nav.launch.py run_follower:=False` で
localization を先に起動し、評価器を動かしてから follower を投入）:

```bash
ros2 run susumu_object_perception evaluate_gps_vs_slam.py \
  --estimate-frame map --robot-frame base_footprint \
  --duration-sec 32 --sample-period 0.5 \
  --min-samples 20 --min-path-length 1.5 \
  --max-direct-error 0.25 --max-aligned-error 0.05 \
  --out-prefix /tmp/outdoor_gps_localization_cycle_eval3 \
  --use-sim-time --require-pass
```

- GPS topic: `/TurtleBot3Burger/gps` (`geometry_msgs/msg/PointStamped`)
- follower result: `complete`, reached `4/4`, missed `[]`
- follower GPS path length: `2.084m`
- localization samples: `63`
- GPS path length: `2.077m`
- `map` TF estimate path length: `2.075m`
- direct error: mean `0.001m`, max `0.007m`, rmse `0.002m`
- aligned error: mean `0.001m`, max `0.006m`, rmse `0.002m`
- validation: pass

直前の試行では `map->odom` の stamp が wall time になり、`/odom->base_footprint` の sim time と
噛み合わず TF extrapolation が出た。`webots_outdoor_gps_nav.launch.py` の localization node に
`use_sim_time:=True` を渡すことで解消した。

採用した変更:

- `outdoor_gps_waypoint_nav_node.py`: sparse outdoor 用の mapless GPS waypoint follower。
- `outdoor_gps_localization_node.py`: GPS/IMU/odom から `/outdoor_gps/odometry` と `map->odom`
  を publish する Nav2 互換の global localization bridge。
- `webots_outdoor_gps_nav.launch.py`: SLAM/AMCL/Nav2 を切り、Webots + GPS localization +
  任意の GPS follower を起動する屋外専用 entry point。
- `maps/outdoor_gps_smoke_waypoints.yaml`: 0.8m 四角形の smoke waypoint。

### GPS localization + Nav2 navigation prototype（2026-06-21）

GPS localization bridge を Nav2 へ接続する段階として、`launch/webots_outdoor_gps_nav2.launch.py`、
`config/nav2_params_outdoor_gps.yaml`、`scripts/outdoor_nav2_waypoint_nav_node.py` を追加した。

設計:

- `webots_simulation.launch.py` は `nav:=False` / `slam:=False` で起動し、AMCL・SLAM・map_server を
  起動しない。
- `outdoor_gps_localization_node.py` が `map->odom` を publish する。
- Nav2 は `nav2_bringup/launch/navigation_launch.py` だけを起動する。公式 docs の state estimation
  と同じく `map->odom` は位置推定系、`odom->base_link` はオドメトリ系の責務に分ける。
- global/local costmap は static map を使わず、`/scan` の obstacle layer + inflation layer の
  rolling window とする。GPS は global localization、LiDAR は local obstacle/traversability の
  役割に留める。
- `outdoor_nav2_waypoint_nav_node.py` は `maps/outdoor_gps_smoke_waypoints.yaml` の相対 `[x, y]` を
  `map` frame の `NavigateToPose` として順に投げ、JSON/CSV/Markdown の評価レポートを残す。

実行:

```bash
ros2 launch susumu_object_perception webots_outdoor_gps_nav2.launch.py \
  world:=outdoor.wbt mode:=realtime run_waypoints:=True \
  output_prefix:=/tmp/outdoor_nav2_gps_cycle1_fixed
```

ライブ結果（`outdoor.wbt` / `mode:=realtime` / 0.8m square smoke）:

- Nav2 lifecycle: `Managed nodes are active`
- waypoint result: `complete`
- reached: `4/4`
- missed: `[]`
- TF path length: `2.425m`
- final sampled distance: `0.389m`
- Nav2 goal checker tolerance: `0.35m`（最終サンプル周期の都合で report の最後の距離はやや大きい）

同じ Nav2 接続構成で走行中に GPS 真値と `map->base_footprint` TF を比較した結果:

- samples: `105`
- GPS path length: `2.142m`
- `map` TF estimate path length: `2.142m`
- direct error: mean `0.0006m`, max `0.0056m`, rmse `0.0013m`
- aligned error: mean `0.0006m`, max `0.0062m`, rmse `0.0012m`
- validation: pass

5m 級 waypoint 評価（`outdoor.wbt` / `mode:=realtime` / 2026-06-21）:

| waypoint | 採用 | 結果 | 走行/評価値 | 判断 |
|---|---|---|---|---|
| `maps/outdoor_gps_5m_axis_waypoints.yaml` | ✅ 採用 | `complete`, reached `4/4`, missed `[]` | runner TF path `18.753m`, final distance `0.355m`。GPS/TF eval samples `168`, GPS path `18.789m`, TF path `18.785m`, direct error mean `0.002m` / max `0.007m`, aligned error mean `0.002m` / max `0.007m`, validation pass | sparse outdoor の 5m 級 GPS/Nav2 接続 baseline として採用。障害物ストレスを外し、長距離の `map->odom` / rolling costmap / `NavigateToPose` が崩れないことを確認する用途 |
| `maps/outdoor_gps_5m_waypoints.yaml` | ❌ 未採用 | `complete`, reached `2/4`, missed `[1, 3]` | runner TF path `27.988m`, final distance `5.267m`。GPS/TF eval samples `168`, GPS path `13.360m`, TF path `13.552m`, direct error mean `0.004m` / max `0.121m`, aligned error mean `0.004m` / max `0.118m`, strict threshold `0.08m` は fail | 自己位置の平均誤差は小さい一方、#1 `[-5,-5]` で約 `(-3.91,-4.72)` に止まり goal まで `1.121m` 残った。#3 `[0,0]` では `13.286m` 走って final `5.267m` まで逸れた。5m 級 baseline ではなく、障害物/経路追従ストレス試験として残す |
| `maps/outdoor_gps_clearance_patrol_waypoints.yaml` | ✅ 採用 | `complete`, reached `4/4`, missed `[]` | runner TF path `13.131m`, final distance `0.330m`。GPS/TF eval samples `148`, GPS path `13.149m`, TF path `13.146m`, direct error mean `0.002m` / max `0.007m`, aligned error mean `0.002m` / max `0.007m`, validation pass | 5m 四角経路の失敗から、`x<=-4` 付近の深い左側コーナーを直接目標にせず `x=-2` corridor で西側を巡回する route selection へ切り替えた。任意の角を waypoint にするのではなく、clearance のある route graph / lane で waypoint を選ぶ方針の採用版 |
| `config/nav2_params_outdoor_gps_smac_rpp.yaml` + `maps/outdoor_gps_5m_waypoints.yaml` | ❌ 未採用 | `mission_timeout`, reached `1/4`, missed `[1, 2, 3]` | runner TF path `14.395m`, final distance `3.921m`。GPS/TF eval samples `217`, GPS path `13.574m`, TF path `13.679m`, direct error mean `0.004m` / max `0.008m`, aligned error mean `0.003m` / max `0.010m`, validation pass | SmacPlanner2D + RPP は起動・自己位置は正常だが、#1 `[-5,-5]` で約 `(-3.91,-4.88)`、#2 `[-5,0]` で約 `(-3.92,-0.37)` に残り、DWB より悪化した。controller/planner 差し替えだけでは解決しないため未採用 |

上の比較から、5m 級でまず悪いのは GPS global localization ではなく、障害物を含む経路での
planner/controller/local costmap の組み合わせ。Nav2 公式の GPS tutorial は rolling global
costmap を選択肢として示し、Costmap 2D / DWB / SimpleProgressChecker docs はそれぞれ
環境表現・Dynamic Window Approach・進捗判定の責務を分けている。次サイクルは局所的な
1パラメータ調整ではなく、障害物あり route selection と controller/costmap 構成をまとめて評価する。
2026-06-21 の追加比較では、SmacPlanner2D + RPP の大きな差し替えでも `x<=-4` 付近の failure は
解消せず、clearance route selection へ切り替えると同じ Nav2 DWB 構成で完走した。このため、
屋外 sparse world では「Nav2 が任意のGPS点へ自由空間計画で行く」前提を弱め、既知の走行可能
corridor / route graph / keepout を持つ上位 route selection を本線にする。

途中で見つけた問題と対応:

- `config/nav2_params_outdoor_gps.yaml` の costmap `width` / `height` を `4.0` / `12.0` のような
  float で書くと、Humble の costmap が整数として宣言済みの parameter へ double を設定する形になり
  `InvalidParameterTypeException` で落ちた。既存 params と同じ整数表記（`4` / `12`）に変更。
- `outdoor_nav2_waypoint_nav_node.py` の goal timeout を ROS timer で実装すると、sim time の初期
  ジャンプで最初の waypoint だけ即時 timeout した。timeout は wall clock (`time.monotonic`) で判定するよう変更。
- Nav2 の起動を Webots/GPS より早くしすぎると lifecycle が不安定になることがあったため、
  `webots_outdoor_gps_nav2.launch.py` では Nav2 起動を `10s`、waypoint runner 起動を `22s` に遅らせた。
- 5m 級評価では `webots_outdoor_gps_nav2.launch.py` に `goal_timeout_sec` と `mission_timeout_sec`
  launch 引数を追加し、短い smoke と長めの waypoint 評価で timeout を切り替えられるようにした。
- Rotation Shim Controller は最新 docs の入れ子例をそのまま使うと Humble では
  `FollowPath.primary_controller` が未初期化として configure 失敗した。Humble 互換調査が済むまで
  未採用。今回の比較は RPP を直接 controller plugin にした。

採用した変更:

- `webots_outdoor_gps_nav2.launch.py`: sparse outdoor 用の GPS localization + Nav2 navigation entry point。
- `nav2_params_outdoor_gps.yaml`: static map なし、rolling window obstacle costmap の Nav2 params。
- `outdoor_nav2_waypoint_nav_node.py`: `NavigateToPose` で屋外相対 waypoint を順に送り、結果を保存する評価 runner。
- `maps/outdoor_gps_5m_axis_waypoints.yaml`: 5m 級 GPS/Nav2 接続 baseline。障害物ストレスを外して
  長距離の基礎性能を評価する。
- `maps/outdoor_gps_clearance_patrol_waypoints.yaml`: `outdoor.wbt` の採用 patrol route。
  任意の左奥角を目標にするのではなく、失敗した `x<=-4` corridor を避けて西側カバーを `x=-2`
  corridor へ落とす。

採用しなかった変更:

- AMCL / slam_toolbox / map_server を屋外 GPS Nav2 構成へ戻さない。`map->odom` は GPS localization が
  供給するため、AMCL/SLAM を同時起動すると TF 競合になる。
- `robot_localization/navsat_transform_node` への本格移行はまだ未採用。Webots GPS が
  `PointStamped` で出ているため、`NavSatFix` 化、ENU 原点、covariance、heading source を整理してから行う。
- `maps/outdoor_gps_5m_waypoints.yaml` は 5m 級 baseline としては未採用。障害物/経路追従ストレス試験
  として残し、次サイクルで原因を大きく見る。
- `config/nav2_params_outdoor_gps_smac_rpp.yaml` は未採用。SmacPlanner2D + RPP は起動できるが、
  5m 四角経路では DWB 版より到達数が下がった。

次の低成績箇所:

1. `outdoor_gps_clearance_patrol_waypoints.yaml` を手作りから自動生成へ進める。候補は
   WBT 由来の keepout/corridor、または route graph から waypoint を作る generator。
2. `x<=-4` 付近で詰まる理由を `/scan` / global_costmap / local_costmap で説明する。
   失敗を見つけて避けるだけでなく、次の route generator の keepout 根拠にする。
3. `robot_localization/navsat_transform_node` 形式へ寄せる。まず Webots `PointStamped` GPS を
   `NavSatFix` または local odometry 相当へ変換し、公式 GPS waypoint follower へ接続する。

| world | サイズ | 内容 | 状況 |
|---|---|---|---|
| `outdoor.wbt` | 20 x 20m | PottedTree x4, SimpleBuilding x2, 車 x1 | ❌ scan 設定の試行錯誤でも実用品質に至らず |
| `city_robot.wbt` | 20 x 20m | outdoor + Pedestrian | ❌ 同上 |
| `village_center.wbt` | 800 x 800m（実走行は中心付近） | Webots 公式の村中央。フェンス・椅子・樹木・建物が密集 | △ **occ マークは出るが品質は不十分**（後述） |

## 屋内と屋外は完全に別物として扱う（重要）

屋内マッピングとは設定もコードもタスクも完全に分離する。屋外を動かすために屋内設定を改変
することは禁止。詳細は [`mapping_indoor.md`](mapping_indoor.md#屋内と屋外は完全に別物として扱う重要)
を参照。

具体的には:

- `launch/webots_simulation.launch.py` の `pointcloud_to_laserscan` は屋内向け実績値で
  固定（min_height:0.1, max_height:2.0, range_max:40, use_inf:True）。屋外も拾えるようにする
  目的での調整は禁止。
- `launch/webots_indoor_mapping.launch.py`（屋内専用）と
  `launch/webots_outdoor_mapping.launch.py`（屋外実験用）は別ファイル。屋外実験 launch から
  屋内 launch / 屋内 params を参照しない。
- `config/nav2_params_webots_explore.yaml`（屋内）と
  `config/nav2_params_webots_explore_outdoor.yaml`（屋外）は別ファイルで管理。`slam_toolbox`
  の `max_laser_range` も両ファイルで個別に持つ。

## 実験的な屋外マッピング（実用品質ではないが動かせる）

```bash
ros2 launch susumu_object_perception webots_outdoor_mapping.launch.py \
  world:=village_center.wbt map_name:=village_center mode:=realtime \
  explore_radius:=12.0
```

`explore_radius` は frontier 探索をロボット初期位置から半径 R[m] に制限するパラメータ。
広域 world でも狭い範囲だけ走らせるための仕掛けで、village_center のように `Floor` が散在
する world でも床のある一点に置けば走行できる。

### village_center で観測されたこと（2026-06-20、実用品質には届かない）

- 240 秒 / 半径 12m 制限で `frontier nearly gone → exploration complete`
- 保存地図: 36.6 x 40m、free 819m2、occ 14.3m2（102 クラスタ、4 セル以上）
- `eval_map_quality.py`: 最大連結成分 100%、判定 OK
- ロボット中心の半径 12m 円の中で Cypress 樹木・Fence・街灯・建物の占有マークは出る
- **scan は屋内設定（min_height:0.1, max_height:2.0, use_inf:True）のまま流用しただけ**

これは「outdoor.wbt で occ=0（建物消失）」「v4 設定で自己位置喪失」だった状態よりは前進だが、
本格運用にはまだ確認が足りない。

### GPS 真値との自己位置比較（2026-06-21）

屋外マッピングで未取得だった「GPS 真値と TF 推定位置の差分」を測るため、
`scripts/evaluate_gps_vs_slam.py` を追加した。Webots の GPS topic を自動検出し、GPS 由来の相対
移動量と TF（既定は `map -> base_footprint`、必要に応じて `odom`）を同時記録して
JSON/CSV/Markdown に出す。

対応した GPS topic/type:

- `/gps` または `/TurtleBot3Burger/gps`、および `*/gps` の自動検出
- `sensor_msgs/msg/NavSatFix`
- `geometry_msgs/msg/PointStamped`
- `geometry_msgs/msg/Vector3Stamped`
- `nav_msgs/msg/Odometry`

静止確認:

```bash
ros2 launch susumu_object_perception webots_simulation.launch.py \
  world:=outdoor.wbt nav:=False rviz:=False perception:=False \
  omni_perception:=False image_recognition:=False colored_slam:=False mode:=fast

ros2 run susumu_object_perception evaluate_gps_vs_slam.py \
  --estimate-frame odom --duration-sec 10 --sample-period 0.5 \
  --min-samples 5 --min-path-length 0.0 \
  --out-prefix /tmp/outdoor_gps_odom_static_eval
```

結果: `/TurtleBot3Burger/gps` (`geometry_msgs/msg/PointStamped`) を自動検出し、20 samples、
GPS path 0.000m、odom path 0.000m、direct max error 0.000m で pass。

移動確認（評価器が不整合を検出できることの確認、`outdoor.wbt` / `mode:=fast` / 手動直進）:

```bash
ros2 run susumu_object_perception evaluate_gps_vs_slam.py \
  --estimate-frame odom --duration-sec 15 --sample-period 0.5 \
  --min-samples 10 --min-path-length 0.5 \
  --max-direct-error 0.5 --max-aligned-error 0.5 \
  --out-prefix /tmp/outdoor_gps_odom_move_eval --require-pass

ros2 topic pub -r 10 /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.2}, angular: {z: 0.0}}"
```

結果: 30 samples、GPS path 3.965m、odom path 7.383m、direct max error 3.418m、
rigid aligned max error 1.865m で fail。GPS は約 3.96m で止まっている一方、odom は進み続けた。
これは屋外マッピング採用品質の証明ではなく、衝突・滑り・fast 実行時の物理/制御ループ差などで
TF 推定と真値が乖離する状況を検出できることの確認として扱う。

realtime 実マッピング中の `map` 評価（`village_center.wbt` / `explore_radius:=8.0` /
`save_map:=False` / 約70秒）:

```bash
ros2 launch susumu_object_perception webots_outdoor_mapping.launch.py \
  world:=village_center.wbt mode:=realtime rviz:=False save_map:=False \
  explore_radius:=8.0 goal_timeout_sec:=45.0

ros2 run susumu_object_perception evaluate_gps_vs_slam.py \
  --estimate-frame map --duration-sec 70 --sample-period 1.0 \
  --min-samples 20 --min-path-length 0.5 \
  --max-direct-error 2.0 --max-aligned-error 1.0 \
  --out-prefix /tmp/village_center_map_gps_realtime_eval
```

結果: 68 samples、GPS path 8.888m、map 推定 path 8.704m、direct max error 0.610m、
rigid aligned max error 0.070m で pass。direct error 約0.55mは初期 map 原点と GPS 原点の
並進差が主で、剛体合わせ後の残差は小さい。短時間の自己位置整合は良好だが、240秒級の
完走マッピング、保存地図品質、wbt 配置との occupied 整合はまだ別途評価が必要。

`fast` は軽量確認だけに使い、採用判断には使わない。

参考にした一次情報:

- Webots GPS reference: https://www.cyberbotics.com/doc/reference/gps
- slam_toolbox README: https://github.com/SteveMacenski/slam_toolbox
- Nav2 Mapping and Localization setup guide: https://docs.nav2.org/setup_guides/sensors/mapping_localization.html

### wbt 配置との定量照合（2026-06-21）

`check_map_vs_world.py` は屋内向けの単純パーサで、village_center では遠方の少数オブジェクトだけを
拾って軸が広がり、地図が小さく潰れていた。改善として次を追加した。

- 既定表示を地図範囲へ固定（全 world を見たい場合は `--show-all-world`）
- `StreetLight` / `PicketFence` / `PicketFenceWithDoor` / `Cypress` / bench / bin /
  building 系など、village_center の主要 PROTO を追加
- `SimpleBuilding` の `corners` polygon と `Pose` 内の `IndexedFaceSet` ground polygon を重畳
- 高さ 0.1m の `DEF DELIMITER Pose` は scan の occupied 期待値から外し、cyan の低い路面マーカーとして表示
- `--report` で wbt サンプル点から地図 occupied までの距離を JSON 出力

```bash
ros2 run susumu_object_perception check_map_vs_world.py \
  --wbt webots_worlds/village_center.wbt \
  --map maps/village_center.yaml \
  --out /tmp/village_center_vs_world_quant.png \
  --report /tmp/village_center_vs_world_quant.json \
  --object-report /tmp/village_center_vs_world_quant_objects.csv
```

初回改善時の現行地図での結果:

- 改善前: `Floor 0 / Wall 0 / 障害物 9` しか拾えず、地図が小さく潰れる
- 改善後: `building=27 / fence=37 / floor=4 / marking=6 / obstacle=157`
- occupied 0.5m 以内: 69 inside samples 中 55（near ratio 0.797）
- kind 別: `fence` 0.675、`obstacle` 0.966

つまり、樹木・ベンチ等の点状障害物は現行地図に比較的出ている一方、フェンスはまだ
occupied 0.5m 以内に入らないサンプルが多い。次に直すなら、フェンスの期待形状（PROTO 寸法の
近似）と、地図側の細長い occupied の欠落を分けて調べる。

2026-06-21 の追加改善で、`PicketFence` / `PicketFenceWithDoor` を公式 boundingObject に合わせた。
`PicketFence` は `translation` を中心に横長矩形を置くのではなく、ローカル `-Y` 方向へ伸びる
細い Box（`translation 0 (-0.85 * numberOfSegments) 0.55`、`size 0.04 (1.7 * numberOfSegments) 1.1`）
として評価する。`PicketFenceWithDoor` は公式の3つの Box を個別に投影する。さらに、4隅だけでなく
中心線上を 0.25m 間隔でサンプリングし、object 単位の coverage も JSON に出すようにした。

公式形状 + 中心線サンプリングでの結果:

- parsed: `building=27 / fence=57 / floor=4 / marking=6 / obstacle=157`
- 全体: 258 inside samples 中 172 が occupied 0.5m 以内（near ratio 0.667）
- fence: 229 inside samples 中 144（sample near ratio 0.629）
- fence object: 57 個中 15 個が地図範囲内にサンプルを持ち、そのうち 11 個は少なくとも一部が
  occupied 0.5m 以内（object near-any ratio 0.733）
- fence mean object coverage: 0.521

従来の `fence=0.675` より sample near ratio は下がったが、これは期待形状を公式 boundingObject に
寄せ、フェンス線の途中の欠落を拾えるようになったためで、評価器としては改善。次の低成績箇所は
`picket fence with door(7):1`、`picket fence with door(10):1/2`、`picket fence(27)` など、
official shape で coverage が低い fence が本当に地図で欠けているか、または scan 高さ/範囲条件で
拾えていないかを調べること。

参考にした一次情報:

- Webots PicketFence PROTO: https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/garden/protos/PicketFence.proto
- Webots PicketFenceWithDoor PROTO: https://raw.githubusercontent.com/cyberbotics/webots/R2025a/projects/objects/garden/protos/PicketFenceWithDoor.proto
- Webots Lidar reference: https://raw.githubusercontent.com/cyberbotics/webots/R2025a/docs/reference/lidar.md

### fence 欠落のセル状態診断（2026-06-21）

低 coverage fence が「未探索 unknown」なのか「既知 free に潰れている」のかを分けるため、
`check_map_vs_world.py` の JSON/CSV に sample ごとの地図セル状態を追加した。分類は保存 PGM の値で
`occupied < 50`、`free >= 250`、それ以外を `unknown` とする。`--object-report` を指定すると
object 単位で coverage、occupied/free/unknown/outside sample 数を CSV に出す。

採用した変更:

- `--report`: sample ごとに `cell_value` と `cell_class` を追加
- `--object-report`: object 単位の coverage / cell count CSV を追加
- 標準出力に `--worst-kind`（既定 `fence`）の低 coverage object を表示

採用しなかった変更:

- `pointcloud_to_laserscan` や SLAM/Nav2 のパラメータ変更は未採用。今回の結果だけでは、
  細い/低い fence を 2D scan 地図の occupied として必ず残すべきか、まだ判断できない。

現行 `maps/village_center.yaml` の再評価:

- 全体: 258 inside samples 中 172 が occupied 0.5m 以内（near ratio 0.667）
- fence: 229 inside samples 中 144（sample near ratio 0.629）
- fence cell counts: `occupied=73 / free=90 / unknown=66 / outside=743`
- fence object coverage: 57 個中 15 個が地図範囲内、mean object coverage 0.521

低 coverage fence の内訳:

| object | coverage | inside | near | cell counts | max dist |
|---|---:|---:|---:|---|---:|
| `picket fence with door(6):1` | 0.000 | 2 | 0 | `unknown=2` | 2.496m |
| `picket fence with door(10):2` | 0.000 | 6 | 0 | `free=6` | 0.934m |
| `picket fence with door(7):1` | 0.000 | 8 | 0 | `free=8` | 2.295m |
| `picket fence with door(10):1` | 0.000 | 8 | 0 | `free=8` | 2.214m |
| `picket fence(27)` | 0.200 | 15 | 3 | `occupied=1 / free=14` | 2.673m |

このため、少なくとも `picket fence with door(7):1`、`picket fence with door(10):1/2`、
`picket fence(27)` は「未探索」よりも、地図が free として確定している箇所に fence 真値が
重なっている。次は該当 fence 付近で `/scan` と `/lidar/points` を確認し、Webots Lidar が
boundingObject を拾えていないのか、SLAM 側が細線 occupied を保持できていないのかを分ける。
Webots Lidar は OpenGL depth から depth/range を生成し、`minRange`/`maxRange` 外は infinity に
なるため、まずは該当地点で range が返っているかを見る。

### fence 検査 waypoint 生成（2026-06-21）

低 coverage fence 周辺で `/scan` と `/lidar/points/point_cloud` を確認するには、毎回同じ位置・同じ向きで
対象を見る必要がある。そのため `scripts/generate_fence_probe_waypoints.py` を追加し、
`check_map_vs_world.py` の WBT/地図照合結果から、低 coverage fence を見る検査用 waypoint を生成する。

```bash
ros2 run susumu_object_perception generate_fence_probe_waypoints.py \
  --wbt webots_worlds/village_center.wbt \
  --map maps/village_center.yaml \
  --out maps/village_center_fence_probes.yaml \
  --out-png maps/village_center_fence_probes.png \
  --max-targets 6
```

採用した変更:

- `maps/village_center_fence_probes.yaml`: 低 coverage fence 6件を対象にした検査 waypoint。
- `maps/village_center_fence_probes.png`: 地図上で robot pose → target を確認する PNG。
- `waypoint_nav_node.py` / `waypoint_viz_node.py`: 既存の `[x, y]` に加え、`[x, y, yaw_rad]` を
  後方互換で読めるようにした。yaw 付き waypoint は NavigateToPose の向きに反映し、RViz では矢印表示する。
- 2026-06-21 追加: 検査 waypoint をロボット起点から到達可能な free 連結成分へ限定し、
  出力順を coverage 順から nearest-neighbor 順へ変更した。最初に遠方 goal を投げると
  DWB が progress failure に入りやすかったため。
- 2026-06-21 追加: `scripts/fence_probe_sensor_check_node.py` を追加した。probe waypoint 付近で
  `/scan` と `/lidar/points/point_cloud` の target 方位ヒットを自動集計し、
  `<output_prefix>.json/.csv/.md` を出す。

採用しなかった変更:

- まだ `pointcloud_to_laserscan`、SLAM、Nav2 のパラメータは変えていない。今回の生成物は
  sensor 側か SLAM 側かを切り分けるための再現性確保で、地図品質そのものの改善ではない。

生成結果:

| # | object | coverage | pose `(x,y,yaw)` | target | clearance |
|---:|---|---:|---|---|---:|
| 0 | `picket fence(27)` | 0.200 | `(4.25, -0.40, 129.1deg)` | `(3.30, 0.77)` | 2.70m |
| 1 | `picket fence with door(10):1` | 0.000 | `(6.55, 6.80, -20.4deg)` | `(7.95, 6.28)` | 2.35m |
| 2 | `picket fence with door(10):2` | 0.000 | `(8.40, 6.55, -48.0deg)` | `(9.40, 5.43)` | 2.12m |
| 3 | `picket fence with door(10):0` | 0.500 | `(9.25, 6.80, -51.9deg)` | `(10.17, 5.62)` | 1.95m |
| 4 | `picket fence with door(7):1` | 0.000 | `(14.60, 7.60, 41.0deg)` | `(15.73, 8.58)` | 1.00m |
| 5 | `picket fence(26)` | 0.500 | `(15.00, 6.95, -88.6deg)` | `(15.07, 4.16)` | 0.35m |

次のライブ検証:

```bash
ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
  world:=village_center.wbt \
  waypoints:=village_center_fence_probes.yaml \
  mode:=realtime \
  slam:=False \
  map_file:=$HOME/ros2_ws/src/susumu_object_perception/maps/village_center.yaml \
  loop:=False \
  rviz:=True

ros2 bag record /scan /lidar/points/point_cloud /tf /tf_static \
  /waypoint_nav/status

ros2 run susumu_object_perception fence_probe_sensor_check_node.py \
  --ros-args \
  -p probe_file:=$HOME/ros2_ws/src/susumu_object_perception/maps/village_center_fence_probes.yaml \
  -p output_prefix:=/tmp/village_center_fence_probe_sensor_check \
  -p min_samples_per_probe:=3
```

判定観点:

1. 該当 waypoint で `/lidar/points/point_cloud` に target 方位の点が出るか。
2. 点群には出るが `/scan` に出ないなら、`pointcloud_to_laserscan` の高さ帯・range・投影条件が原因。
3. `/scan` に出るが保存地図では free なら、SLAM/Karto 側の raytrace・細線保持・自己位置差が原因。
4. 点群にも出ないなら、Webots Lidar/PROTO 視認性または該当 fence を 2D scan 地図で期待する妥当性を見直す。

ライブ評価（2026-06-21 / 改善サイクル4）:

```bash
ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
  world:=village_center.wbt \
  waypoints:=village_center_fence_probes.yaml \
  mode:=realtime \
  slam:=False \
  map_file:=$HOME/ros2_ws/src/susumu_object_perception/maps/village_center.yaml \
  loop:=False \
  rviz:=False \
  perception:=False omni_perception:=False image_recognition:=False colored_slam:=False

ros2 run susumu_object_perception fence_probe_sensor_check_node.py \
  --ros-args \
  -p probe_file:=maps/village_center_fence_probes.yaml \
  -p output_prefix:=/tmp/cycle4_fence_probe_sensor_check \
  -p timeout_sec:=120.0 \
  -p min_samples_per_probe:=2
```

採用した変更:

- `fence_probe_sensor_check_node.py` のレポートに `nearest_pose_distance_m`、
  `nearest_pose_yaw_error_deg`、`nearest_pose_time_sec` を追加した。未サンプル時に
  「到達していない」のか「yaw 条件だけ満たさない」のかを次回切り分けるため。

採用しなかった変更:

- `pointcloud_to_laserscan`、SLAM、Nav2 tuning はまだ変えていない。今回の結果は、fence 欠落の
  原因が sensor/scan projection/SLAM のどこにあるかを分けるための評価で、地図生成値の採用変更ではない。

Nav2 実行結果:

- `waypoint_nav_node.py`: `reached=5/6 missed=[0]`
- #0 `(4.25, -0.40, yaw=129deg)` は `Failed to make progress` 後に timeout で skipped。
- #1-#5 は到達。#4/#5 は sensor check の `timeout_sec:=120.0` 後に到達したため今回の sensor 集計には入っていない。
- 参考にした一次情報: Nav2 の Controller Server は progress checker plugin で進捗判定を行い、
  progress checker は一定時間内に必要移動量を満たさない場合に失敗扱いにする。
  `NavigateToPose` は pose へ obstacle avoidance と recovery behavior 付きで移動する action。
  - https://docs.nav2.org/configuration/packages/configuring-controller-server.html
  - https://docs.nav2.org/configuration/packages/nav2_controller-plugins/pose_progress_checker.html
  - https://api.nav2.org/actions/humble/navigatetopose.html

sensor check 結果:

| # | object | map coverage | samples | scan seen | cloud seen | diagnosis |
|---:|---|---:|---:|---:|---:|---|
| 0 | `picket fence(27)` | 0.200 | 0 | 0 | 0 | `no_live_samples` |
| 1 | `picket fence with door(10):1` | 0.000 | 2 | 0 | 2 | `cloud_only_check_pointcloud_to_laserscan` |
| 2 | `picket fence with door(10):2` | 0.000 | 2 | 1 | 2 | `scan_and_cloud_see_target` |
| 3 | `picket fence with door(10):0` | 0.500 | 2 | 1 | 2 | `scan_and_cloud_see_target` |
| 4 | `picket fence with door(7):1` | 0.000 | 0 | 0 | 0 | `no_live_samples` |
| 5 | `picket fence(26)` | 0.500 | 0 | 0 | 0 | `no_live_samples` |

解釈:

- #1 は点群では 2/2 見えているが `/scan` では 0/2。まず `pointcloud_to_laserscan` の
  高さ帯、range、target cone、TF、または fence 位置の z 仮定を切り分ける。
- #2/#3 は点群 2/2、scan 1/2。少なくとも Webots Lidar と `/scan` が対象 fence を拾える瞬間はある。
  保存地図で coverage 0.0-0.5 に留まる原因は純粋なセンサ不可視だけではなく、SLAM の細線保持、
  raytrace、自己位置差、または観測回数不足が候補。
- #0 は今回の低成績箇所。到達前に progress failure で missed になったため、sensor/SLAM の判断材料がない。
- #4/#5 は Nav2 としては到達したが、sensor check の timeout 後だった。次回は timeout を延ばすか、
  sensor check を waypoint_nav の開始直後から動かす。

次の低成績箇所:

1. #0 の到達性改善。probe pose を起点側へ寄せる、または #0 を順序から後ろへ回してから再評価する。
2. #1 の cloud-only 問題。`/lidar/points/point_cloud` には出るため、`/scan` 変換条件を局所 bag で確認する。
3. 240秒級の `village_center.wbt` realtime マッピングで `evaluate_gps_vs_slam.py --estimate-frame map` を再実行する。

### 不足している検証（実用品質と呼ぶには必要）

1. **wbt 配置との整合性は初回定量化済みだが未合格**。`check_map_vs_world.py` は
   village_center 固有の `PicketFence`、`Cypress`、`StreetLight` 等を拾えるようになった。
   ただし公式 boundingObject + 中心線サンプリングでは fence sample near ratio 0.629、
   mean object coverage 0.521 に留まる。セル状態診断では fence inside samples のうち
   `free=90 / unknown=66` で、低 coverage fence の多くは既知 free 上にある。保存地図側の
   細長い occupied 欠落、SLAM/scan 条件、または低い/細いフェンスを 2D scan で期待する妥当性を
   分けて調べる。検査 waypoint と sensor check ノードは生成済みだが、realtime で probe へ
   実到達した状態の `/scan` / 点群 bag 記録は未実施。
2. **map 推定位置の正確性は短時間のみ確認済み**。`village_center.wbt` + `mode:=realtime` +
   `webots_outdoor_mapping.launch.py` の約70秒 run では、GPS path 8.888m に対して
   `map -> base_footprint` の剛体合わせ後 max error 0.070m で良好。ただし 240秒級の完走 run、
   広い範囲のループ、地図保存後の downstream 利用まで含めた品質は未確認。
3. **地図 bbox がロボット走行範囲より大きく出る**。explore_radius=12m に対し地図 36.6 x 40m。
   scan が遠方まで届いて遠方の occupied / free を焼くため、走行領域より広く見える。これが
   実害かどうかは下流タスク（ウェイポイント生成、巡回ナビ）で確認が必要。
4. **outdoor.wbt / city_robot.wbt のように特徴の少ない屋外 world は依然として未対応**。
   今回動いたのは village_center の中心付近に物体が密に並んでいたからで、設定の改善ではない。
5. **scan の仕様（屋内向け）が屋外で本当に妥当か再検証していない**。Karto の挙動は次節のとおりで、
   屋内設定そのままだと「広域 free を rasterize する」効果は限定的。

## 屋外で scan / SLAM が両立しない理由（試行錯誤の記録）

2026-06-20 に複数の設定を試した。いずれも片方しか満たせず、地図品質と SLAM 姿勢安定性の
両立に至らなかった。`Karto` のソース（`AddScan()`）を直接読んだ上での結論:

**Karto の挙動**:
1. `rangeReading >= scan.range_max` または `inf` の ray は完全に無視（free すら書かない）
2. `rangeReading >= RangeThreshold(slam の max_laser_range)` の ray は RangeThreshold で
   打ち切り、その点まで free として raytrace（端点は occupied としてマークしない）
3. `rangeReading < RangeThreshold` の hit ray は free raytrace + 端点を occupied としてマーク

**試した設定と症状**（outdoor.wbt / city_robot.wbt 等の特徴の少ない広域屋外）:

| 設定 | 症状 |
|---|---|
| (a) `use_inf:False, inf_epsilon:-0.5` で未ヒットを 15.5m の偽 hit にする | Karto がこれを (2) として処理し広い free 地図を出すが、別 ray の free 通過で建物 occupied が圧倒的に上書きされる → **地図全体が free（occ=0）になり建物・木・車が完全消失** |
| (b) `use_inf:True, max_height:5.0/10.0, range_max:20, max_laser_range:18` で実 hit を増やそうとする | 建物 occupied は残るが、未ヒット ray が完全無視されるため `/scan` の有効 hit が疎（723 点中 77 点等）になり、SLAM scan match の姿勢推定が不安定化 → **自己位置喪失** |
| (c) `use_inf:True, max_height:2.5, range_max:16, max_laser_range:15` で scan match 用 hit を増やす | 建物 occupied は残り、自己位置もある程度安定するが、地図が広域 world の全体を捉えきれない |
| (d) **屋内設定のまま** village_center で `explore_radius=12` 制限 | occ マークは出るが、上記「不足している検証」の項目が未取得で実用品質には届かない |

「広域空間の free 拡大」「特徴の少ない屋外の建物 occupied 保護」「scan match に必要な十分な hit 数」
の 3 つを同時に満たす設定が見つかっていない。MID-360 の対称 ±30° FOV と特徴の疎な広域 world
の組み合わせに固有の問題で、scan / SLAM パラメータの調整だけでは解けない可能性が高い。

## 取り組む場合の前提

将来このタスクに本格対応する場合の前提:

1. **屋内設定には絶対に影響させない**。屋外専用の `pointcloud_to_laserscan` ノードを
   別途立て、出力 topic を `/scan_outdoor` のように分け、屋外専用 slam_toolbox に渡す。
2. **より特徴の多い world で検証する**。Webots 公式の `village.wbt`、`city.wbt`
   (`/usr/local/webots/projects/vehicles/worlds/`) のような、建物・歩道・街灯・フェンスが
   密に配置された world で検証する。`outdoor.wbt` / `city_robot.wbt` のような特徴疎な world
   では原理的に困難。
3. **wbt 配置との定量整合を改善する**。`check_map_vs_world.py --report` の kind 別距離を使い、
   coverage が低い fence が「地図の欠落」か「scan 条件で見えない構造物」かを切り分ける。
4. **GPS 真値と map 推定位置の差分を長時間で測る**。`evaluate_gps_vs_slam.py` を使い、
   `mode:=realtime` の 240秒級マッピング run で `map -> base_footprint` と GPS 真値を比較する。
   odom だけの確認や `mode:=fast` の結果は採用判断に使わない。
5. **SLAM アルゴリズムも見直す**。slam_toolbox(Karto) は屋内向け前提が強い。屋外開放空間
   では Cartographer の global SLAM、LIO-SAM、FAST-LIO 等のアルゴリズムも検討する。
6. **scan を 3D 系に切り替える**。`pointcloud_to_laserscan` で 2D に潰さず、3D 点群 SLAM
   （colorized_pointcloud_mapper の路線）にする方が、広域屋外に向く可能性。

## 関連

- [マッピング（屋内）](mapping_indoor.md)
- [Webots シミュレーション環境ガイド](../webots_simulation.md)
- [world 一覧と使い分け](../worlds.md)
- [ロボット / LiDAR 構成](../robot_lidar.md)
- [MID-360 LiDAR 調査・Webots マッピングの罠](../mid360_lidar_research.md)

## 段差・縁石で yaw が崩れる問題の調査（2026-06-21）

ユーザー方針: **world は変更しない**。段差対策はアルゴリズム、監視、Nav2/SLAM 設定で行う。

cycle15 の `live_slam_truth_monitor.py` で、GPS 位置列だけでは見落としていた絶対方位ズレを
`/imu` yaw と `map->base_footprint` yaw の差として検出できた。

- `maps/village_square_trimmed_cycle15_yaw_monitor_truth_monitor.json`
- samples `230`, events `15`
- first yaw event: `idx=101`, truth `(6.06,-2.28)`, `yaw_error=11.2deg`
- その後 `yaw_error` は最大 `108.7deg`、`aligned_error` は最大 `2.58m`
- drift 開始点は `PicketFence { translation 7.0 -2.5 ... }` と `plaza_pavement` 端の近傍。

ネット調査からの判断:

1. Webots では `Solid.boundingObject` が衝突形状で、見た目と衝突形状は一致しなくてもよい。
   ただし今回は world を変えないため、これは「段差が物理接触を起こし得る」という原因確認に留める。
   - https://cyberbotics.com/doc/reference/solid
   - https://cyberbotics.com/doc/guide/tutorial-2-modification-of-the-environment
2. Nav2 の ObstacleLayer は 2D raycasting の costmap。低い段差・縁石を「踏めるか/踏めないか」
   で評価する機能ではない。`min_obstacle_height` / `max_obstacle_height` は高さで点を採るだけ。
   - https://docs.nav2.org/configuration/packages/costmap-plugins/obstacle.html
3. Nav2 の VoxelLayer は 3D raycasting で 3D 環境モデルを持ち、2D planning/control へ潰す。
   MID360 点群から低い段差や縁石を local costmap に入れる候補になるが、単体では
   「通行可能な小段差」と「危険な段差」の分類まではしない。
   - https://docs.nav2.org/configuration/packages/costmap-plugins/voxel.html
4. `robot_localization` は wheel odom と IMU など複数 odometry source を EKF/UKF で融合し、
   滑らかな `odom=>base_link` を作る用途に使える。段差で wheel/SLAM yaw が飛ぶ場合、
   IMU yaw を odom prior に入れる候補。ただし Webots 真値を SLAM に直接戻す設計にはしない。
   - https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html
5. 研究・実装例では、段差/縁石/荒れ地は 2D occupancy ではなく **traversability** として扱い、
   3D LiDAR の elevation grid map から height difference、roughness、slope を計算して
   走行リスクにする。これは今回の「worldを変えずに段差で壊れないようにする」方向と合う。
   - https://github.com/leggedrobotics/traversability_estimation
   - https://www.researchgate.net/publication/322432059_A_terrain_description_method_for_traversability_analysis_based_on_elevation_grid_map

採用候補（world 変更なし）:

1. **短期: yaw drift watchdog**
   `live_slam_truth_monitor.py` の yaw event を検証用だけでなく、マッピング中の安全停止/ゴール中断の
   トリガーにする。`yaw_error > 8deg` または IMU yaw rate と SLAM yaw rate の差が急増したら、
   frontier goal をキャンセルし、直近 hazardous pose を blacklist する。
2. **短期: local 3D段差ハザード層**
   `/lidar/points/point_cloud` をロボット近傍の elevation grid にし、セルごとの
   `max_z-min_z`、近傍 height jump、点密度を計算する。TurtleBot3 が乗り越えるべきでない
   2-3cm 以上の急な段差や縁石候補を local costmap の lethal/inscribed inflation として入れる。
   これは SLAM 用 `/scan` とは分離し、屋外 Nav2 だけに接続する。
3. **中期: robot_localization による odom prior 安定化**
   Webots の wheel odom と `/imu` を EKF で融合し、slam_toolbox の `odom_frame` prior を
   滑らかにする。真値GPSは評価専用に残し、SLAM 入力には入れない。
4. **中期: frontier goal の traversability check**
   `frontier_explore_node.py` が ComputePathToPose の経路を採用する前に、経路上の段差ハザードセルを
   サンプリングし、危険セルを含む中間ゴールを後退・再選択する。

次に実装する低成績箇所:

1. `live_slam_truth_monitor.py` の yaw event を `frontier_explore_node.py` が購読し、
   event 発生時に現在 goal を cancel / 周辺を blacklist する。
2. その次に local 3D段差ハザード層を追加し、`village_square_trimmed.wbt` の
   `(6,-2)〜(9,0)` 付近を避けて走れるか評価する。
