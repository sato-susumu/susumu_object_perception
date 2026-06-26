# マッピングタスク（屋外） — 特徴が多い屋外 world の自律地図作成

このページは README のタスク一覧「マッピング（屋外）」の詳細ページ。屋内マッピングは別タスクで
[`mapping_indoor.md`](mapping_indoor.md) を参照。

## 入出力

| 項目 | 内容 |
|---|---|
| 入力 | `webots_worlds/village_square_trimmed.wbt` 等の屋外 world、3D LiDAR、必要なら GPS |
| 実行 | `launch/webots_outdoor_mapping.launch.py`、`scripts/evaluate_glim_map_variants.py`、`scripts/check_map_vs_world.py` |
| 出力（最終） | `outputs/mapping_outdoor/village_square_trimmed_glim2d.yaml` / `.pgm`、`outputs/mapping_outdoor/<world>_gt.yaml` / `.pgm`（評価専用）、 `outputs/mapping_outdoor/<world>_gt_preview.png` (`generate_webots_ground_truth_map.py --preview` が出す真値地図のレビュー画像、 iter37 で contracts に追加)（契約名・git 追跡） |
| 出力（中間） | `experiments/mapping_outdoor/<YYYY-MM-DD>_<label>/`（cycleNN snapshot/promote/variants/monitor/risk/yaw 等の比較ログ）、`experiments/mapping_outdoor/glim/`（GLIM TUM/PLY 中間版）。gitignore |
| 主な確認 | `scripts/check_map_vs_world.py`、`scripts/eval_map_quality.py` |

## 現在の本線

屋外本線は **GLIM で 3D 点群を作る → trajectory 条件を比較 → Nav2 用 2D 地図化 → waypoint 生成 →
保存地図巡回で評価** にする。`slam_toolbox` の `/scan` 2D SLAM を屋外本線にはしない。

理由:

- MID360 の 3D 情報を 2D LaserScan へ潰すと、段差・縁石・フェンス付近で yaw drift や占有欠落が出やすい。
- 特徴の少ない `outdoor.wbt` / `city_robot.wbt` は、2D scan match だけでは安定した地図を作りにくい。
- 都市部・公園のようにフェンス、植栽、ベンチ、街灯、小建物が複数方向にある world では、
  GLIM の 3D 点群から Nav2 用 2D map を作る方が筋が良い。

重要: `outputs/mapping_outdoor/*_gt.yaml` は正解データであり、`map_file` や waypoint 生成の入力にしない。
world 由来地図は評価専用。Nav2 が読む地図は、GLIM のセンサ点群から作った保存地図にする。

## 屋外専用成果物

| 役割 | 成果物 |
|---|---|
| world | `webots_worlds/village_square_trimmed.wbt`, `webots_worlds/village_park_trimmed.wbt` |
| 3D マッピング | `launch/webots_outdoor_glim_mapping.launch.py` + `config/glim_webots/` |
| PLY 保存 | `scripts/save_pointcloud2_to_ply.py` |
| trajectory 保存 | `scripts/save_pose_trajectory_to_tum.py` |
| 2D map 生成 | `scripts/glim_cloud_to_2d_map.py` |
| 2D map 候補比較 | `scripts/evaluate_glim_map_variants.py` |
| waypoint 生成 | `scripts/generate_outdoor_waypoints.py` |
| route 展開診断 | `scripts/expand_waypoint_route.py` |
| 保存地図巡回 | `launch/webots_outdoor_waypoint_nav.launch.py` |
| 正解データ生成/照合 | `scripts/generate_webots_ground_truth_map.py` / `scripts/check_map_vs_world.py`（評価専用） |
| **段差検出** | `susumu_object_perception/step_detector_node.py` (iter16 で新規追加)。 IMU roll/pitch、 加速度 z 急変、 odom-cmd_vel 進行率差で段差/坂/スタックを検出。 `/step_detector/{status,event,tilt_deg}` 出力 |
| **段差リカバリ (frontier 連携)** | `frontier_explore_node.py` が `/step_detector/event` の `tilt`/`stuck` を購読し、 発生時のロボット現在位置を `step_detector_blacklist_radius` (既定 1.5m) で hazard 化。 現在の goal をキャンセル + 同位置への再 goal を抑止する。 `webots_outdoor_mapping.launch.py` で既定 ON (iter17 で統合) |
| **段差イベント分析 (後追い)** | `scripts/visualize_step_events.py` (iter24 で新規追加)。 launch ログから `step_detector event=<type> ... around (x, y)` パターンを抽出し、 保存地図に段差検知点を色分け (tilt=赤 / stuck=橙) で重ねる PNG + JSON。 「どこで段差ハマりが何回起きたか」 を客観確認できる |
| 巡回診断 | `scripts/nav2_pose_costmap_monitor_node.py`、plan / actual corridor trace |

## 実行手順

`village_square_trimmed.wbt` の例:

```bash
ros2 launch susumu_object_perception webots_outdoor_glim_mapping.launch.py \
  world:=village_square_trimmed.wbt \
  mode:=realtime \
  rviz:=True

ros2 run susumu_object_perception save_pose_trajectory_to_tum.py \
  --topic /glim_ros/pose_corrected \
  --out experiments/mapping_outdoor/glim/village_square_trimmed_pose.tum \
  --duration-sec 600 \
  --timeout-sec 660 \
  --min-poses 100 \
  --qos reliable

ros2 run susumu_object_perception save_pointcloud2_to_ply.py \
  --topic /slam/glim_colorized_points_map \
  --out experiments/mapping_outdoor/glim/village_square_trimmed_points.ply \
  --timeout-sec 30 \
  --min-points 5000 \
  --qos sensor_data

ros2 run susumu_object_perception evaluate_glim_map_variants.py \
  --cloud experiments/mapping_outdoor/glim/village_square_trimmed_points.ply \
  --wbt webots_worlds/village_square_trimmed.wbt \
  --out-prefix experiments/mapping_outdoor/village_square_trimmed_glim2d_eval \
  --trajectory topic_pose=experiments/mapping_outdoor/glim/village_square_trimmed_pose.tum \
  --adopt-prefix outputs/mapping_outdoor/village_square_trimmed_glim2d \
  --waypoints-out outputs/waypoint_generation/village_square_trimmed_glim2d_waypoints.yaml \
  --waypoint-max-segment-length 4.0

ros2 run susumu_object_perception generate_webots_ground_truth_map.py \
  --wbt webots_worlds/village_square_trimmed.wbt \
  --out outputs/mapping_outdoor/village_square_trimmed_gt.yaml \
  --preview experiments/mapping_outdoor/village_square_trimmed_gt.png

ros2 run susumu_object_perception check_map_vs_world.py \
  --wbt webots_worlds/village_square_trimmed.wbt \
  --map outputs/mapping_outdoor/village_square_trimmed_glim2d.yaml \
  --out experiments/mapping_outdoor/village_square_trimmed_glim2d_vs_world.png \
  --report experiments/mapping_outdoor/village_square_trimmed_glim2d_vs_world.json \
  --object-report experiments/mapping_outdoor/village_square_trimmed_glim2d_vs_world.csv

ros2 launch susumu_object_perception webots_outdoor_waypoint_nav.launch.py \
  world:=village_square_trimmed.wbt \
  map_file:=$HOME/ros2_ws/src/susumu_object_perception/outputs/mapping_outdoor/village_square_trimmed_glim2d.yaml \
  waypoints:=$HOME/ros2_ws/src/susumu_object_perception/outputs/waypoint_generation/village_square_trimmed_glim2d_waypoints.yaml \
  mode:=realtime \
  loop:=False
```

`village_park_trimmed.wbt` も同じ手順で、ファイル名の prefix を `village_park_trimmed` に変える。
GLIM の loop closure 後出力を使う場合だけ、`ros2 run glim_ros offline_viewer` で `/tmp/dump` を開き、
Export Points と `traj_lidar.txt` を評価入力に追加する。

## 履歴サマリ

個別サイクルの詳細ログは長くなりすぎるため、判断に必要な要点だけ残す。

### 採用済み

- 屋外本線は GLIM-first。`webots_outdoor_glim_mapping.launch.py`、PLY 保存、TUM trajectory 保存、
  GLIM 2D map variant 評価、waypoint 生成、保存地図巡回を一連の評価経路にする。
- `evaluate_glim_map_variants.py` で trajectory なし / topic pose / GLIM dump trajectory を横並び評価し、
  JSON/CSV/Markdown に採用判断を残す。
- `generate_outdoor_waypoints.py` は屋外専用 wrapper として維持する。長距離 edge は
  `--waypoint-max-segment-length` で抑える。
- `nav2_pose_costmap_monitor_node.py` と plan / actual corridor trace を診断基盤として使う。
  waypoint/edge 単位で、計画が悪いのか、実軌跡が膨らむのか、復帰不能 pose に入るのかを分ける。
- `expand_waypoint_route.py` は候補生成・診断用に採用する。巡回順を壊さず、保存地図上の geodesic path
  に沿って中間 goal を追加できる。
- GPS / IMU / Nav2 prototype は sparse outdoor の別系統として有効。`outdoor_gps_clearance_patrol_waypoints.yaml`
  は 5m 級の GPS/Nav2 接続 baseline として残す。

### 未採用

- 屋外で `slam_toolbox` 2D SLAM を本線に戻すこと。局所 watchdog では yaw drift 後の地図崩れを回復できない。
- world 形状を変えて段差を消すこと。ユーザー方針により world は変更しない。
- Webots GPS や world 由来真値を SLAM / Nav2 / waypoint 生成へ入力すること。真値は評価専用。
- route clearance、edge clearance cost、local static layer、no-recovery BT、safe-pose guard の既定化。
  offline 指標や一部 reached 数は改善しても、live では `reached` が悪化または復帰不能 pose が増えた。
- `expand_waypoint_route.py` の出力をそのまま屋外本線 waypoint にすること。path tracking error は改善したが、
  危険 corridor を忠実に辿って lethal 近傍へ入る問題が残った。
- `outdoor.wbt` / `city_robot.wbt` のような特徴の少ない広域 world を、屋外 GLIM/SLAM 地図作成の
  合格対象にすること。現段階では sparse outdoor は GPS/route graph 系の別課題として扱う。

### 代表値

| 条件 | 結果 | 判断 |
|---|---|---|
| GLIM 2D map variant 評価 | topic pose trajectory が unknown を大きく減らすケースあり | trajectory 横並び評価を採用 |
| promoted map + segmented waypoint | live は概ね `reached=9..16/53` 程度で未合格 | 長距離 edge だけが主因ではない |
| edge clearance weighted route | offline shortfall は大幅改善、live は `reached=14/56` | 既定未採用 |
| route expansion 2m | path tracking error は改善、live `reached=22/96` | 診断用。既定未採用 |
| safe-pose guard | `reached=11/96` / `9/96` | recovery 方式として既定未採用 |
| GPS/Nav2 5m axis baseline | `reached=4/4`、GPS/TF error 数 mm〜cm 級 | sparse outdoor baseline として採用 |

### 次に見る低成績箇所

1. lethal pose に入った後の再計画ではなく、lethal 前の経路ブラックリスト化を評価する。
2. monitor の `pose_global_lethal_static_free`、plan corridor、actual corridor から、危険 edge を
   route 生成側で除外する。
3. 短時間の直接制御 escape を屋外専用・既定 OFF の候補として評価する。採用判断は reached 数と
   lethal/near-lethal event 数で行う。
4. GLIM map の loop closure 後 trajectory と topic pose trajectory の差を、同一 PLY で比較し続ける。

### 次に試すべき新規ツール候補 (2026-06-26 調査)

過去の cycle 改善で reached が頭打ちの場合、Nav2 / GLIM の既存パッケージで未活用のものを評価する。
**いずれも実機ライブ評価が要るので、屋外 cycle 専用 launch で opt-in 起動する形にしてから採用判断を行う。**

| 候補 | 入手元 | 期待効果 | 評価方法 |
|---|---|---|---|
| **Nav2 Collision Monitor** | nav2 公式パッケージ `nav2_collision_monitor` | cmd_vel への emergency-stop filter で lethal pose 進入を防ぐ。「lethal 前のブラックリスト化」と相補的 | 屋外 cycle で `slowdown_polygon` を設定して reached 数 / collision event 数の変化を見る |
| **Nav2 Route Server** | nav2 公式パッケージ `nav2_route` (新規) | outdoor lanes/corridors の graph-based 経路計画。SLAM 地図 + 注釈グラフで通行可能な corridor だけを使った route 生成が可能 | 屋外 cycle で `nav2_route` を planner_server と並走させ、reached/path_length を比較 |
| **GLIM v1.2.0 (2026/01)** | github.com/koide3/glim | GTSAM 4.3 / CUDA 13.1 対応の最新版。 MID360 公式サポート | 既存 GLIM (cycle19 で評価済み) を v1.2 にアップグレードし、loop closure 性能の差を比較 |
| **LiDAR scan matching aided INS** | 一般的な fusion 手法 | sparse outdoor の GPS denied で位置精度を保つ。 sparse outdoor 系の課題に直接効く | sparse outdoor 試験 world で EKF に LiDAR scan match 由来 odom を追加して reached を測る |

優先度: **Collision Monitor (1)** が最も軽量 (Nav2 既存 / 設定ファイルだけで効く)。 次に **Route Server (2)**。

## 屋内と屋外は完全に別物として扱う

屋内マッピングとは設定もコードもタスクも完全に分離する。屋外を動かすために屋内設定を改変しない。
詳細は [`mapping_indoor.md`](mapping_indoor.md#屋内と屋外は完全に別物として扱う重要) を参照。

- `launch/webots_simulation.launch.py` の `pointcloud_to_laserscan` は屋内向け実績値から動かさない。
- 屋外実験 launch から屋内 launch / 屋内 params を参照しない。
- `config/nav2_params_webots_explore.yaml`（屋内）と屋外向け params は別ファイルで管理する。
- 変更後の評価では `mode:=realtime` を使う。`fast` は起動確認だけ。

## 合格基準

屋外マッピングを採用扱いにするには、最低限次を満たす。

1. GLIM 由来の 3D 点群と trajectory が保存され、同じ入力から 2D map を再生成できる。
2. `validate_map_assets.py` で `.yaml/.pgm` のペアが OK。
3. `check_map_vs_world.py` で主要なフェンス・植栽・建物の coverage が説明可能な範囲にある。
4. 生成 waypoint が保存地図上の free / clearance 条件を満たし、確認 PNG でレビューできる。
5. `webots_outdoor_waypoint_nav.launch.py mode:=realtime` の bounded mission で reached/missed と
   monitor summary が JSON/CSV/Markdown に残る。
6. world 真値や GPS 真値を本体へ入力していないことが明確。

## 参考にした一次情報

- GLIM Home: https://koide3.github.io/glim/
- GLIM Getting started: https://koide3.github.io/glim/quickstart.html
- GLIM Installation: https://koide3.github.io/glim/installation.html
- GLIM Docker images: https://koide3.github.io/glim/docker.html
- GLIM Important parameters: https://koide3.github.io/glim/parameters.html
- GLIM Sensor setup guide: https://github.com/koide3/glim/wiki/Sensor-setup-guide
- TUM RGB-D dataset file formats: https://cvg.cit.tum.de/data/datasets/rgbd-dataset/file_formats
- ROS 2 PointCloud2 message: https://docs.ros.org/en/ros2_packages/humble/api/sensor_msgs/msg/PointCloud2.html
- ROS 2 QoS policies: https://docs.ros.org/en/rolling/Concepts/Intermediate/About-Quality-of-Service-Settings.html
- Webots WorldInfo: https://cyberbotics.com/doc/reference/worldinfo
- Webots Solid / boundingObject: https://cyberbotics.com/doc/reference/solid
- Nav2 Navigation Concepts: https://docs.nav2.org/concepts/index.html
- Nav2 GPS navigation tutorial: https://docs.nav2.org/tutorials/docs/navigation2_with_gps.html
- Nav2 Costmap 2D docs: https://docs.nav2.org/configuration/packages/configuring-costmaps.html
- robot_localization setup guide: https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html

## 関連

- [マッピング（屋内）](mapping_indoor.md)
- [ウェイポイント生成](waypoint_generation.md)
- [巡回ナビ](waypoint_navigation.md)
- [Webots シミュレーション環境ガイド](../webots_simulation.md)
- [world 一覧と使い分け](../worlds.md)
- [ロボット / LiDAR 構成](../robot_lidar.md)
- [MID-360 LiDAR 調査・Webots マッピングの罠](../mid360_lidar_research.md)
