# 巡回ナビタスク — ウェイポイントを Nav2 で順に回る

このページは README のタスク一覧「巡回ナビ」の詳細ページ。`generate_waypoints.py` が作った
ウェイポイント YAML を読み、Webots world 内で Nav2 の `NavigateToPose` を順に投げて巡回する。

## 入出力

| 項目 | 内容 |
|---|---|
| 入力 | `webots_worlds/<world>.wbt`、`outputs/waypoint_generation/<world>_waypoints.yaml` |
| 実行 | `launch/webots_waypoint_nav.launch.py` |
| 出力（ライブ） | `/waypoint_nav/status`、`/waypoints/markers`、Nav2 の走行結果 |
| 出力（最終） | `outputs/waypoint_generation/<world>_patrol_report.{json,csv,md}` (reached/missed 詳細)、`outputs/waypoint_generation/<world>_patrol_result.png` (地図に reached=緑/苦戦=黄/missed=赤 を重ねた可視化)。 `scripts/run_all_tasks.sh` が巡回完了後に `visualize_patrol_result.py` を必ず呼んで PNG を生成 |
| 出力（中間） | `experiments/waypoint_navigation/<YYYY-MM-DD>_<label>/`（`report_prefix.{json,csv,md}` / localization cycle / EKF/odom 比較 / radius multiplier 評価。gitignore） |
| 併用 | `perception:=True omni_perception:=True image_recognition:=True` で巡回しながら認識も実行可能 |

## 実行

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash

ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
  world:=indoor.wbt \
  waypoints:=indoor_waypoints.yaml \
  mode:=realtime \
  loop:=True
```

認識も併走させる場合:

```bash
ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
  world:=indoor.wbt waypoints:=indoor_waypoints.yaml mode:=realtime \
  perception:=True omni_perception:=True image_recognition:=True
```
<!-- iter92: 例を indoor ペアに統一 (iter89 で default 修正、 city_waypoints.yaml は legacy) -->


巡回後に PNG 可視化を手動で再生成する (run_all_tasks.sh は自動実行):
```bash
ros2 run susumu_object_perception visualize_patrol_result.py \
  --map outputs/mapping_indoor/<world>.yaml \
  --report /path/to/report.json \
  --out outputs/waypoint_generation/<world>_patrol_result.png
```

`waypoints` は `outputs/waypoint_generation/` 配下のファイル名で渡す。絶対パスではなく `indoor_waypoints.yaml` のように指定する。
既定は `slam:=True` で、巡回しながら `/map` を作り Nav2 の static layer に渡す。既存地図を使う
実験用に `slam:=False map_file:=<yaml>` も渡せる。Nav2 パラメータを差し替える場合は
`nav_params_file:=<yaml>` を使う。
waypoint YAML の `waypoints` は従来の `[x, y]` に加え、検査用途の `[x, y, yaw_rad]` も読める。
yaw がある場合は NavigateToPose の向きに反映し、`waypoint_viz_node.py` は矢印で表示する。
評価結果をファイルに残す場合は `report_prefix:=/abs/path/to/report` を渡す。各 waypoint 確定時に
`<prefix>.json`, `<prefix>.csv`, `<prefix>.md` を更新するため、長い巡回を途中で止めても
reached/missed が残る。`mission_timeout_sec:=<秒>` を渡すと wall-clock で評価全体を打ち切り、
未実行 waypoint を `mission_timeout` として report に残す。
自己位置精度も評価する場合は `truth_monitor:=True truth_report_prefix:=/abs/path/to/truth` を渡す。
Webots GPS/IMU truth と `map->base_footprint` の aligned error / heading error / yaw error を
`<prefix>.json`, `.csv`, `.md` に残す。真値 `/gps` は検証だけに使い、AMCL / SLAM / Nav2 へは入力しない。
監視ノードは `/waypoint_nav/status` の `mission complete` / `mission_timeout` を見て自動停止する。
各サンプルには直近の waypoint index と status text も残し、Markdown/JSON には waypoint 別の
max aligned / heading / yaw error と worst waypoint を出す。drift が終盤だけに出る場合は、AMCL
だけでなく地図終端形状、接近経路、goal tolerance の切り分けに使う。
`truth_odom_frame:=odom`（既定）も同時に評価し、`odom->base_footprint` の aligned / heading / yaw
error を map 推定とは別に出す。空文字にすると odom 評価だけ無効化できる。
`ekf_odom:=True` を渡すと評価専用 `robot_localization` EKF を起動し、`filtered_odom_topic`
（既定 `/odometry/filtered`）を truth monitor が同時評価する。EKF config は
既定 `config/ekf_odom_twist_imu_eval.yaml` で `publish_tf:false` にしているため、既存 Nav2 の TF へは入らない。
この既定 config は `/odom` の x/y pose を使わず、wheel odom の twist と `/imu` の yaw/yaw-rate だけを
融合する。`config/ekf_odom_eval.yaml` は pose+twist 比較用で、位置改善としては未採用。
EKF を `odom->base_link` の TF 発行元として切り分ける場合は
`ekf_odom_params_file:=config/ekf_odom_twist_imu_tf.yaml` と
`ros2_control_params_file:=config/webots_ros2control_ekf_odom_tf.yaml` を同時に渡す。後者は
diffdrive controller の `/odom` topic は残し、`enable_odom_tf:false` で TF だけ止める。通常起動の
既定は従来どおり diffdrive TF のまま。EKF TF 評価時は `ekf_odom_start_sec:=2.0` で EKF を早めに起動し、
`nav_start_delay_sec:=2.5` で Webots controller 接続後の Nav2 起動を少し遅らせると、`odom->base_link`
の初期 TF 待ちを避けやすい。
`config/webots_ros2control_ekf_odom_tf_radius1046.yaml` は wheel radius scale 切り分け用。
path length 比は改善するが odom aligned と進行性が悪化したため、通常の EKF TF 評価推奨値にはしない。
認識性能を優先する indoor run の実験では、通常巡回用 `indoor_waypoints.yaml` とは別に
`indoor_recognition_waypoints.yaml` を渡せる。これは occupied 小〜中サイズ成分を見る追加視点入りの
点列。ただし 2026-06-20/21 のライブ認識評価では余分検出増加または recall 低下で悪化したため未採用。
通常の完走合格や認識採用評価では `indoor_waypoints.yaml` を使う。

## 仕組み

`webots_waypoint_nav.launch.py` は内部で `webots_nav.launch.py` を起動し、Webots + Nav2 + SLAM を立てる。
その後:

- `waypoint_viz_node.py` が `/waypoints/markers` に番号付きウェイポイントと経路を出す。
- `waypoint_nav_node.py` が各点へ `NavigateToPose` を送る。
- Nav2 lifecycle bringup 中の一時的な goal reject は同じ点をリトライする
  （既定 `goal_reject_retries=8`、`goal_reject_retry_sec=2.0`）。リトライを使い切っても
  reject される点だけ missed にする。
- `goal_timeout_sec` を超えた点はスキップし、次の点へ進む。
- 1 周が終わると reached/missed を `/waypoint_nav/status` に出す。
- `report_prefix` が空でなければ、JSON/CSV/Markdown の reached/missed report を書く。
  長い屋外巡回では `mission_timeout_sec` で bounded にし、partial report を評価値として残す。
- `loop:=True` なら次の周回に入る。

1 点で詰まって全体が止まらないことを優先している。最終的な巡回品質は、missed を減らす方向で
ウェイポイントや Nav2 パラメータを調整する。

## 合格基準

1. **同じ world・同じ起点前提で走っている**
   ウェイポイント生成に使った地図と同じ world を使う。ロボット起動位置を変えた場合、map 座標がずれるので
   そのまま使わない。

2. **ウェイポイントが RViz で正しく表示される**
   `/waypoints/markers` が map 上に出て、壁や unknown 上に点がない。

3. **一周が完了する**
   `/waypoint_nav/status` に `lap finished` が出る。合格版では missed が無い、または原因説明できる少数に
   収まっていること。missed が多い場合は waypoint 生成、地図、Nav2 tuning の順に疑う。

4. **転倒・衝突がない**
   `/fall_detector/status` が転倒を示さない。`break_room` などバンパー付き world では
   `/collision_diagnostic/event` が出ない、または原因が説明済み。

5. **ナビの把握不全がない**
   `/scan` と local costmap が障害物を拾い、ロボットが壁・家具に継続的に押し付けられない。

## 制約と注意

- 品質評価では `mode:=realtime` を使う。`fast` は軽量な起動確認用。fast で問題が出たら realtime で再確認する。
- `webots_waypoint_nav.launch.py` の既定 `mode` は `fast` なので、評価時は明示的に `mode:=realtime` を渡す。
- 屋内 `indoor.wbt` の合格確認は `slam:=True` で行う。2026-06-20 の認識併走フル巡回では
  `reached=22/22 missed=[]` を確認済み。 2026-06-26 のライブ巡回テスト (iter1+iter5 で WP を
  19→8 に削減、 generate_waypoints.py に dedupe + grid NMS 追加後) は `reached=8/8 missed=[]`
  (約 6 分完走、 recovery 多発なし)。 旧 19 WP の `reached=13/19` から完走率改善を実証。
- `slam:=False map_file:=outputs/mapping_indoor/indoor.yaml nav_params_file:=config/nav2_params.yaml` の静的地図 AMCL
  モードは過去 `reached=22/22 missed=[]` を維持していた (22 WP 時代)。 iter46 で indoor_waypoints.yaml が
  9 WP に縮小されてからは `reached=9/9 missed=[]` (iter119 で SLAM 巡回モードで実測、 `outputs/waypoint_generation/indoor_patrol_result.png`)。
- `break_room.wbt` (19 WP) の `slam:=True` 巡回も iter126 で `reached=19/19 missed=[]` を実証
  (約 5.4 分完走、 recovery 0、 一発成功、 WP duration mean=17.1s)。 成果物は
  `outputs/waypoint_generation/break_room_patrol_{report.json,csv,md, result.png}` に配置。
  自己位置評価の要点は次に集約する。

  | 区分 | 要点 |
  |---|---|
  | 採用中 | AMCL は `max_beams=90`, `update_min_d/a=0.10`。静的地図 AMCL で max aligned は概ね `0.2m` 台を維持 |
  | 診断基盤 | truth monitor は waypoint context、`map->base_footprint`、`odom->base_footprint`、評価用 EKF を同時に記録する |
  | EKF | 採用 config は `config/ekf_odom_twist_imu_eval.yaml`。`/odom` x/y pose は融合せず、wheel twist + IMU yaw/yaw-rate を使う |
  | opt-in | EKF が `odom->base_link` TF を出す構成は評価用に残すが、通常既定は diffdrive TF のまま |
  | 未採用 | wheel radius multiplier `1.046` の既定化。path 比は改善したが odom aligned と進行性が悪化した |

  シミュレータ真値 `/gps` は AMCL / EKF / Nav2 へ入力せず、検証専用に留める。Nav2 パラメータの根拠と
  採用判断は [Nav2 tuning](../nav2_tuning.md) に残す。
- 実験用 `indoor_recognition_waypoints.yaml` は 2026-06-21 に `view_clearance=0.6m` で再生成した
  22 点列では `reached=22/22 missed=[]` を確認済み。ただし認識評価は通常巡回より悪化したため、
  ナビ合格・認識採用の基準にはしない。
- `waypoint_nav_node.py` は `NavigateToPose` を順に送る。`FollowWaypoints` 丸投げではない。
- 屋外巡回では `webots_outdoor_waypoint_nav.launch.py` が既定で `step_detector_avoid:=True`
  を渡し、 同 launch が `step_detector_node` も起動する (iter19 で統合)。 巡回中に
  段差/縁石/スタックを検知したら `goal_timeout_sec` (既定 120s) 満了を待たず即座に
  current WP を missed として次の WP へ進む。 屋内 (`webots_waypoint_nav.launch.py`
  単独) は既定 False のまま (段差が無い world では不要)。
- ウェイポイント YAML を手で直す場合も、必ず確認用 PNG/RViz で壁・unknown 上にないことを見る。
- Nav2 パラメータを変えたら [Nav2 tuning](../nav2_tuning.md) の現在値表と調整履歴サマリを更新する。

## 参考にした一次情報

- Nav2 Waypoint Follower: https://docs.nav2.org/configuration/packages/configuring-waypoint-follower.html
- Nav2 NavigateToPose: https://docs.nav2.org/configuration/packages/bt-plugins/actions/NavigateToPose.html
- Nav2 AMCL configuration: https://docs.nav2.org/configuration/packages/configuring-amcl.html
- Nav2 Smoothing Odometry using Robot Localization: https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html
- Nav2 Transform setup / REP-105 summary: https://docs.nav2.org/setup_guides/transformation/setup_transforms.html
- ROS 2 Control diff_drive_controller parameters: https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html
- ROS 2 Launch design / process orchestration: https://design.ros2.org/articles/roslaunch.html
- robot_localization EKF example parameters: https://github.com/cra-ros-pkg/robot_localization/blob/ros2/params/ekf.yaml
- Webots TurtleBot3Burger PROTO wheel radius: https://raw.githubusercontent.com/cyberbotics/webots/R2022b/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto
- nav_msgs/Odometry frame contract: https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html
- ROS 2 Actions: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

## 関連

- [ウェイポイント生成タスク](waypoint_generation.md)
- [マッピング（屋内）タスク](mapping_indoor.md)
- [Nav2 tuning](../nav2_tuning.md)
