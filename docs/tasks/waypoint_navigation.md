# 巡回ナビタスク — ウェイポイントを Nav2 で順に回る

このページは README のタスク一覧「巡回ナビ」の詳細ページ。`generate_waypoints.py` が作った
ウェイポイント YAML を読み、Webots world 内で Nav2 の `NavigateToPose` を順に投げて巡回する。

## 入出力

| 項目 | 内容 |
|---|---|
| 入力 | `webots_worlds/<world>.wbt`、`maps/<world>_waypoints.yaml` |
| 実行 | `launch/webots_waypoint_nav.launch.py` |
| 出力 | `/waypoint_nav/status`、`/waypoints/markers`、任意の `report_prefix.{json,csv,md}`、Nav2 の走行結果 |
| 併用 | `perception:=True omni_perception:=True image_recognition:=True` で巡回しながら認識も実行可能 |

## 実行

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash

ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
  world:=city_robot.wbt \
  waypoints:=city_waypoints.yaml \
  mode:=realtime \
  loop:=True
```

認識も併走させる場合:

```bash
ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
  world:=city_robot.wbt waypoints:=city_waypoints.yaml mode:=realtime \
  perception:=True omni_perception:=True image_recognition:=True
```

`waypoints` は `maps/` 配下のファイル名で渡す。絶対パスではなく `city_waypoints.yaml` のように指定する。
既定は `slam:=True` で、巡回しながら `/map` を作り Nav2 の static layer に渡す。既存地図を使う
実験用に `slam:=False map_file:=<yaml>` も渡せる。Nav2 パラメータを差し替える場合は
`nav_params_file:=<yaml>` を使う。
waypoint YAML の `waypoints` は従来の `[x, y]` に加え、検査用途の `[x, y, yaw_rad]` も読める。
yaw がある場合は NavigateToPose の向きに反映し、`waypoint_viz_node.py` は矢印で表示する。
評価結果をファイルに残す場合は `report_prefix:=/abs/path/to/report` を渡す。各 waypoint 確定時に
`<prefix>.json`, `<prefix>.csv`, `<prefix>.md` を更新するため、長い巡回を途中で止めても
reached/missed が残る。`mission_timeout_sec:=<秒>` を渡すと wall-clock で評価全体を打ち切り、
未実行 waypoint を `mission_timeout` として report に残す。
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
  `reached=22/22 missed=[]` を確認済み。
- `slam:=False map_file:=maps/indoor.yaml nav_params_file:=config/nav2_params.yaml` の静的地図 AMCL
  モードも、2026-06-20 の `mode:=realtime` 試験で `reached=22/22 missed=[]` を確認済み。
  これはナビ完走の確認であり、同条件の認識評価は採用中の SLAM 巡回結果より悪化したため
  認識成果物の採用条件にはしない。Nav2 パラメータの根拠と履歴は [Nav2 tuning](../nav2_tuning.md) に残す。
- 実験用 `indoor_recognition_waypoints.yaml` は 2026-06-21 に `view_clearance=0.6m` で再生成した
  22 点列では `reached=22/22 missed=[]` を確認済み。ただし認識評価は通常巡回より悪化したため、
  ナビ合格・認識採用の基準にはしない。
- `waypoint_nav_node.py` は `NavigateToPose` を順に送る。`FollowWaypoints` 丸投げではない。
- ウェイポイント YAML を手で直す場合も、必ず確認用 PNG/RViz で壁・unknown 上にないことを見る。
- Nav2 パラメータを変えたら [Nav2 tuning](../nav2_tuning.md) の現在値表と調整履歴を更新する。

## 参考にした一次情報

- Nav2 Waypoint Follower: https://docs.nav2.org/configuration/packages/configuring-waypoint-follower.html
- Nav2 NavigateToPose: https://docs.nav2.org/configuration/packages/bt-plugins/actions/NavigateToPose.html
- ROS 2 Actions: https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html

## 関連

- [ウェイポイント生成タスク](waypoint_generation.md)
- [マッピング（屋内）タスク](mapping_indoor.md)
- [Nav2 tuning](../nav2_tuning.md)
