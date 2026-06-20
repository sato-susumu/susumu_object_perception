# 巡回ナビタスク — ウェイポイントを Nav2 で順に回る

このページは README のタスク一覧「巡回ナビ」の詳細ページ。`generate_waypoints.py` が作った
ウェイポイント YAML を読み、Webots world 内で Nav2 の `NavigateToPose` を順に投げて巡回する。

## 入出力

| 項目 | 内容 |
|---|---|
| 入力 | `webots_worlds/<world>.wbt`、`maps/<world>_waypoints.yaml` |
| 実行 | `launch/webots_waypoint_nav.launch.py` |
| 出力 | `/waypoint_nav/status`、`/waypoints/markers`、Nav2 の走行結果 |
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

## 仕組み

`webots_waypoint_nav.launch.py` は内部で `webots_nav.launch.py` を起動し、Webots + Nav2 + SLAM を立てる。
その後:

- `waypoint_viz_node.py` が `/waypoints/markers` に番号付きウェイポイントと経路を出す。
- `waypoint_nav_node.py` が各点へ `NavigateToPose` を送る。
- `goal_timeout_sec` を超えた点はスキップし、次の点へ進む。
- 1 周が終わると reached/missed を `/waypoint_nav/status` に出す。
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
- `waypoint_nav_node.py` は `NavigateToPose` を順に送る。`FollowWaypoints` 丸投げではない。
- ウェイポイント YAML を手で直す場合も、必ず確認用 PNG/RViz で壁・unknown 上にないことを見る。
- Nav2 パラメータを変えたら [Nav2 tuning](../nav2_tuning.md) の現在値表と調整履歴を更新する。

## 関連

- [ウェイポイント生成タスク](waypoint_generation.md)
- [マッピング（屋内）タスク](mapping_indoor.md)
- [Nav2 tuning](../nav2_tuning.md)
