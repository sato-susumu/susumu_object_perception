# 認識タスク — 物体検出・分類・信号認識・予測

このページは README のタスク一覧「認識」のタスクページ。詳細なアルゴリズム、ノード構成、パラメータは
[認識タスク — LiDAR sensing/perception パイプライン](../autoware_perception.md) と
[信号認識](../traffic_light_recognition.md) に集約している。

## 入出力

| 項目 | 内容 |
|---|---|
| 入力 | 3D LiDAR 点群、全天球画像、2D `/map` |
| 実行 | `simulation.launch.py`、`webots_simulation.launch.py`、`webots_waypoint_nav.launch.py perception:=True omni_perception:=True image_recognition:=True` |
| 主な出力 | `/perception/tracked_objects`、`/perception/tracked_objects_classified`、`/perception/predicted_objects`、`/perception/predicted_costmap`、`/perception/traffic_signals`、RViz markers |
| Nav2 連携 | prediction のみを `/perception/predicted_costmap` として自作 costmap layer に max 合成 |

## 実行

```bash
# Gazebo cafe world: LiDAR perception + 画像認識 + 信号認識
ros2 launch susumu_object_perception simulation.launch.py

# Webots city: 車・歩行者・信号の認識
ros2 launch susumu_object_perception webots_city.launch.py mode:=realtime

# 巡回しながら認識
ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
  world:=city_robot.wbt waypoints:=city_waypoints.yaml mode:=realtime \
  perception:=True omni_perception:=True image_recognition:=True
```

CPU が厳しい場合は `image_recognition:=False` で YOLO 物体分類と全天球信号認識を切る。LiDAR perception は残る。

## 合格基準

1. **LiDAR 検出・追跡が成立している**
   `/perception/tracked_objects` が出て、移動物体に ID と速度が継続して付く。壁や地図外のゴーストが
   常駐しない。

2. **画像分類が late fusion として機能している**
   `/perception/tracked_objects_classified` と `/perception/object_classes/markers` が出る。近距離の車・人など、
   全天球画像上で十分な大きさに写る対象を COCO/Autoware クラスへ分類できる。

3. **信号認識が全周で機能している**
   `/perception/traffic_signals`、`/perception/traffic_light/rois`、`/perception/traffic_light/poses` が出る。
   全天球の隣接ビューで同じ信号が重複しても、方向統合で 1 件にまとまる。

4. **予測 costmap が Nav2 を壊していない**
   `/perception/predicted_costmap` が毎フレーム置換で出る。`PredictedCostmapLayer` は max 合成で他層を壊さず、
   static の壁を消さない。STVL 層へ戻さない。

5. **メッセージ型は既存型を使う**
   独自 `.msg` を追加しない。Autoware 型、標準型、`visualization_msgs` など既存型で表現する。

## 制約と注意

- HD 地図は使わない。2D 占有格子 `/map` を ROI/予測の壁判定に使う。
- YOLO 初期化失敗時に classic 方式へ勝手にフォールバックしない。失敗は `[FATAL]` として扱う。
- Nav2 costmap に焼くのは prediction の結果だけ。検出・追跡そのものや 3D 点群は焼かない。
- 人の現在位置と進路先は `prediction_node.py` が `/perception/predicted_costmap` に反映する。

## 関連

- [認識パイプライン詳細](../autoware_perception.md)
- [信号認識](../traffic_light_recognition.md)
- [全天球カメラ・LiDAR 色付き点群メモ](../omni_lidar_camera.md)
- [ノード接続図](../node_topology.md)
