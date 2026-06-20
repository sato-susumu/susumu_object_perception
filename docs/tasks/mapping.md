# マッピングタスク — 事前地図なし環境の自律地図作成

このページは README のタスク一覧「マッピング」の詳細ページ。事前地図のない Webots world を
frontier 探索で走らせ、`slam_toolbox` が作る 2D OccupancyGrid を `maps/<name>.pgm/.yaml` として
保存するところまでを扱う。

ウェイポイント生成・巡回ナビは別タスク。ここでの合格対象は**地図そのものの品質**だけ。

## 入出力

| 項目 | 内容 |
|---|---|
| 入力 | `webots_worlds/<world>.wbt`、3D LiDAR 由来の `/scan`、SLAM/Nav2 |
| 実行 | `launch/webots_city_mapping.launch.py` |
| 出力 | `maps/<map_name>.pgm`、`maps/<map_name>.yaml` |
| 主な確認 | RViz の `/map`、`scripts/map_progress_monitor.py`、`scripts/eval_map_quality.py`、`scripts/check_map_vs_world.py` |

## 実行

```bash
cd ~/ros2_ws
colcon build --packages-select susumu_object_perception --symlink-install
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
cd ~/ros2_ws/src/susumu_object_perception

# 重要: マッピング品質を評価する実行は realtime 固定。
ros2 launch susumu_object_perception webots_city_mapping.launch.py \
  world:=city_robot.wbt map_name:=city mode:=realtime

# 屋内/屋外の例
ros2 launch susumu_object_perception webots_city_mapping.launch.py \
  world:=break_room.wbt map_name:=break_room mode:=realtime
ros2 launch susumu_object_perception webots_city_mapping.launch.py \
  world:=outdoor.wbt map_name:=outdoor mode:=realtime sweep_mode:=True sweep_radius:=7.0
```

探索完了時に `save_map:=True` なら `maps/<map_name>.pgm/.yaml` が自動保存される。手動保存する場合:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_ws/src/susumu_object_perception/maps/<map_name>
```

## 合格基準

各 world について、次を満たしたらマッピングタスク合格。

1. **未開拓を残さず広く開拓できている**
   探索済み範囲の bounding box 内に unknown が大きく残っていない。到達可能な frontier を回り尽くし、
   自由空間の最大連結成分が大半を占める。広い屋外では `/scan` 点数が減ること自体は問題にしない。

2. **幾何が正しい**
   円形影がない。壁が単一線で、二重・三重にぶれていない。斜めノイズが目立たない。寸法が実 world と
   一致する。例: `indoor.wbt` は床 5 x 10 m の矩形として見える。

3. **world 定義と地図を照合できている**
   地図内部の連結性だけで合格にしない。`wbt` が定義する Floor、Wall の `translation`/`size`、
   建物・木・家具の位置と、地図上の occupied/free 配置・寸法を照合する。ロボット軌跡から放射状に
   スキャンを寄せ集めただけの星形地図は不合格。

4. **次タスクに渡せる保存物になっている**
   `maps/<map_name>.yaml` が保存され、PGM 画像への相対パス、`resolution`、`origin` が正しい。
   ウェイポイント生成はこの保存地図を入力にする。

## 必須制約

- 対象 world は実在する `wbt` のみ: `indoor.wbt` / `break_room.wbt` / `outdoor.wbt` /
  `city_robot.wbt`。`kitchen.wbt` / `home.wbt` は存在しない。
- マッピング品質を評価するときは必ず `mode:=realtime`。`fast` は Webots の物理に ROS 制御ループが
  追従できず odom が過大積算し、地図が崩れる。
- 全 `wbt` の Lidar `tiltAngle` は 0。非ゼロは Webots の点群高さ異常で円形影の原因になる。
- `/scan` は 2D LiDAR ではなく、3D LiDAR 点群から `pointcloud_to_laserscan` で作る。perception OFF でも
  `/scan` は出る。
- frontier 探索は未開拓優先。`gain` を大きく、`min_frontier_cells` を小さくしすぎない範囲で、
  広い未踏領域へ展開させる。
- 連続クリーン再起動で FastRTPS SHM が壊れ `/scan` が出ない場合は、SHM 無効化プロファイルか
  `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` を使う。

## 確認手順

```bash
# 探索中: 移動量と開拓余地を見る
ros2 run susumu_object_perception map_progress_monitor.py

# 保存後: 地図統計を確認
ros2 run susumu_object_perception eval_map_quality.py maps/<map_name>.yaml

# 保存後: wbt の真値構造と重ねて確認
ros2 run susumu_object_perception check_map_vs_world.py \
  --map maps/<map_name>.yaml \
  --wbt webots_worlds/<world>.wbt \
  --out maps/<map_name>_vs_world.png
```

数値だけでなく、`check_map_vs_world.py` の重畳図と RViz で目視確認する。移動量が十分あるのに
地図が広がらない場合は SLAM/環境を疑う。移動量が少ない場合は探索ゴール選択や Nav2 を疑う。

## 終了処理

Webots/RViz/Nav2 が残ると次回検証に混ざる。検証後は落とす。

```bash
ps aux | grep -E "webots|rviz|component_container|ros2 launch susumu|driver|pointcloud|frontier|slam|nav2|spawner" \
  | grep -v grep | awk '{print $2}' | xargs -r kill -9
```

## 関連

- [Webots シミュレーション環境ガイド](../webots_simulation.md)
- [world 一覧と使い分け](../worlds.md)
- [ロボット / LiDAR 構成](../robot_lidar.md)
- [MID-360 LiDAR 調査・Webots マッピングの罠](../mid360_lidar_research.md)
