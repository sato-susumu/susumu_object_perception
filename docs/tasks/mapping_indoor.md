# マッピングタスク（屋内） — 屋内 world の自律地図作成

このページは README のタスク一覧「マッピング（屋内）」の詳細ページ。事前地図のない屋内 Webots
world を frontier 探索で走らせ、`slam_toolbox` が作る 2D OccupancyGrid を
`maps/<name>.pgm/.yaml` として保存するところまでを扱う。

屋外マッピングは別タスク。[`mapping_outdoor.md`](mapping_outdoor.md) を参照（現状は
**「特徴の少ない広域屋外世界は未対応」** として保留）。

ウェイポイント生成・巡回ナビは別タスク。ここでの合格対象は**地図そのものの品質**だけ。

## 入出力

| 項目 | 内容 |
|---|---|
| 入力 | `webots_worlds/<world>.wbt`、3D LiDAR 由来の `/scan`、SLAM/Nav2 |
| 実行 | `launch/webots_indoor_mapping.launch.py` |
| 出力 | `maps/<map_name>.pgm`、`maps/<map_name>.yaml` |
| 主な確認 | RViz の `/map`、`scripts/map_progress_monitor.py`、`scripts/eval_map_quality.py`、`scripts/check_map_vs_world.py` |

## 対応 world

| world | サイズ | 内容 |
|---|---|---|
| `indoor.wbt` | 5 x 10m | 壁 + 家具 |
| `break_room.wbt` | 7.7 x 12.86m | 壁 + 家具 + バンパー(4方向 TouchSensor) |
| `cafe.wbt`(Gazebo) | — | 壁 + 家具 + 人(HuNavSim) |

いずれも壁・家具・人など SLAM の scan match で姿勢を取りやすい「特徴の多い」屋内環境。

## 屋内と屋外は完全に別物として扱う（重要）

設計方針として、**屋内マッピングと屋外マッピングは設定もコードもタスクも完全に分離**する。
屋外を動かすために屋内設定を改変すると屋内が壊れる事故が過去に起きた（2026-06-20: p2l の
`min_height/max_height/range_max/use_inf` を屋外向けに変えたら、屋内で建物 occupied が消える
ほどの劣化が起きた）。今後は次の分離を厳守:

- `launch/webots_simulation.launch.py` の `pointcloud_to_laserscan` の値は
  **屋内向け実績値（min_height:0.1, max_height:2.0, range_max:40, use_inf:True）から動かさない**。
  屋外も拾えるようにする目的での調整は禁止。屋外用に変える必要が出たら屋外専用 launch を
  用意するか、scan を屋外専用名に remap するなど屋内に影響しない設計にする。
- `config/nav2_params_webots_explore.yaml`（屋内 frontier 探索の `slam_toolbox` 含む）と
  `config/nav2_params_webots_explore_outdoor.yaml`（屋外向け）は別ファイルで管理し、
  片方の調整がもう片方に波及しないようにする。`slam_toolbox.max_laser_range` も
  両ファイルで個別に持つ。
- 屋外向けの実験的なコード追加（perimeter sweep、forward_step 拡張、Smac planner 切替など）は
  屋外専用 launch / params だけで完結させる。屋内 launch から辿るパスには触らない。

## 実行

```bash
cd ~/ros2_ws
colcon build --packages-select susumu_object_perception --symlink-install
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
cd ~/ros2_ws/src/susumu_object_perception

# 重要: マッピング品質を評価する実行は realtime 固定。
ros2 launch susumu_object_perception webots_indoor_mapping.launch.py \
  world:=indoor.wbt map_name:=indoor mode:=realtime

ros2 launch susumu_object_perception webots_indoor_mapping.launch.py \
  world:=break_room.wbt map_name:=break_room mode:=realtime
```

マッピング中に状態が分かりにくい場合は `rviz:=True` で RViz を出す。設定済みの
`rviz/simulation.rviz` には `/map`、`/scan`、`/frontier_explore/markers` が入っている。

CPU を SLAM/Nav2 に集中させたい場合、`image_recognition`、`colored_slam`、
`collision_diagnostics` は既定 OFF。必要なときだけ明示的に ON にする。

探索完了時に `save_map:=True` なら `maps/<map_name>.pgm/.yaml` が自動保存される。手動保存する場合:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_ws/src/susumu_object_perception/maps/<map_name>
```

## 合格基準

各 world について、次を満たしたらマッピングタスク合格。

1. **未開拓を残さず開拓できている**
   探索済み範囲の bounding box 内に unknown が大きく残っていない。到達可能な frontier を回り尽くし、
   自由空間の最大連結成分が大半を占める。

2. **幾何が正しい**
   円形影がない。壁が単一線で、二重・三重にぶれていない。斜めノイズが目立たない。寸法が実 world と
   一致する。例: `indoor.wbt` は床 5 x 10 m の矩形として見える。

3. **world 定義と地図を照合できている**
   地図内部の連結性だけで合格にしない。`wbt` が定義する Floor、Wall の `translation`/`size`、
   家具・人の位置と、地図上の occupied/free 配置・寸法を照合する。

4. **次タスクに渡せる保存物になっている**
   `maps/<map_name>.yaml` が保存され、PGM 画像への相対パス、`resolution`、`origin` が正しい。
   ウェイポイント生成はこの保存地図を入力にする。

## 必須制約

- 対象 world は屋内のみ: `indoor.wbt` / `break_room.wbt` / `cafe.wbt`。屋外 world は
  [`mapping_outdoor.md`](mapping_outdoor.md) を参照（現状未対応）。
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
python3 -u scripts/map_progress_monitor.py --interval 10 --duration 180

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

注: `eval_map_quality.py` の「壁率 / 最大連結成分 / 連結片数」は free 空間の連結性しか見ない。
**家具・人・壁などの占有マークが地図に正しく現れているか**は `check_map_vs_world.py` の重畳図
で目視確認する。

## 終了処理

Webots/RViz/Nav2 が残ると次回検証に混ざる。検証後は落とす。

```bash
ps aux | grep -E "webots|rviz|component_container|ros2 launch susumu|driver|pointcloud|frontier|slam|nav2|spawner" \
  | grep -v grep | awk '{print $2}' | xargs -r kill -9
```

## 関連

- [マッピング（屋外）](mapping_outdoor.md)
- [Webots シミュレーション環境ガイド](../webots_simulation.md)
- [world 一覧と使い分け](../worlds.md)
- [ロボット / LiDAR 構成](../robot_lidar.md)
- [MID-360 LiDAR 調査・Webots マッピングの罠](../mid360_lidar_research.md)
