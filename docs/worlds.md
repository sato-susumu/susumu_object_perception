# world 一覧と使い分け

このページは README から分離した world 情報の置き場。Gazebo Classic 版と Webots 版で world の扱いが
違うため、起動引数と用途を分けて整理する。

## Gazebo Classic

`simulation.launch.py` は HuNavSim の world 生成器を使い、`hunav_gazebo_wrapper/worlds/<base_world>` に
歩行者エージェントを合成した `generatedWorld.world` を起動する。

| world | 用途 | 状態 |
|---|---|---|
| `cafe.world` | 既定。HuNavSim 歩行者 5 人が通常歩行速度で巡回しやすい | 推奨 |
| `house.world` | 過去の家 world。狭い通路・家具密集で歩行者が固着しやすい | 非推奨 |

起動例:

```bash
# 既定 cafe
ros2 launch susumu_object_perception simulation.launch.py

# house に戻す場合は map/base_world/configuration_file を揃える
ros2 launch susumu_object_perception simulation.launch.py \
  map:=/home/taro/ros2_ws/src/susumu_object_perception/maps/house.yaml \
  base_world:=house.world \
  configuration_file:=/home/taro/ros2_ws/src/susumu_object_perception/config/agents_house.yaml
```

HuNav の `robot_name` は spawn entity 名と同じ `turtlebot3` に揃える。

## Webots

Webots 系 launch は `webots_worlds/<world>.wbt` を直接読む。`world` 引数は拡張子込みで渡す。

| world | 用途 | 備考 |
|---|---|---|
| `indoor.wbt` | 屋内ナビ・マッピング | 床 5 x 10 m の矩形。地図寸法照合の基準にしやすい |
| `break_room.wbt` | 室内ナビ・衝突診断 | 4 方向バンパー付き。`collision_diagnostic_node.py` の確認に使う |
| `outdoor.wbt` | 屋外マッピング | 20 m 平地 + 建物/木。特徴が少ないので frontier/sweep の確認向け |
| `city_robot.wbt` | 車・歩行者・信号 + センサ付き TB3 | 認識・巡回・街環境の主対象 |
| `calibration.wbt` | 全天球カメラ/LiDAR 色付け検証 | 色パネル・箱・円柱などを置いた検証 world |
| `calibration_vlp16.wbt` | VLP-16 退避版の色付け検証 | `lidar_model:=vlp16` と合わせる |
| `indoor_vlp16.wbt` / `outdoor_vlp16.wbt` | VLP-16 退避版 | 通常は MID-360 版を使う |

マッピングタスクの対象は、実在する `indoor.wbt` / `break_room.wbt` / `outdoor.wbt` / `city_robot.wbt`。
`kitchen.wbt` / `home.wbt` は存在しない。過去の PGM が残っていても新規タスクの対象にしない。

## 共通制約

- マッピング品質を評価する Webots 実行は `mode:=realtime`。
- 全 `wbt` の Lidar `tiltAngle` は 0。非ゼロにしない。
- Webots 同梱 world にロボットを組み込む場合は `WorldInfo { basicTimeStep 20 }` を入れる。
- Webots の EXTERNPROTO は初回にネットワークから取得されることがある。初回起動時はネット接続を前提にする。
- `fast` は軽量確認用。地図・ナビ品質の最終判断は realtime で行う。

## 関連

- [マッピングタスク](tasks/mapping.md)
- [Webots シミュレーション環境ガイド](webots_simulation.md)
- [launch 一覧](launch.md)
