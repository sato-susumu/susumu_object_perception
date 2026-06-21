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
| `outdoor.wbt` | sparse outdoor の過去実験 | 20 m 平地 + 建物 2 棟 + 樹木 4 本。今回の屋外本線（特徴豊富な都市/公園）には合わない |
| `city_robot.wbt` | 車・歩行者・信号 + センサ付き TB3 | 認識・街環境の対象。マッピング本線ではなく、特徴豊富な屋外地図作成は trimmed world を使う |
| `village_center.wbt` | 参照元 / 広域実験 | Webots 公式 `village_center.wbt` をパッケージ取り込み（SumoInterface 削除、TurtleBot3 を (-18.31, +97.52) の床パッチ上に追加）。大きく重いため、通常の屋外本線は下の trimmed world を使う |
| `village_square_trimmed.wbt` | **屋外マッピング本線 1** | `village_center` の街区風に小さく切り出した 34m 四方 world。小建物・街灯・ベンチ・植栽・フェンスを配置し、四辺を通行止めバリアで閉じる。ロボット初期位置は `(0,0)` |
| `village_park_trimmed.wbt` | **屋外マッピング本線 2** | `village_center` の公園風に小さく切り出した 34m 四方 world。フェンス・Pergolas・ベンチ・植栽・planter を配置し、四辺を通行止めバリアで閉じる。ロボット初期位置は `(0,0)` |
| `calibration.wbt` | 全天球カメラ/LiDAR 色付け検証 | 色パネル・箱・円柱などを置いた検証 world |
| `calibration_vlp16.wbt` | VLP-16 退避版の色付け検証 | `lidar_model:=vlp16` と合わせる |
| `indoor_vlp16.wbt` / `outdoor_vlp16.wbt` | VLP-16 退避版 | 通常は MID-360 版を使う |

マッピングタスクの屋外本線は `village_square_trimmed.wbt` と `village_park_trimmed.wbt`。
`outdoor.wbt` / `city_robot.wbt` は特徴が少ない過去実験として扱う。`kitchen.wbt` / `home.wbt` は存在しない。
過去の PGM が残っていても新規タスクの対象にしない。

`maps/village_square_trimmed_gt.yaml` / `maps/village_park_trimmed_gt.yaml` は world 由来の正解データ。
地図作成そのものは `slam_toolbox` で行い、GT は作成後の照合・評価にだけ使う。

## 共通制約

- マッピング品質を評価する Webots 実行は `mode:=realtime`。
- 全 `wbt` の Lidar `tiltAngle` は 0。非ゼロにしない。
- Webots 同梱 world にロボットを組み込む場合は `WorldInfo { basicTimeStep 20 }` を入れる。
- Webots の EXTERNPROTO は初回にネットワークから取得されることがある。初回起動時はネット接続を前提にする。
- `fast` は軽量確認用。地図・ナビ品質の最終判断は realtime で行う。
- GPS が出る world では `evaluate_gps_vs_slam.py` で GPS 真値と `map`/`odom` TF の差分を残せる。

## 関連

- [マッピング（屋内）タスク](tasks/mapping_indoor.md) / [マッピング（屋外）タスク](tasks/mapping_outdoor.md)（屋外は未対応）
- [Webots シミュレーション環境ガイド](webots_simulation.md)
- [launch 一覧](launch.md)
