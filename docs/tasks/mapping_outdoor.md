# マッピングタスク（屋外） — 屋外 world の自律地図作成（実用未対応）

このページは README のタスク一覧「マッピング（屋外）」の詳細ページ。屋内マッピングは別タスク
で[`mapping_indoor.md`](mapping_indoor.md) を参照。

## 現状: 屋外マッピングは実用品質ではない

このパッケージの屋外マッピングは**実用品質に達していない**。実験的に動かせる構成
（`webots_outdoor_mapping.launch.py` + `explore_radius` 範囲制限）は用意したが、地図品質・
自己位置の正確性・wbt 整合性のいずれも本番運用には不足する。

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

### 不足している検証（実用品質と呼ぶには必要）

1. **wbt 配置との整合性が未確認**。`check_map_vs_world.py` は Floor / Wall / SimpleBuilding /
   PottedTree など限られた PROTO しか解析できず、village_center 固有の PicketFence・Cypress・
   StreetLight 等は対象外。「地図の occupied マークが wbt 配置と本当に一致しているか」は
   目視確認のみで定量検証されていない。
2. **自己位置の正確性が未確認**。GPS 真値（`/TurtleBot3Burger/gps`）と map 推定位置の比較を
   していない。屋内 indoor / break_room では realtime + 屋内設定で GPS 一致を確認済みだが、
   屋外では未取得。
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
3. **wbt 配置との定量整合を取れる照合スクリプトを書く**。`check_map_vs_world.py` を拡張し、
   PicketFence / Cypress / StreetLight などの PROTO も Floor 上に投影できるようにする。
4. **GPS 真値と map 推定位置の差分を測る**。屋内同様のドリフト検証を屋外でも実施する。
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
