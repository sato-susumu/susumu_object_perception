# マッピングタスク（屋外） — 屋外 world の自律地図作成（未対応）

このページは README のタスク一覧「マッピング（屋外）」の詳細ページ。屋内マッピングは別タスク
で[`mapping_indoor.md`](mapping_indoor.md) を参照。

## 現状: 特徴の少ない広域屋外世界は未対応

このパッケージの屋外マッピング（`outdoor.wbt` / `city_robot.wbt` のような 20m 級開放空間 +
建物 2 棟・植木数本程度の world）は **未対応**。

| world | サイズ | 特徴量 | 対応 |
|---|---|---|---|
| `outdoor.wbt` | 20 x 20m | PottedTree x4, SimpleBuilding x2, 車 x1 | ❌ 未対応 |
| `city_robot.wbt` | 20 x 20m | outdoor + Pedestrian | ❌ 未対応 |

理由は次節「未対応の理由」のとおり、`/scan` 変換（pointcloud_to_laserscan）と
`slam_toolbox / Karto` の挙動が「特徴の疎な広域空間」を地図化する設計になっていないため。

将来「もっと特徴の多い屋外世界」（建物多数・壁・フェンス・街灯などが密に配置された world）が
追加されれば対応の余地がある。今は対象 world を持たないため未対応扱い。

## 屋内と屋外は完全に別物として扱う（重要）

屋内マッピングとは設定もコードもタスクも完全に分離する。屋外を動かすために屋内設定を改変
することは禁止。詳細は [`mapping_indoor.md`](mapping_indoor.md#屋内と屋外は完全に別物として扱う重要)
を参照。

具体的には:

- `launch/webots_simulation.launch.py` の `pointcloud_to_laserscan` は屋内向け実績値で
  固定（min_height:0.1, max_height:2.0, range_max:40, use_inf:True）。屋外も拾えるようにする
  目的での調整は禁止。
- 屋外専用の設定が必要になったら、屋外専用 launch を別ファイルで用意するか、scan を
  屋外専用 topic 名（例: `/scan_outdoor`）に remap して slam_toolbox / nav2 にも別パスを
  作るなど、屋内に影響しない設計にする。
- `config/nav2_params_webots_explore_outdoor.yaml` は屋外向け slam_toolbox の
  `max_laser_range`、Smac planner、global costmap rolling window などの実験的な値を持つが、
  屋外を動かす実証ができるまで本番運用しない。

## 未対応の理由（試行錯誤の記録）

2026-06-20 に複数の設定を試した。いずれも片方しか満たせず、地図品質と SLAM 姿勢安定性の
両立に至らなかった。`Karto` のソース（`AddScan()`）を直接読んだ上での結論:

**Karto の挙動**:
1. `rangeReading >= scan.range_max` または `inf` の ray は完全に無視（free すら書かない）
2. `rangeReading >= RangeThreshold(slam の max_laser_range)` の ray は RangeThreshold で
   打ち切り、その点まで free として raytrace（端点は occupied としてマークしない）
3. `rangeReading < RangeThreshold` の hit ray は free raytrace + 端点を occupied としてマーク

**試した設定と症状**:

| 設定 | 症状 |
|---|---|
| (a) `use_inf:False, inf_epsilon:-0.5` で未ヒットを 15.5m の偽 hit にする | Karto がこれを (2) として処理し広い free 地図を出すが、別 ray の free 通過で建物 occupied が圧倒的に上書きされる → **地図全体が free（occ=0）になり建物・木・車が完全消失** |
| (b) `use_inf:True, max_height:5.0/10.0, range_max:20, max_laser_range:18` で実 hit を増やそうとする | 建物 occupied は残るが、未ヒット ray が完全無視されるため `/scan` の有効 hit が疎（723点中 77点等）になり、SLAM scan match の姿勢推定が不安定化 → **自己位置喪失** |
| (c) `use_inf:True, max_height:2.5, range_max:16, max_laser_range:15` で scan match 用 hit を増やす | 建物 occupied は残り、自己位置もある程度安定するが、地図が広域 world の全体を捉えきれない（外周まで届かず、屋内向け値そのままでは広域 raytrace ができない） |

つまり「広域空間の free 拡大」と「建物 occupied の保護」と「scan match に必要な十分な hit 数」
の 3 つを同時に満たす scan 仕様が無い。これは MID-360 の対称 ±30° FOV と特徴の疎な広域 world
の組み合わせに固有の問題。

## 取り組む場合の前提

将来このタスクに取り組む場合の前提を残す:

1. **屋内設定には絶対に影響させない**。屋外専用の `pointcloud_to_laserscan` ノードを別途立て、
   出力 topic を `/scan_outdoor` のように分け、屋外専用 slam_toolbox に渡す。
2. **より特徴の多い world で検証する**。Webots 公式の `village.wbt`、`city.wbt`
   (`/usr/local/webots/projects/vehicles/worlds/city.wbt`) のような、建物・歩道・街灯・
   フェンスが密に配置された world で検証する。`outdoor.wbt` / `city_robot.wbt` のような
   特徴疎な world では原理的に困難。
3. **SLAM アルゴリズムも見直す**。slam_toolbox(Karto) は屋内向け前提が強い。屋外開放空間
   では Cartographer の global SLAM、LIO-SAM、FAST-LIO 等のアルゴリズムも検討する。
4. **scan を 3D 系に切り替える**。`pointcloud_to_laserscan` で 2D に潰さず、
   3D 点群 SLAM（colorized_pointcloud_mapper の路線）にする方が、広域屋外に向く可能性。

## 関連

- [マッピング（屋内）](mapping_indoor.md)
- [Webots シミュレーション環境ガイド](../webots_simulation.md)
- [world 一覧と使い分け](../worlds.md)
- [ロボット / LiDAR 構成](../robot_lidar.md)
- [MID-360 LiDAR 調査・Webots マッピングの罠](../mid360_lidar_research.md)
