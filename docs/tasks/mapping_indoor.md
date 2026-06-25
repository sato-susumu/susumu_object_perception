# マッピングタスク（屋内） — 屋内 world の自律地図作成

このページは README のタスク一覧「マッピング（屋内）」の詳細ページ。事前地図のない屋内 Webots
world を frontier 探索で走らせ、`slam_toolbox` が作る 2D OccupancyGrid を
`outputs/mapping_*/<name>.pgm/.yaml` として保存するところまでを扱う。

屋外マッピングは別タスク。[`mapping_outdoor.md`](mapping_outdoor.md) を参照（現状は
**「特徴の少ない広域屋外世界は未対応」** として保留）。

ウェイポイント生成・巡回ナビは別タスク。ここでの合格対象は**地図そのものの品質**だけ。

## 入出力

| 項目 | 内容 |
|---|---|
| 入力 | `webots_worlds/<world>.wbt`、3D LiDAR 由来の `/scan`、SLAM/Nav2 |
| 実行 | `launch/webots_indoor_mapping.launch.py` |
| 出力（最終） | `outputs/mapping_indoor/<map_name>.pgm`、`outputs/mapping_indoor/<map_name>.yaml`、`outputs/mapping_indoor/<map_name>_vs_world.{png,json}`（契約名・git 追跡。`_vs_world` は save_map 完了直後に `check_map_vs_world.py` が自動生成する world 真値との重ね合わせ＋アライメント JSON） |
| 出力（中間） | `experiments/mapping_indoor/<YYYY-MM-DD>_<label>/`（PGM 試作版、map_progress 等の cycle ログ。gitignore）、`experiments/mapping_indoor/legacy/` （旧採用版で対応 wbt が無いもの、house.yaml 等） |
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

探索完了時に `save_map:=True` なら `outputs/mapping_indoor/<map_name>.pgm/.yaml` が自動保存される。手動保存する場合:

```bash
ros2 run nav2_map_server map_saver_cli \
  -f ~/ros2_ws/src/susumu_object_perception/outputs/mapping_indoor/<map_name> \
  --ros-args \
  -p save_map_timeout:=20.0 \
  -p map_subscribe_transient_local:=true
```

`frontier_explore_node.py` の自動保存も同じく `save_map_timeout=20s` と
`map_subscribe_transient_local=true` で `map_saver_cli` を呼び、CLI の終了コードと保存後の YAML/参照画像を
検査する。保存後は YAML だけでなく参照画像も揃っていることを確認する。`outputs/mapping_*/*.pgm` は今後すべて
git 追跡対象にする。YAML だけが残った状態だと Nav2、waypoint 生成、認識の map support/overlay が
後段で失敗するため、保存地図は `.yaml` と `.pgm` を必ずペアで扱う。確認用 `.png` は再生成可能なので
引き続き追跡しない。

```bash
cd ~/ros2_ws/src/susumu_object_perception
ros2 run susumu_object_perception validate_map_assets.py outputs/mapping_indoor/<map_name>.yaml
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
   `outputs/mapping_indoor/<map_name>.yaml` が保存され、PGM 画像への相対パス、`resolution`、`origin` が正しい。
   `validate_map_assets.py outputs/mapping_indoor/<map_name>.yaml` が `OK` になる。ウェイポイント生成はこの保存地図を入力にする。

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
ros2 run susumu_object_perception eval_map_quality.py outputs/mapping_indoor/<map_name>.yaml

# 保存後: YAML が参照する PGM/PNG が実在することを確認
ros2 run susumu_object_perception validate_map_assets.py outputs/mapping_indoor/<map_name>.yaml

# 保存後: wbt の真値構造と重ねて確認
ros2 run susumu_object_perception check_map_vs_world.py \
  --map outputs/mapping_indoor/<map_name>.yaml \
  --wbt webots_worlds/<world>.wbt \
  --out experiments/mapping_indoor/<map_name>_vs_world.png \
  --report experiments/mapping_indoor/<map_name>_vs_world.json
```

数値だけでなく、`check_map_vs_world.py` の重畳図と RViz で目視確認する。移動量が十分あるのに
地図が広がらない場合は SLAM/環境を疑う。移動量が少ない場合は探索ゴール選択や Nav2 を疑う。

注: `eval_map_quality.py` の「壁率 / 最大連結成分 / 連結片数」は free 空間の連結性しか見ない。
**家具・人・壁などの占有マークが地図に正しく現れているか**は `check_map_vs_world.py` の重畳図
で目視確認する。

## 直近の保存結果

| 日付 | world / map | 条件 | 結果 |
|---|---|---|---|
| 2026-06-21 | `indoor.wbt` → `outputs/mapping_indoor/indoor.yaml/.pgm` | `mode:=realtime`, SLAM `/map` を `map_saver_cli` で保存 | `validate_map_assets.py` OK。`eval_map_quality.py`: `99x201`, `5.0x10.1m`, 壁率 `2.6%`, 最大連結成分 `99%`, 判定 `OK(微小片あり)` |
| 2026-06-21 | `break_room.wbt` → `outputs/mapping_indoor/break_room.yaml/.pgm` | `mode:=realtime`, SLAM `/map` を `map_saver_cli` で保存 | `validate_map_assets.py` OK。`eval_map_quality.py`: `188x140`, `9.4x7.0m`, 壁率 `2.3%`, 最大連結成分 `100%`, 判定 `OK`。`check_map_vs_world.py`: wall `near_ratio_inside=0.848`, obstacle `0.750`。**ただし unknown 55% で world 7.7x12.86m の半分しか地図化できておらず、6/25 に再マッピングで置換** |
| 2026-06-25 | `break_room.wbt` → `outputs/mapping_indoor/break_room.yaml/.pgm` | `mode:=realtime`, 改善 launch (`done_frontier_cells=5`, `done_after_empty=20`, `min_frontier_cells=2`, `approach_setback=1.0`, `stall_timeout_sec=180`) | `eval_map_quality.py`: `259x157`, `13.0x7.9m`, 壁率 `11.1%`, 最大連結成分 `100%`, **unknown 22% (旧 55% から大幅改善)**, 判定 `OK`。`check_map_vs_world.py`: wall `inside=17/17`, obstacle `inside=8/8` (旧 10/17, 4/8 から全数到達)。waypoint 13→38 個、カバー領域 17→44 m² |

次は地図品質が崩れた場合に、衝突ログ・`/scan`・SLAM 設定のどこで占有が欠けたかを切り分ける。
また、`break_room` の改善 launch 既定値 (上記 2026-06-25 条件) は `indoor.wbt` でも同じ
値を使えば「より粘り強い探索」が効くため、屋内 launch の既定値として採用済み。

## Roadmap-Explorer 比較検証（2026-06-24）

ROSCon 2025 で発表された **[Roadmap-Explorer](https://github.com/suchetanrs/roadmap-explorer)**
（`suchetanrs/roadmap-explorer` v1.0.0, Apache-2.0）で地図品質を改善できるか、自作
`frontier_explore_node` と同一の slam_toolbox / Nav2 基盤の上で比較した。**検証で結論が出たため、
Roadmap-Explorer 本体（別 workspace `~/exploration_ws`）と本リポの検証一式
（`experiments/roadmap_explorer/` の launch / params / 計測ノード）は削除済み**。本ページが結論の
正本で、再現手順の要点は下記「統合で必須だった調整・リスク」「実測比較」に残してある。
再検証する場合は `git clone https://github.com/suchetanrs/roadmap-explorer.git` から始める。

**Roadmap-Explorer とは**: frontier クラスタからナビゲーション・ロードマップ（グラフ）を構築し、
時間制限付き TSP で「globally coherent な探索順序」を解く frontier 探索。Nav2 lifecycle + BT
プラグイン構成で `navigate_to_pose` を使う。著者主張は greedy な frontier 系より 25–45% 速い。
入力は `/map`(OccupancyGrid)・フレーム `map`/`base_link` で本リポの slam_toolbox 構成と整合。

### 結果（自作 frontier との比較、いずれも `mode:=realtime` / CycloneDDS）

| world | 手法 | 寸法[m] | 壁率 | 最大連結成分 | 連結片 | unknown | world照合(wall / obstacle) |
|---|---|---|---|---|---|---|---|
| indoor | 自作 frontier | 5.0x10.1 | 2.6% | 99% | 2 | 0 | —（既存基準） |
| indoor | **Roadmap-Explorer** | 5.0x10.0 | 0.9% | 99% | 2 | 0 | wall 0.833 / obstacle 0.75 |
| break_room | 自作 frontier | 9.4x7.0 | 2.3% | 100% | 1 | 0 | wall 0.848 / obstacle 0.750 |
| break_room | **Roadmap-Explorer** | 12.9x7.9 | 4.4% | 97% | 2 | 0 | wall 0.782 / obstacle 0.875 |
| outdoor(参考) | **Roadmap-Explorer** | 7.8x6.4 | 0.6% | 100% | 1 | 0 | obstacle 1.0（地図化できた範囲のみ） |

- **屋内では自作 frontier と同等品質の地図が作れた**。indoor は壁が単一線で二重化せず（壁率 0.9%）、
  寸法も実 world 一致。break_room はむしろ自作より**広く開拓**できた（自作 9.4x7.0m に対し
  12.9x7.9m で実寸 12.86m に近い）し、**家具をより多く検出**（obstacle 0.875 > 自作 0.750）。
- 自作 frontier も既に屋内で合格水準（壁率 2–3%・連結 99–100%）に達しているため、
  **屋内で「地図品質が劇的に改善する」ほどの差は出なかった**（探索効率＝速さの差は別途）。

### 統合で必須だった調整・リスク

1. **内部 costmap の `inflation_radius` を 0.10 → 0.55 にするのが必須**。上流既定 0.10 は
   waffle 内接半径 0.225 より小さく、狭い屋内でフロンティアゴールが壁際に置かれ
   `Could not compute path from <現在地>` が多発→スタック→その場旋回で slam match が乱れ
   壁が二重化（indoor 壁率 2.6%→**8.3%**、連結片 2→4 に劣化）。0.55 で経路失敗 0 件・壁率 0.9% に回復。
2. **`robot_radius` を実行時に 0.10 へ強制上書きされる**。Roadmap-Explorer は探索開始時に
   `/global_costmap` の `robot_radius` を 0.10、`/planner_server` の `GridBased.allow_unknown` を
   true に**動的書換**する（`ExplorationBT.cpp` の "workaround" コメント）。本リポの安全設計
   （`robot_radius: 0.22` で壁から離す）を上書きするので、人のいる環境や狭所では衝突リスクが
   上がりうる。導入するならここの評価が前提。
3. **CycloneDDS 必須**（上流既知問題: FastDDS で costmap2D ノードが segfault。本リポの SHM 破損の罠とも整合）。
4. **アクション起動が必要**（lifecycle active 後に `Explore` アクションを叩く）。本 launch は自動化済み。

### チューニングで現行 frontier より良くなるか（2026-06-24 追加分析）

「Roadmap-Explorer をチューニングすれば屋内で現行 frontier より良くなるか」を、追加の実機検証は
せず今回のログ + コード読解で見極めた。**まず前提の切り分けが重要**:

- **地図の幾何品質（壁の鮮明さ・寸法精度・連結性）を決めているのは slam_toolbox であって探索
  アルゴリズムではない**。自作 frontier と Roadmap-Explorer は**同じ slam_toolbox の `/map` を共有**
  する（`config/nav2_params_webots_explore.yaml` の slam_toolbox 設定が地図生成の本体）。探索側が
  品質に効くのは間接経路だけ: ①動きの滑らかさ（急旋回→scan match 悪化）②カバレッジ（回り尽くすか）
  ③完走時間（短いほど odom ドリフト累積が少ない）。

軸ごとの伸びしろ:

| 軸 | 現状の余地 | 根拠 |
|---|---|---|
| ①動きの滑らかさ | **ほぼ無し** | ログ解析で Roadmap-Explorer は controller へ 4Hz 一定供給・1 秒以上の停止 0・nav2 recovery(spin/backup) 0・スタックは indoor で 3 回だけ（各 0.3–0.5s で自己回復）。既に「品質を下げる動き」をしていない |
| ②カバレッジ完全性 | **少し有り** | break_room で右側に unknown が残った。Roadmap-Explorer は `increment_search_distance_by`(35m)で探索半径を段階拡大し、`FullPathOptimizer` がローカル枯渇時に "Global repositioning" で取り残しを系統的に潰す機構を持つ。`goal_hysteresis_threshold`(0.15)・`min_frontier_cluster_size`(1.0)・`information_gain` 系の調整で「取り残しゼロ」を詰める余地はある |
| ③完走時間 | **未計測（本来の売り）** | 著者主張は greedy frontier 比 25–45% 速い。TSP ロードマップで往復を減らす。今回は完走時間を測っていないので**速度で勝てるかは本検証では未確定** |

**見立て**: 屋内の**幾何品質**は slam_toolbox が上限を決めており、現行 frontier も Roadmap-Explorer
も既にその上限近く（壁率 2–3%・連結 99–100%）。**探索側をチューニングしても品質は頭打ちが先に来る**
ので、「品質を上げる」目的での Roadmap-Explorer チューニングは費用対効果が低い。伸ばせるのは
**カバレッジ完全性（取り残しゼロ）と完走速度**で、これは Roadmap-Explorer の設計上の強みと一致する。
逆に**品質上限そのものを上げたいなら slam_toolbox 側（loop closure / scan match / resolution）を
触るのが本筋**で、これは探索アルゴリズムとは独立な軸。

### カバレッジ完全性・完走時間の実測比較（2026-06-24、indoor.wbt）

前節で「未計測」としていた**完走速度とカバレッジ完全性を実測**した。公平を期すため、
`/map` の unknown 率を一定間隔で記録する計測ノードを launch と同時起動し、**両手法とも
「計測ノードが最初に `/map` を受けた時刻」を共通の `elapsed=0`** にして unknown 率の時系列を
取った（同じ slam_toolbox/Nav2 基盤、同じ起点）。
カバレッジ調整版 params（`min_frontier_cluster_size` 0.5・`closeness_rejection_threshold` 0.25・
local 密度↑、`..._coverage.yaml`）も測った。

| 手法 | 開拓開始→飽和の**正味探索時間** | 飽和時 unknown率※ | occ(壁セル) | path 計画失敗 | 飽和時 幾何(壁率/連結/片) |
|---|---|---|---|---|---|
| 自作 frontier | **60.0s**(23.5→83.5s) | 14.72% | 1446 | — | 7.2% / 93% / 6 |
| Roadmap-Explorer baseline | 65.0s(33.4→98.4s) | 12.14%（最終） | 1588 | 110 | 7.8% / 94% / 5 |
| Roadmap-Explorer coverage版 | 70.0s(33.5→103.5s) | 12.76%（最終） | 1485 | **847** | 7.3% / 92% / 5 |

※ unknown 率は地図 bounding box 基準（外周の到達不能セルを含む参考値）。**室内の実カバレッジは
3 手法とも完全**: `check_map_vs_world.py` で家具 obstacle ratio=1.0（4/4 検出）、wall ratio は
自作 1.0 / RE baseline 1.0 / RE coverage **0.875**（coverage 版だけ壁照合が低下）。目視でも
3 手法とも外周壁・什器・家具が正しく、室内 unknown はほぼ無い。

**実測で分かったこと（事前の見立てと違った点）:**

- **完走速度は Roadmap-Explorer が速くなかった。むしろ自作 frontier が最速（正味 60s）**。
  著者主張「greedy frontier 比 25–45% 速い」は、この小規模屋内（5x10m）では再現しなかった。
  TSP ロードマップの利点は広い環境で往復を減らす点にあり、狭い indoor では探索が短く、
  自作のシンプルな nearest+情報利得法と差がつかない（むしろ機敏な分わずかに速い）。
- **カバレッジ完全性は Roadmap-Explorer がわずかに上**（bounding box unknown 12.1% < 14.7%、
  壁セル occ も 1588 > 1446）。ただし**室内の実カバレッジは 3 手法とも完全**で、差は外周の
  到達不能セルの扱いの違い。実用上の優劣はほぼ無い。
- **カバレッジ調整版は逆効果だった**。`min_cluster`/`closeness` を下げて細かい取り残しを拾わせたら、
  到達不能な微小フロンティアへの無駄打ちが激増（path 計画失敗 110→**847**）し、最終カバレッジは
  改善せず（12.76% ≧ baseline 12.14%）、壁照合はむしろ悪化（wall 1.0→0.875）。
  **indoor は baseline params が既にカバレッジ最適に近く、調整の余地は小さい**。
- **自作 frontier は「厳密な完走判定」が機能しない**。到達不能な縁フロンティア（130 セル級）が残ると
  `done_after_empty` がリセットされ続けて `exploration complete` に至らず、別ランでは 400s 走っても
  完了しなかった。さらに**長く走らせると品質が劣化**（再試行でその場旋回が増え、壁率 2.6%→9.2%・
  連結片 2→6 に悪化）。これは「カバレッジは早期に飽和するが完了を宣言できない」設計上の弱点。

**結論（カバレッジ完全性と完走時間について）:**
- **カバレッジ完全性**: 室内実カバレッジは元から 3 手法とも完全。今回のパラメータ調整での「向上」は
  確認できず、むしろ無駄打ちが増えた。**indoor で取り残しを減らす余地はほぼ無い**（残る unknown は
  外周や家具裏の物理的到達不能領域）。カバレッジを実質的に上げるには探索 params ではなく、
  到達不能領域を許容する完了判定や、別経路から覗ける視点計画が要る（費用対効果は低い）。
- **完走時間**: indoor では自作 frontier が最速（60s）。Roadmap-Explorer は遅くはないが速くもない
  （65–70s）。**「Roadmap-Explorer に変えれば速くなる」は小規模屋内では成り立たない**。
  ただし自作 frontier の完了判定の弱点（完走宣言できず・長走で劣化）は実運用で問題になりうるので、
  自作側を直すなら「到達不能フロンティアの早期確定」と「カバレッジ飽和での自動停止」が要点。

### 判断

- **屋内マッピングの「地図品質」改善目的では、Roadmap-Explorer に乗り換える積極的な理由は薄い**。
  自作 frontier が既に屋内で合格水準に達しており、品質は同等。むしろ `robot_radius` 強制上書き
  （安全設計の上書き）と、別 workspace の C++ 依存（pcl_ros/nav2_behavior_tree 等）・実行時
  パラメータ書換という運用コスト／リスクが増える。
- ただし **探索効率（完走時間）・大規模屋内**では TSP ロードマップの利点が出る可能性がある。
- **屋外（`outdoor.wbt`, 床 20x20m）は参考検証した結果、Roadmap-Explorer でも SLAM の限界が支配的**
  だった。探索アルゴリズム自体は破綻せず動いた（経路失敗 0・TF ドリフト 0）が、開放空間で LiDAR が
  捉える壁面特徴が乏しく、**20x20m のうち 7.8x6.4m しか地図化できず**（ロボットも原点付近から
  ほぼ出られない）、`SimpleBuilding` 2 棟は地図に現れなかった。これは Roadmap-Explorer 固有の問題
  ではなく、frontier+SLAM という枠組み全体の原理的限界（自作 frontier も同じ場所で同じ限界に当たる）。
  → 屋外は [`mapping_outdoor.md`](mapping_outdoor.md)（現状未対応）。**探索アルゴリズムの差し替えでは
  屋外マッピングは解決しない**（SLAM／センサ側の手当てが先）。
- 結論: **屋内品質改善のための導入は見送り**。実測でも完走時間は自作 frontier が最速で、品質・
  カバレッジも同等だった（上記「実測比較」）。**検証で結論が出たため Roadmap-Explorer 本体と検証一式は
  削除済み**で、本ページに数値・結論・再現の要点を記録として残す。将来「広域屋内の効率探索」が課題に
  なったら、本ページの記録を起点に再 clone して再検証する。

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
