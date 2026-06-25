# ウェイポイント生成タスク — 保存地図から巡回点列を作る

このページは README のタスク一覧「ウェイポイント生成」の詳細ページ。マッピングタスクで合格した
保存地図から、Nav2 で巡回しやすいウェイポイント YAML と確認用 PNG をオフライン生成する。

## 入出力

| 項目 | 内容 |
|---|---|
| 入力 | `maps/<world>.yaml` と対応する PGM |
| 実行 | `scripts/generate_waypoints.py`（`ros2 run susumu_object_perception generate_waypoints.py`） |
| 出力 | `maps/<world>_waypoints.yaml`、`maps/<world>_waypoints.png` |
| 次タスク | [巡回ナビ](waypoint_navigation.md) |

## 実行

```bash
cd ~/ros2_ws/src/susumu_object_perception
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash

ros2 run susumu_object_perception generate_waypoints.py \
  --map maps/city.yaml \
  --out maps/city_waypoints.yaml \
  --spacing 1.5 \
  --clearance 0.4 \
  --connect-clearance 0.30
```

出力 YAML は map 座標の点列。PNG は地図上にウェイポイント番号、巡回経路、配置可能領域、
連結対象領域を重ねた確認用画像。

## 生成アルゴリズム

`generate_waypoints.py` は「なるべく広く回る」と「Nav2 で完走できる」を両立させるため、次の順で処理する。

1. PGM/YAML を読み、free / occupied / unknown を分類する。
2. occupied と unknown からの距離変換を作る。
3. **連結用 clearance**（既定 `0.30m`）で通れる領域を作り、必要なら **経路用 clearance**
   （`--route-clearance`）で絞った最大連結成分だけを巡回対象にする。
4. **配置用 clearance**（既定 `0.60m`、例では `0.40m`）を満たすセルだけから、`spacing` グリッドで候補点を作る。
5. 点間距離は直線距離ではなく、連結成分上の**測地距離**を使う。
6. 必要なら `--edge-clearance` / `--edge-clearance-weight` で、通行可能だが obstacle / unknown に近い
   edge を route graph の順序として選びにくくする。
7. 最近傍法 + 2-opt で巡回順を作り、YAML/PNG を保存する。`--edge-risk-report` を指定すると、
   edge ごとの clearance risk CSV/JSON/Markdown も保存する。

連結用と配置用の clearance を分けるのが重要。連結判定まで厳しくすると、ドアや家具の隙間で部屋が
分断される。配置判定は壁から離したいので、連結判定より厳しくする。
屋外ではさらに `route-clearance` と `edge-clearance` を分けられる。`route-clearance` は通行可否の
二値判定で、`edge-clearance` は「通れるが inflation 近傍で危ない」edge を避ける soft cost。
ただし過去の live 評価では edge-clearance weighted waypoint は未採用なので、
屋外 wrapper の既定値にはしていない。
屋外の sparse route を後処理で細かい中間goal列に変換する実験には
`scripts/expand_waypoint_route.py` を使う。これは `generate_outdoor_waypoints.py` の出力順を保ったまま、
保存地図の route passable 最短経路に沿って各 edge を `--max-segment-length` 以下へ展開する。
過去評価では 2.0m 版で path tracking error は改善したが、危険 corridor を忠実に辿って
`pose_global_lethal_static_free` が増えたため、候補生成・診断用に留め、屋外本線 waypoint には採用しない。

## SWAGGER-lite 疎ウェイポイント生成（2026-06-25）

巡回ルールをシンプルにしつつ確実に巡回するため、SWAGGER（Sparse WAypoint Graph Generation for
Efficient Routing）の「自由空間の local maxima に疎なノードを置く」考えを取り入れた
`--candidate-mode sparse_graph` を追加した。**アイデアの出典は SWAGGER だが、SWAGGER 本体のコード・
cuCIM・CUDA・scikit-image には依存せず、scipy.ndimage + numpy だけで考え方のみ移植**している
（詳細・出典は `generate_waypoints.py` の `_sparse_graph_candidates` docstring）。

- `--candidate-mode grid|sparse_graph`: grid は従来（spacing 格子代表点、地図面積に比例して点が増える）。
  sparse_graph は距離変換の局所最大 + NMS で疎ノードを作り、点数が通路・部屋・分岐の複雑さに近づく
  （indoor で grid 27 点 → sparse の骨格 5〜6 点）。
- `--graph-cover-radius`: 局所最大だけだと部屋の壁際が手薄になるので、既存ノードからこの距離を超えて
  手薄な place 領域に点を足す（SWAGGER の free-space sampling 相当）。indoor で coverage max 2.44m→1.60m。
- `--relink-long-jumps`: NN+2-opt の順序で什器を大迂回する長い区間を、既存ウェイポイントを経由して
  短いジャンプの連続に置き換える（重複を許す）。indoor で最大測地ジャンプ 6.1m→2.0m。
- 検査ツール `scripts/check_waypoints.py`: 各点の clearance と placeable 自由空間のカバレッジ偏りを
  数値判定（壁に近い点 0 か、3m 超手薄 0 か）。`reached=N/N` だけでは見えない配置品質を客観確認する。

採用生成例（indoor）:

```bash
ros2 run susumu_object_perception generate_waypoints.py \
  --map maps/indoor.yaml --out maps/indoor_sparse_waypoints.yaml \
  --candidate-mode sparse_graph --spacing 1.5 --graph-cover-radius 2.0 \
  --clearance 0.6 --connect-clearance 0.30 --relink-long-jumps 3.0 --max-segment-length 2.0
ros2 run susumu_object_perception check_waypoints.py \
  --map maps/indoor.yaml --waypoints maps/indoor_sparse_waypoints.yaml --clearance 0.6
```

### 巡回状況の後追い確認（Nav2 feedback + 可視化）

「その通り巡回できているか」を `reached=N/N` だけで判断せず、後から客観確認できるようにした。

- `waypoint_nav_node.py` が Nav2 の feedback（`number_of_recoveries`・`distance_remaining`）を各
  ウェイポイント記録に残す（`nav_recoveries` / `nav_distance_remaining_m`）。スキップ時も「なぜ
  詰まったか（recovery 多発・ABORTED）」が JSON/CSV に残る。`report_prefix:=...` で出力。
- `scripts/visualize_patrol_result.py`: 巡回結果 JSON と地図から、各点を緑（reached）/黄（reached だが
  recovery 多発で苦戦）/赤（missed）で色分けした PNG を作る。巡回後 1 枚で「どこで苦戦・スキップしたか」
  が分かる。

この可視化で「reached=16/18 でも実は 9 点が recovery 多発で苦戦」という、点数だけでは見えない問題を
検出できた。さらに **苦戦の真因が waypoint 配置でなく SLAM の `map→odom` TF 遅延**
（`Transform data too old`×908 回 → controller が姿勢変換失敗 → recovery → ABORTED）だと切り分けられた。
巡回専用 `config/nav2_params_webots_patrol.yaml` で local_costmap の `transform_tolerance` を
0.2→0.5 に緩めたところ、`Transform data too old` 908→249 回、**reached 16/18→18/18・苦戦 9→6 点**に改善
（完走）。`transform_tolerance` を変えたら [`nav2_tuning.md`](../nav2_tuning.md) も更新する。

## 合格基準

1. **入力地図がマッピングタスク合格済み**
   幾何が崩れた地図からウェイポイントを作らない。地図不良をウェイポイント側で補正しない。

2. **巡回対象が到達可能な最大連結成分に乗っている**
   PNG 上でウェイポイントが壁・unknown・別連結成分に置かれていない。経路線が壁を直線で跨いで見えても、
   測地順としては通行可能領域上の近い点同士になっていること。

3. **地図全体を偏りなくカバーしている**
   到達可能な主要空間に点が配置され、端や別室が抜けていない。点が少なすぎる場合は `spacing` を下げる。
   点が多すぎて巡回が長すぎる場合は `spacing` を上げる。

4. **Nav2 で完走できる点列になっている**
   生成ログの最大測地ジャンプが過大でない。通常は `spacing` 近傍、広くても数 m 程度に収まる。
   壁越しの大ジャンプが出る場合は地図の連結性、`connect-clearance`、`clearance` を見直す。

5. **成果物が追跡可能**
   `<world>_waypoints.yaml` と `<world>_waypoints.png` を同時に残す。PNG はレビュー時に最初に見る成果物。

## 調整指針

| 症状 | 主に見る値 | 対処 |
|---|---|---|
| 点が壁に近く、巡回中にこすりやすい | `--clearance` | 上げる |
| 部屋や通路が別連結成分になり、点が置かれない | `--connect-clearance` | 下げる |
| 屋外で inflation 近傍の経路を二値的に除外したい | `--route-clearance` | 屋外実験用。上げると安全側だが、連結成分が分断される |
| 屋外で狭い corridor を巡回順として選びにくくしたい | `--edge-clearance` / `--edge-clearance-weight` | 実験用。offline risk は下がるが live では本線未採用 |
| 危険 edge の根拠を残したい | `--edge-risk-report` | CSV/JSON/Markdown に edge ごとの min clearance と shortfall を保存 |
| sparse route を中間goal列へ展開してpath追従性を診断したい | `expand_waypoint_route.py` | 屋外実験用。live では本線未採用 |
| 点が少なく巡回範囲が粗い | `--spacing` | 下げる |
| 点が多く一周が長すぎる | `--spacing` | 上げる |
| 壁越しのような大ジャンプがある | 地図品質、`--connect-clearance` | 地図を確認し、必要なら再マッピング |
| 認識対象の近くを通らない | `--object-viewpoints` / `--view-clearance` | 実験用。通常巡回点は維持し、occupied 小〜中サイズ成分を見る追加視点を入れる |

### 屋外 edge clearance cost（実験用）

`--edge-clearance-weight` は既定 `0.0` で従来互換。正の値を指定すると、距離変換上で
`--edge-clearance` 未満のセルに soft penalty を掛け、NN + 2-opt の距離行列に使う。
これは waypoint の候補位置や通行可否を変えず、巡回順だけを「広い corridor を通る edge」へ寄せる。

過去の屋外評価では、`edge_clearance=0.75m`, `edge_clearance_weight=8.0` により offline の
clearance shortfall は大きく下がった。しかし live smoke では到達数が既定より悪化し、実軌跡が
global inflation / static occupied へ入る問題は残った。そのため診断用・候補生成用として残し、
屋外の既定にはしていない。

参考: Nav2 Costmap 2D <https://docs.nav2.org/configuration/packages/configuring-costmaps.html>、
Inflation Layer <https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html>。

### 屋外 route expansion（実験用）

`expand_waypoint_route.py` は生成済み waypoint YAML を入力に取り、保存地図上の geodesic path に沿って
中間goalを追加する後処理ツール。巡回順の決定は `generate_outdoor_waypoints.py` に任せ、
実行時に Nav2 へ渡すpose列だけを密にする。

```bash
ros2 run susumu_object_perception expand_waypoint_route.py \
  --map maps/village_square_trimmed_glim2d.yaml \
  --waypoints maps/village_square_trimmed_glim2d_waypoints.yaml \
  --out maps/village_square_trimmed_glim2d_expanded_2m_waypoints.yaml \
  --max-segment-length 2.0 \
  --connect-clearance 0.35 \
  --route-clearance 0.35 \
  --clearance 0.75 \
  --limit-radius 14.0 \
  --report-prefix maps/village_square_trimmed_glim2d_expanded_2m_report
```

履歴サマリ: 2.0m 展開は path tracking error を改善したが、保存地図上の geodesic path が
Nav2 inflation / local obstacle に対して安全とは限らず、lethal 近傍へ入りやすかった。
route expansion は controller 診断には有効だが、展開結果をそのまま採用 waypoint にしない。
次は live の high-cost / lethal event を使って危険 corridor blacklist と組み合わせる。

### 認識巡回向けの追加視点

`--object-viewpoints` は既定 OFF の実験用オプション。指定すると、保存地図上の occupied 小〜中サイズ成分を
world 真値なしで拾い、その成分に近い安全セルを追加ウェイポイントにする。通常の spacing 代表点は
壁から遠い通路中央を選ぶため、家具・植物・冷蔵庫の前を十分近く通らない場合に使う。
map 境界の壁や地図端の小片を拾うと Nav2 が `worldToMap failed` になるため、`--view-map-border-margin`
で境界成分を除外する。
`--view-clearance` を省略した場合は `--clearance` と同じ値を使う。Nav2 の costmap は robot radius /
footprint で衝突判定し、inflation layer が障害物周辺の近接コストを作るため、認識用追加視点だけ
通常ウェイポイントより狭い clearance に置くと、オフライン測地距離では接続していても実走で詰まる。
参考: Nav2 Costmap 2D <https://docs.nav2.org/configuration/packages/configuring-costmaps.html>、
Inflation Layer <https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html>、
Tuning Guide の Robot Footprint vs Radius
<https://docs.nav2.org/tuning/index.html#robot-footprint-vs-radius>。

屋内認識用の採用例:

```bash
ros2 run susumu_object_perception generate_waypoints.py \
  --map maps/indoor.yaml \
  --out maps/indoor_recognition_waypoints.yaml \
  --spacing 1.5 \
  --clearance 0.6 \
  --connect-clearance 0.30 \
  --object-viewpoints 8 \
  --object-max-area 0.40 \
  --view-map-border-margin 0.25
```

履歴サマリ: `indoor_recognition_waypoints.yaml` は `view_clearance=0.6m` でナビ完走できるが、
通常巡回より認識 F1 が悪化した。object-viewpoints は到達性改善の実装だけ残し、認識成果物の
採用条件にはしない。

## 関連

- [マッピング（屋内）タスク](mapping_indoor.md)
- [巡回ナビタスク](waypoint_navigation.md)
- [launch 一覧](../launch.md)
