# 認識タスク — 物体検出・分類・信号認識・予測

このページは README のタスク一覧「認識」のタスクページ。詳細なアルゴリズム、ノード構成、パラメータは
[認識タスク — LiDAR sensing/perception パイプライン](../autoware_perception.md) と
[信号認識](../traffic_light_recognition.md) に集約している。

## 入出力

| 項目 | 内容 |
|---|---|
| 入力 | 3D LiDAR 点群、全天球画像、2D `/map` |
| 実行 | `simulation.launch.py`、`webots_simulation.launch.py`、`webots_waypoint_nav.launch.py perception:=True omni_perception:=True image_recognition:=True` |
| 主な出力 | `/perception/tracked_objects`、`/perception/tracked_objects_classified`、`/perception/predicted_objects`、`/perception/predicted_costmap`、`/perception/traffic_signals`、RViz markers、`maps/<world>_recognition_overlay.png`、`maps/<world>_recognition_eval.{md,json,csv,png}` |
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

# 屋内認識評価向けの追加視点実験（現時点では採用条件にしない）
ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
  world:=indoor.wbt waypoints:=indoor_recognition_waypoints.yaml mode:=realtime \
  perception:=True omni_perception:=True image_recognition:=True indoor_objects:=True
```

CPU が厳しい場合は `image_recognition:=False` で YOLO 物体分類と全天球信号認識を切る。LiDAR perception は残る。
Webots 系 launch では `object_yolo_weights:=...` で `object_classifier_node.py` の YOLO weight を
差し替えられる。屋内認識の採用既定は `yolov8s-seg.pt`。`yolov8m-seg.pt` は 2026-06-20 の
フル巡回評価で悪化したため、比較実験用に留める（下の「追加改善5」）。
`object_yolo_imgsz:=...` で YOLO 推論画像サイズ、`object_crop_fovs_deg:=75,55,40` のように
複数クロップ画角も指定できる。ただし 2026-06-21 の屋内フル巡回では multi-FOV + imgsz 960 が
余分検出とラベル誤りを増やしたため、既定は従来通り `object_yolo_imgsz:=640` かつ
`object_crop_fovs_deg:=''`（単一 `crop_fov_deg`）にする。2026-06-21 の追加改善9で
multi-FOV 時の代表選択を信頼度最大から FOV 間 class 合意 + 中心/mask overlap スコアへ変更したが、
ライブ評価は採用値に届かなかったため、複数 FOV は引き続き controlled comparison 用に留める。
分類ゲートの調査時は `object_classifier_debug:=True` を付けると、
`/perception/object_classifier/debug` に YOLO 候補ごとの採否理由（bbox 面積、中心ずれ、mask overlap、
植物色比率など）が `diagnostic_msgs/DiagnosticArray` で出る。これは次の改善で、正解候補をどの
ゲートが落としているかを bag / echo で確認するための診断用で、既定は False。

### 最終成果物（地図への認識結果重畳）

認識タスクの最後に、`object_memory_node.py` が保存した SQLite DB を保存地図へ重ね、
物体 ID・ラベル・存在確率・観測回数付き PNG を作る。地図が小さくラベル領域が足りない場合、
`render_recognition_overlay.py` は既定で地図を読みやすい大きさに自動拡大する。必要なら
`--scale` で倍率を明示する。
描画や map support フィルタの前に、YAML が参照する地図画像が実在することを確認する。

```bash
ros2 run susumu_object_perception validate_map_assets.py maps/indoor.yaml
```

```bash
# 認識実行中または実行後に object_memory の DB を使って描画する
ros2 run susumu_object_perception render_recognition_overlay.py \
  --map maps/indoor.yaml \
  --db /tmp/indoor_object_memory_pruned.sqlite3 \
  --out maps/indoor_recognition_overlay.png \
  --min-existence 0.5 \
  --min-hits 2 \
  --scale 8 \
  --ignore-class 'dining table' \
  --ignore-class couch
```

DB は `object_memory_node.py` の `db_path` パラメータで指定する。認識巡回と同時に記録する例:

```bash
ros2 run susumu_object_perception object_memory_node.py --ros-args \
  -p use_sim_time:=True \
  -p input_topic:=/perception/tracked_objects_classified \
  -p db_path:=/tmp/indoor_object_memory.sqlite3 \
  -p reset_db:=True \
  -p min_hits:=1 \
  -p require_fine_class:=True \
  -p min_fine_conf:=0.15 \
  -p require_map_support:=True \
  -p map_support_dist:=0.45 \
  -p map_support_class_distances:='plant=0.55,table=0.55' \
  -p static_class_geometry_filter:=True \
  -p static_duplicate_merge_dist:=1.7 \
  -p static_cross_class_merge_dist:=0.75 \
  -p static_compatible_class_groups:='chair,couch' \
  -p static_merge_class_priority:='chair,couch' \
  -p visible_range:=0.0
```

`visible_range:=0.0` は認識タスクの最終成果物向け設定。通常の物体メモリでは「見えるはずなのに
見えない」物体を削除するが、認識結果レビューでは巡回中に一度認識した物体を累積して残すため、
negative observation を実質無効化する。
`require_fine_class:=True` は YOLO の細クラスが確認できた物体だけを記憶し、tracker の幾何推定だけで
付いた `pedestrian` などの誤登録を抑える。`require_map_support:=True` は地図上の占有セルから
離れすぎた候補を静的物体メモリに入れないための屋内認識向け設定。
`map_support_class_distances` は、地図占有セルと物体中心がずれやすい semantic class だけ
map support 距離を上書きする。現在の採用値は既定 `0.45m` に対し `plant=0.55,table=0.55`。
一律 `0.55m` より余分な chair を抑え、植物/テーブルの正解候補は残しやすい。
`static_class_geometry_filter:=True` は、画像分類がクロップ内の背景を拾って壁片や家具の一部クラスタを
`potted plant` / `couch` などとして登録するケースを、クラス別の平面サイズ・縦横比で DB 登録前に抑える。
このフィルタが有効な場合、ルール未定義の静的COCOクラス（例: `bench` / `bed` / `tv`）は認識レビューの
対象外として登録しない。Webots indoor の PottedTree が YOLO で `umbrella` になるケースは
`potted plant` に正規化して扱う。
`static_duplicate_merge_dist:=1.7` は、LiDAR クラスタ分割や視点差で同一静的物体が複数 DB object に
なった場合に、同じ semantic class の近接候補を統合する。
`static_cross_class_merge_dist:=0.75` と `static_compatible_class_groups:='chair,couch'` は、
Armchair が YOLO/視点差で `chair` と `couch` に割れるケースを同一の静的座席候補として統合する。
統合時は hits を合算し、existence は複数の正観測として合成する。`dining table` は Table/Sofa を
一旦評価対象外にしている現段階でも互換統合には入れない。table まで座席互換へ入れると全対象評価で
ラベル副作用が出たため。
オンライン登録時の `map_support_dist` を一律 0.45m まで詰めると植物/テーブルの正しい候補も
蓄積前に落ち、一律 0.55m まで緩めると chair の余分候補が増えた。現在は class別 map support
(`plant=0.55,table=0.55`) で recall と余分検出のバランスを取る。

```bash
ros2 run susumu_object_perception filter_object_memory_db.py \
  --db /tmp/indoor_object_memory.sqlite3 \
  --out-db /tmp/indoor_object_memory_pruned.sqlite3 \
  --map maps/indoor.yaml \
  --map-support-dist 0.45 \
  --map-support-class-dist plant=0.55,table=0.55 \
  --static-class-geometry-filter \
  --merge-same-class-dist 1.7 \
  --merge-compatible-dist 0.75 \
  --merge-compatible-group 'chair,couch' \
  --merge-class-priority 'chair,couch'
```

以降の `render_recognition_overlay.py` / `evaluate_recognition_vs_world.py` は、最終成果物として
`/tmp/indoor_object_memory_pruned.sqlite3` を使う。これは world 真値を使う評価後処理ではなく、
保存地図と認識DBだけで、occupied セルから離れた静的候補やクラス幾何として不自然な候補を落とす
認識タスク側の最終整理。
Webots 系 launch の `object_classifier_node.py` は屋内既定で `yolov8s-seg.pt` を使い、
YOLO bbox だけでなく segmentation mask がクロップ中心 ROI に乗る候補だけを採る
（`require_mask_center:=True`）。これは Autoware の ROI-cluster fusion と同じく、画像側の
検出を LiDAR クラスタへ付ける前に整合を確認する考え方で、屋内の植物系 false positive を抑えるための
認識本体側ゲート。植物系ラベルには `plant_color_min_frac` の色整合性ゲートも併用する。

Webots 系 launch の `object_classifier_node.py` には分類済みトラックを定期再確認する
`reclassify_interval_sec` があるが、屋内フル巡回評価では一時的な YOLO miss で正解記憶の
hits が伸びにくくなったため既定は `0.0`（無効）にしている。クロップ背景を一度拾っただけの
ラベル対策は、まず `center_tolerance_frac`、`require_mask_center`、`plant_color_min_frac`、
`static_class_geometry_filter` で行う。
細クラス未確定時の即時 clear（`publish_unknown_fine_class_clears`）も同じ理由で既定無効にし、
`object_memory_node.py` 側の `fine_class_ttl` による自然失効に任せる。
`min_consistent_hits` は追加ゲートとして用意しているが、2 以上は recall を落としやすいため
Webots 巡回既定は 1 に留める。

### 最終評価（world との照合）

認識タスクの最後に、`object_memory_node.py` の DB を Webots world の静的物体配置と照合し、
検出結果が正しいかをレポートとして残す。`evaluate_recognition_vs_world.py` は world の
`PottedTree` / `Sofa` / `Armchair` / `Table` / `Fridge` / `Pedestrian` などを正解物体として読み、
ロボット初期位置を原点にした map 座標へ変換して、DB 内の認識結果と距離・ラベルでマッチングする。

出力は Markdown / JSON / CSV / PNG。Markdown は人間が読む合否記録、JSON/CSV は後処理用、
PNG は地図上に TP / ラベル誤り / 未検出 / 余分な検出を重ねた確認図。COCO 既定分類器で表現できない
`Cabinet` / `CardboardBox` / `FloorLight` などは既定では評価対象にせず、スキップ理由をレポートに残す。
必要なら `--target-type Type=label1,label2` で world type と許容ラベルを追加する。

```bash
ros2 run susumu_object_perception evaluate_recognition_vs_world.py \
  --wbt webots_worlds/indoor.wbt \
  --map maps/indoor.yaml \
  --db /tmp/indoor_object_memory.sqlite3 \
  --out-prefix maps/indoor_recognition_eval \
  --min-existence 0.5 \
  --min-hits 2 \
  --match-distance 1.0
```

`--match-distance` は world 真値と map/SLAM/検出位置のずれを許容する距離ゲート[m]。既定は 1.0m。
屋内地図で地図照合自体に数十 cm のずれがある場合は、結果にその前提を明記してから値を広げる。
`--map` を渡すと、評価対象 world object ごとに保存地図の最寄り occupied セル距離
`nearest_map_occupied_m` も出る。`--expected-map-support-dist` はこの診断だけに使う距離しきい値で、
既定は `0.55m`。F1 や正解判定は `--match-distance` のまま変えない。
評価対象を一時的に外す場合は `--ignore-type Sofa --ignore-type Table` のように指定する。
除外した type は期待物体から外し、その type の許容ラベル検出も余分検出として数えない。

```bash
ros2 run susumu_object_perception evaluate_recognition_vs_world.py \
  --wbt webots_worlds/indoor.wbt \
  --map maps/indoor.yaml \
  --db /tmp/indoor_object_memory.sqlite3 \
  --out-prefix maps/indoor_recognition_eval_ignore_table_sofa \
  --min-existence 0.5 \
  --min-hits 2 \
  --match-distance 1.0 \
  --ignore-type Table \
  --ignore-type Sofa
```

### 屋内フル巡回の直近結果（2026-06-20）

条件:
`webots_waypoint_nav.launch.py world:=indoor.wbt waypoints:=indoor_waypoints.yaml mode:=realtime loop:=False
perception:=True omni_perception:=True image_recognition:=True indoor_objects:=True slam:=True`。
巡回は `lap finished (reached=22/22 missed=[])` で完了。

`indoor_recognition_waypoints.yaml` は追加視点入りの実験用点列。2026-06-21 時点では
`view_clearance=0.6m` で 22 点に再生成し、ナビは `reached=22/22 missed=[]` まで改善した。
ただしライブ認識評価では採用中の通常巡回より悪化したため、通常の合格・採用評価は
`indoor_waypoints.yaml` を使う（下の「追加改善3」「追加改善7」参照）。

成果物:

| ファイル | 内容 |
|---|---|
| `maps/indoor_recognition_overlay.png` | 認識結果をラベル付きで地図へ重畳 |
| `maps/indoor_recognition_eval.md/json/csv/png` | world との比較（Table/Sofa も評価対象） |
| `maps/indoor_recognition_eval_ignore_table_sofa.md/json/csv/png` | Table/Sofa を一時除外した比較 |

評価:

| 条件 | expected | detections | correct | wrong_label | extra | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 全対象 | 9 | 4 | 3 | 1 | 0 | 0.750 | 0.333 | 0.462 |
| Table/Sofa 除外 | 7 | 2 | 2 | 0 | 0 | 1.000 | 0.286 | 0.444 |

#### 2026-06-20 追加改善（batch YOLO + cache 修正 + DB整理）

採用結果:

| 条件 | expected | detections | correct | wrong_label | extra | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 全対象 | 9 | 5 | 4 | 0 | 1 | 0.800 | 0.444 | 0.571 |
| Table/Sofa 除外 | 7 | 3 | 3 | 0 | 0 | 1.000 | 0.429 | 0.600 |

採用した変更:

- `object_classifier_node.py` は未分類/再分類対象クロップを cycle 内で集め、YOLO の list 入力 +
  `batch` 指定でまとめて推論する。Ultralytics 公式 docs の predict/config で、source list と batch
  指定が明記されているため、この方針を採用した。
- COCO 細クラス（`chair` / `dining table` / `potted plant` 等）は Autoware label では `UNKNOWN`
  に丸まるため、cache 利用判定を Autoware label ではなく `coco_name` の有無で行うよう修正した。
  これで fine class が TTL 内に継続 publish され、object_memory の hits が伸びる。
- `object_memory_node.py` に `static_duplicate_merge_dist` を追加し、同じ semantic class の近接重複を
  DB 作成段階で統合する。chair の平面面積上限も 0.6m² に下げ、CardboardBox 近傍の大型 chair 誤登録を
  抑える。
- `filter_object_memory_db.py` で最終成果物前に map support 0.45m + 静的幾何フィルタを適用する。
  採用 run では余分だった `chair #10` と `potted plant #12/#32` が、world 真値を使わずに削除された。

不採用にした調整:

- `map_support_dist:=0.45` を object_memory のオンライン登録に直接使うと、別 run で
  `correct=1 extra=0` まで recall が落ちた。登録時は 0.55m のまま蓄積し、最終整理で 0.45m を使う。
- `min_hits` の単純な緩和/強化は採用しない。正解数と余分検出のトレードオフが大きく、認識本体の改善に
  なりにくい。

同じ巡回条件の bbox 版（`yolov8s.pt`、mask gate 無し）は Table/Sofa 除外で
`correct=2 extra=2 precision=0.500 recall=0.286 F1=0.364`。segmentation mask gate 版は
正解数を下げずに余分検出を 2→0 に減らしたため採用する。

統合なしで低 hits を最終成果物に含める実験は採用しない。`min_hits=4` は extra が 1 増えて正解数は増えず、
`min_hits=3` は extra が 3 増えて正解数は増えず、`min_hits=2` は correct が 3 になる一方で
extra が 8、wrong_label が 1 まで戻った。正解を増やすには単純なしきい値緩和ではなく、
分類・位置推定・同一物体統合側の改善が必要。

#### 2026-06-20 追加改善2（座席クラス揺れの統合）

採用結果（Table/Sofa はユーザー指定により一旦評価対象外）:

| 条件 | expected | detections | correct | wrong_label | extra | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 全対象 | 9 | 7 | 5 | 0 | 2 | 0.714 | 0.556 | 0.625 |
| Table/Sofa 除外 | 7 | 4 | 4 | 0 | 0 | 1.000 | 0.571 | 0.727 |

採用した変更:

- `object_memory_node.py` / `filter_object_memory_db.py` の近接重複統合を、同一 semantic class だけでなく
  指定した互換クラス群にも拡張した。屋内採用値は `chair,couch` のみ、距離 0.75m。
- 統合はペア逐次ではなく連結成分単位で行う。同じ物体の部分観測が複数IDに割れても、互いに近い候補群を
  まとめて hit-weighted average へ畳める。
- 統合時の existence は max ではなく `1-(1-pa)(1-pb)` 型で合成する。近接した複数の正観測を同一物体と
  判断したなら、存在確率も複数観測として上げるため。
- `render_recognition_overlay.py` に `--ignore-class` を追加し、Table/Sofa 除外評価と同じ対象だけを
  地図重畳画像へ出せるようにした。

この変更で、以前は `chair #18 hits=1` と `couch #19 hits=1` に割れていた Armchair 近傍候補が
`chair #18 hits=2 existence=0.51` へ統合され、Table/Sofa 除外評価では正解数が 3→4、余分検出は 0 のまま。
`dining table` を互換統合に含める案は、Armchair の hits はさらに増えるが全対象評価で Table/Sofa 周辺の
ラベル副作用が出たため採用しない。

#### 2026-06-20 追加改善3（認識向け追加視点、未採用）

試した変更:

- `generate_waypoints.py` に `--object-viewpoints` を追加し、保存地図上の occupied 小〜中サイズ成分を
  見る追加視点を入れられるようにした。
- map 境界成分を拾った追加視点が Nav2 で `worldToMap failed` になったため、
  `--view-map-border-margin` を追加して境界成分を除外するよう修正した。

ライブ評価（境界マージン追加前の `indoor_recognition_waypoints.yaml`）:

| 条件 | expected | detections | correct | wrong_label | extra | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 全対象 | 9 | 8 | 2 | 1 | 5 | 0.250 | 0.222 | 0.235 |
| Table/Sofa 除外 | 7 | 5 | 2 | 1 | 2 | 0.400 | 0.286 | 0.333 |

巡回は `reached=25/26 missed=[3]`。追加視点により Fridge / PottedTree[4] / Armchair への幾何距離は
縮まったが、クロップ背景の誤分類と余分検出が増え、採用中の通常巡回 + 座席統合結果
（Table/Sofa 除外 `correct=4 wrong_label=0 extra=0 F1=0.727`）より悪化した。したがって
`indoor_recognition_waypoints.yaml` は現時点では採用せず、実験用成果物として残す。

#### 2026-06-20 追加改善4（保存地図AMCL巡回、未採用）

SLAM中の map frame 変動が静的物体メモリの座標ずれや Fridge 近傍の誤分類に影響している可能性を
切り分けるため、保存済み地図 + AMCL でも同じ通常ウェイポイントを巡回した。

実行条件:
`webots_waypoint_nav.launch.py world:=indoor.wbt waypoints:=indoor_waypoints.yaml mode:=realtime loop:=False
perception:=True omni_perception:=True image_recognition:=True indoor_objects:=True slam:=False
map_file:=maps/indoor.yaml nav_params_file:=config/nav2_params.yaml`。
巡回自体は `lap finished (reached=22/22 missed=[])` で完了。

ライブ評価:

| 条件 | expected | detections | correct | wrong_label | extra | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 全対象 | 9 | 6 | 3 | 2 | 1 | 0.500 | 0.333 | 0.400 |
| Table/Sofa 除外 | 7 | 4 | 3 | 0 | 1 | 0.750 | 0.429 | 0.545 |

AMCL run のフィルタ後DBは 8 件で、`Fridge` / `Armchair` / 左下 `PottedTree` /
`BunchOfSunFlowers` が未検出、右上に余分な `potted plant` が 1 件残った。全対象では
`Sofa` と `Armchair` 近傍を `dining table` として扱うラベル誤りも出た。
採用中の通常 SLAM 巡回 + 座席統合結果（Table/Sofa 除外 `correct=4 wrong_label=0 extra=0 F1=0.727`）
より悪化したため、保存地図AMCL化は認識改善策としては採用しない。ナビ自体の確認結果は
[巡回ナビタスク](waypoint_navigation.md) に残す。

#### 2026-06-20 追加改善5（YOLOv8m-seg重み・地図スナップ、未採用）

Webots 系 launch に `object_yolo_weights` 引数を追加し、通常 SLAM 巡回で `yolov8m-seg.pt` を試した。
`/perception/tracked_objects_classified` は約 8.7Hz で出ており、処理詰まりは主因ではない。
巡回自体も `lap finished (reached=22/22 missed=[])` で完了した。

ライブ評価:

| 条件 | expected | detections | correct | wrong_label | extra | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 全対象 | 9 | 7 | 2 | 2 | 3 | 0.286 | 0.222 | 0.250 |
| Table/Sofa 除外 | 7 | 3 | 0 | 1 | 2 | 0.000 | 0.000 | 0.000 |

`Fridge` は回復せず、`chair` / `dining table` / `couch` の誤対応が増え、正しい `potted plant` 候補も
最終 map support フィルタで落ちた。採用中の `yolov8s-seg.pt` + mask/色ゲート + 座席統合結果
（Table/Sofa 除外 `F1=0.727`）より大きく悪化したため、既定 weight は変更しない。
`object_yolo_weights` 引数は、今後の controlled comparison 用として残す。

同じく、検出位置を保存地図の occupied 成分へ単純にスナップする後処理も、採用 DB 相当で試した。
Table/Sofa 除外 F1 は `0.727` から `0.667` または `0.615` に下がったため採用しない。
位置を正解へ寄せるには、world 真値を使わない単純補正ではなく、クロップの対応付け・クラスタ形状・
同一物体統合側の改善が必要。

#### 2026-06-21 追加改善6（分類ゲート診断、採用）

未検出の `Fridge` / `PottedTree` / `BunchOfSunFlowers` を増やすには、単純なしきい値緩和ではなく、
YOLO 候補がどの段で落ちているかを見てから gate を狙って調整する必要がある。そこで
`object_classifier_node.py` に `publish_debug_diagnostics` を追加し、Webots 系 launch から
`object_classifier_debug:=True` で有効化できるようにした。

診断 topic は `/perception/object_classifier/debug`。各 status は YOLO 候補 1 件を表し、
`message` に `accepted` / `box_area` / `center_tolerance` / `center_window_overlap` /
`mask_center_overlap` / `plant_color` / `no_yolo_detection` の理由を入れる。`values` には
class、conf、area_frac、center_dx/dy、center_overlap、mask_overlap、plant_color を入れる。
独自 msg は増やさず、ROS 2 標準の `diagnostic_msgs/DiagnosticArray` を使う。ROS 2 の
diagnostic_msgs はロボット状態/コンポーネント診断用の標準メッセージ群で、DiagnosticArray は
DiagnosticStatus の配列として診断情報を送る用途と説明されている。
参考: <https://docs.ros.org/en/humble/p/diagnostic_msgs/>、
<https://index.ros.org/p/diagnostic_msgs/>

この変更自体は分類結果を変えないため、採用評価値は追加改善2のまま。

2026-06-21 に `object_classifier_debug:=True` のまま通常 22 点巡回を実行し、
`lap finished (reached=22/22 missed=[])` を確認した。debug 集計では rejection の主因が
`center_tolerance` / `mask_center_overlap` の植物・花瓶系候補で、`no_yolo_detection` は 3 件だけだった。
一方、この run の評価は全対象 `correct=3 wrong_label=1 extra=3 F1=0.375`、Table/Sofa 除外
`correct=2 wrong_label=0 extra=2 F1=0.364` で悪化した。したがって debug topic は診断用として採用するが、
debug 有効 run の認識結果は採用しない。中心/mask gate を単純に緩めると余分検出が増える可能性が高いため、
次の改善は候補対応付け・LiDAR 位置・静的物体統合側を優先する。

#### 2026-06-21 追加改善7（認識追加視点の到達性修正、認識は未採用）

`generate_waypoints.py` の `--view-clearance` 既定を、通常 waypoint の `--clearance` と同じ値にした。
Nav2 の costmap は robot radius / footprint で衝突判定し、inflation layer が障害物周辺の近接コストを
作るため、認識用追加視点だけ 0.45m に緩めると、オフライン測地距離では接続していても実走で詰まる。
参考: Nav2 Costmap 2D <https://docs.nav2.org/configuration/packages/configuring-costmaps.html>、
Inflation Layer <https://docs.nav2.org/configuration/packages/costmap-plugins/inflation.html>、
Tuning Guide の Robot Footprint vs Radius
<https://docs.nav2.org/tuning/index.html#robot-footprint-vs-radius>。

比較結果:

| 点列 | ナビ結果 | 全対象 F1 | Table/Sofa 除外 F1 | 判定 |
|---|---|---:|---:|---|
| 境界マージン追加前 26 点 | `reached=25/26 missed=[3]` | 0.235 | 0.333 | 未採用 |
| 境界マージン済み 26 点 (`view_clearance=0.45m`) | `reached=25/26 missed=[3]` | 0.375 | 0.545 | 未採用 |
| 安全側 22 点 (`view_clearance=0.6m`) | `reached=22/22 missed=[]` | 0.143 | 0.200 | 未採用 |

安全側 22 点は `maps/indoor_recognition_waypoints.yaml` として再生成済み
（object-viewpoints 3 件、測地経路長 22.1m、最大測地ジャンプ 2.9m）。ナビの到達性は改善したが、
認識は `PottedTree[3]` 1 件だけが正解で、余分な `potted plant` が 2 件残った。したがって
認識成果物の採用 run は引き続き通常 `indoor_waypoints.yaml` を使う。

#### 2026-06-21 追加改善8（multi-FOV / imgsz 960、未採用）

`object_classifier_node.py` に `crop_fovs_deg` と `yolo.imgsz` を追加し、Webots 系 launch から
`object_crop_fovs_deg` / `object_yolo_imgsz` で指定できるようにした。Ultralytics 公式 docs では
`imgsz` は推論時の画像サイズ、`batch` は推論 batch サイズとして扱われるため、既存の batch 推論に
複数クロップを載せる形で実装した。

通常 22 点巡回で次を試した:

```bash
ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
  world:=indoor.wbt waypoints:=indoor_waypoints.yaml mode:=realtime loop:=False \
  perception:=True omni_perception:=True image_recognition:=True indoor_objects:=True slam:=True \
  object_crop_fovs_deg:=75,55,40 object_yolo_imgsz:=960
```

巡回は `lap finished (reached=22/22 missed=[])`。最終DBフィルタは採用 run と同じ
map support 0.45m + 静的幾何フィルタ + `chair,couch` 互換統合を使った。

| 条件 | expected | detections | correct | wrong_label | extra | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 全対象 | 9 | 8 | 1 | 3 | 4 | 0.125 | 0.111 | 0.118 |
| Table/Sofa 除外 | 7 | 5 | 1 | 1 | 3 | 0.200 | 0.143 | 0.167 |

大きい `imgsz` と複数FOVで検出候補は増えたが、信頼度最大のクロップを選ぶだけでは背景家具や壁付近の
物体を拾いやすく、余分検出とラベル誤りが増えた。したがって `crop_fovs_deg` / `yolo.imgsz` は
controlled comparison 用の実験フックとして残し、採用既定は変更しない。次に multi-FOV を試す場合は、
信頼度最大ではなく FOV 間の同一クラス合意、LiDAR クラスタ投影との重なり、または中心 mask/色ゲートを
FOV ごとに比較する選択規則を先に入れる。

#### 2026-06-21 追加改善9（multi-FOV 合意選択、既定未採用）

追加改善8の低成績箇所は、複数 FOV / 大きい `imgsz` で候補は増える一方、クロップごとの
信頼度最大だけで代表を選ぶため、背景家具や壁際物体が LiDAR 対象へ誤付与されることだった。
実装前に上流情報を確認した。Autoware の image projection based fusion は image bbox/segmentation と
LiDAR cluster 等を融合して obstacle の classification/detection を refine する設計で、ROI cluster
fusion も 2D detector の ROI で cluster label を上書き/フィルタする。Ultralytics の segmentation
結果は instance ごとに mask / class / confidence / box を持つため、既存の中心ROI/mask overlap と
FOV間の同一 class 合意を代表選択へ使える。

参考にした一次情報:

- Autoware image projection based fusion:
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/>
- Autoware ROI cluster fusion:
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/docs/roi-cluster-fusion/>
- Ultralytics predict mode:
  <https://docs.ultralytics.com/modes/predict>
- Ultralytics segmentation results:
  <https://docs.ultralytics.com/tasks/segment>

採用したコード変更:

- `object_classifier_node.py` の `YoloClassifier` が、各クロップの採用済み候補
  (`class/conf/center_overlap/mask_overlap`) を `last_candidates` として保持する。
- 同一 track に複数 FOV crop がある場合、`_select_multifov_detection()` が
  `class_stability_key` ごとに候補をまとめ、FOV support、center overlap、mask overlap を加点して
  代表 class を選ぶ。単一 FOV では従来通り confidence 最大を使うため既定挙動は変わらない。
- 追加パラメータは `multi_fov_agreement_bonus=0.18`,
  `multi_fov_center_overlap_weight=0.10`, `multi_fov_mask_overlap_weight=0.12`。

ライブ評価:

```bash
ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
  world:=indoor.wbt waypoints:=indoor_waypoints.yaml mode:=realtime loop:=False \
  rviz:=False perception:=True omni_perception:=True image_recognition:=True \
  indoor_objects:=True slam:=True \
  object_crop_fovs_deg:=75,55,40 object_yolo_imgsz:=960 \
  report_prefix:=/tmp/susumu_cycle28_multifov/nav mission_timeout_sec:=900.0
```

同時に `object_memory_node.py` を認識評価用設定で起動し、`/tmp/susumu_cycle28_multifov/indoor_object_memory.sqlite3`
へ記録した。巡回は `reached=22/22 missed=[]`, elapsed `196.526s`。`/perception/tracked_objects_classified`
はおおむね `8-10Hz` で publish され、処理詰まりは主因ではなかった。

注意: 今回の後処理時点で `maps/indoor.yaml` に対応する `maps/indoor.pgm` が source 側に無く、
install 側の `indoor.pgm` も source 側を指す壊れた symlink だった。そのため採用 run と同じ
`map_support_dist=0.45` と overlay PNG 生成は再現できず、`map_support_dist=-1.0`（map support なし）
に、静的幾何フィルタ + `chair,couch` 互換統合をかけて比較した。成果物:

- `maps/indoor_recognition_cycle28_multifov_consensus_eval.md/json/csv`
- `maps/indoor_recognition_cycle28_multifov_consensus_eval_ignore_table_sofa.md/json/csv`

評価:

| 条件 | expected | detections | correct | wrong_label | extra | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 全対象（map support なし） | 9 | 13 | 5 | 0 | 8 | 0.385 | 0.556 | 0.455 |
| Table/Sofa 除外（map support なし） | 7 | 10 | 4 | 0 | 6 | 0.400 | 0.571 | 0.471 |

追加改善8の confidence 最大 multi-FOV（全対象 F1 `0.118`、Table/Sofa 除外 F1 `0.167`）よりは
大きく戻ったが、採用中の単一 FOV + mask/色ゲート + 座席統合（Table/Sofa 除外 F1 `0.727`）には届かない。
また map support なしで余分検出が多く、今回の結果だけで既定変更はできない。したがって
multi-FOV 合意選択は実験フックとして残し、既定は `object_crop_fovs_deg:=''`, `object_yolo_imgsz=640`
のままにする。

未検出は引き続き `Fridge` / `PottedTree[4]` / `BunchOfSunFlowers`。`Fridge` は今回も
`refrigerator/fridge` として出ず、PottedTree / SunFlowers は余分な `potted plant` と混ざる。
次に見る低成績箇所は、まず `maps/indoor.pgm` を復旧または再保存して map support/overlay 評価を
再現可能にすること。そのうえで multi-FOV を続けるなら、FOV 合意だけでなく LiDAR cluster 投影との
重なりをスコアへ入れる。Fridge は COCO 語彙内だが今回も未検出なので、crop/遮蔽診断または
専用重みの controlled comparison を優先する。

#### 2026-06-21 追加改善10（map asset validation、採用）

追加改善9で `maps/indoor.yaml` はあるが `maps/indoor.pgm` が無く、map support フィルタと overlay
PNG 生成が再現できないことが分かった。当時の `.gitignore` では `maps/*.pgm` / `maps/*.png` は
各自生成物として除外され、例外は `maps/cafe.pgm` だけだったため、まず評価前に「YAML が参照する画像が
実体として存在するか」を検査する仕組みを追加した。その後 2026-06-21 cycle32 で方針を変更し、
`*.pgm` はすべて commit 対象、確認用 `*.png` は再生成可能なので追跡外とした。

方針確認で参照した一次情報:

- Nav2 Map Server docs:
  <https://docs.nav2.org/configuration/packages/configuring-map-server.html>
  （Map Server / Map Saver が map と metadata の load/save を扱う）
- `nav2_map_server` package:
  <https://docs.ros.org/en/humble/p/nav2_map_server/>
- Navigation2 issue #3078:
  <https://github.com/ros-navigation/navigation2/issues/3078>
  （map_saver の相対 image path とサブディレクトリ保存で loader が画像を見つけられない事例）

採用した変更:

- `validate_map_assets.py` を追加し、`maps/*.yaml` のうち `image:` を持つ occupancy map YAML について、
  YAML 基準で相対画像パスを解決し、画像実体の欠落または壊れた symlink を検出する。
- `filter_object_memory_db.py` / `render_recognition_overlay.py` /
  `evaluate_recognition_vs_world.py` は、map image 欠落時に traceback ではなく
  `validate_map_assets.py` と `map_saver_cli` を示す短いエラーで終了する。
- `CMakeLists.txt` の install 対象に `validate_map_assets.py` を追加した。

検証:

```bash
ros2 run susumu_object_perception validate_map_assets.py maps/cafe.yaml
# OK: maps/cafe.pgm

ros2 run susumu_object_perception validate_map_assets.py maps/indoor.yaml
# NG: maps/indoor.pgm (image_missing)

ros2 run susumu_object_perception validate_map_assets.py --only-bad
# image_missing: 31 map YAMLs（cafe 以外の運用/実験 map 画像が未生成）
```

後処理スクリプトの欠落時出力も確認した。例:
`map image missing: maps/indoor.pgm referenced by maps/indoor.yaml. Run ... validate_map_assets.py ...`
となり、低レベルの `FileNotFoundError` traceback では止まらない。

採用/未採用:

- 採用: map asset validation と認識後処理の明示エラー化。
- 未採用: world 由来 `indoor_gt.pgm` などで `maps/indoor.pgm` を代用すること。map support は
  認識DBの最終整理に使うため、world 真値を入れると評価専用データを認識成果物へ混ぜることになる。

次に見る低成績箇所:

1. `webots_indoor_mapping.launch.py world:=indoor.wbt map_name:=indoor mode:=realtime` などで
   sensor/SLAM 由来の `maps/indoor.pgm` を再保存し、`validate_map_assets.py maps/indoor.yaml` を OK にする。
2. 同様に `break_room.pgm` も再保存し、屋内タスクの採用地図ペアを再現可能にする。
3. map support/overlay が再現できる状態で、追加改善9の multi-FOV 合意選択を再評価する。

#### 2026-06-21 追加改善11（map_saver 完了検査、採用）

追加改善10で map asset 欠落を検出できるようにしたが、欠落の再発防止はまだ弱かった。
`frontier_explore_node.py` の完了時保存は `subprocess.Popen()` で `map_saver_cli` を投げるだけで、
CLI が timeout しても frontier 側では成功/失敗が分からず、`maps/<name>.yaml` だけ残って
`maps/<name>.pgm` が無い状態を見逃し得る。

方針確認で参照した一次情報:

- Nav2 Map Saver docs:
  <https://docs.nav2.org/configuration/packages/map_server/configuring-map-saver.html>
  （Map Saver は CLI でも使え、`save_map_timeout` を持つ）
- Nav2 Map Server docs:
  <https://docs.nav2.org/configuration/packages/configuring-map-server.html>
  （Map Server / Map Saver が grid map と metadata の load/save を扱う）
- Navigation2 issue #1864:
  <https://github.com/ros-navigation/navigation2/issues/1864>
  （slam_toolbox の低頻度 `/map` に対して timeout と transient-local QoS が問題になる事例）

採用した変更:

- `frontier_explore_node.py` の保存処理を `Popen()` 投げっぱなしから、background thread 内の
  `subprocess.run()` に変更した。rclpy の探索完了処理はブロックせず、`map_saver_cli` の終了コードを
  `/frontier_explore/status` とログへ出す。
- `map_saver_cli` へ `save_map_timeout:=20.0` に加え、
  `map_subscribe_transient_local:=true` を渡す。低頻度 publish の `/map` を保存するときの timeout /
  QoS 問題を避けるため。
- CLI 成功後に `<prefix>.yaml` を読み、`image:` が指す PGM/PNG 実体が存在するかを検査する。
  YAML 欠落、YAML parse 失敗、`image:` 欠落、画像欠落はすべて `map save failed: ...` として見える。
- `mapping_indoor.md` の手動保存コマンドにも同じ `save_map_timeout` /
  `map_subscribe_transient_local` と `validate_map_assets.py` 確認を明記した。

評価:

| 検証 | 結果 |
|---|---|
| synthetic `/map` を publish し、`map_saver_cli -f /tmp/susumu_map_save_smoke --ros-args -p save_map_timeout:=5.0 -p map_subscribe_transient_local:=true` | `/tmp/susumu_map_save_smoke.yaml/.pgm` 保存成功 |
| `validate_map_assets.py /tmp/susumu_map_save_smoke.yaml` | OK |
| `frontier_explore_node.py` の保存後 asset helper | 生成済みペアは OK、欠落 prefix は `map yaml missing after save` |
| `webots_indoor_mapping.launch.py world:=indoor.wbt ... save_map:=False` 起動中の `/map` を `/tmp/susumu_cycle30_live_indoor_snapshot` へ保存 | `99x201`, `5.0x10.1m`, 壁率 `2.6%`, 最大連結成分 `99%`, 判定 `OK(微小片あり)` |

採用/未採用:

- 採用: frontier 完了時の map_saver 終了コード確認、transient-local QoS、保存後 asset 検査。
- 未採用: map_saver 失敗時に world 由来 `*_gt.pgm` を自動代用すること。認識成果物に評価専用データを
  混ぜる問題は追加改善10と同じ。

次に見る低成績箇所:

1. `webots_indoor_mapping.launch.py world:=indoor.wbt map_name:=indoor mode:=realtime` を実行し、
   自動保存ログが `map saved: maps/indoor.yaml -> maps/indoor.pgm` になることを確認する。
2. `break_room` でも同じ確認を行い、`validate_map_assets.py maps/indoor.yaml maps/break_room.yaml` を
   両方 OK にする。
3. その後、map support/overlay ありで追加改善9の multi-FOV 合意選択を再評価する。

#### 2026-06-21 追加改善12（`maps/indoor.pgm` 復旧、multi-FOV は再評価後も未採用）

追加改善11の次に見る低成績箇所は、実際に `maps/indoor.pgm` を sensor/SLAM 由来の realtime map として
再保存し、map support / overlay 評価を復旧することだった。方針は追加改善10/11と同じく Nav2
Map Saver / Map Server の公式 docs と、slam_toolbox の低頻度 `/map` で timeout / transient-local QoS が
問題になる Navigation2 issue #1864 を根拠にした。

参考:

- Nav2 Map Saver:
  <https://docs.nav2.org/configuration/packages/map_server/configuring-map-saver.html>
- Nav2 Map Server:
  <https://docs.nav2.org/configuration/packages/configuring-map-server.html>
- Navigation2 issue #1864:
  <https://github.com/ros-navigation/navigation2/issues/1864>

採用した成果物:

- `webots_indoor_mapping.launch.py world:=indoor.wbt mode:=realtime save_map:=False` を起動し、
  SLAM `/map` が出た状態で `map_saver_cli -f maps/indoor --ros-args -p save_map_timeout:=20.0
  -p map_subscribe_transient_local:=true` を実行した。
- `maps/indoor.pgm` を復旧し、`maps/indoor.yaml` の origin は `[-1.28, -5.01, 0]` になった。

地図評価:

| map | validate_map_assets | 寸法 | 壁率 | 最大連結成分 | 判定 |
|---|---|---:|---:|---:|---|
| `maps/indoor.yaml` | OK | `5.0x10.1m` (`99x201`) | `2.6%` | `99%` | `OK(微小片あり)` |

これで `filter_object_memory_db.py` / `render_recognition_overlay.py` /
`evaluate_recognition_vs_world.py` が `maps/indoor.pgm` 欠落で止まらず、map support ありの評価を再実行できた。
追加改善9の multi-FOV 合意選択 run を、`map_support_dist=0.45` + 静的幾何フィルタ +
`chair,couch` 互換統合で再評価した結果:

| 条件 | expected | detections | correct | wrong_label | extra | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 全対象（map support あり） | 9 | 3 | 1 | 0 | 2 | 0.333 | 0.111 | 0.167 |
| Table/Sofa 除外（map support あり） | 7 | 2 | 1 | 0 | 1 | 0.500 | 0.143 | 0.222 |

成果物:

- `maps/indoor_recognition_cycle31_multifov_map_support_eval.md/json/csv/png`
- `maps/indoor_recognition_cycle31_multifov_map_support_eval_ignore_table_sofa.md/json/csv/png`
- `maps/indoor_recognition_cycle31_multifov_map_support_overlay.png`

採用/未採用:

- 採用: `maps/indoor.pgm` 復旧。認識の map support/overlay 評価は再現可能になった。
- 未採用: 追加改善9の multi-FOV 合意選択を既定にすること。map support ありでは
  Table/Sofa 除外 F1 が `0.222` まで下がり、採用中の単一 FOV + mask/色ゲート + 座席統合
  （Table/Sofa 除外 F1 `0.727`）に届かない。

次に見る低成績箇所:

1. `break_room.wbt` でも `maps/break_room.pgm` を realtime SLAM 由来で再保存し、
   `validate_map_assets.py maps/indoor.yaml maps/break_room.yaml` を両方 OK にする。
2. 認識本体の次改善は multi-FOV ではなく、採用中の単一 FOV 条件で未検出の
   `Fridge` / `PottedTree` / `BunchOfSunFlowers` を増やす方向に戻す。
3. Fridge は COCO 語彙内なので crop/遮蔽診断、PottedTree/SunFlowers は植物色ゲートと
   static geometry の過剰削除を優先して見る。

#### 2026-06-21 追加改善13（`maps/break_room.pgm` 復旧と PGM 追跡方針、採用）

追加改善12の次に見る低成績箇所は、`maps/break_room.yaml` に対応する `maps/break_room.pgm` を
sensor/SLAM 由来の realtime map として再保存し、屋内採用地図ペアを両方再現可能にすることだった。
また、YAML だけが残ると Nav2 / waypoint / 認識評価が後段で壊れるため、ユーザー方針として
`*.pgm` は今後すべて commit 対象にする。

参考:

- Nav2 Map Saver:
  <https://docs.nav2.org/configuration/packages/map_server/configuring-map-saver.html>
- Nav2 Map Server:
  <https://docs.nav2.org/configuration/packages/configuring-map-server.html>
- Navigation2 issue #1864:
  <https://github.com/ros-navigation/navigation2/issues/1864>
- Git `gitignore` docs:
  <https://git-scm.com/docs/gitignore>

採用した成果物:

- `webots_indoor_mapping.launch.py world:=break_room.wbt mode:=realtime save_map:=False` を起動し、
  SLAM `/map` が出た状態で `map_saver_cli -f maps/break_room --ros-args -p save_map_timeout:=20.0
  -p map_subscribe_transient_local:=true` を実行した。
- `maps/break_room.pgm` を復旧し、`maps/break_room.yaml` の origin は `[-3.11, -2.7, 0]` になった。
- `.gitignore` から `maps/*.pgm` と `!maps/cafe.pgm` の例外を削除し、全 PGM を commit 対象にした。
  `maps/*.png` は確認用画像として引き続き追跡外。

地図評価:

| map | validate_map_assets | 寸法 | 壁率 | 最大連結成分 | 判定 |
|---|---|---:|---:|---:|---|
| `maps/indoor.yaml` | OK | `5.0x10.1m` (`99x201`) | `2.6%` | `99%` | `OK(微小片あり)` |
| `maps/break_room.yaml` | OK | `9.4x7.0m` (`188x140`) | `2.3%` | `100%` | `OK` |

`check_map_vs_world.py --map maps/break_room.yaml --wbt webots_worlds/break_room.wbt` では、
wall の `near_ratio_inside=0.848`、obstacle の `near_ratio_inside=0.750` だった。`PottedTree` は
3 個中 2 個が近傍 occupied、床サンプルは大半が free/unknown 側で、Nav2 用の大枠地図としては使える。

採用/未採用:

- 採用: `maps/break_room.pgm` 復旧と、全 `*.pgm` を commit 対象にする ignore 方針。
- 未採用: 確認用 overlay PNG まで追跡対象にすること。PNG は再生成でき、差分ノイズが大きい。

次に見る低成績箇所:

1. 認識本体は採用中の単一 FOV 条件で、未検出の `Fridge` / `PottedTree` / `BunchOfSunFlowers` を増やす。
2. `PottedTree` は `break_room` 地図で一部 occupied として出ているため、植物候補の static geometry
   フィルタが強すぎないか確認する。
3. Fridge は COCO 語彙内なので、全天球 crop の向き・サイズ・遮蔽を先に診断する。

#### 2026-06-21 追加改善14（class別 map support、採用）

追加改善13の次に見る低成績箇所は、`PottedTree` などの植物候補が保存地図の occupied セルから
少し離れているだけで最終整理から消え、multi-FOV 合意選択の map support あり評価が
Table/Sofa 除外 F1 `0.222` まで落ちることだった。一律に map support を緩めると、今度は chair などの
余分検出が増えるため、semantic class ごとに map support 距離を上書きできるようにした。

参考:

- Autoware image projection based fusion:
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/>
- Autoware ROI cluster fusion:
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/docs/roi-cluster-fusion/>
- Ultralytics predict settings:
  <https://docs.ultralytics.com/usage/cfg>
- COCO dataset docs:
  <https://docs.ultralytics.com/datasets/detect/coco>
- Autoware Universe issue #4680:
  <https://github.com/autowarefoundation/autoware_universe/issues/4680>

Autoware の ROI cluster fusion は LiDAR cluster と 2D ROI の重なりでラベルを上書きする。上流 issue でも
小さい物体は 2D 検出を強めに使い、LiDAR は validation / depth として使う方向が議論されている。
今回はその考え方に合わせ、保存地図との距離ゲートを全クラス一律ではなく semantic class 別に調整する。

採用した変更:

- `filter_object_memory_db.py` に `--map-support-class-dist` を追加した。
  例: `--map-support-dist 0.45 --map-support-class-dist plant=0.55,table=0.55`。
- `object_memory_node.py` に `map_support_class_distances` パラメータを追加し、オンライン登録時も同じ
  class別距離を使えるようにした。`plant=0.55,table=0.55` のように指定する。
- `-1` を指定した class は map support を無効化できる。ただし今回の採用値では使わない。

評価は追加改善9/12の multi-FOV 合意選択 DB
`/tmp/susumu_cycle28_multifov/indoor_object_memory.sqlite3` を、復旧済み `maps/indoor.yaml/.pgm` で
同じ静的幾何フィルタ・近接統合にかけ直した。

| 最終整理条件 | expected | detections | correct | extra | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 一律 `0.45m`（旧） | 9 | 3 | 1 | 2 | 0.333 | 0.111 | 0.167 |
| 一律 `0.55m` | 9 | 8 | 4 | 4 | 0.500 | 0.444 | 0.471 |
| class別 `plant=0.55,table=0.55`（採用） | 9 | 7 | 4 | 3 | 0.571 | 0.444 | 0.500 |
| class別 `plant=1.50,table=0.55` | 9 | 10 | 5 | 5 | 0.500 | 0.556 | 0.526 |
| map support なし | 9 | 13 | 5 | 8 | 0.385 | 0.556 | 0.455 |

Table/Sofa 除外:

| 最終整理条件 | expected | detections | correct | extra | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|
| 一律 `0.45m`（旧） | 7 | 2 | 1 | 1 | 0.500 | 0.143 | 0.222 |
| 一律 `0.55m` | 7 | 6 | 3 | 3 | 0.500 | 0.429 | 0.462 |
| class別 `plant=0.55,table=0.55`（採用） | 7 | 5 | 3 | 2 | 0.600 | 0.429 | 0.500 |
| class別 `plant=1.50,table=0.55` | 7 | 8 | 4 | 4 | 0.500 | 0.571 | 0.533 |
| map support なし | 7 | 10 | 4 | 6 | 0.400 | 0.571 | 0.471 |

採用/未採用:

- 採用: class別 map support と `plant=0.55,table=0.55`。一律 `0.45m` から全対象 F1 `0.167`→`0.500`,
  Table/Sofa 除外 F1 `0.222`→`0.500` に改善し、一律 `0.55m` より extra が 1 件少ない。
- 未採用: 一律 `0.75m`、map support なし、`plant=1.50m` を既定にすること。`plant=1.50m` は
  F1 だけは上がるが、map support の検証力を大きく弱め、余分検出が 5 件まで増える。
- 未採用: multi-FOV を既定にすること。class別 map support 後も、採用中の単一 FOV + mask/色ゲート +
  座席統合（Table/Sofa 除外 F1 `0.727`）には届かない。

成果物:

- `maps/indoor_recognition_cycle5_class055_eval.md/json/csv/png`
- `maps/indoor_recognition_cycle5_class055_eval_ignore_table_sofa.md/json/csv/png`

次に見る低成績箇所:

1. `Fridge` は今回も DB に出ていない。COCO 語彙内なので、全天球 crop の向き・サイズ・遮蔽を
   debug crop / `/perception/object_classifier/debug` で見る。
2. `BunchOfSunFlowers` は COCO では植物系に寄せるしかないため、plant 色ゲートと LiDAR cluster 位置の
   対応を確認する。
3. `PottedTree[3]` は `plant=1.50m` なら拾えるが、現状の保存地図では occupied から 1.482m 離れる。
   地図側に植物の occupied が出ていないのか、検出中心がずれているのかを切り分ける。

#### 2026-06-21 追加改善15（missed 近傍検出診断、採用）

追加改善14の次に見る低成績箇所は、`Fridge` / `BunchOfSunFlowers` が「分類だけ外れている」のか、
そもそも `object_memory` の検出中心が評価対象近傍に無いのかを切り分けられないことだった。
以前の `evaluate_recognition_vs_world.py` は `missed_without_near_detection_count` を実質 missed 総数として
出しており、false negative の原因を見誤りやすかった。

参考:

- Ultralytics YOLO performance metrics:
  <https://docs.ultralytics.com/guides/yolo-performance-metrics>
- COCO dataset docs:
  <https://docs.ultralytics.com/datasets/detect/coco>
- Autoware ROI cluster fusion:
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/docs/roi-cluster-fusion/>

Ultralytics の評価 docs は precision / recall / F1 と false positive / false negative の解釈を重視し、
COCO docs では `potted plant` / `refrigerator` / `vase` が既定語彙にあることを確認できる。
Autoware ROI cluster fusion は 2D ROI と LiDAR cluster の重なりでラベルを上書きし、debug image も
出す設計なので、FN を「近傍クラスタあり・ラベル違い」と「近傍クラスタなし」に分ける診断は次の改善方針に直結する。

採用した変更:

- `evaluate_recognition_vs_world.py` の missed 各行に `nearest_detection` と
  `nearest_label_detection` を JSON/CSV/Markdown で出すようにした。
- summary に `missed_count`, `missed_with_near_detection_count`,
  `missed_without_near_detection_count`, `missed_with_near_label_detection_count` を追加した。
  `missed_without_near_detection_count` は今回から、match distance 内に検出が無い missed の数を表す。
- Markdown の `Missed Ground Truth` 表に `nearest_any` / `nearest_label` 列を追加した。

評価は追加改善14の採用条件
`/tmp/susumu_cycle5_class055.sqlite3` + `plant=0.55,table=0.55` の最終整理結果を診断レポートとして
再出力した。F1 自体は変わらないが、FN の内訳が分かるようになった。

| 条件 | expected | detections | correct | missed | missed near | missed no-near | missed near same-label | extra | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 全対象 | 9 | 7 | 4 | 5 | 2 | 3 | 0 | 3 | 0.571 | 0.444 | 0.500 |
| Table/Sofa 除外 | 7 | 5 | 3 | 4 | 0 | 4 | 0 | 2 | 0.600 | 0.429 | 0.500 |

Table/Sofa 除外の missed はすべて match distance `1.0m` 以内に検出が無かった:

| missed | nearest any | nearest same accepted label |
|---|---|---|
| `Fridge[1]` | `#1 potted plant`, `1.65m` | なし |
| `PottedTree[3]` | `#16 chair`, `2.34m` | `#39 potted plant`, `2.58m` |
| `PottedTree[4]` | `#39 potted plant`, `2.26m` | `#39 potted plant`, `2.26m` |
| `BunchOfSunFlowers[1]` | `#16 chair`, `1.62m` | `#2 potted plant`, `1.77m` |

採用/未採用:

- 採用: missed の近傍検出診断。今後は FN をラベル問題と検出中心/登録問題に分けて見る。
- 未採用: nearest detection を正解扱いにすること、または match distance を緩めて F1 を上げること。
  今回の診断は評価の説明力を上げるためで、合格基準は変えない。

成果物:

- `maps/indoor_recognition_cycle6_class055_diagnostic_eval.md/json/csv/png`
- `maps/indoor_recognition_cycle6_class055_diagnostic_eval_ignore_table_sofa.md/json/csv/png`

次に見る低成績箇所:

1. `Fridge` は COCO 語彙内なのに nearest same-label が無い。single-FOV live で
   `object_classifier_debug:=True` を使い、Fridge 方向の crop が作られているか、YOLO 候補が出ているかを見る。
2. `BunchOfSunFlowers` は nearest plant が `1.77m` 離れている。LiDAR cluster / object_memory の中心が
   期待位置からずれているのか、別の植物候補に吸われているのかを調べる。
3. `PottedTree[3]` / `[4]` も same-label 検出はあるが 2m 以上離れるため、map support より前の
   detection/tracking/association の位置ずれを優先して見る。

#### 2026-06-21 追加改善16（missed の保存地図 occupied 診断、採用）

追加改善15で missed に近傍DB検出があるかは分かるようになったが、近傍DB検出が無い場合に
「保存地図には物体相当の occupied があるのに object_memory が拾っていない」のか、
「保存地図にも occupied が無く、LiDAR/SLAM 側で物体が見えていない」のかはまだ分からなかった。
そこで world 真値位置から保存地図の最寄り occupied セル距離を評価レポートへ追加した。

参考:

- Autoware detected object validation:
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_detected_object_validation/>
- Autoware occupancy grid based validator:
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_detected_object_validation/occupancy-grid-based-validator/>
- Autoware ROI cluster fusion:
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/docs/roi-cluster-fusion/>
- Ultralytics YOLO performance metrics:
  <https://docs.ultralytics.com/guides/yolo-performance-metrics>

Autoware detected object validation は pointcloud や occupancy grid に基づいて明らかな false positive を
落とす設計を持つ。今回の評価診断も同じ方向で、world 真値、認識DB、保存地図を横断して、
FN を「地図 occupied は近いがDB検出が無い」「地図 occupied も遠い」に分ける。

採用した変更:

- `evaluate_recognition_vs_world.py` に `--expected-map-support-dist` を追加した。既定は `0.55m`。
  これは診断用で、F1 や `--match-distance` の正解判定には使わない。
- `--map` 指定時、expected object に `nearest_map_occupied_m` と `has_map_support` を付与する。
- summary に `expected_with_map_support_count` と `missed_with_map_support_count` を追加した。
- Markdown / CSV の correct と missed 表に保存地図 occupied 距離を出す。

評価は追加改善14/15と同じ
`/tmp/susumu_cycle5_class055.sqlite3` + `maps/indoor.yaml` + `expected_map_support_dist=0.55m` で再出力した。
F1 は変わらない。

| 条件 | expected | detections | correct | missed | missed no-near DB | expected map support | missed map support | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 全対象 | 9 | 7 | 4 | 5 | 3 | 8 | 4 | 0.571 | 0.444 | 0.500 |
| Table/Sofa 除外 | 7 | 5 | 3 | 4 | 4 | 6 | 3 | 0.600 | 0.429 | 0.500 |

Table/Sofa 除外の missed 内訳:

| missed | map_occ_m | map_support | nearest DB | nearest same label | 判断 |
|---|---:|---|---|---|---|
| `Fridge[1]` | `0.45` | True | `#1 potted plant`, `1.65m` | なし | 地図には occupied があり、Fridge DB が無い。分類/crop/DB登録側を疑う |
| `PottedTree[3]` | `1.62` | False | `#16 chair`, `2.34m` | `#39 potted plant`, `2.58m` | 地図 occupied も遠い。LiDAR/SLAM/物体形状側の欠落を疑う |
| `PottedTree[4]` | `0.22` | True | `#39 potted plant`, `2.26m` | `#39 potted plant`, `2.26m` | 地図 occupied はあるがDB中心が遠い。tracking / association の位置ずれを疑う |
| `BunchOfSunFlowers[1]` | `0.33` | True | `#16 chair`, `1.62m` | `#2 potted plant`, `1.77m` | 地図 occupied はあるが植物DB中心が遠い。別植物候補への吸収を疑う |

採用/未採用:

- 採用: world 真値ごとの保存地図 occupied 距離診断。次の live debug で見るべき層を絞れる。
- 未採用: `has_map_support` を正解判定へ入れること。これは評価説明用で、認識評価の合格基準は変えない。

成果物:

- `maps/indoor_recognition_cycle7_map_occupancy_diagnostic_eval.md/json/csv/png`
- `maps/indoor_recognition_cycle7_map_occupancy_diagnostic_eval_ignore_table_sofa.md/json/csv/png`

次に見る低成績箇所:

1. `Fridge` は map occupied が `0.45m` と近いのに Fridge DB が無い。single-FOV live で
   Fridge 方向 crop / YOLO 候補 / fine_class publish の有無を直接見る。
2. `BunchOfSunFlowers` は map occupied が `0.33m` と近いが nearest plant DB が `1.77m`。
   object_memory の association と植物候補の吸収を確認する。
3. `PottedTree[3]` は map occupied が `1.62m` と遠いため、LiDAR/SLAM 側の占有欠落を見る。

#### 2026-06-21 追加改善17（Fridge/Bunch/PottedTree 診断用 viewpoint、採用）

追加改善16で、Fridge と BunchOfSunFlowers は保存地図上の occupied が近いのに
1m 内のDB検出が無いことが分かった。一方で通常巡回路や `--object-viewpoints` を既定化すると、
過去評価では余分検出とラベル誤りが増えた。そこで採点用巡回路は変えず、missed 対象を正面から見る
controlled live debug 用の yaw 付き waypoint を別成果物として生成することにした。

方針確認で参照した一次情報:

- Autoware detected object validation:
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_detected_object_validation/>
- Autoware occupancy grid based validator:
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_detected_object_validation/occupancy-grid-based-validator/>
- Autoware ROI cluster fusion:
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/docs/roi-cluster-fusion/>
- Ultralytics YOLO performance metrics:
  <https://docs.ultralytics.com/guides/yolo-performance-metrics>

Autoware の validator は occupancy grid や点群で検出物体を検証し、ROI cluster fusion は camera ROI と
LiDAR cluster の重なりでラベルを扱う。今回の改善も同じ切り分けで、SLAM map 上の通行可能領域と
world 真値の対象位置から「安全に近づいて対象を向く」診断 waypoint を作り、Fridge/Bunch が
crop/YOLO の段階で落ちているのか、LiDAR/object_memory association で落ちているのかを次の live run で見る。
Ultralytics の指標では recall は実物体を拾える割合、F1 は precision/recall のバランスなので、
今回も評価の正解距離や採点基準は変えず、missed の原因分解を進める。

採用した変更:

- `generate_recognition_debug_viewpoints.py` を追加した。`evaluate_recognition_vs_world.py` と同じ
  Webots world parser を使い、指定した world type の map 座標を取得する。
- 保存地図の free/occupied/unknown と distance transform から、start component 上で
  `clearance>=0.60m`、対象から `1.0-2.2m`、対象方向 line-of-sight ありの候補を探索する。
- 出力 YAML は `waypoint_nav_node.py` が読める `[x, y, yaw]` 形式にし、対象ごとの
  map support / 候補数 / LOS 候補数を Markdown/JSON/CSV に残す。
- `CMakeLists.txt` の install 対象へ追加した。

生成コマンド:

```bash
ros2 run susumu_object_perception generate_recognition_debug_viewpoints.py \
  --map maps/indoor.yaml \
  --wbt webots_worlds/indoor.wbt \
  --target-type Fridge \
  --target-type BunchOfSunFlowers \
  --target-type PottedTree \
  --out maps/indoor_recognition_cycle8_debug_viewpoints.yaml \
  --report-prefix maps/indoor_recognition_cycle8_debug_viewpoints \
  --require-los
```

今回の評価値:

| target | map occupied | map support | free+clear candidates | LOS candidates | selected viewpoint |
|---|---:|---|---:|---:|---|
| `PottedTree[1]` | `0.10m` | True | 405 | 405 | `(-0.06, -2.79, -60deg)` |
| `PottedTree[2]` | `0.10m` | True | 294 | 294 | `(0.83, 3.42, -175deg)` |
| `Fridge[1]` | `0.45m` | True | 93 | 32 | `(0.61, 4.05, 155deg)` |
| `PottedTree[3]` | `1.62m` | False | 455 | 455 | `(2.22, -2.97, 65deg)` |
| `PottedTree[4]` | `0.22m` | True | 99 | 99 | `(-0.28, -3.18, -110deg)` |
| `BunchOfSunFlowers[1]` | `0.33m` | True | 539 | 456 | `(2.39, -1.42, 140deg)` |

summary: `target_count=6`, `generated_waypoints=6`, `targets_with_map_support=5`,
`selected_with_line_of_sight=6`, `no_candidate_count=0`, `component_source=start`。

採用/未採用:

- 採用: Fridge/Bunch/PottedTree を対象にした診断用 yaw 付き viewpoint 生成。
  次の live run で `object_classifier_debug:=True` と合わせ、crop/YOLO 候補と object_memory association を見る。
- 未採用: この world 真値由来 waypoint を採点用の通常巡回路へ入れること。対象選択に world 真値を使うため、
  recall 改善の本線ではなく原因切り分け専用にする。
- 未採用: `--match-distance` を広げる、または `has_map_support` を正解扱いにして F1 を上げること。
  F1 は追加改善16と同じく全対象 `0.500`、Table/Sofa 除外 `0.500` のまま。

成果物:

- `scripts/generate_recognition_debug_viewpoints.py`
- `maps/indoor_recognition_cycle8_debug_viewpoints.yaml`
- `maps/indoor_recognition_cycle8_debug_viewpoints.md/json/csv`

次に見る低成績箇所:

1. `maps/indoor_recognition_cycle8_debug_viewpoints.yaml` を `webots_waypoint_nav.launch.py` に渡し、
   `object_classifier_debug:=True` で `Fridge[1]` の crop が生成されているか、YOLO に
   `refrigerator` 候補が出るかを見る。
2. `BunchOfSunFlowers[1]` と `PottedTree[4]` は map support があるため、debug waypoint から
   plant 候補が出るか、出る場合に object_memory が別植物へ吸収していないかを見る。
3. `PottedTree[3]` は map support が無いまま LOS viewpoint だけ生成できた。live debug で
   LiDAR/SLAM map 側に occupied が立つかを優先して見る。

#### 2026-06-21 追加改善18（object classifier debug recorder、採用）

追加改善17で Fridge/Bunch/PottedTree を向く waypoint は作れたが、`ros2 topic echo` やログ断片だけでは
YOLO 候補、fine class、track 位置、waypoint のどれに問題があるかを後から集計しづらかった。
そこで `/perception/object_classifier/debug` と `/perception/object_fine_classes`、分類済み track、
`/waypoint_nav/status` を同時に記録し、target waypoint YAML の `targets` と突き合わせる recorder を追加した。

方針確認で参照した一次情報:

- ROS 2 `diagnostic_msgs/DiagnosticArray`:
  <https://docs.ros2.org/foxy/api/diagnostic_msgs/msg/DiagnosticArray.html>
- Autoware image projection based fusion:
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/>
- Autoware ROI cluster fusion:
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/docs/roi-cluster-fusion/>
- Ultralytics predict mode:
  <https://docs.ultralytics.com/modes/predict>
- Ultralytics instance segmentation:
  <https://docs.ultralytics.com/tasks/segment>

Autoware の image projection based fusion は 2D detector の ROI と LiDAR cluster を対応付けて
classification を扱う。今回の recorder も同じ切り分けで、LiDAR track が対象位置にあるか、
その track の crop に YOLO 候補が出たか、候補がどの gate で reject されたかを target ごとに残す。
Ultralytics の segmentation 結果は instance ごとに class/conf/box/mask を持つため、
既存の debug 候補（class/conf/mask overlap/reject reason）を集計対象にした。

採用した変更:

- `record_recognition_debug.py` を追加した。入力:
  `/perception/object_classifier/debug`、`/perception/object_fine_classes`、
  `/perception/tracked_objects_classified`、`/waypoint_nav/status`。
- `maps/indoor_recognition_cycle8_debug_viewpoints.yaml` の `targets` を読み、
  track 位置が target から `1.2m` 以内ならその target に集計する。
  live debug waypoint では `--active-waypoint-fallback` も使い、位置照合できない debug status を
  現在向いている target へ暫定集計できるようにした。
- 出力は `<prefix>.md/json/csv`。target ごとに tracked/debug/no_yolo/accepted/selected/fine class、
  accepted classes、fine classes、reject reasons、最寄り track 距離を残す。
- `CMakeLists.txt` の install 対象へ追加した。

静的/合成検証:

- `python3 -m py_compile scripts/record_recognition_debug.py`: OK
- topic が無い空起動で `target_count=6` / all zero の report 生成: OK
- 合成 `DiagnosticArray` で waypoint #2 に `refrigerator` accepted/fine class を publish し、
  `Fridge[1]` に `accepted_classes.refrigerator=1`, `fine_classes.refrigerator=1` と集計されることを確認。
- `ros2 run susumu_object_perception record_recognition_debug.py ... --duration-sec 1.0`: OK

live debug run:

```bash
ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
  world:=indoor.wbt \
  waypoints:=/home/taro/ros2_ws/src/susumu_object_perception/maps/indoor_recognition_cycle8_debug_viewpoints.yaml \
  mode:=realtime rviz:=False perception:=True omni_perception:=True image_recognition:=True \
  indoor_objects:=True slam:=True loop:=False object_classifier_debug:=True \
  report_prefix:=/tmp/susumu_cycle9_live/nav mission_timeout_sec:=420.0 goal_timeout_sec:=70.0
```

同時に `object_memory_node.py` を認識評価用設定で起動し、
`record_recognition_debug.py --target-waypoints maps/indoor_recognition_cycle8_debug_viewpoints.yaml
--out-prefix /tmp/susumu_cycle9_live/debug --active-waypoint-fallback` で記録した。

ナビ結果:

| waypoint | target | result | duration_sec |
|---:|---|---|---:|
| 0 | `PottedTree[1]` | reached | 19.762 |
| 1 | `PottedTree[2]` | reached | 38.962 |
| 2 | `Fridge[1]` | reached | 11.261 |
| 3 | `PottedTree[3]` | reached | 45.761 |
| 4 | `PottedTree[4]` | reached | 18.362 |
| 5 | `BunchOfSunFlowers[1]` | reached | 35.512 |

summary: `reached=6/6`, `missed=[]`, mission elapsed `170.547s`。

recorder 結果:

| target | tracked | debug | no_yolo | accepted | selected | fine | nearest | accepted classes | fine classes | 主な reject |
|---|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| `PottedTree[1]` | 110 | 49 | 0 | 19 | 49 | 480 | 0.17 | chair:8, dining table:7, potted plant:2, bench:2 | dining table:253, bench:105, potted plant:91, chair:31 | center_tolerance:21 |
| `PottedTree[2]` | 70 | 58 | 1 | 16 | 51 | 587 | 0.19 | potted plant:4, dining table:4, chair:3, bed:2 | dining table:224, bench:131, chair:129, potted plant:67 | center_tolerance:36 |
| `Fridge[1]` | 20 | 30 | 0 | 5 | 16 | 82 | 0.10 | potted plant:3, bed:1, couch:1 | potted plant:69, dining table:13 | center_tolerance:14, mask_center_overlap:11 |
| `PottedTree[3]` | 791 | 100 | 1 | 37 | 76 | 837 | 0.32 | potted plant:8, couch:7, dining table:7, chair:7 | dining table:554, chair:81, bench:78, couch:71, potted plant:43 | center_tolerance:46 |
| `PottedTree[4]` | 0 | 16 | 0 | 4 | 11 | 85 |  | dining table:2, bench:1, chair:1 | dining table:52, couch:33 | center_tolerance:7 |
| `BunchOfSunFlowers[1]` | 304 | 99 | 0 | 27 | 80 | 487 | 0.03 | dining table:6, chair:6, potted plant:5, couch:5, bench:3 | dining table:165, potted plant:165, bench:68, chair:38, couch:28 | center_tolerance:57 |

判断:

- `Fridge[1]`: debug waypoint で track は近い (`nearest=0.10m`)。`no_yolo=0` なので crop/YOLO 実行は
  できているが、`refrigerator/fridge` 候補は 0。accepted/selected/fine は主に `potted plant`。
  低成績の主因は map support や crop 未生成ではなく、YOLO が Fridge 外観を COCO refrigerator として
  出していないこと。
- `BunchOfSunFlowers[1]`: track は近い (`nearest=0.03m`) し `potted plant` も fine class 165 回出るが、
  `dining table` も同数程度出て selected が割れる。植物だけでなく周辺什器/背景を crop が拾っている。
- `PottedTree[4]`: active waypoint fallback では debug は取れたが、position match の tracked は 0。
  LiDAR track の位置照合または target 近傍の形状推定をさらに見る必要がある。

debug waypoint run の object_memory DB を既定後処理した結果、`deleted=3`、pruned rows は 7。
ただしこの run は対象物を見るための短い診断経路で、通常巡回 coverage を持たない。採点DBとしては悪化した:

| 条件 | expected | detections | correct | wrong_label | missed | extra | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|
| 全対象 | 9 | 5 | 0 | 1 | 8 | 4 | 0.000 | 0.000 | 0.000 |
| Table/Sofa 除外 | 7 | 3 | 0 | 0 | 7 | 3 | 0.000 | 0.000 | 0.000 |

成果物:

- `scripts/record_recognition_debug.py`
- `maps/indoor_recognition_cycle9_debug_recorder.md/json/csv`
- `maps/indoor_recognition_cycle9_debug_nav.md/json/csv`
- `maps/indoor_recognition_cycle9_debug_waypoints_eval.md/json/csv/png`
- `maps/indoor_recognition_cycle9_debug_waypoints_eval_ignore_table_sofa.md/json/csv/png`

採用/未採用:

- 採用: target 別 debug recorder。今後の live run で crop/YOLO/fine class/object_memory の層を分けて見る。
- 未採用: debug waypoint run を通常 recognition scoring の採用値にすること。`reached=6/6` でも
  F1 は `0.000` で、通常巡回 coverage の代替ではない。
- 未採用: Fridge を `potted plant` として救済する label alias。world 真値では Fridge は COCO 語彙内の
  `refrigerator` なので、誤ラベルを正解化しない。

次に見る低成績箇所:

1. `Fridge[1]` の実クロップ画像を保存し、YOLO が refrigerator を出さない理由を目視する。
   そのうえで `yolov8m-seg.pt` / newer YOLO 重み / `imgsz` の controlled comparison を
   Fridge viewpoint だけで行う。
2. `BunchOfSunFlowers[1]` は `potted plant` と `dining table` が同程度出るため、target crop の
   中心/mask/色ゲートが背景什器を拾っていないか、保存 crop と candidate overlay で確認する。
3. `PottedTree[4]` は position-matched tracked が 0 のため、debug status の active waypoint fallback ではなく、
   LiDAR track 位置・shape 推定・map frame 変換のどこで target 近傍から外れるかを見る。

#### 2026-06-21 追加改善19（debug crop 保存と Fridge crop YOLO 比較、採用）

追加改善18の次に見る低成績箇所は、`Fridge[1]` の実クロップを保存し、YOLO が
`refrigerator` を出さない理由を目視・重み比較で切り分けることだった。方針決定前に以下を確認した:

- Ultralytics predict docs:
  `model.predict()` は画像リストを batch 入力でき、`conf` / `imgsz` / `batch` を推論時に指定できる。
  公式の `save_crop` は検出 box ごとの crop 保存で、今回ほしい「LiDAR track 方向の入力 crop」
  そのものではないため、自前保存にした。
  <https://docs.ultralytics.com/modes/predict>
  <https://docs.ultralytics.com/reference/engine/results/#ultralytics.engine.results.Results.save_crop>
- OpenCV docs: `cv2.imwrite()` で画像配列を PNG として保存できる。
  <https://opencv24-python-tutorials.readthedocs.io/en/latest/py_tutorials/py_gui/py_image_display/py_image_display.html>
- PyTorch docs: PyTorch 2.6 以降は `torch.load` が `weights_only=True` を既定にし、古い
  module checkpoint では信頼済み重みに限って `weights_only=False` または allowlist が必要になる。
  既存 `object_classifier_node.py` と同じ条件で比較するため、offline 比較スクリプトでも指定 weight 読込時だけ
  `weights_only=False` を使う。
  <https://docs.pytorch.org/docs/2.12/notes/serialization.html#torch-load-with-weights-only-true>

実装:

- `object_classifier_node.py` に `debug_crop_dir` / `debug_crop_min_interval_sec` /
  `debug_crop_max_per_track` / `debug_crop_write_rejected` を追加し、YOLO 入力 crop を PNG と
  `metadata.jsonl` で保存する。metadata には object id、object 位置、FOV、selected class、
  accepted/rejected candidate、bbox を残す。
- Webots 系 launch に `object_debug_crop_dir` 引数を通し、通常は空文字で無効、診断 run だけ有効にする。
- `scripts/evaluate_debug_crops_yolo.py` を追加し、保存 crop を target object id で絞り、
  複数 YOLO weight を同じ gate 条件で比較する。

Fridge 専用 viewpoint:

```bash
python3 scripts/generate_recognition_debug_viewpoints.py \
  --map maps/indoor.yaml \
  --wbt webots_worlds/indoor.wbt \
  --target-type Fridge \
  --out maps/indoor_recognition_cycle10_fridge_debug_viewpoint.yaml \
  --report-prefix maps/indoor_recognition_cycle10_fridge_debug_viewpoint \
  --require-los
```

結果は `generated_waypoints=1/1`、`map_support=True`、LOS=True、waypoint は
`(0.609, 4.048, 155deg)`。ライブ run は以下の条件:

```bash
ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
  world:=indoor.wbt \
  waypoints:=/home/taro/ros2_ws/src/susumu_object_perception/maps/indoor_recognition_cycle10_fridge_debug_viewpoint.yaml \
  mode:=realtime rviz:=False perception:=True omni_perception:=True image_recognition:=True \
  indoor_objects:=True slam:=True loop:=False object_classifier_debug:=True \
  object_debug_crop_dir:=/tmp/susumu_cycle10_fridge/crops \
  report_prefix:=/tmp/susumu_cycle10_fridge/nav mission_timeout_sec:=220.0 goal_timeout_sec:=90.0
```

ライブ評価:

| report | 値 |
|---|---:|
| waypoint reached | `1/1` |
| nav elapsed | `26.556s` |
| recorder elapsed | `94.007s` |
| crop metadata rows | `18` |
| Fridge target crops used in YOLO comparison | `17` |

recorder の Fridge 集計:

| target | tracked | debug | no_yolo | accepted | selected | fine | nearest | accepted classes | selected classes | fine classes |
|---|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| `Fridge[1]` | 3 | 78 | 0 | 31 | 63 | 537 | 0.97 | potted plant:9, bench:6, couch:6, chair:6, dining table:2 | bench:16, couch:16, potted plant:15, chair:11, bed:5 | bench:181, bed:95, dining table:90, chair:76, potted plant:60 |

保存 crop の offline YOLO 比較:

| weight | crops | refrigerator raw | refrigerator accepted | selected classes |
|---|---:|---:|---:|---|
| `yolov8s-seg.pt` | 17 | 2 | 0 | vase:3, bench:3, potted plant:4, bed:1, chair:3, couch:1 |
| `yolov8m-seg.pt` | 17 | 2 | 0 | vase:1, chair:2, dining table:2, potted plant:1, couch:7, bench:2 |

`refrigerator` raw は最大でも conf `0.051` で、accepted しきい値 `0.15` 未満。目視した raw
`refrigerator` crop は Fridge が背景に小さく写り、中央は sofa / plant / lamp / boxes だった。
したがって今回の低成績は「Fridge 本体を十分な大きさで見ているのに YOLO が外す」より、
LiDAR track と crop 中心が Fridge 本体に乗っていない問題が主因と見る。

成果物:

- `scripts/evaluate_debug_crops_yolo.py`
- `maps/indoor_recognition_cycle10_fridge_debug_viewpoint.md/json/csv/yaml`
- `maps/indoor_recognition_cycle10_fridge_debug_nav.md/json/csv`
- `maps/indoor_recognition_cycle10_fridge_debug_recorder.md/json/csv`
- `maps/indoor_recognition_cycle10_fridge_crops/metadata.jsonl`
- `maps/indoor_recognition_cycle10_fridge_crops/*.png`
- `maps/indoor_recognition_cycle10_fridge_crop_yolo_compare.md/json/csv`

採用/未採用:

- 採用: object classifier の debug crop 保存機能。普段は無効で、`object_debug_crop_dir` を指定した
  診断 run だけ成果物を出す。
- 採用: 保存 crop の offline YOLO weight 比較スクリプト。ライブを毎回回さず、同じ入力 crop で
  weight / imgsz / gate の差分を比較できる。
- 未採用: `yolov8m-seg.pt` を屋内認識の既定にすること。Fridge crop では `refrigerator accepted=0` のまま、
  selected は `couch` へ寄り、追加改善5のフル巡回悪化とも整合する。
- 未採用: `refrigerator` の accepted しきい値を下げること。raw conf は最大 `0.051` で、周辺 crop に
  低 confidence で出ているだけのため、下げると余分検出を増やす可能性が高い。

次に見る低成績箇所:

1. `Fridge[1]` は分類器の重み差より、LiDAR 検出/追跡 cluster が Fridge 本体に乗るかを先に見る。
   `detected_objects_shaped` / `detected_objects_merged` / `tracked_objects` の target 近傍分布を
   Fridge viewpoint で保存し、crop center の由来を切り分ける。
2. Fridge 診断 viewpoint は map LOS だけでは sofa/plant/lamp に crop が吸われる。world 真値を使う
   診断 waypoint 生成では、target 中心への LOS だけでなく foreground obstacle と画像上の見込みサイズを
   スコアに入れる。
3. `BunchOfSunFlowers[1]` も同じ crop 保存機能で、`potted plant` と `dining table` が割れる crop を
   目視・weight 比較する。

#### 2026-06-21 追加改善20（perception stage 診断 recorder、採用）

追加改善19の次に見る低成績箇所は、`Fridge[1]` の crop 中心が Fridge 本体に乗らない原因を、
LiDAR 検出・map ROI・tracker のどの層で起きるか切り分けることだった。方針決定前に以下を確認した:

- Autoware image projection based fusion は、画像 bbox/segmentation と LiDAR cluster/box/segmentation を
  統合して obstacle detection / classification を refine する設計。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/>
- ROI cluster fusion は、cluster を画像へ投影し、2D detector ROI との重なりで cluster label を
  上書き/フィルタする。今回の late fusion でも、crop がどの 3D cluster 由来かをまず見える化する必要がある。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/docs/roi-cluster-fusion/>
- Autoware euclidean cluster は点群を小クラスタへ分ける目的のモジュールで、後段で shape/tracker と
  組み合わせる前提。cluster の過分割/未分割は detection_by_tracker 側でも扱われる問題。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_euclidean_cluster/>
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_detection_by_tracker/>

実装:

- `scripts/record_recognition_debug.py` を拡張し、既存の YOLO candidate / fine class / tracked 集計に加え、
  `DetectedObjects` stage と `TrackedObjects` stage の target 最近傍距離を記録する。
- 既定 stage は `detected:/perception/detected_objects`、
  `shaped:/perception/detected_objects_shaped`、`merged:/perception/detected_objects_merged`、
  `in_map:/perception/detected_objects_in_map`、`tracked_raw:/perception/tracked_objects`。
- target ごとに stage の message 数、nearest sample 数、`stage_match_distance` 内の数、
  最近距離、最近 shape、距離ビンを JSON/CSV/Markdown に出す。
- 単一 target の debug waypoint run では、`heading to waypoint` status を取り逃がしても
  active waypoint fallback が target へ帰属できるようにした。

ライブ評価は追加改善19と同じ Fridge 専用 waypoint:

```bash
ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
  world:=indoor.wbt \
  waypoints:=/home/taro/ros2_ws/src/susumu_object_perception/maps/indoor_recognition_cycle10_fridge_debug_viewpoint.yaml \
  mode:=realtime rviz:=False perception:=True omni_perception:=True image_recognition:=True \
  indoor_objects:=True slam:=True loop:=False object_classifier_debug:=True \
  object_debug_crop_dir:=/tmp/susumu_cycle11b_fridge/crops \
  report_prefix:=/tmp/susumu_cycle11b_fridge/nav mission_timeout_sec:=180.0 goal_timeout_sec:=90.0
```

```bash
ros2 run susumu_object_perception record_recognition_debug.py \
  --target-waypoints maps/indoor_recognition_cycle10_fridge_debug_viewpoint.yaml \
  --out-prefix /tmp/susumu_cycle11b_fridge/debug \
  --duration-sec 130 --write-period-sec 2.0 \
  --active-waypoint-fallback --stage-match-distance 2.0
```

ナビ結果:

| waypoint reached | nav elapsed | recorder elapsed | crop rows |
|---:|---:|---:|---:|
| `1/1` | `28.747s` | `130.006s` | `22` |

Fridge target の recorder 集計:

| target | tracked | debug | accepted | selected | fine | nearest |
|---|---:|---:|---:|---:|---:|---:|
| `Fridge[1]` | 15 | 104 | 36 | 86 | 675 | 0.17 |

stage 診断:

| stage | messages | samples | within 2.0m | nearest | nearest shape | distance bins |
|---|---:|---:|---:|---:|---|---|
| `detected` | 1138 | 1136 | 1125 | 0.32 | 0.00x0.00x0.00 | 1.0-1.5:1119, <0.5:5, 0.5-1.0:1, >=2.0:11 |
| `shaped` | 1138 | 1137 | 1126 | 0.19 | 0.39x0.51x1.70 | 1.0-1.5:1112, 1.5-2.0:8, <0.5:6, >=2.0:11 |
| `merged` | 1137 | 1136 | 1125 | 0.19 | 0.39x0.51x1.70 | 1.0-1.5:1110, 1.5-2.0:9, <0.5:6, >=2.0:11 |
| `in_map` | 1137 | 1136 | 1125 | 0.19 | 0.39x0.51x1.70 | 1.0-1.5:1110, 1.5-2.0:9, <0.5:6, >=2.0:11 |
| `tracked_raw` | 1137 | 329 | 15 | 0.17 | 0.49x0.70x1.70 | >=2.0:314, <0.5:15 |

判断:

- `detected`→`shaped`→`merged`→`in_map` までは Fridge 近傍 cluster が残る。map ROI は主因ではない。
- `tracked_raw` は同じ run で Fridge 近傍に 15 samples だけ出るが、大半の samples は `>=2m`。
  crop は近傍 track `0e000000...` で 1 枚保存され、selected は `potted plant`。目視でも対象 crop は
  Fridge ではなく potted plant が中心だった。
- したがって次の改善は YOLO 重みや map ROI ではなく、tracker の track 作成/維持、または debug
  viewpoint の target 視線が foreground plant に吸われる問題を扱う。

成果物:

- `maps/indoor_recognition_cycle11_fridge_stage_nav.md/json/csv`
- `maps/indoor_recognition_cycle11_fridge_stage_recorder.md/json/csv`
- `maps/indoor_recognition_cycle11_fridge_stage_crops/metadata.jsonl`
- `maps/indoor_recognition_cycle11_fridge_stage_crops/*.png`

採用/未採用:

- 採用: `record_recognition_debug.py` の perception stage 診断。今後、target missed の原因を
  検出/shape/merge/map ROI/tracker/crop の層で切り分ける。
- 未採用: map ROI の緩和。`in_map` に Fridge 近傍 cluster は残っており、今回の主因ではない。
- 未採用: YOLO 重み/しきい値変更。追加改善19と同じく、crop 中心が対象に乗っていないため先に upstream を見る。

次に見る低成績箇所:

1. Fridge 近傍の `in_map` 検出が `tracked_raw` で短時間 track にしかならない原因を、
   object_tracker の `min_hits` / association gate / map wall margin / static track 出力条件で切り分ける。
2. Fridge target crop が potted plant 中心になるため、debug viewpoint 生成に foreground obstacle / image
   projected size の評価を追加し、Fridge を直接見られる診断点を選ぶ。
3. tracker 側を触る場合は、通常巡回の Table/Sofa 除外 F1 `0.727` を壊さないよう、Fridge 専用 run だけで
   よく見えても採用せず、通常 `indoor_waypoints.yaml` の再評価まで行う。

#### 2026-06-21 追加改善21（object tracker publish/reject 診断、採用）

追加改善20の次に見る低成績箇所は、Fridge 近傍の `in_map` 検出が tracker 出力で短時間 track にしか
ならない原因を、`min_hits` / association gate / map wall margin / static track 出力条件に分けて見ることだった。
方針決定前に以下を確認した:

- Autoware multi object tracker は検出との association、track lifecycle、存在確率を分けて扱う。
  本パッケージの Python tracker も同じ考え方で、publish 前の track lifecycle と publish 時の map filter を
  分けて診断する必要がある。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_multi_object_tracker/>
- Autoware map based prediction/filtering 系は地図外・壁・走行可能領域外を別段で扱う設計が多い。
  今回の `object_tracker_node.py` は、検出段の `map_roi_filter_node.py` に加えて publish 段にも
  wall-margin filter を持つため、どちらで落ちたかを標準 DiagnosticArray で直接見る。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_map_based_prediction/>
- ROS 2 の `diagnostic_msgs/DiagnosticArray` は key/value 付きの標準診断型なので、独自 msg を増やさず
  track ごとの publish/reject 理由を流せる。
  <https://docs.ros.org/en/humble/p/diagnostic_msgs/>

実装:

- `object_tracker_node.py` に `publish_debug_diagnostics` を追加した。True のとき
  `/perception/object_tracker/debug` に track ごとの `published` / `min_hits` / `map_blocked` を
  `DiagnosticArray` で出す。通常は False。
- Webots 系 launch に `object_tracker_debug` と controlled comparison 用 `object_tracker_min_hits` を追加した。
  既定は `object_tracker_debug:=False`, `object_tracker_min_hits:=2` で従来挙動を維持する。
- `record_recognition_debug.py` が `/perception/object_tracker/debug` を購読し、target 近傍の tracker reason を
  集計するようにした。

ライブ評価は Fridge 専用 waypoint で 2 条件を比較した:

| 条件 | waypoint | nav elapsed | target tracked | tracker nearest | tracker reasons | `tracked_raw` nearest | `tracked_raw` within 2m |
|---|---:|---:|---:|---:|---|---:|---:|
| 既定 `object_tracker_min_hits:=2` | `1/1` | `27.984s` | 0 | 0.38 | `min_hits:12` | 3.95 | 0 |
| 比較 `object_tracker_min_hits:=1` | `1/1` | `26.659s` | 0 | 0.31 | `map_blocked:24` | 2.43 | 0 |

既定条件では Fridge 近傍の track は `hits=1` のまま `min_hits=2` に届かない。`min_hits=1` へ緩めると
`min_hits` は通るが、同じ近傍 track は静止 track と判定され `wall_margin_static_cells=22` の
publish 段 map filter で `map_blocked` になる。したがって `min_hits` だけを緩めても Fridge は
出ない。

成果物:

- `maps/indoor_recognition_cycle12_tracker_debug_nav.md/json/csv`
- `maps/indoor_recognition_cycle12_tracker_debug_recorder.md/json/csv`
- `maps/indoor_recognition_cycle12_tracker_debug_crops/metadata.jsonl`
- `maps/indoor_recognition_cycle12_tracker_minhit1_nav.md/json/csv`
- `maps/indoor_recognition_cycle12_tracker_minhit1_recorder.md/json/csv`
- `maps/indoor_recognition_cycle12_tracker_minhit1_crops/metadata.jsonl`

採用/未採用:

- 採用: `/perception/object_tracker/debug` と recorder 側の tracker reason 集計。
- 採用: `object_tracker_min_hits` launch 引数。既定 2 を保ち、診断 run でだけ controlled comparison する。
- 未採用: `object_tracker_min_hits:=1` の既定化。Fridge 専用 run でも `tracked_raw` within 2m は 0 のままで、
  publish 段 map filter に落ちる。通常巡回へ入れる根拠がない。
- 未採用: publish 段 map filter の無条件緩和。wall-margin filter は壁際静止ゴースト対策として入っており、
  通常巡回 F1 / extra を再評価せずに外すと過去の壁 FP を戻す可能性が高い。

次に見る低成績箇所:

1. Fridge のような壁近傍静止物体を出すには、`indoor_objects:=True` のときだけ
   `wall_margin_static_cells` を小さくする、または class/crop support が取れた track だけ publish 段
   map filter を緩める controlled comparison を行う。
2. ただし Fridge 専用 run だけでは採用しない。`indoor_waypoints.yaml` の通常巡回で
   Table/Sofa 除外 F1 `0.727` と extra を壊さないことを確認する。
3. Fridge crop は foreground potted plant に吸われやすい。tracker wall margin と並行して、
   診断 waypoint 生成の foreground obstacle / projected size スコアも見る。

#### 2026-06-21 追加改善22（tracker wall margin controlled comparison、診断採用）

追加改善21の次に見る低成績箇所は、Fridge のような壁近傍静止物体を、壁ゴースト抑制を壊さず
出す条件だった。方針決定前に以下を確認した:

- Autoware multi object tracker は association / track lifecycle を扱う。今回の問題は tracker の
  lifecycle だけではなく、publish 段の map filter で落ちるため、出力段の条件を独立して比較する。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_multi_object_tracker/>
- Autoware detected object validation には occupancy grid based validator があり、占有格子と検出物体を
  比較して false positive を落とす。2D 地図による publish 段 filter を「検証段」として扱い、
  既定値を動かさず controlled comparison だけを行う方針にする。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_detected_object_validation/occupancy-grid-based-validator/>
- 同じ validation 群には object position filter もあり、検出/追跡そのものと、通す領域の制約を分けている。
  本パッケージでも `min_hits` と wall margin を別引数にして切り分ける。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_detected_object_validation/object-position-filter/>

実装:

- `object_tracker_wall_margin_moving_cells` / `object_tracker_wall_margin_static_cells` を
  `autoware_perception.launch.py`、`webots_simulation.launch.py`、`webots_nav.launch.py`、
  `webots_waypoint_nav.launch.py` に追加した。
- 既定はノード既定と同じ `moving=6`, `static=22`。通常起動の挙動は変えない。
- `object_tracker_min_hits:=1 object_tracker_wall_margin_static_cells:=3` を Fridge 専用 waypoint で
  比較した。これは「前回 `min_hits=1` で `map_blocked` になった track を publish 側へ通せるか」の
  切り分けで、採用候補ではない。

ライブ評価:

| 条件 | waypoint | nav elapsed | target tracked | tracker nearest | tracker reasons | `tracked_raw` nearest | `tracked_raw` within 2m | accepted classes | selected classes |
|---|---:|---:|---:|---:|---|---:|---:|---|---|
| cycle12 既定 `min_hits=2, static_margin=22` | `1/1` | `27.984s` | 0 | 0.38 | `min_hits:12` | 3.95 | 0 | none | none |
| cycle12 `min_hits=1, static_margin=22` | `1/1` | `26.659s` | 0 | 0.31 | `map_blocked:24` | 2.43 | 0 | none | none |
| cycle13 `min_hits=1, static_margin=3` | `1/1` | `29.577s` | 24 | 0.21 | `published:24, map_blocked:12` | 0.21 | 25 | `potted plant:15, couch:4, bench:3, dining table:3, chair:3` | `potted plant:41, couch:14, chair:7, bench:5, vase:5` |

`static_margin=3` で Fridge 近傍 track は publish され、`tracked_raw` within 2m は `0` から `25` に
増えた。したがって Fridge 未検出の原因の一部は publish 段 wall margin だった。一方で分類結果は
`potted plant` が優勢で、Fridge としてはまだ認識できていない。`static_margin=3` を既定化すると
壁際静止ゴースト抑制を弱めるため、通常巡回の Table/Sofa 除外 F1 `0.727` と extra を再評価するまで
採用しない。

成果物:

- `maps/indoor_recognition_cycle13_tracker_margin3_nav.md/json/csv`
- `maps/indoor_recognition_cycle13_tracker_margin3_recorder.md/json/csv`
- `maps/indoor_recognition_cycle13_tracker_margin3_crops/metadata.jsonl`
- `maps/indoor_recognition_cycle13_tracker_margin3_crops/*.png`

採用/未採用:

- 採用: `object_tracker_wall_margin_moving_cells` / `object_tracker_wall_margin_static_cells` launch 引数。
  既定値を保ったまま、壁近傍静止物の controlled comparison を再現可能にする。
- 採用: Fridge 未検出の切り分け結果。`min_hits=1` だけでは不十分で、publish 段の
  `wall_margin_static_cells=22` が近傍 track を落としていた。
- 未採用: `object_tracker_wall_margin_static_cells:=3` の既定化。Fridge 専用 run では track を出せたが、
  分類は `potted plant` 優勢で、通常巡回の壁ゴースト FP への影響も未評価。
- 未採用: tracker 側だけで Fridge 認識を完了したとみなす判断。今回で tracker 出力までは改善したが、
  final class はまだ Fridge ではない。

次に見る低成績箇所:

1. Fridge crop が foreground plant / sofa 系へ吸われる分類側の原因を見る。保存 crop に対して
   `refrigerator` raw candidate の有無、中心/mask gate、複数FOV、視点選定を比較する。
2. `static_margin=3` は通常巡回 F1 / extra を壊すリスクがあるため、分類側で Fridge support を作ってから、
   class/crop support が取れた静止 track だけ map filter を緩める条件を検討する。
3. 通常巡回へ入れる候補は `indoor_waypoints.yaml` で Table/Sofa 除外 F1 `0.727` 以上、extra 非増加を
   確認してから採用する。

#### 2026-06-21 追加改善23（YOLO predict/accept 分離と foreground 視点診断、診断採用）

追加改善22の次に見る低成績箇所は、Fridge crop が `potted plant` に吸われる分類側の切り分けだった。
方針決定前に以下を確認した:

- Ultralytics YOLO predict mode は `conf` を推論時の検出しきい値として扱う。低しきい値 run は
  「候補を観測する診断」であり、そのまま分類採用しきい値を下げる判断とは分ける。
  <https://docs.ultralytics.com/modes/predict/>
- Ultralytics COCO の class 定義には `potted plant` と `refrigerator` が含まれる。したがって
  Fridge は語彙外ではなく、crop / 見え方 / しきい値 / cluster 対応の問題として扱う。
  <https://docs.ultralytics.com/datasets/detect/coco/>
- Autoware の roi cluster fusion は LiDAR cluster と画像 ROI を対応付け、ROI しきい値や
  cluster 内点の条件で label を付ける。今回も「YOLO の画像候補」と「LiDAR track の中心・mask 整合」を
  分けて診断する。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/docs/roi-cluster-fusion/>

実装:

- `object_yolo_conf` / `object_min_accept_conf` を `webots_simulation.launch.py`、
  `webots_nav.launch.py`、`webots_waypoint_nav.launch.py` に追加した。既定はどちらも `0.15` で、
  通常起動の挙動は変えない。
- `object_classifier_node.py` の debug candidate に `min_accept_conf` reject reason を追加した。
  これにより recorder の accepted count は、最終 cache に採用され得る候補だけを数える。
- `evaluate_debug_crops_yolo.py` も同じ順序で `min_accept_conf` を評価するように揃えた。
- `generate_recognition_debug_viewpoints.py` に foreground blocker と projected-size score を追加した。
  Fridge では foreground-aware 条件でも従来と同じ waypoint `(0.61, 4.05, 155deg)` が選ばれ、
  foreground count は `0` だった。

ライブ評価:

| 条件 | waypoint | nav elapsed | target tracked | tracker reasons | `tracked_raw` nearest | raw refrigerator | refrigerator accepted | accepted classes | selected classes |
|---|---:|---:|---:|---|---:|---:|---:|---|---|
| cycle13 `min_hits=1, static_margin=3, yolo_conf=0.15` | `1/1` | `29.577s` | 24 | `published:24, map_blocked:12` | 0.21 | 0 | 0 | `potted plant:15, couch:4, bench:3, dining table:3, chair:3` | `potted plant:41, couch:14, chair:7, bench:5, vase:5` |
| cycle14 `min_hits=1, static_margin=0, yolo_conf=0.01, min_accept=0.15` | `1/1` | `43.04s` | 411 | `published:228, map_blocked:2` | 0.22 | 5-6 | 0 | `potted plant:41, bed:31, chair:29, couch:11, dining table:10` | `potted plant:262, bed:164, couch:123, dining table:89, chair:8` |

cycle14 の低しきい値 run では `refrigerator` raw candidate は出たが、`yolov8s-seg.pt` は
`raw=5 accepted=0`、`yolov8m-seg.pt` は `raw=6 accepted=0` だった。reject reason には
`min_accept_conf:72` が出ており、raw refrigerator の多くは `0.02` 前後の低信頼度だった。
`static_margin=0` は Fridge 近傍 track を大量に publish できる一方、accepted/selected は依然として
`potted plant` / `bed` / `couch` 優勢で、Fridge 認識は改善しなかった。

成果物:

- `maps/indoor_recognition_cycle14_margin0_conf001_nav.md/json/csv`
- `maps/indoor_recognition_cycle14_margin0_conf001_recorder.md/json/csv`
- `maps/indoor_recognition_cycle14_margin0_conf001_crops/metadata.jsonl`
- `maps/indoor_recognition_cycle14_margin0_conf001_crops/*.png`
- `maps/indoor_recognition_cycle14_margin0_conf001_yolo_compare.md/json/csv`
- `maps/indoor_recognition_cycle14_fridge_foreground_viewpoint.md/json/csv/yaml`

採用/未採用:

- 採用: `object_yolo_conf` / `object_min_accept_conf` launch 引数。低しきい値で raw 候補だけを観測し、
  採用しきい値は別に保つ controlled comparison を再現可能にする。
- 採用: `min_accept_conf` reject reason。debug/recorder の accepted count が最終分類採用条件と
  一致するため、低信頼 raw candidate を誤って「採用候補」と読まない。
- 採用: foreground-aware debug viewpoint generator。今回 Fridge では foreground は主因ではなかったと
  分かったため、次は crop/cluster 対応やモデル側を見る。
- 未採用: `object_yolo_conf:=0.01` の既定化。raw refrigerator は観測できるが、accepted refrigerator は
  `0` のままで、debug 量と誤候補だけが増える。
- 未採用: `object_min_accept_conf` の引き下げ。raw refrigerator は信頼度が低すぎ、Fridge 以外の低信頼
  false positive を採る可能性が高い。
- 未採用: `object_tracker_wall_margin_static_cells:=0` の既定化。Fridge 近傍 track は出るが、
  壁近傍静止 track が大量に出て、通常巡回 F1 / extra への影響が未評価。

次に見る低成績箇所:

1. Fridge は threshold ではなく crop/cluster 対応またはモデル証拠が不足している。次は実 track の
   3D shape/点群中心から crop 中心を補正し、Fridge 本体に ROI が乗るかを見る。
2. `Fridge[1]` の評価は「world target から 2m 以内」だけでは近傍 plant/sofa track を拾うため、
   recorder に target association の説明力を足す。少なくとも track id ごとの最近距離・shape・crop を
   併記して、Fridge 本体 track と foreground track を分ける。
3. モデル側は `yolov8m-seg.pt` でも accepted refrigerator が `0` だったため、次に試すなら
   単純な重み大型化より、Fridge 専用 crop の向き/サイズ調整、または固定クラスの軽量 custom classifier
   の小規模比較を行う。

#### 2026-06-22 追加改善24（target recorder の track id association、診断採用）

追加改善23の次に見る低成績箇所は、`Fridge[1]` の評価が「world target から 2m 以内」の集計だけでは
近傍 plant/sofa track を拾い、Fridge 本体 track と混ざることだった。方針決定前に以下を確認した:

- Autoware.Auto の ROI associator は、3D object を画像平面へ投影し、ROI と対応付けてから
  semantic 情報を使う。今回も target 全体集計ではなく、track id ごとに距離・shape・画像分類結果を
  併記する必要がある。
  <https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/tracking-roi-associator.html>
- Autoware Universe の roi cluster fusion も、cluster と画像 ROI を対応付けて label を上書きする。
  Fridge の低成績は「YOLO 候補があるか」だけでなく、「どの 3D track の crop か」を追う。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/docs/roi-cluster-fusion/>
- `record_recognition_debug.py` は `diagnostic_msgs/DiagnosticArray` を集計している。ROS 2 の
  DiagnosticArray は複数 status をまとめる message なので、status.name の track id をキーに
  per-object 集計を追加できる。
  <https://docs.ros2.org/foxy/api/diagnostic_msgs/msg/DiagnosticArray.html>

実装:

- `record_recognition_debug.py` に `object_associations` を追加した。target ごとに track id をキーにし、
  nearest distance、last/nearest shape、stage counts、YOLO debug counts、accepted/selected/fine class、
  tracker debug reason を JSON/CSV/Markdown に出す。
- `/perception/tracked_objects_classified`、`/perception/tracked_objects` stage、`/perception/object_tracker/debug`、
  `/perception/object_classifier/debug`、`/perception/object_fine_classes` を同じ track id で束ねる。
- `active_waypoint_fallback` で帰属したが位置情報が無い object は nearest を空欄にする。これにより
  「target 近傍の本体 track」と「debug waypoint 中に見えているだけの別 track」を表で区別できる。

ライブ評価:

1回目は FastRTPS SHM の `open_and_lock_file failed` が多発し、`spawner_*` が
`/controller_manager/list_controllers` に到達できず未採用 run とした。`docs/tasks/mapping_indoor.md` の
既知の罠に従い、2回目は `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` で同条件を再実行した。

条件: `min_hits=1`, `static_margin=0`, `object_yolo_conf=0.01`, `object_min_accept_conf=0.15`,
Fridge 専用 waypoint。これは分類改善の採用候補ではなく、association 診断の live 入力である。

| 条件 | waypoint | nav elapsed | target tracked | debug | accepted | selected | fine | nearest | accepted classes | selected classes | tracker reasons |
|---|---:|---:|---:|---:|---:|---:|---:|---:|---|---|---|
| cycle15 association live (`rmw_cyclonedds_cpp`) | `1/1` | `33.164s` | 19 | 263 | 13 | 159 | 4204 | 0.05 | `potted plant:8, couch:4, bench:1` | `potted plant:98, couch:43, bench:18` | `published:19, map_blocked:7` |

track id 別の主な結果:

| object | nearest | tracked | stages | debug | accepted | selected | fine | tracker | shape | selected/accepted |
|---|---:|---:|---|---:|---:|---:|---:|---:|---|---|
| `0a000000` | 0.05 | 9 | `tracker_debug:16, tracked_raw:9, tracked:9, fine_class:9` | 4 | 1 | 4 | 9 | 16 | `0.59x0.64x1.70` | selected `potted plant:4`, accepted `potted plant:1` |
| `0d000000` | 0.30 | 10 | `tracked_raw:10, tracked:10, tracker_debug:10` | 0 | 0 | 0 | 0 | 10 | `0.24x0.56x1.70` | 画像分類なし |
| `06000000` | 1.23 | 0 | `tracked_raw:649` | 0 | 0 | 0 | 668 | 0 | `0.63x0.49x1.70` | Fridge 本体からは遠い |
| `09000000` / `12000000` / `13000000` | 空欄 | 0 | 空欄 | 18/49/25 | 1/1/1 | 18/18/25 | 489/423/373 | 0 | 空欄 | active waypoint fallback 由来で、target 近傍 track ではない |

この表で、Fridge 最近傍の安定 track は `0a000000` で、shape は冷蔵庫らしい縦長 `0.59x0.64x1.70`、
距離も `0.05m` まで寄っていることが分かった。一方、その同じ track の crop は live YOLO で
`potted plant` accepted/selected になっている。つまり前回までの「selected が potted plant 多数」という
target 全体集計は、遠い/位置不明 track も混ざるが、本体 track だけを見ても `refrigerator` にはなっていない。

保存 crop の offline YOLO 比較:

| weight | crops | refrigerator raw | refrigerator accepted | selected classes |
|---|---:|---:|---:|---|
| `yolov8s-seg.pt` | 23 | 7 | 0 | `potted plant:9, couch:5, bench:2, vase:1` |
| `yolov8m-seg.pt` | 23 | 11 | 1 | `potted plant:7, couch:5, suitcase:4, bed:2, refrigerator:1` |

`yolov8m-seg.pt` は `refrigerator` accepted を 1 件出したが、該当 crop は object `10000000` で
Fridge から `3.57m` 離れており、Fridge 本体 track ではなかった。したがって大型重みへの切替は
今回も採用しない。

成果物:

- `maps/indoor_recognition_cycle15_object_assoc_nav.md/json/csv`
- `maps/indoor_recognition_cycle15_object_assoc_recorder.md/json/csv`
- `maps/indoor_recognition_cycle15_object_assoc/crops/metadata.jsonl`
- `maps/indoor_recognition_cycle15_object_assoc/crops/*.png`
- `maps/indoor_recognition_cycle15_object_assoc_yolo_compare.md/json/csv`

採用/未採用:

- 採用: `record_recognition_debug.py` の track id 別 `object_associations` 出力。今後の低成績診断で、
  target 近傍の本体 track と fallback/遠方 track を分けて読む。
- 採用: live 検証時に FastRTPS SHM が壊れた場合は `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp` で
  通信層を切り替えて再試行する運用知見。既定 launch は変えない。
- 未採用: `yolov8m-seg.pt` を屋内認識既定にすること。Fridge から 3.57m 離れた crop でのみ
  refrigerator accepted が出ており、本体 track の改善ではない。
- 未採用: `static_margin=0` / `yolo_conf=0.01` の既定化。今回も診断入力としてのみ使い、
  通常巡回 F1 / extra を評価していない。

次に見る低成績箇所:

1. Fridge 本体 track `0a000000` は `0.59x0.64x1.70` の縦長 shape と target 距離 `0.05m` まで
   取れている。次はこの track の crop だけに限定し、FOV/中心オフセット/shape projection を比較して
   `refrigerator` raw/accepted が出る視野条件を探す。
2. `0d000000` は Fridge 近傍で publish されるが画像分類に入っていない。なぜ crop が保存されないか
   reclassify interval / max inferences / crop projection のどこで落ちるかを見る。
3. target 全体集計は今後も残すが、採用判断は `object_associations` の nearest 付き track を優先する。

#### 2026-06-22 追加改善25（crop yaw/pitch offset 診断、部分採用）

追加改善24の次に見る低成績箇所は、Fridge 本体 track の crop 条件だった。cycle15 では
Fridge 最近傍 track `0a000000` が target 距離 `0.05m`、shape `0.59x0.64x1.70` まで取れたが、
live では `potted plant` selected/accepted になっていた。方針決定前に以下を確認した:

- Autoware の image projection based fusion は LiDAR object / cluster と画像 ROI を対応付けて
  classification/detection を refine する構成で、今回も LiDAR track ごとに crop/ROI 条件を比較する。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/>
- Autoware Universe の roi cluster fusion は 2D detector の ROI と 3D cluster の対応を見て label を
  付ける。単純に YOLO の最大信頼度だけでなく、track 中心に対する bbox/mask overlap を維持する。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/docs/roi-cluster-fusion/>
- Autoware.Auto の ROI associator も 3D object を画像へ投影して ROI と対応付ける。Fridge 診断では
  target 全体ではなく track id 別の crop 条件を読む。
  <https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/tracking-roi-associator.html>
- Ultralytics の predict mode は `conf` を推論時の候補しきい値として扱うため、今回も
  `object_yolo_conf=0.01` / `object_min_accept_conf=0.15` を診断用に分ける。
  <https://docs.ultralytics.com/modes/predict/>

実装:

- `object_classifier_node.py` に `crop_yaw_offsets_deg` / `crop_pitch_offsets_deg` を追加した。
  空なら従来通り `0` のみ。指定時は track 方向ベクトルを camera basis の right/up に沿って角度分だけ
  ずらし、同一 FOV の crop を複数作る。
- `/perception/object_classifier/debug` と raw crop `metadata.jsonl` に
  `crop_yaw_offset_deg` / `crop_pitch_offset_deg` を出す。`record_recognition_debug.py` の `fovs` 集計も
  `75.0/yaw-18.0/pitch0.0` のように offset 込みで数える。
- `webots_simulation.launch.py` / `webots_nav.launch.py` / `webots_waypoint_nav.launch.py` に
  `object_crop_yaw_offsets_deg`, `object_crop_pitch_offsets_deg`,
  `object_debug_crop_max_per_track` を通した。
- launch 引数 `object_crop_pitch_offsets_deg:=0` が ROS parameter YAML で INTEGER 推定されないよう、
  crop offset / FOV / debug crop dir は `ParameterValue(..., value_type=str)` で渡す。
- raw crop 保存の間引きキーを track id だけから `track+FOV+yaw+pitch` に変更した。これにより
  `object_debug_crop_max_per_track:=-1` では同一 track の yaw offset 違いがすべて保存される。

ライブ評価:

条件: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, Fridge 専用 waypoint,
`object_tracker_min_hits=1`, `object_tracker_wall_margin_static_cells=0`,
`object_yolo_conf=0.01`, `object_min_accept_conf=0.15`,
`object_crop_yaw_offsets_deg=-18,0,18`, `object_crop_pitch_offsets_deg=0`。
これは Fridge 診断用で、通常巡回の採用値ではない。

| run | waypoint | nav elapsed | recorder elapsed | debug | accepted | selected | fine | accepted classes | selected classes |
|---|---:|---:|---:|---:|---:|---:|---:|---|---|
| cycle16 offset recorder | `1/1` | `33.553s` | `120.001s` | 345 | 14 | 221 | 3568 | `couch:8, potted plant:6` | `couch:180, potted plant:41` |
| crop 保存修正後の短時間確認 | `1/1` | `33.665s` | - | - | - | - | - | - | - |

cycle16 recorder では target 近傍の `tracked_raw` 最近距離は `1.22m` で、cycle15 の
`0a000000` ほど Fridge 本体に寄った track は同じ形では再現しなかった。offset 別の debug status 数は
`yaw-18:105`, `yaw0:116`, `yaw+18:124`。target 全体の selected/accepted は引き続き
`couch` / `potted plant` 優勢で、`refrigerator` accepted は出なかった。

修正後の raw crop 保存確認では `26 track x 3 yaw = 78` 件を保存できた。Fridge 近傍として
cycle16 で最も説明力があった `06000000` は以下:

| object | yaw | selected | accepted | 主な raw 候補 |
|---|---:|---|---|---|
| `06000000` | -18 | なし | なし | `potted plant 0.80` は `mask_center_overlap` reject |
| `06000000` | 0 | `potted plant 0.765` | `potted plant 0.765`, `bed 0.294` | `refrigerator` なし |
| `06000000` | +18 | なし | なし | `potted plant 0.83` は `center_tolerance` reject |

保存 crop の offline YOLO 比較:

| 対象 | weight | crops | refrigerator raw | refrigerator accepted | selected classes |
|---|---|---:|---:|---:|---|
| 全 offset crop | `yolov8s-seg.pt` | 78 | 15 | 0 | `potted plant:16, couch:12, vase:4, dining table:4, bench:4, bed:3, chair:2` |
| 全 offset crop | `yolov8m-seg.pt` | 78 | 24 | 0 | `couch:19, potted plant:15, suitcase:10, vase:5, bench:4, dining table:3, bed:2, tv:1` |
| `06000000` のみ | `yolov8s-seg.pt` | 3 | 1 | 0 | `potted plant:1` |
| `06000000` のみ | `yolov8m-seg.pt` | 3 | 1 | 0 | `potted plant:1` |

成果物:

- `maps/indoor_recognition_cycle16_fridge_offset_nav.md/json/csv`
- `maps/indoor_recognition_cycle16_fridge_offset_recorder.md/json/csv`
- `maps/indoor_recognition_cycle16_fridge_offset_crop_save_nav.md/json/csv`
- `maps/indoor_recognition_cycle16_fridge_offset_crops/metadata.jsonl`
- `maps/indoor_recognition_cycle16_fridge_offset_crops/*.png`
- `maps/indoor_recognition_cycle16_fridge_offset_yolo_compare_all.md/json/csv`
- `maps/indoor_recognition_cycle16_fridge_offset_yolo_compare_060.md/json/csv`

採用/未採用:

- 採用: crop yaw/pitch offset を controlled comparison 用 launch 引数として追加する。既定は空文字で
  従来通り center crop だけなので、通常巡回挙動は変えない。
- 採用: debug diagnostics / recorder / raw crop metadata に offset を残す。Fridge のような低成績箇所で、
  crop 条件を track id 別に比較できる。
- 採用: debug crop 保存の interval key 修正。offset 違いが保存されないと offline 比較が壊れるため、
  診断基盤として採用する。
- 未採用: `object_crop_yaw_offsets_deg=-18,0,18` の既定化。全 crop / 近傍 track とも
  `refrigerator accepted=0` で、誤候補と推論負荷だけが増える。
- 未採用: `yolov8m-seg.pt` の既定化。全 offset crop で refrigerator raw は増えたが accepted は `0`。
- 未採用: `object_yolo_conf=0.01`, `static_margin=0`, `min_hits=1` の既定化。今回も診断入力であり、
  通常巡回 F1 / extra を評価していない。

次に見る低成績箇所:

1. 単純な yaw offset では Fridge 本体の `refrigerator accepted` は増えなかった。次は
   track shape (`0.6m x 0.6m x 1.7m` 程度) を画像へ投影し、中心点 crop ではなく shape projection
   から crop window / FOV を決める。
2. Fridge track は live ごとに `0a000000` / `06000000` など対応が揺れる。次の評価では recorder を
   classifier 起動直後から走らせるか、対象 track の crop 保存を target waypoint 到達後にも明示的に
   再実行して、同一 track で比較する。
3. COCO YOLO の raw refrigerator は出るが mask/center gate に乗らない。shape projection 後も
   accepted が出なければ、固定クラスの軽量 custom classifier の小規模比較へ進む。

#### 2026-06-22 追加改善26（shape-height crop center 診断、部分採用）

追加改善25の次に見る低成績箇所は、Fridge 本体 track の中心点 crop が `potted plant` に吸われる問題だった。
方針決定前に、追加改善25と同じ一次情報/公式情報を再確認した:

- Autoware の image projection based fusion は LiDAR object / cluster と画像 ROI を対応付けて
  classification/detection を refine する。Fridge 診断でも、2D YOLO 候補単体ではなく
  3D track の投影条件を変えて比較する。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/>
- Autoware Universe の roi cluster fusion は 3D cluster を画像へ投影し、重なる ROI で label を付ける。
  今回は簡易段階として、3D shape の高さ方向から複数の crop 中心を作り、full 3D bbox projection の前に
  crop 中心の高さずれを切り分ける。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/docs/roi-cluster-fusion/>
- Autoware.Auto の ROI associator は 3D detection と image ROI の association を明示的に扱う。
  target 全体集計ではなく、Fridge 最近傍 track `06000000` の crop を個別に読む。
  <https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/tracking-roi-associator.html>
- Ultralytics の predict mode は `conf` を推論候補しきい値として扱うため、今回も
  `object_yolo_conf=0.01` と `object_min_accept_conf=0.15` を分けた診断条件にする。
  <https://docs.ultralytics.com/modes/predict/>

実装:

- `object_classifier_node.py` に `crop_shape_center_height_fracs` を追加した。空なら従来どおり
  `0.0` のみ。指定時は track pose の `z` に `shape.dimensions.z * frac` を足した点を camera frame へ
  変換し、同一 FOV/yaw/pitch で複数の高さ crop を作る。
- `/perception/object_classifier/debug`、raw crop `metadata.jsonl`、`record_recognition_debug.py` の crop key に
  `crop_shape_height_frac` と `crop_center_xyz` を出す。crop ファイル名も `_h0.50` のように高さを含める。
- `webots_simulation.launch.py` / `webots_nav.launch.py` / `webots_waypoint_nav.launch.py` に
  `object_crop_shape_center_height_fracs` を通した。既定は空文字なので通常巡回の挙動は変えない。

ライブ評価:

条件: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, Fridge 専用 waypoint,
`object_tracker_min_hits=1`, `object_tracker_wall_margin_static_cells=0`,
`object_yolo_conf=0.01`, `object_min_accept_conf=0.15`,
`object_crop_shape_center_height_fracs=0,0.5,0.75`。
これは Fridge 診断用で、通常巡回の採用値ではない。

| waypoint | nav elapsed | recorder elapsed | target tracked | debug | accepted | selected | fine | nearest | near object | near shape |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| `1/1` | `34.291s` | `120.002s` | 0 | 0 | 0 | 0 | 4085 | `1.36m` | `06000000` | `0.38x0.62x1.70` |

recorder は classifier 起動後ではなく waypoint 到達後に開始したため、debug status は拾えなかった。
一方、raw crop metadata は `24 track x 3 height = 72` 件保存でき、Fridge 最近傍 `06000000` では
`h=0.0/0.5/0.75` のすべてで selected が `potted plant` だった。

保存 crop の offline YOLO 比較:

| 対象 | weight | crops | refrigerator raw | refrigerator accepted | selected classes |
|---|---|---:|---:|---:|---|
| 全 height crop | `yolov8s-seg.pt` | 72 | 5 | 0 | `potted plant:31, couch:11, vase:8, bed:1, bench:1` |
| 全 height crop | `yolov8m-seg.pt` | 72 | 6 | 0 | `potted plant:29, couch:14, vase:7, suitcase:3, bed:2, dining table:1, chair:1, bench:1` |
| `06000000` のみ | `yolov8s-seg.pt` | 3 | 2 | 0 | `potted plant:3` |
| `06000000` のみ | `yolov8m-seg.pt` | 3 | 1 | 0 | `potted plant:3` |

高さ別では全 crop の raw refrigerator は `yolov8s`: `h0=2, h0.5=1, h0.75=2`,
`yolov8m`: `h0=2, h0.5=2, h0.75=1` で、accepted は全高さで `0` だった。

成果物:

- `maps/indoor_recognition_cycle17_fridge_shape_crop_nav.md/json/csv`
- `maps/indoor_recognition_cycle17_fridge_shape_crop_recorder.md/json/csv`
- `maps/indoor_recognition_cycle17_fridge_shape_crop_crops/metadata.jsonl`
- `maps/indoor_recognition_cycle17_fridge_shape_crop_crops/*.png`
- `maps/indoor_recognition_cycle17_fridge_shape_crop_yolo_compare_all.md/json/csv`
- `maps/indoor_recognition_cycle17_fridge_shape_crop_yolo_compare_060.md/json/csv`

採用/未採用:

- 採用: `crop_shape_center_height_fracs` を controlled comparison 用 launch 引数として追加する。
  既定は空文字で従来 crop のみなので、通常巡回の挙動は変えない。
- 採用: debug diagnostics / raw crop metadata / recorder crop key に高さ crop 情報を残す。Fridge のように
  3D shape は取れているが画像分類が外れる対象で、track id 別に crop 条件を比較できる。
- 未採用: `object_crop_shape_center_height_fracs=0,0.5,0.75` の既定化。全 crop / 近傍 track とも
  `refrigerator accepted=0` で、誤候補と推論負荷だけが増える。
- 未採用: `yolov8m-seg.pt` の既定化。全 height crop でも raw は増えるが accepted は `0`。
- 未採用: `object_yolo_conf=0.01`, `static_margin=0`, `min_hits=1` の既定化。今回も診断入力であり、
  通常巡回 F1 / extra を評価していない。

次に見る低成績箇所:

1. 高さ方向に crop 中心をずらしても Fridge 本体の `refrigerator accepted` は増えなかった。次は
   3D bbox の 8 corners を画像へ投影して、中心点 crop ではなく shape-aware window / FOV を決める。
2. recorder は今回も classifier 初期 crop を拾えなかった。次の live では recorder を launch 直後から
   同時起動するか、classifier 側で target waypoint 到達後にも debug status を定期 publish する。
3. shape-aware window 後も accepted が出なければ、COCO YOLO の汎用分類だけで Fridge を拾う方針を
   いったん止め、固定クラスの軽量 custom classifier の小規模比較へ進む。

#### 2026-06-22 追加改善27（3D bbox corner projection crop 診断、部分採用）

追加改善26の次に見る低成績箇所は、Fridge 本体 track の 3D bbox corner projection から
shape-aware crop window / FOV を決めることだった。方針決定前に以下を確認した:

- Autoware の image projection based fusion は 2D image の bounding box / segmentation と、
  LiDAR 由来の point cloud / obstacle / bounding box / cluster を統合して obstacle detection を
  refine する。今回も LiDAR track の 3D bbox を画像方向へ投影し、crop 条件を比較する。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/>
- Autoware Universe の roi cluster fusion は cluster を image plane へ投影し、2D detector ROI と
  overlap したときに label を上書きする。中心点だけでなく 3D cluster/bbox の投影範囲を使うのが
  既知の基本形なので、bbox corners から角度範囲を取る。
  <https://autowarefoundation.github.io/autoware_universe/main/perception/autoware_image_projection_based_fusion/docs/roi-cluster-fusion/>
- Autoware.Auto の ROI associator は 3D object を画像平面へ投影し、IoU heuristic で ROI と対応付ける。
  Fridge 診断では IoU fusion までは入れず、まず crop window を 3D bbox 投影由来にする。
  <https://autowarefoundation.gitlab.io/autoware.auto/AutowareAuto/tracking-roi-associator.html>
- Ultralytics の predict mode は `conf` を推論候補しきい値として扱う。今回も raw 候補確認用に
  `object_yolo_conf=0.01`、採用判定は `object_min_accept_conf=0.15` のまま分ける。
  <https://docs.ultralytics.com/modes/predict/>

実装:

- `object_classifier_node.py` に `crop_shape_bbox_margins_deg` を追加した。空なら無効。指定時は
  track pose + `shape.dimensions` から floor-origin の 3D bbox 8 corners を作り、camera frame へ変換する。
- bbox 中心方向を基準に各 corner の yaw/pitch 角度範囲を計算し、`2 * (最大半角 + margin)` を
  perspective crop FOV として使う。FOV は `10..140deg` に clamp する。
- debug diagnostics / raw crop `metadata.jsonl` / recorder crop key に
  `crop_mode=bbox`, `crop_shape_bbox_margin_deg`, `crop_shape_bbox_projected_fov_deg` を残す。
- `webots_simulation.launch.py` / `webots_nav.launch.py` / `webots_waypoint_nav.launch.py` に
  `object_crop_shape_bbox_margins_deg` を通した。既定は空文字なので通常巡回の挙動は変えない。

ライブ評価:

条件: `RMW_IMPLEMENTATION=rmw_cyclonedds_cpp`, Fridge 専用 waypoint,
`object_tracker_min_hits=1`, `object_tracker_wall_margin_static_cells=0`,
`object_yolo_conf=0.01`, `object_min_accept_conf=0.15`,
`object_crop_shape_bbox_margins_deg=0,4,10`。
これは Fridge 診断用で、通常巡回の採用値ではない。

| waypoint | nav elapsed | recorder elapsed | target tracked | debug | accepted | selected | fine | nearest | near object | near shape |
|---:|---:|---:|---:|---:|---:|---:|---:|---:|---|---|
| `1/1` | `31.364s` | `135.001s` | 15 | 597 | 29 | 308 | 5606 | `0.22m` | `08000000` | `0.36x0.64x1.70` |

今回の run では Fridge 最近傍 track `08000000` が `0.22m` まで出ており、追加改善26より
本体 track に近い条件で比較できた。live selected/accepted は引き続き `potted plant` 優勢だった。
保存 crop は `22 track x (center + bbox margin 0/4/10) = 88` 件。bbox projected FOV は
track により `13..111deg` 程度になった。

保存 crop の offline YOLO 比較:

| 対象 | weight | crops | refrigerator raw | refrigerator accepted | selected classes |
|---|---|---:|---:|---:|---|
| 全 crop | `yolov8s-seg.pt` | 88 | 9 | 0 | `potted plant:22, vase:5, couch:4, bench:3, chair:2, book:2` |
| 全 crop | `yolov8m-seg.pt` | 88 | 8 | 0 | `potted plant:26, couch:5, vase:5, chair:4, suitcase:3, bench:2, bed:1` |
| `08000000` のみ | `yolov8s-seg.pt` | 4 | 0 | 0 | `potted plant:3` |
| `08000000` のみ | `yolov8m-seg.pt` | 4 | 1 | 0 | `potted plant:3` |

条件別の refrigerator raw / accepted:

| weight | center | bbox margin 0 | bbox margin 4 | bbox margin 10 |
|---|---|---|---|---|
| `yolov8s-seg.pt` | raw `4`, accepted `0` | raw `2`, accepted `0` | raw `2`, accepted `0` | raw `0`, accepted `0` |
| `yolov8m-seg.pt` | raw `4`, accepted `0` | raw `1`, accepted `0` | raw `1`, accepted `0` | raw `1`, accepted `0` |

`08000000` の `yolov8m` raw refrigerator は center crop のみで、`conf=0.030` の
`min_accept_conf` reject だった。bbox crop は Fridge 本体近傍でも `potted plant` を抑えきれず、
`refrigerator accepted` は増えなかった。

成果物:

- `maps/indoor_recognition_cycle18_fridge_bbox_crop_nav.md/json/csv`
- `maps/indoor_recognition_cycle18_fridge_bbox_crop_recorder.md/json/csv`
- `maps/indoor_recognition_cycle18_fridge_bbox_crop_crops/metadata.jsonl`
- `maps/indoor_recognition_cycle18_fridge_bbox_crop_crops/*.png`
- `maps/indoor_recognition_cycle18_fridge_bbox_crop_yolo_compare_all.md/json/csv`
- `maps/indoor_recognition_cycle18_fridge_bbox_crop_yolo_compare_080.md/json/csv`

採用/未採用:

- 採用: `crop_shape_bbox_margins_deg` を controlled comparison 用 launch 引数として追加する。
  既定は空文字で従来 crop のみなので、通常巡回の挙動は変えない。
- 採用: debug diagnostics / raw crop metadata / recorder crop key に bbox 投影 crop 情報を残す。
  今後の低成績診断で、中心点 crop と bbox 投影 crop を同じ track id で比較できる。
- 未採用: `object_crop_shape_bbox_margins_deg=0,4,10` の既定化。全 crop / 最近傍 track とも
  `refrigerator accepted=0` で、誤候補と推論負荷だけが増える。
- 未採用: `yolov8m-seg.pt` の既定化。最近傍 track で raw refrigerator は 1 件出たが
  `conf=0.030` の reject で、accepted は `0`。
- 未採用: `object_yolo_conf=0.01`, `static_margin=0`, `min_hits=1` の既定化。今回も診断入力であり、
  通常巡回 F1 / extra を評価していない。

次に見る低成績箇所:

1. Fridge は 3D bbox 投影 crop でも accepted が出ない。次は COCO YOLO の汎用分類だけで Fridge を拾う
   方針をいったん止め、固定クラスの軽量 custom classifier の小規模比較へ進む。
2. `08000000` は shape `0.36x0.64x1.70` で本体に近いが、画像上は potted plant として強く出る。
   custom classifier 比較では、この crop 群を hard negative / positive サンプルとして使う。
3. bbox 投影 crop は診断基盤として残すが、通常巡回では `yolov8s-seg.pt` + 既定 center crop を維持する。

#### 2026-06-22 方針整理（未採用の追加診断を削除）

追加改善27の後に試した言語ラベル比較系の offline 診断と関連評価は、ユーザー方針により削除した。
ランタイム既定には採用せず、今後の候補にも含めない。保存 crop と recorder を使う評価プロトコルは
引き続き有効なので、固定クラス分類器の検証に流用する。

採用/未採用:

- 採用: 通常巡回 center crop + target association の評価プロトコル。診断 waypoint だけで採用判断しないための
  比較基準として使う。
- 未採用: 言語ラベル比較の runtime fallback、緩いしきい値、全 crop fallback。
- 削除: 専用 evaluator、install entry、関連生成ファイル。

次に見る低成績箇所:

1. Fridge positive crop が `cabinet` / `dining table` / `chair` 相当に寄る原因を、
   hard positive/negative crop と固定クラスの軽量 custom classifier で切り分ける。
2. Fridge 形状、壁近傍、高さ、map support を事前条件に使い、候補 track を絞った上で TP/FP を測る。
3. Webots Fridge 専用の synthetic positive/negative crop 生成を比較候補にする。

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

4. **認識結果を地図上でレビューできる**
   `maps/<world>_recognition_overlay.png` に認識結果がラベル付きで保存される。各ラベルは地図上の物体位置を
   指し、ID・クラス名・存在確率・観測回数が読めること。小さい地図では描画倍率を上げ、ラベルが潰れた
   状態の PNG を合格成果物にしない。

5. **world 真値との比較結果を残す**
   `maps/<world>_recognition_eval.md` に、world 上の評価対象物体数、検出数、正解マッチ、ラベル誤り、
   未検出、余分な検出、precision / recall / F1 が保存される。`maps/<world>_recognition_eval.png` では
   TP / ラベル誤り / 未検出 / 余分な検出が地図上で区別できること。COCO 既定分類器の語彙外で評価から
   外した world 物体は、スキップ理由を同レポートに残す。

6. **予測 costmap が Nav2 を壊していない**
   `/perception/predicted_costmap` が毎フレーム置換で出る。`PredictedCostmapLayer` は max 合成で他層を壊さず、
   static の壁を消さない。STVL 層へ戻さない。

7. **メッセージ型は既存型を使う**
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
