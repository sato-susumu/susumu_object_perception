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
`object_crop_fovs_deg:=''`（単一 `crop_fov_deg`）にする。
分類ゲートの調査時は `object_classifier_debug:=True` を付けると、
`/perception/object_classifier/debug` に YOLO 候補ごとの採否理由（bbox 面積、中心ずれ、mask overlap、
植物色比率など）が `diagnostic_msgs/DiagnosticArray` で出る。これは次の改善で、正解候補をどの
ゲートが落としているかを bag / echo で確認するための診断用で、既定は False。

### 最終成果物（地図への認識結果重畳）

認識タスクの最後に、`object_memory_node.py` が保存した SQLite DB を保存地図へ重ね、
物体 ID・ラベル・存在確率・観測回数付き PNG を作る。地図が小さくラベル領域が足りない場合、
`render_recognition_overlay.py` は既定で地図を読みやすい大きさに自動拡大する。必要なら
`--scale` で倍率を明示する。

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
  -p map_support_dist:=0.55 \
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
オンライン登録時の `map_support_dist` を
0.45m まで詰めると正しい候補も蓄積前に落ちたため、巡回中は 0.55m で recall を保ち、最終成果物の前に
次の地図・幾何フィルタでDBを整理する。

```bash
ros2 run susumu_object_perception filter_object_memory_db.py \
  --db /tmp/indoor_object_memory.sqlite3 \
  --out-db /tmp/indoor_object_memory_pruned.sqlite3 \
  --map maps/indoor.yaml \
  --map-support-dist 0.45 \
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
