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
```

CPU が厳しい場合は `image_recognition:=False` で YOLO 物体分類と全天球信号認識を切る。LiDAR
perception は残る。Webots 系 launch では `object_yolo_weights:=...` で
`object_classifier_node.py` の YOLO weight を差し替えられる。

屋内認識の採用既定:

| 項目 | 採用値 / 方針 |
|---|---|
| YOLO weight | `yolov8s-seg.pt` |
| 画像サイズ / crop | `object_yolo_imgsz:=640`、単一 `crop_fov_deg` |
| segmentation gate | `require_mask_center:=True` |
| 植物色 gate | `plant_color_min_frac` を併用 |
| YOLO しきい値 | `object_yolo_conf:=0.15`、`object_min_accept_conf:=0.15` |
| tracker wall margin | 既定 static margin は維持。Fridge 診断では controlled comparison 用に launch 引数で切替 |
| semantic DB | `require_fine_class:=True`、`require_map_support:=True`、`static_class_geometry_filter:=True` |
| map support | 既定 `0.45m`、class 別 `plant=0.55,table=0.55` |
| 座席統合 | `chair,couch` を互換統合、優先順 `chair,couch` |

## 最終成果物

認識タスクの最後に、`object_memory_node.py` が保存した SQLite DB を保存地図へ重ね、
物体 ID・ラベル・存在確率・観測回数付き PNG を作る。地図が小さくラベル領域が足りない場合、
`render_recognition_overlay.py` は既定で地図を読みやすい大きさに自動拡大する。

```bash
ros2 run susumu_object_perception validate_map_assets.py maps/indoor.yaml

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

認識巡回と同時に DB を記録する例:

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

最終整理:

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

`visible_range:=0.0` は認識レビュー向け設定。巡回中に一度認識した物体を累積して残すため、
negative observation を実質無効化する。通常の物体メモリ運用ではこの限りではない。

## 最終評価

`evaluate_recognition_vs_world.py` は Webots world の静的物体配置と認識 DB を照合し、Markdown / JSON /
CSV / PNG を出す。world 真値は **検証だけ** に使い、認識本体や DB 整理には使わない。

```bash
ros2 run susumu_object_perception evaluate_recognition_vs_world.py \
  --wbt webots_worlds/indoor.wbt \
  --map maps/indoor.yaml \
  --db /tmp/indoor_object_memory_pruned.sqlite3 \
  --out-prefix maps/indoor_recognition_eval \
  --min-existence 0.5 \
  --min-hits 2 \
  --match-distance 1.0
```

`--match-distance` は world 真値と map/SLAM/検出位置のずれを許容する距離ゲート[m]。既定は `1.0m`。
`--map` を渡すと、評価対象 world object ごとに保存地図の最寄り occupied セル距離も出る。
評価対象を一時的に外す場合は `--ignore-type Sofa --ignore-type Table` のように指定する。

## 履歴サマリ

詳細な個別サイクルログは長くなりすぎるため、判断に必要な要点だけ残す。

### 採用済み

- 屋内フル巡回は `indoor.wbt` + `indoor_waypoints.yaml` + `mode:=realtime` を採用条件にする。
  追加視点入り `indoor_recognition_waypoints.yaml` はナビ完走できても認識 F1 が悪化したため採用条件にしない。
- `yolov8s-seg.pt` + segmentation mask gate + 植物色 gate を屋内既定にする。
- COCO 細クラスは `/perception/object_fine_classes` で DB に渡し、`object_memory_node.py` 側で
  `chair` / `couch` / `dining table` / `potted plant` 等を記憶する。
- 最終 DB 整理では map support、静的幾何フィルタ、同一/互換クラス統合を使う。
- `maps/*.pgm` はすべて commit 対象。YAML だけでは後段の map support / overlay / Nav2 が再現できない。
- debug recorder、crop 保存、stage/tracker 診断、track id association、crop offset/height/bbox 診断は
  **診断基盤**として採用する。通常巡回の採用判断は `indoor_waypoints.yaml` の評価で行う。

### 未採用

- `yolov8m-seg.pt` の既定化。Fridge の raw 候補は増える場合があるが、通常巡回で余分検出や誤分類が増えた。
- multi-FOV / imgsz 960 の既定化。合意選択や map support と組み合わせても採用値に届かなかった。
- 認識向け追加視点を通常採点に入れること。到達性を改善しても、クロップ背景や余分検出が増えた。
- `object_yolo_conf` / `object_min_accept_conf` の恒久的な引き下げ。低信頼 raw candidate を増やす診断には使うが、
  採用候補としては false positive リスクが高い。
- tracker wall margin や min_hits の無条件緩和。Fridge 専用 run の改善だけでは、壁際静止ゴースト対策を壊す。
- 言語ラベル比較、LLM、open-vocabulary、CLIP、class synonym 辞書、物体検索/追従の runtime fallback。
  ユーザー方針により削除済みで、今後の候補にも含めない。

### 代表値

| 条件 | expected | detections | correct | wrong_label | extra | precision | recall | F1 |
|---|---:|---:|---:|---:|---:|---:|---:|---:|
| 屋内初期ベースライン 全対象 | 9 | 4 | 3 | 1 | 0 | 0.750 | 0.333 | 0.462 |
| batch YOLO + cache 修正 + DB整理 全対象 | 9 | 5 | 4 | 0 | 1 | 0.800 | 0.444 | 0.571 |
| class別 map support 代表 run 全対象 | 9 | 7 | 4 | 3 | 0 | 0.571 | 0.444 | 0.500 |
| class別 map support 代表 run Table/Sofa 除外 | 7 | 5 | 3 | 2 | 0 | 0.600 | 0.429 | 0.500 |
| 通常巡回の採用目安 Table/Sofa 除外 | 7 | 4 | 4 | 0 | 0 | 1.000 | 0.571 | 0.727 |

評価値は run 条件・対象除外条件で変わる。採用判断では Markdown/JSON/CSV/PNG の成果物を残し、
単発の debug waypoint 結果だけで既定値を変えない。

### 次に見る低成績箇所

1. 通常巡回 Fridge positive crop が `cabinet` / `dining table` / `chair` 相当に寄る原因を、
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
- world 真値は検証専用。認識本体や採用候補生成へ入力しない。

## 関連

- [認識パイプライン詳細](../autoware_perception.md)
- [信号認識](../traffic_light_recognition.md)
- [全天球カメラ・LiDAR色付き点群メモ](../omni_lidar_camera.md)
- [ノード接続図](../node_topology.md)
