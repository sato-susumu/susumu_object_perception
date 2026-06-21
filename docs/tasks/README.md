# タスク別ドキュメント

このディレクトリは、README のタスク一覧に対応する詳細ページの入口。各ページがタスクごとの
目的・入出力・実行手順・合格基準・制約の正本になる。README / AGENTS.md は索引に留め、
条件や採用値を変えたら該当タスクページを更新する。

## 現在地

| タスク | 状態 | 採用中の主成果物 / 条件 | 次に見る低成績箇所 | 詳細 |
|---|---|---|---|---|
| マッピング（屋内） | 運用対象 | `maps/indoor.yaml`、`maps/break_room.yaml`。評価実行は `mode:=realtime` | 地図品質が崩れた場合は衝突・`/scan`・SLAM 設定を切り分ける | [mapping_indoor.md](mapping_indoor.md) |
| マッピング（屋外） | 方針転換中 | 屋外本線は **GLIMで3D点群を作る → trajectory条件を比較 → Nav2用2D地図化 → waypoint生成**。cycle25 で plan / actual corridor 監視を採用。cycle26 で `expand_waypoint_route.py` を診断・候補生成用に追加。cycle27 で outdoor-only `safe_pose_guard` を追加したが、Nav2 goal で安全姿勢へ戻す方式は live 悪化のため既定未採用。cycle21-24 の各局所対策も既定未採用。world 由来 `*_gt.yaml` は評価専用 | 次は lethal pose に入った後の Nav2 再計画ではなく、lethal 前の経路ブラックリスト化、または短時間の直接制御 escape を屋外専用に評価する。cycle27 は `reached=11/96` と `9/96` で停止 | [mapping_outdoor.md](mapping_outdoor.md) |
| ウェイポイント生成 | 屋内採用済み | `maps/indoor_waypoints.yaml`。確認 PNG も同時生成 | 認識用追加視点はナビ完走しても認識が悪化したため未採用 | [waypoint_generation.md](waypoint_generation.md) |
| 巡回ナビ | 屋内採用済み | `indoor.wbt` + `indoor_waypoints.yaml` は `reached=22/22 missed=[]` | missed が出たら waypoint 生成、地図、Nav2 tuning の順で見る | [waypoint_navigation.md](waypoint_navigation.md) |
| 認識 | 改善継続中 | 屋内採用値は `yolov8s-seg.pt` + mask/色ゲート + 座席統合。Table/Sofa 除外 F1 `0.727` | 未検出の Fridge / PottedTree / BunchOfSunFlowers。multi-FOV/imgsz 960 は未採用 | [recognition.md](recognition.md) |
| カラー点群出力 | 基本機能あり、精密較正は課題 | `/perception/colorized_points`、`/slam/*colorized_points_map`、PLY 保存 | 投影誤差 1deg 未満を断言できない。realtime 4方向 validation と較正入力の整備 | [colorized_pointcloud.md](colorized_pointcloud.md) |

## 読む順

初めて触る場合は次の順で読む。

1. [mapping_indoor.md](mapping_indoor.md): 屋内地図を作る。屋外設定を屋内へ混ぜない制約もここ。
2. [waypoint_generation.md](waypoint_generation.md): 保存地図から巡回点列と確認 PNG を作る。
3. [waypoint_navigation.md](waypoint_navigation.md): Nav2 で一周完走させる。
4. [recognition.md](recognition.md): 巡回しながら物体・信号を認識し、地図重畳と world 評価を残す。
5. [colorized_pointcloud.md](colorized_pointcloud.md): 全天球画像で LiDAR 点群に色を付け、必要なら地図として保存する。

## 成果物の扱い

| 成果物 | 作るタスク | 使うタスク |
|---|---|---|
| `maps/<world>.yaml` / `.pgm` または `maps/<world>_glim2d.yaml` / `.pgm` | マッピング | ウェイポイント生成、Nav2、認識評価、カラー点群レビュー |
| `maps/<world>_gt.yaml` / `.pgm` | 屋外マッピング評価 | SLAM 地図の評価のみ。Nav2 や waypoint 生成には使わない |
| `maps/<world>_waypoints.yaml` / `.png` | ウェイポイント生成 | 巡回ナビ、認識、カラー点群記録 |
| `maps/<world>_recognition_overlay.png` | 認識 | 認識レビュー |
| `maps/<world>_recognition_eval.{md,json,csv,png}` | 認識 | 採用/未採用判断 |
| `maps/colorized/*.ply` | カラー点群出力 | 点群レビュー、外部可視化 |

## 更新ルール

- 合格基準、制約、採用パラメータ、未採用実験の理由は該当タスクページに残す。
- `config/nav2_params.yaml` を変えたら [../nav2_tuning.md](../nav2_tuning.md) も更新する。
- 方針決定や詰まり時にネット調査した場合は、参照先と判断理由をタスクページか関連 docs に残す。
- 実験結果は「ナビ結果」「評価指標」「採用/未採用」「次に試す条件」が分かる形で追記する。
