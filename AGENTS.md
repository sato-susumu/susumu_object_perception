# susumu_object_perception — AGENTS

AIエージェント（および新規参加者）向けの作業ガイド。このパッケージで変更を加えるときの
前提・規約・落とし穴をまとめる。設計（全体構造・状態遷移・シーケンス図）は
[`docs/software_design.md`](docs/software_design.md)、詳細な構築履歴は
[`SETUP.md`](SETUP.md)、利用方法は [`README.md`](README.md) を参照。

## 何のパッケージか

ROS 2 Humble + **Gazebo Classic 11** 上の**シミュレーター**統合パッケージ。

- **cafe world**（既定）で **HuNavSim が5人の歩行者**を通常歩行速度で歩かせる
  （house world は歩行者が固着しやすく非推奨。SETUP.md「Phase H」参照）
- **3D LiDAR(VLP-16相当) のみ搭載 TurtleBot3** を spawn（2D LiDAR は非搭載）
- **Nav2** で自律移動（`/scan` + `/velodyne_points` を costmap の障害物に使用。
  `/scan` は 3D 点群から pointcloud_to_laserscan で生成）
- **Teleop / 自動巡回 GUI**（`teleop_gui_node.py`）で手動操縦・自動巡回
- **Autoware LiDAR sensing/perception パイプライン**（既定 ON）。`/velodyne_points` を
  Autoware 純正の crop_box → ground_filter → euclidean_cluster で検出し、Python 自作の
  `object_tracker_node.py`（追跡）→ `perception_marker_node.py`（可視化）で補完する。
  詳細は [`docs/autoware_perception.md`](docs/autoware_perception.md)。

> シミュレーターに **Autoware 互換の perception** を載せた構成。検出までは Autoware
> モジュール、apt に無い追跡/形状推定は Python で自作補完する（自作時は Autoware
> 公式ソースを参照し設計・既定値を踏襲する方針）。検出・追跡結果は RViz 可視化が主だが、
> **prediction の予測のみ Nav2 costmap に連携する**（`prediction_node` が「人がこれから行く先」を
> OccupancyGrid `/perception/predicted_costmap` として毎フレーム作り直して出し、**自作 C++
> costmap 層 `susumu_object_perception::PredictedCostmapLayer` が max 合成で costmap に乗せる**）。検出・追跡
> そのものは costmap に焼かず、Nav2 は現在位置の障害物回避を従来どおり生センサ `/scan`
> `/velodyne_points`（STVL層）で行う。HD 地図は使わず点群ジオメトリのみで検出する。
> （※ 標準層では実現できず自作 C++ 層が必要だった: 点群方式=ObstacleLayer/STVL は古い予測が
> 蓄積し costmap がぐちゃぐちゃ、StaticLayer は壁を上書き消去。max 合成の自作層で「他層を
> 壊さず・蓄積せず」を両立。真値検証で壁 LETHAL 100%維持・全体 22%・進路前方占有 58%(0.5m)・
> ナビ可能。詳細は docs/autoware_perception.md「Nav2 連携」。）
>
> 旧来の追従機能（`person_detector_node` / `follow_person_node`）は削除済みで、別
> パッケージ `susumu_lidar_perception` にあったが、**他ブランチ・別パッケージの過去
> 実装は参照しない**（本 perception は main からクリーンに再実装したもの）。

ビルド種別は **`ament_cmake`**（Pythonノードは `install(PROGRAMS)` +
`ament_python_install_package` で同梱）。

## 絶対に守る制約・方針

- **`git commit` と `git push` は勝手にしない。** ユーザーが明示的に「commit して」
  「push して」と指示したときだけ実行する。それ以外は変更を作業ツリーに残すだけにとどめ、
  勝手にコミット・プッシュしない（ブランチを切る等の判断も同様にユーザーに委ねる）。
- **Gazebo は Classic 11**。Ignition/Gazebo Sim ではない。
  - HuNavSim は必ず **`v1.0-humble`** ブランチを使う（`v2.0` は Gazebo Sim 用で動かない）。
- **独自メッセージは作らない**。標準型のみ（`Twist` / `PoseWithCovarianceStamped` /
  `nav2_msgs/NavigateToPose`）。
- ワークスペースの source は **`install/local_setup.bash`** を使う。
  `install/setup.bash` は**古いスナップショットを指す prefix-chain** で、新規パッケージが
  見えず `package not found` になる（既知の罠。SETUP.md「Phase B」参照）。
- **Nav2（`config/nav2_params.yaml`）を調整したら、必ず
  [`docs/nav2_tuning.md`](docs/nav2_tuning.md) の「現在値」表と「調整履歴」を更新する。**
  値だけ変えてドキュメントを放置しない（理由が失われ次の調整で振り出しに戻る）。

## ビルド・実行

```bash
cd ~/ros2_ws
colcon build --packages-select susumu_object_perception --symlink-install

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash     # ← setup.bash ではない
export TURTLEBOT3_MODEL=waffle

ros2 launch susumu_object_perception simulation.launch.py              # 全部入り（GUI含む）
ros2 launch susumu_object_perception simulation.launch.py gui:=false   # GUI無効
```

Pythonノードはファイル名で起動する（console_scripts ではない）:
`ros2 run susumu_object_perception teleop_gui_node.py`。
→ ノードを増やすときは CMakeLists の `install(PROGRAMS ...)` に**ファイルを追加し、
かつソースに実行ビット(`chmod +x`)を立てる**こと（忘れると `No executable found`）。

## アーキテクチャ / データフロー

エントリは `launch/simulation.launch.py`。取り込まれる部品 launch は `launch/include/`。

```
include/hunav_house.launch.py : hunav_loader → hunav_gazebo_world_generator
                        → generatedWorld.world（actor5体+HuNavプラグイン）→ gzserver/gzclient
                        + hunav_agent_manager（SFM behavior駆動）
include/spawn_robot.launch.py : robot_state_publisher(URDF) + spawn_entity(SDF: 3D LiDAR等のプラグイン)
                       + pointcloud_to_laserscan(/velodyne_points → /scan)
nav2_bringup         : AMCL(/scan) + costmap(obstacle_layer=/scan,
                       predicted_layer=自作PredictedCostmapLayer: prediction の予測
                       OccupancyGrid /perception/predicted_costmap を max 合成）
                       + planner/controller
                       ※ STVL層は廃止（人の通過跡が voxel_decay 秒残り「移動軌跡の
                         コスト」が出たため。人の現在位置は予測層が焼く）
teleop_gui_node.py   : Twist→/cmd_vel（手動）/ NavigateToPose（部屋自動巡回）
                       / /initialpose（原点ワープ時のAMCL再初期化）
```

simulation.launch.py は上記を **TimerActionで段階起動**（gazebo→+8s robot→+12s nav2/rviz
→+15s gui）。順序依存（robotが居ないとnav2のTFが揃わない等）があるため、遅延値を
むやみに詰めない。

## フレーム/トピックの約束（変更時は両側を揃える）

| 役割 | 値 | 定義場所 |
|---|---|---|
| 速度司令 | `cmd_vel` | SDF diff_drive ↔ nav2 controller / teleop_gui |
| オドメトリ | frame/topic `odom`（`publish_odom_tf:true`） | SDF diff_drive ↔ amcl odom_frame |
| ベース | `base_footprint`(amcl) / `base_link`(costmap) | SDF / URDF / nav2_params |
| 2D スキャン | `/scan`, frame `velodyne_link` | **pointcloud_to_laserscan が /velodyne_points から生成**（2D LiDAR は非搭載）↔ amcl scan_topic ↔ nav2 obstacle_layer |
| 3D LiDAR | `/velodyne_points`, frame `velodyne_link` | SDF gpu_ray ↔ Autoware perception 入力 ↔ pointcloud_to_laserscan（→ /scan）。※ Nav2 costmap には現在 STVL を使わない（廃止） |
| HuNav追跡対象 | robot_name=`turtlebot3`（spawn entity名と一致必須） | hunav_house / spawn_robot |

## 重要ファイル

- `models/turtlebot3_waffle_3d/model.sdf` … Gazeboプラグイン本体。3D LiDARは `gpu_ray`
  センサ + `libgazebo_ros_ray_sensor.so`（`output_type: PointCloud2`）。
  変更後は `gz sdf -k model.sdf` で spec 検証する。
- `config/nav2_params.yaml` … waffle.yaml ベース。costmap の動的障害物層は自作
  **`predicted_layer`（`susumu_object_perception::PredictedCostmapLayer`）**。prediction が出す予測
  OccupancyGrid `/perception/predicted_costmap`（人の現在位置 + 進路先）を max 合成で焼く。
  obstacle_layer は生 `/scan`。**STVL（`stvl_layer`）は廃止**（人の通過跡が `voxel_decay`
  秒残り「移動軌跡のコスト」が出たため。人の現在位置は予測層が毎フレーム焼き直すので軌跡は
  残らない）。詳細は `docs/nav2_tuning.md`。
- `config/agents_house.yaml` … HuNav 5人。**公式 `hunav_gazebo_wrapper/scenarios/
  agents_house.yaml` のコピー**（動作実績あり）。通常歩行速度（`max_vel:1.5`,
  `vel:0.6`〜`0.8`, 各3ゴール）、**`once:true`+`cyclic_goals:true`** で巡回し続ける。
- `susumu_object_perception/teleop_gui_node.py` … tkinter GUI。矢印/テンキー手動操縦、AUTOトグルで
  `PATROL_WAYPOINTS` を Nav2 で巡回、WARPで原点へワープ＋AMCL再初期化。
- `launch/include/autoware_perception.launch.py` … Autoware 3 モジュール（crop_box →
  ground_filter → euclidean_cluster）を 1 component_container にまとめ、自作 tracker /
  marker を起動する perception パイプライン。plugin 名・remap は実体検証済み。
- `config/autoware_*.param.yaml` … 上記 Autoware モジュールの屋内向けパラメータ。
  ground/cluster を調整したら `docs/autoware_perception.md` のパラメータ表も更新する。
- `susumu_object_perception/shape_estimation_node.py` … OBB 形状推定。euclidean_cluster の検出は
  位置のみで shape が空なので、no_ground 点群から各検出近傍を集めて **Autoware の
  L字フィット（bounding_box.cpp の closeness criterion / 1°grid search）を踏襲**して
  OBB の寸法・向きを埋める。apt に shape_estimation は無く、universe 版は型
  （tier4_perception_msgs）が世代不整合なため、アルゴリズムのみ公式踏襲し型は標準で自作。
  入力 `/perception/detected_objects` → 出力 `/perception/detected_objects_shaped`。
- `susumu_object_perception/detection_by_tracker_node.py` … 過分割統合。euclidean が 1 人を複数に割る
  over-segmentation を、前フレームの tracker 位置・サイズを参照して統合（**Autoware
  Cluster Merger 踏襲**）。tracker 出力を購読する循環構造。**統合後 shape は包含 BBox では
  なく点群を L字フィット再推定**（包含だと巨大化する。`shape_estimation_node.fit_l_shape`
  を再利用）。入力 `detected_objects_shaped` → 出力 `detected_objects_merged`。
- `susumu_object_perception/object_tracker_node.py` … DetectedObjects→TrackedObjects の自作トラッカー。
  Autoware multi_object_tracker のソースを踏襲（ハンガリアン法 + マハラノビス χ²ゲート
  11.62 + existence_probability の Bayes 更新/半減期 decay + CV 速度クランプ）。
  **classification も 2D 地図で推定**: 出力段に来たトラック（=地図 free space に居る）が
  移動なら `PEDESTRIAN`、静止なら `UNKNOWN`（Autoware の HD マップ walkable-area 推定の
  2D 占有格子版）。
- `susumu_object_perception/prediction_node.py` … 将来軌跡予測。Autoware `map_based_prediction` の
  **2D 占有格子版**。tracked_objects を等速(CV)で予測し、予測点が 2D 地図の occupied
  セルに入ったら打ち切る（壁めり込み回避）。HD 地図要素の代わりに `/map` を使う。
  入力 `/perception/tracked_objects` → 出力 `/perception/predicted_objects`
  （PredictedObjects）。マルチモーダル通路追従は今後の拡張。
  **Nav2 連携**: 「人がこれから行く先」を OccupancyGrid `/perception/predicted_costmap`(map)
  として**毎フレーム作り直して**出す（最有力1本・近傍2s・人幅6セル円盤膨張、confidence 0.25
  以上）。これを自作 C++ 層 `susumu_object_perception::PredictedCostmapLayer` が max 合成で costmap に乗せる。
  詳細・標準層が使えなかった経緯は docs/autoware_perception.md「Nav2 連携」。
- `src/predicted_costmap_layer.cpp` / `include/susumu_object_perception/predicted_costmap_layer.hpp` …
  自作 C++ costmap_2d::Layer プラグイン（`susumu_object_perception::PredictedCostmapLayer`）。予測
  OccupancyGrid `/perception/predicted_costmap` を購読し、占有セルを **max 合成**で costmap に
  乗せる（他層を壊さず・毎フレーム置換で蓄積せず）。pluginlib 登録は `predicted_costmap_layer.xml`、
  CMakeLists で SHARED lib をビルド。nav2_params の `predicted_layer` で local/global 両 costmap に
  適用。**標準層（ObstacleLayer/STVL=蓄積、StaticLayer=他層上書き）では両立できなかったため自作**
  （リポジトリ初の C++ ノード。他は全て rclpy）。
- `susumu_object_perception/perception_marker_node.py` … Detected/Tracked/Predicted を MarkerArray 可視化
  （検出=青 / 移動=赤 / 静止=緑のボックス、`#ID 速度[km/h]` テキスト、速度矢印、
  予測パス=黄の LINE_STRIP）。spencer/leg_tracker の作法に倣う（トラック識別色・背景なし）。
  表示方法・色を自由に作り込むため自作（純正プラグインは不使用）。RViz では
  `/perception/markers` を MarkerArray Display で表示。

## 変更時の検証手順

Gazebo 起動は安定しているので、**基本はライブ起動で確認する**（下の「動作確認の作法」参照）。
ライブ起動の前後で、まず以下の静的検証を通しておくと早く落とせる:

```bash
# SDF/URDF/YAML/launch の静的検証
gz sdf -k models/turtlebot3_waffle_3d/model.sdf
xacro urdf/turtlebot3_waffle_3d.urdf.xacro > /dev/null
python3 -c "import yaml; yaml.safe_load(open('config/nav2_params.yaml'))"
ros2 launch susumu_object_perception simulation.launch.py --show-args   # launch記述のパース確認
```

## やりがちな失敗

- `install/setup.bash` を source して `package not found` → `local_setup.bash` を使う。
- 新Pythonノードに実行ビットを立て忘れて `No executable found`。
- HuNavSim を `v2.0` で入れて Gazebo起動失敗。
- robot spawn の entity名と HuNav の robot_name 不一致で `Robot model ... not found` が出続ける。
- **Nav2 paramsを `turtlebot3_navigation2` の waffle.yaml から作らない。**
  そのパッケージは新しいNav2の `::` プラグイン名形式で、インストール済み Nav2 1.1.20
  （`/` 形式）と不整合 → `planner_server` が `NavfnPlanner does not exist` で落ちる。
  **同梱バージョンと一致する `nav2_bringup/params/nav2_params.yaml` をベースにする**こと。
  現行の `config/nav2_params.yaml` は対処済み（planner=`nav2_navfn_planner/NavfnPlanner`）。
- **歩行者が動かない → `once` を疑う。** `agents_house.yaml` の `once: false` だと
  HuNav の behavior 駆動が回らず、ほとんどのエージェントが数十秒で停止する（過去の
  誤設定）。正しくは公式 house シナリオと同じ **`once: true` + `cyclic_goals: true`**
  （現行設定）。困ったら `hunav_gazebo_wrapper/scenarios/agents_house.yaml` を
  そのままコピーするのが確実（動作実績あり）。詳細は SETUP.md「Phase G」。
- **HuNav はロボットが必須。** `hunav_house.launch.py` 単体（ロボット spawn なし）だと
  `Robot model turtlebot3 not found` が出続け、アクターが T ポーズ・床埋まり・空中に
  なり動かない。人の確認は必ず full の `simulation.launch.py` で行う。
- **GUI(tkinter) が出ない**: ヘッドレス環境では `tk` import に失敗し GUI は起動しない
  （ノードは error ログを出して終了）。表示が必要なら `gui:=false` で外すか X 環境で実行。
- **`colcon build --symlink-install` は削除ファイルを install から消さない。**
  ノードやlaunchを消したら `rm -rf build/susumu_object_perception install/susumu_object_perception` してから再ビルド
  しないと、消したはずの旧ファイルが install 配下に残る。

## 動作確認の作法（このリポジトリ）

- Gazebo起動launchは **`run_in_background:true`（デタッチ）** で起動し、出力ファイルを
  `Read`/`grep` でポーリングする（フォアグラウンド+timeout は起動完了まで待てず不向き）。
- 確認すべき要点: planner作成ログ、`Managed nodes are active`、Teleop GUI の起動、
  手動操縦/AUTO巡回で `/cmd_vel` が出ること、odom座標の変化。
- 終了処理: `ps aux | grep -E "gzserver|component_container|teleop_gui" | awk '{print $2}'
  | xargs -r kill -9`。`pkill`は環境によりツールのexit code 1を招くので xargs+kill が安全。
