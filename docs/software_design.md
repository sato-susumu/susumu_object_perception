# ソフトウェアデザイン — susumu_object_perception

ROS 2 Humble + Gazebo Classic 11 上で、HuNavSim 制御の歩行者が動く家を 3D-LiDAR
TurtleBot3 が走り回る**シミュレーター**の設計ドキュメント。Nav2 自律移動と、手動操縦
／部屋自動巡回を行う Teleop GUI を備える。

利用方法・起動コマンドは [`../README.md`](../README.md)、構築の詳細手順・ハマりどころは
[`../SETUP.md`](../SETUP.md) を参照。

## 目次

- [1. 全体構造](#1-全体構造)
- [2. launch 構成と起動順序](#2-launch-構成と起動順序)
- [3. ノード／プロセス詳細](#3-ノードプロセス詳細)
- [4. トピック・フレーム・座標系](#4-トピックフレーム座標系)
- [5. 主なパラメータ](#5-主なパラメータ)
- [6. Teleop GUI の状態遷移](#6-teleop-gui-の状態遷移)
- [7. シーケンス図](#7-シーケンス図)
- [8. Autoware perception パイプラインとの違い](#8-autoware-perception-パイプラインとの違い)
- [9. ディレクトリ構成](#9-ディレクトリ構成)
- [10. 設計上の判断・既知の制約](#10-設計上の判断既知の制約)

---

## 1. 全体構造

既定は cafe world で HuNav 歩行者 5 人が歩く**シミュレーター**。Nav2 は生のセンサトピック
（`/scan`・`/lidar/points`）を障害物入力に使う（人も普通の障害物として costmap に乗る）。
これに加えて **Autoware LiDAR perception パイプライン**（既定 ON）を載せ、3D LiDAR 点群から
物体（特に歩く人）を検出・追跡し RViz に可視化する。Autoware 純正モジュールで検出し、apt に
無い段は Python 自作で補完する。詳細は [8 章](#8-autoware-perception-パイプラインとの違い)・
[`autoware_perception.md`](autoware_perception.md)。

perception と Nav2 の連携は **prediction の予測のみ**。`prediction_node` が「人がこれから
行く先」を OccupancyGrid `/perception/predicted_costmap` として毎フレーム出し、自作 C++
costmap 層 `PredictedCostmapLayer` が max 合成で costmap に焼く。検出・追跡そのものは
costmap に焼かず可視化のみで、Nav2 は現在位置の障害物回避を生センサ（`/scan`）で行う。

```mermaid
flowchart LR
    HUNAV["HuNavSim<br/>(hunav_agent_manager)"]:::hd -- "actor位置 (SFM)" --> GZ
    GZ["Gazebo Classic<br/>(generatedWorld.world)"]:::ext -- "/scan (2D)" --> NAV2
    GZ -- "/lidar/points (3D)" --> NAV2
    GZ -- "/odom + TF" --> NAV2
    NAV2["Nav2<br/>(AMCL + costmap + planner/controller)"]:::aw -- "/cmd_vel" --> GZ
    RVIZ["RViz2"]:::ext -- "/navigate_to_pose<br/>(2D Goal Pose)" --> NAV2
    GUI["Teleop GUI<br/>(teleop_gui_node)"]:::own -- "/cmd_vel (手動)" --> GZ
    GUI -- "/navigate_to_pose (自動巡回)" --> NAV2
    GUI -- "/initialpose (原点ワープ)" --> NAV2
    NAV2 -. "costmap / path" .-> RVIZ
    GZ -- "/lidar/points (3D)" --> AWP
    AWP["Autoware perception<br/>(検出→追跡→予測→可視化)"]:::own -. "/perception/markers" .-> RVIZ
    AWP -- "/perception/predicted_costmap<br/>(予測コストマップ)" --> NAV2

    classDef aw fill:#2e7d32,stroke:#1b5e20,color:#fff;
    classDef own fill:#e65100,stroke:#bf360c,color:#fff;
    classDef hd fill:#1565c0,stroke:#0d47a1,color:#fff;
    classDef ext fill:#455a64,stroke:#263238,color:#fff;
```

> 実線=データ依存（毎周期流れる）、破線=可視化目的の出力（RViz への markers / costmap・path
> 表示）。色は [8 章](#8-autoware-perception-パイプラインとの違い)と同じ凡例（緑=Autoware 純正、
> 橙=自作、青=外部入力/HuNav、紺=境界プロセス）。perception の内部段（crop_box→ground_filter
> →cluster→自作 ROI/追跡/予測/可視化）と Autoware 本来のパイプラインとの差分は [8 章](#8-autoware-perception-パイプラインとの違い)。

設計方針:

| 方針 | 内容 |
|---|---|
| メッセージ型は自作せず既存を使う | **独自 `.msg` を定義しない**。標準型（`Twist` / `LaserScan` / `PointCloud2` / `PoseWithCovarianceStamped` / `nav2_msgs/NavigateToPose`）や、用途に合う既存型（`autoware_perception_msgs` / `visualization_msgs` 等）が既にあればそれを使う。無い場合も「まず既存型で表現できないか」を優先する |
| Gazebo は Classic 11 | Ignition/Gazebo Sim ではない。HuNavSim は `v1.0-humble` ブランチ必須（`v2.0` は Gazebo Sim 用） |
| Nav2 の障害物層 | 現在位置は `/scan`（obstacle_layer, 2D）。人の現在位置 + 進路先は perception の予測 OccupancyGrid を自作 `predicted_layer`（`PredictedCostmapLayer`）が max 合成で焼く。AMCL も生 `/scan`。※ 旧 3D 障害物層（STVL）は人の通過跡が残るため廃止（[`nav2_tuning.md`](nav2_tuning.md)） |
| 段階起動 | プロセス間に順序依存があるため `TimerAction` で遅延起動する（[2章](#2-launch-構成と起動順序)） |
| GUI と Nav2 は /cmd_vel を共有 | 手動入力時は自動巡回を OFF にして Nav2 ゴールをキャンセルし、Twist を直接 publish（手動優先） |

---

## 2. launch 構成と起動順序

`simulation.launch.py`（`launch/` 直下）が全体のエントリポイント。`launch/include/` の
部品 launch を include し、`TimerAction` で段階起動する。起動の流れは
[7 章の起動シーケンス図](#7-シーケンス図)を参照。

### launch ファイル一覧

| ファイル | 役割 | 単体起動 |
|---|---|---|
| `launch/simulation.launch.py` | 全部入り（下記すべて + RViz2 + GUI）。エントリポイント | ○ |
| `launch/include/hunav_house.launch.py` | Gazebo（既定 cafe world）+ HuNavSim 歩行者5人 | ○ |
| `launch/include/spawn_robot.launch.py` | 3D-LiDAR TurtleBot3 を spawn + robot_state_publisher | ○（要 Gazebo 起動済み） |
| `launch/include/test_robot_empty.launch.py` | 空 world + ロボット単体（3D LiDAR / TF 確認用）。`simulation` からは include されない検証専用 | ○ |

### 起動タイムライン

| 時刻 | 起動対象 | 遅延の理由 |
|---|---|---|
| +0s | Gazebo（既定 cafe world）+ HuNavSim 5人 | — |
| +8s | ロボット spawn + robot_state_publisher | Gazebo が先に立ち上がっている必要がある |
| +12s | Nav2（AMCL + costmap + planner/controller） | ロボット／TF が揃っている必要がある |
| +12s | RViz2 | — |
| +15s | Teleop GUI | navigate_to_pose アクションサーバ（Nav2）が存在する必要がある |

> 遅延値はプロセス間の順序依存（robot が居ないと Nav2 の TF が揃わない等）を満たすための
> 値。むやみに詰めると初期化レースで起動に失敗する。

### 主な launch 引数（`simulation.launch.py`）

| 引数 | 既定 | 意味 |
|---|---|---|
| `use_nav2` | True | Nav2 スタックを起動する |
| `use_perception` | True | Autoware perception パイプライン（LiDAR 検出・追跡・予測）を起動する（[8 章](#8-autoware-perception-パイプラインとの違い)） |
| `image_recognition` | True | 画像認識を起動する。Gazebo は 6 面カメラ→全天球合成(`omni_image_node`) + LiDAR 検出物体の YOLO 分類(`object_classifier`) + 全天球信号認識(`traffic_light_*`)。YOLO が重ければ False（LiDAR perception は残る） |
| `use_rviz` | True | RViz2 を起動する |
| `gui` | True | Teleop / 自動巡回 GUI を起動する |
| `map` | `maps/cafe.yaml` | マップ yaml のフルパス（既定は cafe world。house に戻すなら `maps/house.yaml`） |
| `params_file` | `config/nav2_params.yaml` | Nav2 パラメータ yaml のフルパス |
| `x_pose` / `y_pose` / `yaw` | 0.0 / 0.0 / 0.0 | ロボットの spawn 姿勢（マップの空きスペース） |

---

## 3. ノード／プロセス詳細

| プロセス | パッケージ | 役割 |
|---|---|---|
| `hunav_loader` | hunav_agent_manager | `agents_cafe.yaml`（既定）を読み込む |
| `hunav_gazebo_world_generator` | hunav_gazebo_wrapper | world（既定 cafe）+ エージェント → `generatedWorld.world` を生成 |
| `gzserver` / `gzclient` | gazebo_ros | 生成したワールドを実行（HuNav プラグイン入り） |
| `hunav_agent_manager` | hunav_agent_manager | エージェント behavior を駆動（Social Force Model） |
| `robot_state_publisher` | robot_state_publisher | URDF から TF を publish（`base_link → lidar_link` 等） |
| `spawn_entity.py` | gazebo_ros | SDF モデルを spawn（diff_drive / laser / 3D MID-360 (ODE MultiRayShape) / VLP-16 は gpu_ray / imu / camera プラグイン） |
| Nav2 スタック | nav2_bringup | AMCL + costmap + planner + controller + bt_navigator |
| `teleop_gui` | susumu_object_perception | Teleop / 部屋自動巡回 GUI（[6章](#6-teleop-gui-の状態遷移)） |
| Autoware perception | autoware_* + susumu_object_perception | crop_box / ground_filter / euclidean_cluster（純正）+ 自作 4 ノード（[8章](#8-autoware-perception-パイプラインとの違い)） |

### teleop_gui_node の内部構造

tkinter のウィンドウをメインスレッドで動かし、rclpy を別スレッドで spin する。

| 要素 | 内容 |
|---|---|
| `_tick`（10Hz タイマ） | 手動コマンドが有効な間、`/cmd_vel` に `Twist` を再 publish（diff_drive は連続送信が必要） |
| `_auto_watchdog`（1Hz タイマ） | 自動巡回 ON の間、常にゴールが飛んでいる状態を維持。`WAYPOINT_TIMEOUT_S` 超過で次へスキップ |
| `set_manual` / `stop_manual` | GUI ボタン／キーから呼ばれる手動操縦。手動入力で自動巡回を OFF にする |
| `set_auto` / `_send_next_waypoint` | `PATROL_WAYPOINTS` を Nav2（NavigateToPose）で順に巡回 |
| `warp_to_origin` | `gz model` でロボットを原点へ移動し、`/initialpose` で AMCL を再初期化 |

---

## 4. トピック・フレーム・座標系

### 主要トピック

| トピック | 型 | publish する側 | subscribe する側 | 説明 |
|---|---|---|---|---|
| `/lidar/points` | `sensor_msgs/PointCloud2` | Gazebo (MID-360 plugin) | perception 前処理 / pointcloud_to_laserscan / RViz | 3D LiDAR 点群（perception 入力。`/scan` 生成元） |
| `/scan` | `sensor_msgs/LaserScan` | pointcloud_to_laserscan | AMCL / Nav2 obstacle_layer | 3D 点群から生成（自己位置 + 2D 障害物入力） |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2 controller / teleop_gui | Gazebo diff_drive | 速度司令（Nav2 と GUI 手動操縦が共有） |
| `/navigate_to_pose` | `nav2_msgs/NavigateToPose` | RViz / teleop_gui | Nav2 bt_navigator | ゴール指定（RViz 2D Goal Pose / GUI 自動巡回） |
| `/initialpose` | `geometry_msgs/PoseWithCovarianceStamped` | teleop_gui | AMCL | 原点ワープ時の AMCL 再初期化 |
| `/odom` | `nav_msgs/Odometry` | Gazebo diff_drive | Nav2 | オドメトリ |
| `/perception/no_ground/pointcloud` | `sensor_msgs/PointCloud2` | Autoware ground_filter | cluster / shape_estimation / RViz | 地面除去点群（後段の検出・形状推定の入力） |
| `/perception/predicted_costmap` | `nav_msgs/OccupancyGrid` | prediction_node | `PredictedCostmapLayer`（Nav2 costmap） | 予測コストマップ（人の現在位置 + 進路先）。max 合成で焼く |
| `/perception/tracked_objects` | `autoware_perception_msgs/TrackedObjects` | object_tracker_node | prediction_node / perception_marker_node / detection_by_tracker_node | 追跡結果（ID・速度・向き付き） |
| `/perception/markers` | `visualization_msgs/MarkerArray` | perception_marker_node | RViz | perception 可視化（検出=青/移動=赤/静止=緑、`#ID 速度[km/h]`） |
| `/perception/tracked_objects_classified` | `autoware_perception_msgs/TrackedObjects` | object_classifier_node | 下流 | LiDAR 検出物体を全天球画像 YOLO で分類し classification を上書き（late fusion） |
| `/perception/object_classes/markers` | `visualization_msgs/MarkerArray` | object_classifier_node | RViz | 物体の 3D 位置にクラス名テキスト |
| `/perception/traffic_signals` | `autoware_perception_msgs/TrafficSignalArray` | traffic_light_detector_node | 下流 | 全天球画像から検出した信号の色（全天球モードは id=方位deg） |
| `/perception/traffic_light/poses` | `geometry_msgs/PoseArray` | traffic_light_localizer_node | RViz | 信号の 3D 位置（検出方向 × LiDAR 点群） |

### フレーム／トピックの約束（変更時は両側を揃える）

| 役割 | 値 | 定義場所 |
|---|---|---|
| 速度司令 | `cmd_vel` | SDF diff_drive ↔ nav2 controller / teleop_gui |
| オドメトリ | frame/topic `odom`（`publish_odom_tf:true`） | SDF diff_drive ↔ amcl odom_frame |
| ベース | `base_footprint`(amcl) / `base_link`(costmap) | SDF / URDF / nav2_params |
| 2D スキャン | `/scan`, frame `lidar_link` | pointcloud_to_laserscan（/lidar/points→/scan）↔ amcl ↔ nav2 obstacle_layer |
| 3D LiDAR | `/lidar/points`, frame `lidar_link` | SDF MID-360 plugin ↔ perception ↔ pointcloud_to_laserscan（→ /scan）。※ Nav2 への 3D STVL 層は廃止 |
| HuNav 追跡対象 | robot_name=`turtlebot3`（spawn entity 名と一致必須） | hunav_house / spawn_robot |

---

## 5. 主なパラメータ

### Teleop GUI（`susumu_object_perception/teleop_gui_node.py`、モジュール定数）

| 定数 | 既定 | 意味 |
|---|---|---|
| `LINEAR_SPEED` | 0.22 | 手動前後進の速度 [m/s]（waffle 最大 ~0.26） |
| `ANGULAR_SPEED` | 0.9 | 手動旋回速度 [rad/s] |
| `PUBLISH_HZ` | 10.0 | 手動 Twist の再送レート [Hz] |
| `PATROL_WAYPOINTS` | 11点 | 自動巡回するルーム中心の経路（部屋を順に巡る） |
| `WAYPOINT_TIMEOUT_S` | 25.0 | 1ウェイポイントで詰まったら次へ進むまでの時間 [s] |
| `ROBOT_ENTITY` | `turtlebot3` | 原点ワープ時に動かす Gazebo モデル名（spawn 名と一致必須） |

### 歩行者（`config/agents_house.yaml`、各エージェント）

公式 `hunav_gazebo_wrapper/scenarios/agents_house.yaml` のコピー（動作実績あり）。
5人は通常歩行速度で**巡回し続ける**設定（**`once: true` + `cyclic_goals: true`**、
各3ゴールの三角ルート）。速度や経路を変えたいときはこのファイルを編集する。

> ⚠️ `once: false` にすると HuNav の behavior 駆動が回らず、ほとんどのエージェントが
> 数十秒で停止する。歩かせ続けたいときも **`once: true`** のままにすること。

| パラメータ | 値 | 意味 |
|---|---|---|
| `max_vel` | 1.5 | エージェントの最大速度 [m/s] |
| `behavior.vel` | 0.6〜0.8 | 目標巡航速度 [m/s] |
| `goal_radius` | 0.3 | ゴール到達判定半径 [m] |
| `obstacle_force_factor` | 10.0 | 障害物回避力の係数 |
| `social_force_factor` | 5.0 | 対人社会力の係数 |
| `other_force_factor` | 20.0 | その他の力の係数 |
| `once` / `cyclic_goals` | **true** / true | ゴール列を巡回し続ける（`false` にすると停止するので注意） |

### Nav2（`config/nav2_params.yaml`、抜粋）

| 項目 | 値 | 意味 |
|---|---|---|
| obstacle_layer.scan.topic | `/scan` | 2D 障害物入力（3D 点群から生成した /scan） |
| 予測コストマップ層 | `predicted_layer`（自作 `susumu_object_perception::PredictedCostmapLayer`） | perception の予測 OccupancyGrid `/perception/predicted_costmap`（人の現在位置 + 進路先）を max 合成で焼く。毎フレーム作り直すので軌跡が残らない。※ 旧 3D STVL 層は廃止（[`nav2_tuning.md`](nav2_tuning.md)） |
| planner | `nav2_navfn_planner/NavfnPlanner` | Nav2 1.1.20 と整合する `/` 形式のプラグイン名 |
| amcl.scan_topic | `scan` | AMCL は /scan（3D 点群から生成）で自己位置推定 |

> Nav2 パラメータの調整指針・症状別の対処・変更履歴は
> [`nav2_tuning.md`](nav2_tuning.md) にまとめている。**Nav2 を調整したら必ず更新すること。**

---

## 6. Teleop GUI の状態遷移

`teleop_gui_node` は「停止」「手動操縦」「自動巡回」の3状態を持つ。手動入力は常に
自動巡回より優先される。

```mermaid
stateDiagram-v2
    [*] --> Idle : 起動
    Idle --> Manual : 矢印キー/テンキー押下 (set_manual)
    Manual --> Idle : キー/ボタン解放 (stop_manual)

    Idle --> Auto : AUTO トグル ON (set_auto True)
    Auto --> Idle : AUTO トグル OFF (set_auto False)

    Auto --> Manual : 矢印キー/テンキー押下<br/>(自動巡回をキャンセルし手動優先)
    Manual --> Idle : 解放後は停止<br/>(Auto には自動復帰しない)

    Auto --> Auto : ウェイポイント到達/中断<br/>→ 次のウェイポイントへ
    Auto --> Auto : WAYPOINT_TIMEOUT_S 超過<br/>→ スキップして次へ

    Idle --> Warp : Warp ボタン押下
    Manual --> Warp : Warp ボタン押下
    Auto --> Warp : Warp ボタン押下<br/>(Auto を OFF にする)
    Warp --> Idle : gz model 移動 + /initialpose 再初期化 完了
```

| 状態 | /cmd_vel | Nav2 ゴール | 説明 |
|---|---|---|---|
| Idle | 停止（空 Twist） | なし | 待機 |
| Manual | 押下中の Twist を 10Hz 再送 | キャンセル済み | 手動操縦。自動巡回より優先 |
| Auto | Nav2 controller が出力 | NavigateToPose（巡回） | 部屋を順に自動巡回 |
| Warp | 停止 | キャンセル | ロボットを原点へワープし AMCL 再初期化 |

---

## 7. シーケンス図

### 起動シーケンス（`simulation.launch.py`）

```mermaid
sequenceDiagram
    participant LD as simulation.launch
    participant HN as HuNav + Gazebo
    participant RB as spawn_robot
    participant NV as Nav2
    participant GUI as teleop_gui

    LD->>HN: +0s 起動 (world生成→gzserver/gzclient→agent_manager)
    LD->>RB: +8s ロボット spawn + robot_state_publisher
    Note over HN,RB: +8s = Gazebo が先に立ち上がっていないと<br/>spawn_entity が world に注入できない
    Note over RB: base_link→lidar_link 等の TF が揃う
    LD->>NV: +12s AMCL + costmap + planner/controller (+RViz)
    Note over RB,NV: +12s = ロボット/TF が揃う前に AMCL を<br/>起動すると odom→base_link が無く落ちる
    Note over NV: navigate_to_pose アクションサーバ起動
    LD->>GUI: +15s GUI ウィンドウ表示
    Note over NV,GUI: +15s = navigate_to_pose アクションサーバが<br/>存在しないと GUI の自動巡回が接続できない
```

### 自動巡回（AUTO ON 時の1サイクル）

```mermaid
sequenceDiagram
    participant U as ユーザー
    participant G as teleop_gui
    participant N as Nav2
    participant R as ロボット(Gazebo)

    U->>G: AUTO トグル ON
    G->>N: NavigateToPose (waypoint[i])
    N->>R: /cmd_vel で移動
    alt 到達 (SUCCEEDED) or 中断 (ABORTED)
        N-->>G: result
        G->>G: i = (i+1) % len(waypoints)
        G->>N: NavigateToPose (次の waypoint)
    else WAYPOINT_TIMEOUT_S (25s) 超過
        G->>N: cancel_goal
        G->>G: i = (i+1) % len(waypoints) でスキップ
        G->>N: NavigateToPose (次の waypoint)
    end
    Note over G,N: _auto_watchdog(1Hz) が<br/>ゴール途切れを検知し再送
```

### 原点ワープ

```mermaid
sequenceDiagram
    participant U as ユーザー
    participant G as teleop_gui
    participant GZ as Gazebo
    participant A as AMCL

    U->>G: Warp ボタン押下
    G->>G: 自動巡回 OFF + /cmd_vel 停止 (stop_manual)
    G->>GZ: gz model -m turtlebot3 -x 0.0 -y 0.0 -z 0.05 -Y 0.0
    Note over GZ: Gazebo Classic CLI でエンティティを原点へ移動
    G->>A: /initialpose (原点, cov x/y=0.25 yaw=0.068, 数回 publish)
    Note over A: 自己位置推定を原点に再シード<br/>(古い場所に残ると Nav2 が誤計画するため)
```

---

## 8. Autoware perception パイプラインとの違い

本パッケージの perception は **Autoware 本来の物体認識パイプラインを部分的に再現**した
もの。検出の前処理〜クラスタ化までは **Autoware 純正モジュールをそのまま使う**が、apt で
入手できない段（追跡など）や本シミュレーターに不要な段は、**Python 自作で代替**するか
**省略**している。HD 地図も使わず、2D 占有格子地図 `/map` 照合で代替する。
詳細は [`autoware_perception.md`](autoware_perception.md)。

下図は **Autoware 本来の構成（上段）** と **本実装（下段）** を段ごとに対応させたもの。

```mermaid
flowchart TB
    subgraph AW["Autoware 本来の perception"]
        direction LR
        A_pre["crop_box →<br/>ground_filter"]:::aw
        A_det["euclidean_cluster +<br/>shape_estimation"]:::aw
        A_roi["HD 地図 ROI<br/>(drivable area)"]:::hd
        A_trk["multi_object_tracker<br/>(C++ 純正)"]:::aw
        A_pred["map_based_prediction<br/>(将来軌跡)"]:::aw
        A_viz["autoware_perception_<br/>rviz_plugin"]:::aw
        A_out(["Planning へ"]):::ext
        A_pre --> A_det --> A_roi --> A_trk --> A_pred --> A_out
        A_trk -.-> A_viz
    end

    subgraph SS["本実装 (susumu_object_perception)"]
        direction LR
        S_pre["crop_box →<br/>ground_filter"]:::aw
        S_det["euclidean_cluster +<br/>shape_estimation_node.py<br/>(自作 / L字フィット踏襲)"]:::own
        S_dbt["detection_by_tracker_node.py<br/>(過分割統合 / Cluster Merger踏襲)"]:::own
        S_roi["2D 占有格子地図照合<br/>map_roi_filter_node.py"]:::own
        S_trk["object_tracker_node.py<br/>(自作 / 純正踏襲)"]:::own
        S_pred["prediction_node.py<br/>(2D地図 マルチモーダル予測)"]:::own
        S_viz["perception_marker_<br/>node.py (自作)"]:::own
        S_cm["/perception/predicted_costmap<br/>(OccupancyGrid)"]:::own
        S_layer["PredictedCostmapLayer<br/>(自作 C++ 層 / max 合成)"]:::own
        S_nav(["Nav2 costmap へ連携"]):::aw
        S_pre --> S_det --> S_dbt --> S_roi --> S_trk --> S_pred
        S_pred --> S_viz
        S_pred --> S_cm --> S_layer --> S_nav
        S_trk -.->|前フレーム参照| S_dbt
        S_trk -.-> S_viz
    end

    A_pre -. "そのまま流用" .-> S_pre
    A_det -. "L字フィットのみ踏襲(型は自作)" .-> S_det
    A_roi -. "HD地図→2D地図照合で代替" .-> S_roi
    A_trk -. "C++→Python 自作" .-> S_trk
    A_pred -. "HD地図→2D占有格子で代替" .-> S_pred
    A_viz -. "純正プラグイン→自作Marker" .-> S_viz
    A_out -. "Planning→Nav2 costmap (予測のみ連携)" .-> S_nav

    classDef aw fill:#2e7d32,stroke:#1b5e20,color:#fff;
    classDef own fill:#e65100,stroke:#bf360c,color:#fff;
    classDef hd fill:#1565c0,stroke:#0d47a1,color:#fff;
    classDef skip fill:#757575,stroke:#424242,color:#fff;
    classDef ext fill:#455a64,stroke:#263238,color:#fff;
```

| 色 | 意味 |
|---|---|
| 🟩 緑 | **Autoware 純正をそのまま使用**（crop_box / ground_filter / euclidean_cluster）。本実装の Nav2 連携先（costmap）も同色 |
| 🟧 橙 | **自作**（Python ノード群 + C++ costmap 層 `PredictedCostmapLayer`） |
| 🟦 青 | **HD 地図依存**（本実装では 2D 占有格子地図照合で代替） |
| 🟦 紺 | **境界**（Autoware では Planning、本実装では Nav2 costmap） |

> **Nav2 連携（最新）**: 検出・追跡そのものは costmap に焼かない（可視化のみ）。連携するのは
> **prediction の予測だけ** — `prediction_node` が「人がこれから行く先」を OccupancyGrid
> `/perception/predicted_costmap` として毎フレーム作り直して出し、自作 C++ 層
> `susumu_object_perception::PredictedCostmapLayer` が **max 合成**で local/global costmap に
> 乗せる（他層を壊さず・蓄積させず）。標準層（ObstacleLayer/STVL=蓄積、StaticLayer=他層上書き）
> では両立できず自作した。Nav2 は現在位置の障害物回避を従来どおり生センサ `/scan` で行う。

段ごとの対応:

| 段階 | Autoware 本来 | 本実装 | 差分の理由 |
|---|---|---|---|
| 前処理 | crop_box → ground_filter | 同じ（純正） | 屋内向けパラメータのみ調整。`ground_filter` は ring/channel 必須のため `pointcloud_to_autoware_node.py` で PointXYZIRC へ変換して投入 |
| 検出 | euclidean_cluster + shape_estimation | euclidean_cluster（純正）+ `shape_estimation_node.py`（自作） | apt に shape_estimation 無し、universe 版は型（tier4_perception_msgs）が世代不整合。L字フィット（rotating calipers + closeness criterion）の**アルゴリズムだけ Autoware 公式（bounding_box.cpp）を踏襲**し、型は標準型で自作。no_ground 点群から各検出近傍を集めて OBB 推定 |
| 過分割統合 | detection_by_tracker（Cluster Merger + IoU 分割） | `detection_by_tracker_node.py`（自作、過分割統合のみ） | 前フレームの tracker 位置・サイズを参照し、1 物体が複数クラスタに割れた検出を統合（Autoware Cluster Merger 踏襲）。統合後の shape は**包含 BBox ではなく点群を L字フィットで再推定**（巨大化回避＝Autoware と同じ）。under-segmentation の IoU 反復分割は未実装 |
| ROI 絞り | HD 地図（drivable area） | 2D 占有格子地図 `/map` 照合（自作） | HD 地図を持たないため。壁・地図外・未知に当たる検出を `map_roi_filter_node.py` で除外 |
| 追跡 | multi_object_tracker（C++ 純正） | `object_tracker_node.py`（自作 Python） | apt に無い。Autoware のソース（ハンガリアン法＋マハラノビス χ²＋existence_probability Bayes 更新）を踏襲して再実装 |
| 分類 | HD マップの walkable-area 上の物体を歩行者と推定 | `object_tracker_node.py`（2D 占有格子で代替）+ `object_classifier_node.py`（**画像分類**） | ①幾何ベース: free space で移動=`PEDESTRIAN`/静止=`UNKNOWN`（tracker 出力段）。②**画像ベース（追加）**: tracked_objects の方向の全天球クロップを YOLOv8(COCO) で分類し classification を上書き（LiDAR×カメラ late fusion）。トラック ID キャッシュ + レート上限で間引き。yolo 初期化失敗は `[FATAL]`（classic 等へ自動フォールバックしない） |
| 予測 | map_based_prediction（HD 地図のレーン/crosswalk に沿う。crosswalk マルチパス） | `prediction_node.py`（自作、2D 占有格子版） | HD 地図が無いので **2D 占有格子で代替**。等速 CV 予測 + 予測点が occupied セルなら打ち切り（壁めり込み回避）。**マルチモーダル化**: 進行方向を中心に複数角度で扇状に複数パスを出し（crosswalk マルチパスの 2D 版）、直進ほど高 confidence・伸びた長さで重み付け。出力 `/perception/predicted_objects`（PredictedObjects） |
| 信号認識 | map_based_detector（HD 地図の信号位置を画像投影）→ fine_detector → classifier | `traffic_light_detector_node.py`（自作、全天球） + `traffic_light_localizer_node.py`（3D 位置） | HD 地図が無いので地図起点 ROI は使えず、**全天球画像を全周 N 分割の透視ビュー**に展開して全方位を検出（classic=HSV+円形度 / yolo=YOLOv8）。色は色相 + 灯位置で判定。3D 位置は検出方向 × LiDAR 点群で推定。出力型は Autoware `TrafficSignalArray` に合わせる |
| 可視化 | autoware_perception_rviz_plugin | `perception_marker_node.py`（自作 MarkerArray） | 表示方法・色を自由に作り込むため。spencer / leg_tracker など Nav2 系プラグインの作法に合わせた |
| 下流 | Planning | Nav2 costmap（予測のみ連携） | 検出・追跡は可視化のみで costmap に焼かない。連携するのは **prediction の予測だけ**: `/perception/predicted_costmap`（OccupancyGrid）を自作 C++ 層 `PredictedCostmapLayer` が max 合成で local/global costmap に乗せる（標準層は蓄積/上書きで両立不可のため自作）。Nav2 の現在位置回避は従来どおり生 `/scan`。※ 旧 3D STVL 層は廃止（[`nav2_tuning.md`](nav2_tuning.md)） |

---

## 9. ディレクトリ構成

```
susumu_object_perception/
├── launch/
│   ├── simulation.launch.py        # 全部入り エントリポイント（gui:=false でGUI無効）
│   └── include/                    # simulation が取り込む部品 launch
│       ├── hunav_house.launch.py      # world + 5人HuNav のみ（cafe/house 切替）
│       ├── spawn_robot.launch.py      # 3D LiDAR TB3 spawn + robot_state_publisher
│       ├── autoware_perception.launch.py # Autoware perception パイプライン（8章）
│       └── test_robot_empty.launch.py # 空world + ロボット単体（3D LiDAR確認用）
├── susumu_object_perception/
│   ├── teleop_gui_node.py            # Teleop / 部屋自動巡回 GUI
│   ├── pointcloud_to_autoware_node.py # PointXYZI → PointXYZIRC（ground_filter 用）
│   ├── shape_estimation_node.py      # OBB 推定（Autoware L字フィット踏襲、自作）
│   ├── detection_by_tracker_node.py  # 過分割統合（Autoware Cluster Merger 踏襲、自作）
│   ├── map_roi_filter_node.py        # 2D 地図照合 ROI（壁/地図外/未知の検出を除外）
│   ├── object_tracker_node.py        # DetectedObjects → TrackedObjects 追跡（自作）
│   ├── prediction_node.py            # 2D 将来軌跡予測（CV+壁回避、自作）
│   └── perception_marker_node.py     # Detected/Tracked/Predicted → MarkerArray 可視化（自作）
├── config/
│   ├── agents_cafe.yaml           # HuNav 5人（cafe、既定）
│   ├── agents_house.yaml          # HuNav 5人（house）
│   ├── autoware_*.param.yaml      # crop_box / ground_filter / euclidean_cluster 等
│   └── nav2_params.yaml           # Nav2（obstacle_layer=/scan、予測層=自作 predicted_layer。STVL 廃止）
├── models/turtlebot3_waffle_3d/   # waffle + 3D LiDAR の Gazebo SDF
├── urdf/turtlebot3_waffle_3d.urdf.xacro  # TF用URDF
├── maps/cafe.{pgm,yaml}           # cafe のマップ（既定）
├── maps/house.{pgm,yaml}          # house のマップ
├── rviz/simulation.rviz           # RViz設定（3D点群 + perception 可視化付き）
├── docs/software_design.md        # 本ドキュメント
├── docs/tasks/                    # タスク別の制約・合格基準・実行手順
├── docs/worlds.md                 # Gazebo/Webots world の使い分け
├── docs/robot_lidar.md            # ロボット / LiDAR 構成と topic/frame 契約
├── docs/autoware_perception.md    # perception パイプライン詳細
├── docs/nav2_tuning.md            # Nav2 調整ガイド
├── LICENSE                        # MIT License
├── README.md / AGENTS.md / CLAUDE.md / SETUP.md
└── CMakeLists.txt / package.xml
```

---

## 10. 設計上の判断・既知の制約

| 項目 | 内容 |
|---|---|
| source は `local_setup.bash` | `install/setup.bash` は古いスナップショットを指す prefix-chain で、新規パッケージが見えず `package not found` になる |
| Python ノードはファイル名で起動 | console_scripts ではない。`ros2 run susumu_object_perception teleop_gui_node.py`。ノード追加時は CMakeLists の `install(PROGRAMS ...)` に追加し実行ビットを立てる |
| HuNavSim は `v1.0-humble` 必須 | `v2.0` は Gazebo Sim 依存でビルド／起動に失敗する |
| Nav2 params のベース | `turtlebot3_navigation2` の waffle.yaml は `::` 形式で Nav2 1.1.20 と不整合。同梱バージョンと一致する `nav2_bringup/params/nav2_params.yaml` をベースにする |
| 歩行者が動かない | `agents_house.yaml` の `once: false` だと HuNav の behavior 駆動が回らず数十秒で停止する。**`once: true` + `cyclic_goals: true`**（公式 house シナリオと同じ）が正解。HuNav はロボット必須で、人だけ起動すると T ポーズ・床埋まりになる |
| GUI(tkinter) はヘッドレス不可 | X 環境がないと import に失敗し GUI は起動しない。不要時は `gui:=false` |
| `--symlink-install` の削除漏れ | colcon は削除ファイルを install から消さない。ノード／launch を消したら `rm -rf build/susumu_object_perception install/susumu_object_perception` してから再ビルドする |
