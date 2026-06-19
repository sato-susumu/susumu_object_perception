# susumu_object_perception

ROS 2 Humble のシミュレーター統合パッケージ。**3D LiDAR + 全天球カメラを載せた移動ロボット**が、
シミュレータ上の環境を**自律的に地図化し・巡回し・周囲の物体を検出/識別する**ところまでを一貫して扱う。

## 目指す構想

実機の自律移動ロボットに必要な「**地図を作る → 経路を巡る → 周囲を認識する**」のループを、
**シミュレータ上で Autoware 互換の perception を中心に統合・検証する**ことを目指す。

1. **環境を知る（マッピング）** — 事前地図のない環境を frontier 探索で自律的に動き回り SLAM 地図を作る
2. **環境を巡る（ナビゲーション）** — 作った地図から巡回ウェイポイントを生成し Nav2 で巡回する
3. **環境を理解する（認識）** — 巡回しながら LiDAR で物体を検出・追跡し、カメラ画像（YOLO）で種類を識別、
   信号も認識する。人の進路先は予測して Nav2 の障害物回避に先回りで反映する

検出は Autoware 純正モジュールを使い、apt に無い段は Autoware アルゴリズムを踏襲して自作補完する。
複数のシミュレータ（Gazebo Classic / Webots）と複数の world（カフェ・屋内外・街・室内）で同じ
perception スタックを回せるようにしている。

> 例: **HuNavSim が制御する5人の歩行者**が動く**カフェ（cafe world）**を、3D LiDAR 搭載 TurtleBot3 が
> Nav2 で自律走行する構成が既定。手動操縦／自動巡回ができる **Teleop GUI** と、**Autoware 流の LiDAR
> perception パイプライン**（既定 ON、RViz 可視化）を備え、prediction の予測のみ Nav2 costmap に連携する。

- **ノード接続図 / トピック I/O 一覧**（どのノードがどのトピックで繋がっているか・Mermaid 図）: [`docs/node_topology.md`](docs/node_topology.md)
- 設計（全体構造・状態遷移・シーケンス図・パラメータ・ディレクトリ構成）: [`docs/software_design.md`](docs/software_design.md)
- Nav2 の調整（パラメータ・症状別の指針・変更履歴）: [`docs/nav2_tuning.md`](docs/nav2_tuning.md)
- perception パイプライン詳細: [`docs/autoware_perception.md`](docs/autoware_perception.md)
- 構築の詳細手順・ハマりどころ: [`SETUP.md`](SETUP.md)
- Webots 版シミュレーション: [`docs/webots_simulation.md`](docs/webots_simulation.md)

---

## できること

| 機能 | 内容 | 備考 |
|---|---|---|
| カフェ + 5人の歩行者 | HuNavSim（Social Force Model）が5人をカフェ内に配置し歩き回らせる | Gazebo。`vel: 0.6`〜`0.8`（通常歩行速度） |
| 3D LiDAR TurtleBot3 | waffle/burger 上部に MID-360 近似の3D LiDAR + 全天球カメラを搭載 | `/lidar/points`（PointCloud2）を出力 |
| Nav2 自律移動 | ゴール指定で人を含む障害物を避けて自律走行 | 現在位置回避は 2D `/scan`、進路先は予測コストマップ |
| Teleop / 自動巡回 GUI | 矢印（＋テンキー）で手動操縦、トグルで自動巡回、原点ワープ | tkinter（Gazebo） |
| Autoware 流 perception | 3D LiDAR 点群から検出〜追跡〜将来軌跡予測まで | 既定 ON、RViz 可視化が主 |
| 予測コストマップ連携 | prediction の予測だけ Nav2 costmap に焼く | 自作 C++ 層が max 合成、毎フレーム作り直し |
| **画像認識（物体分類・信号）** | LiDAR 検出物体を全天球クロップ→**YOLOv8(COCO)** で分類、全天球を全周分割して**信号検出**（色判定） | late fusion。`car`/`pedestrian` 等を識別、信号は 3D 位置も推定 |
| **frontier 自律マッピング** | 事前地図のない Webots 環境を frontier 探索で動き回って SLAM 地図を作成・保存 | `webots_city_mapping.launch.py`。情報利得で広い未踏領域を優先 |
| **ウェイポイント巡回** | 保存地図から巡回ウェイポイントを自動生成・可視化し、Nav2 で巡回 | `generate_waypoints.py`（到達可能領域に限定）→ `webots_waypoint_nav.launch.py`（各点タイムアウトで詰まってもスキップして一巡） |
| **色付き 3D 点群地図** | 全天球カメラで色付けした点群を SLAM フレームで蓄積し PLY 保存 | `omni_perception:=True` + `/slam/save_colorized_map` |
| **転倒検知** | IMU の傾きでロボットの転倒を常時監視し警告 | `fall_detector_node.py`。launch に統合済み、`/fall_detector/status` |

> 「人を検知して右隣を歩く」追従機能は持たない（旧 `susumu_lidar_perception` へ分離）。
> 各 launch の詳細・引数は [`docs/launch.md`](docs/launch.md) を参照。

---

## perception パイプライン

検出までは **Autoware 純正モジュール**、apt に無い段や HD 地図依存の段は **2D 占有格子地図と
Autoware アルゴリズムの踏襲で自作補完**している。

```mermaid
flowchart LR
  PC["/lidar/points"]:::hd
  MAP["/map<br/>(2D占有格子)"]:::hd

  crop["crop_box"]:::aw
  ground["ground_filter"]:::aw
  cluster["euclidean_cluster"]:::aw

  shape["shape_estimation<br/>L字フィットOBB"]:::own
  merge["detection_by_tracker<br/>過分割統合"]:::own
  roi["map_roi"]:::own
  tracker["tracker<br/>追跡+2D分類"]:::own
  pred["prediction<br/>将来軌跡予測"]:::own

  marker["marker<br/>RViz可視化"]:::own
  costmap["PredictedCostmapLayer<br/>Nav2 costmap連携"]:::own

  PC --> crop --> ground --> cluster --> shape --> merge --> roi
  MAP --> roi
  roi --> tracker --> pred
  pred --> marker
  pred --> costmap

  classDef aw fill:#2e7d32,stroke:#1b5e20,color:#fff;
  classDef own fill:#e65100,stroke:#bf360c,color:#fff;
  classDef hd fill:#1565c0,stroke:#0d47a1,color:#fff;
```

### 予測コストマップ連携

prediction が**人の現在位置 + 進路先**を予測 OccupancyGrid `/perception/predicted_costmap` として出し、
自作 C++ costmap 層 `susumu_object_perception::PredictedCostmapLayer` が **max 合成**で Nav2 costmap に焼く
（人の「これから行く先」を先回りで障害物化）。毎フレーム作り直すので移動軌跡が残らない。

```mermaid
flowchart LR
  scan["/scan"]:::hd
  predcm["/perception/predicted_costmap"]:::own
  obstacle["obstacle_layer"]:::aw
  predlayer["predicted_layer<br/>PredictedCostmapLayer"]:::own
  stvl["STVL層<br/>廃止"]:::skip
  cm["Nav2 costmap"]:::ext

  scan --> obstacle --> cm
  predcm --> predlayer --> cm
  stvl -.- cm

  classDef aw fill:#2e7d32,stroke:#1b5e20,color:#fff;
  classDef own fill:#e65100,stroke:#bf360c,color:#fff;
  classDef hd fill:#1565c0,stroke:#0d47a1,color:#fff;
  classDef skip fill:#757575,stroke:#424242,color:#fff;
  classDef ext fill:#455a64,stroke:#263238,color:#fff;
```

詳細は [`docs/autoware_perception.md`](docs/autoware_perception.md) / [`docs/nav2_tuning.md`](docs/nav2_tuning.md)。

---

## world について

既定は **cafe world**。家（house world）の素材も同梱しているが、house は狭い通路・家具密集により
歩行者が固着しやすい（[`SETUP.md`](SETUP.md) Phase H）。人がよく動き回るのは cafe。house に切り替えるには
起動引数で `map`・`base_world`・`configuration_file` を house 用に渡す。

---

## 必要環境・依存

| 種別 | 内容 |
|---|---|
| ベース | ROS 2 Humble / Gazebo Classic 11 / Nav2 / TurtleBot3(waffle) |
| 外部クローン | HuNavSim `hunav_sim` / `hunav_gazebo_wrapper`（`v1.0-humble`）、`people_msgs`（ソース） |
| ヘッダlib | `lightsfm`（`/usr/local/include` へ `make install`） |
| Python | tkinter（GUI） |

セットアップ手順は [`SETUP.md`](SETUP.md) の「Phase 0」を参照。

---

## ビルド

```bash
cd ~/ros2_ws
colcon build --symlink-install        # または --packages-select susumu_object_perception hunav_* people_msgs

# ★ source は setup.bash ではなく local_setup.bash を使うこと（理由はSETUP.md参照）
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
export TURTLEBOT3_MODEL=waffle
```

---

## 実行

全部入り（カフェ + 5人 + 3D LiDAR TB3 + Nav2 + RViz2 + Teleop GUI）:

```bash
ros2 launch susumu_object_perception simulation.launch.py
```

- RViz2 の **"2D Goal Pose"** で目的地を指定 → 人を避けて自律移動。
- **Teleop GUI** ウィンドウ:
  - 矢印ボタンを「押している間」だけ走行（テンキー 8/2/4/6、矢印キーも同じ）。
  - **自動巡回** トグルを ON にすると、Nav2 でカフェ内を順番に自動巡回。
  - **原点へワープ** で、隅にハマって動けなくなったロボットを原点へ戻す。

GUI を出したくないときは `gui:=false`:

```bash
ros2 launch susumu_object_perception simulation.launch.py gui:=false
```

Webots 版（屋外街など）も用意している（詳細は [`docs/webots_simulation.md`](docs/webots_simulation.md)）:

```bash
ros2 launch susumu_object_perception webots_simulation.launch.py world:=outdoor
```

VLP-16 版を明示して起動する場合:

```bash
ros2 launch susumu_object_perception simulation.launch.py lidar_model:=vlp16
ros2 launch susumu_object_perception webots_simulation.launch.py world:=outdoor_vlp16.wbt lidar_model:=vlp16
```

---

## launch（エントリポイント）

主な launch は `simulation.launch.py`（Gazebo 全部入り）、`webots_simulation.launch.py`（Webots）、
`webots_city_mapping.launch.py`（自律マッピング）、`webots_waypoint_nav.launch.py`（ウェイポイント巡回）。

**各 launch が何を起動するか・全引数の一覧は [`docs/launch.md`](docs/launch.md) を参照。**

---

## ロボット / LiDAR 構成と制約

Gazebo Classic の標準ロボットは TurtleBot3 Waffle に上部 3D LiDAR を載せた構成で、
URDF/SDF の識別子、topic、frame はセンサ製品名に依存しない汎用名にしている。LiDAR link は
`lidar_link`、点群 topic は `/lidar/points`。標準 `lidar_model:=mid360` は
`liblivox_mid360_sensor.so`（LCAS/livox_laser_simulation_ros2 由来、ODE MultiRayShape 方式）が
MID-360 の scan pattern CSV（`config/mid360_scan_patterns/mid360.csv`）を読み、`x,y,z,intensity,tag,line`
付き PointCloud2 を出す（frame は sensor 名 = `lidar_link`）。VLP-16 版は `models/turtlebot3_waffle_vlp16/` と
`urdf/turtlebot3_waffle_vlp16.urdf.xacro` に残してあり、`lidar_model:=vlp16` で使う。

Webots の標準 world は Webots 標準 `Lidar` による MID-360 近似で、device 名は `lidar3d`、
frame は `lidar_link`、topic は `/lidar/points/point_cloud`。仰角中心を MID-360 の +22.5° に
合わせる `tiltAngle` を設定済み。VLP-16 用 world は `*_vlp16.wbt` として別に残している。

制約事項:

- Gazebo Classic 版 MID-360 は ODE MultiRayShape で CSV の非反復角度列に実 ray を撃つ。
  per-point timestamp は出さない（`x,y,z,intensity,tag,line`、tag/line はダミー 0）。
- Webots 標準 `Lidar` では Livox/MID-360 の非反復 scan pattern を直接指定できないため、FOV・レンジ・点密度の近似に留めている。
- Nav2/AMCL 用 `/scan` は 2D LiDAR ではなく、3D LiDAR 点群から `pointcloud_to_laserscan` で生成する。
- downstream の perception、色付き点群、GLIM 設定は汎用 topic/frame に寄せており、旧 `/velodyne_points` / `velodyne_link` 前提ではない。

---

## ライセンス

MIT License（[`LICENSE`](LICENSE)）。TurtleBot3 モデルは ROBOTIS、HuNavSim は
robotics-upo に帰属。
