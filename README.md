# susumu_sim

ROS 2 Humble + Gazebo Classic 11 で、**HuNavSim が制御する5人の歩行者**が動く
**複数の部屋がつながった家**を、**3D LiDAR 搭載 TurtleBot3** が走り回るシミュレーション
パッケージ。Nav2 による自律移動に加え、**LiDAR点群から人を検知して「ある1人の右隣を歩く」**
追従機能を備える。

> 構築の詳細手順・ハマりどころは [`SETUP.md`](SETUP.md) を参照。

---

## できること

1. **家 + 5人の歩行者** — HuNavSim（Social Force Model）が5人を各部屋に配置し、
   ロボットを避けながら歩かせる。
2. **3D LiDAR TurtleBot3** — waffle に Velodyne VLP-16 相当の16ch 3D LiDAR を搭載し、
   `/velodyne_points`（PointCloud2）を出力。
3. **Nav2 自律移動** — house マップ上でゴールを指定すると、3D LiDAR で人を含む障害物を
   避けながら自律走行。
4. **人追従（右隣歩行）** — `/velodyne_points` のみから人（動物体）を検知し、1人をターゲットに
   ロックオン、その人の**進行方向の右隣**へ Nav2 ゴールを送り続けて並走する。
   **HuNavSim の真値（`/people` 等）は一切使わない**。

---

## 必要環境・依存

| 種別 | 内容 |
|---|---|
| ベース | ROS 2 Humble / Gazebo Classic 11 / Nav2 / TurtleBot3(waffle) |
| 外部クローン | HuNavSim `hunav_sim` / `hunav_gazebo_wrapper`（`v1.0-humble`）、`people_msgs`（ソース） |
| ヘッダlib | `lightsfm`（`/usr/local/include` へ `make install`） |
| Python | numpy / scipy / sensor_msgs_py |

セットアップ手順は [`SETUP.md`](SETUP.md) の「Phase 0」を参照。

---

## ビルド

```bash
cd ~/ros2_ws
colcon build --symlink-install        # または --packages-select susumu_sim hunav_* people_msgs

# ★ source は setup.bash ではなく local_setup.bash を使うこと（理由はSETUP.md参照）
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
export TURTLEBOT3_MODEL=waffle
```

---

## 実行

### 全部入り（家 + 5人 + 3D LiDAR TB3 + Nav2 + RViz2）

```bash
ros2 launch susumu_sim simulation.launch.py
```
RViz2 の **"2D Goal Pose"** で目的地を指定 → 人を避けて自律移動。

### 人追従つき（右隣を歩く）

```bash
ros2 launch susumu_sim simulation.launch.py follow:=true
```
起動後、検知した5人のうち1人にロックオンし、その人の右隣を並走する。
（手動ゴール指定は不要。追従ノードが自動でNav2にゴールを送る。）

### 段階起動（デバッグ）

```bash
# 人だけ
ros2 launch susumu_sim hunav_house.launch.py

# 空ワールドでロボット単体 + 3D LiDAR 確認
ros2 launch susumu_sim test_robot_empty.launch.py        # gui:=false で headless

# 既に動いているシミュに追従パイプラインだけ後付け
ros2 launch susumu_sim follow_person.launch.py
```

---

## 追従パイプラインの仕組み（LiDARのみ）

```
/velodyne_points (3D LiDAR, PointCloud2)
  │
  ├─ person_detector_node
  │    1. TFで odom 系へ変換
  │    2. 高さ帯フィルタ（床・天井除去, 0.1〜1.9 m）
  │    3. XY平面グリッド連結成分クラスタリング（scipy.ndimage）
  │    4. 人サイズのクラスタを抽出し、フレーム間トラッキングで速度推定
  │       → 動いているクラスタ = 人（動物体）
  │    出力: /perception/persons (PoseArray, 向き=進行方向),
  │          /perception/persons/markers (RViz可視化)
  │
  │    また人を除去した /velodyne_points_filtered (3D) と /scan_filtered (2D) を publish。
  │    → Nav2 の costmap はこの2つを使うので、歩く人が障害物の帯を残さない。
  │
  └─ follow_person_node
       1. 起動時に最寄りの人をターゲットとしてロックオン
       2. 毎周期、最近傍ゲーティングで同一ターゲットを再同定・進行方向を更新
          （速度＆heading を強平滑化 → ゴールの揺れ＝方向転換を抑制）
       3. 進行方向の「右」へ side_offset[m] ずらした位置を算出
       4. Nav2 (navigate_to_pose) にゴール送信 → Nav2が経路計画＋障害物回避
       5. ロスト時はその場で待機し、timeout内は同一ターゲットを再取得。超過でロック解除
       出力: /follow/goal_marker (RViz: 黄色矢印=現在の目標位置)
```

> **歩く人が costmap を塞ぐ / 軌跡の帯が残る問題への対処**: 検知ノードは「動いている人」を
> 半径 `person_clear_radius`(0.5m) で除去した **3D点群 `/velodyne_points_filtered` と
> 2Dスキャン `/scan_filtered` の両方**を出力し、Nav2 の costmap はこの2つを障害物入力に使う。
> → 人の移動軌跡に障害物の帯が残らず、ロボットが人の右側へ回り込める。
> 壁など静止物は残るので回避は維持。**3Dだけ・2Dだけのフィルタでは不十分**（残った
> センサが帯を作る）点に注意。AMCL の自己位置推定は生 `/scan` を使う。

> **「右隣」= 人の進行方向に対して右側**。人がほぼ静止しているときは直近の進行方向を流用。

### 主なパラメータ

**`follow_person_node`**

| パラメータ | 既定 | 意味 |
|---|---|---|
| `side_offset` | 0.8 | 人の右に保つ横距離 [m] |
| `back_offset` | 0.2 | 並走位置からの後退量 [m]（+で後ろ） |
| `lock_gate` | 1.6 | ターゲット再同定のゲート距離 [m] |
| `lost_timeout` | 20.0 | ロスト後にロック解除するまで [s] |
| `select_max_range` | 8.0 | ロック対象にできる最大距離 [m] |

**`person_detector_node`**

| パラメータ | 既定 | 意味 |
|---|---|---|
| `range_min` | 0.45 | センサ近傍を除外（自己点群対策）[m] |
| `publish_only_moving` | true | 動いている物体のみを人として出力 |
| `moving_speed` | 0.04 | 「動いている」とみなす速度しきい [m/s] |
| `max_misses` | 30 | トラックを維持する未検知許容フレーム数 |
| `person_clear_radius` | 0.5 | costmap用に人の点群を除去する半径 [m] |

```bash
# 例: 右隣を1.0mに広げて起動
ros2 launch susumu_sim simulation.launch.py follow:=true
ros2 param set /follow_person side_offset 1.0
```

### 歩行者の速度設定（`config/agents_house.yaml`）

5人はゆっくり（`max_vel: 0.5`, `vel: 0.2`）・**無限に歩き続ける**設定
（`once: false` + `cyclic_goals: true`）。速度を変えたいときはこのファイルを編集。

### うまく追従しないとき

| 症状 | 対処 |
|---|---|
| ロボットが動かない | `/perception/persons` に人が出ているか確認。出ていなければ人が静止している（速度を上げるか `moving_speed` を下げる） |
| 至近の壁/自分をロックする | `range_min` を上げる（既定0.45m） |
| すぐターゲットを見失う | `lost_timeout` / `lock_gate` を上げる |
| `No valid trajectories`（立ち往生） | 障害物に近すぎ。`inflation_radius` 調整やスポーン位置を空きスペースへ |
| 人の移動軌跡が障害物の帯として残る | costmap が `/scan_filtered` と `/velodyne_points_filtered` を使っているか確認（生`/scan`・生`/velodyne_points`を使うと帯が残る）。`person_clear_radius` を上げる |
| ロボットが方向転換しすぎる | `heading_smoothing` / `vel_smoothing` を上げ、`goal_eps` を上げる |

---

## 主要トピック

| トピック | 型 | 向き | 説明 |
|---|---|---|---|
| `/velodyne_points` | `sensor_msgs/PointCloud2` | Gazebo→検知 | 3D LiDAR生点群 |
| `/scan` | `sensor_msgs/LaserScan` | Gazebo→AMCL/検知 | 2D LiDAR生スキャン（AMCL自己位置推定用） |
| `/velodyne_points_filtered` | `sensor_msgs/PointCloud2` | 検知→Nav2 | 人を除去した3D点群（voxel_layer入力） |
| `/scan_filtered` | `sensor_msgs/LaserScan` | 検知→Nav2 | 人を除去した2Dスキャン（obstacle_layer入力） |
| `/perception/persons` | `geometry_msgs/PoseArray` | 検知→追従 | LiDAR検知した人＝動物体（向き=進行方向） |
| `/perception/persons/markers` | `visualization_msgs/MarkerArray` | 検知→RViz | 人の可視化（赤=移動/緑=静止 + 速度矢印） |
| `/follow/goal_marker` | `visualization_msgs/MarkerArray` | 追従→RViz | 追従の現在ゴール（黄色矢印） |
| `/cmd_vel` | `geometry_msgs/Twist` | Nav2→ロボット | 速度司令 |

---

## ディレクトリ構成

```
susumu_sim/
├── launch/
│   ├── simulation.launch.py      # 全部入り（follow:=true で追従ON）
│   ├── hunav_house.launch.py     # 家 + 5人HuNav のみ
│   ├── spawn_robot.launch.py     # 3D LiDAR TB3 spawn + robot_state_publisher
│   ├── test_robot_empty.launch.py# 空world + ロボット単体（3D LiDAR確認用）
│   └── follow_person.launch.py   # 人検知 + 右隣追従
├── susumu_sim/
│   ├── person_detector_node.py   # LiDAR → 人検知・トラッキング
│   └── follow_person_node.py     # ロックオン → 右隣ゴール送信
├── config/
│   ├── agents_house.yaml         # HuNav 5人の設定
│   └── nav2_params.yaml          # Nav2（3D点群を障害物層へ）
├── models/turtlebot3_waffle_3d/  # waffle + 3D LiDAR の Gazebo SDF
├── urdf/turtlebot3_waffle_3d.urdf.xacro  # TF用URDF
├── maps/house.{pgm,yaml}         # 家のマップ
└── rviz/simulation.rviz          # RViz設定（3D点群表示付き）
```

---

## ライセンス

Apache-2.0。TurtleBot3 モデルは ROBOTIS、HuNavSim は robotics-upo に帰属。
