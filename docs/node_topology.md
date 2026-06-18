# ノード接続図 / トピック I/O 一覧

このパッケージの ROS 2 ノードのつながりと、各ノードの入力・出力トピックをまとめる。
ノードが増えて全体像が見えにくくなったため、**どのノードがどのトピックで繋がっているか**を
一望できるようにしたもの。各ノードの内部設計は [`software_design.md`](software_design.md)、
セマンティック物体メモリ系は [`semantic_object_memory.md`](semantic_object_memory.md) を参照。

> 凡例: 図中の色は **緑=Autoware 純正モジュール / オレンジ=自作 Python ノード /
> 青=外部（Gazebo/Nav2/地図）/ 灰=GUI・補助**。トピック名は `simulation.launch.py` の
> 既定配線（`semantic_memory:=True image_recognition:=True`）に準拠。

---

## 全体像（サブシステム間の流れ）

```mermaid
flowchart TD
  GZ["Gazebo Classic 11<br/>(SDF: MID-360 LiDAR / 6面カメラ)"]:::ext
  PERC["perception パイプライン<br/>(検出→追跡→予測)"]:::own
  IMG["画像認識<br/>(全天球合成→YOLO分類 / 信号認識)"]:::own
  MEM["セマンティック物体メモリ + 行動層<br/>(記憶→クエリ/追従/探索)"]:::own
  NAV["Nav2<br/>(AMCL / costmap / planner)"]:::ext
  GUI["GUI 群<br/>(teleop / 記憶一覧)"]:::aux

  GZ -->|"/lidar/points"| PERC
  GZ -->|"/omni_camera/*"| IMG
  GZ -->|"/lidar/points → /scan"| NAV
  PERC -->|"/perception/tracked_objects"| IMG
  IMG -->|"/perception/tracked_objects_classified"| MEM
  PERC -->|"/perception/predicted_costmap"| NAV
  MEM -->|"NavigateToPose"| NAV
  GUI -->|"/cmd_vel / /semantic_query / /object_seek"| MEM
  GUI -->|"/cmd_vel / NavigateToPose"| NAV
  NAV -->|"/cmd_vel"| GZ

  classDef own fill:#e65100,stroke:#bf360c,color:#fff;
  classDef ext fill:#1565c0,stroke:#0d47a1,color:#fff;
  classDef aux fill:#455a64,stroke:#263238,color:#fff;
```

---

## 1. perception パイプライン（LiDAR 検出 → 追跡 → 予測）

`launch/include/autoware_perception.launch.py` が起動。Autoware 純正 3 モジュール（緑）と
自作 Python ノード（オレンジ）が直列に繋がる。

```mermaid
flowchart TD
  PTS["/lidar/points<br/>(PointCloud2)"]:::ext
  MAP["/map<br/>(OccupancyGrid)"]:::ext

  P2A["pointcloud_to_autoware"]:::own
  CROP["crop_box_filter"]:::aw
  GND["ground_filter"]:::aw
  CLU["euclidean_cluster"]:::aw
  SHAPE["shape_estimation"]:::own
  DBT["detection_by_tracker"]:::own
  ROI["map_roi_filter"]:::own
  TRK["object_tracker"]:::own
  PRED["prediction"]:::own
  MARK["perception_marker"]:::own

  PTS --> P2A -->|"/perception/points_autoware"| CROP
  CROP -->|"/perception/cropped/pointcloud"| GND
  GND -->|"/perception/no_ground/pointcloud"| CLU
  CLU -->|"/perception/detected_objects"| SHAPE
  GND -.->|"no_ground (点群)"| SHAPE
  SHAPE -->|"/perception/detected_objects_shaped"| DBT
  TRK -.->|"/perception/tracked_objects (前フレーム参照)"| DBT
  GND -.->|"no_ground (点群)"| DBT
  DBT -->|"/perception/detected_objects_merged"| ROI
  MAP --> ROI
  ROI -->|"/perception/detected_objects_in_map"| TRK
  MAP --> TRK
  TRK -->|"/perception/tracked_objects"| PRED
  MAP --> PRED
  PRED -->|"/perception/predicted_objects"| MARK
  PRED -->|"/perception/predicted_costmap (→Nav2)"| OUT(["Nav2 costmap"]):::ext
  TRK -->|"/perception/tracked_objects (→画像認識/メモリ)"| OUT2(["画像認識へ"]):::own
  ROI -.->|"detected_objects_in_map"| MARK
  TRK -.->|"tracked_objects"| MARK
  MARK -->|"/perception/markers"| RVIZ(["RViz"]):::ext

  classDef aw fill:#2e7d32,stroke:#1b5e20,color:#fff;
  classDef own fill:#e65100,stroke:#bf360c,color:#fff;
  classDef ext fill:#1565c0,stroke:#0d47a1,color:#fff;
```

> `detection_by_tracker` は `object_tracker` の前フレーム出力を参照する循環構造（破線）。

---

## 2. 画像認識（全天球合成 → YOLO 物体分類 / 信号認識）

`image_recognition:=True` で起動。Gazebo の 6 面カメラを全天球に合成し、その画像で
LiDAR 検出物体（tracked_objects）を YOLO 分類し、信号も認識する。

```mermaid
flowchart TD
  CAMS["/omni_camera/&lt;6面&gt;/image_raw<br/>(Image ×6)"]:::ext
  TRK["/perception/tracked_objects<br/>(TrackedObjects)"]:::own
  LID["/lidar/points<br/>(PointCloud2)"]:::ext

  OMNI["omni_image"]:::own
  CLS["object_classifier"]:::own
  TLD["traffic_light_detector"]:::own
  TLM["traffic_light_marker"]:::own
  TLL["traffic_light_localizer"]:::own

  CAMS --> OMNI
  OMNI -->|"/omni_camera/image_raw (合成全天球)"| CLS
  TRK --> CLS
  CLS -->|"/perception/tracked_objects_classified (→メモリ)"| OUT(["メモリへ"]):::own
  CLS -->|"/perception/object_fine_classes (DiagnosticArray, COCO細クラス→メモリ)"| OUT
  CLS -->|"/perception/object_classes/markers"| RVIZ(["RViz"]):::ext

  OMNI -->|"/omni_camera/image_raw"| TLD
  TLD -->|"/perception/traffic_signals (autoware型)"| OUT2(["走行制御 等"]):::own
  TLD -->|"/perception/traffic_light/rois"| TLM
  TLD -->|"/perception/traffic_light/rois"| TLL
  OMNI -->|"image"| TLM
  TLM -->|"/perception/traffic_light/image_annotated"| RVIZ
  LID --> TLL
  TLL -->|"/perception/traffic_light/poses + /markers"| RVIZ

  classDef own fill:#e65100,stroke:#bf360c,color:#fff;
  classDef ext fill:#1565c0,stroke:#0d47a1,color:#fff;
```

> COCO 細クラス（chair 等）は Autoware label に無いため、`object_classifier` が
> `object_id→COCO名` を `/perception/object_fine_classes`（DiagnosticArray）で副配信し、
> メモリが什器を区別して記憶する。詳細は [`semantic_object_memory.md`](semantic_object_memory.md)。

---

## 3. セマンティック物体メモリ + 行動層

`semantic_memory:=True` で起動。検出物体を map 座標で永続記憶し、自然語クエリで移動・
追従・探索する。SQLite DB（`~/.ros/object_memory.sqlite3`）を介して疎結合。

```mermaid
flowchart TD
  TC["/perception/tracked_objects_classified<br/>(TrackedObjects)"]:::own
  FC["/perception/object_fine_classes<br/>(DiagnosticArray)"]:::own
  MAP["/map (OccupancyGrid)"]:::ext

  MEM["object_memory"]:::own
  DB[("object_memory.sqlite3")]:::ext
  QRY["semantic_query"]:::own
  SEEK["object_seeker"]:::own
  MGUI["object_memory_gui"]:::aux
  TGUI["teleop_gui"]:::aux
  NAV(["Nav2 NavigateToPose"]):::ext

  TC --> MEM
  FC --> MEM
  MAP --> MEM
  MEM -->|"書込"| DB
  MEM -->|"/semantic_memory/markers"| RVIZ(["RViz"]):::ext

  DB -->|"読出"| QRY
  DB -->|"読出"| SEEK
  DB -->|"読出"| MGUI

  TGUI -->|"/semantic_query (String)"| QRY
  MGUI -->|"/semantic_query (String)"| QRY
  QRY -->|"NavigateToPose"| NAV
  QRY -->|"/semantic_query/result"| OUT(["結果通知"]):::aux

  TGUI -->|"/object_seek (String)"| SEEK
  TC --> SEEK
  SEEK -->|"NavigateToPose"| NAV
  SEEK -->|"/object_seek/status"| OUT
  SEEK -->|"/cmd_vel (見失い停止)"| GZ(["Gazebo"]):::ext

  TGUI -->|"/cmd_vel / /initialpose / NavigateToPose"| NAV

  classDef own fill:#e65100,stroke:#bf360c,color:#fff;
  classDef ext fill:#1565c0,stroke:#0d47a1,color:#fff;
  classDef aux fill:#455a64,stroke:#263238,color:#fff;
```

---

## ノード別 I/O 一覧表

`…/` は `susumu_object_perception/`。型は ROS 2 メッセージ型。

### perception パイプライン

| ノード | 入力 | 出力 |
|---|---|---|
| `pointcloud_to_autoware` | `/lidar/points` (PointCloud2) | `/perception/points_autoware` (PointCloud2) |
| `crop_box_filter`(AW) | `/perception/points_autoware` | `/perception/cropped/pointcloud` |
| `ground_filter`(AW) | `/perception/cropped/pointcloud` | `/perception/no_ground/pointcloud` |
| `euclidean_cluster`(AW) | `/perception/no_ground/pointcloud` | `/perception/detected_objects` (DetectedObjects) |
| `shape_estimation` | `/perception/detected_objects` + `/perception/no_ground/pointcloud` | `/perception/detected_objects_shaped` |
| `detection_by_tracker` | `…_shaped` + `/perception/tracked_objects`(前F) + `no_ground` | `/perception/detected_objects_merged` |
| `map_roi_filter` | `/perception/detected_objects_merged` + `/map` | `/perception/detected_objects_in_map` |
| `object_tracker` | `/perception/detected_objects_in_map` + `/map` | `/perception/tracked_objects` (TrackedObjects) |
| `prediction` | `/perception/tracked_objects` + `/map` | `/perception/predicted_objects` + `/perception/predicted_costmap` |
| `perception_marker` | `…_in_map` + `tracked_objects` + `predicted_objects` | `/perception/markers` (MarkerArray) |

### 画像認識 / 信号

| ノード | 入力 | 出力 |
|---|---|---|
| `omni_image` | `/omni_camera/<6面>/image_raw` (Image) | `/omni_camera/image_raw` (合成) ほか |
| `object_classifier` | `/omni_camera/image_raw` + `/perception/tracked_objects` | `/perception/tracked_objects_classified` + `/perception/object_fine_classes` (DiagnosticArray) + `/perception/object_classes/markers` + `…/image_annotated` |
| `object_image_crop` | `/omni_camera/image_raw` + `/perception/tracked_objects` | `/perception/object_crops/image_rect` |
| `traffic_light_detector` | `/omni_camera/image_raw` | `/perception/traffic_signals` + `/perception/traffic_light/rois` |
| `traffic_light_marker` | `image` + `…/rois` | `/perception/traffic_light/image_annotated` |
| `traffic_light_localizer` | `/lidar/points` + `…/rois` | `/perception/traffic_light/poses` + `…/markers` |
| `colorized_pointcloud` | `/omni_camera/image_raw` + `/lidar/points` | `/perception/colorized_points` |
| `pointcloud_intensity` | `/lidar/points` | `/lidar/points_intensity` |

### セマンティック物体メモリ / 行動層 / GUI

| ノード | 入力 | 出力・接続 |
|---|---|---|
| `object_memory` | `/perception/tracked_objects_classified` + `/perception/object_fine_classes` + `/map` + TF | `/semantic_memory/markers` + **DB 書込** |
| `semantic_query` | `/semantic_query` (String) + **DB 読** + TF | `/semantic_query/result` (String) + **NavigateToPose** |
| `object_seeker` | `/object_seek` (String) + `/perception/tracked_objects_classified` + **DB 読** + TF | `/object_seek/status` (String) + `/cmd_vel` + **NavigateToPose** |
| `object_memory_gui` | **DB 読** | `/semantic_query` (String) |
| `teleop_gui` | (GUI 操作) | `/cmd_vel` + `/initialpose` + `/semantic_query` + `/object_seek` + **NavigateToPose** |

> `patrol_waypoints.py` はモジュール（ノードではない）。`PATROL_WAYPOINTS`（cafe 巡回 8 点）を
> `teleop_gui`（自動巡回）と `object_seeker`（SEARCH モード）が import して共有する。

### 外部・補助

| トピック | 供給 | 利用 |
|---|---|---|
| `/scan` (LaserScan) | pointcloud_to_laserscan（`/lidar/points` から生成） | AMCL / Nav2 obstacle_layer |
| `/perception/predicted_costmap` (OccupancyGrid) | prediction | Nav2 `PredictedCostmapLayer`（C++ 層） |
| `navigate_to_pose` (Action) | Nav2 | teleop_gui / semantic_query / object_seeker |
