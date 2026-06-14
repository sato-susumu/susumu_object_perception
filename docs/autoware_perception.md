# Autoware LiDAR sensing/perception パイプライン

3D LiDAR の点群から周囲の物体（特に歩く人）を**検出・追跡**するパイプライン。
検出までは **Autoware 純正モジュール**を使い、Autoware 公式に存在するが apt で入手
できない**追跡（multi_object_tracker）・可視化は Python で自作**して補完する。

> このフェーズの目的は **perception の確立と可視化**。Nav2 とは連携させない
> 検出・追跡の結果は RViz の MarkerArray で可視化する。**さらに、Autoware の地面
> 除去済み点群 `/perception/no_ground/pointcloud` を Nav2 costmap(voxel_layer) の入力に
> 使う**（生 `/velodyne_points` は地面を含み costmap を埋め尽くして自動巡回不可に
> なるため。下「自動巡回」参照）。tracked_objects 自体は costmap には焼かない。

> HD 地図は使わない。Autoware の検出は本来 HD 地図（drivable area / ROI）で絞るが、
> ここでは点群ジオメトリのみで検出し、代わりに **2D 占有格子地図 `/map` と照合する
> `map_roi_filter_node.py`** で壁・地図外・未知に当たる検出を除外する（HD 地図 ROI の
> 2D 代替）。

## データフロー

```
/velodyne_points (PointCloud2 / PointXYZI, frame: velodyne_link)
  │
  ├─[自作Py] pointcloud_to_autoware_node.py        PointXYZI → PointXYZIRC 変換
  │     → /perception/points_autoware   ※ground_filter は ring/channel 必須（後述）
  │
  ├─[Autoware] autoware_crop_box_filter            ROI クロップ（±13m=店内, z -0.5..2.0）
  │     CropBoxFilterNode → /perception/cropped/pointcloud
  │
  ├─[Autoware] autoware_ground_filter              Scan Ground Filter で地面除去
  │     GroundFilterComponent → /perception/no_ground/pointcloud ──┐
  │                                                                 │（Nav2 voxel_layer へ）
  ├─[Autoware] autoware_euclidean_cluster_object_detector  クラスタ化│
  │     EuclideanClusterNode → /perception/detected_objects         │
  │                                                                 │
  ├─[自作Py] map_roi_filter_node.py                2D 地図照合 ROI   │
  │     壁/地図外/未知の検出を除外 → /perception/detected_objects_in_map
  │                                                                 │
  ├─[自作Py] object_tracker_node.py                フレーム間追跡    │
  │     → /perception/tracked_objects (TrackedObjects)              │
  │                                                                 │
  └─[自作Py] perception_marker_node.py             RViz 可視化       │
        → /perception/markers (MarkerArray)                         │
                                                                    ▼
  Nav2 costmap の 3D 障害物層（STVL=stvl_layer）は mark に地面除去済み
  /perception/no_ground/pointcloud を使う（生 /velodyne_points は地面 46% を含み
  costmap を埋め尽くすため。下「自動巡回」参照）。clear は STVL の frustum 用に生
  /velodyne_points を使う。詳細は docs/nav2_tuning.md。
```

上 3 つの Autoware モジュールは composable node なので 1 つの `component_container`
（`autoware_perception_container`）にまとめてロードする（intra-process 通信）。
自作 Python ノードは rclpy なので通常 Node として別プロセスで起動する。

起動は `launch/include/autoware_perception.launch.py`。`simulation.launch.py` から
`use_perception:=True`（既定）で robot spawn の後（+18s）に TimerAction で起動する。
追跡は `odom ← velodyne_link` の TF を使うため robot/TF が揃ってから起動する必要がある。

## 使う Autoware パッケージ（apt）

```bash
sudo apt-get install -y \
  ros-humble-autoware-crop-box-filter \
  ros-humble-autoware-ground-filter \
  ros-humble-autoware-euclidean-cluster-object-detector \
  ros-humble-autoware-vehicle-info-utils \
  ros-humble-autoware-perception-msgs
```

| モジュール | plugin（component） | 役割 |
|---|---|---|
| `autoware_crop_box_filter` | `autoware::crop_box_filter::CropBoxFilterNode` | ROI クロップ |
| `autoware_ground_filter` | `autoware::ground_filter::GroundFilterComponent` | 地面除去（Scan Ground Filter） |
| `autoware_euclidean_cluster_object_detector` | `autoware::euclidean_cluster::EuclideanClusterNode` | クラスタ化 → DetectedObjects |

plugin 名は `ros2 component types` で実体を確認して確定した（後述の落とし穴参照）。

## 自作モジュール（Python）の設計と Autoware ソースとの対応

apt に **`autoware_multi_object_tracker`（追跡）と `autoware_shape_estimation`（形状推定）が
無い**ため Python で補完する。実装は Autoware 公式ソース
（`github.com/autowarefoundation/autoware.universe` の `perception/autoware_multi_object_tracker`）
を読み、設計判断とパラメータ既定値を踏襲した**縮小版**。

### object_tracker_node.py（multi_object_tracker の縮小版）

| 観点 | Autoware の実装 | 本実装の選択と根拠 |
|---|---|---|
| 追跡フレーム | `world_frame_id: map` | `odom`。地図前提を避ける。固定フレームならロボット自己移動を見かけ速度から除ける |
| モーションモデル | 歩行者は CTRV `[x,y,yaw,v,ω]` | 等速 CV `[x,y,vx,vy]`。点群重心の追跡では向き推定が不安定。屋内低速なら CV で十分 |
| 速度上限 | `cv_motion_model.hpp` の `max_vx/vy` | `max_vel`（既定 2.78m/s、歩行者上限）で毎ステップ速度をクランプし発散を防ぐ |
| アソシエーション | GNN / Mu-SSP（大域最適） | ハンガリアン法 `scipy.optimize.linear_sum_assignment`。同じ大域最適でクロスに強い |
| 距離尺度・ゲート | 非車両はマハラノビス距離 + χ²(2自由度) 閾値 **11.62**（99.6%） | 予測共分散からマハラノビス距離を計算し、同じ 11.62 でゲート。保険にユークリッド距離ゲートも併用 |
| existence_probability | Bayes 更新（測定 TP=0.9/FP=0.2、非測定は半減期 **0.5s** で指数減衰） | 同式で更新。`tp/fp/decay_half_life` をパラメータ化 |
| トラック削除 | `isExpired`：経過時間 1.0s / 確率下限 / 共分散 | 経過時間 `max_age_sec`（1.0s）と確率下限 `min_existence` で削除 |
| プロセスノイズ | 位置小・速度大（加速度吸収） | `q_pos=0.025, q_vel=2.0`（Autoware の傾向を踏襲、屋内向けに調整） |
| is_stationary | トラッカー型（StaticTracker）で決定 | 型分割を持たないので **速度 + 累積変位の二段判定**で代替（静止什器の誤動的化を防ぐ） |
| ちらつき抑制 | 確率クランプ・確信度判定 | `min_hits`（既定 2）未満のトラックは出力しない |
| 壁際 FP 除去 | （地図照合は持たない） | **出力段で 2D 地図照合**。壁近傍に張り付く静止トラック（壁上の緑ボックス）を消す。下記参照 |

出力は Autoware 標準型 `autoware_perception_msgs/TrackedObjects`。`object_id`(UUID) は
内部 ID 先頭 4 バイトに埋め込み、可視化側で復元する。独自メッセージは作らない。

#### トラッカー出力段の 2D 地図照合（壁際の緑ボックス対策）

検出段の `map_roi_filter_node.py` で壁検出を弾いても、散発的に通った壁付近の検出を
トラッカーが予測ドリフトで生かし続け、**壁際に張り付く静止トラック（壁上の緑ボックス）**
として残る。これを出力段でも 2D 地図（`/map`）と照合して断つ。`_track_blocked_on_map`
がトラック位置を map 座標へ変換し、占有セル近傍なら出力しない。

ground truth（シミュ内の人・机の真値位置）で検証した結果、机とゴーストは
hits/existence/変位では区別できない（どちらも不動・高 existence）が、**壁からの距離**で
分離できる（ゴーストは壁 0.5〜1.4m、机は壁 1.5m 以上）。そこで壁 margin を二段にする:

- **移動トラック（歩行者）**: `wall_margin_moving_cells`（既定 6 = 0.30m）。壁ぎりぎりを
  歩く人を取りこぼさないよう狭く保つ。
- **静止トラック**（`_is_stationary` が真）: `wall_margin_static_cells`（既定 22 = 1.10m）。
  壁から離れた不動ゴーストまで壁扱いにして消す。机は壁から 1.5m 以上離れているため
  この margin でも巻き込まれず残る。

加えて、SLAM 地図 `maps/cafe.pgm` には机も薄く占有セルとして焼き込まれているため、
margin を広げると机検出まで壁判定で消えてしまう。そこで **`maps/clear_tables.py` で机
5卓の周辺 0.5m を free 化した地図**を使う（机は `/velodyne_points` → voxel_layer で動的に
障害物化するので static から消しても Nav2 の衝突回避は効く）。

ground truth 検証（cafe world, 歩行者5・机5）: 当初 壁 FP 10個 → 二段 margin + 机消去地図で
**常駐 1個 + 散発 1〜2個**まで削減。人 5・机 4 は安定して残存。残る常駐 1個は最奥小部屋の
壁 1.4m 地点のゴーストで、机の最寄り壁 1.51m と差が小さく、これ以上 margin を上げると机を
巻き込むため現状で許容する。

### perception_marker_node.py（自作可視化）

可視化は自作の `perception_marker_node.py`（標準 `visualization_msgs/MarkerArray`）で行う。
Autoware 純正 `autoware_perception_rviz_plugin` も使えるが、表示方法・色を自由に
作り込みたいので自作する。RViz では `/perception/markers` を MarkerArray Display で表示。

- 青（半透明）: 検出クラスタ `/perception/detected_objects_in_map`（壁除去後）
- 赤: 追跡中かつ**移動**物体（`is_stationary=false`）
- 緑: 追跡中だが**静止**物体（壁・什器）
- 白テキスト: `<ラベル名>  <速度>[km/h]`（Autoware 純正プラグインと同じ文字列。
  ラベル名は classification を `UNKNOWN`/`PEDESTRIAN`/`CAR` 等に対応づけ）
- 黄矢印: 速度ベクトル（移動物体のみ）

> 純正プラグインを使う場合は `autoware_perception_rviz_plugin/DetectedObjects`・
> `/TrackedObjects` を .rviz に置き、`Object Fill Type: Fill`（既定 skeleton は細線で
> 見えにくい）、クラス色は `UNKNOWN: {Color, Alpha}` で .rviz 上書き可能（実機検証済み）。
> 本構成では自作マーカーを採用しているため未使用。

### shape_estimation について

`autoware_shape_estimation` は今回**導入していない**。euclidean_cluster が出す
DetectedObjects の shape をそのまま使い、空なら人サイズの既定 BBox を補う。
Autoware の shape_estimation は歩行者=Cylinder（最小外接円）、車両=L字フィット
（rotating calipers + closeness criterion）で向き推定するが、屋内・低速・可視化目的
では重心追跡で足りるため見送った。将来 OBB が必要になれば
`bounding_box.cpp` の L字フィット（角度 1°grid search で closeness 最大化）を
`scipy.optimize.minimize_scalar` で再現できる。

## パラメータ（`config/`）

| ファイル | 対象 | 屋内向けの主な調整 |
|---|---|---|
| `autoware_crop_box.param.yaml` | crop_box | ±15m / z -0.5..2.0。frame は velodyne_link（launch で個別指定） |
| `autoware_ground_filter.param.yaml` | ground_filter | `grid_mode_switch_radius: 8.0`、`grid_size_m: 0.3`、`non_ground_height_threshold: 0.15` |
| `autoware_euclidean_cluster.param.yaml` | euclidean_cluster | `tolerance: 0.4`、`min_cluster_size: 5`、`use_height: false` |
| `autoware_vehicle_info.param.yaml` | vehicle_info | 乗用車サイズ → TurtleBot3 waffle の極小値（自車近傍の人を消さない） |

ground_filter / cluster はチューニング対象。値を変えたら本表も更新すること。

## ビルド・実行

```bash
cd ~/ros2_ws
colcon build --packages-select susumu_sim --symlink-install
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
export TURTLEBOT3_MODEL=waffle

# 全部入り（perception 既定 ON、可視化は RViz の "Perception Markers" / "No-Ground Cloud"）
ros2 launch susumu_sim simulation.launch.py

# perception だけ切る
ros2 launch susumu_sim simulation.launch.py use_perception:=false
```

確認トピック:

```bash
ros2 topic hz /perception/no_ground/pointcloud   # 地面除去後の点群
ros2 topic echo /perception/detected_objects --once   # クラスタ検出
ros2 topic echo /perception/tracked_objects --once    # ID・速度付き追跡
```

## 自動巡回（Nav2 costmap）と地面除去

**症状（ライブで判明）:** 自動巡回が全く動かない。原因は Nav2 の costmap が地面を
障害物として焼き、local_costmap の **約 90% が LETHAL** で埋まっていたこと。プランナー
が経路を引けない。

**根本原因:** Nav2 costmap が**生の `/velodyne_points`（地面点を 46% 含む）**と、地面
すれすれの高さ帯で作った `/scan` を入力にしていた。velodyne_link は地面 +0.21m に
あり、`min_obstacle_height` などの高さフィルタも velodyne_link 基準で評価されるため、
地面（z≈-0.21）が障害物高さ帯に入って床全面が障害物化していた。

**対処:**
1. costmap voxel_layer の入力を `/velodyne_points` →
   **`/perception/no_ground/pointcloud`（Autoware で地面除去済み）**に変更
   （`config/nav2_params.yaml`、local/global 両方）。高さ帯は velodyne_link 基準で
   `min -0.18 / max 1.8`。
2. `/scan`（obstacle_layer / AMCL 用）の生成高さ帯を `min_height -0.20 → 0.0`（地面
   +0.21m 以上）に上げ、地面を 2D スキャンからも除外（`spawn_robot.launch.py`）。

**結果:** local_costmap LETHAL 90% → 37%（残りは地図の壁=static_layer）。プランナーで
経路生成成功（(0,0)→(0,3) で 138 waypoints）、NavigateToPose でゴール到達を確認。
→ **「ちゃんと地面除去できているか」は costmap の LETHAL 率で確認するのが早い**
（`/local_costmap/costmap` の data>=99 の割合）。90% 近ければ地面が焼かれている。

## 2D 地図照合 ROI フィルタ（map_roi_filter_node.py）

検出のうち 2D 占有格子地図 `/map` 上で「壁(占有)／地図外／未知」に当たるものを除外し、
地図内フリースペースの物体だけ通す。HD 地図 ROI フィルタの 2D 代替。

- 各検出重心を map 座標へ TF 変換し、乗るセルの占有値で判定。
- `wall_margin_cells`（既定 3、地図 res 0.05m なら ±15cm）で壁周辺も占有扱いにし、壁に
  貼り付いた静止クラスタ（壁上の緑ボックス）を落とす。人は壁から離れるので残る。
- ライブ確認: 生検出 18 → 照合後 10（margin=3）。移動トラック 6（人）は残存。
- map<-velodyne_link の TF が要る（map->odom は AMCL/Nav2 提供）。Nav2 無し起動時は
  TF 不在で**素通し**にして perception を止めない。

## 試行錯誤・落とし穴の記録

- **【最重要・ライブ起動で判明】ground_filter は Gazebo の PointXYZI を受け付けない。**
  `autoware_ground_filter` は入力点群が ring/channel を持つ Autoware 独自型
  `PointXYZIRC` / `PointXYZIRCAEDT` であることを要求し、Gazebo velodyne プラグインの
  生 `/velodyne_points`（古典的 `PointXYZI`, point_step=16, x/y/z/intensity が全て
  float）を渡すと

  ```
  [ground_filter]: The pointcloud layout is not compatible with PointXYZIRCAEDT or PointXYZIRC. Aborting
  [ground_filter]: The pointcloud layout is compatible with PointXYZI. You may be using legacy code/data
  ```

  を出して**地面除去以降が一切流れない**（静的検証・component ロード確認だけでは
  気付けない。実点群を流して初めて出る）。対処として自作の
  `pointcloud_to_autoware_node.py` をパイプライン先頭に入れ、各点の仰角
  `atan2(z, hypot(x,y))` を 16 等分して `channel`(ring) を復元し、`intensity` を
  uint8 化、`return_type=1` を付けて `PointXYZIRC`（offset: x@0 y@4 z@8 intensity@12
  return_type@13 channel@14、point_step=16）へ変換する。numpy 構造化 dtype の
  itemsize がちょうど 16 でパディングが入らないことを確認済み（点ごとの struct
  ループは数万点で遅いので構造化配列で一括 tobytes する）。

- **plugin 名は launch ファイルのコメントを信じない。** ground_filter の
  `ground_filter.launch.py` には `autoware::ground_filter::GroundFilterComponent` と
  あるが、別資料で見かける `ScanGroundFilterComponent` は誤り。`ros2 component types`
  で実登録名を確認して確定した。crop_box は `CropBoxFilterNode`、cluster は
  `EuclideanClusterNode`。

- **crop_box の frame はパラメータファイルでは渡せない。** param file は範囲と
  `negative` のみ。`input_frame` / `output_frame` / `input_pointcloud_frame` は
  launch でノードパラメータとして個別に渡す。入力 `/velodyne_points` は velodyne_link
  なので 3 つとも velodyne_link にして無変換で処理させる。

- **euclidean_cluster の出力 frame は入力点群の frame を引き継ぐ。** EuclideanClusterNode
  （非 voxel 版）は `input_frame` パラメータを持たず、出力 DetectedObjects は
  velodyne_link になる。追跡は固定フレームで行う必要があるので、tracker 側で
  `odom ← velodyne_link` を TF 変換してから追跡する。

- **vehicle_info を乗用車デフォルトのままにしない。** ground_filter 等が自車サイズで
  足元の点を除外する。デフォルト（wheel_base 2.74m など）だと TurtleBot3 周辺の人の
  点まで消えかねないので waffle の極小値にする。

- **追跡は robot spawn より後に起動する。** `odom ← velodyne_link` の TF が無いと
  tracker が変換に失敗して何も出さない。simulation.launch.py では robot(+15s) の後
  +18s で perception を起動している。遅延値をむやみに詰めない。

- **`--symlink-install` でノードを増やしたとき。** 実行ビット（`chmod +x`）を立て、
  CMakeLists の `install(PROGRAMS)` に追加する。忘れると `No executable found`。

## 検証状況

- Autoware 3 モジュール: apt インストール後 `ros2 component types` で plugin 登録を確認。
- 自作 tracker のコアロジック: Gazebo なしで単体テスト済み。
  - 等速 1.0m/s 追跡で速度推定 ≈1.0m/s
  - 静止点は低速判定（誤動的化なし）
  - 速度クランプ（max_vel 2.78 で頭打ち）
  - existence の半減期 decay（0.8 → 0.5s 後 0.4）
  - マハラノビスゲート通過
  - すれ違い（クロス）時の ID 取り違えなし（ハンガリアン法）
- launch パース: `ros2 launch ... --show-args` で確認済み。
- **Gazebo 実起動でのライブ確認済み（cafe world + HuNav 歩行者5人）:**
  - パイプライン全段が ~9Hz で流れる: `/velodyne_points`(9.2) →
    `/perception/points_autoware`(9.2) → `/perception/cropped/pointcloud`(9.2) →
    `/perception/no_ground/pointcloud`(9.1) → `/perception/detected_objects`(9.2)。
  - `/perception/tracked_objects` で 30+ トラックを追跡、`frame_id=odom`。
  - 定常状態で **移動 4〜5 / 静止 29 前後**に分離（cafe の歩行者5人に一致）。移動
    トラックの速度 1.0〜2.5m/s は HuNav の歩行速度（vel 0.6〜0.8, max_vel 1.5）と整合。
  - `/perception/markers`（自作可視化）で検出=青 / 移動=赤 / 静止=緑、テキストは
    `<ラベル名>  <速度>[km/h]`。
  - **ライブで判明した実問題と対処:** (1) ground_filter が PointXYZI を拒否 →
    `pointcloud_to_autoware_node.py` で PointXYZIRC へ変換（上記「落とし穴」参照）。
    (2) 静止什器がクラスタの揺れで「移動」と誤判定 → 移動判定を初期位置からの累積
    ではなく直近 `disp_window`(2.0s) 窓内の実移動量に変更し、閾値を vel 0.3 / disp 0.7
    に引き上げて収束。
- 残課題（可視化フェーズのスコープ外）: クラスタの分裂・融合で短命の移動トラックが
  一時的に湧く。euclidean_cluster のチューニングや人サイズフィルタ追加で軽減可能。
