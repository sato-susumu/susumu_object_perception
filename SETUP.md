# susumu_sim 構築手順メモ

ROS2 Humble + Gazebo Classic 11 環境で、**HuNavSim で制御した5人**を **複数の部屋がつながった家(house)** で歩かせ、
**3D LiDAR搭載 TurtleBot3** が彼らを避けながら Nav2 で自律移動するシミュレーションを構築する手順の記録。

- 対象環境: ROS2 Humble / Gazebo Classic 11 / Nav2 / TurtleBot3
- ワークスペース: `/home/taro/ros2_ws`
- パッケージ: `susumu_sim`

---

## 全体構成

| 要素 | 採用 |
|---|---|
| シミュレータ | Gazebo Classic 11 |
| 場所 | `turtlebot3_house`（複数部屋がつながった家） ※ HuNav wrapper 同梱の `house.world` を使用 |
| 人(歩行者) | HuNavSim 5人（`agents_house.yaml`、Social Force Model で回避行動） |
| ロボット | TurtleBot3 waffle + 追加 3D LiDAR（Velodyne VLP-16相当 16ch） |
| 3D LiDAR | Gazebo `gpu_ray` センサ + `gazebo_ros_ray_sensor` で `/velodyne_points`(PointCloud2) を出力 |
| 自律移動 | Nav2（RViz2 でゴール指定 → 人を回避して移動） |

### HuNavSim の動作原理（重要）

1. `hunav_loader` が `agents_house.yaml`（人の初期位置・goal・behavior）を読み込む
2. `hunav_gazebo_world_generator` が `house.world` + agents 設定 → `generatedWorld.world` を生成
   （HuNavプラグイン + actor モデルを埋め込む）
3. Gazebo が生成された world を起動
4. ロボットは別 launch で spawn し、Nav2 を起動

> オリジナルの wrapper は **PMB2 ロボット**（PAL Robotics）を spawn する。
> 本プロジェクトでは PMB2 を **TurtleBot3 + 3D LiDAR** に置き換える。

---

## Phase 0: 依存パッケージの導入 ✅完了

### 0-1. HuNavSim のクローン（Gazebo Classic 用 v1.0-humble ブランチ）

```bash
cd ~/ros2_ws/src
git clone -b v1.0-humble https://github.com/robotics-upo/hunav_sim.git
git clone -b v1.0-humble https://github.com/robotics-upo/hunav_gazebo_wrapper.git
```

> ⚠️ `v2.0` ブランチは Gazebo Sim(Ignition Fortress)向け。本環境は Gazebo **Classic 11** なので **v1.0-humble** を使う。

### 0-2. people_msgs（apt に無いのでソースから）

```bash
cd ~/ros2_ws/src
git clone -b ros2 --depth 1 https://github.com/wg-perception/people.git people_repo
mv people_repo/people_msgs ./people_msgs
rm -rf people_repo
```

### 0-3. lightsfm（Social Force Model ヘッダライブラリ。これが無いと hunav_agent_manager がビルド不可）

```bash
cd ~/ros2_ws/third_party   # colcon の src 外に置く（colcon パッケージではないため）
git clone https://github.com/robotics-upo/lightsfm.git
cd lightsfm
sudo make install          # /usr/local/include/lightsfm/ にヘッダをコピー
```

### 0-4. rosdep & ビルド

```bash
cd ~/ros2_ws
rosdep install --from-paths src/people_msgs src/hunav_sim src/hunav_gazebo_wrapper --ignore-src -r -y
# behaviortree_cpp などが入る

source /opt/ros/humble/setup.bash
colcon build --packages-select people_msgs hunav_msgs --symlink-install
colcon build --packages-select hunav_agent_manager hunav_evaluator hunav_rviz2_panel hunav_gazebo_wrapper --symlink-install
```

ビルド成功で以下の実行ファイルが登録される:
- `hunav_agent_manager` / `hunav_loader`（hunav_agent_manager パッケージ）
- `hunav_gazebo_world_generator`（hunav_gazebo_wrapper パッケージ）

---

## Phase A: house world + 5人HuNav を起動 ✅完了

- wrapper 同梱の `scenarios/agents_house.yaml`（5人: agent1,2,7,4,5）を
  `susumu_sim/config/agents_house.yaml` にコピーして使用。
  - behavior: REGULAR / SURPRISED / CURIOUS / IMPASSIVE など混在、各部屋に配置、cyclic_goals。
- `susumu_sim/launch/hunav_house.launch.py` を作成（PMB2 spawn なし、HuNav部分のみ）。

### パッケージ構成

```
susumu_sim/
├── CMakeLists.txt / package.xml   # ament_cmake。launch/config/urdf/worlds/maps/rviz をinstall
├── config/agents_house.yaml       # 5人の歩行者設定
└── launch/hunav_house.launch.py   # hunav_loader → world_generator → gazebo → agent_manager
```

### 起動方法

```bash
cd ~/ros2_ws
colcon build --packages-select susumu_sim --symlink-install
source install/setup.bash
export TURTLEBOT3_MODEL=waffle
ros2 launch susumu_sim hunav_house.launch.py
```

### 動作確認結果

- 生成された `generatedWorld.world` に **actor 5体** + HuNavプラグインが埋め込まれることを確認。
- Gazebo起動、`/clock` `/human_states` `/people` トピックが publish される。
- `hunav_agent_manager` が BT ノード登録・起動。

### ⚠️ ハマりどころ（重要メモ）

1. **`from scripts import GazeboRosPaths` は使わない。**
   `GazeboRosPaths.get_paths()` は ament の全パッケージを走査するため、ワークスペース内に
   install ディレクトリが欠けた壊れたパッケージ（例: `susumu_object_tracker`）があると
   `PackageNotFoundError` で launch 全体が落ちる。→ 本launchでは get_paths() を使わず、
   `hunav_gazebo_wrapper` の models/media を `GAZEBO_MODEL_PATH`/`GAZEBO_RESOURCE_PATH` に
   直接追記する方式にした。

2. **`[hunav_plugin]: Robot model turtlebot3 not found` は Phase A では正常。**
   プラグインはロボットに対する social force を計算するためロボットを探すが、Phase A では
   まだロボットを spawn していないため出る。Phase B で TB3 を spawn すれば解消する。

3. world生成は loader 起動 → 2秒後に generator → さらに2秒後に Gazebo、という
   タイマー連鎖。起動直後はトピックが出ないので確認は ~15秒待ってから。

---

## Phase B: 3D LiDAR搭載 TurtleBot3 の作成 ✅完了

waffle の Gazebo プラグイン群は **SDF モデル**（`turtlebot3_gazebo/models/turtlebot3_waffle/model.sdf`）側に
あり、URDF にはない。そこで以下の2ファイルを作成した。

| ファイル | 役割 |
|---|---|
| `models/turtlebot3_waffle_3d/model.sdf` | waffle SDF をコピーし、上部に **VLP-16相当 16ch 3D LiDAR**（`gpu_ray` センサ）を追加。`libgazebo_ros_ray_sensor.so` で `/velodyne_points`(PointCloud2, frame=`velodyne_link`) を出力。Gazebo へ spawn する実体。 |
| `urdf/turtlebot3_waffle_3d.urdf.xacro` | 標準 waffle URDF を include し、`base_link -> velodyne_link` の TF フレームを追加。`robot_state_publisher` 用（TF専用、Gazeboプラグインは持たない）。 |

3D LiDAR 仕様: 水平 900 samples / 360°、垂直 16ch / ±15°、range 0.3〜30 m、10 Hz。

### 関連 launch

- `launch/spawn_robot.launch.py` … robot_state_publisher + spawn_entity（再利用部品）。
  `entity_name` のデフォルトは `turtlebot3`（HuNavプラグインが追跡する robot_name と一致させる）。
- `launch/test_robot_empty.launch.py` … 空world + ロボット spawn の単体確認用（`gui:=false`で headless）。

### 検証

- `gz sdf -k model.sdf` → **Check complete**（gpu_rayセンサ/プラグインが spec 適合）。
- `xacro turtlebot3_waffle_3d.urdf.xacro` → 展開成功、velodyne_link を含む。
- launch description のビルド成功（5アクション）。
- Gazebo `libGpuRayPlugin.so` / `libgazebo_ros_ray_sensor.so` 存在を確認。

### ⚠️ 超重要ハマりどころ: ワークスペースの source は `local_setup.bash` を使う

このワークスペースの `install/setup.bash` は **古いスナップショット**を指す prefix-chain で、
新規追加した `susumu_sim` / `hunav_*` / `people_msgs` が **含まれない**
（`ros2 launch susumu_sim ...` が "package not found" になる）。

→ **必ず `install/local_setup.bash` を source する**（こちらは `_local_setup_util` 経由で
最新のパッケージ一覧を持つ）。

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash   # setup.bash ではなく local_setup.bash
```

> 注: 本環境（サンドボックス）では Gazebo を起動する launch をバックグラウンド/timeout 付きで
> 回すと標準出力が取りこぼされることがある。実機（通常の端末）では問題なく出力される。
> ライブ確認は Phase D の統合 launch を通常端末で実行して行うのが確実。

## Phase C: Nav2マップ + 障害物回避構成 ✅完了

### マップ

- HuNav wrapper 同梱の house マップ（`house.pgm` / `house.yaml`）を
  `susumu_sim/maps/` にコピーして使用（SLAM 不要）。
  - resolution 0.05, origin `[-9.4, -5.57, 0]`, サイズ 375×222。

### Nav2 パラメータ（`config/nav2_params.yaml`）

TurtleBot3 `waffle.yaml` をベースに、**3D LiDAR を障害物回避に使う**よう改造:

- **AMCL**: `scan_topic: scan`（2D LiDAR で自己位置推定）、`set_initial_pose: true`
  （spawn 位置で自動初期化、RViz で初期姿勢を与えなくても動く）。
- **local/global costmap の voxel_layer**: observation を `/scan`(LaserScan) から
  **`/velodyne_points`(PointCloud2)** に変更。
  - `z_resolution: 0.1`, `z_voxels: 16`, height 0.05〜2.0 m。
  - → 歩いている人を含め、あらゆる高さの障害物を 3D で costmap にマーク。
- **obstacle_layer** は従来どおり 2D `/scan`（床近くの障害物）を併用。

> 役割分担: **2D LiDAR = 自己位置推定(AMCL) + 低い障害物**、
> **3D LiDAR = 立体的な動的障害物（人）の回避**。

## Phase D: 統合 launch + 動作確認 ✅完了

`launch/simulation.launch.py` で全要素を一括起動する。起動順（TimerActionで段階化）:

1. `hunav_house.launch.py` を include → Gazebo(house) + 5人HuNav 起動（`navigation:=True`
   なので static map->odom は出さず、Nav2/AMCL に任せる）。
2. +8秒: `spawn_robot.launch.py` を include → 3D LiDAR TB3 を spawn + robot_state_publisher。
3. +12秒: `nav2_bringup/bringup_launch.py` を include → AMCL + Nav2（map=house.yaml,
   params=nav2_params.yaml, slam=False, autostart=True）。
4. +12秒: RViz2（`rviz/simulation.rviz`、Velodyne PointCloud表示を追加済み）。

### フレーム/トピックの整合（確認済み）

| 項目 | ロボット(SDF) | Nav2(params) | 一致 |
|---|---|---|---|
| 速度司令 | `cmd_vel` (subscribe) | controller → `cmd_vel` | ✓ |
| オドメトリ | frame/topic `odom`, `publish_odom_tf:true` | odom_frame `odom` | ✓ |
| ベース | `base_footprint` | amcl base `base_footprint`, costmap base `base_link` | ✓ |
| 2D LiDAR | `scan` / frame `base_scan` | amcl scan_topic `scan` | ✓ |
| 3D LiDAR | `/velodyne_points` / frame `velodyne_link` | voxel_layer pointcloud `/velodyne_points` | ✓ |

### 検証

- 全 launch description のビルド成功（simulation: 12アクション）。
- `gz sdf -k` でモデル spec 適合、xacro 展開 OK、nav2/rviz の YAML 妥当。
- Phase A で HuNav+Gazebo パイプラインのライブ動作（プラグイン稼働）を確認済み。

> 本サンドボックス環境では Gazebo を起動する launch をバックグラウンド/timeout で回すと
> プロセスが即終了し標準出力が残らないため、**統合 launch のライブ目視確認は通常端末で**
> 実施すること（下記「実行方法」参照）。

---

## ▶ 実行方法（まとめ）

```bash
# 1. 初回のみ依存をビルド（Phase 0 を未実施なら）
cd ~/ros2_ws
colcon build --symlink-install   # もしくは個別に susumu_sim と hunav_* を

# 2. 環境を source（★ setup.bash ではなく local_setup.bash）
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
export TURTLEBOT3_MODEL=waffle

# 3. 全部入りで起動
ros2 launch susumu_sim simulation.launch.py              # 手動ゴールで自律移動
ros2 launch susumu_sim simulation.launch.py follow:=true # 人を検知して右隣を追従
```

- 手動: 起動後 RViz2 の **"2D Goal Pose"** で家の中の目的地を指定 → TB3 が 3D LiDAR で
  5人を避けながら自律移動。
- 追従(`follow:=true`): 起動後、検知した5人のうち1人にロックオンし、その人の右隣を並走
  （手動ゴール不要）。

### 段階的に動かす（デバッグ用）

```bash
# 人だけ（Phase A）
ros2 launch susumu_sim hunav_house.launch.py

# 空worldでロボット単体＋3D LiDAR確認（Phase B）
ros2 launch susumu_sim test_robot_empty.launch.py        # gui:=false で headless

# Nav2なしで人＋ロボットだけ
ros2 launch susumu_sim simulation.launch.py use_nav2:=false use_rviz:=true
```

### 確認用コマンド

```bash
ros2 topic echo /velodyne_points --once        # 3D点群（PointCloud2）
ros2 topic echo /people --once                 # HuNavの5人
ros2 topic list | grep -E "velodyne|scan|people|human_states|cmd_vel"
ros2 run tf2_tools view_frames                 # TFツリー（map->odom->base_footprint->...->velodyne_link）
```

---

## Phase E: LiDARのみで人を検知し「右隣を歩く」追従 ✅完了

> 要件: **HuNavSimの真値を覗かず**、LiDARデータから人を検知してNav2でついていく。
> その人の**進行方向の右隣**を歩く。

### ノード構成（`susumu_sim/` Pythonモジュール）

| ノード | 役割 |
|---|---|
| `person_detector_node.py` | `/velodyne_points` を TF で odom 系へ→高さ帯フィルタ→scipy.ndimage の XYグリッド連結成分クラスタリング→人サイズ抽出→簡易最近傍トラッキングで速度推定。動くクラスタ=人。出力 `/perception/persons`(PoseArray, 向き=進行方向) と `/perception/persons/markers`。**`/people`等のHuNav真値は不使用。** |
| `follow_person_node.py` | 起動時に最寄りの人をロックオン→毎周期 最近傍ゲーティングで同一ターゲット再同定→進行方向の右へ `side_offset` ずらした位置を算出→`navigate_to_pose`(Nav2)へ送信。ロスト時はその場待機し timeout で解除。出力 `/follow/goal_marker`。 |

### launch

- `launch/follow_person.launch.py` … 検知+追従の2ノード。
- `launch/simulation.launch.py` に **`follow:=true`** を追加（Nav2起動後 +18秒で追従開始）。

```bash
ros2 launch susumu_sim simulation.launch.py follow:=true
```

### ament_cmake で Python ノードを同梱する際の要点

- `CMakeLists.txt`: `ament_cmake_python` + `ament_python_install_package(${PROJECT_NAME})`
  + `install(PROGRAMS susumu_sim/*.py DESTINATION lib/${PROJECT_NAME})`。
- ⚠️ **ソースの .py に実行ビット(`chmod +x`)が必要**。無いと `ros2 run` が
  `No executable found` になる。
- 起動は **ファイル名**で: `ros2 run susumu_sim person_detector_node.py`。

### 検証

- 両ノードを rclpy で構築成功（subs/pubs/TF/action client OK）。
- **右側幾何の単体テスト**: 人が (2,0) で進行方向=東(+x) のとき、ゴールは (1.80, **−0.80**)
  = 人の**右** 0.8 m・後方 0.2 m。`y` が負＝進行方向の右側で正しい。
- 全 launch のビルド成功（simulation: 14アクション、follow: 6アクション）。

### ⚠️ 設計上の注意

- 検知は **`/velodyne_points` だけ**を入力にしている（要件の「HuNavを覗かない」を厳守）。
  HuNav の `/people` は答え合わせ用に echo するのは可だが、追従ロジックには絶対に使わない。
- 「右」= **人の進行方向に対する右**（heading の右法線 `(sin h, -cos h)`）。
  人がほぼ静止のときは直近の進行方向を流用する。

---

## 🔬 シミュレーターでの実動作確認（実施済み）

`simulation.launch.py follow:=true` をバックグラウンド起動し、実際のGazeboで検証した。

### ✅ 確認できたこと

- Gazebo(house+5人) 起動、3D LiDAR TB3 spawn 成功。
- Nav2 全ライフサイクル active（planner/controller/smoother/bt_navigator）。
- `person_detector`: `/velodyne_points` から **~9.3 Hz で 4〜6人を検知**・publish。
- `follow_person`: 人をロックオン → Nav2 が追従ゴールを実行（`Reached the goal` / 連続preemption）。
- ロボットが実際に移動し、算出した右側ゴールへ向かうことを odom で確認。

### 🐞 発見して修正したバグ

1. **Nav2 planner プラグイン名の形式不一致（致命的）**
   - 症状: `planner_server` が `[FATAL] ... class nav2_navfn_planner::NavfnPlanner does not exist`
     で落ち、navigation が起動しなかった。
   - 原因: ベースにした `turtlebot3_navigation2/param/waffle.yaml`(v2.3.6) は新しい Nav2 の
     **`::` 形式**プラグイン名を使うが、インストール済み Nav2 は **1.1.20**（`/` 形式）。
     パッケージ間のバージョン不整合。
   - 対処: `nav2_params.yaml` を **同バージョンの `nav2_bringup/params/nav2_params.yaml`
     (1.1.20)** をベースに作り直し（planner=`nav2_navfn_planner/NavfnPlanner` 等）、その上で
     3D LiDAR の voxel_layer 改造と AMCL `set_initial_pose` を再適用。

2. **追従ターゲットのロストが頻発（挙動改善）**
   - 症状: 7〜14秒ごとに `Target lost` し別人に乗り換わる。
   - 原因: follow ノードが「前回位置」中心の狭いゲート(1.0m)で再同定していたため、
     1 m/s 超で歩く人が次フレームでゲート外に出ていた。
   - 対処: ターゲットの**速度を推定し、予測位置中心でゲーティング**するよう変更。
     `lock_gate` 1.0→1.6m、`lost_timeout` 5→8s に調整。→ 1人を **~18秒以上**継続追従。

> どちらもシミュレーターを実際に動かさなければ顕在化しないバグだった（静的検証だけでは
> 検出できない）。実機確認の重要性の好例。

3. **ロボットが追従移動しない（検知が偽物をロック）**
   - 症状: ロボットがほぼ初期位置から動かない。
   - 原因: 検知ノードが **ロボット至近(0.89m)の自己点群/壁を「人」としてロック**していた。
     本物の人は2.9m以上先にいたのに、最寄り選択で偽物を掴み「既に右隣 → 動く必要なし」状態。
     加えて **全クラスタが速度0=静止判定**で、歩く人を見分けられていなかった。
   - 対処（`person_detector_node.py`）:
     - **センサ近傍フィルタ** `range_min=0.45m`（自己点群・至近誤検知を除外）
     - **速度推定を長基線化**（直近 vs 0.5秒前の履歴差分）→ 低速歩行をジッタから分離
     - **「動いている物体のみ」を `/perception/persons` に出力**（`publish_only_moving`）。
       要件の「動物体」に合致し、壁・家具・自己点群を自動除外。

---

## 📝 調整: 人の移動速度を1/3に + 確実な継続追跡

### 人の速度を1/3（`config/agents_house.yaml`）

| 項目 | 変更前 | 変更後 |
|---|---|---|
| `max_vel` | 1.5 | **0.5** |
| behavior `vel` | 0.6 / 0.8 | **0.2 / 0.27** |
| `once` | true（30〜40秒で behavior 失効→停止） | **false**（失効しない） |

> `cyclic_goals: true` と合わせ、**5人がゆっくり・無限に歩き続ける**。
> （`once: true` のままだと数十秒で止まり追跡対象が消える問題があった。）

### 確実にずっと追跡（パラメータ調整）

- `person_detector`: `moving_speed` 0.10→**0.04**（低速でも人と認識）、
  `max_misses` 15→**30**（短時間のオクルージョンでトラックを維持）。
- `follow_person`: `lost_timeout` 8→**20s**、`select_max_range` 6→**8m**、
  `min_speed_for_heading` 0.12→**0.05**、`goal_eps` 0.25→**0.2**。

### 実機検証（実施済み）

- 人の移動速度が低下（真値で確認）。
- **約55秒間 ロスト0回**で同一人物を継続追跡（preemption 84回＝活発に追従、
  `No valid trajectory` エラー 0）。ロボットが人を追って継続移動することを odom で確認。

---

## 🔧 追加調整: 右側到達・方向転換・軌跡コストマップ

利用者からの実機フィードバックに基づく改善。

### A. 人の進行方向の右側に行けない → 解決

**原因（利用者が特定）**: 歩く人が3D LiDARで障害物として検知され、**移動軌跡に沿って
costmapに障害物の帯が残り**、ロボットが人の横へ回り込めなかった。加えて inflation が
右側ゴール(0.8m)を覆っていた。

**対処**:
1. **検知ノードが「動いている人」の点群を除去した点群 `/velodyne_points_filtered` を
   publish**（`person_clear_radius=0.5m`）。Nav2 の voxel_layer はこのフィルタ済み点群を
   使う → **歩く人が costmap に障害物の帯を残さない**。壁など静止物は残すので回避は維持。
   - 点群はセンサ系(`velodyne_link`)のまま出力（Nav2 のレイトレース・クリアを効かせるため）。
2. `inflation_radius` 0.55→**0.35**、`side_offset` 0.8→**1.0m**（膨張域の外に確実に出す）。

→ 実機で `Reached the goal` 複数回・`No valid trajectory` 0 を確認（以前は立ち往生）。

### B. ロボットの方向転換が多い → 解決

**原因**: 追従ゴールの heading がターゲット速度のノイズで揺れ、右側位置が振れていた。

**対処（`follow_person_node`）**: 速度を強平滑化（`vel_smoothing=0.8`）し、**heading 自体を
角度ローパス**（`heading_smoothing=0.85`、wrap-safe）。`goal_eps` 0.25→0.35 でゴール再送も抑制。
→ ゴール更新(preemption)が 84→約22/同程度時間 に減少。

### C. 人(歩行者)の方向転換が多すぎる → 解決

**原因**: 各エージェントが3ゴールの三角形を周回し各頂点で旋回＋social力で互いに/ロボットを
避けて蛇行していた。

**対処（`agents_house.yaml`）**:
- **social_force_factor 5.0→1.0 / other_force_factor 20.0→2.0 / obstacle_force_factor 10.0→4.0**
  （蛇行を抑制）
- **ゴールを3→2個に**（g2を除外）→ 2点間を直線的に往復（旋回は両端のみ）
- `goal_radius` 0.3→0.8（ゴール手前で滑らかに切替）

→ 真値で進行方向がほぼ一定（往復）になることを確認。

### D. 人の移動軌跡のコストマップが残り続ける → 解決

**原因（核心）**: 3D点群はフィルタ済みにしたが、**Nav2 の `obstacle_layer` が生の 2D
`/scan` を使っていた**。2D LiDARは人を除去していないため、歩く人の軌跡に沿って
costmap に障害物の帯がマークされ、残り続けていた。

**対処**:
1. **検知ノードが 2D scan からも人を除去**し `/scan_filtered` を publish。
   各ビームのヒット点を odom 系へ変換し、移動中トラックの `person_clear_radius` 内なら
   range を ∞ にする（壁などは温存）。
2. 両 costmap の `obstacle_layer` を `/scan` → **`/scan_filtered`** に変更。
   （AMCL は自己位置推定のため生 `/scan` のまま。）

→ これで動的障害物の入力は **2D・3D とも人除去済み**に統一。
**実機検証**: global costmap の lethal セル数が増え続けず安定（軌跡が蓄積しない）こと、
`No valid trajectory` 0・復旧0で、ロボットが人を追って継続移動（15秒で3.1m）することを確認。

> まとめ: 「人を costmap に残さない」には **3D点群と2Dスキャンの両方**から人を
> 除去する必要がある。片方だけだと残ったセンサが軌跡の帯を作る。

---

## トラブルシュート memo

- `fatal error: lightsfm/sfm.hpp: No such file` → Phase 0-3 の lightsfm 未導入。
- `no rosdep rule for 'people_msgs'` → apt に無い。Phase 0-2 でソース導入。
- v2.0 を間違えて入れると Gazebo Sim 依存でビルド/起動に失敗する → v1.0-humble を使う。
