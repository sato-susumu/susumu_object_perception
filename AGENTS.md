# susumu_sim — AGENTS

AIエージェント（および新規参加者）向けの作業ガイド。このパッケージで変更を加えるときの
前提・規約・落とし穴をまとめる。詳細な構築履歴は [`SETUP.md`](SETUP.md)、利用方法は
[`README.md`](README.md) を参照。

## 何のパッケージか

ROS 2 Humble + **Gazebo Classic 11** 上のシミュレーション統合パッケージ。

- 家(house) world で **HuNavSim が5人の歩行者**を歩かせる
- **3D LiDAR(VLP-16相当) 搭載 TurtleBot3** を spawn
- **Nav2** で自律移動（3D点群を costmap の障害物として使用）
- **LiDARのみで人を検知し「ある1人の右隣を歩く」追従**ノード（`person_detector` + `follow_person`）

ビルド種別は **`ament_cmake`**（Pythonノードは `install(PROGRAMS)` + `ament_python_install_package`
で同梱）。

## 絶対に守る制約・方針

- **Gazebo は Classic 11**。Ignition/Gazebo Sim ではない。
  - HuNavSim は必ず **`v1.0-humble`** ブランチを使う（`v2.0` は Gazebo Sim 用で動かない）。
- **追従機能は HuNavSim の真値を覗かない**。`/people` `/human_states` 等は使用禁止。
  人の検知は **`/velodyne_points`（3D LiDAR）のみ**から行う（これが要件）。
- **独自メッセージは作らない**。標準型のみ（`PointCloud2` / `PoseArray` / `MarkerArray` /
  `nav2_msgs/NavigateToPose`）。
- ワークスペースの source は **`install/local_setup.bash`** を使う。
  `install/setup.bash` は**古いスナップショットを指す prefix-chain** で、新規パッケージが
  見えず `package not found` になる（既知の罠。SETUP.md「Phase B」参照）。

## ビルド・実行

```bash
cd ~/ros2_ws
colcon build --packages-select susumu_sim --symlink-install

source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash     # ← setup.bash ではない
export TURTLEBOT3_MODEL=waffle

ros2 launch susumu_sim simulation.launch.py              # 全部入り
ros2 launch susumu_sim simulation.launch.py follow:=true # 追従ON
```

Pythonノードはファイル名で起動する（console_scripts ではない）:
`ros2 run susumu_sim person_detector_node.py` / `follow_person_node.py`。
→ ノードを増やすときは CMakeLists の `install(PROGRAMS ...)` に**ファイルを追加し、
かつソースに実行ビット(`chmod +x`)を立てる**こと（忘れると `No executable found`）。

## アーキテクチャ / データフロー

```
hunav_house.launch.py : hunav_loader → hunav_gazebo_world_generator
                        → generatedWorld.world（actor5体+HuNavプラグイン）→ gzserver/gzclient
                        + hunav_agent_manager（SFM behavior駆動）
spawn_robot.launch.py : robot_state_publisher(URDF) + spawn_entity(SDF: 3D LiDAR等のプラグイン)
nav2_bringup         : AMCL(生 /scan) + costmap(obstacle_layer=/scan_filtered,
                       voxel_layer=/velodyne_points_filtered) + planner/controller
follow_person.launch : person_detector_node ─┬→ /perception/persons → follow_person_node
                                              │                         → navigate_to_pose
                                              ├→ /velodyne_points_filtered ┐ (人を除去)
                                              └→ /scan_filtered            ┴→ Nav2 costmap
```

> 検知ノードは「人検知(/perception/persons)」と「costmap用に人を除去したセンサ
> (/velodyne_points_filtered, /scan_filtered)」の両方を出力する。後者を Nav2 が使うことで
> 歩く人が costmap に軌跡の帯を残さない。

simulation.launch.py は上記を **TimerActionで段階起動**（gazebo→+8s robot→+12s nav2/rviz
→+18s follow）。順序依存（robotが居ないとnav2のTFが揃わない等）があるため、遅延値を
むやみに詰めない。

## フレーム/トピックの約束（変更時は両側を揃える）

| 役割 | 値 | 定義場所 |
|---|---|---|
| 速度司令 | `cmd_vel` | SDF diff_drive ↔ nav2 controller |
| オドメトリ | frame/topic `odom`（`publish_odom_tf:true`） | SDF diff_drive ↔ amcl odom_frame |
| ベース | `base_footprint`(amcl) / `base_link`(costmap) | SDF / URDF / nav2_params |
| 2D LiDAR(生) | `/scan`, frame `base_scan` | SDF laser ↔ amcl scan_topic ↔ person_detector |
| 3D LiDAR(生) | `/velodyne_points`, frame `velodyne_link` | SDF gpu_ray ↔ person_detector |
| 2D LiDAR(人除去) | `/scan_filtered` | person_detector ↔ nav2 obstacle_layer |
| 3D LiDAR(人除去) | `/velodyne_points_filtered` | person_detector ↔ nav2 voxel_layer |
| HuNav追跡対象 | robot_name=`turtlebot3`（spawn entity名と一致必須） | hunav_house / spawn_robot |

## 重要ファイル

- `models/turtlebot3_waffle_3d/model.sdf` … Gazeboプラグイン本体。3D LiDARは `gpu_ray`
  センサ + `libgazebo_ros_ray_sensor.so`（`output_type: PointCloud2`）。
  変更後は `gz sdf -k model.sdf` で spec 検証する。
- `config/nav2_params.yaml` … waffle.yaml ベース。**voxel_layer の observation を
  `/velodyne_points`(PointCloud2)** に変更してある。AMCLは2Dの`/scan`のまま。
- `susumu_sim/person_detector_node.py` … 高さ帯フィルタ + scipy.ndimage グリッドCCクラスタ
  + 簡易最近傍トラッキング。`/people`を読んでいないことを必ず維持。
- `susumu_sim/follow_person_node.py` … ロックオン + 右側ゴール幾何。
  「右」は**進行方向の右**（heading法線 `(sin h, -cos h)`）。

## 変更時の検証手順（サンドボックスでGazebo実起動が不安定な場合）

実環境ではライブ起動で確認するのが基本だが、Gazebo起動がブロックされる環境では以下で代替:

```bash
# SDF/URDF/YAML/launch の静的検証
gz sdf -k models/turtlebot3_waffle_3d/model.sdf
xacro urdf/turtlebot3_waffle_3d.urdf.xacro > /dev/null
python3 -c "import yaml; yaml.safe_load(open('config/nav2_params.yaml'))"
# launch記述のビルド確認 / ノードの構築確認（rclpy.init→Node()）も有効

# 右側幾何の単体確認（person@(2,0) heading=east → goal.y が負＝右）
```

> 既知の環境制約: 本リポジトリのサンドボックスでは、Gazeboを起動するlaunchを
> バックグラウンド/timeoutで回すとプロセスが即終了し標準出力が残らないことがある。
> ライブ目視は通常端末で行う。

## やりがちな失敗

- `install/setup.bash` を source して `package not found` → `local_setup.bash` を使う。
- 新Pythonノードに実行ビットを立て忘れて `No executable found`。
- HuNavSim を `v2.0` で入れて Gazebo起動失敗。
- 追従ノードで `/people` を読んでしまう（要件違反）。検知は `/velodyne_points` のみ。
- robot spawn の entity名と HuNav の robot_name 不一致で `Robot model ... not found` が出続ける。
- **Nav2 paramsを `turtlebot3_navigation2` の waffle.yaml から作らない。**
  そのパッケージは新しいNav2の `::` プラグイン名形式で、インストール済み Nav2 1.1.20
  （`/` 形式）と不整合 → `planner_server` が `NavfnPlanner does not exist` で落ちる。
  **同梱バージョンと一致する `nav2_bringup/params/nav2_params.yaml` をベースにする**こと。
  現行の `config/nav2_params.yaml` は対処済み（planner=`nav2_navfn_planner/NavfnPlanner`）。
- **`agents_house.yaml` の `once: true` は数十秒で behavior が失効し人が止まる。**
  継続的に歩かせたいなら `once: false` + `cyclic_goals: true`（現行設定）。
  「ロボットが追従しない」原因が「人が止まっている」ことは多い。まず `/people`(真値,
  デバッグ可) と `/perception/persons`(検知) の両方で人が動いているか確認する。
- **検知がロボット至近の自己点群/壁を人としてロックする問題に注意。**
  `person_detector` は `range_min`(センサ近傍除外) と `publish_only_moving`(動物体のみ)で
  対処済み。追従が「目の前のゴールで動かない」ときはこれを疑う。
- **人の移動軌跡が costmap に帯として残る問題は「3Dと2Dの両方」から人を除去して解決。**
  `person_detector` は人を除去した `/velodyne_points_filtered`(3D) と `/scan_filtered`(2D) を
  出力し、Nav2 の voxel_layer/obstacle_layer がそれぞれを使う。**片方だけフィルタすると
  残ったセンサ（特に生 `/scan` の obstacle_layer）が帯を作る**。AMCL は生 `/scan` のまま。
  検証は global costmap の lethal セル数が増え続けないことで確認できる。

## 動作確認の作法（このリポジトリ）

- Gazebo起動launchは **`run_in_background:true`（デタッチ）** で起動し、出力ファイルを
  `Read`/`grep` でポーリングする。フォアグラウンド+timeoutだと即終了して出力が残らない。
- 確認すべき要点: planner作成ログ、`Managed nodes are active`、`person_detector up`、
  `Locked target`、`Reached the goal`/preemption、odom座標の変化。
- 終了処理: `ps aux | grep -E "gzserver|component_container|follow_person" | awk '{print $2}'
  | xargs -r kill -9`。`pkill`は環境によりツールのexit code 1を招くので xargs+kill が安全。
