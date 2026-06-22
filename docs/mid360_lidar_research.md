# MID-360 LiDAR 移行調査メモ

調査日: 2026-06-18

目的: 既存の VLP-16 版ロボット定義を残しつつ、標準で使う 3D LiDAR を Livox MID-360 相当に変更できるか確認する。

## scan pattern CSV の取得（リポジトリに含めない）

`config/mid360_scan_patterns/*.csv`（`mid360.csv` 約 16MB、`mid360-real-centr.csv` 約 25MB）は
大容量のため **`.gitignore` でリポジトリから除外**している。Gazebo MID-360 プラグイン
(`liblivox_mid360_sensor`) はこの CSV を読むので、clone 後に別途取得・配置する。

```bash
# LCAS/livox_laser_simulation_ros2 の scan_mode/ から取得（mid360.csv はここ由来と同一）
cd /tmp && git clone --depth 1 https://github.com/LCAS/livox_laser_simulation_ros2.git
cp /tmp/livox_laser_simulation_ros2/scan_mode/mid360.csv \
   ~/ros2_ws/src/susumu_object_perception/config/mid360_scan_patterns/
# mid360-real-centr.csv は ctu-mrs/Mid360_simulation_plugin 由来（任意。既定は mid360.csv を使用）
```

CSV が無いと MID-360 プラグインは `Failed to read MID-360 scan pattern` 等で点群を出さない。
ライセンス表記（`LCAS_LIVOX_LICENSE` / `ROS2_LIVOX_SIMULATION_LICENSE`）は同ディレクトリに残してある。

## 結論

変更は可能。標準ロボットは MID-360 に変更し、VLP-16 版は明示的な別ファイルとして残せばよい。

ただし、標準の Gazebo Classic `gpu_ray` と Webots `Lidar` だけでは MID-360 の非反復スキャンパターンを完全再現できない。点群密度・FOV・レンジを合わせた近似は簡単にできるが、Livox らしい scan pattern や per-point timestamp を厳密に使う SLAM 検証では専用プラグインが必要になる。

## ローカル確認

Gazebo Classic 側:

- `launch/include/spawn_robot.launch.py` は `urdf/turtlebot3_waffle_3d.urdf.xacro` と `models/turtlebot3_waffle_3d/model.sdf` を固定参照している。
- `urdf/turtlebot3_waffle_3d.urdf.xacro` は `base_link -> velodyne_link` を z=0.20m で追加している。
- `models/turtlebot3_waffle_3d/model.sdf` は `velodyne_vlp16` `gpu_ray` を持ち、`/velodyne_points` / `velodyne_link` を publish する。設定は 900 x 16、水平 360deg、垂直 -15..+15deg、10Hz、range 0.3..30m。

Webots 側:

- `resource/turtlebot_webots_3d.urdf` は Webots device `velodyne` を `/velodyne_points`、frame `velodyne_link` として有効化している。
- `webots_worlds/outdoor.wbt`、`indoor.wbt`、`calibration.wbt` は `Lidar { name "velodyne" ... }` を持つ。現状は水平 360deg、垂直 0.5236rad、16 layers、range 0.3..30m。
- `launch/webots_simulation.launch.py` は `/velodyne_points/point_cloud` と `velodyne_link` を downstream に固定している。

Downstream の固定箇所:

- `/scan` 生成: `pointcloud_to_laserscan` が `/velodyne_points` または `/velodyne_points/point_cloud` を入力にしている。
- Autoware perception: `autoware_perception.launch.py` は input topic だけ launch 引数化済みだが、crop_box の frame は `velodyne_link` 固定。
- 色付き点群: `colorized_pointcloud_node.py` と `pointcloud_intensity_node.py` の既定が `/velodyne_points/point_cloud`。
- GLIM: `config/glim_webots/config_ros.json` が `/velodyne_points/point_cloud_intensity` を読む。
- docs/rviz/config も `velodyne` 名を多数参照している。

## MID-360 仕様

公式仕様で実装に効く値:

- FOV: horizontal 360deg、vertical -7deg..+52deg
- close proximity blind zone: 0.1m
- range: 40m at 10% reflectivity、70m at 80% reflectivity
- point rate: 200,000 points/s first return
- frame rate: 10Hz typical
- range precision: <=2cm at 10m
- angular precision: <0.15deg
- built-in IMU
- dimensions: 65 x 65 x 60mm、weight 265g

Livox ROS Driver 2 は ROS 2 Humble を対象に含み、MID-360 用 launch を持つ。出力は Livox PointCloud2、Livox CustomMsg、標準 PointXYZI 相当の形式が選べる。

参照:

- https://www.livoxtech.com/mid-360/specs
- https://github.com/Livox-SDK/livox_ros_driver2

## 実装方針案

### 案 A: 互換名のまま差し替え

`/velodyne_points` と `velodyne_link` の名前を維持し、センサ中身だけ MID-360 相当にする。

利点:

- downstream 変更が最小。
- 既存の perception、色付き点群、GLIM、RViz がほぼそのまま動く。

欠点:

- topic/frame 名が実機センサ名と合わない。
- VLP-16 と MID-360 を同時比較すると意味が混ざる。

### 案 B: MID-360 名にする

`mid360_link` と `/mid360_points` を使い、launch 引数で downstream に伝播させる。

利点:

- 実機・ログ・ドキュメントと意味が揃う。
- VLP-16 版を明確に残せる。

欠点:

- launch/config/docs の更新箇所が増える。
- `autoware_perception.launch.py` の frame 固定を `lidar_frame` 引数化する必要がある。

推奨は案 B。`lidar_model:=mid360|vlp16`、`lidar_frame`、`lidar_points_topic`、Webots 用の `lidar_points_topic_actual` を launch 引数化し、既定を MID-360 にする。VLP-16 は `*_vlp16` の別ファイルへ分けて `lidar_model:=vlp16` で呼べるようにする。

## シミュレーター別の可否

### Gazebo Classic 11

近似実装は可能。

- 既存の `gpu_ray` を使い、FOV/range/update_rate/samples を MID-360 相当に変更した `models/turtlebot3_waffle_mid360/model.sdf` を追加する。
- 200k points/s / 10Hz を近似するなら 1 frame 約 20k points。例: 720 x 28 = 20160 points/frame。
- vertical FOV は min=-0.122173rad、max=0.907571rad にする。
- range は min=0.1m、max=40m を基本にし、屋内負荷が問題なければ 70m も試す。
- noise は range precision に合わせて stddev 0.02m 付近から始める。

完全再現が必要な場合のネット調査結果:

- Livox 公式 `livox_laser_simulation` は MID-360 の scan_mode CSV を持つが、README 上は ROS Melodic / Gazebo 9 前提。
- ROS 2 port の `livox_laser_simulation_RO2` は Foxy/Humble tested とされ、PointCloud2 と Livox CustomMsg を出せる。
- `ctu-mrs/Mid360_simulation_plugin` は Gazebo 11 対応・standalone・歪み補正あり。ただし ROS Noetic/catkin 前提なので、この ROS 2 Humble パッケージで使うなら port か外部 workspace 連携が必要。

参照:

- https://github.com/Livox-SDK/livox_laser_simulation
- https://github.com/stm32f303ret6/livox_laser_simulation_RO2
- https://github.com/ctu-mrs/Mid360_simulation_plugin

### Webots

近似実装は可能。既存 Webots 検証環境で最初に動かすならこちらが安全。

- `webots_worlds/*` の `Lidar` を MID-360 相当に変更するか、`*_mid360.wbt` を追加する。
- 水平 `fieldOfView 6.283185`、垂直 `verticalFieldOfView 1.029744`、range `minRange 0.1` `maxRange 40.0` から始める。
- 点群数は `horizontalResolution * numberOfLayers`。720 x 28 なら 10Hz で約 201k points/s 相当。
- Webots の `Lidar` は `tiltAngle` と `verticalFieldOfView` を持つため、MID-360 の -7..+52deg を表すには垂直中心を +22.5deg へ傾ける設定を実機検証する必要がある。

制約:

- Webots `Lidar` は layers/grid ベース。Livox の非反復スキャンパターンそのものは再現しない。
- 公式 docs にも point cloud mode は計算負荷が高いとあるため、720 x 28 以上は実時間性を確認する。
- `Lidar` には `numberOfLayers < verticalFieldOfView * horizontalResolution / fieldOfView` の制約がある。720 x 28, FOV 2pi, vertical 1.03rad は制約内。

参照:

- https://raw.githubusercontent.com/cyberbotics/webots/master/docs/reference/lidar.md

## 次に実装するなら

1. 現在の VLP-16 定義を別名ファイルに退避する。
   - Gazebo: `urdf/turtlebot3_waffle_vlp16.urdf.xacro`、`models/turtlebot3_waffle_vlp16/model.sdf`
   - Webots: `resource/turtlebot_webots_vlp16.urdf`、`webots_worlds/*_vlp16.wbt`
2. 標準ファイルまたは既定 launch は MID-360 を指すようにする。
   - Gazebo: `urdf/turtlebot3_waffle_mid360.urdf.xacro`、`models/turtlebot3_waffle_mid360/model.sdf`
   - Webots: `resource/turtlebot_webots_mid360.urdf`、`webots_worlds/*_mid360.wbt`
3. launch に `lidar_model` / `lidar_frame` / `lidar_points_topic` を追加し、既定を MID-360 にする。
4. Autoware perception の frame 固定を `lidar_frame` 引数化する。
5. GLIM config は MID-360 用に `config/glim_webots_mid360/` を追加する。
6. 検証は `/scan`、Autoware 検出、色付き点群、GLIM colored SLAM の順に行う。

## Livox simulation plugin 検討

追加調査日: 2026-06-18

目的: MID-360 を単なる regular grid LiDAR として近似するだけでなく、Livox の非反復 scan pattern までシミュレーションできる plugin を使うべきか判断する。

### 候補

| 候補 | ROS 2 / Humble | Gazebo Classic 11 | MID-360 pattern | 所感 |
|---|---:|---:|---:|---|
| `stm32f303ret6/livox_laser_simulation_RO2` | あり | 動作確認済み | `scan_mode/mid360.csv` | 第一候補。ROS 2 package として build 可能。要小パッチ。 |
| `Livox-SDK/livox_laser_simulation` | なし（ROS 1） | README は Gazebo 9 | `scan_mode/mid360.csv` | 公式系だが ROS 1/Melodic 前提。直接採用しない。 |
| `ctu-mrs/Mid360_simulation_plugin` | なし（ROS 1/catkin） | README は Gazebo 11 対応 | `mid360-real-centr.csv` | 歪み補正・standalone は良いが ROS 2 port が必要。将来候補。 |

参照:

- https://github.com/stm32f303ret6/livox_laser_simulation_RO2
- https://github.com/Livox-SDK/livox_laser_simulation
- https://github.com/ctu-mrs/Mid360_simulation_plugin

### ROS 2 port のローカル検証

一時 workspace `/tmp/mid360_plugin_build` で `stm32f303ret6/livox_laser_simulation_RO2` を build した。

結果:

- `source /opt/ros/humble/setup.bash`
- `source ~/ros2_ws/install/local_setup.bash`
- `colcon build --packages-select ros2_livox_simulation`
- build 成功。警告のみ。
- `livox_ros_driver2` はこの環境に既に存在した: `~/ros2_ws/install/livox_ros_driver2`
- Gazebo Classic 11.10.2 で plugin load 成功。
- テスト world で `/mid360` と `/mid360_PointCloud2` が publish された。
- `/mid360_PointCloud2.header.frame_id = mid360_link`
- fields は `x,y,z`、`point_step=12`
- `samples=20000` に対して `width=40000` だった。

試行メモ:

- `ros2_livox_simulation` の CMake には `libprotobuf.so.9` と `libboost_chrono.so.1.71.0` の古い直指定があるが、この環境では link は成功し、実体は Ubuntu 22.04 の `libprotobuf.so.23` / Boost 1.74 に解決された。
- plugin の PointCloud2 publisher は `<topic>_PointCloud2` 固定。`<topic>/mid360_points</topic>` のように指定しても実トピックは `/mid360_points_PointCloud2` になる。
- plugin の ROS 2 `CustomMsg` publisher は `<topic>` に出る。これは `livox_ros_driver2/msg/CustomMsg` で、既存パッケージ内に独自 `.msg` を追加する必要はない。
- `src/livox_points_plugin.cpp` は PointCloud に同一点を2回 `emplace_back()` しているように見える。テストでも `samples=20000` が `width=40000` になったため、採用前に修正する。
- PointCloud2 は intensity を持たない。既存 `pointcloud_intensity_node.py` は x/y/z だけで疑似 intensity を作れるため GLIM 用には問題ない。Autoware 変換も intensity 無しを処理できる。

### 採用判断

採用するなら `stm32f303ret6/livox_laser_simulation_RO2` をベースにする。

ただし、そのまま外部 package として使うより、以下を最小 patch した fork または workspace 内 package として固定するのが安全。

必須修正:

1. PointCloud2 の二重点追加を直す。
2. PointCloud2 topic を `<pointcloud2_topic>` で直接指定できるようにする。既存互換のため未指定時だけ `<topic>_PointCloud2` にする。
3. `frame_id` を `<frame_name>` で指定できるようにする。未指定時だけ sensor name を使う。
4. `samples` の既定を 20,000 points/frame 付近にする。200,000 points/s / 10Hz に合わせるため。
5. package.xml の license が TODO なので、外部依存として扱う場合も README に由来と注意点を書く。

推奨構成:

- Webots: native `Lidar` で MID-360 近似。Gazebo plugin は Webots では使えない。
- Gazebo Classic: plugin 版 MID-360 を標準にする。`gpu_ray` grid 近似は fallback として残す。
- VLP-16: `*_vlp16` ファイルとして退避し、`lidar_model:=vlp16` で明示起動。

実装時の topic/frame 案:

- plugin raw PointCloud2: `/mid360_points`
- plugin CustomMsg: `/mid360/livox`
- frame: `mid360_link`
- `/scan` 生成: `/mid360_points` -> `/scan`
- Autoware perception: `input_pointcloud:=/mid360_points`, `lidar_frame:=mid360_link`
- 色付き点群: `input_cloud:=/mid360_points`
- GLIM intensity cloud: `/mid360_points_intensity`

### ctu-mrs / fratopa 版の再検討

追加調査日: 2026-06-18

`ctu-mrs/Mid360_simulation_plugin` と元の `fratopa/Mid360_simulation_plugin` も確認した。

ネット調査結果:

- README 上の主張は、Gazebo 11 対応、ROS Noetic 対応、standalone、Livox SDK/driver 不要、custom message formats、point cloud distortion 補正。
- 論文 `SIMULATION OF LOW-COST MEMS-LIDAR AND ANALYSIS OF ITS EFFECT ON THE PERFORMANCES OF STATE-OF-THE-ART SLAMS` と紐づいている。MID-360 の helicoidal non-repeating scan pattern を使い、SLAM の odometry / mapping quality を評価する目的で作られている。
- `fratopa` README では出力形式として `sensor_msgs/PointCloud`、`sensor_msgs/PointCloud2(x,y,z)`、`sensor_msgs/PointCloud2(x,y,z,intensity,tag,line,timestamp)`、Livox custom msg を選べる。

ローカルソース確認:

- この環境には ROS Noetic/catkin が無いので、ctu-mrs 版そのものの build/run はしていない。
- `ctu-mrs` clone の default branch は `refactoring`。一時 clone の commit は `dca0420 the pose is now loaded from the parent link for easier manipulation with the SDF`。
- package は ROS 1 `catkin`、`roscpp`、`tf`、`message_generation` 前提。
- `scan_mode/mid360-real-centr.csv` は 800,000 行。ROS 2 port の `mid360.csv` と比べると azimuth が 180deg ずれているように見える（例: `88.99deg` vs `268.99deg`）。実装側の座標系補正とセットで見る必要がある。

ROS 2 port に取り込む価値がある差分:

1. ray start の歪み補正
   - ROS 2 port は ray start を `minDist * axis + offset.Pos()` にしている。
   - ctu-mrs 版は `minDist * axis + offset.Pos() - minDist * axis`、つまり実質 `offset.Pos()` にしている。
   - README の「distortion corrected」はこの周辺の修正と関係している可能性が高い。採用するなら ROS 2 port 側にも取り込む。
2. 出力形式選択
   - ctu-mrs 版は `publish_pointcloud_type` で出力形式を選べる。
   - 特に `PointCloud2(x,y,z,intensity,tag,line,timestamp)` は GLIM/FAST-LIO 系の検証に有利。
3. frame/topic 指定
   - ctu-mrs 版は `ros_topic` と `frameName` を SDF から読む。
   - ROS 2 port は `<topic>_PointCloud2` 固定、frame は `raySensor->Name()` 固定なので、ctu-mrs 版の設計へ寄せたほうが扱いやすい。
4. timestamp/line
   - ctu-mrs 版は `line` と per-point `timestamp` を持つ point type を用意している。
   - ROS 2 port の CustomMsg 側にも `offset_time` はあるが、PointCloud2 は x/y/z のみで、現状のままだと Livox らしい downstream 検証には弱い。

注意点:

- ctu-mrs 版をそのまま採用するのは不可。ROS 1/catkin なので、この ROS 2 Humble package へ入れるには port が必要。
- ctu-mrs 版にも細かい癖はある。たとえば `PublishPointCloud()` 内で `frame_id` を `sensor_frame_name_` にした直後に `"livox"` で上書きしている箇所がある。採用時は丸ごと移植ではなく、必要な修正だけを ROS 2 port へ移す。
- `libprotobuf.so.9` 直 link は ROS 2 port / ctu-mrs 派生どちらにも残りがち。Ubuntu 22.04 では `libprotobuf.so.23` なので CMake は整理する。

更新後の推奨:

- ベースは `stm32f303ret6/livox_laser_simulation_RO2`。
- ただし、実装時に ctu-mrs 版から以下を取り込む:
  - ray start 歪み補正
  - direct `pointcloud2_topic`
  - explicit `frame_name`
  - `publish_pointcloud_type`
  - `PointCloud2(x,y,z,intensity,tag,line,timestamp)` 出力
  - `mid360-real-centr.csv` を使うかどうかの比較スイッチ

## Webots での MID-360 再現性

追加調査日: 2026-06-18

結論: Webots では「MID-360 風の FOV/点密度/レンジの 3D LiDAR」は十分作れるが、Livox の非反復 scan pattern を Webots 標準 `Lidar` へ直接指定する公開手段は見当たらない。

ネット調査:

- `Webots Livox MID-360 simulation Lidar`、`Webots Livox Mid360 Lidar point cloud ROS2`、`Webots non-repetitive lidar Livox simulation`、`site:github.com Webots "Mid-360" Lidar` などで検索。
- 具体的な Webots 用 MID-360 plugin / PROTO / scan pattern 実装は見つからなかった。
- 出てくる実装・記事はほぼ Gazebo 系。modern Gazebo では MID-360 を `gpu_lidar` で近似する記事があり、Gazebo Classic では Livox plugin 系が中心。
- Isaac Sim の forum でも MID-360 の非反復 scan を simulator が標準対応しないことが話題になっている。これは Webots でも同じ性質の問題。
- CoppeliaSim forum でも Mid360 simulation の相談があるが、標準モデルの話ではなく「どう近似するか」という温度感。

Webots 公式仕様から分かること:

- Webots `Lidar` は `horizontalResolution`、`fieldOfView`、`verticalFieldOfView`、`numberOfLayers`、`near/minRange/maxRange`、`type fixed|rotating`、`projection planar|cylindrical` を持つ。
- point cloud API の点は `x,y,z,layer_id,time` を持つ。
- point cloud mode は計算負荷が高い。
- `numberOfLayers < verticalFieldOfView * horizontalResolution / fieldOfView` の制約がある。
- rotating lidar の point cloud が歪む場合、simulation time step を下げる必要がある。

ローカル確認:

- `webots_ros2_driver` には `Ros2Lidar` plugin があり、`LaserScan` と `PointCloud2` を publish する。
- Webots C/C++ API は `getRangeImage()` / `getPointCloud()` / `getLayerPointCloud()` などを提供している。
- API から任意の Livox CSV scan pattern を `Lidar` ノードへ直接与える口は見当たらない。

実装方針:

1. Webots 標準 MID-360 近似
   - `fieldOfView 6.283185`
   - `verticalFieldOfView 1.029744`（59deg）
   - `tiltAngle` で中心を +22.5deg 相当に調整するか、センサ姿勢で合わせる。
   - `horizontalResolution 720`, `numberOfLayers 28` から開始し、10Hz で約 201,600 points/s 相当。
   - `minRange 0.1`, `maxRange 40.0` から開始。
2. Webots Livox pattern 風後処理
   - 高密度 regular `Lidar` を出し、別 ROS 2 node で `mid360.csv` / `mid360-real-centr.csv` の角度列に近い点だけを subsample / interpolate して `/mid360_points` を publish する。
   - これは「見た目と点分布を近づける」手段で、Gazebo plugin のような ray-level の厳密な衝突判定ではない。
3. Webots ではまず標準近似で SLAM/色付き点群の回帰検証を安定させる。
   - 高忠実度 MID-360 pattern 検証は Gazebo Classic plugin 側へ寄せる。

Webots の位置づけ:

- 本プロジェクトでは「Webots は全天球カメラ・色付き点群・GLIM の統合検証を動かす主環境」として有用。
- 「MID-360 のスキャンパターン忠実性」は Gazebo Classic plugin のほうが向いている。
- Webots で無理に scan pattern を完全再現するより、標準近似 + 必要なら後処理 node に留めるのが現実的。

## 最終実装サマリ

ロボット/URDF/トピック/フレーム名はセンサ名に寄せず汎用名に統一する。

| 項目 | 現行 |
|---|---|
| 標準 Gazebo ロボット | `models/turtlebot3_waffle_3d/model.sdf` |
| VLP-16 退避版 | `models/turtlebot3_waffle_vlp16/model.sdf` |
| 標準 URDF | `urdf/turtlebot3_waffle_3d.urdf.xacro` |
| LiDAR frame | `lidar_link` |
| Gazebo topic | `/lidar/points` |
| Webots topic | `/lidar/points/point_cloud` |
| intensity 付与後 topic | `/lidar/points_intensity` |

Gazebo Classic の MID-360 は、`LCAS/livox_laser_simulation_ros2` 由来の ODE
`LivoxOdeMultiRayShape` 方式を本パッケージへ vendoring して使う。CSV の各
(azimuth, zenith) に実 ray を撃つため、旧自作の「RaySensor 格子 + CSV 最近傍」方式で出た
上方向欠落・自己近傍偽点を避けられる。

| 項目 | 内容 |
|---|---|
| vendored source | `src/livox_ros2/livox_points_plugin.cpp`, `src/livox_ros2/livox_ode_multiray_shape.cpp`, `include/livox_ros2/*` |
| library | `liblivox_mid360_sensor.so` |
| scan pattern | `config/mid360_scan_patterns/mid360.csv` |
| 主な修正 | 無効 ray の原点出力を skip、QoS を SensorDataQoS、相対 CSV path を package share から解決、sensor 名を `lidar_link` に統一 |

検証済みの要点:

- `gz sdf -k` と `xacro` は標準/MID-360・VLP-16 退避版とも通る。
- Gazebo では `/lidar/points` が `frame_id=lidar_link`, `is_dense=true` で出る。
- 原点偽点は 0、仰角は MID-360 仕様に近い `[-7deg,+50deg]` 程度。
- `/scan` は pointcloud_to_laserscan で生成され、TF `base_link -> lidar_link` は z=0.20。
- LCAS 版の PointCloud2 は `x,y,z,intensity,tag,line`。per-point timestamp は無く、GLIM 用 intensity は `pointcloud_intensity_node` で補完する。

## Webots 側の注意

Webots は標準 `Lidar` による MID-360 近似で、非反復 scan pattern は再現しない。
`tiltAngle != 0` は Webots の既知バグ
[cyberbotics/webots #37](https://github.com/cyberbotics/webots/issues/37)
により点の高さが崩れ、2D SLAM 地図に円形影を作る。全 world で `tiltAngle 0` を使う。
このため FOV は実機 MID-360 の非対称 FOV から外れるが、2D SLAM/Nav では地面高さが正しくなる利点を優先する。

`mode:=fast` は odom を過大積算しやすい。マッピング/ナビ評価は `mode:=realtime` を使う。

屋外 `/scan` では未ヒット ray を有限値化すると free raytrace が実 occupied を消す問題があった。
屋内設定は `use_inf:True` 系に戻しており、屋外マッピングは GLIM-first 方針に切り替えている。
詳細は [`docs/tasks/mapping_outdoor.md`](tasks/mapping_outdoor.md) を参照。
