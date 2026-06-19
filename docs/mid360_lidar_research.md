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

## 最終実装メモ

実装日: 2026-06-18

ユーザー指定により、ロボット/URDF/トピック/フレーム名は MID-360 や Velodyne に寄せず、汎用名にした。

- 標準 Gazebo ロボット: `models/turtlebot3_waffle_3d/model.sdf`
- VLP-16 退避版: `models/turtlebot3_waffle_vlp16/model.sdf`
- 標準 URDF: `urdf/turtlebot3_waffle_3d.urdf.xacro`
- VLP-16 退避版 URDF: `urdf/turtlebot3_waffle_vlp16.urdf.xacro`
- LiDAR frame: `lidar_link`
- LiDAR pointcloud topic: `/lidar/points`
- Webots pointcloud topic: `/lidar/points/point_cloud`
- intensity 付与後 topic: `/lidar/points_intensity`

Gazebo Classic の MID-360 実装:

- `libmid360_livox_sensor.so` を追加した。
- `ctu-mrs/Mid360_simulation_plugin` の `mid360-real-centr.csv` を scan pattern として取り込んだ。
- PointCloud2 は `x,y,z,intensity,tag,line,timestamp` を持つ形式を既定にした。
- ROS 1/catkin 版の plugin そのものは ROS 2 Humble へ直接使えないため、ROS 2 sensor plugin として package 内に実装した。

試行錯誤:

1. ctu-mrs / ROS2 Livox 系の ODE MultiRayShape 移植を試した。
   - `<plugin>` 内に `<ray>` 設定が必要で、最初は `Missing element description for [ray]` で Gazebo spawn が詰まった。
   - `<plugin>` 内にも `<ray>` を追加して読み込みは通した。
   - ただし Gazebo 11 の ODE ray 配列/space 管理と噛み合わず、壁ターゲットを置いても `/lidar/points` が `width=0` のままだった。
2. 最終的に Gazebo 標準 `RaySensor` の range 格子を、MID-360 CSV の角度列で最近傍サンプリングする方式にした。
   - 完全な ray-level の Livox 非反復衝突判定ではない。
   - ただし Gazebo Classic 上で安定して動き、MID-360 風の角度列・点密度・PointCloud2 フィールドを downstream に流せる。

runtime 確認:

- `colcon build --packages-select susumu_object_perception --symlink-install` 成功。
- `gz sdf -k models/turtlebot3_waffle_3d/model.sdf` 成功。
- `gz sdf -k models/turtlebot3_waffle_vlp16/model.sdf` 成功。
- `xacro urdf/turtlebot3_waffle_3d.urdf.xacro` 成功。
- `xacro urdf/turtlebot3_waffle_vlp16.urdf.xacro` 成功。
- 空 Gazebo world に標準ロボットと壁ターゲットを spawn し、`/lidar/points` を確認。
  - `frame_id: lidar_link`
  - `width=18171`
  - `point_step=32`
  - fields: `x,y,z,intensity,tag,line,timestamp`

制約:

- Gazebo Classic 版は MID-360 CSV 角度列を使うが、衝突判定は Gazebo 標準 RaySensor の格子 range から最近傍取得している。厳密な Livox ray-level simulation ではない。
- Webots 版は標準 `Lidar` による MID-360 近似。非反復 scan pattern は Webots 標準機能では再現していない。
- VLP-16 版は `lidar_model:=vlp16` で選べるが、標準は MID-360 近似。

## 検証と LCAS 版への置き換え

検証日: 2026-06-18

ライブ起動で取得データを検証した結果、上記「自作 RaySensor 格子 + CSV 最近傍」方式に
致命的な問題が見つかったため、ODE MultiRayShape 方式の LCAS 版へ置き換えた。

### 自作版で見つかった問題（ライブ検証）

`test_robot_empty.launch.py` で空 world にロボットと壁を spawn して `/lidar/points` を実測:

1. 上方向のカバレッジが枯渇。前方の壁（高さ3m）が仰角 +7° までしか取れず上半分が欠落。
   原因は垂直格子を 360° 全周に薄く張っていたこと（7.35°/本）。垂直を MID-360 の仰角域に
   絞り 128 本に密化したら全周仰角は [-6°,+52°] に改善したが、次の問題が残った。
2. 上向き ray が全周でロボット近傍（原点中心、水平 0.1〜0.4m）に偽点の環を作る。
   front range をデバッグ出力すると、empty world で前方上向き層が 0.26〜1.5m の有限値を
   返していた。RaySensor 格子の充填順と方向ベクトル生成の規約が噛み合わず、range と方向が
   ミスマッチした偽点が出る。格子最近傍方式の構造的欠陥で、格子を密にするほど顕在化する。

### 置き換え方針

`LCAS/livox_laser_simulation_ros2`（package 名 `ros2_livox_simulation`、MIT）を採用し、
プラグイン部分を本パッケージ内へ vendoring した。`stm32f303ret6/livox_laser_simulation_RO2`
の整備版で、ODE `LivoxOdeMultiRayShape` で CSV の各 (azimuth, zenith) に実 ray を撃つため、
自作版の「格子に最近傍」由来の歪み・自己遮蔽・上方向欠落が原理的に起きない。

- vendored ソース: `src/livox_ros2/livox_points_plugin.cpp`, `src/livox_ros2/livox_ode_multiray_shape.cpp`,
  `include/livox_ros2/*`。ライブラリは `liblivox_mid360_sensor.so`。
- 自作版 `mid360_livox_points_plugin.cpp` / `libmid360_livox_sensor.so` は削除した。
- CSV は `config/mid360_scan_patterns/mid360.csv`（LCAS 版と同一）。
- 依存に `livox_ros_driver2` / `std_msgs` を追加（CustomMsg 用）。`livox_ros_driver2` はこの環境に既存。

LCAS 版に加えた本パッケージ向けの修正:

1. 無効 ray（空振り）を (0,0,0) で出力していたのをスキップに変更。元実装は 1 フレーム 2 万点の
   うち 7 割が原点偽点だった。`is_dense=true` で密点群にした。
2. publisher の QoS を RELIABLE → `SensorDataQoS`（BEST_EFFORT）に変更。下流（pointcloud_to_laserscan,
   Autoware crop_box, GLIM）は sensor QoS で購読するため、RELIABLE だと QoS 不一致で受信できなかった。
3. `csv_file_name` が相対パスのとき本パッケージ share から解決するようにした（SDF に絶対パスを書かないため）。
4. `libprotobuf.so.9` / `libboost_chrono.so.1.71.0` の直リンクを開発版シンボリックリンク（`protobuf` /
   `boost_chrono`）に変更。Ubuntu 22.04 では実体が `.so.23` / `1.74` のため。
5. sensor 名を `lidar_link` にした。LCAS 版は frame_id = sensor 名のため、これで TF と整合する。

注意点:

- LCAS 版の PointCloud2 は `x,y,z,intensity,tag,line`（tag/line はダミー 0、timestamp なし）。
  自作版にあった per-point timestamp は無い。GLIM の intensity cloud は `pointcloud_intensity_node` で補完する。
- LCAS 版は static な孤立センサ（world 直書きの非ロボット link）だと `OnNewLaserScans` が発火しない。
  ロボットモデルに組み込んだ本パッケージの使い方では問題なく発火する。

### 検証結果（LCAS 版 + 上記修正）

Gazebo（`test_robot_empty.launch.py`、本番 `model.sdf`、空 world + 壁）:

- CSV が share から相対解決され spawn 成功、`Caught exception` なし。
- `/lidar/points`: frame=`lidar_link`、`is_dense=true`、原点偽点 0、仰角 [-7.1°,+50.4°]（MID-360 仕様 [-7,+52] と一致）。
- 壁が全仰角で x≈一定（平面が正しく再現、自己遮蔽の偽点なし）。
- BEST_EFFORT で受信成功（約 6.8Hz）。
- `/scan` も pointcloud_to_laserscan が生成（前方壁を中央距離 2.45m で検出）。
- TF `base_link → lidar_link` が z=0.20 で出る。

Webots（`webots_simulation.launch.py world:=indoor.wbt`）:

- 標準 `Lidar` の MID-360 近似。`tiltAngle` 未設定だと仰角中心が 0°（[-29.8°,+45.0°]）で MID-360 と
  ずれていたため、当初 `indoor/outdoor/calibration.wbt` の Lidar に `tiltAngle 0.392699`（+22.5°）を追加
  し仰角 [-7.3°,+52.2°]（MID-360 仕様 [-7,+52]）に合わせていた。20160 点（720×28）。
  - Webots の `tiltAngle` は正で上向き。最初 `-0.392699` を入れたら下（[-52.2°,+45.0°]）に向いたため符号を反転した。

- **【重要・2026-06-19】`tiltAngle` を 0 に戻した（全 wbt）**。SLAM 地図の中心に「円形の影」が出る
  問題を基礎から検証した結果、**Webots Lidar の既知バグ
  [cyberbotics/webots #37 "Wrong Lidar Point Height when Tilt Angle is Non-Zero"](https://github.com/cyberbotics/webots/issues/37)**
  （2018 報告・**未修正**）が原因と判明。`tiltAngle≠0` だと点の高さが過大に計算され、平らな地面が
  **原点中心の同心円状に地上 0.5〜2.5m へ持ち上がって** 2D 地図に焼かれる（円形の影の正体）。
  - 検証（outdoor 平地、点群を地上高さ=z_lidar+0.2 で解析）: `tiltAngle 0.39` では地上 0.5m 以上の
    点が **3979 個**（原点中心の同心円。建物・植木は別途正しく見えるので world は正常）。`tiltAngle 0`
    にすると **340 個**（建物/植木の実物体のみ）に激減し、下向きビームの地上高さが正しく ≈0、上向き
    ビームは空に抜けて消えた。
  - 副作用: `tiltAngle 0` で FOV は対称（仰角 ±29〜30° 相当）に戻り MID-360 実機の非対称 FOV [-7,+52]
    からは外れる。だが 2D SLAM/Nav は水平中心の方がむしろ素直で、円形影が消え地図品質が大幅改善する
    メリットが勝る。色付き点群（omni camera fusion）が上向き情報を使う場合は別途要検証。
  - 関連: `mode:=fast` も odom を ~21% 過大積算しドリフトさせる（地図崩れ・「RViz では進むが Webots で
    衝突」の主因）。マッピング/ナビは `mode:=realtime` を使う。/scan は生点群を高さ帯（lidar 基準
    z>=0.1=地上約0.3m、地面は z≈-0.2 に正しく乗る）で 2D 化（`webots_simulation.launch.py` の
    pointcloud_to_laserscan）。詳細は `docs/webots_simulation.md` のマッピング節も参照。

### 環境側の別問題（MID-360 とは無関係、ついでに修正）

ワークスペースに dangling な install 残骸（`susumu_object_tracker` / `susumu_gtts` / `susumu_dummy_agi`）が
あり、`AMENT_PREFIX_PATH` に残った壊れた symlink のせいで本パッケージの全 launch が起動時に
`package not found` で即クラッシュしていた。`rm -rf install/{...} build/{...}` で削除し、解消した。
