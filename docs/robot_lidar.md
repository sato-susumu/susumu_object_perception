# ロボット / LiDAR 構成と制約

このページは README から分離したロボット・センサ構成の詳細。トピック、frame、LiDAR model を変更する場合は
供給側と利用側を同時に更新する。

## 標準構成

| 環境 | ロボット | LiDAR | カメラ | 備考 |
|---|---|---|---|---|
| Gazebo Classic | TurtleBot3 Waffle 拡張 SDF | MID-360 相当 3D LiDAR | 6 面カメラ合成の全天球 | `simulation.launch.py` |
| Webots | TurtleBot3 Burger 系 URDF 拡張 | Webots `Lidar` による MID-360 近似 | Webots cylindrical 全天球カメラ + RGB カメラ | `webots_simulation.launch.py` |

2D LiDAR は標準構成から外している。Nav2/AMCL 用 `/scan` は 3D 点群から
`pointcloud_to_laserscan` で生成する。

## トピック / frame 契約

| 役割 | 値 | 供給側 | 利用側 |
|---|---|---|---|
| 速度司令 | `/cmd_vel` | Nav2 controller / teleop GUI | SDF diff_drive / Webots diffdrive |
| オドメトリ | topic/frame `odom` | simulator diffdrive | Nav2 / SLAM / AMCL |
| ベース | `base_link` / `base_footprint` | URDF/SDF/TF | Nav2 costmap / AMCL |
| 3D LiDAR frame | `lidar_link` | URDF/SDF/Webots TF | perception / pointcloud_to_laserscan / colorization |
| Gazebo 3D 点群 | `/lidar/points` | `liblivox_mid360_sensor.so` | perception / `/scan` 生成 |
| Webots 3D 点群 | `/lidar/points/point_cloud` | `webots_ros2_driver` | perception / `/scan` 生成 / colorization |
| 2D scan | `/scan`, frame `lidar_link` | pointcloud_to_laserscan | Nav2 obstacle_layer / AMCL / SLAM |
| 全天球画像 | `/omni_camera/image_raw/image_color` | Webots cylindrical camera / Gazebo 6面合成 | 画像分類 / 信号認識 / 色付き点群 |
| 色付き点群 | `/perception/colorized_points` | `colorized_pointcloud_node.py` | RViz / colorized mapper |

旧 Velodyne 前提の `/velodyne_points` / `velodyne_link` は使わない。

## Gazebo Classic の LiDAR

標準 `lidar_model:=mid360` は `models/turtlebot3_waffle_3d/model.sdf` の `ray` センサと
`liblivox_mid360_sensor.so` を使う。LCAS/livox_laser_simulation_ros2 由来の ODE MultiRayShape 方式で、
`config/mid360_scan_patterns/mid360.csv` の scan pattern に沿って ray を撃つ。

出力 PointCloud2:

- topic: `/lidar/points`
- frame: `lidar_link`
- fields: `x`, `y`, `z`, `intensity`, `tag`, `line`
- per-point timestamp は出さない
- `tag` / `line` はダミー 0

VLP-16 退避版は `models/turtlebot3_waffle_vlp16/` と `urdf/turtlebot3_waffle_vlp16.urdf.xacro` に残してあり、
`lidar_model:=vlp16` で選ぶ。

SDF/URDF 変更後は最低限これを通す:

```bash
gz sdf -k models/turtlebot3_waffle_3d/model.sdf
xacro urdf/turtlebot3_waffle_3d.urdf.xacro > /dev/null
```

## Webots の LiDAR

Webots の標準 world は Webots `Lidar` による MID-360 近似。Webots 標準 Lidar では Livox/MID-360 の
非反復 scan pattern を直接指定できないため、FOV・レンジ・点密度の近似に留めている。

- device 名: `lidar3d`
- frame: `lidar_link`
- raw topic: `/lidar/points/point_cloud`
- Nav2/AMCL 用 scan: `/scan`
- `tiltAngle` は 0 固定

`/scan` 生成の高さ帯は `webots_simulation.launch.py` で定義する。地面を落とし、壁・家具・人を拾うため、
`lidar_link` 基準で `min_height 0.1`、`max_height 2.0`、`range_min 0.3` を使う。
SLAM 用の未ヒット ray は `range_max 16.0`、`use_inf false`、`inf_epsilon -0.5` で 15.5m の有限値にする。
`slam_toolbox` の `max_laser_range 15.0` と合わせ、占有端点を置かず 15m まで自由空間として raytrace
させるため。広い world で `+inf` のままにすると、空に抜けた方位が地図に反映されず原点周辺の星形に
留まりやすい。

## 制約

- 独自メッセージ型は追加しない。点群は `sensor_msgs/PointCloud2`、scan は `LaserScan`、認識は
  Autoware 型や標準型を使う。
- Nav2 costmap に 3D 点群を STVL で焼かない。現在位置の障害物回避は `/scan`、人の進路先は
  prediction costmap layer が担う。
- downstream の perception、色付き点群、GLIM 設定は汎用 topic/frame に寄せる。
- LiDAR topic/frame を変える場合は、launch、perception、pointcloud_to_laserscan、colorized point cloud、
  GLIM config、RViz を同時に確認する。

## 関連

- [MID-360 LiDAR 調査](mid360_lidar_research.md)
- [認識パイプライン](autoware_perception.md)
- [カラー点群出力タスク](tasks/colorized_pointcloud.md)
- [ノード接続図](node_topology.md)
