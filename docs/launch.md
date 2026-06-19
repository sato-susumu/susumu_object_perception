# launch（エントリポイント）一覧と引数

各 launch が何を起動するか、および引数の一覧。概要・機能は [`../README.md`](../README.md) を参照。

## 何が起動するか一覧

✅=既定で起動 / ○=引数で起動可 / —=起動しない。Sim 列は使うシミュレータ。

| launch | Sim | robot | Nav2 | SLAM | RViz | GUI | perception | 備考 |
|---|---|---|---|---|---|---|---|---|
| `simulation.launch.py` | Gazebo | ✅ | ✅ | — | ✅ | ✅ | ✅ | カフェ+5人歩行者。全部入りエントリ |
| `webots_simulation.launch.py` | Webots | ✅ | ✅ | ○ | ✅ | — | ✅ | `world:=outdoor.wbt`/`indoor.wbt` 指定 |
| `webots_outdoor.launch.py` | Webots | ✅ | ✅ | ○ | ✅ | — | ✅ | world=outdoor 固定ショートカット |
| `webots_indoor.launch.py` | Webots | ✅ | ✅ | ○ | ✅ | — | ✅ | world=indoor 固定ショートカット |
| `webots_nav.launch.py` | Webots | ✅ | ✅ | ✅ | ✅ | — | ✅ | robot+Nav2+SLAM フルスタック（自律走行可） |
| `webots_city_mapping.launch.py` | Webots | ✅ | ✅ | ✅ | ○ | — | ○ | **frontier 探索で事前地図なし環境を自律マッピング**。`world:=<wbt> map_name:=<name>`。完了時 `maps/` に自動保存 |
| `webots_waypoint_nav.launch.py` | Webots | ✅ | ✅ | ✅ | ✅ | — | ○ | **保存ウェイポイントを Nav2 で巡回**。`world:=<wbt> waypoints:=<world>_waypoints.yaml`。`perception:=True` で巡回中の物体認識も |
| `webots_slam.launch.py` | — | — | — | ✅ | — | — | — | slam_toolbox を1個だけ起動する補助 |
| `webots_city.launch.py` | Webots | ✅ | ✅ | — | ✅ | — | ✅ | **既定 `ros2:=True`: city にセンサ付き TB3 を置き ROS2 認識（LiDAR + 全天球 + YOLO 物体分類 + 信号認識）。`ros2:=False` で SUMO 車100台の眺めるデモ**※ |

※ `webots_city ros2:=False` は SUMO 制御の車を眺めるだけで ROS2 連携しない（`/scan` 等は出ない）。
既定の `ros2:=True` は `city_robot.wbt`（車 BmwX5 + 歩行者 Pedestrian + 信号 + センサ付き TB3）を
起動し、`/cmd_vel` で対象に近づくと car/person/信号を認識する（遠方は全天球で小さく映り苦手）。

> **Webots のセンサ構成**: indoor/outdoor.wbt は **3D LiDAR（MID-360 近似）+ RGB カメラ + 全天球カメラ**を搭載
> （2D LiDAR LDS-01 は廃止）。
> - 3D LiDAR → `/lidar/points/point_cloud`(PointCloud2, frame `lidar_link`)
> - カメラ → `/camera/image_raw/image_color`(Image, 1920×1080, Intel RealSense R200 相当)
> - `/scan` は `pointcloud_to_laserscan` が 3D 点群から生成（2D LiDAR の代替、Nav2/AMCL 用）
>
> **Webots の perception**: 上記 3D LiDAR を入力に Gazebo と同じ Autoware perception パイプライン
> （検出・追跡・予測・可視化）が既定 `perception:=True` で動く。RViz2 も既定 `rviz:=True`。
> 見るだけにしたいときは `perception:=False rviz:=False` を付ける。
>
> **Webots の nav/SLAM の住み分け**: `webots_simulation`/`outdoor`/`indoor` は `nav` 既定 `True`
> で Nav2(AMCL ベース)が立つが、自律走行には初期位置指定が要る。SLAM で地図を作りながら
> 完全自走したいときは `webots_nav.launch.py`（slam_toolbox 同梱）を使う。
> Webots を見るだけなら `nav:=False` を付ける。詳細は [`webots_simulation.md`](webots_simulation.md)。

## 自律マッピングとウェイポイント巡回

事前地図のない Webots 環境を frontier 探索で自律マッピングし、地図からウェイポイントを生成して
巡回する一連のフロー（詳細は [`mapping_and_patrol.md`](mapping_and_patrol.md) があれば参照）:

```bash
# 1) 事前地図なしの環境を frontier 探索で自律マッピング（完了時 maps/<name> に自動保存）
ros2 launch susumu_object_perception webots_city_mapping.launch.py world:=city_robot.wbt map_name:=city mode:=fast

# 2) 保存地図から巡回ウェイポイントを生成（壁から離れた自由空間を巡回順に）
ros2 run susumu_object_perception generate_waypoints.py \
  --map maps/city.yaml --out maps/city_waypoints.yaml --spacing 1.5 --clearance 0.4

# 3) ウェイポイントに沿って Nav2 で巡回（perception:=True で巡回中の物体認識も）
ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
  world:=city_robot.wbt waypoints:=city_waypoints.yaml perception:=True omni_perception:=True
```

> **注**: 連続したクリーン再起動で FastRTPS の共有メモリトランスポートが壊れ `/scan` が出なく
> なることがある（`open_and_lock_file failed` が多発し SLAM が地図を作れない）。その場合は SHM を
> 無効化した FastRTPS プロファイル（UDP 強制）を `FASTRTPS_DEFAULT_PROFILES_FILE` で指定して起動する。

## Webots 系 launch の引数

| 引数 | 既定 | 対象 | 意味 |
|---|---|---|---|
| `world` | `outdoor.wbt` | webots_simulation | `webots_worlds/` の world ファイル名（拡張子込み） |
| `lidar_model` | `mid360` | webots_simulation/outdoor/indoor/nav/calibration/SLAM | LiDAR profile。`mid360` または `vlp16` |
| `nav` | `True` | simulation/outdoor/indoor | Nav2 を起動（大文字必須。小文字 `true` は NameError） |
| `slam` | `False` | simulation/outdoor/indoor | SLAM(slam_toolbox)を起動（大文字必須） |
| `perception` | `True` | simulation/outdoor/indoor | Autoware perception を起動（3D LiDAR `/lidar/points/point_cloud` 入力） |
| `omni_perception` | `True` | webots_nav 等 | 全天球色付き点群 + YOLO 物体分類 + 信号認識 |
| `rviz` | `True` | simulation/outdoor/indoor | RViz2 を起動 |
| `mode` | `realtime` | webots 全般 | Webots 起動モード（realtime / fast / pause） |
| `nav_params_file` | （空） | webots_simulation/nav | Nav2 params 差し替え（探索は `config/nav2_params_webots_explore.yaml`） |

## `simulation.launch.py`（Gazebo）の主な引数

| 引数 | 既定 | 意味 |
|---|---|---|
| `use_nav2` | True | Nav2 スタックを起動する |
| `use_perception` | True | Autoware 流 perception パイプライン（LiDAR 検出・追跡・予測）を起動する |
| `image_recognition` | True | 画像認識（6面カメラ→全天球合成 + LiDAR 検出物体の YOLO 分類 + 全天球信号認識）を起動する。YOLO が重ければ False |
| `use_rviz` | True | RViz2 を起動する |
| `gui` | True | Teleop / 自動巡回 GUI を起動する |
| `lidar_model` | `mid360` | 3D LiDAR profile。`mid360`（標準）または `vlp16` |
| `map` | `maps/cafe.yaml` | マップ yaml のフルパス（house に戻すなら `maps/house.yaml`） |
| `params_file` | `config/nav2_params.yaml` | Nav2 パラメータ yaml のフルパス |
| `x_pose` / `y_pose` / `yaw` | 0.0 / 0.0 / 0.0 | ロボットの spawn 姿勢 |

> 起動順序や各部品の構成は
> [`software_design.md`](software_design.md#2-launch-構成と起動順序) を参照。
