# Webots シミュレーション環境ガイド（ROS2 Humble）

屋外・屋内の大型シミュレーションを Webots で動かすための手引き。**実際に動かして確認した手順**を
まとめている（車・信号・歩行者が動く街、屋内外切替、ROS2 連携、Nav2 自律ナビまで）。

> なぜ Webots か: つくばチャレンジ級の「まちなか走行」を Gazebo 以外も含めて調査した結果
> （[[tsukuba-challenge-research]] 参照）、**車・信号が動く + 人が動く + 屋内外切替 + ROS2 公式対応**を
> 最もバランスよく満たすのが Webots だった。Apache-2.0、CPU でも動く、`city`/`village`/`apartment` 等の
> world が標準同梱、SUMO 連携で車が信号を守って自律走行する。

---

## 0. 結論（最初に読む）

実際に動かせたもの:
- **屋外の街 `city_traffic.wbt`**: SUMO 連携で車が交通シミュ走行（最大100台）、信号機・歩行者が動く
- **屋内 `apartment.wbt`**: e-puck / iRobot Create が障害物回避で走行
- **ROS2 連携**: TurtleBot3(LiDAR) が `/scan` `/odom` `/cmd_vel` を出力、`/cmd_vel` 送信でロボットが動く
- **Nav2 自律ナビ**: SLAM(slam_toolbox)で地図を作りながら、ゴール送信 → `SUCCEEDED` で完走
- **屋内外切替**: 同じ TurtleBot3 を `world:=outdoor.wbt` / `world:=indoor.wbt` で切り替え、どちらも ROS2 連携

最重要のハマりどころ（後述）:
1. **`nav:=true`(小文字)は launch がクラッシュ → `nav:=True`(大文字)が必須**
2. **`slam:=True` は SLAM が二重起動して TF が壊れる → slam は別プロセスで1個だけ起動**
3. **`GAZEBO_RESOURCE_PATH` ならぬ Webots でも環境変数の上書きに注意**（SUMO_HOME 等）

---

## 1. セットアップ

### 1-1. webots_ros2（ROS2 連携）+ SUMO（交通シミュ）を apt 導入

```bash
sudo apt-get update
sudo apt-get install -y ros-humble-webots-ros2 sumo sumo-tools
```

- `ros-humble-webots-ros2`: ROS2 と Webots の連携パッケージ（turtlebot/tesla 等のデモ込み）
- `sumo` / `sumo-tools`: 都市交通シミュレータ。**Webots の街で車を信号通り走らせるのに必要**

### 1-2. Webots 本体を導入

`webots_ros2` だけでは Webots 本体は入らない（初回起動時に自動DLされる方式だが、明示導入が確実）。
公式 .deb を入れる:

```bash
cd /tmp
wget https://github.com/cyberbotics/webots/releases/download/R2025a/webots_2025a_amd64.deb
sudo apt-get install -y /tmp/webots_2025a_amd64.deb
which webots   # /usr/local/bin/webots が出れば OK
```

同梱の world は `/usr/local/webots/projects/` 配下にある。

### 1-3. 環境変数（毎回 source / export する）

```bash
source /opt/ros/humble/setup.bash
export DISPLAY=:0            # GUI 表示先（ヘッドレスでも Webots は X を要求する）
export SUMO_HOME=/usr/share/sumo   # ← これが無いと city_traffic で「SUMO not found」になる
```

---

## 2. 同梱 world の場所（よく使うもの）

| 用途 | world ファイル |
|---|---|
| 屋外・街（車SUMO走行+信号+歩行者） | `/usr/local/webots/projects/vehicles/worlds/city_traffic.wbt` |
| 屋外・街（他バリエーション） | `city.wbt` / `village.wbt` / `village_realistic.wbt` / `highway.wbt`（同 vehicles/worlds） |
| 屋内（ロボット稼働） | `/usr/local/webots/projects/samples/environments/indoor/worlds/apartment.wbt` |
| 歩行者（社会力で歩く） | `/usr/local/webots/projects/humans/pedestrian/worlds/pedestrian.wbt` |
| 3D LiDAR(Velodyne) PROTO | `/usr/local/webots/projects/devices/velodyne/`（3D LiDAR 化に使える） |

---

## 3. 起動手順

### 3-1. 単体で world を見る（GUI）

```bash
export DISPLAY=:0 SUMO_HOME=/usr/share/sumo
# GUI で開く（gazebo と違い webots コマンド1つで server+client）
webots /usr/local/webots/projects/vehicles/worlds/city_traffic.wbt
```

### 3-2. ヘッドレス／高速で走らせる（検証向け）

```bash
export DISPLAY=:0 SUMO_HOME=/usr/share/sumo
webots --batch --mode=fast --no-rendering --stdout --stderr \
  /usr/local/webots/projects/vehicles/worlds/city_traffic.wbt
```

- `--batch`: 終了ダイアログ等を抑制
- `--mode=fast`: 可能な限り高速にシミュレーション
- `--no-rendering`: 画面描画を省く（**ただし屋内 apartment 等は描画前提で落ちることがある。その場合は外す**）

city_traffic 起動後、ログに以下が出れば成功:
```
Using SUMO from /usr/share/sumo
Connect to SUMO...
```
→ SUMO プロセス（`/usr/share/sumo/bin/sumo`）が立ち、車が信号を守って自律走行する。

### 3-3. ROS2 連携で起動（LiDAR ロボット + /scan 等）

```bash
source /opt/ros/humble/setup.bash
export DISPLAY=:0 SUMO_HOME=/usr/share/sumo
ros2 launch webots_ros2_turtlebot robot_launch.py rviz:=false
```

出てくる ROS2 トピック: `/scan`(2D LiDAR) `/scan/point_cloud` `/odom` `/cmd_vel` `/imu` `/tf`。

**動作確認（ロボットを ROS2 から動かす）**:
```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.15}, angular: {z: 0.3}}" -r 10
# 別端末で odom を見ると位置が変化する → ROS2 制御成功
ros2 topic echo /odom --once
```

---

## 4. Nav2 自律ナビ（SLAM で地図を作りながら）

webots_ros2_turtlebot は `nav`/`slam` 引数で Nav2/SLAM を起動できるが、**罠があるので推奨手順を示す**。

### 推奨手順（TF 競合を避ける確実版）

```bash
# 端末1: TurtleBot3 + Nav2（SLAM はここでは起動しない）
source /opt/ros/humble/setup.bash
export DISPLAY=:0 SUMO_HOME=/usr/share/sumo
ros2 launch webots_ros2_turtlebot robot_launch.py nav:=True slam:=False rviz:=false
#  ↑ nav:=True は大文字！ slam:=False で同梱SLAMの二重起動を防ぐ

# 端末2: slam_toolbox を「1個だけ」起動（map→odom TF を供給）
source /opt/ros/humble/setup.bash
ros2 launch slam_toolbox online_async_launch.py use_sim_time:=true

# 端末3: ゴールを送って自律ナビ
source /opt/ros/humble/setup.bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 0.8, y: 0.0}, orientation: {w: 1.0}}}}" --feedback
```

成功すると:
```
Goal accepted with ID: ...
distance_remaining: 0.95 → 0.27 → ...   （残距離が減っていく）
Goal finished with status: SUCCEEDED    （ゴール到達）
```
`/odom` の位置がゴール方向に移動していれば、Nav2 が実際にロボットを自律走行させている。

> 補足: ここでは確実な 2D LiDAR(LDS-01) で Nav2 を完走させた。3D LiDAR 化したい場合は Webots の
> Velodyne PROTO（`projects/devices/velodyne/`）をロボットの extensionSlot に足し、
> pointcloud_to_laserscan で /scan を作るか、3D 対応の costmap 層を使う。

---

## 5. 屋内外切替 × ROS2 連携

### 仕組み（なぜ切替できるか）

`webots_ros2_turtlebot/robot_launch.py` は **`world` という launch 引数**を持ち、
`webots_ros2_turtlebot/worlds/<world>` を読み込む。ロボット（TurtleBot3）の ROS2 連携
（`controller "<extern>"` による外部制御 + LiDAR/odom の publish）は **world に依存しない**ので、
**ロボットを置いた world ファイルを差し替えるだけで、同じロボット・同じ ROS2 トピック構成のまま
屋外↔屋内を切り替えられる**。これが Webots での「屋内外切替」の最も素直な方法。

> 「屋外と屋内が地続きで繋がった1つの world で、建物に入ると屋内になる」レベルは Webots でも
> Gazebo/Isaac でも難しく、**world を分けて切り替える**のが現実解（後述の事例でも実機は
> マルチセンサ融合で対応し、シミュは環境を分けるのが一般的）。

### 用意した world（`susumu_object_perception/webots_worlds/` に保存、`webots_ros2_turtlebot/worlds/` に配置）

- `indoor.wbt`: 同梱デモ（`turtlebot3_burger_example.wbt`）のコピー = 壁・窓・家具のある屋内
- `outdoor.wbt`: TurtleBot3 + 地面20×20 + 木4本 + 建物2棟（屋外）。屋外なので `/gps` も出る

> world は EXTERNPROTO を `https://raw.githubusercontent.com/cyberbotics/webots/...` から
> オンライン取得する。初回はネット接続が要る（取得後はキャッシュされる）。

### 配置（自作 world を webots_ros2_turtlebot に置く）

```bash
WDIR=/opt/ros/humble/share/webots_ros2_turtlebot/worlds
sudo cp ~/ros2_ws/src/susumu_object_perception/webots_worlds/indoor.wbt  $WDIR/
sudo cp ~/ros2_ws/src/susumu_object_perception/webots_worlds/outdoor.wbt $WDIR/
```

### 切替コマンド

```bash
source /opt/ros/humble/setup.bash; export DISPLAY=:0

# 屋外（/scan /odom /cmd_vel /gps が出る）
ros2 launch webots_ros2_turtlebot robot_launch.py world:=outdoor.wbt rviz:=false

# 屋内（/scan /odom /cmd_vel が出る）
ros2 launch webots_ros2_turtlebot robot_launch.py world:=indoor.wbt rviz:=false

# 同梱の街 world に置き換えたい場合（要: TurtleBot3 を含む world を用意）
# city/village は車用 world なので、そのままでは TurtleBot3 の ROS2 連携は付かない。
# outdoor.wbt を雛形に、SimpleBuilding/Road 等を足して街化するのが早い。
```

実証済み（両 world で `Controller successfully connected` + 下記トピック）:

| world | /scan | /odom | /cmd_vel | /gps |
|---|---|---|---|---|
| `outdoor.wbt` | ✅ | ✅ | ✅ | ✅ |
| `indoor.wbt` | ✅ | ✅ | ✅ | — |

どちらも `nav:=True` を足せば各 world で Nav2 自律ナビも可能（§4 の推奨手順を参照）。

### 切替を1コマンド化したいなら

`world` を環境変数や引数で受けるラッパー launch / シェルを書けば
`./run.sh outdoor` のように切り替えられる。最小例:

```bash
# run_webots.sh
WORLD=${1:-outdoor}.wbt   # 引数: outdoor / indoor
source /opt/ros/humble/setup.bash; export DISPLAY=:0 SUMO_HOME=/usr/share/sumo
ros2 launch webots_ros2_turtlebot robot_launch.py world:=$WORLD rviz:=false
```

---

## 6. ハマりどころ（実際に踏んだ罠）

| 症状 | 原因 | 対処 |
|---|---|---|
| `NameError: name 'true' is not defined` で launch がクラッシュ、全プロセスが連鎖シャットダウン | **`nav:=true` の小文字 true が Python 名として評価される** | **`nav:=True` `slam:=True`（大文字）で渡す** |
| Nav2 起動後 `TF_OLD_DATA ignoring data from the past for frame odom` が大量、ロボットが動かない | **`slam:=True` が cartographer と slam_toolbox を二重起動して map→odom が競合** | **`slam:=False` にして slam_toolbox を別プロセスで1個だけ起動** |
| `SUMO not found. Please install it...` | SUMO は入れたが `SUMO_HOME` 未設定 | **`export SUMO_HOME=/usr/share/sumo`** |
| 屋内 world（apartment）が `--no-rendering` で即死 | 屋内 world は描画前提 | `--no-rendering` を外して起動（`--minimize` で最小化はOK） |
| world が起動時に固まる/即死 | EXTERNPROTO のオンライン取得待ち（37個等） | ネット接続を確認。github raw が遅い時は待つ。取得後はキャッシュで速い |
| `process has died, exit code -2/-6` が大量 | たいてい上記 1 か 2 の**二次的なシャットダウン連鎖**。最初の ERROR を見る | ログを上から見て**最初の ERROR**（`is not defined` 等）を特定する |
| ROS2 トピックが `ros2 topic list` で出ない | DDS ディスカバリ/daemon の遅延 | `ros2 daemon stop && ros2 daemon start` してから再取得 |

> ヘッドレス環境では Webots も X を要求するため `DISPLAY=:0` が要る（X サーバが居る前提）。
> GUI を見たいときは `--no-rendering`/`--minimize` を外して起動する。

---

## 7. プロセスの停止

Webots + ROS2 launch は子プロセスが孤児化しやすい。確実に止めるには:

```bash
pkill -9 -f "ros2 launch webots"   # 親の launch を先に
pkill -9 -f webots-bin             # Webots 本体
pkill -9 -f ros2_supervisor
pkill -9 -f sync_slam_toolbox; pkill -9 -f async_slam_toolbox
# 残骸が居たら PID 直接: ps -eo pid,args | grep webots-bin → kill -9 <pid>
```

> `pkill` はマッチが無いと終了コード 1 を返す。**シェルスクリプトで `pkill` を `&&`/`;` で連鎖すると
> そこで止まって後続（肝心の起動コマンド）に到達しないことがある**。`pkill ... || true` で吸収するか、
> 停止と起動を別コマンドに分ける。

---

## 8. 似た事例・参考記事

我々の取り組み（Webots + ROS2 + slam_toolbox + Nav2、屋内外切替）に近い事例。

### Webots + ROS2 + Nav2/SLAM の実践（最も近い）
- **Webots 公式 wiki「Navigate TurtleBot3」**（cyberbotics/webots_ros2）—
  `robot_launch.py nav:=true` で Webots+RViz+Nav2 を起動し、RViz の「Navigation2 Goal」で
  ゴール指定。本ガイド §4 とほぼ同じ。https://github.com/cyberbotics/webots_ros2/wiki/Navigate-TurtleBot3
- **Husarion「Webots: ROSbot 2R + SLAM Toolbox」** — Webots で ROSbot を走らせ slam_toolbox で
  地図生成。Docker Compose 付きで再現性が高い。**我々の構成と一番近い実践例**。
  https://husarion.com/tutorials/vulcanexus/webots-rosbot/
- **The Robotics Back-End「ROS2 Nav2 - Generate a Map with slam_toolbox」** — slam_toolbox で
  地図を作る手順の丁寧な解説。https://roboticsbackend.com/ros2-nav2-generate-a-map-with-slam_toolbox/

### Webots + ROS2（自作ロボット/world）— 日本語
- **Zenn「WebotsとROS 2で自作モデルを動かす」**（tasada038）— urdf2webots で URDF→PROTO 変換、
  boundingObject 調整、.wbt 配置。自作ロボットを Webots に持ち込む手順。
  https://zenn.dev/tasada038/articles/d84f74b808cf7f
- **「ROS2とWebotsの連携 調査編/実装編」**（odome.hatenablog.com）— ヒューマノイド動歩行を例に
  webots_ros2 連携を調査・実装。https://odome.hatenablog.com/entry/2022/09/26/233921
- **demura.net「Webotsシミュレータでルンバを動かそう」** — iRobot Create2 の ROS2 制御例。
  https://demura.net/robot/ros2/20567.html

### 屋内外を跨ぐナビゲーション（研究・実機寄り）
- **IndoorSim-to-OutdoorReal**（arXiv:2305.01098）— **屋内シミュだけで学習した視覚ナビを、
  屋外実機(Spot)で数百m zero-shot 走行**させた研究。屋外の勾配・歩道はシミュせず、衛星画像や
  ラフスケッチの「context-map」で進路ヒントを与える。→ **「屋内外を1つの連続 world で繋ぐのは
  難しいので、シミュは環境を分け、屋外特有部分は別手段で補う」という我々の判断と同じ方向**。
  https://arxiv.org/abs/2305.01098
- **VAULT: Mobile Mapping System for ROS2**（arXiv:2506.09583）— 複数センサ融合で屋内外とも
  ロバストに自己位置推定する ROS2 システム。実機の屋内外横断はマルチセンサ融合で対応する例。
- **JKU-ITS Last Mile Delivery Robot**（arXiv:2305.18276）— 3D LiDAR+RGB-D+IMU+GPS で屋外配送。
  実機は GPS+LiDAR SLAM 併用（屋内外で自己位置手段を切替）。

> **要点**: 「屋外も屋内も地続きの1 world で、建物に入ると屋内」レベルはどのシミュでも難しく、
> 研究・実機でも **シミュは環境を分ける / 自己位置は屋内外で手段を切り替える（LiDAR↔GPS）** のが
> 定石。我々の Webots `world:=` 切替はこの定石に沿った素直な実装。

## 9. 関連（本リポジトリ）

- つくばチャレンジ級シミュの調査・他候補（CARLA/Isaac/Habitat/Arena 等）: memory `tsukuba-challenge-research`
- Webots セットアップの要点 memory: `webots-ros2-nav2-setup`
- 本リポジトリ本体（Gazebo Classic + Autoware perception）: [`software_design.md`](software_design.md)
