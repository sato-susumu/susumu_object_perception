# susumu_sim

ROS 2 Humble + Gazebo Classic 11 で、**HuNavSim が制御する5人の歩行者**が動く
**カフェ（cafe world）**を、**3D LiDAR 搭載 TurtleBot3** が走り回る
**シミュレーター**パッケージ。Nav2 による自律移動に加え、**手動操縦／自動巡回が
できる Teleop GUI** を備える。

> このパッケージは純粋なシミュレーターです。「人を検知して右隣を歩く」追従機能は
> 別パッケージ（`susumu_lidar_perception`）へ分離されました。

> **world について**: 既定は **cafe world**。家（house world）の素材も同梱しているが、
> house は狭い通路・家具密集により歩行者が固着しやすい（[`SETUP.md`](SETUP.md) Phase H）。
> 人がよく動き回るのは cafe。house に切り替えるには起動引数で
> `map`・`base_world`・`configuration_file` を house 用に渡す。

- 設計（全体構造・状態遷移・シーケンス図・パラメータ・ディレクトリ構成）は
  [`docs/software_design.md`](docs/software_design.md) を参照。
- Nav2 の調整（パラメータ・症状別の指針・変更履歴）は
  [`docs/nav2_tuning.md`](docs/nav2_tuning.md) を参照。
- 構築の詳細手順・ハマりどころは [`SETUP.md`](SETUP.md) を参照。

---

## できること

1. **カフェ + 5人の歩行者** — HuNavSim（Social Force Model）が5人をカフェ内に配置し、
   通常の歩行速度（`max_vel: 1.5`, `vel: 0.6`〜`0.8`）で歩き回らせる。
2. **3D LiDAR TurtleBot3** — waffle に Velodyne VLP-16 相当の16ch 3D LiDAR を搭載し、
   `/velodyne_points`（PointCloud2）を出力。
3. **Nav2 自律移動** — cafe マップ上でゴールを指定すると、3D LiDAR で人を含む障害物を
   避けながら自律走行。
4. **Teleop / 自動巡回 GUI** — 矢印ボタン（＋テンキー）でロボットを手動操縦。トグルを
   ON にすると Nav2 経由でカフェ内を順に自動巡回。スタックした時の「原点ワープ」も用意。

---

## 必要環境・依存

| 種別 | 内容 |
|---|---|
| ベース | ROS 2 Humble / Gazebo Classic 11 / Nav2 / TurtleBot3(waffle) |
| 外部クローン | HuNavSim `hunav_sim` / `hunav_gazebo_wrapper`（`v1.0-humble`）、`people_msgs`（ソース） |
| ヘッダlib | `lightsfm`（`/usr/local/include` へ `make install`） |
| Python | tkinter（GUI） |

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

### 全部入り（カフェ + 5人 + 3D LiDAR TB3 + Nav2 + RViz2 + Teleop GUI）

```bash
ros2 launch susumu_sim simulation.launch.py
```

- RViz2 の **"2D Goal Pose"** で目的地を指定 → 人を避けて自律移動。
- **Teleop GUI** ウィンドウ:
  - 矢印ボタンを「押している間」だけ走行（テンキー 8/2/4/6、矢印キーも同じ）。
  - **自動巡回** トグルを ON にすると、Nav2 でカフェ内を順番に自動巡回。
  - **原点へワープ** で、隅にハマって動けなくなったロボットを原点へ戻す。

GUI を出したくないときは `gui:=false`:

```bash
ros2 launch susumu_sim simulation.launch.py gui:=false
```

---

## launch（エントリポイント）

| ファイル | 役割 |
|---|---|
| `simulation.launch.py` | 全部入り（カフェ + 5人 + ロボット + Nav2 + RViz2 + GUI）。エントリポイント |

主な引数:

| 引数 | 既定 | 意味 |
|---|---|---|
| `use_nav2` | True | Nav2 スタックを起動する |
| `use_rviz` | True | RViz2 を起動する |
| `gui` | True | Teleop / 自動巡回 GUI を起動する |
| `map` | `maps/cafe.yaml` | マップ yaml のフルパス（house に戻すなら `maps/house.yaml`） |
| `x_pose` / `y_pose` / `yaw` | 0.0 / 0.0 / 0.0 | ロボットの spawn 姿勢 |

> `launch/include/` 配下には `simulation.launch.py` が内部で取り込む部品 launch
> （家＋人、ロボット spawn、検証用の空ワールド）があります。起動順序や各部品の
> 構成は [`docs/software_design.md`](docs/software_design.md#2-launch-構成と起動順序) を参照。

---

## ライセンス

MIT License（[`LICENSE`](LICENSE)）。TurtleBot3 モデルは ROBOTIS、HuNavSim は
robotics-upo に帰属。
