# susumu_object_perception

ROS 2 Humble + Gazebo Classic 11 で、**HuNavSim が制御する5人の歩行者**が動く
**カフェ（cafe world）**を、**3D LiDAR 搭載 TurtleBot3** が走り回る
**シミュレーター**パッケージ。Nav2 による自律移動に加え、**手動操縦／自動巡回が
できる Teleop GUI**、および **Autoware 流の LiDAR perception パイプライン**（既定 ON）を
備える。perception は Autoware の構成に沿って、3D LiDAR 点群から
**検出 → 形状推定(OBB) → 過分割統合 → 2D 地図 ROI → 追跡 + 分類 → 将来軌跡予測**
までを行い、RViz に可視化する。検出までは Autoware 純正モジュール、apt に無い段や HD 地図
依存の段は **2D 占有格子地図と Autoware アルゴリズムの踏襲で自作補完**している。

> 「人を検知して右隣を歩く」追従機能は持ちません（旧 `susumu_lidar_perception` へ分離）。
> perception は **可視化が主**ですが、**prediction の予測のみ Nav2 costmap に連携**します:
> 人の現在位置 + 進路先を予測 OccupancyGrid にして、自作 C++ costmap 層
> `susumu_object_perception::PredictedCostmapLayer` が max 合成で焼きます（人の「これから行く先」を先回りで
> 障害物化）。Nav2 の現在位置回避は 2D `/scan` も担います。詳細は
> [`docs/autoware_perception.md`](docs/autoware_perception.md) /
> [`docs/nav2_tuning.md`](docs/nav2_tuning.md)。

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
5. **Autoware 流 perception パイプライン**（既定 ON）— 3D LiDAR から
   検出（crop_box→ground_filter→euclidean_cluster, 純正）→ 形状推定(L字フィット OBB)
   → 過分割統合(Cluster Merger) → 2D 地図 ROI → 追跡 + 2D 分類(歩行者推定)
   → マルチモーダル将来軌跡予測、を行い RViz に可視化。HD 地図の代わりに **2D 占有格子
   地図**を、apt に無い段は **Autoware アルゴリズムの踏襲で自作**して補完
   （[`docs/autoware_perception.md`](docs/autoware_perception.md)）。
6. **予測コストマップ連携** — prediction が人の現在位置 + 進路先を予測 OccupancyGrid にして
   出し、自作 C++ costmap 層 `PredictedCostmapLayer` が max 合成で Nav2 costmap に焼く。
   毎フレーム作り直すので移動軌跡が残らない（[`docs/nav2_tuning.md`](docs/nav2_tuning.md)）。

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
colcon build --symlink-install        # または --packages-select susumu_object_perception hunav_* people_msgs

# ★ source は setup.bash ではなく local_setup.bash を使うこと（理由はSETUP.md参照）
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
export TURTLEBOT3_MODEL=waffle
```

---

## 実行

### 全部入り（カフェ + 5人 + 3D LiDAR TB3 + Nav2 + RViz2 + Teleop GUI）

```bash
ros2 launch susumu_object_perception simulation.launch.py
```

- RViz2 の **"2D Goal Pose"** で目的地を指定 → 人を避けて自律移動。
- **Teleop GUI** ウィンドウ:
  - 矢印ボタンを「押している間」だけ走行（テンキー 8/2/4/6、矢印キーも同じ）。
  - **自動巡回** トグルを ON にすると、Nav2 でカフェ内を順番に自動巡回。
  - **原点へワープ** で、隅にハマって動けなくなったロボットを原点へ戻す。

GUI を出したくないときは `gui:=false`:

```bash
ros2 launch susumu_object_perception simulation.launch.py gui:=false
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
