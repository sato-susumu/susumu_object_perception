# Nav2 調整ガイド — susumu_object_perception

`config/nav2_params.yaml` の調整に関する設計意図・現在値・調整の指針・変更履歴をまとめる。

> ## ⚠️ 運用ルール（最重要）
>
> **`config/nav2_params.yaml` を変更したら、必ず本ドキュメントを更新してから完了とする。**
>
> - [§2 現在値](#2-現在値要点) の表に新しい値を反映する。
> - [§5 調整履歴](#5-調整履歴) に「日付 / 変更 / 理由・結果」を1行追記する。
>
> 値だけ変えてここを放置すると、なぜその値にしたのかが失われ、次の調整で振り出しに戻る。

関連: 全体設計は [`software_design.md`](software_design.md)、構築履歴は
[`../SETUP.md`](../SETUP.md)。

---

## 1. 構成の前提

| 項目 | 値 | 備考 |
|---|---|---|
| Nav2 バージョン | 1.1.20（Humble 同梱） | プラグイン名は `/` 形式（`::` 形式の新形式は不可） |
| ベース params | `nav2_bringup/params/nav2_params.yaml` | TurtleBot3 waffle 向けに調整 |
| ローカライズ | AMCL（`slam:=False`） | 生 `/scan` を使用 |
| プランナ | `nav2_smac_planner/SmacPlanner2D` | 2D costmap 上のA*系グリッドプランナ。Navfn の再計画失敗を避けるため採用 |
| コントローラ | `dwb_core::DWBLocalPlanner` | DWB ローカルプランナ |
| ロボット | TurtleBot3 waffle | 最大 0.26 m/s / 1.82 rad/s |

### コストマップ層の構成（入力 → 層 → costmap → 制御）

各 costmap（local / global）は次の層を重ねて合成する。生センサ `/scan` は障害物層、
perception の予測 OccupancyGrid は自作 `predicted_layer` が受け持つ。STVL 層は廃止済み
（[§2.1 層の遍歴](#21-3d-障害物層の遍歴1表に集約) 参照）。

```mermaid
flowchart LR
    SCAN["/scan"]:::hd
    PRED["/perception/predicted_costmap"]:::hd
    MAP["static map"]:::hd
    STVL["STVL層<br/>(廃止)"]:::skip

    SL["static_layer"]:::ext
    OL["obstacle_layer<br/>(local only)"]:::ext
    PL["predicted_layer<br/>(自作C++<br/>max合成)"]:::own
    IL["inflation_layer"]:::ext

    CM["local/global<br/>costmap"]:::ext
    DWB["DWB<br/>controller"]:::ext
    CMD["/cmd_vel"]:::ext

    MAP --> SL
    SCAN --> OL
    PRED --> PL
    STVL -.-> CM

    SL --> CM
    OL --> CM
    PL --> CM
    CM --> IL --> CM
    CM --> DWB --> CMD

    classDef aw fill:#2e7d32,stroke:#1b5e20,color:#fff;
    classDef own fill:#e65100,stroke:#bf360c,color:#fff;
    classDef hd fill:#1565c0,stroke:#0d47a1,color:#fff;
    classDef skip fill:#757575,stroke:#424242,color:#fff;
    classDef ext fill:#455a64,stroke:#263238,color:#fff;
```

---

## 2. 現在値（要点）

`config/nav2_params.yaml` の調整対象になりやすいパラメータ。**変更時はこの表も更新する。**

### AMCL（保存地図 `slam:=False` の自己位置推定）

| パラメータ | 現在値 | 意味 / 調整の効果 |
|---|---|---|
| `laser_model_type` | `likelihood_field` | 地図の likelihood field で `/scan` 観測を評価する |
| `max_beams` | 90 | 1 scan から AMCL 更新に使う最大ビーム数。60 から増やし、狭い屋内終盤の向きずれを抑える |
| `update_min_d` | 0.10 | この距離 [m] 以上移動したら AMCL filter を更新する。0.25 から詰めて、低速巡回中の補正間隔を短くする |
| `update_min_a` | 0.10 | この回転 [rad] 以上で AMCL filter を更新する。0.20 から詰めて、旋回中の補正を早める |
| `min_particles` / `max_particles` | 500 / 2000 | particle 数の下限 / 上限 |
| `pf_err` / `pf_z` | 0.05 / 0.99 | KLD sampling の誤差・信頼度 |
| `laser_likelihood_max_dist` | 2.0 | 障害物からの距離を likelihood field へ反映する最大距離 [m] |

### コントローラ（`controller_server` / `FollowPath` = DWB）

| パラメータ | 現在値 | 意味 / 調整の効果 |
|---|---|---|
| `controller_frequency` | 20.0 | 制御ループ周波数 [Hz] |
| `max_vel_x` | 0.26 | 前進最大速度 [m/s]（waffle 上限） |
| `max_vel_theta` | 1.0 | 旋回最大速度 [rad/s] |
| `min_vel_x` | 0.0 | 後退は無効 |
| `sim_time` | 1.7 | 軌道予測の先読み時間 [s]。短いと近視眼的、長いと滑らか |
| `xy_goal_tolerance` | 0.25 | ゴール到達判定の位置許容 [m] |
| `yaw_goal_tolerance` | 0.25 | ゴール到達判定の角度許容 [rad] |
| `BaseObstacle.scale` | 0.08 | DWB が障害物近傍の軌道を嫌う重み。壁際へ寄りすぎる場合は上げる |

### コストマップ共通（`local_costmap` / `global_costmap`）

| パラメータ | 現在値 | 意味 / 調整の効果 |
|---|---|---|
| `robot_radius` | 0.22 | ロボット半径 [m]。膨張の基準 |
| `resolution` | 0.05 | コストマップ解像度 [m/cell] |
| `inflation_layer.inflation_radius` | 0.45 | 障害物膨張半径 [m]。大きいほど壁から離れる／狭所を通れなくなる |
| `inflation_layer.cost_scaling_factor` | 2.0 | 膨張コストの減衰。大きいほど壁際コストが急減。低めにすると通路全体に緩いポテンシャルが残る |
| `local_costmap.plugins` | `obstacle_layer`, `predicted_layer`, `inflation_layer` | 近傍の即時障害物は local で回避する |
| `global_costmap.plugins` | `static_layer`, `predicted_layer`, `inflation_layer` | 保存地図ベースの大域計画を安定させるため、global には `/scan` の `obstacle_layer` を入れない |
| `local_costmap.obstacle_layer` 入力 | `/scan` | 2D 障害物。高さ帯 `min_height 0.0`（地面+0.21m以上）で地面を除外 |
| `local_costmap.obstacle_layer.footprint_clearing_enabled` | true | ロボット足元は現在存在している自由空間としてセンサ由来障害物を clear する |
| `local_costmap.obstacle_layer.observation_persistence` | 0.0 | 2D scan は**最新フレームの観測だけ**で costmap を作る（古い観測を貯めない） |
| `local_costmap.obstacle_layer.raytrace/obstacle_max_range` | 6.0 / 5.0 | raytrace（clear）距離 ≥ mark 距離。人が動いて空いた空間を確実に clear するため clear を mark より広く取る |
| `global_costmap.update/publish_frequency` | 3.0 / 2.0 | 動的予測層を早く反映するため global を高頻度更新（既定 1.0/1.0 から引き上げ） |
| **予測コストマップ層** | **`predicted_layer`（自作 `susumu_object_perception::PredictedCostmapLayer`）** | perception 連携。`prediction_node` の予測 OccupancyGrid `/perception/predicted_costmap`(map) を `max` 合成で costmap に乗せる。人の**現在位置**（全トラック）と**進路先**（移動トラック）の両方をこの層が担う（STVL 廃止後の唯一の動的障害物層） |
| `predicted_layer` 入力 | `/perception/predicted_costmap` | prediction が毎フレーム作り直す予測格子。現在位置（全トラック）+ 最有力予測パス（移動トラック、近傍2s、confidence しきい無し＝移動なら必ず焼く）。点列は**線分補間**で繋ぎ（飛び石防止）、人幅+方向ズレ吸収ぶん **8 セル円盤膨張** |
| `predicted_layer.occupied_threshold` | 50 | 予測格子のこの値以上のセルを LETHAL で焼く |

### プランナ（`planner_server` / `GridBased`）

| パラメータ | 現在値 | 意味 / 調整の効果 |
|---|---|---|
| `plugin` | `nav2_smac_planner/SmacPlanner2D` | 保存地図AMCL巡回で Navfn が free な目標への再計画に失敗したため、屋内探索で実績のある Smac 2D に統一 |
| `tolerance` | 0.5 | 目標近傍探索の許容 [m] |
| `allow_unknown` | true | unknown を通行候補に含める。保存地図の端・未確定セルで詰まりにくくする |
| `max_planning_time` | 3.5 | 1回の計画に使う最大時間 [s] |
| `cost_travel_multiplier` | 5.0 | コストの高いセルを避ける重み。大きいほど壁際や障害物近傍を避ける |
| `use_final_approach_orientation` | false | 最終接近姿勢を強制しない。巡回点では向きより到達を優先 |

> 障害物層は**人を除去しない**（人も普通の障害物として避ける）が、**地面は除去する**。
> 生 `/lidar/points` は地面点を 46% 含み、costmap の ~90% が LETHAL になって経路が
> 引けなくなる。Autoware ground_filter の出力 `/perception/no_ground/pointcloud` を使う
> ことで地面だけを除き、壁・人・什器は障害物として残す。
> 「地面除去できているか」は `/local_costmap/costmap` の LETHAL(>=99) 率で確認できる
> （90% 近ければ地面が焼かれている。正常時は 30〜40% 程度＝地図の壁が主）。

### 2.1 3D 障害物層の遍歴（1表に集約）

動的障害物（人）を costmap に乗せる層は3世代を経て、現在は自作 `predicted_layer` に確定した。
各方式の入力・蓄積特性・壁保持・通過跡・結果を比較する（時系列の経緯は [§5 調整履歴](#5-調整履歴)）。

> 注: 以下の履歴表・調整履歴に出てくる `/velodyne_points` は当時のトピック名。2026-06-18 の
> MID-360 化で 3D LiDAR トピックは `/lidar/points`、frame は `lidar_link` に改名済み。履歴は
> 当時の事実として原文のまま残す。現行の入力トピックは `/lidar/points` 系で読み替えること。

| 時期 | 層 | 入力 | 蓄積 | 壁保持 | 軌跡（通過跡） | 結果 |
|---|---|---|---|---|---|---|
| ～2026-06-14 | `voxel_layer`（Nav2 標準） | 生 `/velodyne_points`（地面除去前） | する | 維持 | 残る | 地面点 46% を焼き LETHAL ~90% で経路不能 → 入力を地面除去点群に変更 |
| 2026-06-15 | `stvl_layer`（STVL） | mark=`/perception/no_ground/pointcloud`、clear=生 `/velodyne_points`、`voxel_decay:3.0` | 時間減衰（3s） | 維持 | **`voxel_decay`(3s) 残る**＝移動軌跡のコストが出る | レイ非到達領域も寿命切れで消えるが、人の通過跡が3秒残る欠点 → **廃止** |
| ～試行 | `predicted_layer`（ObstacleLayer 点群方式） | 予測点群 | する | 維持 | 蓄積 | 古い予測が蓄積し LETHAL **55%** でぐちゃぐちゃ → 不採用 |
| ～試行 | `predicted_layer`（StaticLayer 方式） | 予測 OccupancyGrid | 置換 | **壁消失（LETHAL 0%）** | 残らない | 他層を上書きして壁が消える → 不採用 |
| **現在** | **`predicted_layer`（自作 C++ `susumu_object_perception::PredictedCostmapLayer`、max 合成）** | **`/perception/predicted_costmap`** | **毎フレーム置換（蓄積しない）** | **100% 維持** | **残らない**（毎フレーム全消去） | **壁 100%・全体 22%（健全）・進路 0.5m 先占有 58%・ナビ可。標準層では実現不能だったため自作** |

> **なぜ標準層では不可だったか**: ObstacleLayer/STVL（点群方式）は古い予測が蓄積し costmap が
> ぐちゃぐちゃに、StaticLayer（OccupancyGrid 方式）は他層を上書きして壁を消す。自作層だけが
> 「予測の占有セルだけを **max 合成**で乗せ（壁を壊さない）＋毎フレーム最新格子で置換（蓄積しない）」
> を両立できた。真値検証の詳細は `docs/autoware_perception.md`「Nav2 連携」。

---

## 3. よくある症状と調整指針

| 症状 | 疑うパラメータ | 調整方向 |
|---|---|---|
| 壁/家具に寄りすぎてこすり抜けで詰まる | `inflation_radius` / `robot_radius` | 上げる（障害物から離れる） |
| 狭いドア・通路を通れない | `inflation_radius` | 下げる（膨張を薄く）／`cost_scaling_factor` を上げる |
| ゴール手前で止まる・到達しない | `xy_goal_tolerance` / `yaw_goal_tolerance` | 上げる（判定を緩める） |
| カクついて方向転換が多い | `sim_time` | 上げる（先読みを長く） |
| `No valid trajectories`（立ち往生） | `inflation_radius` / スポーン位置 | 膨張を下げる／開けた場所へ |
| 動的障害物（人）の軌跡が残る | （STVL 廃止で解決済み） | 旧 STVL の `voxel_decay`(3s) 残留問題は廃止で解消（[§2.1](#21-3d-障害物層の遍歴1表に集約)）。現在は `predicted_layer` が毎フレーム焼き直すため軌跡は残らず、2D `/scan` の obstacle_layer も raytrace clearing で消える |
| 自己位置がずれて誤計画 | AMCL（`max_beams`, `update_min_d`, `update_min_a`, `/initialpose`） | まず `truth_monitor:=True` で Webots GPS/IMU truth と `map->base_footprint` を評価する。大きくずれる場合は AMCL 更新頻度・ビーム数・初期姿勢を疑う。GUI の「原点へワープ」で再初期化 |
| AMCL 調整後も短周期の姿勢・速度が揺れる | `robot_localization` EKF（wheel odom twist + `/imu` yaw） | Nav2/REP-105 では AMCL が `map->odom`、オドメトリ系が `odom->base_link` を担う。`robot_localization` を入れる場合は `odom->base_link` の平滑化として評価し、AMCL の代替にしない。真値 `/gps` は評価専用で、EKF 入力には使わない。cycle05 では `/odom` の x/y pose を融合せず twist と IMU yaw だけを使う構成が raw odom drift を大きく下げた。cycle06/07 の TF 置換は opt-in で競合なく完走し、起動順調整で初期 `odom->base_link` 待ちは 0 件。cycle08 の wheel radius multiplier `1.046` は path length 比を `0.999` にしたが odom aligned と進行性が悪化したため既定化は保留 |

> **歩行者（HuNav）が動かない問題は Nav2 ではない。** これは `config/agents_house.yaml`
> 側（init_pose / goals が壁・家具・別部屋にある等）の問題。Nav2 調整では直らないので
> 切り分けること（[software_design.md](software_design.md) の歩行者設定を参照）。

---

## 4. 調整の手順

1. 変更前の値と症状を記録（下の「調整履歴」に追記）。
2. `config/nav2_params.yaml` を編集。
3. `colcon build --packages-select susumu_object_perception --symlink-install` で install に反映。
4. ライブ起動して `/cmd_vel`・costmap・到達ログで効果を確認。
   - 起動中なら `ros2 param set /controller_server FollowPath.<param> <値>` で
     一部パラメータは再起動なしに試せる（恒久化は yaml 編集が必要）。
5. **本ドキュメントの「現在値」表と「調整履歴」を更新**してコミット。

---

## 5. 調整履歴

新しいものを上に追記する。

| 日付 | 変更 | 理由 / 結果 |
|---|---|---|
| 2026-06-22 | 自己位置評価 cycle01-08 を実施し、AMCL 更新値、truth/odom/EKF 診断、EKF TF opt-in、wheel radius scale をまとめて評価した。採用: `amcl.max_beams=90`, `update_min_d/a=0.10`、truth monitor の waypoint/odom/filtered 指標、`config/ekf_odom_twist_imu_eval.yaml`、EKF TF 評価用の `config/webots_ros2control_ekf_odom_tf.yaml` と起動順引数。未採用: pose+twist EKF、EKF TF の通常既定化、wheel radius multiplier `1.046` の通常推奨化 | AMCL 採用値は `reached=22/22`, max aligned `0.185m`。twist+IMU EKF は filtered max aligned `0.209m`, max yaw `3.08deg` で評価用既定に採用。EKF TF + 起動順調整は `base_link->odom` wait `0`、map max `0.227m`、EKF max `0.191m` で opt-in 採用。wheel radius `1.046` は path 比 `0.999` まで改善したが EKF max aligned `0.291m` と progress failure 1 回で未採用。詳細値は `docs/tasks/waypoint_navigation.md` の集約表と `maps/indoor_localization_cycle*_nav.*` / `_truth.*` を参照。次は radius multiplier `1.02`〜`1.03`、wheel separation、左右差を小さく切り分ける |
| 2026-06-21 | 屋外専用 `config/nav2_params_webots_explore_outdoor.yaml` で `local_costmap.plugins` に `static_layer` を入れる実験を2条件（`footprint_clearing_enabled:false/true`）で実施したが、既定未採用。最終設定は `local_costmap.plugins=["obstacle_layer","inflation_layer"]` に戻した | サイクル22で #6 の robot pose が保存地図/static と global costmap 上は occupied なのに local costmap は free だったため、DWB に保存地図を見せる仮説を検証した。`false` 版は `reached=14/53`, monitor samples `346`, `pose_static_lethal=220`。`true` 版も `reached=14/53`, monitor samples `410`, `pose_global_lethal_static_free=134`, `pose_global_lethal=115`。どちらも cycle20 既定 `reached=16/53` より悪く、#14 で `(4.4〜4.6,1.5〜1.7)` 付近の static/global/local lethal に入る主因を解けない。Nav2 StaticLayer 公式 docs と上流 `static_layer.cpp` を確認し、local static は切り分けには有効だが既定 tuning としては採用しない |
| 2026-06-20 | `planner_server.GridBased.plugin` を `nav2_navfn_planner/NavfnPlanner` から `nav2_smac_planner/SmacPlanner2D` に変更。Smac 2D の `max_planning_time:3.5`, `cost_travel_multiplier:5.0`, `use_final_approach_orientation:false` を設定。`local_costmap.obstacle_layer.footprint_clearing_enabled:true` を明示。`global_costmap.plugins` から `obstacle_layer` を外し、global は `static_layer + predicted_layer + inflation_layer` に限定。`inflation_radius:0.45`, `cost_scaling_factor:2.0`, `DWB BaseObstacle.scale:0.08` へ変更 | 保存地図AMCL巡回（`slam:=False map_file:=maps/indoor.yaml nav_params_file:=config/nav2_params.yaml`）で、相対パス解決後も Navfn が waypoint #6 `(-0.28,-3.38)` へ `failed to create plan` を繰り返した。Smac 2D では原因が `Starting point in lethal space` と分かった。footprint clearing 後および global obstacle 除去後の評価は `reached=21/22 missed=[6]`。#6 直前の推定位置・#6・#7 は保存地図上 free だったが、走行中にロボットが壁際へ寄り、再計画時の footprint 内に static/inflation の lethal が入った。Nav2 公式の tuning guide は通路全体に滑らかな inflation potential を作り、Smac 2D は `cost_travel_multiplier` で高コスト領域から離すと説明しているため、waypoint 数を減らさず中央寄せを強めた。最終評価（`mode:=realtime`）は `reached=22/22 missed=[]`、#6 も成功。参照: Nav2 Tuning Guide / Smac 2D Planner docs / DWB Controller docs |
| 2026-06-15 | **STVL 層（`stvl_layer`）を local/global から削除**。人の現在位置の障害物化を `predicted_layer`（予測層）に統合し、`prediction_node` が全トラックの現在位置 + 移動トラックの進路先を予測 OccupancyGrid に焼く。予測パスは confidence しきい撤廃（移動なら必ず焼く）、点列を**線分補間**で連続描画、膨張 6→8 セル | **STVL は人の通過跡を `voxel_decay`(3s) 残すので「移動軌跡のコスト」が出る**問題。予測層は毎フレーム全消去するので軌跡が残らない。これで現在位置・進路先を一括で担う。検証: 進路が出るフレーム 95%→**100%**、進路上の連続性 60%→**77%**、costmap 全体 LETHAL 25%（健全）、壁 100% 維持、ナビ可能 |
| 2026-06-15 | **予測コストマップ層を自作 C++ プラグイン `susumu_object_perception::PredictedCostmapLayer` に確定**（local/global）。`prediction_node` の予測 OccupancyGrid `/perception/predicted_costmap`(map) を `max` 合成で乗せる。`occupied_threshold:50` | **perception を Nav2 に連携する初の層**。当初 ObstacleLayer 点群方式 → **古い予測が蓄積し costmap が LETHAL 55% でぐちゃぐちゃ**になりナビ不能。次に StaticLayer(OccupancyGrid)方式 → **他層を上書きして壁が消失(LETHAL 0%)**。最終的に **max 合成の自作 C++ 層**で「他層を壊さず(壁100%維持)・蓄積せず(全体22%健全)」を両立。真値検証で移動中の人の進路 0.5m 先占有 58%、NavigateToPose ゴール受理 OK。**標準層では毎フレーム入れ替えデータを costmap に入れられない（ObstacleLayer=蓄積/StaticLayer=上書き）のが教訓** |
| 2026-06-15 | **3D 障害物層を Nav2 標準 `voxel_layer` → STVL（`spatio_temporal_voxel_layer/SpatioTemporalVoxelLayer`）に置換**（local/global 両方）。`voxel_decay:3.0`(線形)。mark=`/perception/no_ground/pointcloud`、clear=生 `/velodyne_points`(VLP16 frustum, `model_type:1`)。`ros-humble-spatio-temporal-voxel-layer` を apt 導入。2D `obstacle_layer`（/scan）と static_layer は未変更 | **persistence:0 + raytrace だけでは歩く人の跡が消えきらない**（人がレイを遮った背後はクリアされず残る）問題への対策。STVL は voxel に観測時刻を持たせ `voxel_decay` 秒で**時間減衰により自動消去**するため、レイが当たらない領域も寿命切れで消える。動的環境向けの定番手法を既存パッケージ（新規開発なし）で採用 |
| 2026-06-14 | voxel_layer 入力を `/velodyne_points` → `/perception/no_ground/pointcloud`（Autoware 地面除去済み）に変更、高さ帯 min/max=-0.18/1.8。`/scan` の生成高さ帯 min_height -0.20→0.0 | **自動巡回が動かなかった**原因が、生点群の地面（46%）を costmap が障害物化し local_costmap の 90% が LETHAL だったこと。地面除去点群に切替で 90%→37% になり経路生成・ゴール到達を確認 |
| 2026-06-14 | obstacle_layer/voxel_layer の入力を生 `/scan`・`/velodyne_points` に設定 | 純粋シミュレーター化に伴い、人も普通の障害物として costmap に乗せる |

> 構築・調整の詳細な経緯は [`../SETUP.md`](../SETUP.md) を参照。

## 6. 参照した一次情報

- Nav2 AMCL configuration: https://docs.nav2.org/configuration/packages/configuring-amcl.html
- Nav2 tuning guide: https://docs.nav2.org/tuning/index.html
- Nav2 AMCL source (`shouldUpdateFilter`): https://github.com/ros-navigation/navigation2/blob/main/nav2_amcl/src/amcl_node.cpp
- Nav2 Smoothing Odometry using Robot Localization: https://docs.nav2.org/setup_guides/odom/setup_robot_localization.html
- Nav2 Transform setup / REP-105 summary: https://docs.nav2.org/setup_guides/transformation/setup_transforms.html
- ROS 2 Control diff_drive_controller parameters: https://control.ros.org/humble/doc/ros2_controllers/diff_drive_controller/doc/userdoc.html
- ROS 2 Launch design / process orchestration: https://design.ros2.org/articles/roslaunch.html
- robot_localization EKF example parameters: https://github.com/cra-ros-pkg/robot_localization/blob/ros2/params/ekf.yaml
- Webots TurtleBot3Burger PROTO wheel radius: https://raw.githubusercontent.com/cyberbotics/webots/R2022b/projects/robots/robotis/turtlebot/protos/TurtleBot3Burger.proto
- nav_msgs/Odometry frame contract: https://docs.ros2.org/foxy/api/nav_msgs/msg/Odometry.html
