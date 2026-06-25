# 外部キャリブレーションタスク — 全天球カメラ + 3D LiDAR

このページは README / AGENTS のタスク一覧「外部キャリブレーション」の詳細ページ。
シミュレータ上で **全天球カメラと 3D LiDAR の外部パラメータ（`lidar_link -> omni_camera_link`）**
を推定し、色付き点群・物体クロップなど LiDAR×カメラ連携の TF をキャリブ結果で置き換えるところまでを扱う。

手法の詳細・投影モデル・実測値・落とし穴の正本は [`../omni_lidar_camera.md`](../omni_lidar_camera.md)。
ここはタスクとしての目的・入出力・実行・合格基準・制約に絞る（重複定義しない）。

## 入出力

| 項目 | 内容 |
|---|---|
| 入力 | `webots_worlds/calibration.wbt`（4 方位に AprilTag 36h11 パネル）、`/omni_camera/image_raw/image_color`（equirect）、`/lidar/points/point_cloud` |
| 実行 | `launch/webots_calibration.launch.py apriltag_calib:=True` |
| 出力（最終） | `~/ros2_ws/src/susumu_object_perception/outputs/extrinsic_calibration/calib.json`（`results.T_lidar_camera` = `[x,y,z,qx,qy,qz,qw]`、`p_lidar = T * p_camera`） |
| 出力（中間） | `experiments/extrinsic_calibration/<YYYY-MM-DD>_<label>/`（試行版 calib、PnP/平面フィットの中間ログ、複数回測定。gitignore） |
| 利用 | calib.json を `omni_calibration_json:=...` で渡すと `omni_sensor_tf_node` が `lidar_link -> omni_camera_link` TF を置換。色付き点群 / 物体クロップ / 色付き SLAM 地図が同じ TF を使う |

## 方式（2 系統）

| 方式 | 内容 | 位置づけ |
|---|---|---|
| **AprilTag 既知ターゲット** | calibration.wbt の 4 方位パネルに AprilTag 36h11 を貼り、全天球を透視ビュー展開 → `cv2.aruco` 検出 → `solvePnP` でタグのカメラ座標、LiDAR は方位切出し+平面 RANSAC で板中心、Umeyama で 6DoF 推定。`apriltag_extrinsic_calib_node.py` | 本タスクの主対象。`apriltag_ros` 非依存・独自 msg 無し |
| **ターゲットレス** | `direct_visual_lidar_calibration`（equirectangular 対応）を Docker 経由で実行 | 既存導線。`run_direct_visual_lidar_calibration_docker.sh` |

どちらも出力は同じ `calib.json` 形式で、`omni_sensor_tf_node.py` / `scripts/direct_calib_to_tf.py` が読める。

## 実行（AprilTag 方式）

```bash
cd ~/ros2_ws
colcon build --packages-select susumu_object_perception --symlink-install
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/local_setup.bash
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

# タグテクスチャ生成（初回のみ。webots_worlds/apriltag_textures/ に出る）
ros2 run susumu_object_perception generate_apriltag_textures.py --ids 0,1,2,3

# キャリブ実行 → calib.json を出力
ros2 launch susumu_object_perception webots_calibration.launch.py \
  mode:=realtime rviz:=False perception:=False colored_slam:=False \
  apriltag_calib:=True

# 得た TF で色付け（活用。omni_sensor_tf_node が calib.json を読む）
ros2 launch susumu_object_perception webots_calibration.launch.py \
  colored_slam:=True \
  omni_calibration_json:=~/ros2_ws/src/susumu_object_perception/outputs/extrinsic_calibration/calib.json
# 色付き点群を PLY 保存
ros2 service call /slam/save_colorized_map std_srvs/srv/Trigger {}
```

## 合格基準

1. **検出と推定**
   calibration.wbt で 4 方位の AprilTag を全て検出し、`calib.json` に `T_lidar_camera` を出力できる。
2. **精度（色付け用途の基準）**
   真値 `lidar_link -> omni_camera_link = z 0.55 / 無回転` に対し **回転 1°未満・対応点 RMS 1cm 程度**。
   現状 realtime で回転 0.32°・RMS 9.6mm を達成。
3. **活用**
   calib.json を `omni_calibration_json:=...` で渡すと TF が初期値 `[0,0,0.55]` からキャリブ実測値へ置換され、
   `colorized_pointcloud_node` がそのキャリブ TF で色付けする。色付け品質（カラー物体の色一致スコア）が
   初期 TF と同等以下に劣化しない（実測: green 0.970→0.976 / magenta 0.985→0.982）。

## 制約・注意

- **並進絶対誤差は現状 24mm 残り、1cm 未満は未達**（x の -23mm 系統オフセット）。主因は LiDAR がタグ板の
  下半分にしか点を返さず点群重心が板中心より下に偏ること（MID-360 上向き FOV + パネル高 0.75m のシミュ
  固有幾何）。板厚補正では消えない。1cm 未満を狙うならパネルを LiDAR 水平面（z0.20）中心へ下げる等が要る。
  色付け用途には回転 0.32°・RMS 9.6mm で十分なので、精密化は必要になってから。
- **LiDAR 点群トピックは `/lidar/points/point_cloud`**（Webots driver の `/point_cloud` サフィックス）。
- **Webots の `Box` テクスチャは回転（`rotation`）の有無で貼られ方が変わる**。タグ板は回転で配置せず
  Box 寸法 + translation で正対させる（回転ありだとタグが白飛びして未検出になる）。
- **CycloneDDS 推奨**（全天球画像が大きく、FastRTPS SHM の罠を避ける）。
- 評価モードは `mode:=realtime`。
- 独自 `.msg` は作らない。検出は OpenCV `cv2.aruco`、出力は vlcal 互換 calib.json。

## 試行・未採用 (2026-06-26)

「パネル z を LiDAR 水平面 (z=0.20) 中心へ下げる」案をライブで検証。 結果:

| パネル z | 結果 | 判定 |
|---|---|---|
| **0.75 (採用)** | RMS 9.6mm / transl err 24.2mm | **採用維持** |
| 0.20 (LiDAR水平面) | cam tags=[] (camera から見えない) | 不可 |
| 0.475 (LiDAR と camera の中間) | RMS 12.7mm / transl err 33.8mm | 改悪 |

- z=0.20 は omni camera (z=0.75) から見て真下になり画角外で検出できず。
- z=0.475 は LiDAR 視点での「板上半分の重心ズレ」と camera 見下げ角の「タグ歪み」の
  両方が重畳し採用版より精度低下。

**結論**: パネルだけ動かしても精度改善しない。 1cm 未満を狙うなら cam/LiDAR 配置の
変更か、 LiDAR 点群の理論補正 (上向き FOV で偏った重心を解析的にシフト) が要る。
実験ファイルは `experiments/extrinsic_calibration/2026-06-26_panel_z020/` に保存。

**iter10 (lidar_z_use_range_mid 追試)**: 重心 z を「点群 z 範囲中央 (max+min)/2」
に置換するパラメータを実装し検証。 結果は RMS 10.0mm / translation 27.8mm で
採用版より悪化。 「下半分点群の範囲中央」自体が板物理中心と一致せず、 補正が
逆効果。 引き続き採用版 (mean、 板厚補正のみ) を維持。 機能はパラメータとして
残るので将来の検証 (重み付き重心、 板上端推定) で活用できる。

**iter25 (board_height_assumption 追試)**: 「上端 z (max) を板上端と仮定し、 そこから
板高の半分を引いた値を中心とする」 案を実装し検証。 calibration.wbt のパネル高
0.3m を `board_height_assumption:=0.3` で渡してライブ実行。 結果は z translation
0.8635m (期待 0.55m、 **314mm 大幅悪化**) / RMS 10.0mm。 採用版から **z が大きく
ずれた**。 原因: `max()` ベース推定が天井反射等の外れ値に敏感で、 板上端ではなく
高い位置の点に支配される。 → **不採用**、 採用版 (mean、 板厚補正のみ) を維持。
実装はそのまま残し、 既定 0.0 で OFF。 将来別のロバスト推定 (中央値、 上位
10% 平均) を試す際の足場として活用可能。
実験ファイル: `experiments/extrinsic_calibration/2026-06-26_iter25_board_height_assumption/`

**iter26 (board_top_quantile 追試)**: iter25 の max() を上位分位平均に置き換えた
ロバスト推定を試す。 q=0.1/0.3/0.5 で z translation err = 294/190/91mm。 quantile を
上げると採用版 (mean) に収束するだけで、 **採用版を超えるロバスト推定値は存在しない**
ことが実機実証で判明。 真因は「下半分点群の偏り」 vs 「上端の外れ値」 のトレードオフ
で、 数値補正では超えられない。 1cm 未満達成には LiDAR 物理モデルの解析的補正が
必要。 → **不採用**、 機能はパラメータとして残す (既定 0.0 で OFF、 既存と完全互換)。
実験ファイル: `experiments/extrinsic_calibration/2026-06-26_iter26_top_quantile/`

## 関連

- [全天球カメラ + LiDAR 色付き点群メモ](../omni_lidar_camera.md)（手法・実測・落とし穴の正本）
- [カラー点群出力タスク](colorized_pointcloud.md)（キャリブ結果を使う下流タスク）
- 検証画像: `docs/images/apriltag_calib_*.png`（全天球画像と各方位の透視ビュー）
