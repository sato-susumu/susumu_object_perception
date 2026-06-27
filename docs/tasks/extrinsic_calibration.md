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
| 出力（最終） | `~/ros2_ws/src/susumu_object_perception/outputs/extrinsic_calibration/calib.json`（`results.T_lidar_camera` = `[x,y,z,qx,qy,qz,qw]`、`p_lidar = T * p_camera`）、 `outputs/extrinsic_calibration/calib_summary.{png,json,md}` (真値 vs 推定値 bar chart + RMS/RPY/quaternion テーブルの可視化、採用基準の機械可読 summary、`run_all_tasks.sh` の `run_calib` が自動生成) |
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
  omni_calibration_json:=~/ros2_ws/src/susumu_object_perception/outputs/extrinsic_calibration/calib.json \
  strict_omni_calibration_json:=True
# 色付き点群を PLY 保存
ros2 service call /slam/save_colorized_map std_srvs/srv/Trigger {}

# キャリブ結果のサマリ PNG を生成 (iter33 で追加、 run_all_tasks.sh は自動実行)
# 真値 vs 推定値 bar chart + 数値テーブル (RMS / RPY / quaternion / 各軸 diff)
ros2 run susumu_object_perception visualize_calib_result.py \
  --calib outputs/extrinsic_calibration/calib.json \
  --out outputs/extrinsic_calibration/calib_summary.png \
  --json-out outputs/extrinsic_calibration/calib_summary.json \
  --md-out outputs/extrinsic_calibration/calib_summary.md \
  --require-pass
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

4. **採用判定が機械可読に残る**
   `calib_summary.json` の `validation_passed` が true。既定基準は used tags `>=4`、RMS `<=10mm`、
   回転角 `<=1deg`、並進誤差 `<=30mm`。現在値は RMS `9.60mm`、回転角 `0.317deg`、
   並進誤差 `24.2mm`。iter41 以降、`calib_summary.json` は `summary`、`criteria`、
   `failures` を持つ。iter52 以降は `schema_version: 3` とし、`results.T_lidar_camera` が
   7 要素・有限値・単位 quaternion（既定 norm 誤差 `<=1e-3`）であること、AprilTag ID が
   重複なく `>=4` 個あることも採用判定に含める。`visualize_calib_result.py --require-pass` は
   NG 時に非ゼロ終了し、`run_all_tasks.sh` の calibration phase と `validate_contracts.py` は
   `calib_summary.json` の `validation_passed=true` と schema 3 の構造検証まで検査する。iter43 以降、
   `calib_summary.json` は入力 `calib.json` の `calib_sha256` も持ち、`validate_contracts.py` が
   現在の `calib.json` と一致することを再計算して検査する。これにより `calib.json` だけが
   差し替わった stale summary を検出できる。iter58 以降、`omni_sensor_tf_node.py` と
   `scripts/direct_calib_to_tf.py` も `T_lidar_camera` の 7 要素・有限値・quaternion norm
   (`calibration_quaternion_norm_tolerance` / 既定 `1e-3`) を読み込み時に検査する。strict 起動時に
   壊れた calib が初期 TF や identity quaternion へ丸め込まれないことを runtime 側でも保証する。

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
- 厳密検証では `strict_omni_calibration_json:=True` を指定する。`calib.json` が壊れている /
  パスが違う場合に初期 TF へ silently fallback せず、起動を失敗させるため。
  `calibration_quaternion_norm_tolerance` は既定 `1e-3` で、`calib_summary.json` の判定基準と揃える。

方針参考:
- OpenCV `solvePnP`: 3D object points と 2D image points から object pose を推定する。
  <https://docs.opencv.org/4.x/d5/d1f/calib3d_solvePnP.html>
- direct_visual_lidar_calibration: equirectangular camera model と `T_lidar_camera` を扱える。
  <https://koide3.github.io/direct_visual_lidar_calibration/programs/>

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

### 試行済みで採用しなかった補正案 (実装は削除済み)

数値ベースの z 補正案を 3 件試したがいずれも採用版より悪化したため、 ソースから
完全削除した。 学び:

- **「点群 z 範囲中央 (max+min)/2」**: RMS 10.0mm / translation 27.8mm → 悪化
  (実機 iter10、 calibration.wbt)
- **「板高仮定で z_top - board_height/2」**: z translation 0.8635m / 314mm
  大幅悪化 (実機 iter25)。 真因は max() ベース推定が天井反射等の外れ値に支配される
- **「上位分位平均でロバスト z_top 推定」**: q=0.1/0.3/0.5 で err=294/190/91mm。
  quantile を上げると採用版 (mean) に収束するだけ (実機 iter26)。 「下半分偏り」 vs
  「上端外れ値」 のトレードオフで数値補正では超えられない

**結論**: 数値的な z 補正では 1cm 未満は達成不能。 1cm 未満達成には LiDAR 物理
モデルの解析的補正 (上向き FOV 上限から偏り量を sin θ で計算) または target 配置の
見直しが必要。 短期的にはこれ以上踏み込まない。

## 関連

- [全天球カメラ + LiDAR 色付き点群メモ](../omni_lidar_camera.md)（手法・実測・落とし穴の正本）
- [カラー点群出力タスク](colorized_pointcloud.md)（キャリブ結果を使う下流タスク）
- 検証画像: `docs/images/apriltag_calib_*.png`（全天球画像と各方位の透視ビュー）
