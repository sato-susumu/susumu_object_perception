#!/usr/bin/env bash
# 全タスクを順番に実行するエントリポイント。/goal「全タスクを順番に全部実行」を1コマンドで満たす。
#
# 各タスクの最終成果物が outputs/<task>/ に再生成される。世界が複数あるケース
# (mapping_indoor: indoor + break_room、waypoint_generation: 全 map) は順次回す。
#
# 注意: Webots GUI が要る。所要時間 60〜90 分目安。途中で止めるには Ctrl-C を 2 回叩く。
#
# 個別タスクだけ走らせたいときは、引数で開始ステップを指定する。
#   bash scripts/run_all_tasks.sh wp   # ウェイポイント生成から
#   bash scripts/run_all_tasks.sh recog
#
# 設定: 1 タスクあたり既定 480s（8分）でタイムアウト。長時間 world は STEP_TIMEOUT_SEC で延長可。

PKG="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
LOGDIR="${LOGDIR:-/tmp/run_all_tasks_$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$LOGDIR"
STEP_TIMEOUT_SEC=${STEP_TIMEOUT_SEC:-480}
COLOR_SEC=${COLOR_SEC:-90}
CALIB_SEC=${CALIB_SEC:-120}

cd "$PKG"
source /opt/ros/humble/setup.bash
source /home/taro/ros2_ws/install/local_setup.bash
export TURTLEBOT3_MODEL=waffle
export RMW_IMPLEMENTATION=rmw_cyclonedds_cpp

killall_webots() {
  pkill -9 -f "webots-bin" 2>/dev/null || true
  pkill -9 -f "ros2_supervisor" 2>/dev/null || true
  pkill -9 -f "component_container" 2>/dev/null || true
  pkill -9 -f "frontier_explore" 2>/dev/null || true
  pkill -9 -f "waypoint_nav_node" 2>/dev/null || true
  pkill -9 -f "object_memory_node" 2>/dev/null || true
  pkill -9 -f "webots_controller_TurtleBot3" 2>/dev/null || true
  pkill -9 -f "webots_ros2_driver" 2>/dev/null || true
  pkill -9 -f "ros2 launch" 2>/dev/null || true
  sleep 3
  rm -rf /tmp/webots/taro 2>/dev/null || true
}

run_mapping() {
  local WORLD=$1
  local NAME=$2
  local LOG="$LOGDIR/mapping_${NAME}.log"
  echo "[$(date -Iseconds)] TASK 1 mapping $WORLD" | tee "$LOG"
  timeout "$STEP_TIMEOUT_SEC" ros2 launch susumu_object_perception webots_indoor_mapping.launch.py \
    world:="$WORLD" map_name:="$NAME" mode:=realtime save_map:=True \
    rviz:=False image_recognition:=False collision_diagnostics:=False \
    >> "$LOG" 2>&1
  echo "[$(date -Iseconds)] mapping $NAME done exit=$?" | tee -a "$LOG"
  killall_webots
  ls -la "$PKG/outputs/mapping_indoor/$NAME".{pgm,yaml} 2>&1 | tee -a "$LOG"
}

run_wp() {
  local NAME=$1
  local YAML=$2
  local LOG="$LOGDIR/wp_${NAME}.log"
  echo "[$(date -Iseconds)] TASK 2 WP $NAME" | tee "$LOG"
  python3 "$PKG/scripts/generate_waypoints.py" \
    --map "$YAML" \
    --out "$PKG/outputs/waypoint_generation/${NAME}_waypoints.yaml" \
    2>&1 | tee -a "$LOG" | tail -5
}

run_patrol() {
  local NAME=$1
  local WORLD=$2
  local WP=$3
  local LOG="$LOGDIR/patrol_${NAME}.log"
  echo "[$(date -Iseconds)] TASK 3 patrol $WORLD" | tee "$LOG"
  # report_prefix を渡して JSON/CSV/Markdown を必ず出す → PNG 可視化の入力にする
  local REPORT_PREFIX="$PKG/outputs/waypoint_generation/${NAME}_patrol_report"
  timeout "$STEP_TIMEOUT_SEC" ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
    world:="$WORLD" waypoints:="$WP" mode:=realtime \
    rviz:=False loop:=False image_recognition:=False \
    perception:=False omni_perception:=False \
    report_prefix:="$REPORT_PREFIX" \
    >> "$LOG" 2>&1
  echo "[$(date -Iseconds)] patrol $NAME exit=$?" | tee -a "$LOG"
  killall_webots
  # === 巡回結果 PNG 必須生成 (ユーザー指示: 巡回タスクで PNG 結果出力) ===
  local MAP="$PKG/outputs/mapping_indoor/${NAME}.yaml"
  local REPORT_JSON="${REPORT_PREFIX}.json"
  local PATROL_PNG="$PKG/outputs/waypoint_generation/${NAME}_patrol_result.png"
  if [[ -f "$REPORT_JSON" && -f "$MAP" ]]; then
    echo "[$(date -Iseconds)] rendering patrol PNG -> $PATROL_PNG" | tee -a "$LOG"
    python3 "$PKG/scripts/visualize_patrol_result.py" \
      --map "$MAP" --report "$REPORT_JSON" --out "$PATROL_PNG" \
      >> "$LOG" 2>&1 || echo "  -> patrol PNG render failed" | tee -a "$LOG"
  else
    echo "[$(date -Iseconds)] WARN: report=$REPORT_JSON or map=$MAP missing; skip patrol PNG" | tee -a "$LOG"
  fi
}

run_recog() {
  local NAME=$1
  local WORLD=$2
  local WP=$3
  local LOG="$LOGDIR/recog_${NAME}.log"
  local DB="$HOME/.ros/object_memory.sqlite3"
  echo "[$(date -Iseconds)] TASK 4 recognition patrol $WORLD" | tee "$LOG"
  rm -f "$DB"
  # 認識巡回でも reached/missed PNG を出すため report_prefix を渡す
  local RECOG_PATROL_PREFIX="$PKG/outputs/recognition/${NAME}_recognition_patrol_report"
  timeout "$STEP_TIMEOUT_SEC" ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
    world:="$WORLD" waypoints:="$WP" mode:=realtime \
    rviz:=False loop:=False \
    perception:=True omni_perception:=True image_recognition:=True \
    report_prefix:="$RECOG_PATROL_PREFIX" \
    >> "$LOG" 2>&1
  echo "[$(date -Iseconds)] recog $NAME exit=$?" | tee -a "$LOG"
  killall_webots
  # === 認識結果の可視化 PNG 必須生成 ===
  # webots_simulation.launch.py で object_memory_node が ~/.ros/object_memory.sqlite3
  # に書く前提。DB が無いときは「認識が動かなかった」サインとして強調表示する。
  local MAP="$PKG/outputs/mapping_indoor/${NAME}.yaml"
  local OVERLAY_PNG="$PKG/outputs/recognition/${NAME}_recognition_overlay.png"
  local EVAL_PREFIX="$PKG/outputs/recognition/${NAME}_recognition_eval"
  local WBT="$PKG/webots_worlds/${WORLD}"
  if [[ ! -f "$DB" ]]; then
    echo "[$(date -Iseconds)] WARN: DB missing ($DB); skip PNG visualization" | tee -a "$LOG"
    return
  fi
  if [[ -f "$MAP" ]]; then
    echo "[$(date -Iseconds)] rendering overlay -> $OVERLAY_PNG" | tee -a "$LOG"
    python3 "$PKG/scripts/render_recognition_overlay.py" \
      --map "$MAP" --db "$DB" --out "$OVERLAY_PNG" \
      --min-existence 0.5 --min-hits 2 \
      >> "$LOG" 2>&1 || echo "  -> overlay render failed" | tee -a "$LOG"
  fi
  if [[ -f "$WBT" && -f "$MAP" ]]; then
    echo "[$(date -Iseconds)] evaluating vs world -> ${EVAL_PREFIX}.{md,json,csv,png}" | tee -a "$LOG"
    python3 "$PKG/scripts/evaluate_recognition_vs_world.py" \
      --wbt "$WBT" --map "$MAP" --db "$DB" \
      --out-prefix "$EVAL_PREFIX" \
      --min-existence 0.5 --min-hits 2 --match-distance 1.0 \
      >> "$LOG" 2>&1 || echo "  -> eval failed" | tee -a "$LOG"
  fi
  # 認識巡回の reached/missed PNG も生成 (ユーザー指示: 巡回タスクで PNG 結果出力)
  local RECOG_PATROL_JSON="${RECOG_PATROL_PREFIX}.json"
  local RECOG_PATROL_PNG="$PKG/outputs/recognition/${NAME}_recognition_patrol_result.png"
  if [[ -f "$RECOG_PATROL_JSON" && -f "$MAP" ]]; then
    echo "[$(date -Iseconds)] rendering recognition patrol PNG -> $RECOG_PATROL_PNG" | tee -a "$LOG"
    python3 "$PKG/scripts/visualize_patrol_result.py" \
      --map "$MAP" --report "$RECOG_PATROL_JSON" --out "$RECOG_PATROL_PNG" \
      >> "$LOG" 2>&1 || echo "  -> recog patrol PNG failed" | tee -a "$LOG"
  fi
}

run_color() {
  local NAME=$1
  local WORLD=$2
  local LOG="$LOGDIR/color_${NAME}.log"
  echo "[$(date -Iseconds)] TASK 5 colorized $WORLD" | tee "$LOG"
  ros2 launch susumu_object_perception webots_simulation.launch.py \
    world:="$WORLD" nav:=False rviz:=False \
    omni_perception:=True colored_slam:=True mode:=realtime \
    perception:=False image_recognition:=False \
    >> "$LOG" 2>&1 &
  local PID=$!
  sleep "$COLOR_SEC"
  ros2 service call /slam/save_colorized_map std_srvs/srv/Trigger {} >> "$LOG" 2>&1 || true
  sleep 5
  kill -9 $PID 2>/dev/null || true
  killall_webots
  local LATEST
  LATEST=$(ls -t "$PKG/outputs/colorized_pointcloud/colorized_map_"*.ply 2>/dev/null | head -1)
  if [[ -n "$LATEST" ]]; then
    cp -f "$LATEST" "$PKG/outputs/colorized_pointcloud/colorized_pointcloud_${NAME}_apriltag_calib_final.ply"
    echo "[$(date -Iseconds)] color $NAME saved as final" | tee -a "$LOG"
  fi
  # === ユーザー指示「評価できることは必ず PNG 生成」 への対応 (iter31) ===
  # 採用 PLY を check_colorized_cloud.py で可視化 (XY top-view + XZ side-view)。
  # ブレ・壁の二重化・床下散乱を一目確認できる成果物として outputs/ に置く。
  local FINAL_PLY="$PKG/outputs/colorized_pointcloud/colorized_pointcloud_${NAME}_apriltag_calib_final.ply"
  local CHECK_PNG="$PKG/outputs/colorized_pointcloud/colorized_pointcloud_${NAME}_apriltag_calib_final_check.png"
  if [[ -f "$FINAL_PLY" ]]; then
    echo "[$(date -Iseconds)] rendering colorized check PNG -> $CHECK_PNG" | tee -a "$LOG"
    python3 "$PKG/scripts/check_colorized_cloud.py" "$FINAL_PLY" --out "$CHECK_PNG" \
      >> "$LOG" 2>&1 || echo "  -> colorized PNG render failed" | tee -a "$LOG"
  fi
}

run_calib() {
  local LOG="$LOGDIR/calib.log"
  echo "[$(date -Iseconds)] TASK 6 calibration" | tee "$LOG"
  ros2 launch susumu_object_perception webots_calibration.launch.py \
    mode:=realtime rviz:=False perception:=False colored_slam:=False \
    apriltag_calib:=True \
    >> "$LOG" 2>&1 &
  local PID=$!
  sleep "$CALIB_SEC"
  kill -9 $PID 2>/dev/null || true
  killall_webots
  echo "[$(date -Iseconds)] calib done" | tee -a "$LOG"
  cat "$PKG/outputs/extrinsic_calibration/calib.json" 2>/dev/null | head -5 | tee -a "$LOG"
}

START=${1:-mapping}
SKIP_MAPPING=${SKIP_MAPPING:-0}

run_vs_world() {
  local NAME=$1
  local WBT="$PKG/webots_worlds/${NAME}.wbt"
  local YAML="$PKG/outputs/mapping_indoor/${NAME}.yaml"
  local PNG="$PKG/outputs/mapping_indoor/${NAME}_vs_world.png"
  local JSON="$PKG/outputs/mapping_indoor/${NAME}_vs_world.json"
  local LOG="$LOGDIR/vs_world_${NAME}.log"
  if [[ ! -f "$WBT" ]]; then
    echo "[$(date -Iseconds)] vs_world skip $NAME: wbt missing ($WBT)" | tee "$LOG"
    return
  fi
  if [[ ! -f "$YAML" ]]; then
    echo "[$(date -Iseconds)] vs_world skip $NAME: yaml missing ($YAML)" | tee "$LOG"
    return
  fi
  echo "[$(date -Iseconds)] vs_world $NAME" | tee "$LOG"
  python3 "$PKG/scripts/check_map_vs_world.py" \
    --wbt "$WBT" --map "$YAML" --out "$PNG" --report "$JSON" \
    >> "$LOG" 2>&1
  echo "  -> $PNG" | tee -a "$LOG"
}

case "$START" in mapping|all)
  if [[ "$SKIP_MAPPING" == "1" ]]; then
    echo "[$(date -Iseconds)] TASK 1 SKIP_MAPPING=1; 既存 outputs/mapping_indoor/ を採用版として使う"
    ls -la "$PKG/outputs/mapping_indoor/"
    # SKIP でも vs_world は必ず最新化する。launch 統合と同じ仕組みで品質確認画像を残す。
    for NAME in indoor break_room cafe; do
      run_vs_world "$NAME"
    done
  else
    run_mapping indoor.wbt indoor
    run_mapping break_room.wbt break_room
    # launch 内で frontier_explore_node が自動生成するが、save_map が失敗した場合の
    # フォールバックとしてここでも呼ぶ（冪等）。cafe は wbt 無しなので関数側で skip。
    for NAME in indoor break_room cafe; do
      run_vs_world "$NAME"
    done
  fi
;; esac

case "$START" in mapping|wp|all)
  for NAME in indoor break_room cafe; do
    YAML="$PKG/outputs/mapping_indoor/$NAME.yaml"
    PGM="$PKG/outputs/mapping_indoor/$NAME.pgm"
    if [[ -f "$YAML" && -f "$PGM" ]]; then run_wp "$NAME" "$YAML"; fi
  done
;; esac

case "$START" in mapping|wp|patrol|all)
  run_patrol indoor indoor.wbt indoor_waypoints.yaml
;; esac

case "$START" in mapping|wp|patrol|recog|all)
  run_recog indoor indoor.wbt indoor_waypoints.yaml
;; esac

case "$START" in mapping|wp|patrol|recog|color|all)
  run_color indoor indoor.wbt
;; esac

case "$START" in mapping|wp|patrol|recog|color|calib|all)
  run_calib
;; esac

echo "===== ALL DONE ====="
ls -la "$PKG/outputs/"*/
