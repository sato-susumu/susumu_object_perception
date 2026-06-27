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
set -o pipefail

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
  # eval PNG 生成 (iter32 追加、 ユーザー指示「評価できることは必ず PNG」 対応)。
  # vs_world.png は wbt 真値必要 (cafe など Gazebo world は不可) のため、 内部品質
  # eval PNG を常に出して contracts に置く。
  local MAP_YAML="$PKG/outputs/mapping_indoor/${NAME}.yaml"
  if [[ -f "$MAP_YAML" ]]; then
    echo "[$(date -Iseconds)] rendering mapping eval -> ${NAME}_eval.{png,json,md}" | tee -a "$LOG"
    if ! python3 "$PKG/scripts/eval_map_quality.py" "$MAP_YAML" \
      --png-dir "$PKG/outputs/mapping_indoor/" \
      --json-out "$PKG/outputs/mapping_indoor/${NAME}_eval.json" \
      --md-out "$PKG/outputs/mapping_indoor/${NAME}_eval.md" \
      >> "$LOG" 2>&1; then
      echo "  -> mapping eval render failed" | tee -a "$LOG"
      return 1
    fi
  fi
}

run_wp() {
  local NAME=$1
  local YAML=$2
  local LOG="$LOGDIR/wp_${NAME}.log"
  echo "[$(date -Iseconds)] TASK 2 WP $NAME" | tee "$LOG"
  if ! python3 "$PKG/scripts/generate_waypoints.py" \
    --map "$YAML" \
    --out "$PKG/outputs/waypoint_generation/${NAME}_waypoints.yaml" \
    2>&1 | tee -a "$LOG" | tail -5; then
    echo "  -> waypoint generation failed" | tee -a "$LOG"
    return 1
  fi
  local CHECK_JSON="$PKG/outputs/waypoint_generation/${NAME}_waypoints_check.json"
  local CHECK_MD="$PKG/outputs/waypoint_generation/${NAME}_waypoints_check.md"
  local MAP_CLEARANCE=0.6
  echo "[$(date -Iseconds)] checking waypoint quality -> ${NAME}_waypoints_check.{json,md}" | tee -a "$LOG"
  if ! python3 "$PKG/scripts/check_waypoints.py" \
    --map "$YAML" \
    --waypoints "$PKG/outputs/waypoint_generation/${NAME}_waypoints.yaml" \
    --clearance "$MAP_CLEARANCE" \
    --connect-clearance 0.30 \
    --json-out "$CHECK_JSON" \
    --md-out "$CHECK_MD" \
    --require-pass \
    >> "$LOG" 2>&1; then
    echo "  -> waypoint quality check failed" | tee -a "$LOG"
    return 1
  fi
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
  local PATROL_SUMMARY_JSON="$PKG/outputs/waypoint_generation/${NAME}_patrol_result.json"
  local PATROL_SUMMARY_MD="$PKG/outputs/waypoint_generation/${NAME}_patrol_result.md"
  if [[ -f "$REPORT_JSON" && -f "$MAP" ]]; then
    echo "[$(date -Iseconds)] rendering patrol PNG -> $PATROL_PNG" | tee -a "$LOG"
    if ! python3 "$PKG/scripts/visualize_patrol_result.py" \
      --map "$MAP" --report "$REPORT_JSON" --out "$PATROL_PNG" \
      --json-out "$PATROL_SUMMARY_JSON" --md-out "$PATROL_SUMMARY_MD" \
      --require-pass \
      >> "$LOG" 2>&1; then
      echo "  -> patrol PNG render/validation failed" | tee -a "$LOG"
      return 1
    fi
  else
    echo "[$(date -Iseconds)] WARN: report=$REPORT_JSON or map=$MAP missing; skip patrol PNG" | tee -a "$LOG"
    return 1
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
    local IGNORE_EVAL_PREFIX="$PKG/outputs/recognition/${NAME}_recognition_eval_ignore_table_sofa"
    local EVAL_SUMMARY_JSON="$PKG/outputs/recognition/${NAME}_recognition_eval_summary.json"
    local EVAL_SUMMARY_MD="$PKG/outputs/recognition/${NAME}_recognition_eval_summary.md"
    echo "[$(date -Iseconds)] evaluating vs world (ignore Table/Sofa) -> ${IGNORE_EVAL_PREFIX}.{md,json,csv,png}" | tee -a "$LOG"
    python3 "$PKG/scripts/evaluate_recognition_vs_world.py" \
      --wbt "$WBT" --map "$MAP" --db "$DB" \
      --out-prefix "$IGNORE_EVAL_PREFIX" \
      --min-existence 0.5 --min-hits 2 --match-distance 1.0 \
      --ignore-type Table --ignore-type Sofa \
      >> "$LOG" 2>&1 || echo "  -> ignore Table/Sofa eval failed" | tee -a "$LOG"
    if [[ -f "${EVAL_PREFIX}.json" && -f "${IGNORE_EVAL_PREFIX}.json" ]]; then
      echo "[$(date -Iseconds)] summarizing recognition evals -> ${NAME}_recognition_eval_summary.{json,md}" | tee -a "$LOG"
      if ! python3 "$PKG/scripts/summarize_recognition_eval.py" \
        "${EVAL_PREFIX}.json" "${IGNORE_EVAL_PREFIX}.json" \
        --json-out "$EVAL_SUMMARY_JSON" --md-out "$EVAL_SUMMARY_MD" \
        --min-best-f1 0.70 \
        --require-best-label ignore_table_sofa \
        --require-pass \
        >> "$LOG" 2>&1; then
        echo "  -> recognition eval summary failed" | tee -a "$LOG"
        return 1
      fi
    else
      echo "  -> recognition eval JSONs missing; cannot summarize" | tee -a "$LOG"
      return 1
    fi
  fi
  # 認識巡回の reached/missed PNG も生成 (ユーザー指示: 巡回タスクで PNG 結果出力)
  local RECOG_PATROL_JSON="${RECOG_PATROL_PREFIX}.json"
  local RECOG_PATROL_PNG="$PKG/outputs/recognition/${NAME}_recognition_patrol_result.png"
  local RECOG_PATROL_SUMMARY_JSON="$PKG/outputs/recognition/${NAME}_recognition_patrol_result.json"
  local RECOG_PATROL_SUMMARY_MD="$PKG/outputs/recognition/${NAME}_recognition_patrol_result.md"
  if [[ -f "$RECOG_PATROL_JSON" && -f "$MAP" ]]; then
    echo "[$(date -Iseconds)] rendering recognition patrol PNG -> $RECOG_PATROL_PNG" | tee -a "$LOG"
    python3 "$PKG/scripts/visualize_patrol_result.py" \
      --map "$MAP" --report "$RECOG_PATROL_JSON" --out "$RECOG_PATROL_PNG" \
      --json-out "$RECOG_PATROL_SUMMARY_JSON" --md-out "$RECOG_PATROL_SUMMARY_MD" \
      >> "$LOG" 2>&1 || echo "  -> recog patrol PNG failed" | tee -a "$LOG"
  fi
}

run_signal_stats_summary() {
  local LOG="$LOGDIR/traffic_light_summary.log"
  local STATS="$PKG/outputs/traffic_light_recognition/city_traffic_stats.json"
  local SUMMARY_JSON="$PKG/outputs/traffic_light_recognition/city_traffic_stats_summary.json"
  local SUMMARY_MD="$PKG/outputs/traffic_light_recognition/city_traffic_stats.md"
  echo "[$(date -Iseconds)] TASK 4b traffic light stats summary" | tee "$LOG"
  if [[ ! -f "$STATS" ]]; then
    echo "  -> traffic light raw stats missing: $STATS" | tee -a "$LOG"
    return 1
  fi
  if ! python3 "$PKG/scripts/summarize_traffic_light_stats.py" \
    --stats "$STATS" \
    --json-out "$SUMMARY_JSON" \
    --md-out "$SUMMARY_MD" \
    --max-unique-signal-ids 8 \
    --min-top-signal-ratio 0.25 \
    --require-pass \
    >> "$LOG" 2>&1; then
    echo "  -> traffic light stats summary failed" | tee -a "$LOG"
    return 1
  fi
}

run_color() {
  local NAME=$1
  local WORLD=$2
  local LOG="$LOGDIR/color_${NAME}.log"
  local CALIB_JSON="$PKG/outputs/extrinsic_calibration/calib.json"
  echo "[$(date -Iseconds)] TASK 5 colorized $WORLD" | tee "$LOG"
  if [[ ! -f "$CALIB_JSON" ]]; then
    echo "  -> calibration JSON missing: $CALIB_JSON" | tee -a "$LOG"
    return 1
  fi
  ros2 launch susumu_object_perception webots_simulation.launch.py \
    world:="$WORLD" nav:=False rviz:=False \
    omni_perception:=True colored_slam:=True mode:=realtime \
    perception:=False image_recognition:=False \
    omni_calibration_json:="$CALIB_JSON" strict_omni_calibration_json:=True \
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
  else
    echo "  -> no new colorized_map_*.ply was saved" | tee -a "$LOG"
    return 1
  fi
  # === ユーザー指示「評価できることは必ず PNG 生成」 への対応 (iter31) ===
  # 採用 PLY を check_colorized_cloud.py で可視化 (XY top-view + XZ side-view)。
  # ブレ・壁の二重化・床下散乱を一目確認できる成果物として outputs/ に置く。
  # JSON/Markdown も残し、占有セル数や床下点率を後続 run と比較できるようにする。
  local FINAL_PLY="$PKG/outputs/colorized_pointcloud/colorized_pointcloud_${NAME}_apriltag_calib_final.ply"
  local CHECK_PNG="$PKG/outputs/colorized_pointcloud/colorized_pointcloud_${NAME}_apriltag_calib_final_check.png"
  local CHECK_JSON="$PKG/outputs/colorized_pointcloud/colorized_pointcloud_${NAME}_apriltag_calib_final_check.json"
  local CHECK_MD="$PKG/outputs/colorized_pointcloud/colorized_pointcloud_${NAME}_apriltag_calib_final_check.md"
  local TRUE_X=0.0
  local TRUE_Y=0.0
  case "$NAME" in
    indoor)
      TRUE_X=5.0
      TRUE_Y=10.0
      ;;
    break_room|breakroom)
      TRUE_X=12.86
      TRUE_Y=7.70
      ;;
  esac
  if [[ -f "$FINAL_PLY" ]]; then
    echo "[$(date -Iseconds)] rendering colorized check PNG -> $CHECK_PNG" | tee -a "$LOG"
    if ! python3 "$PKG/scripts/check_colorized_cloud.py" "$FINAL_PLY" \
      --out "$CHECK_PNG" --json-out "$CHECK_JSON" --md-out "$CHECK_MD" \
      --true-x "$TRUE_X" --true-y "$TRUE_Y" \
      --require-pass \
      >> "$LOG" 2>&1; then
      echo "  -> colorized PNG/check render failed" | tee -a "$LOG"
      return 1
    fi
  fi

  # 全採用 PLY のファイル形式・点数・RGB property 健全性を機械可読 summary に残す。
  # 各 world の run 後に再実行しても軽く、最後の run では3本分が揃う。
  local COLOR_SUMMARY_JSON="$PKG/outputs/colorized_pointcloud/colorized_pointcloud_quality_summary.json"
  local COLOR_SUMMARY_MD="$PKG/outputs/colorized_pointcloud/colorized_pointcloud_quality_summary.md"
  local COLOR_PLY_ARGS=()
  for PLY in \
    "$PKG/outputs/colorized_pointcloud/colorized_pointcloud_indoor_apriltag_calib_final.ply" \
    "$PKG/outputs/colorized_pointcloud/colorized_pointcloud_indoor_goal_run_final.ply" \
    "$PKG/outputs/colorized_pointcloud/colorized_pointcloud_breakroom_apriltag_calib_final.ply"; do
    [[ -f "$PLY" ]] && COLOR_PLY_ARGS+=(--ply "$PLY")
  done
  if (( ${#COLOR_PLY_ARGS[@]} > 0 )); then
    echo "[$(date -Iseconds)] writing colorized quality summary" | tee -a "$LOG"
    if ! python3 "$PKG/scripts/validate_colorized_pointcloud_quality.py" \
      "${COLOR_PLY_ARGS[@]}" --min-colored-ratio 0.95 \
      --json-out "$COLOR_SUMMARY_JSON" --md-out "$COLOR_SUMMARY_MD" \
      >> "$LOG" 2>&1; then
      echo "  -> colorized quality summary failed" | tee -a "$LOG"
      return 1
    fi
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
  # calib 結果サマリー PNG を生成 (iter33 追加、 ユーザー指示「PNG 必須」 対応)。
  # bar chart で真値 vs 推定値 + テーブルで数値 (RMS/RPY/quaternion) を可視化。
  local CALIB_JSON="$PKG/outputs/extrinsic_calibration/calib.json"
  local CALIB_PNG="$PKG/outputs/extrinsic_calibration/calib_summary.png"
  local CALIB_SUMMARY_JSON="$PKG/outputs/extrinsic_calibration/calib_summary.json"
  local CALIB_SUMMARY_MD="$PKG/outputs/extrinsic_calibration/calib_summary.md"
  if [[ -f "$CALIB_JSON" ]]; then
    echo "[$(date -Iseconds)] rendering calib summary PNG -> $CALIB_PNG" | tee -a "$LOG"
    if ! python3 "$PKG/scripts/visualize_calib_result.py" \
      --calib "$CALIB_JSON" --out "$CALIB_PNG" \
      --json-out "$CALIB_SUMMARY_JSON" --md-out "$CALIB_SUMMARY_MD" \
      --require-pass \
      >> "$LOG" 2>&1; then
      echo "  -> calib summary render/validation failed" | tee -a "$LOG"
      return 1
    fi
  else
    echo "  -> calibration JSON missing: $CALIB_JSON" | tee -a "$LOG"
    return 1
  fi
}

START=${1:-mapping}
SKIP_MAPPING=${SKIP_MAPPING:-0}

run_vs_world() {
  local NAME=$1
  local WBT="$PKG/webots_worlds/${NAME}.wbt"
  local YAML="$PKG/outputs/mapping_indoor/${NAME}.yaml"
  local PNG="$PKG/outputs/mapping_indoor/${NAME}_vs_world.png"
  local JSON="$PKG/outputs/mapping_indoor/${NAME}_vs_world.json"
  local CSV="$PKG/outputs/mapping_indoor/${NAME}_vs_world.csv"
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
  if ! python3 "$PKG/scripts/check_map_vs_world.py" \
    --wbt "$WBT" --map "$YAML" --out "$PNG" --report "$JSON" \
    --object-report "$CSV" \
    --min-near-ratio-inside 0.90 \
    --min-wall-near-ratio-inside 0.85 \
    --min-obstacle-near-ratio-inside 0.75 \
    --require-pass \
    >> "$LOG" 2>&1; then
    echo "  -> vs_world validation failed" | tee -a "$LOG"
    return 1
  fi
  echo "  -> $PNG / $JSON / $CSV" | tee -a "$LOG"
}

run_mapping_quality_summary() {
  local LOG="$LOGDIR/mapping_quality_summary.log"
  local MAP_ARGS=()
  for NAME in indoor break_room cafe; do
    local YAML="$PKG/outputs/mapping_indoor/${NAME}.yaml"
    [[ -f "$YAML" ]] && MAP_ARGS+=("$YAML")
  done
  if (( ${#MAP_ARGS[@]} == 0 )); then
    echo "[$(date -Iseconds)] mapping quality summary skip: no maps" | tee "$LOG"
    return
  fi
  echo "[$(date -Iseconds)] mapping quality summary" | tee "$LOG"
  if ! python3 "$PKG/scripts/eval_map_quality.py" "${MAP_ARGS[@]}" \
    --json-out "$PKG/outputs/mapping_indoor/mapping_indoor_quality_summary.json" \
    --md-out "$PKG/outputs/mapping_indoor/mapping_indoor_quality_summary.md" \
    >> "$LOG" 2>&1; then
    echo "  -> mapping quality summary failed" | tee -a "$LOG"
    return 1
  fi
  if ! python3 "$PKG/scripts/validate_map_assets.py" "${MAP_ARGS[@]}" \
    --json-out "$PKG/outputs/mapping_indoor/mapping_indoor_assets_summary.json" \
    --md-out "$PKG/outputs/mapping_indoor/mapping_indoor_assets_summary.md" \
    >> "$LOG" 2>&1; then
    echo "  -> mapping assets summary failed" | tee -a "$LOG"
    return 1
  fi
}

run_mapping_outdoor_contract_summary() {
  local LOG="$LOGDIR/mapping_outdoor_contracts.log"
  local MAP_ARGS=()
  for NAME in village_square_trimmed village_park_trimmed; do
    local YAML="$PKG/outputs/mapping_outdoor/${NAME}_gt.yaml"
    [[ -f "$YAML" ]] && MAP_ARGS+=("$YAML")
  done
  if (( ${#MAP_ARGS[@]} != 2 )); then
    echo "[$(date -Iseconds)] ERROR: mapping_outdoor gt maps missing; found ${#MAP_ARGS[@]}/2" | tee "$LOG"
    return 1
  fi
  echo "[$(date -Iseconds)] mapping_outdoor asset summary" | tee "$LOG"
  if ! python3 "$PKG/scripts/validate_map_assets.py" "${MAP_ARGS[@]}" \
    --json-out "$PKG/outputs/mapping_outdoor/mapping_outdoor_assets_summary.json" \
    --md-out "$PKG/outputs/mapping_outdoor/mapping_outdoor_assets_summary.md" \
    >> "$LOG" 2>&1; then
    echo "  -> mapping_outdoor assets summary failed" | tee -a "$LOG"
    return 1
  fi
}

case "$START" in mapping|all)
  if [[ "$SKIP_MAPPING" == "1" ]]; then
    echo "[$(date -Iseconds)] TASK 1 SKIP_MAPPING=1; 既存 outputs/mapping_indoor/ を採用版として使う"
    ls -la "$PKG/outputs/mapping_indoor/"
    # SKIP でも vs_world は必ず最新化する。launch 統合と同じ仕組みで品質確認画像を残す。
    for NAME in indoor break_room cafe; do
      run_vs_world "$NAME"
    done
    run_mapping_quality_summary || exit 1
    run_mapping_outdoor_contract_summary || exit 1
  else
    run_mapping indoor.wbt indoor || exit 1
    run_mapping break_room.wbt break_room || exit 1
    # launch 内で frontier_explore_node が自動生成するが、save_map が失敗した場合の
    # フォールバックとしてここでも呼ぶ（冪等）。cafe は wbt 無しなので関数側で skip。
    for NAME in indoor break_room cafe; do
      run_vs_world "$NAME"
    done
    run_mapping_quality_summary || exit 1
    run_mapping_outdoor_contract_summary || exit 1
  fi
;; esac

case "$START" in mapping|wp|all)
  WP_MISSING=()
  for NAME in indoor break_room cafe; do
    YAML="$PKG/outputs/mapping_indoor/$NAME.yaml"
    PGM="$PKG/outputs/mapping_indoor/$NAME.pgm"
    if [[ -f "$YAML" && -f "$PGM" ]]; then
      run_wp "$NAME" "$YAML" || exit 1
    else
      WP_MISSING+=("$NAME")
    fi
  done
  if (( ${#WP_MISSING[@]} > 0 )); then
    echo "[$(date -Iseconds)] ERROR: waypoint input maps missing/incomplete: ${WP_MISSING[*]}"
    exit 1
  fi
;; esac

case "$START" in mapping|wp|patrol|all)
  run_patrol indoor indoor.wbt indoor_waypoints.yaml
;; esac

case "$START" in mapping|wp|patrol|recog|all)
  run_recog indoor indoor.wbt indoor_waypoints.yaml
  run_signal_stats_summary || exit 1
;; esac

case "$START" in mapping|wp|patrol|recog|color|all)
  run_color indoor indoor.wbt || exit 1
;; esac

case "$START" in mapping|wp|patrol|recog|color|calib|all)
  run_calib
;; esac

echo "===== ALL DONE ====="
python3 "$PKG/scripts/validate_contracts.py" || exit 1
ls -la "$PKG/outputs/"*/
