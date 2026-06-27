#!/usr/bin/env bash
# capture_overview.sh
#
# 1 つの world で mapping → waypoint 生成 → patrol+recognition をライブで回し、
# 各フェーズの途中経過 PNG を experiments/overview_capture/<日付>_<world>/frames/<phase>/
# に蓄積する。最後に render_overview_gif.py で docs/images/overview.gif を生成する。
#
# 使い方:
#   bash scripts/capture_overview.sh
#   bash scripts/capture_overview.sh indoor.wbt 2026-06-27_overview
#   SKIP_MAPPING=1 SKIP_RECOG=1 bash scripts/capture_overview.sh   # waypoint 段だけ作り直し
#
# 出力 GIF の置き場 (重要):
#   既定では experiments/overview_capture/<日付>_<label>/overview.gif に置く。
#   後日の試行錯誤で docs/images/overview.gif を勝手に上書きしないため。
#   docs/images/overview.gif に正式版として昇格させたいときだけ OUT_GIF を明示する:
#     OUT_GIF=docs/images/overview.gif bash scripts/capture_overview.sh
#
# 後日の再実行は同じコマンドでOK。出力ディレクトリが日付付きで分離されるので過去成果物を上書きしない。

set -euo pipefail

WORLD=${1:-indoor.wbt}
LABEL=${2:-$(date +%Y-%m-%d)_overview}

WORLD_NAME=${WORLD%.wbt}
PKG_DIR=$(cd "$(dirname "$0")/.." && pwd)
EXP_DIR="$PKG_DIR/experiments/overview_capture/${LABEL}"
FRAMES_DIR="$EXP_DIR/frames"
LOG_DIR="$EXP_DIR/logs"
DB_PATH="$HOME/.ros/object_memory_overview.sqlite3"

mkdir -p "$FRAMES_DIR/mapping" "$FRAMES_DIR/waypoints" \
         "$FRAMES_DIR/recognition" "$LOG_DIR"

echo "[overview] world=$WORLD label=$LABEL"
echo "[overview] frames    -> $FRAMES_DIR"
echo "[overview] logs      -> $LOG_DIR"

cleanup_ros() {
    echo "[overview] cleaning up ROS processes..."
    pkill -f 'capture_overview_frames.py' 2>/dev/null || true
    pkill -f 'webots-controller' 2>/dev/null || true
    pkill -f 'component_container' 2>/dev/null || true
    pkill -f 'webots --batch' 2>/dev/null || true
    pkill -f 'frontier_explore_node' 2>/dev/null || true
    pkill -f 'waypoint_nav_node' 2>/dev/null || true
    pkill -f 'object_classifier_node' 2>/dev/null || true
    pkill -f 'object_memory_node' 2>/dev/null || true
    pkill -f 'auto_patrol_node' 2>/dev/null || true
    pkill -f 'ros2 launch' 2>/dev/null || true
    sleep 3
    pkill -9 -f 'webots' 2>/dev/null || true
}

trap cleanup_ros EXIT

source /opt/ros/humble/setup.bash
source "$HOME/ros2_ws/install/local_setup.bash"
export TURTLEBOT3_MODEL=waffle

# ─────────────────────────────
# Phase A: mapping ライブ + /map スナップショット
# ─────────────────────────────
if [[ "${SKIP_MAPPING:-0}" != "1" ]]; then
    echo "[overview] === Phase A: mapping ==="
    rm -rf "$FRAMES_DIR/mapping"
    mkdir -p "$FRAMES_DIR/mapping"

    ros2 launch susumu_object_perception webots_indoor_mapping.launch.py \
        world:="$WORLD" mode:=realtime rviz:=False save_map:=True \
        > "$LOG_DIR/mapping_launch.log" 2>&1 &
    MAP_LAUNCH_PID=$!

    # /map が立ち上がるのを少し待つ
    sleep 20

    python3 "$PKG_DIR/scripts/capture_overview_frames.py" \
        --phase mapping \
        --out-dir "$FRAMES_DIR/mapping" \
        --period-sec 3.0 \
        --min-change-cells 400 \
        --max-frames 80 \
        > "$LOG_DIR/capture_mapping.log" 2>&1 &
    CAP_PID=$!

    # 完了待ち: ログに "Mapping done" or "frontier 完了" or "map saved" が出るまで
    # 最大 18 分待つ
    MAX_WAIT=1080
    waited=0
    while (( waited < MAX_WAIT )); do
        if grep -qE "Mapping done|frontier 完了|map saved|frontier exhausted|done_after_empty 達成" \
                "$LOG_DIR/mapping_launch.log" 2>/dev/null; then
            echo "[overview] mapping done detected"
            break
        fi
        if ! kill -0 $MAP_LAUNCH_PID 2>/dev/null; then
            echo "[overview] mapping launch died"
            break
        fi
        sleep 10
        waited=$((waited + 10))
    done

    sleep 4
    kill $CAP_PID 2>/dev/null || true
    kill $MAP_LAUNCH_PID 2>/dev/null || true
    sleep 3
    cleanup_ros

    n_map=$(ls "$FRAMES_DIR/mapping"/*.png 2>/dev/null | wc -l)
    echo "[overview] mapping frames captured: $n_map"
fi

# ─────────────────────────────
# Phase B: waypoint 生成過程フレーム
# ─────────────────────────────
if [[ "${SKIP_WAYPOINT:-0}" != "1" ]]; then
    echo "[overview] === Phase B: waypoint generation ==="
    rm -rf "$FRAMES_DIR/waypoints"
    mkdir -p "$FRAMES_DIR/waypoints"

    # mapping_indoor の最終 PGM/YAML から waypoint を生成 (上書きしないように一時 YAML)
    MAP_YAML="$PKG_DIR/outputs/mapping_indoor/${WORLD_NAME}.yaml"
    if [[ ! -f "$MAP_YAML" ]]; then
        echo "[overview] WARN: $MAP_YAML not found (mapping skipped?)"
    fi

    WP_YAML="$PKG_DIR/outputs/waypoint_generation/${WORLD_NAME}_waypoints.yaml"
    if [[ ! -f "$WP_YAML" ]]; then
        echo "[overview] generating waypoints..."
        ros2 run susumu_object_perception generate_waypoints.py \
            --map "$MAP_YAML" \
            --out "$WP_YAML" \
            --spacing 1.0 --clearance 0.5 \
            > "$LOG_DIR/generate_waypoints.log" 2>&1 || true
    fi

    if [[ -f "$WP_YAML" && -f "$MAP_YAML" ]]; then
        python3 "$PKG_DIR/scripts/render_waypoint_frames.py" \
            --map-yaml "$MAP_YAML" \
            --waypoints-yaml "$WP_YAML" \
            --out-dir "$FRAMES_DIR/waypoints" \
            --target-height 480 \
            --hold-final 6 \
            > "$LOG_DIR/render_waypoints.log" 2>&1
        n_wp=$(ls "$FRAMES_DIR/waypoints"/*.png 2>/dev/null | wc -l)
        echo "[overview] waypoint frames: $n_wp"
    else
        echo "[overview] SKIP Phase B (waypoint sources not ready)"
    fi
fi

# ─────────────────────────────
# Phase C: patrol + recognition ライブ
# ─────────────────────────────
if [[ "${SKIP_RECOG:-0}" != "1" ]]; then
    echo "[overview] === Phase C: recognition during patrol ==="
    rm -rf "$FRAMES_DIR/recognition"
    mkdir -p "$FRAMES_DIR/recognition"
    rm -f "$DB_PATH"

    ros2 launch susumu_object_perception webots_waypoint_nav.launch.py \
        world:="$WORLD" \
        waypoints:="${WORLD_NAME}_waypoints.yaml" \
        mode:=realtime rviz:=False loop:=False \
        slam:=False \
        map_file:="$PKG_DIR/outputs/mapping_indoor/${WORLD_NAME}.yaml" \
        perception:=True omni_perception:=True image_recognition:=True \
        > "$LOG_DIR/recog_launch.log" 2>&1 &
    REC_LAUNCH_PID=$!

    # ROS グラフ初期化を待つ
    sleep 25

    # object_memory DB を上書き先に切替
    DB_PATH_NODE="$HOME/.ros/object_memory.sqlite3"

    python3 "$PKG_DIR/scripts/capture_overview_frames.py" \
        --phase recognize \
        --out-dir "$FRAMES_DIR/recognition" \
        --db-path "$DB_PATH_NODE" \
        --map-yaml "$PKG_DIR/outputs/mapping_indoor/${WORLD_NAME}.yaml" \
        --waypoints-yaml "$PKG_DIR/outputs/waypoint_generation/${WORLD_NAME}_waypoints.yaml" \
        --period-sec 3.5 \
        --max-frames 200 \
        > "$LOG_DIR/capture_recognition.log" 2>&1 &
    CAP_PID=$!

    # 巡回完了 or 時間切れまで待つ
    MAX_WAIT=900
    waited=0
    while (( waited < MAX_WAIT )); do
        if grep -qE "patrol done|reached=[0-9]+/[0-9]+ missed|mission complete" \
                "$LOG_DIR/recog_launch.log" 2>/dev/null; then
            echo "[overview] patrol done detected"
            # 認識が落ち着くまで少し待つ
            sleep 15
            break
        fi
        if ! kill -0 $REC_LAUNCH_PID 2>/dev/null; then
            echo "[overview] recog launch died"
            break
        fi
        sleep 15
        waited=$((waited + 15))
    done

    kill $CAP_PID 2>/dev/null || true
    kill $REC_LAUNCH_PID 2>/dev/null || true
    sleep 3
    cleanup_ros

    n_rec=$(ls "$FRAMES_DIR/recognition"/*.png 2>/dev/null | wc -l)
    echo "[overview] recognition frames: $n_rec"
fi

# ─────────────────────────────
# Phase D: ラベル付き加工版を別フォルダに作る
# ─────────────────────────────
echo "[overview] === Phase D: prepare labeled frames ==="
LABELED_DIR="$EXP_DIR/frames_labeled"
ROTATE_MODE="${ROTATE_MODE:-auto}"  # auto / none / cw / ccw
rm -rf "$LABELED_DIR"
python3 "$PKG_DIR/scripts/prepare_overview_frames.py" \
    --frames-root "$FRAMES_DIR" \
    --out-root   "$LABELED_DIR" \
    --rotate     "$ROTATE_MODE" \
    | tee "$LOG_DIR/prepare_frames.log"

# ─────────────────────────────
# Phase E: GIF 合成
# ─────────────────────────────
echo "[overview] === Phase E: GIF render ==="
# 既定は実験フォルダ内。docs に昇格させたいときだけ OUT_GIF を明示で上書き。
OUT_GIF="${OUT_GIF:-$EXP_DIR/overview.gif}"
mkdir -p "$(dirname "$OUT_GIF")"
python3 "$PKG_DIR/scripts/render_overview_gif.py" \
    --frames-root "$LABELED_DIR" \
    --out "$OUT_GIF" \
    --target-width 1000 \
    --fps 10 \
    --max-per-phase 40 \
    --hold-last-frames 8 \
    --max-mb 10 \
    | tee "$LOG_DIR/render_gif.log"

ls -lh "$OUT_GIF"
echo "[overview] DONE -> $OUT_GIF"
