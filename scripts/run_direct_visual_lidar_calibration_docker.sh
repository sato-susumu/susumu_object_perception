#!/usr/bin/env bash
set -euo pipefail

if [[ $# -lt 2 ]]; then
  echo "usage: $0 BAG_DIR PREPROCESSED_DIR [preprocess|initial_guess|calibrate|viewer]" >&2
  exit 2
fi

bag_dir="$(realpath "$1")"
preprocessed_dir="$(realpath -m "$2")"
step="${3:-preprocess}"
image="koide3/direct_visual_lidar_calibration:humble"

mkdir -p "$preprocessed_dir"

case "$step" in
  preprocess)
    docker run --rm \
      -v "$bag_dir:/tmp/input_bags" \
      -v "$preprocessed_dir:/tmp/preprocessed" \
      "$image" \
      ros2 run direct_visual_lidar_calibration preprocess \
        --image_topic /omni_camera/image_raw/compressed \
        --points_topic /velodyne_points/point_cloud_intensity \
        --camera_model equirectangular \
        --intensity_channel intensity \
        -d \
        /tmp/input_bags /tmp/preprocessed
    ;;
  initial_guess)
    docker run --rm --net host --gpus all \
      -e DISPLAY="${DISPLAY:-}" \
      -v "$HOME/.Xauthority:/root/.Xauthority:ro" \
      -v "$preprocessed_dir:/tmp/preprocessed" \
      "$image" \
      ros2 run direct_visual_lidar_calibration initial_guess_manual /tmp/preprocessed
    ;;
  calibrate)
    docker run --rm --net host --gpus all \
      -e DISPLAY="${DISPLAY:-}" \
      -v "$HOME/.Xauthority:/root/.Xauthority:ro" \
      -v "$preprocessed_dir:/tmp/preprocessed" \
      "$image" \
      ros2 run direct_visual_lidar_calibration calibrate /tmp/preprocessed
    ;;
  viewer)
    docker run --rm --net host --gpus all \
      -e DISPLAY="${DISPLAY:-}" \
      -v "$HOME/.Xauthority:/root/.Xauthority:ro" \
      -v "$preprocessed_dir:/tmp/preprocessed" \
      "$image" \
      ros2 run direct_visual_lidar_calibration viewer /tmp/preprocessed
    ;;
  *)
    echo "unknown step: $step" >&2
    exit 2
    ;;
esac
