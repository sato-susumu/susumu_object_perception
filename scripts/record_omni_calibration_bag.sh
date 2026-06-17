#!/usr/bin/env bash
set -euo pipefail

out_dir="${1:-$HOME/ros2_ws/omni_calibration_bags/$(date +%Y%m%d_%H%M%S)}"
mkdir -p "$(dirname "$out_dir")"

ros2 bag record \
  -o "$out_dir" \
  --compression-mode file \
  --compression-format zstd \
  /omni_camera/image_raw/compressed \
  /omni_camera/image_raw/camera_info \
  /omni_camera/equirect/camera_info \
  /velodyne_points/point_cloud \
  /velodyne_points/point_cloud_intensity \
  /perception/colorized_points \
  /tf \
  /tf_static \
  /clock
