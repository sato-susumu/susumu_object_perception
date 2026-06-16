#include "susumu_object_perception/predicted_costmap_layer.hpp"

#include <algorithm>

#include "nav2_costmap_2d/costmap_math.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace susumu_object_perception
{

void PredictedCostmapLayer::onInitialize()
{
  auto node = node_.lock();
  if (!node) {
    throw std::runtime_error{"PredictedCostmapLayer: failed to lock node"};
  }

  declareParameter("enabled", rclcpp::ParameterValue(true));
  declareParameter("topic", rclcpp::ParameterValue(std::string("/perception/predicted_costmap")));
  declareParameter("occupied_threshold", rclcpp::ParameterValue(50));

  node->get_parameter(name_ + "." + "enabled", enabled_);
  node->get_parameter(name_ + "." + "topic", topic_);
  node->get_parameter(name_ + "." + "occupied_threshold", occupied_threshold_);

  current_ = true;

  // 予測 OccupancyGrid を購読（最新フレームだけ保持。毎フレーム作り直されるので
  // latch 不要。reliability は default = reliable）。
  rclcpp::QoS qos(1);
  sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
    topic_, qos,
    std::bind(&PredictedCostmapLayer::gridCallback, this, std::placeholders::_1));

  RCLCPP_INFO(
    node->get_logger(),
    "PredictedCostmapLayer initialized. subscribing %s (occupied_threshold=%d)",
    topic_.c_str(), occupied_threshold_);
}

void PredictedCostmapLayer::gridCallback(
  const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
  std::lock_guard<std::mutex> lock(mutex_);
  latest_ = msg;
}

void PredictedCostmapLayer::updateBounds(
  double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
  double * min_x, double * min_y, double * max_x, double * max_y)
{
  if (!enabled_) {
    return;
  }
  std::lock_guard<std::mutex> lock(mutex_);
  if (!latest_) {
    return;
  }
  // 予測格子がカバーする範囲を更新領域に含める（map 全域を毎回見るのは重いので
  // 占有セルが来た範囲だけ広げてもよいが、予測格子は map と同サイズで占有は疎なので
  // 簡潔に格子全体の bbox を含める）。
  const auto & info = latest_->info;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;
  const double w = info.width * info.resolution;
  const double h = info.height * info.resolution;
  *min_x = std::min(*min_x, ox);
  *min_y = std::min(*min_y, oy);
  *max_x = std::max(*max_x, ox + w);
  *max_y = std::max(*max_y, oy + h);
}

void PredictedCostmapLayer::updateCosts(
  nav2_costmap_2d::Costmap2D & master_grid,
  int min_i, int min_j, int max_i, int max_j)
{
  if (!enabled_) {
    return;
  }
  nav_msgs::msg::OccupancyGrid::SharedPtr grid;
  {
    std::lock_guard<std::mutex> lock(mutex_);
    grid = latest_;
  }
  if (!grid) {
    return;
  }

  const auto & info = grid->info;
  const double res = info.resolution;
  const double ox = info.origin.position.x;
  const double oy = info.origin.position.y;
  const unsigned int gw = info.width;
  const unsigned int gh = info.height;

  // master costmap の更新ウィンドウ内の各セルについて、対応する予測格子セルが占有なら
  // **max 合成**でコストを乗せる（他層を上書きしない。だから壁が消えない）。
  // 蓄積しないのは、毎フレーム latest_ が「予測以外は free」の新しい格子に置き換わるため
  // （古い予測セルは今回の格子で 0 になり、max でも乗らない）。
  for (int j = min_j; j < max_j; ++j) {
    for (int i = min_i; i < max_i; ++i) {
      // master セル (i,j) → world 座標 → 予測格子セル。
      double wx, wy;
      master_grid.mapToWorld(
        static_cast<unsigned int>(i), static_cast<unsigned int>(j), wx, wy);
      const int gx = static_cast<int>((wx - ox) / res);
      const int gy = static_cast<int>((wy - oy) / res);
      if (gx < 0 || gy < 0 ||
        gx >= static_cast<int>(gw) || gy >= static_cast<int>(gh))
      {
        continue;
      }
      const int8_t v = grid->data[gy * gw + gx];
      if (v >= occupied_threshold_) {
        const unsigned char old = master_grid.getCost(i, j);
        if (cost_value_ > old) {
          master_grid.setCost(i, j, cost_value_);
        }
      }
    }
  }
}

}  // namespace susumu_object_perception

PLUGINLIB_EXPORT_CLASS(susumu_object_perception::PredictedCostmapLayer, nav2_costmap_2d::Layer)
