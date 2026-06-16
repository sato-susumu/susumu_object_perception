// 予測コストマップ層（perception 連携）。
//
// prediction_node が出す予測 OccupancyGrid `/perception/predicted_costmap`（人が
// これから行く先を毎フレーム作り直した格子、map frame）を購読し、costmap に
// **max 合成**で焼く自作 costmap_2d::Layer プラグイン。
//
// なぜ自作 C++ 層か（標準層では不可だった経緯）:
//   - ObstacleLayer / STVL（点群方式）: 古い予測が蓄積して costmap が埋まる
//     （raytrace clearing は観測線上しか消せない）。
//   - StaticLayer（OccupancyGrid 方式）: 後段の層が前段を**上書き**するので、
//     予測層（ほぼ全 free）が壁の static_layer を消してしまう（壁が消える）。
//   いずれも costmap を壊した。本層は「**毎フレーム最新の OccupancyGrid だけを読み、
//   占有セルだけを max 合成で乗せる**」ことで、他層を壊さず（max）かつ蓄積もしない
//   （毎フレーム最新で置換）を両立する。

#ifndef SUSUMU_SIM__PREDICTED_COSTMAP_LAYER_HPP_
#define SUSUMU_SIM__PREDICTED_COSTMAP_LAYER_HPP_

#include <memory>
#include <mutex>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav_msgs/msg/occupancy_grid.hpp"

namespace susumu_object_perception
{

class PredictedCostmapLayer : public nav2_costmap_2d::Layer
{
public:
  PredictedCostmapLayer() = default;

  void onInitialize() override;
  void updateBounds(
    double robot_x, double robot_y, double robot_yaw,
    double * min_x, double * min_y, double * max_x, double * max_y) override;
  void updateCosts(
    nav2_costmap_2d::Costmap2D & master_grid,
    int min_i, int min_j, int max_i, int max_j) override;

  void reset() override {}
  bool isClearable() override {return false;}

private:
  void gridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

  rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr sub_;
  nav_msgs::msg::OccupancyGrid::SharedPtr latest_;
  std::mutex mutex_;

  std::string topic_;
  int occupied_threshold_{50};  // OccupancyGrid 値がこれ以上のセルを障害物にする
  unsigned char cost_value_{254};  // 焼くコスト（254=LETHAL_OBSTACLE）
};

}  // namespace susumu_object_perception

#endif  // SUSUMU_SIM__PREDICTED_COSTMAP_LAYER_HPP_
