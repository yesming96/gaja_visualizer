/**
 * @file visual_publisher.cpp
 * @author Eunmin Hwang (yesming.em@gmail.com)
 * @brief
 * @date 2024-02-01
 *
 * @copyright Copyright (c) 2023 Eunmin Hwang all rights reserved.
 *
 */

#include "visual_publisher.hpp"

namespace gaja_visualizer {
VisualPublisher::VisualPublisher() : Node("gaja_visualizer") {
  if (not init()) {
    spdlog::error("Failed to Initialize");
  };
}

bool VisualPublisher::init() {
  // pub ego vehicle
  pub_ego_vehicle_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("ego_vehicle", 1);
  timer_ego_vehicle_ = this->create_wall_timer(10ms, std::bind(&VisualPublisher::ego_vehicle_callback, this));

  // pub other vehicles

  // pub road vehicle
}

void VisualPublisher::ego_vehicle_callback() {
  auto msg = geometry_msgs::msg::PolygonStamped();
  msg.header.frame_id = "map";
  auto size = msg.polygon.points.size();
  auto vehicle_pose = gaja::StateBoard::get_instance().get_pose();
  for (size_t i = 0; i < 5; ++i) {
    geometry_msgs::msg::Point32 temp;
    temp.x = i;
    temp.y = i;
    msg.polygon.points.push_back(temp);
  }
  RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", msg.polygon.points.size());

  pub_ego_vehicle_->publish(msg);
}
}  // namespace gaja_visualizer