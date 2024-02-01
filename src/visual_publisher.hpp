/**
 * @file visual_publisher.hpp
 * @author Eunmin Hwang (yesming.em@gmail.com)
 * @brief
 * @date 2024-02-01
 *
 * @copyright Copyright (c) 2023 Eunmin Hwang all rights reserved.
 *
 */

#pragma once

#include <chrono>
#include <functional>
#include <geometry_msgs/msg/polygon_stamped.hpp>
#include <memory>
#include <string>

#include "../gaja/include/board/state_board.hpp"
#include "rclcpp/rclcpp.hpp"
#include "spdlog/spdlog.h"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */
namespace gaja_visualizer {

class VisualPublisher : public rclcpp::Node {
public:
  VisualPublisher();

private:
  bool init();
  void timer_callback();
  void ego_vehicle_callback();

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

  // pub ego vehicle
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_ego_vehicle_;
  rclcpp::TimerBase::SharedPtr timer_ego_vehicle_;
};
}  // namespace gaja_visualizer