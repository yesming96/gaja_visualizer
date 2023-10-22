/**
 * @file test_subscriber.cpp
 * @author Eunmin Hwang (yesming.em@gmail.com)
 * @brief
 * @date 2023-10-17
 *
 * @copyright Copyright (c) 2023 Eunmin Hwang all rights reserved.
 *
 */

#include <memory>

#include "geometry_msgs/msg/polygon_stamped.hpp"
// #include <geometry_msgs/msg/polygon_stamped.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
using std::placeholders::_1;

class MinimalSubscriber : public rclcpp::Node {
public:
  MinimalSubscriber() : Node("gaja_visualizer") {
    subscription_ = this->create_subscription<std_msgs::msg::String>(
        "topic", 10, std::bind(&MinimalSubscriber::topic_callback, this, _1));
    sub_ego_vehicle_ = this->create_subscription<geometry_msgs::msg::PolygonStamped>(
        "ego_vehicle", 10, std::bind(&MinimalSubscriber::ego_vehicle_callback, this, _1));
  }

private:
  void topic_callback(const std_msgs::msg::String& msg) const {
    RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
  }
  void ego_vehicle_callback(const geometry_msgs::msg::PolygonStamped& msg) const {
    RCLCPP_INFO(this->get_logger(), "Ego vehicle: '%d'", msg.polygon.points.size());
  }
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
  rclcpp::Subscription<geometry_msgs::msg::PolygonStamped>::SharedPtr sub_ego_vehicle_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalSubscriber>());
  rclcpp::shutdown();
  return 0;
}