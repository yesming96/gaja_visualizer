/**
 * @file test_publisher.cpp
 * @author Eunmin Hwang (yesming.em@gmail.com)
 * @brief
 * @date 2023-10-17
 *
 * @copyright Copyright (c) 2023 Eunmin Hwang all rights reserved.
 *
 */

#include <chrono>
#include <functional>
#include <memory>
#include <string>

// #include "geometry_msgs/msg/polygon_stamped.hpp"
#include <geometry_msgs/msg/polygon_stamped.hpp>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

using namespace std::chrono_literals;

/* This example creates a subclass of Node and uses std::bind() to register a
 * member function as a callback from the timer. */

class MinimalPublisher : public rclcpp::Node {
public:
  MinimalPublisher() : Node("gaja_visualizer"), count_(0) {
    publisher_ = this->create_publisher<std_msgs::msg::String>("topic", 1);
    timer_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::timer_callback, this));
    pub_ego_vehicle_ = this->create_publisher<geometry_msgs::msg::PolygonStamped>("ego_vehicle", 1);
    timer_pub_ = this->create_wall_timer(500ms, std::bind(&MinimalPublisher::ego_vehicle_callback, this));
    ;
  }

private:
  void timer_callback() {
    auto message = std_msgs::msg::String();
    message.data = "Hello, world! " + std::to_string(count_++);
    RCLCPP_INFO(this->get_logger(), "Publishing: '%s'", message.data.c_str());
    publisher_->publish(message);
  }
  void ego_vehicle_callback() {
    auto msg = geometry_msgs::msg::PolygonStamped();
    msg.header.frame_id = "map";
    auto size = msg.polygon.points.size();
    for (size_t i = 0; i < 5; ++i) {
      geometry_msgs::msg::Point32 temp;
      temp.x = i;
      temp.y = i;
      msg.polygon.points.push_back(temp);
    }
    RCLCPP_INFO(this->get_logger(), "Publishing: '%d'", msg.polygon.points.size());

    pub_ego_vehicle_->publish(msg);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::TimerBase::SharedPtr timer_pub_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;
  rclcpp::Publisher<geometry_msgs::msg::PolygonStamped>::SharedPtr pub_ego_vehicle_;
  size_t count_;
};

int main(int argc, char* argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MinimalPublisher>());
  rclcpp::shutdown();
  return 0;
}