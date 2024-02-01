/**
 * @file main.cpp
 * @author Eunmin Hwang (yesming.em@gmail.com)
 * @brief
 * @date 2024-01-22
 *
 * @copyright Copyright (c) 2024 Eunmin Hwang all rights reserved.
 *
 */

#include <yaml-cpp/yaml.h>  // YAML

#include <memory>
#include <string>
#include <vector>

#include "../gaja/external/spdlog/spdlog.h"
#include "../gaja/include/base.hpp"
#include "rclcpp/executors/multi_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "visual_publisher.hpp"

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  auto executor = rclcpp::executors::MultiThreadedExecutor();
}