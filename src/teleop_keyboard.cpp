/**
 * @file teleop_keyboard.cpp
 * @author Eunmin Hwang (yesming.em@gmail.com)
 * @brief
 * @date 2023-10-18
 *
 * @copyright Copyright (c) 2023 Eunmin Hwang all rights reserved.
 *
 */
#include <stdio.h>
#include <termios.h>
#include <unistd.h>

#include <map>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

// Map for movement keys
std::map<char, std::vector<float>> moveBindings{
    {'i', {1, 0, 0, 0}},  {'j', {0, 0, 0, 1}}, {'l', {0, 0, 0, -1}},
    {'k', {0, 0, 0, 0}},  {'A', {1, 0, 0, 0}}, {'D', {0, 0, 0, 1}},
    {'C', {0, 0, 0, -1}}, {'B', {0, 0, 0, 0}}};

// Reminder message
const char* msg = R"(
Reading from the keyboard and Publishing to Twist!
---------------------------
Moving around:
        i
   j    k    l
---------------------------
Simple Teleoperation with arrow keys
          ⇧
        ⇦   ⇨
          ⇩

          A
        D   C
          B
---------------------------
s/S : stop

NOTE : Increasing or Decreasing will take affect live on the moving robot.
      Consider Stopping the robot before changing it.
CTRL-C to quit
THIS IS A EXACT REPLICA OF https://github.com/ros-teleop/teleop_twist_keyboard 
WITH SOME ADD-ONS BUT IMPLEMENTED WITH C++ and ROS2-foxy.

CHANGES
- The msg includes linear velocity and steering angle.
- The range of linear velocity is [-20, 100] km/h (-5.5 to 27.8 m/s)
- The range of steering angle is [-0.785, 0.785] rad (+-45 deg) 
)";

// Init variables
float linear_vel{0.0};
float steering_angle{0.0};

char key(' ');

// For non-blocking keyboard inputs
int getch(void) {
  int ch;
  struct termios oldt;
  struct termios newt;

  // Store old settings, and copy to new settings
  tcgetattr(STDIN_FILENO, &oldt);
  newt = oldt;

  // Make required changes and apply the settings
  newt.c_lflag &= ~(ICANON | ECHO);
  newt.c_iflag |= IGNBRK;
  newt.c_iflag &= ~(INLCR | ICRNL | IXON | IXOFF);
  newt.c_lflag &= ~(ICANON | ECHO | ECHOK | ECHOE | ECHONL | ISIG | IEXTEN);
  newt.c_cc[VMIN] = 1;
  newt.c_cc[VTIME] = 0;
  tcsetattr(fileno(stdin), TCSANOW, &newt);

  // Get the current character
  ch = getchar();

  // Reapply old settings
  tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

  return ch;
}

int main(int argc, char** argv) {
  // node init
  rclcpp::init(argc, argv);
  auto node = rclcpp::Node::make_shared("teleop");
  // define publisher
  auto _pub =
      node->create_publisher<geometry_msgs::msg::Twist>("/cmd_control", 10);

  geometry_msgs::msg::Twist twist;
  printf("%s", msg);
  printf("speed is %f and steering angle is %f", linear_vel, steering_angle);

  while (rclcpp::ok()) {
    // get the pressed key
    // excute every 10ms
    key = getch();

    // increase/decrease linear velocity or steering angle
    if (key == 'A' || key == 'i') {
      if (linear_vel >= 27.8) {
        printf("Current: speed %f km/h \t [MAX] steering angle %f deg",
               linear_vel * 1000.0 / 3600.0, steering_angle * 57.2958);
      } else {
        // 0.35 = 1km/h
        linear_vel = linear_vel + 0.35;
        printf("Current: speed %f km/h \t steering angle %f deg",
               linear_vel * 1000.0 / 3600.0, steering_angle * 57.2958);
      }
    } else if (key == 'B' || key == 'k') {
      if (linear_vel <= -5.5) {
        printf("Current: speed %f km/h \t [MAX] steering angle %f deg",
               linear_vel * 1000.0 / 3600.0, steering_angle * 57.2958);
      } else {
        linear_vel = linear_vel - 0.35;
        printf("Current: speed %f km/h \t steering angle %f deg",
               linear_vel * 1000.0 / 3600.0, steering_angle * 57.2958);
      }
    } else if (key == 'J' || key == 'd') {
      // 0.35 = 22.5 deg
      if (steering_angle <= -0.785) {
        printf("Current: speed %f km/h \t [MAX] steering angle %f deg",
               linear_vel * 1000.0 / 3600.0, steering_angle * 57.2958);
      } else {
        steering_angle = steering_angle - 0.03;
        printf("Current: speed %f km/h \t steering angle %f deg",
               linear_vel * 1000.0 / 3600.0, steering_angle * 57.2958);
      }
    } else if (key == 'C' || key == 'l') {
      if (steering_angle >= 0.785) {
        printf("Current: speed %f km/h \t [MAX] steering angle %f deg",
               linear_vel * 1000.0 / 3600.0, steering_angle * 57.2958);
      } else {
        steering_angle = steering_angle + 0.03;
        printf("Current: speed %f km/h \t steering angle %f deg",
               linear_vel * 1000.0 / 3600.0, steering_angle * 57.2958);
      }
    } else {
      if (key == 's' || key == 'S') {
        steering_angle = 0.0;
        linear_vel = 0.0;
        printf("\tRobot Stopped..!! \n");
        printf("Current: speed %f km/h \t steering angle %f deg",
               linear_vel * 1000.0 / 3600.0, steering_angle * 57.2958);
      }
      // If ctrl-C (^C) was pressed, terminate the program
      else if (key == '\x03') {
        break;
      } else {
        // pass
      }
    }

    // Update the Twist message
    twist.linear.x = linear_vel;
    twist.linear.y = 0;
    twist.linear.z = 0;

    twist.angular.x = 0;
    twist.angular.y = 0;
    twist.angular.z = steering_angle;

    _pub->publish(twist);
    rclcpp::spin_some(node);
    rclcpp::sleep_for(std::chrono::milliseconds(10));
  }
  return 0;
}