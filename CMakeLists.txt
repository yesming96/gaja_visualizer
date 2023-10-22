cmake_minimum_required(VERSION 3.8)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
project(gaja_visualizer)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

#Build type
if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

# publisher
add_executable(talker src/test_publisher.cpp)
ament_target_dependencies(talker rclcpp std_msgs geometry_msgs)

# subscriber
add_executable(listener src/test_subscriber.cpp)
ament_target_dependencies(listener rclcpp std_msgs geometry_msgs)

# controller
add_executable(controller src/teleop_keyboard.cpp)
ament_target_dependencies(controller rclcpp std_msgs geometry_msgs)

install(TARGETS
  talker
  listener
  controller
  DESTINATION lib/${PROJECT_NAME}
  )

install (DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME} )

ament_package()

