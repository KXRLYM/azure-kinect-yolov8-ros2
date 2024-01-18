cmake_minimum_required(VERSION 3.8)
project(capture_pointcloud)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)

include_directories(
  ${rclcpp_INCLUDE_DIRS}
  ${sensor_msgs_INCLUDE_DIRS}
  ${pcl_conversions_INCLUDE_DIRS}
  ${pcl_ros_INCLUDE_DIRS}
)

add_executable(capture_pointcloud_node src/capture_pointcloud.cpp)
add_executable(publish_pointcloud_node src/publish_pointcloud.cpp)
ament_target_dependencies(capture_pointcloud_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)
ament_target_dependencies(publish_pointcloud_node
  rclcpp
  sensor_msgs
  pcl_conversions
  pcl_ros
)

install(TARGETS
  capture_pointcloud_node
  DESTINATION lib/${PROJECT_NAME}
)

install(TARGETS
  publish_pointcloud_node
  DESTINATION lib/${PROJECT_NAME})


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



ament_package()
