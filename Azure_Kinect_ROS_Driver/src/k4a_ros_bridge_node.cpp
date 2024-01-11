// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

// System headers
//
#include <sstream>

// Library headers
//
#include "rclcpp/rclcpp.hpp"
#include <k4a/k4a.h>

// Project headers
//
#include "azure_kinect_ros_driver/k4a_ros_device.h"
#include "azure_kinect_ros_driver/k4a_2d_transform.hpp"

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);

  // Create Node for handling info and error messages
  rclcpp::Node::SharedPtr node = rclcpp::Node::make_shared("k4a_bridge");

  // Setup the K4A device
  std::shared_ptr<K4AROSDevice> device(new K4AROSDevice);

  k4a_result_t result = device->startCameras();

  if (result != K4A_RESULT_SUCCEEDED)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),"Failed to start cameras");
    return -1;
  }

  result = device->startImu();
  if (result != K4A_RESULT_SUCCEEDED)
  {
    RCLCPP_ERROR_STREAM(node->get_logger(),"Failed to start IMU");
    return -2;
  }

  RCLCPP_INFO(node->get_logger(),"K4A Started");
  RCLCPP_INFO(node->get_logger(),"Transformer Node Started");
  auto transformer_node = std::make_shared<K4A2dTransformNode>(device);
  rclcpp::spin(transformer_node);
  if (result == K4A_RESULT_SUCCEEDED)
  {
    rclcpp::spin(node);

    RCLCPP_INFO(node->get_logger(),"ROS Exit Started");

  }

  device.reset();

  RCLCPP_INFO(node->get_logger(),"ROS Exit");

  rclcpp::shutdown();

  RCLCPP_INFO(node->get_logger(),"ROS Shutdown complete");

  RCLCPP_INFO(node->get_logger(),"Finished ros bridge main");
  return 0;
}