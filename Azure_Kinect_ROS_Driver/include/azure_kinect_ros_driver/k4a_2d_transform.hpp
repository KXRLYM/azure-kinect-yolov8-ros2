#pragma once

#include <rclcpp/rclcpp.hpp>
#include "azure_kinect_ros_driver/k4a_ros_device.h"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "yolov8_msgs/msg/point2_d.hpp"
#include "yolov8_msgs/msg/detection.hpp"
#include <k4a/k4a.h>
#include <chrono>

class K4A2dTransformNode : public rclcpp::Node
{
public:
    explicit K4A2dTransformNode(const std::shared_ptr<K4AROSDevice>& k4a_device);
    // void publishTransform(const std::shared_ptr<K4AROSDevice>& k4a_device);
    void detectionCallback(const yolov8_msgs::msg::DetectionArray::SharedPtr msg, 
    const std::shared_ptr<K4AROSDevice>& k4a_device, const k4a_calibration_t& calibration);

private:
    std::shared_ptr<K4AROSDevice> k4a_device_;
    rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr subscription_;
    rclcpp::Publisher<yolov8_msgs::msg::DetectionArray>::SharedPtr publisher_;
};