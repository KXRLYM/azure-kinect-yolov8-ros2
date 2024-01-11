#include "azure_kinect_ros_driver/k4a_2d_transform.hpp"
#include "azure_kinect_ros_driver/k4a_ros_device.h"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "yolov8_msgs/msg/detection.hpp"
#include "yolov8_msgs/msg/point2_d.hpp"
#include <k4a/k4a.hpp>
#include <rclcpp/rclcpp.hpp>
#include <chrono>

K4A2dTransformNode::K4A2dTransformNode(const std::shared_ptr<K4AROSDevice>& k4a_device) 
    : Node("k4a_2d_transform_node"), k4a_device_(k4a_device)
{
    k4a_calibration_t calibration;
    k4a_device_get_calibration(
        k4a_device->k4a_device_.handle(), K4A_DEPTH_MODE_NFOV_UNBINNED, K4A_COLOR_RESOLUTION_1536P, &calibration
    );

    // Use a lambda function to capture the context and call the member function
    auto callback_func = [this, k4a_device, calibration](const yolov8_msgs::msg::DetectionArray::SharedPtr msg) {
        detectionCallback(msg, k4a_device, calibration);
    };

    subscription_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
        "yolo/detections",
        10,
        callback_func
    );

    publisher_ = this->create_publisher<yolov8_msgs::msg::DetectionArray>("/detections_transformed", 10);
}

void K4A2dTransformNode::detectionCallback(
    const yolov8_msgs::msg::DetectionArray::SharedPtr msg,
    const std::shared_ptr<K4AROSDevice>& k4a_device, 
    const k4a_calibration_t& calibration) {
    // std::vector<yolov8_msgs::msg::Point2D> maskPoints;
    int valid(0);
    k4a_float2_t source;
    k4a_float2_t target;
    k4a::capture capture;
    k4a_device->k4a_device_.get_capture(&capture);

    yolov8_msgs::msg::DetectionArray::SharedPtr transformedArray = std::make_shared<yolov8_msgs::msg::DetectionArray>();
    transformedArray->header = msg->header;

    for (const auto& detection : msg->detections) {
        yolov8_msgs::msg::Detection::SharedPtr transformedDetection = std::make_shared<yolov8_msgs::msg::Detection>();
        transformedDetection->class_id = detection.class_id;
        transformedDetection->class_name = detection.class_name;
        transformedDetection->score = detection.score;
    
        for (const auto& point : detection.mask.data) {
            source.xy.x = point.x;
            source.xy.y = point.y;
            k4a_calibration_color_2d_to_depth_2d(
                &calibration, &source, capture.get_depth_image().handle(), &target, &valid
            );
            if (valid) {
                // std::cout << "Transformed Data: x = " << target.xy.x << ", y = " << target.xy.y << std::endl; 
                yolov8_msgs::msg::Point2D newPoint;
                newPoint.x = target.xy.x;
                newPoint.y = target.xy.y;
                transformedDetection->mask.data.push_back(newPoint);
            }   
        }
        transformedArray->detections.push_back(*transformedDetection);
    }

    if (!transformedArray->detections.empty()) {
        publisher_->publish(*transformedArray);
    }
}
