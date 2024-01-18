#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>


class CapturePointCloudNode : public rclcpp::Node {
public:
  CapturePointCloudNode() : Node("capture_pointcloud_node") {
    subscription_ = create_subscription<sensor_msgs::msg::PointCloud2>(
      "/depth_points", 10, std::bind(&CapturePointCloudNode::pointCloudCallback, this, std::placeholders::_1));
  }

private:
  void pointCloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    // Save point cloud data to a PCD file
    std::cout << "hello" << std::endl;
    savePointCloudToPCD(msg);
    rclcpp::shutdown();
  }

  void savePointCloudToPCD(const sensor_msgs::msg::PointCloud2::SharedPtr msg) {
    std::cout << "hello" << std::endl;
    // Convert sensor_msgs::msg::PointCloud2 to PCL point cloud
    pcl::PCLPointCloud2 pcl_cloud;
    pcl_conversions::toPCL(*msg, pcl_cloud);
    pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::fromPCLPointCloud2(pcl_cloud,*temp_cloud);
    // Save the pcl_cloud to PCD file
    pcl::io::savePCDFileASCII("test_pcd.pcd", *temp_cloud);
  }

  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr subscription_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CapturePointCloudNode>();
  rclcpp::spin(node);
  return 0;
}
