#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

class PublishPointCloudNode : public rclcpp::Node {
public:
  PublishPointCloudNode() : Node("publish_pointcloud_node"), current_cloud_index_(0) {
    // Create publishers for four separate topics
    publisher_1_ = create_publisher<sensor_msgs::msg::PointCloud2>("/captured_pointcloud_1", 10);
    publisher_2_ = create_publisher<sensor_msgs::msg::PointCloud2>("/captured_pointcloud_2", 10);
    publisher_3_ = create_publisher<sensor_msgs::msg::PointCloud2>("/captured_pointcloud_3", 10);
    publisher_4_ = create_publisher<sensor_msgs::msg::PointCloud2>("/captured_pointcloud_4", 10);

    // Set up a timer to trigger the publication in a round-robin fashion
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&PublishPointCloudNode::publishNextCloud, this));
  }

private:
  void publishNextCloud() {
    // Load and publish the point cloud based on the current index
    std::string file_name = "point" + std::to_string(current_cloud_index_ + 1) + ".pcd";
    publishPointCloud(file_name);

    // Increment the index and wrap around if necessary
    current_cloud_index_ = (current_cloud_index_ + 1) % 4;
  }

  void publishPointCloud(const std::string &file_name) {
    // Load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(file_name, *pcl_cloud);
    std::cout << "PointCloud has: " << pcl_cloud->points.size () << " data points." << std::endl;

    
    //pcl::PCLPointCloud2 pcl_pc2; //ros type message
    //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //std::cout << "here" << std::endl;
    //pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    
    // Convert PCL point cloud to sensor_msgs::msg::PointCloud2
    //sensor_msgs::msg::PointCloud2 msg;
    //pcl_conversions::fromPCL(pcl_pc2, msg);
    
    sensor_msgs::msg::PointCloud2::Ptr msg_ptr(new sensor_msgs::msg::PointCloud2);
    pcl::toROSMsg(*pcl_cloud, *msg_ptr);


    // Publish on the corresponding topic based on the file name
    if (file_name == "point1.pcd") {
      msg_ptr->header.frame_id = "left";
      publisher_1_->publish(*msg_ptr);
    } else if (file_name == "point2.pcd") {
      msg_ptr->header.frame_id = "centre";
      publisher_2_->publish(*msg_ptr);
    } else if (file_name == "point3.pcd") {
      msg_ptr->header.frame_id = "right";
      publisher_3_->publish(*msg_ptr);
    } else if (file_name == "point4.pcd") {
      msg_ptr->header.frame_id = "back";
      publisher_4_->publish(*msg_ptr);
    }
  }

  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_1_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_2_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_3_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_4_;
  rclcpp::TimerBase::SharedPtr timer_;
  int current_cloud_index_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PublishPointCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
