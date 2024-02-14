#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/point_cloud2.hpp"
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/passthrough.h>
#include "tf2_ros/transform_listener.h"
#include "tf2_sensor_msgs/tf2_sensor_msgs.hpp"
#include "tf2_ros/buffer.h"
#include "tf2/exceptions.h"
#include "pcl_ros/transforms.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"

#include <Eigen/Dense>

class PublishPointCloudNode : public rclcpp::Node {
public:
  PublishPointCloudNode() : Node("publish_pointcloud_node"), current_cloud_index_(0) {
    // Create publishers for four separate topics
    publisher_1_ = create_publisher<sensor_msgs::msg::PointCloud2>("/captured_pointcloud_1", 10);
    publisher_2_ = create_publisher<sensor_msgs::msg::PointCloud2>("/captured_pointcloud_2", 10);
    publisher_3_ = create_publisher<sensor_msgs::msg::PointCloud2>("/captured_pointcloud_3", 10);
    publisher_4_ = create_publisher<sensor_msgs::msg::PointCloud2>("/captured_pointcloud_4", 10);

    publisher_inverse_kinematics = create_publisher<geometry_msgs::msg::PointStamped>("/inverse_kinematics", 10);
    point.header.frame_id = "world";
    point.header.stamp = this->now();

    Eigen::MatrixXd transformation_matrix(4,4);
    transformation_matrix << 0.0101185, -0.99837769, -0.05603213,  0.38812277,
                            -0.9990645,  -0.00773759, -0.04254,    0.50832947,
                            0.04204442,  0.05641022, -0.99752201,  0.01127935,
                            0.,          0.,          0.,          1.;
            
    
    // this is the base origin of the 
    Eigen::Vector4d original_point(0.0, 0.0, 0.135/2, 1.0);

    Eigen::Vector4d transformed_point = transformation_matrix * original_point;
    point.point.x = transformed_point(0);
    point.point.y = transformed_point(1);
    point.point.z = transformed_point(2);

    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_); 
    tf2::Duration timeout(std::chrono::duration_cast<std::chrono::nanoseconds>(std::chrono::seconds(10)));

    try {
      t_centre = tf_buffer_->lookupTransform(
        "world", 
        "centre",
        tf2::TimePointZero,
        timeout
      );
      t_right = tf_buffer_->lookupTransform(
        "world", 
        "right",
        tf2::TimePointZero,
        timeout
      );
      t_left = tf_buffer_->lookupTransform(
        "world", 
        "left",
        tf2::TimePointZero,
        timeout
      );
      t_back = tf_buffer_->lookupTransform(
        "world", 
        "back",
        tf2::TimePointZero,
        timeout
      );
    } catch (const tf2::TransformException & ex) {
        RCLCPP_INFO(
            this->get_logger(), "Could not transform : %s", ex.what());
        rclcpp::shutdown();
    }

    // Set up a timer to trigger the publication in a round-robin fashion
    timer_ = create_wall_timer(std::chrono::seconds(1), std::bind(&PublishPointCloudNode::publishNextCloud, this));
  }

private:
  void publishNextCloud() {
    std::cout << "Transformed Point: (" 
          << point.point.x << ", " 
          << point.point.y << ", " 
          << point.point.z << ")" << std::endl;
    publisher_inverse_kinematics->publish(point);
    // Load and publish the point cloud based on the current index
    std::string file_name = "point" + std::to_string(current_cloud_index_ + 1) + ".pcd";
    publishTransformedPointCloud(file_name);
    //publishPointCloud(file_name);

    // Increment the index and wrap around if necessary
    current_cloud_index_ = (current_cloud_index_ + 1) % 4;
  }

  void publishTransformedPointCloud(const std::string &file_name) {
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(file_name, *pcl_cloud);

    // Transform the point cloud to the desired frame
    pcl::PointCloud<pcl::PointXYZ>::Ptr output_cloud(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(output_cloud);

    // Publish on the corresponding topic based on the file name
    if (file_name == "point1.pcd") {
      pcl_cloud->header.frame_id = "left";
      pcl_ros::transformPointCloud("world", *pcl_cloud, *output_cloud, *tf_buffer_);
      //pcl::io::savePCDFileASCII("point1_transformed.pcd", *output_cloud);

      pass.setFilterFieldName("x");
      pass.setFilterLimits(0.105,0.255);
      pass.filter(*cloud_filtered);
      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(0.175,0.425);
      pass.filter(*cloud_filtered);
      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(-0.025,0.125);
      pass.filter(*cloud_filtered);

      Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Identity();
      
      transform_1(0, 0) = 0.999793;
      transform_1(0, 1) = 0.011635;
      transform_1(0, 2) = -0.016691;
      transform_1(0, 3) = 0.013451;

      transform_1(1, 0) = -0.011947;
      transform_1(1, 1) = 0.999754;
      transform_1(1, 2) = -0.018688;
      transform_1(1, 3) = -0.007649;

      transform_1(2, 0) = 0.016469;
      transform_1(2, 1) = 0.018884;
      transform_1(2, 2) = 0.999686;
      transform_1(2, 3) = -0.005137;

      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::transformPointCloud (*cloud_filtered, *transformed_cloud, transform_1);

      sensor_msgs::msg::PointCloud2::Ptr msg_ptr(new sensor_msgs::msg::PointCloud2);

      // cloud_filtered vs transformed_cloud
      pcl::toROSMsg(*cloud_filtered, *msg_ptr);
      msg_ptr->header.frame_id = "world";
      publisher_1_->publish(*msg_ptr);

    } else if (file_name == "point2.pcd") {
      pcl_cloud->header.frame_id = "centre";
      pcl_ros::transformPointCloud("world", *pcl_cloud, *output_cloud, *tf_buffer_);
      //pcl::io::savePCDFileASCII("point2_transformed.pcd", *output_cloud);

      pass.setFilterFieldName("x");
      pass.setFilterLimits(0.105,0.252);
      pass.filter(*cloud_filtered);
      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(0.165,0.425);
      pass.filter(*cloud_filtered);
      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(-0.025,0.12);
      pass.filter(*cloud_filtered);

      sensor_msgs::msg::PointCloud2::Ptr msg_ptr(new sensor_msgs::msg::PointCloud2);
      pcl::toROSMsg(*cloud_filtered, *msg_ptr);
      msg_ptr->header.frame_id = "world";
      publisher_2_->publish(*msg_ptr);

    } else if (file_name == "point3.pcd") {
      pcl_cloud->header.frame_id = "right";
      pcl_ros::transformPointCloud("world", *pcl_cloud, *output_cloud, *tf_buffer_);
      //pcl::io::savePCDFileASCII("point3_transformed.pcd", *output_cloud);

      pass.setFilterFieldName("x");
      pass.setFilterLimits(0.105,0.255);
      pass.filter(*cloud_filtered);
      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(0.155,0.425);
      pass.filter(*cloud_filtered);
      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(-0.025,0.125);
      pass.filter(*cloud_filtered);

      Eigen::Matrix4f transform_2 = Eigen::Matrix4f::Identity();
      
      transform_2(0, 0) = 0.997986;
      transform_2(0, 1) = -0.052871;
      transform_2(0, 2) = -0.035065;
      transform_2(0, 3) = 0.022775;

      transform_2(1, 0) = 0.053879;
      transform_2(1, 1) = 0.998142;
      transform_2(1, 2) = 0.028456;
      transform_2(1, 3) = -0.014190;

      transform_2(2, 0) = 0.033495;
      transform_2(2, 1) = -0.030288;
      transform_2(2, 2) = 0.998980;
      transform_2(2, 3) = 0.000073;

      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::transformPointCloud (*cloud_filtered, *transformed_cloud, transform_2);

      sensor_msgs::msg::PointCloud2::Ptr msg_ptr(new sensor_msgs::msg::PointCloud2);
      std::cout<<"hello"<<std::endl;
      pcl::toROSMsg(*transformed_cloud, *msg_ptr);
      msg_ptr->header.frame_id = "world";
      publisher_3_->publish(*msg_ptr);

    } else if (file_name == "point4.pcd") {
      pcl_cloud->header.frame_id = "back";
      pcl_ros::transformPointCloud("world", *pcl_cloud, *output_cloud, *tf_buffer_);
      //pcl::io::savePCDFileASCII("point4_transformed.pcd", *output_cloud);

      pass.setFilterFieldName("x");
      pass.setFilterLimits(0.105,0.255);
      pass.filter(*cloud_filtered);
      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName("y");
      pass.setFilterLimits(0.155,0.425);
      pass.filter(*cloud_filtered);
      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(-0.025,0.125);
      pass.filter(*cloud_filtered);

      Eigen::Matrix4f transform_3 = Eigen::Matrix4f::Identity();

      transform_3(0, 0) = 0.998294;
      transform_3(0, 1) = -0.047034;
      transform_3(0, 2) = 0.034601;
      transform_3(0, 3) = 0.005003;

      transform_3(1, 0) = 0.045616;
      transform_3(1, 1) = 0.998130;
      transform_3(1, 2) = 0.040689;
      transform_3(1, 3) = -0.015805;

      transform_3(2, 0) = -0.036450;
      transform_3(2, 1) = -0.039042;
      transform_3(2, 2) = 0.998573;
      transform_3(2, 3) = 0.010660;

      pcl::PointCloud<pcl::PointXYZ>::Ptr transformed_cloud (new pcl::PointCloud<pcl::PointXYZ> ());
      pcl::transformPointCloud (*cloud_filtered, *transformed_cloud, transform_3);

      sensor_msgs::msg::PointCloud2::Ptr msg_ptr(new sensor_msgs::msg::PointCloud2);
      pcl::toROSMsg(*cloud_filtered, *msg_ptr);
      msg_ptr->header.frame_id = "world";
      publisher_4_->publish(*msg_ptr);
    }
  }

  void transformPointCloud(
    const pcl::PointCloud<pcl::PointXYZ>::Ptr input_cloud,
    sensor_msgs::msg::PointCloud2::SharedPtr& ros_cloud,
    const std::string &frame_id)
  {
    try {
      // Transform point cloud using tf_listener_
      pcl::PointCloud<pcl::PointXYZ> output_cloud;
      pcl_ros::transformPointCloud(frame_id, *input_cloud, output_cloud, *tf_buffer_);
      pcl::toROSMsg(output_cloud, *ros_cloud);
      ros_cloud->header.frame_id = frame_id;
    } catch (const tf2::TransformException & ex) {
      RCLCPP_ERROR(
        this->get_logger(), "Error transforming point cloud: %s", ex.what());
        rclcpp::shutdown();
    }
    
  }
  /**
  void publishPointCloud(const std::string &file_name) {
    // Load PCD file
    pcl::PointCloud<pcl::PointXYZ>::Ptr pcl_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(file_name, *pcl_cloud);
    std::cout << "PointCloud has: " << pcl_cloud->points.size () << " data points." << std::endl;


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::PassThrough<pcl::PointXYZ> pass;
    pass.setInputCloud(pcl_cloud);


    //pcl::PCLPointCloud2 pcl_pc2; //ros type message
    //pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
    //std::cout << "here" << std::endl;
    //pcl::fromPCLPointCloud2(pcl_pc2, *temp_cloud);
    
    // Convert PCL point cloud to sensor_msgs::msg::PointCloud2
    //sensor_msgs::msg::PointCloud2 msg;
    //pcl_conversions::fromPCL(pcl_pc2, msg);

    // Publish on the corresponding topic based on the file name
    if (file_name == "point1.pcd") {
      // left
      pass.setFilterFieldName("y");
      pass.setFilterLimits(-0.15,0.13);
      pass.filter(*cloud_filtered);
      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName("x");
      pass.setFilterLimits(0,0.3);
      pass.filter(*cloud_filtered);
      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.18,0.5);
      pass.filter(*cloud_filtered);
      std::cout << "Filtered Cloud has : " << cloud_filtered->points.size () << " data points." << std::endl;
      sensor_msgs::msg::PointCloud2::Ptr msg_ptr(new sensor_msgs::msg::PointCloud2);
      pcl::toROSMsg(*cloud_filtered, *msg_ptr);
      msg_ptr->header.frame_id = "left";
      pcl::io::savePCDFileASCII("point1_filtered.pcd", *cloud_filtered);
      publisher_1_->publish(*msg_ptr);

    } else if (file_name == "point2.pcd") {
      // centre  
      pass.setFilterFieldName("x");
      pass.setFilterLimits(-0.06,0.06);
      pass.filter(*cloud_filtered);
      pass.setInputCloud(cloud_filtered);
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.21,0.35);
      pass.filter(*cloud_filtered);
      std::cout << "Filtered Cloud has : " << cloud_filtered->points.size () << " data points." << std::endl;
      sensor_msgs::msg::PointCloud2::Ptr msg_ptr(new sensor_msgs::msg::PointCloud2);
      pcl::toROSMsg(*cloud_filtered, *msg_ptr);
      msg_ptr->header.frame_id = "centre";
      pcl::io::savePCDFileASCII("point2_filtered.pcd", *cloud_filtered);
      publisher_2_->publish(*msg_ptr);

    } else if (file_name == "point3.pcd") {
      // right
      pass.filter(*cloud_filtered);
      std::cout << "Filtered Cloud has : " << cloud_filtered->points.size () << " data points." << std::endl;
      sensor_msgs::msg::PointCloud2::Ptr msg_ptr(new sensor_msgs::msg::PointCloud2);
      pcl::toROSMsg(*cloud_filtered, *msg_ptr);
      msg_ptr->header.frame_id = "right";
      pcl::io::savePCDFileASCII("point3_filtered.pcd", *cloud_filtered);
      publisher_3_->publish(*msg_ptr);

    } else if (file_name == "point4.pcd") {
      // back
      pass.setFilterFieldName("z");
      pass.setFilterLimits(0.25,0.75);
      pass.filter(*cloud_filtered);
      std::cout << "Filtered Cloud has : " << cloud_filtered->points.size () << " data points." << std::endl;
      sensor_msgs::msg::PointCloud2::Ptr msg_ptr(new sensor_msgs::msg::PointCloud2);
      pcl::toROSMsg(*cloud_filtered, *msg_ptr);
      msg_ptr->header.frame_id = "back";
      pcl::io::savePCDFileASCII("point4_filtered.pcd", *cloud_filtered);
      publisher_4_->publish(*msg_ptr);

    }
  }
  **/
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_1_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_2_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_3_;
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr publisher_4_;
  rclcpp::Publisher<geometry_msgs::msg::PointStamped>::SharedPtr publisher_inverse_kinematics;
  geometry_msgs::msg::TransformStamped t_centre;
  geometry_msgs::msg::TransformStamped t_right;
  geometry_msgs::msg::TransformStamped t_left;
  geometry_msgs::msg::TransformStamped t_back;
  rclcpp::TimerBase::SharedPtr timer_;
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
  int current_cloud_index_;
  geometry_msgs::msg::PointStamped point;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PublishPointCloudNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
