import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import numpy as np

from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo, Image
import sensor_msgs.msg as msg
from yolov8_msgs.msg import Detection, DetectionArray, Mask, Point2D
import message_filters
from std_msgs.msg import Header
from cv_bridge import CvBridge

class PointCloudOnlyNode(Node):
    def __init__(self):
        super().__init__('point_cloud_only')
        
        self.depth_cam_sub = message_filters.Subscriber(self, Image, "/depth/image_raw")
        self.cam_info_sub = message_filters.Subscriber(self, CameraInfo, "/depth/camera_info")
        self.mask_sub = message_filters.Subscriber(self, DetectionArray, "yolo/detections")

        self.point_pub = self.create_publisher(
            PointCloud2,
            '/depth_points',
            10
        )

        self.mask_pub = self.create_publisher(
            PointCloud2,
            '/mask_points',
            10
        )

        ts = message_filters.ApproximateTimeSynchronizer([self.depth_cam_sub, self.cam_info_sub], 10, 1)
        ts.registerCallback(self.cloudCallback)

        self.bridge = CvBridge()

        self.skip = 8

        
    def cloudCallback(self, image_msg, info_msg):
        height = image_msg.height
        width = image_msg.width

        data_depth = np.array(self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')).reshape(height, width)

        point_cloud =[]

        for x in range(0, height, self.skip):
            for y in range(0, width, self.skip):
                depth = data_depth[x][y]
                point_cloud.append(self.depthToPointCloudPos(info_msg, x, y, depth))

        self.publishCloud(point_cloud, image_msg, self.point_pub)

    def depthToPointCloudPos(self, info_msg, x, y, d):
    
        # We will use this later
        fx = info_msg.k[0]
        fy = info_msg.k[4]
        cx = info_msg.k[2]
        cy = info_msg.k[5]

        point_x = (x - cx) * d / fx
        point_y = (y - cy) * d / fy
        point_z = d
        return x , y, d

    def publishCloud(self, points, img_msg, publisher):
        if len(points) > 0:
            header = img_msg.header
            cloud = point_cloud2.create_cloud_xyz32(header, points)
            publisher.publish(cloud)
            self.get_logger().info("Published matching points as PointCloud2")

def main(args=None):
    rclpy.init(args=args)
    node = PointCloudOnlyNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
