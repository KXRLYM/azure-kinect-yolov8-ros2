import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import numpy as np
import random

from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo, Image
import sensor_msgs.msg as msg
from yolov8_msgs.msg import Detection, DetectionArray, Mask, Point2D
import message_filters
from shapely.geometry import Point, Polygon
from std_msgs.msg import Header
from cv_bridge import CvBridge

class PointCloudDistanceNode(Node):
    def __init__(self):
        super().__init__('ros2_point_cloud_distance')
        
        self.depth_cam_sub = message_filters.Subscriber(self, Image, "/depth/image_raw")
        self.cam_info_sub = message_filters.Subscriber(self, CameraInfo, "/depth/camera_info")
        self.mask_sub = message_filters.Subscriber(self, DetectionArray, "/detections_transformed")

        self.point_pub = self.create_publisher(
            PointCloud2,
            '/depth_points',
            10
        )

        ts = message_filters.ApproximateTimeSynchronizer([self.depth_cam_sub, self.cam_info_sub, self.mask_sub], 10, 0.5)
        ts.registerCallback(self.cloudCallback)

        self.bridge = CvBridge()

        self.field = PointField(name = 'x')
        self.skip = 8
        self.fields = [
            PointField(name = 'x', offset = 0, datatype = PointField.FLOAT32, count = 1),
            PointField(name = 'y', offset = 4, datatype = PointField.FLOAT32, count = 1),
            PointField(name = 'z', offset = 8, datatype = PointField.FLOAT32, count = 1),
            PointField(name = 'rgb', offset = 12, datatype = PointField.UINT32, count = 1)
        ]

        
    def cloudCallback(self, image_msg, info_msg, mask_msg):
        height = image_msg.height
        width = image_msg.width

        data_depth = np.array(self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')).reshape(height, width)

        # creating a 1d array of all valid pointcloud 3d coordinates
        point_cloud =[]

        # creating 1d array of all valid pointcloud 2d coordinates
        point_cloud_2d = []

        for x in range(0, height, self.skip):
            for y in range(0, width, self.skip):
                depth = data_depth[x][y]
                point_cloud.append(self.depthToPointCloudPos(info_msg, x, y, depth, 0xFFFFFF))
                point_cloud_2d.append([x,y])

        for detection in mask_msg.detections:
            if (len(detection.mask.data) > 4):
                colour_code = random.randint(0, 0xFFFFFE)
                self.get_logger().info("receving masks..")
                mask_points = np.array([[int(ele.y), int(ele.x), 0] for ele in detection.mask.data])
    
                # Generate a polygon by connecting the values in points
                polygon = Polygon(mask_points[:, :2])

                # Check if any of cloud points lie in the polygon
                points_inside_polygon = self.is_inside_poly(point_cloud_2d, polygon)

                # Get the depth value and generate a coloured pointcloud - need to zip 2d and 3d then color ones that is true
                for index, (flag, point3d) in enumerate(zip(points_inside_polygon, point_cloud)):
                    if (flag):
                        point3d[3] =  colour_code
        
        # self.publishCloud(full_mask_points, image_msg, self.mask_pub)
        self.publishCloud(point_cloud, image_msg, self.point_pub)

    def is_inside_poly(self, points, polygon):
        return np.array([polygon.contains(Point(p)) for p in points])

    def depthToPointCloudPos(self, info_msg, x, y, d, coloured):
    
        # We will use this later
        fx = info_msg.k[0]
        fy = info_msg.k[4]
        cx = info_msg.k[2]
        cy = info_msg.k[5]

        point_x = (x - cx) * d / fx
        point_y = (y - cy) * d / fy
        point_z = d
        return [x , y, d, coloured]

    def publishCloud(self, points, img_msg, publisher):
        if len(points) > 0:
            header = img_msg.header
            cloud = point_cloud2.create_cloud(header, self.fields, points)
            publisher.publish(cloud)
            self.get_logger().info("Published matching points as PointCloud2")


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
