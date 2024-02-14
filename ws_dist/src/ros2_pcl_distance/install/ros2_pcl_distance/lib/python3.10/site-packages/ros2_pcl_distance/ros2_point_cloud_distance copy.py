import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import numpy as np

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

        self.target_pub = self.create_publisher(
            PointCloud2,
            '/target_points',
            10
        )

        ts = message_filters.ApproximateTimeSynchronizer([self.depth_cam_sub, self.cam_info_sub], 10, 1)
        ts.registerCallback(self.cloudCallback)

        ts_mask = message_filters.ApproximateTimeSynchronizer([self.depth_cam_sub, self.mask_sub], 10, 5)
        ts_mask.registerCallback(self.maskCallback)

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

    def maskCallback(self, image_msg, mask_msg):
        height = image_msg.height
        width = image_msg.width

        x = np.arange(0, height, self.skip)
        y = np.arange(0, width, self.skip)

        meshgrid = self.create_grid(height, width, self.skip)

        # necessary to change the representation of depth data so that it can be manipulated using a np.array
        data_depth = np.array(self.bridge.imgmsg_to_cv2(image_msg, desired_encoding='passthrough')).reshape(height, width)

        match_point_cloud = []
        full_mask_points = []

        for detection in mask_msg.detections:
            if (len(detection.mask.data) > 0):
                self.get_logger().info("receving masks..")
                mask_points = np.array([[int(ele.y), int(ele.x), 0] for ele in detection.mask.data])
                full_mask_points.extend(mask_points)
                
                # Generate a polygon by connecting the values in points
                polygon = Polygon(mask_points[:, :2])

                # Check if any of meshgrid points lie in the polygon
                points_inside_polygon = self.is_inside_poly(tuple(meshgrid.flatten()), polygon)

                # Get the depth value and generate a new point_cloud
                for point, flag in zip(meshgrid.flatten(), points_inside_polygon):
                    if (flag):
                        match_point_cloud.append((point.x, point.y, data_depth[int(point.x)][int(point.y)]))
        
        self.publishCloud(full_mask_points, image_msg, self.mask_pub)
        self.publishCloud(match_point_cloud, image_msg, self.target_pub)

    def create_grid(self, height, width, skip):
        matrix = np.empty((height//skip, width//skip), dtype=object)
        for i in range(0, height, skip):
            for j in range(0, width, skip):
                matrix[i//skip, j//skip] = Point(i,j)
        return matrix


    def is_inside_poly(self, points, polygon):
        return np.array([polygon.contains(Point(p)) for p in points])

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
    node = PointCloudDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
