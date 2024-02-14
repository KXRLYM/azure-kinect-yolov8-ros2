import rclpy
from rclpy.node import Node
import ros2_numpy as rnp
import numpy as np
import random

from sensor_msgs_py import point_cloud2
from sensor_msgs.msg import PointCloud2, PointField, CameraInfo, Image
import sensor_msgs.msg as msg
from geometry_msgs.msg import PointStamped
from yolov8_msgs.msg import Detection, DetectionArray, Mask, Point2D
import message_filters
from shapely.geometry import Point, Polygon
from std_msgs.msg import Header
from cv_bridge import CvBridge

from xarm_move.srv import GetTargetPoint

class PointCloudDistanceNode(Node):
    def __init__(self):
        super().__init__('ros2_point_cloud_distance')

        self.srv = self.create_service(GetTargetPoint, 'get_target_point', self.getTargetPointCallback)
        
        self.pub_target_point = self.create_publisher(PointStamped, "/target_point", 10)

        self.depth_cam_sub = message_filters.Subscriber(self, Image, "/depth/image_raw")
        self.cam_info_sub = message_filters.Subscriber(self, CameraInfo, "/depth/camera_info")
        self.mask_sub = message_filters.Subscriber(self, DetectionArray, "/detections_transformed")

        self.target_point = PointStamped()

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

    def getTargetPointCallback(self, request, response):
        response.target_point = self.target_point
        print("target sending.. ")
        self.get_logger().info("Target at ({:.5f}, {:.5f}, {:.5f})".format(self.target_point.point.x, self.target_point.point.y, self.target_point.point.z))
        return response

    def cloudCallback(self, image_msg, info_msg, mask_msg):
        detection_id = ["cat", "bird", "dog"]
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
                mask_points = np.array([[int(ele.y), int(ele.x), 0] for ele in detection.mask.data])
    
                # Generate a polygon by connecting the values in points
                polygon = Polygon(mask_points[:, :2])

                # Check if any of cloud points lie in the polygon
                points_inside_polygon = self.is_inside_poly(point_cloud_2d, polygon)

                total_mask_num = 0
                sum_x = 0.0
                sum_y = 0.0

                point_cloud_array =np.array(point_cloud).reshape(-1,4)
                # Get the depth value and generate a coloured pointcloud - need to zip 2d and 3d then color ones that is true
                for index, (flag, point3d) in enumerate(zip(points_inside_polygon, point_cloud)):
                    if (flag):
                        total_mask_num += 1 
                        sum_x += point3d[0]
                        sum_y += point3d[1]
                        point3d[3] =  colour_code

                if ((detection.class_name in detection_id) and (total_mask_num > 0)):
                    average_x = sum_x/total_mask_num
                    average_y = sum_y/total_mask_num
                    values, row_index = self.find_closest_values_2d(average_x, average_y, point_cloud_array[:,:2])

                    target_msg = PointStamped()
                    target_msg.header = Header()
                    target_msg.header.frame_id = "depth_camera_link"
                    target_msg.header.stamp = self.get_clock().now().to_msg()
                    target_msg.point.x = point_cloud_array[row_index,0]
                    target_msg.point.y = point_cloud_array[row_index,1]
                    target_msg.point.z = point_cloud_array[row_index,2]
                    self.target_point = target_msg
                    self.get_logger().info("Detetion at ({:.5f}, {:.5f}, {:.5f})".format(target_msg.point.x, target_msg.point.y, target_msg.point.z))
                    self.pub_target_point.publish(target_msg)
        
        # self.publishCloud(full_mask_points, image_msg, self.mask_pub)
        self.publishCloud(point_cloud, image_msg, self.point_pub)


    def find_closest_values_2d(self, target1, target2, array):
        if not array.size:
            return None, None  # Handle empty array case

        array = np.array(array)

        # Calculate Euclidean distance
        distances = np.linalg.norm(array - np.array([target1, target2]), axis=1)

        # Find index of minimum distance
        min_index = np.argmin(distances)

        # Get the closest values and row index
        closest_values = array[min_index]
        row_index = min_index

        return closest_values, row_index
    
    def is_inside_poly(self, points, polygon):
        return np.array([polygon.contains(Point(p)) for p in points])


    def depthToPointCloudPos(self, info_msg, x, y, d, coloured):
    
        fx = 505.26
        fy = 505.459
        cx = 331.293
        cy = 330.211
        point_y = ((x - cx) * d / fx)/1000
        point_x = ((y - cy) * d / fy)/1000
        point_z = d/1000
        return [point_x , point_y, point_z, coloured]

    def publishCloud(self, points, img_msg, publisher):
        if len(points) > 0:
            header = img_msg.header
            cloud = point_cloud2.create_cloud(header, self.fields, points)
            publisher.publish(cloud)



def main(args=None):
    rclpy.init(args=args)
    node = PointCloudDistanceNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
