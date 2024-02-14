import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from sensor_msgs.msg import CameraInfo
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import cv2

class DepthConverterNode(Node):
    def __init__(self):
        super().__init__('depth_converter_node')

        # Create a publisher to publish the converted image
        self.publisher = self.create_publisher(Image, '/depth/image_formatted', qos_profile_sensor_data)
        self.info_pub = self.create_publisher(CameraInfo, '/depth/camera_info_formatted', qos_profile_sensor_data)

        # Create a subscriber to subscribe to the original image topic
        self.subscription = self.create_subscription(
            Image,
            '/depth_to_rgb/image_raw',
            self.image_callback,
            10
        )
        self.subscription


        self.subscription = self.create_subscription(
            CameraInfo,
            '/depth/camera_info',
            self.cam_callback,
            10
        )
        self.subscription


    def image_callback(self, msg):
        self.publisher.publish(msg)

    def cam_callback(self, msg):
        self.info_pub.publish(msg)



def main(args=None):
    rclpy.init(args=args)

    depth_converter_node = DepthConverterNode()

    rclpy.spin(depth_converter_node)

    depth_converter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()