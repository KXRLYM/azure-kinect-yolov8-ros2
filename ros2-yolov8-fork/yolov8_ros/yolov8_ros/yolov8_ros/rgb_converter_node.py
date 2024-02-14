import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data
import cv2

class RgbConverterNode(Node):
    def __init__(self):
        super().__init__('rgb_converter_node')

        # Create a publisher to publish the converted image
        self.publisher = self.create_publisher(Image, '/rgb/image_formatted', qos_profile_sensor_data)

        # Create a subscriber to subscribe to the original image topic
        self.subscription = self.create_subscription(
            Image,
            '/camera/rgb/image_raw',
            self.image_callback,
            10
        )
        self.subscription

        # Create a CvBridge for converting between ROS Image messages and OpenCV images
        self.bridge = CvBridge()

    def image_callback(self, msg):
        encoding = msg.header.frame_id
        try:
            # Convert the ROS Image message to a BGR8 OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            if encoding != 'bgr8':
                cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGRA2BGR)
            
            # Perform your additional processing here if needed

            # Convert the BGR8 OpenCV image back to a ROS Image message
            converted_image_msg = self.bridge.cv2_to_imgmsg(cv_image, encoding='rgb8')

            # Publish the converted image
            self.publisher.publish(converted_image_msg)

        except Exception as e:
            self.get_logger().error(f"Error in image_callback: {e}")



def main(args=None):
    rclpy.init(args=args)

    rgb_converter_node = RgbConverterNode()

    rclpy.spin(rgb_converter_node)

    rgb_converter_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()