import os
import cv2
import rclpy
from sensor_msgs.msg import Image, JointState
from cv_bridge import CvBridge
from rclpy.node import Node
import time

class ImageSubscriberNode(Node):
    def __init__(self):
        super().__init__('image_subscriber_node')
        self.subscription = self.create_subscription(
            Image,
            '/rgb/image_raw',
            self.image_callback,
            10  # QoS profile depth
        )
        self.subscription  # prevent unused variable warning
        self.image_received = False
        self.bridge = CvBridge()

    def image_callback(self, msg):
        time.sleep(3.0)
        if not self.image_received:
            self.get_logger().info('Received the first image.')
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Save the image as a PNG file in a directory
            save_directory = '/home/nam017/ws_calibration/src/HandEyeCalibration-using-OpenCV/KinectCalibration/RGBImgs'
            if not os.path.exists(save_directory):
                os.makedirs(save_directory)

            image_path = os.path.join(save_directory, 'color_image.png')
            cv2.imwrite(image_path, cv_image)

            self.get_logger().info(f'Image saved to: {image_path}')
            self.image_received = True

            self.destroy_node()
            rclpy.shutdown()


def main():
    rclpy.init()
    node = ImageSubscriberNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()