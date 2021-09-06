#!/usr/bin/python3
import rclpy
from rclpy.node import Node
from cv_bridge import CvBridge
from sensor_msgs.msg import Image
import EasyPySpin
from time import sleep


class ImagePublisher(Node):
    def __init__(self) -> None:
        super().__init__('broadcaster')
        self.declare_parameter('topic')
        self.publisher = self.create_publisher(
            Image, self.get_parameter('topic').value, 10)
        self.publisher = self.create_publisher(Image, "/blackfly/image/raw", 10)
        self.bridge = CvBridge()

    def connect_blackfly(self) -> None:
        self.camera = EasyPySpin.VideoCapture(0)
        while not self.camera.isOpened():
            self.get_logger().warn('Blackfly not detected. Re-trying in 2 seconds')
            sleep(2)
            self.camera = EasyPySpin.VideoCapture(0)
        self.get_logger().info('Blackfly Detected!')
        self.timer = self.create_timer(1/20.0, self.timer_callback)

    def timer_callback(self) -> None:
        success, frame = self.camera.read()
        if not success:
            self.get_logger().info('Failed to grab frame')
        else:
            self.publisher.publish(self.bridge.cv2_to_imgmsg(frame, 'mono8'))

    def __del__(self) -> None:
        self.camera.release()


def main(args=None):
    rclpy.init(args=args)
    image_publisher = ImagePublisher()
    image_publisher.connect_blackfly()
    rclpy.spin(image_publisher)

    # termination
    image_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
