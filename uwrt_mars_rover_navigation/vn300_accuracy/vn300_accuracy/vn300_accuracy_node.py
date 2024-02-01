import rclpy
from rclpy.node import Node

from sensor_msgs.msg import NavSatFix

class MinimalSubscriber(Node):

    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            NavSatFix,
            'vectornav/navSatFix',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, msg):
        print(f"{msg.header.stamp} {msg.latitude} {msg.longitude}")
        # self.get_logger().info('I heard: (%d, %d)' % msg.latitude, msg.longitude)


def main(args=None):
    rclpy.init(args=args)

    minimal_subscriber = MinimalSubscriber()

    rclpy.spin(minimal_subscriber)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_subscriber.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
