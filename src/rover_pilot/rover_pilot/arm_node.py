import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Joy
from rover_pilot import ctrl

baud_rate = 115200
port = ctrl.SerialPortChecker(baud_rate, 2).find_port("arm")
armObj = ctrl.drive(port, baud_rate)
armObj.connect()


class MinimalSubscriber(Node):
    def __init__(self):
        super().__init__('minimal_subscriber')
        self.subscription = self.create_subscription(
            Joy,
            '/j0',
            self.listener_callback,
            10)
        self.subscription  # prevent unused variable warning

    def listener_callback(self, data):
        status = armObj.setState(round(data.axes[0]*255), round(data.axes[1]*255), round(data.axes[5]*255), data.buttons)
        armObj.serialWrite()
        self.get_logger().info('"%s"' % status)


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
