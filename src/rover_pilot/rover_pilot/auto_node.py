"""Bridge Nav2 velocity commands to the rover's existing ESP32 protocol.

The current firmware accepts one direction and one PWM byte per command. It
cannot command independent left and right wheel speeds, so this node selects
the stronger normalized forward/turn request for each Twist message.
"""

import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node

from rover_pilot import ctrl


class AutoDriveNode(Node):
    def __init__(self):
        super().__init__('auto_drive_node')

        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baud_rate', 115200)
        self.declare_parameter('max_linear_speed', 0.40)  # m/s, Nav2 config
        self.declare_parameter('max_angular_speed', 1.00)  # rad/s, Nav2 config
        self.declare_parameter('command_timeout', 0.5)  # seconds

        port = self.get_parameter('port').value
        baud_rate = self.get_parameter('baud_rate').value
        self.max_linear = float(self.get_parameter('max_linear_speed').value)
        self.max_angular = float(self.get_parameter('max_angular_speed').value)
        self.command_timeout = float(self.get_parameter('command_timeout').value)

        if self.max_linear <= 0.0 or self.max_angular <= 0.0:
            raise ValueError('max_linear_speed and max_angular_speed must be positive')

        self.drive = ctrl.drive(port, baud_rate)
        self.drive.connect()
        self.last_command_time = time.monotonic()

        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_nav',
            self.cmd_vel_callback,
            10,
        )
        self.watchdog = self.create_timer(0.1, self.stop_if_command_stale)
        self.get_logger().info(
            f'Listening on /cmd_vel_nav; sending rover commands to {port} at {baud_rate} baud'
        )

    def send_stop(self):
        self.drive.setState(0, 0)
        self.drive.serialWrite()

    def cmd_vel_callback(self, msg: Twist):
        # ctrl.drive.setState expects turn on xData and translation on yData.
        # Its sign convention is positive xData = right, negative yData = forward.
        linear = max(-1.0, min(1.0, msg.linear.x / self.max_linear))
        angular = max(-1.0, min(1.0, msg.angular.z / self.max_angular))
        linear_pwm = round(abs(linear) * 255)
        angular_pwm = round(abs(angular) * 255)

        # The existing two-byte firmware protocol supports only one motion
        # direction at a time. Select the stronger normalized request.
        if angular_pwm > linear_pwm:
            x_data = -round(angular * 255)
            y_data = 0
        else:
            x_data = 0
            y_data = -round(linear * 255)

        self.drive.setState(x_data, y_data)
        self.drive.serialWrite()
        self.last_command_time = time.monotonic()

    def stop_if_command_stale(self):
        if time.monotonic() - self.last_command_time > self.command_timeout:
            self.send_stop()
            # Avoid writing stop continuously while no Nav2 commands arrive.
            self.last_command_time = time.monotonic()

    def destroy_node(self):
        try:
            self.send_stop()
        except Exception as exc:  # still release the serial port on shutdown
            self.get_logger().warning(f'Could not send stop command during shutdown: {exc}')
        serial_port = getattr(self.drive, 'serial_port', None)
        if serial_port is not None and serial_port.is_open:
            serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = AutoDriveNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
