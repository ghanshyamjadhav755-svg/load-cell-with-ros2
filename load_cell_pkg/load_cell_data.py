#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import serial
import re

class LoadCellPublisher(Node):

    def __init__(self):
        super().__init__('loadcell_publisher')

        self.publisher_ = self.create_publisher(Float32, '/load_cell_data', 10)

        # Change port according to your system
        self.serial_port = serial.Serial('/dev/ttyACM0', 9600, timeout=1)

        self.timer = self.create_timer(0.1, self.read_serial)

        self.get_logger().info("Load Cell Node Started")

    def read_serial(self):

        if self.serial_port.in_waiting > 0:

            line = self.serial_port.readline().decode('utf-8').strip()

            match = re.search(r"([-+]?\d*\.\d+|\d+)", line)

            if match:
                weight = float(match.group())

                msg = Float32()
                msg.data = weight

                self.publisher_.publish(msg)

                # self.get_logger().info(f"{weight}")


def main(args=None):
    rclpy.init(args=args)

    node = LoadCellPublisher()

    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()