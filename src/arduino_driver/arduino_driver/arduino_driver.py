#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
import serial

class CmdVelBridge(Node):
    def __init__(self):  # <- Diperbaiki
        super().__init__('cmd_vel_serial_bridge')
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel_unstamped',
            self.listener_callback,
            10)
        self.ser = serial.Serial('/dev/ttyACM0', 115200)  # Ganti sesuai port Arduino

    def listener_callback(self, msg):
        linear = msg.linear.x
        angular = msg.angular.z
        data = f"v:{linear:.2f},w:{angular:.2f}\n"
        self.ser.write(data.encode('utf-8'))
        print(f"[SEND] {data.strip()}")

def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Stopped by user.")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
