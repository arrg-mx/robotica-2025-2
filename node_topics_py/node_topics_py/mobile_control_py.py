#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import time

class MobileNodeControl(Node):
    def __init__(self):
        super().__init__("omni_control_node")
        topic_name = "/cmd_vel"
        self.mobile_publisher_ = self.create_publisher(Twist, topic_name, 10)
        self.timer_mobile_ = self.create_timer(1, self.mobile_cbck)
        self.get_logger().info('Control del robot activo')

    def mobile_cbck(self):
        msg = Twist()
        msg.linear.x = 0.05
        msg.linear.y = 0.0
        msg.angular.z = 0.1
        self.mobile_publisher_.publish(msg)
        time.sleep(3)

        msg.linear.x = 0.0
        msg.linear.y = 0.05
        msg.angular.z = -0.1
        self.mobile_publisher_.publish(msg)
        time.sleep(3)

        msg.linear.x = 0.0
        msg.linear.y = 0.0
        msg.angular.z = -0.0
        self.mobile_publisher_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = MobileNodeControl()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()