#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

import time

class MobileTestde(Node):
    def __init__(self):
        super().__init__("mobile_test_node")
        self.speed_pub_ = self.create_publisher(Twist, "/cmd_vel", 10)
        self.timer_mobile_ = self.create_timer(1, self.mobile_cbck)
        self.get_logger().info("Nodo de control del robot omni activo")

    def mobile_cbck(self):
        speed_msg = Twist()
        speed_msg.linear.x = 0.1
        speed_msg.linear.y = 0.0
        speed_msg.angular.z = 0.5
        self.speed_pub_.publish(speed_msg)
        time.sleep(2)

        speed_msg = Twist()
        speed_msg.linear.x = 0.0
        speed_msg.linear.y = 0.1
        speed_msg.angular.z = -0.5
        self.speed_pub_.publish(speed_msg)
        time.sleep(2)

        speed_msg = Twist()
        speed_msg.linear.x = 0.1
        speed_msg.linear.y = -0.1
        speed_msg.angular.z = 0.2
        self.speed_pub_.publish(speed_msg)
        time.sleep(2)

        speed_msg = Twist()
        speed_msg.linear.x = 0.0
        speed_msg.linear.y = 0.0
        speed_msg.angular.z = 0.0
        self.speed_pub_.publish(speed_msg)
        time.sleep(2)

def main(args=None):
    rclpy.init(args=args)
    node = MobileTestde()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()