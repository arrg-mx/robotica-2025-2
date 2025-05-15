#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64, Float64

class PublisherNode(Node):
    def __init__(self):
        super().__init__("publisher_node")
        topic_name = "number_publihser"
        self.number_ = 1 #rad
        self.publisher_node_ = self.create_publisher(Int64,topic_name, 10)
        self.timer_pub_ = self.create_timer(1, self.pub_callback)
        self.get_logger().info('Nodo Publicador esta activo')

    def pub_callback(self):
        msg = Int64()
        msg.data = self.number_ 
        self.publisher_node_.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = PublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()