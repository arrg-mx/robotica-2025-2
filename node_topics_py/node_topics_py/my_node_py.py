#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("my_first_node")
        self.counter_ = 0
        self.get_logger().info("Hola mundo")
        self.create_timer(1,self.counter_callback)

    def counter_callback(self):
        self.counter_ += 1
        self.get_logger().info("counter  " + str(self.counter_))

def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == "__main__":
    main()