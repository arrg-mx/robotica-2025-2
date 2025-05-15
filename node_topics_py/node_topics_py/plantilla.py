#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

class MyNode(Node):
    def __init__(self):
        super().__init__("my_first_node")


def main(args=None):
    rclpy.init(args=args)
    node = MyNode()
    rclpy.spin()
    rclpy.shutdown()



if __name__ == "__main__":
    main()