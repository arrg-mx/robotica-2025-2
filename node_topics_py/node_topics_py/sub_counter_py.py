#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int64

class CounterNode(Node):
    def __init__(self):
        super().__init__("counter_node")
        topic_name = "number_publihser"
        topic_pub_name = "number_counter_publihser"
        self.counter_ = 0

        self.pub_counter_ = self.create_publisher(Int64,topic_pub_name, 10)
        self.sub_counter_ = self.create_subscription(Int64,topic_name,self.counter_callback,10)
        self.get_logger().info('Nodo subcriptor-publicardor activo ')


    def counter_callback(self,msg):
        self.counter_ += msg.data
        new_msg = Int64()
        new_msg.data = self.counter_
        self.pub_counter_.publish(new_msg)
   
def main(args=None):
    rclpy.init(args=args)
    node = CounterNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == "__main__":
    main()