#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

class SmartphoneNode(Node):

    def __init__(self):
        super().__init__("smartphone") 

        self.subscriber_ = self.create_subscription(String, "robot_news", self.callback_robot_news, 10)
        self.get_logger().info("Smartphone has been started")
    

    def callback_robot_news(self, msg):
        self.get_logger().info("Received from robot_news: {}".format(msg.data))

def main(args=None):
    rclpy.init(args=args) # first line to be written in any ROS2 .py node
    node = SmartphoneNode()
    rclpy.spin(node) # allow node to continue to be alive
    rclpy.shutdown() # last line of any ROS2 .py node

if __name__ == "__main__":
    main()