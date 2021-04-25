#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import HardwareStatus

class HardwareStatusPublisherNode(Node):

    def __init__(self):
        super().__init__("hardware_status_publisher") 
        self.hw_status_publisher_ = self.create_publisher(HardwareStatus, "hardware_status", 10) # create topic robot_news in which robot_news will publish
        self.timer = self.create_timer(1.0, self.publish_hw_status) # publish 2 times per sec 
        self.get_logger().info("Hardware Status Publisher has been started")
    
    def publish_hw_status(self):
        msg = HardwareStatus()
        msg.temperature = 32
        msg.are_motors_ready = True
        msg.debug_message = "Nothing special here"
        self.hw_status_publisher_.publish(msg)
    

def main(args=None):
    rclpy.init(args=args) # first line to be written in any ROS2 .py node
    node = HardwareStatusPublisherNode()
    rclpy.spin(node) # allow node to continue to be alive
    rclpy.shutdown() # last line of any ROS2 .py node

if __name__ == "__main__":
    main()