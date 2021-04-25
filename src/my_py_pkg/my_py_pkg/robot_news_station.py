#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.msg import String

class RobotNewsStationNode(Node):

    def __init__(self):
        super().__init__("robot_news_station") 

        self.declare_parameter("robot_name", "C3PO")

        self.robot_name_ = self.get_parameter("robot_name").value
        self.publisher_ = self.create_publisher(String, "robot_news", 10) # create topic robot_news in which robot_news will publish
        self.timer = self.create_timer(0.5, self.publish_news) # publish 2 times per sec 
        self.get_logger().info("[{}] Robot News Station has been started".format(self.robot_name_))
    
    def publish_news(self):
        msg = String()
        msg.data = "Hi, this is {0} from robot news station".format(self.robot_name_)
        self.publisher_.publish(msg)
    

def main(args=None):
    rclpy.init(args=args) # first line to be written in any ROS2 .py node
    node = RobotNewsStationNode()
    rclpy.spin(node) # allow node to continue to be alive
    rclpy.shutdown() # last line of any ROS2 .py node

if __name__ == "__main__":
    main()