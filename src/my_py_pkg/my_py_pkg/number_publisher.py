#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.msg import Int64

class NumberPublisher(Node):

    def __init__(self):
        super().__init__("number_publisher") 

        self.declare_parameter("number_to_publish", 2) #2 is the default value 
        self.declare_parameter("publish_frequency", 1) #1Hz publish frequency
        
        num_to_pb = 2
        try:
            num_to_pb = int(self.get_parameter("number_to_publish").value)
        except Exception as e:
            self.get_logger().error("number_to_publish must be an integer and will be automatically set to 2")
            num_to_pb = 2

        publish_frequency = 1
        try:
            publish_frequency = float(self.get_parameter("publish_frequency").value)
        except Exception as e:
            self.get_logger().error("publish_frequency must be a positive number and will be automatically set to 1Hz")
            publish_frequency = 1

        self.number_ = num_to_pb 
        self.publisher_ = self.create_publisher(Int64, "number", 10) # create topic number in which number_publisher will publish an int
        self.timer = self.create_timer(1.0 / publish_frequency, self.publish_number) # publish 2 times per sec 
        self.get_logger().info("Number Publisher has been started")
    
    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.publisher_.publish(msg)
    

def main(args=None):
    rclpy.init(args=args) # first line to be written in any ROS2 .py node
    node = NumberPublisher()
    rclpy.spin(node) # allow node to continue to be alive
    rclpy.shutdown() # last line of any ROS2 .py node

if __name__ == "__main__":
    main()