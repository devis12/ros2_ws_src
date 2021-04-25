#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

class MyNode(Node):

    def __init__(self):
        super().__init__("py_test") # Node is NOT the executable, therefore name of the node doesn't have to have same name of the file 
        self.start_ms = int(round(time.time() * 1000))
        self.counter = 0
        self.get_logger().info("Hello Py Node!!")
        self.create_timer(1, self.timer_callback) #1Hz
    
    def timer_callback(self):
        self.counter += 1
        ms_passed = int(round(time.time() * 1000)) - self.start_ms
        self.get_logger().info("{0} Hello, {1} ms from the start".format(self.counter, ms_passed))

def main(args=None):
    rclpy.init(args=args) # first line to be written in any ROS2 .py node
    node = MyNode()
    rclpy.spin(node) # allow node to continue to be alive
    rclpy.shutdown() # last line of any ROS2 .py node

if __name__ == "__main__":
    main()