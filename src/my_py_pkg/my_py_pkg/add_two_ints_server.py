#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

class AddTwoIntsServerNode(Node):

    def __init__(self):
        super().__init__("add_two_ints_server") # Node is NOT the executable, therefore name of the node doesn't have to have same name of the file 
        self.server_ = self.create_service(AddTwoInts, "add_two_ints", self.callback_add_two_ints)
        self.get_logger().info("Add two ints server has been started")
    
    def callback_add_two_ints(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info("{0} + {1} = {2}".format(request.a, request.b, response.sum))
        return response # if you forget to return response, you'll get an error when calling service

def main(args=None):
    rclpy.init(args=args) # first line to be written in any ROS2 .py node
    node = AddTwoIntsServerNode()
    rclpy.spin(node) # allow node to continue to be alive
    rclpy.shutdown() # last line of any ROS2 .py node

if __name__ == "__main__":
    main()