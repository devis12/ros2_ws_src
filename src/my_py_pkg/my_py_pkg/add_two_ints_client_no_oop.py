#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from example_interfaces.srv import AddTwoInts

def main(args=None):
    rclpy.init(args=args) # first line to be written in any ROS2 .py node
    node = Node("add_two_ints_no_oop")
    client = node.create_client(AddTwoInts, "add_two_ints")
    
    while(not client.wait_for_service(1.0)):
        node.get_logger().warn("Waiting for server add_two_ints to be up")
    
    '''Service is up now'''
    request = AddTwoInts.Request()
    request.a = 3
    request.b = 7

    future = client.call_async(request)
    rclpy.spin_until_future_complete(node, future)
    
    try:
        response = future.result()
        node.get_logger().info("Response is {0} for request \"{1} + {2}\"".format(response.sum, request.a, request.b))

    except Exception as e:
        node.get_logger().error("Service call failed %r" %(e,))


    rclpy.shutdown() # last line of any ROS2 .py node

if __name__ == "__main__":
    main()