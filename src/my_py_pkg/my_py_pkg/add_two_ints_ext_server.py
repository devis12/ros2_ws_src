#!/usr/bin/env python3

import rclpy
import threading
from rclpy.node import Node

from functools import partial
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup

from example_interfaces.srv import AddTwoInts

class AddTwoIntsExtServerNode(Node):

    def __init__(self):
        super().__init__("add_two_ints_ext_server") # Node is NOT the executable, therefore name of the node doesn't have to have same name of the file 
        self.server_ = self.create_service(AddTwoInts, "add_two_ints_ext", self.ext_srv_callback_add_two_ints)
        self.private_callback_group = MutuallyExclusiveCallbackGroup()
        self.response = AddTwoInts.Response()
        self.processing_response = False
        self.get_logger().info("Add two ints ext server has been started")
    
    def ext_srv_callback_add_two_ints(self, request, response):
        client = self.create_client(AddTwoInts, "add_two_ints", callback_group=self.private_callback_group)

        while(not client.wait_for_service(1.0)):
            self.get_logger().warn("Waiting for server add_two_ints to be up")

        #Forward the request
        future = client.call_async(request)
        self.processing_response = True
        self.get_logger().info("Making call from ext server to server add_two_ints")
        future.add_done_callback(self.callback_add_two_ints)
        
        rate = self.create_rate(2)
        while rclpy.ok() and self.processing_response:
            self.get_logger().info("Add two ints ext server waiting for an answer from server add_two_ints")
            rate.sleep()

        self.get_logger().info("Received response in ext server from server add_two_ints")

        return self.response # if you forget to return response, you'll get an error when calling service

    def callback_add_two_ints(self, future):
        try:
            response = future.result()
            self.get_logger().info(
                "Response READY to be forwarded back: {0} !\"".format(response.sum))
            self.response = response
            self.processing_response = False
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args) # first line to be written in any ROS2 .py node
    executor = MultiThreadedExecutor()
    executor.add_node(AddTwoIntsExtServerNode())

    try:
      executor.spin()
    finally:
      executor.shutdown()

if __name__ == "__main__":
    main()