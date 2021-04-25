#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from functools import partial

from example_interfaces.srv import AddTwoInts


class AddTwoIntsClientNode(Node):

    def __init__(self):
        super().__init__("add_two_ints_client")
        self.ask_call_loop()

    def ask_call_loop(self):
        try:
            print("Insert a: ", end="")
            self.a_ = int(input())
            print("Insert b: ", end="")
            self.b_ = int(input())

            self.call_two_ints_server(self.a_, self.b_)
        except Exception as e:
            print(e)

    def call_two_ints_server(self, a, b):
        client = self.create_client(AddTwoInts, "add_two_ints")

        while(not client.wait_for_service(1.0)):
            self.get_logger().warn("Waiting for server add_two_ints to be up")

        '''Service is up now'''
        request = AddTwoInts.Request()
        request.a = a
        request.b = b

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_add_two_ints, a=a, b=b))

    def callback_add_two_ints(self, future, a, b):
        try:
            response = future.result()
            self.get_logger().info(
                "Response is {0} for request \"{1} + {2}\"".format(response.sum, a, b))

            self.ask_call_loop()

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    
    rclpy.init(args=args)  # first line to be written in any ROS2 .py node
    node = AddTwoIntsClientNode()
    rclpy.spin(node)  # allow node to continue to be alive
    rclpy.shutdown()  # last line of any ROS2 .py node

if __name__ == "__main__":
    main()
