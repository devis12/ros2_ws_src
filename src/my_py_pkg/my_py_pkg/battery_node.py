#!/usr/bin/env python3

import time
import rclpy
from rclpy.node import Node

from functools import partial

from my_robot_interfaces.srv import SetLed


class BatteryNode(Node):

    def __init__(self):
        super().__init__("battery")
        self.charge_ = 6.0
        self.recharge_ = False
        self.timer_ = self.create_timer(1.0, self.update_battery) 

    def update_battery(self):
        if(self.recharge_):
            self.charge_ += 1.0
        else:
            self.charge_ -= 1.5
        
        self.get_logger().info("Battery status: {}".format(self.charge_))

        if(self.charge_ == 6.0):
            self.recharge_ = False # charge complete
            self.get_logger().info("Battery fully charge: request for turning off led")
            self.call_set_led_server(2, self.recharge_)

        elif(self.charge_ == 0.0):
            self.recharge_ = True # recharge
            self.get_logger().info("Battery recharging: request for turning on led")
            self.call_set_led_server(2, self.recharge_)

    def call_set_led_server(self, led_num, led_state):
        client = self.create_client(SetLed, "set_led")

        while(not client.wait_for_service(1.0)):
            self.get_logger().warn("Waiting for server set_led to be up")
        
        '''Service is up now'''
        request = SetLed.Request()
        request.led_num = led_num
        request.state = led_state

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_set_led, led_num=led_num, led_state=led_state))

    def callback_set_led(self, future, led_num, led_state):
        try:
            response = future.result()
            self.get_logger().info(
                "Response is {0} for request \"led_num: {1}, state: {2}\"".format(response.success, led_num, led_state))

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    
    rclpy.init(args=args)  # first line to be written in any ROS2 .py node
    node = BatteryNode()
    rclpy.spin(node)  # allow node to continue to be alive
    rclpy.shutdown()  # last line of any ROS2 .py node

if __name__ == "__main__":
    main()
