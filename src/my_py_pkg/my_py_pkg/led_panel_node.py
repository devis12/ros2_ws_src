#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from my_robot_interfaces.msg import LedStates
from my_robot_interfaces.srv import SetLed

class LedPanelNode(Node):

    def __init__(self):
        super().__init__("led_panel") 
        
        self.declare_parameter("led_states", [0, 0, 0])
        self.led_states_ = self.get_parameter("led_states").value

        self.led_states_publisher_ = self.create_publisher(LedStates, "led_states", 10) # create topic led_states in which the node will publish
        self.timer = self.create_timer(1.0, self.publish_led_states) # publish 2 times per sec 
        
        self.set_led_server_ = self.create_service(SetLed, "set_led", self.callback_set_led)

        self.get_logger().info("Led Node has been started")
    
    def publish_led_states(self):
        msg = LedStates()
        msg.led_states = self.led_states_
        self.led_states_publisher_.publish(msg)
    
    def callback_set_led(self, request, response):
        if(request.led_num >= 0 and request.led_num < len(self.led_states_)):
            self.led_states_[request.led_num] = request.state
            response.success = True
        else:
            response.success = False
        return response
    

def main(args=None):
    rclpy.init(args=args) # first line to be written in any ROS2 .py node
    node = LedPanelNode()
    rclpy.spin(node) # allow node to continue to be alive
    rclpy.shutdown() # last line of any ROS2 .py node

if __name__ == "__main__":
    main()