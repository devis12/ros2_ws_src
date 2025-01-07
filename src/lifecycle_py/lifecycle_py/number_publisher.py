#!/usr/bin/env python3
import rclpy
import math
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, LifecycleState, TransitionCallbackReturn

from example_interfaces.msg import Int64

class NumberPublisherNode(LifecycleNode):
    def __init__(self):
        super().__init__("number_publisher")
        self.number_ = 1
        self.publish_frequency_ = 1.0  
        self.number_publisher_ = None
        self.number_timer_ = None

    # Create ROS2 Communication, connect to HW
    def on_configure(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_configure")
        self.number_publisher_ = self.create_lifecycle_publisher(Int64, "number", 10) # not gonna be able to publish until we get to the active state
        self.number_timer_ = self.create_timer(
            1.0 / self.publish_frequency_, self.publish_number)
        self.number_timer_.cancel()
        
        # raise Exception("Error in on_configure") # This will trigger the on_error callback
        # return TrnaistionCallbackReturn.ERROR  # This will trigger the on_error callback
        return TransitionCallbackReturn.SUCCESS
        # return super().on_configure(previous_state)
        

    # Destroy ROS2 Communication, disconnect from HW
    def on_cleanup(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_cleanup")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS
        # return super().on_cleanup(state)

    def on_activate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_activate")
        self.number_timer_.reset()
        return super().on_activate(previous_state) # ALWAYS call the super() so that lifecyle publisher can be activated (not necessarily return its value)
    
    def on_deactivate(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_deactivate")
        self.number_timer_.cancel()
        return super().on_deactivate(previous_state) # ALWAYS call the super() so that lifecyle publisher can be deactivated (not necessarily return its value)

    def on_shutdown(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_shutdown")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        return TransitionCallbackReturn.SUCCESS

    def publish_number(self):
        msg = Int64()
        msg.data = self.number_
        self.number_publisher_.publish(msg)
        self.number_ += 1
    
    # Process error, deactivate and cleanup
    def on_error(self, previous_state: LifecycleState):
        self.get_logger().info("IN on_error")
        self.destroy_lifecycle_publisher(self.number_publisher_)
        self.destroy_timer(self.number_timer_)
        # do some checks, if ok return SUCCESS and go back to unconfigured, otherwise return FAILURE and it will go to finalized
        return TransitionCallbackReturn.SUCCESS

def main(args=None):
    rclpy.init(args=args)
    node = NumberPublisherNode()
    rclpy.spin(node)
    rclpy.shutdown()
