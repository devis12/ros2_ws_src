#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor 
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
import time


class Node1(Node):
    def __init__(self):
        super().__init__("node1")

        self.cb_group1_ = MutuallyExclusiveCallbackGroup()
        self.cb_group23_ = MutuallyExclusiveCallbackGroup()
        self.timer1_ = self.create_timer(1.0, self.callback_timer1, callback_group=self.cb_group1_) # cannot enter two or more timer1_ cb at the same time
        self.timer2_ = self.create_timer(1.0, self.callback_timer2, callback_group=self.cb_group23_) # canont reenter and not run timer2_ cb and timer3_ cb at the same time
        self.timer3_ = self.create_timer(1.0, self.callback_timer3, callback_group=self.cb_group23_)

    def callback_timer1(self):
        time.sleep(2.0)
        self.get_logger().info("cb 1")

    def callback_timer2(self):
        time.sleep(2.0)
        self.get_logger().info("cb 2")

    def callback_timer3(self):
        time.sleep(2.0)
        self.get_logger().info("cb 3")

class Node2(Node):
    def __init__(self):
        super().__init__("node2")
        self.cb_group45_ = ReentrantCallbackGroup()
        self.timer4_ = self.create_timer(1.0, self.callback_timer4, callback_group=self.cb_group45_) # can reenter and run timer4_ cb and timer5_ cb at the same time
        self.timer5_ = self.create_timer(1.0, self.callback_timer5, callback_group=self.cb_group45_) # can reenter and run timer4_ cb and timer5_ cb at the same time

    def callback_timer4(self):
        time.sleep(2.0)
        self.get_logger().info("cb 4")

    def callback_timer5(self):
        time.sleep(2.0)
        self.get_logger().info("cb 5")

def main(args=None):
    rclpy.init(args=args)
    node1 = Node1()
    node2 = Node2()

    executor = MultiThreadedExecutor()
    executor.add_node(node1) # Add the node to the executor
    executor.add_node(node2) # Add the node to the executor
    executor.spin() # Keep spinning the executor
    
    rclpy.shutdown()


if __name__ == "__main__":
    main()