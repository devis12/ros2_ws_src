#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from functools import partial

from turtlesim.msg import Pose
from turtlesim_final_interfaces.msg import Turtle
from turtlesim_final_interfaces.msg import TurtleArray
from turtlesim_final_interfaces.srv import CatchTurtle
from geometry_msgs.msg import Twist

class ControllerTurtleSol(Node):

    def __init__(self):
        super().__init__("controller_turtle_sol")

        self.declare_parameter("catch_closest_turtle_first", True)

        self.turtle_to_catch_ = None
        self.catch_closest_turtle_first_ = self.get_parameter("catch_closest_turtle_first").value
        self.pose_ = None
        self.cmd_vel_publisher_ = self.create_publisher(Twist, "turtle1/cmd_vel", 10)
        self.pose_subscriber_ = self.create_subscription(Pose, "turtle1/pose", self.callback_turtle_pose, 10)
        self.alive_turtles_subscriber_ = self.create_subscription(TurtleArray, "alive_turtles", self.callback_alive_turtles, 10)
        self.control_loop_timer_ = self.create_timer(0.01, self.control_loop)
    
    def callback_turtle_pose(self, msg):
        self.pose_ = msg
    
    def callback_alive_turtles(self, msg):
        if(len(msg.turtles) > 0):
            if(self.catch_closest_turtle_first_):
                closest_turtle = None
                closest_turtle_distance = None 
                for turtle in msg.turtles:
                    dist_x = turtle.x - self.pose_.x
                    dist_y = turtle.y - self.pose_.y
                    distance = math.sqrt(dist_x**2 + dist_y**2)
                    if closest_turtle == None or distance < closest_turtle_distance:
                        closest_turtle = turtle
                        closest_turtle_distance = distance

                self.turtle_to_catch_ = closest_turtle
            else:
                self.turtle_to_catch_ = msg.turtles[0]

    def control_loop(self):
        if self.pose_ == None or self.turtle_to_catch_ == None:
            return
        
        dist_x = self.turtle_to_catch_.x - self.pose_.x
        dist_y = self.turtle_to_catch_.y - self.pose_.y
        distance = math.sqrt(dist_x**2 + dist_y**2)

        msg = Twist()

        if distance > 0.5:
            #position
            msg.linear.x = 2*distance

            #angle
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.pose_.theta
            if (diff > math.pi):
                diff -= 2*math.pi
            elif (diff < -math.pi):
                diff += 2*math.pi
            msg.angular.z = 6*diff

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.call_catch_turtle_server(self.turtle_to_catch_.name)
            self.turtle_to_catch_ = None
        
        self.cmd_vel_publisher_.publish(msg)
    
    def call_catch_turtle_server(self, turtle_name):
        client = self.create_client(CatchTurtle, "catch_turtle")

        while(not client.wait_for_service(1.0)):
            self.get_logger().warn("Waiting for server catch_turtle to be up")
        
        '''Service is up now'''
        request = CatchTurtle.Request()
        request.name = turtle_name

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_catch_turtle_server, turtle_name=turtle_name))

    def callback_catch_turtle_server(self, future, turtle_name):
        try:
            response = future.result()
            if not response.success:
                self.get_logger().error("Turtle "  + str(turtle_name) +  " could not be caught")
        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))

def main(args=None):
    
    rclpy.init(args=args)  # first line to be written in any ROS2 .py node
    node = ControllerTurtleSol()
    rclpy.spin(node)  # allow node to continue to be alive
    rclpy.shutdown()  # last line of any ROS2 .py node

if __name__ == "__main__":
    main()
