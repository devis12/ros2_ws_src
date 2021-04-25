#!/usr/bin/env python3

import random
import rclpy
from rclpy.node import Node

from functools import partial

from turtlesim.srv import Spawn
from turtlesim_final_interfaces.msg import TurtleSpawn

from turtlesim_final_utils import field_const

class SpawnTurtlesNode(Node):

    def __init__(self):
        super().__init__("spawn_turtles")

        self.declare_parameter("spawning_frequency", 0.25)
        spwn_frequency = self.get_parameter("spawning_frequency").value

        self.counter_ = 2
        self.timer_ = self.create_timer(1.0 / spwn_frequency, self.spawn_turtle) 
        self.publisher_ = self.create_publisher(TurtleSpawn, "spawned_turtles", 10) # create topic spawned_turtles in which spawn_turtles will publish after successfull spawning

        self.get_logger().info("Turtle spawner up")

    def get_random_num(self, min, max):
        return round(random.uniform(min, max), 3)

    def get_random_X(self):
        return self.get_random_num(field_const.MIN_X, field_const.MAX_X)
    
    def get_random_Y(self):
        return self.get_random_num(field_const.MIN_Y, field_const.MAX_Y)
    
    def get_random_THETA(self):
        return self.get_random_num(field_const.MIN_THETA, field_const.MAX_THETA)

    def spawn_turtle(self):
        
        turtle_x = self.get_random_X()
        turtle_y = self.get_random_Y()
        turtle_theta = self.get_random_THETA()
        self.get_logger().info("Spawning 'turtle{0}' in x: {1}, y: {2}, theta: {3}".format(self.counter_, turtle_x, turtle_y, turtle_theta))

        client = self.create_client(Spawn, "spawn")

        while(not client.wait_for_service(1.0)):
            self.get_logger().warn("Waiting for server spawn to be up")
        
        '''Service is up now'''
        request = Spawn.Request()
        request.x = turtle_x
        request.y = turtle_y
        request.theta = turtle_theta
        request.name = 'turtle' + str(self.counter_)

        self.counter_ += 1

        future = client.call_async(request)
        future.add_done_callback(partial(self.callback_spawn_turtle, x=turtle_x, y=turtle_y, theta=turtle_theta, name=request.name))

    def callback_spawn_turtle(self, future, x, y, theta, name):
        try:
            response = future.result()
            if(response.name == name):
                #self.get_logger().info("Response is '{0}' for request '{1}' in x: {2}, y: {3}, theta: {4}".format(response.name, name, x, y, theta))
                
                msg = TurtleSpawn()
                msg.x = x
                msg.y = y
                msg.theta = theta
                msg.name = name
                self.publisher_.publish(msg)

            else:
                self.get_logger().error(
                    "Something unexpected occured during the spawning of '{0}' in x: {1}, y: {2}, theta: {3}".format(response.name, name, x, y, theta))

        except Exception as e:
            self.get_logger().error("Service call failed %r" % (e,))


def main(args=None):
    
    rclpy.init(args=args)  # first line to be written in any ROS2 .py node
    node = SpawnTurtlesNode()
    rclpy.spin(node)  # allow node to continue to be alive
    rclpy.shutdown()  # last line of any ROS2 .py node

if __name__ == "__main__":
    main()
