#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node

from functools import partial

from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
from turtlesim_final_interfaces.msg import TurtleSpawn

from geometry_msgs.msg import Twist

from turtlesim_final_utils import field_const

def compute_distance(t1, t2):
    return math.sqrt( ((t1["x"] - t2["x"])**2) + ((t1["y"] -t2["y"])**2))

def compute_target_theta(x1, x2, y1, y2):
    c1 = abs(x1 - x2)
    c2 = abs(y1 - y2)
    i = math.sqrt(c1**2 + c2**2)
    th = 0.0
    if(x1 < x2):
        th = math.acos(c1/i) if (y1 < y2) else -math.acos(c1/i)
    else:
        th = math.pi + (-math.acos(c1/i) if (y1 < y2) else math.acos(c1/i))
    return th

class ControllerTurtle(Node):

    def __init__(self):
        super().__init__("controller_turtle")

        self.declare_parameter("turtle_controlled", "turtle1")
        self.declare_parameter("step", 0.08)
        self.STEP_ = self.get_parameter("step").value
        self.STEP_T_ = 0.08
        self.turtle_controlled_ = self.get_parameter("turtle_controlled").value

        self.turtles_ = [] #array of turtles waiting to be eaten
        self.target_acquired_ = {} #indicating if the turtle is currently active toward a specific target
        self.eating_ = False # currently eating another turtle
        self.current_position_ = {} #indicating current position of turtle controlled listened throgh topic /<turtle_controlled>/pose

        self.spawned_subscriber_ = self.create_subscription(TurtleSpawn, "spawned_turtles", self.callback_spawned_turtles, 10)
        self.position_subscriber_ = self.create_subscription(Pose, str(self.turtle_controlled_)+"/pose", self.callback_position, 10)
        self.cmd_vel_publisher_ = self.create_publisher(Twist, str(self.turtle_controlled_)+"/cmd_vel", 10)

        self.controller_timer_ = self.create_timer(1.0/16.0, self.move_towards_target) 
        
        self.get_logger().info("Controller for {} up".format(self.turtle_controlled_))
    
    def callback_position(self, msg):
        self.current_position_ = {
            "x": msg.x,
            "y": msg.y,
            "theta": msg.theta,
            "linear_velocity": msg.linear_velocity,
            "angular_velocity": msg.angular_velocity
        }

        #self.get_logger().info("Controller for {0}, received position x: {1} , y {2}".format(self.turtle_controlled_, self.current_position_["x"], self.current_position_["y"]))

    def callback_spawned_turtles(self, msg):

        self.turtles_.append({
            "name": msg.name,
            "x": msg.x,
            "y": msg.y,
            "theta": msg.theta
        })
        #self.get_logger().info("Received from spawned_turtles: spawned {0} in x: {1}, y: {2}, theta: {3}".format(msg.name, msg.x, msg.y, msg.theta))
        
        if bool(self.current_position_) and not bool(self.target_acquired_): # acquired current position and no current target
            self.acquire_target()

    def acquire_target(self):
        #'''find closest target in turtles_'''
        min_d = field_const.MAX_X * 2
        for turtle in self.turtles_:
            d = compute_distance(self.current_position_, turtle)
            if not self.target_acquired_ or d < min_d:
                self.target_acquired_ = turtle
                min_d = d
        
        if self.target_acquired_:
            self.turtles_.remove(self.target_acquired_)

    def move_towards_target(self):
        if self.current_position_ == {} or self.target_acquired_ == {}:
            return
        
        dist_x = self.target_acquired_["x"] - self.current_position_["x"]
        dist_y = self.target_acquired_["y"] - self.current_position_["y"]
        distance = compute_distance(self.current_position_, self.target_acquired_)

        msg = Twist()

        if distance > self.STEP_*2:
            #position
            msg.linear.x = 2*distance

            #angle
            goal_theta = math.atan2(dist_y, dist_x)
            diff = goal_theta - self.current_position_["theta"]
            if (diff > math.pi):
                diff -= 2*math.pi
            elif (diff < -math.pi):
                diff += 2*math.pi
            msg.angular.z = 6*diff

        else:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            if (not self.eating_) and bool(self.current_position_) and bool(self.target_acquired_):
                self.eat_target()
        
        self.cmd_vel_publisher_.publish(msg)

    '''
    def move_towards_target(self):
        if bool(self.current_position_) and bool(self.target_acquired_) and compute_distance(self.current_position_, self.target_acquired_) > self.STEP_*2:
        
            client = self.create_client(TeleportAbsolute, str(self.turtle_controlled_)+"/teleport_absolute")

            while(not client.wait_for_service(1.0)):
                self.get_logger().warn("Waiting for server " + str(self.turtle_controlled_)+"/teleport_absolute" + " to be up")
            
            #Service is up now
            request = TeleportAbsolute.Request()
            
            target_theta = compute_target_theta(self.current_position_["x"], self.target_acquired_["x"], self.current_position_["y"], self.target_acquired_["y"])
            
            step_t = self.STEP_T_ if(self.current_position_["theta"] < target_theta) else -self.STEP_T_
            if(abs(self.current_position_["theta"] - target_theta) < self.STEP_T_*1.5):
                step_t = 0

            step_x = self.STEP_ if ((self.current_position_["x"] - self.target_acquired_["x"]) < 0) else -self.STEP_
            if(abs(self.current_position_["x"] - self.target_acquired_["x"]) < self.STEP_):
                step_x = 0

            step_y = self.STEP_ if ((self.current_position_["y"] - self.target_acquired_["y"]) < 0) else -self.STEP_
            if(abs(self.current_position_["y"] - self.target_acquired_["y"]) < self.STEP_):
                step_y = 0

            request.x = self.current_position_["x"] + step_x
            request.y = self.current_position_["y"] + step_y
            request.theta = (self.current_position_["theta"] + step_t) if abs(step_t) > 0 else target_theta

            future = client.call_async(request)
            future.add_done_callback(self.callback_move_towards_target)
        
        elif (not self.eating_) and bool(self.current_position_) and bool(self.target_acquired_):
            self.eat_target()
            #self.target_acquired_ = {}
            pass
    
    def callback_move_towards_target(self, future):
        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error("Something unexpected occured during teleport_absolute movement")
    '''

    def eat_target(self):
        if (not self.eating_) and bool(self.current_position_) and bool(self.target_acquired_) and compute_distance(self.current_position_, self.target_acquired_) <= self.STEP_*2:
            self.eating_ = True
            client = self.create_client(Kill, "kill")

            while(not client.wait_for_service(1.0)):
                self.get_logger().warn("Waiting for server kill to be up")

            '''Service is up now'''
            request = Kill.Request()
            request.name = self.target_acquired_["name"]

            future = client.call_async(request)
            future.add_done_callback(partial(self.callback_eat_target, name=request.name))
    
    def callback_eat_target(self, future, name):
        try:
            response = future.result()
            self.target_acquired_ = {}
            self.eating_ = False
            self.acquire_target()
        except Exception as e:
            self.get_logger().error("Something unexpected occured during kill of " + str(name))

def main(args=None):
    
    rclpy.init(args=args)  # first line to be written in any ROS2 .py node
    node = ControllerTurtle()
    rclpy.spin(node)  # allow node to continue to be alive
    rclpy.shutdown()  # last line of any ROS2 .py node

if __name__ == "__main__":
    main()
