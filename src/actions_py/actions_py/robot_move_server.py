import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer
from my_robot_interfaces.action import RobotMove
from rclpy.action import CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle, GoalStatus
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class RobotMoveServerNode(Node):

    def __init__(self):
        super().__init__('robot_move_server')
        self._action_server = ActionServer(
            self,
            RobotMove,
            'robot_move',
            execute_callback=self.execute_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())
        
        self._current_position = 50

        self._goal_lock = threading.Lock()
        self._goal_handle = None

        self._goal_queue = []

        self.get_logger().info('Action server ros2 node for robot move has been started')

    def goal_callback(self, goal_request : RobotMove.Goal):
        self.get_logger().info('Received goal request with target position {0} and velocity {1}'.format(goal_request.position, goal_request.velocity))
        
        # Policy: refuse new goal if current goal still active
        # with self._goal_lock:
        #     if self._goal_handle is not None and self._goal_handle.is_active:
        #         self.get_logger().info('A goal is already active: goal request rejected')
        #         return GoalResponse.REJECT
        
        if goal_request.velocity <= 0 or goal_request.position < 0:
            self.get_logger().info('Goal request rejected: invalid goal request')
            return GoalResponse.REJECT

        # Policy: preempt currently active goal if any
        with self._goal_lock:
            if self._goal_handle is not None and self._goal_handle.is_active and self._goal_handle.status == GoalStatus.STATUS_EXECUTING:
                self.get_logger().info('Preempting the current goal active and accept new goal')
                self._goal_handle.abort()
        
        return GoalResponse.ACCEPT

    def handle_accepted_callback(self, goal_handle : ServerGoalHandle):
        self.get_logger().info('Goal has been accepted')
        with self._goal_lock:
            if self._goal_handle is not None:
                self._goal_queue.append(goal_handle)
                self.get_logger().info('Pushing to the queue accepted goal {0}, new queue size = {1}'.format(goal_handle.request, len(self._goal_queue)))
            else:
                self._goal_handle = goal_handle
                goal_handle.execute()

    def cancel_callback(self, goal_handle : ServerGoalHandle):
        self.get_logger().info('Received cancel request')
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle : ServerGoalHandle):
        self.get_logger().info('Executing goal {0}...'.format(goal_handle.request))
        
        # Get the goal request
        goal_position = goal_handle.request.position
        velocity = goal_handle.request.velocity
        if(goal_position < self._current_position):
            velocity = -velocity
        
        feedback = RobotMove.Feedback()
        result = RobotMove.Result()

        # Execute the action
        while self._current_position != goal_position:
            if not goal_handle.is_active:
                self.get_logger().info('Goal has been preempted')
                result.position = self._current_position
                result.message = 'Goal has been preempted'
                self.process_next_goal_in_queue()
                return result
            
            elif goal_handle.is_cancel_requested:
                self.get_logger().info('Goal has been canceled')
                result.position = self._current_position
                result.message = 'Goal has been preempted for cancel request'
                goal_handle.canceled()
                self.process_next_goal_in_queue()
                return result
            
            self._current_position += velocity
            if (self._current_position > goal_position and velocity > 0) or (self._current_position < goal_position and velocity < 0):
                self._current_position = goal_position

            self.get_logger().info('Current position: {0}'.format(self._current_position))
            feedback.current_position = self._current_position
            goal_handle.publish_feedback(feedback)
            time.sleep(1.0)

        # Once done, set goal final state
        if goal_handle.is_active:
            goal_handle.succeed()

        # and send the result
        result.position = self._current_position
        result.message = 'Goal succeeded: position reached'
        self.process_next_goal_in_queue()
        return result

    def process_next_goal_in_queue(self):
        with self._goal_lock:
            if len(self._goal_queue) > 0:
                self._goal_handle = self._goal_queue.pop(0)
                self.get_logger().info('Processing next goal in the queue {0}, new queue size = {1}'.format(self._goal_handle.request, len(self._goal_queue)))
                self._goal_handle.execute()
            else:
                self._goal_handle = None

def main(args=None):
    rclpy.init(args=args)
    action_server = RobotMoveServerNode()
    rclpy.spin(action_server, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()