import rclpy
import time
import threading
from rclpy.node import Node
from rclpy.action import ActionServer
from my_robot_interfaces.action import CountUntil
from rclpy.action import CancelResponse, GoalResponse
from rclpy.action.server import ServerGoalHandle
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup

class CountUntilServerNode(Node):

    def __init__(self):
        super().__init__('count_until_server')
        self._action_server = ActionServer(
            self,
            CountUntil,
            'count_until',
            execute_callback=self.execute_callback,
            handle_accepted_callback=self.handle_accepted_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback,
            callback_group=ReentrantCallbackGroup())
        
        self._goal_lock = threading.Lock()
        self._goal_handle = None

        self._goal_queue = []

        self.get_logger().info('Action server ros2 node for count until has been started')

    def goal_callback(self, goal_request : CountUntil.Goal):
        self.get_logger().info('Received goal request with target number {0} and period {1}'.format(goal_request.target_number, goal_request.period))
        
        # Policy: refuse new goal if current goal still active
        # with self._goal_lock:
        #     if self._goal_handle is not None and self._goal_handle.is_active:
        #         self.get_logger().info('A goal is already active: goal request rejected')
        #         return GoalResponse.REJECT
        
        if goal_request.period < 0 or goal_request.target_number < 0:
            self.get_logger().info('Goal request rejected')
            return GoalResponse.REJECT

        # Policy: preempt currently active goal if any
        # with self._goal_lock:
        #     if self._goal_handle is not None and self._goal_handle.is_active:
        #         self.get_logger().info('Preempting the current goal active and accept new goal')
        #         self._goal_handle.abort()
        
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
        target_number = goal_handle.request.target_number
        period = goal_handle.request.period
        counter = 0
        
        feedback = CountUntil.Feedback()
        result = CountUntil.Result()
        # Execute the action
        for i in range(target_number):
            if not goal_handle.is_active:
                self.get_logger().info('Goal has been preempted')
                result.reached_number = counter
                self.process_next_goal_in_queue()
                return result
            
            elif goal_handle.is_cancel_requested:
                self.get_logger().info('Goal has been canceled')
                result.reached_number = counter
                goal_handle.canceled()
                self.process_next_goal_in_queue()
                return result
            
            counter += 1
            self.get_logger().info('Count: {0}'.format(counter))
            feedback.current_number = counter
            goal_handle.publish_feedback(feedback)
            time.sleep(period)

        # Once done, set goal final state
        goal_handle.succeed()

        # and send the result
        result.reached_number = counter
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
    action_server = CountUntilServerNode()
    rclpy.spin(action_server, MultiThreadedExecutor())
    rclpy.shutdown()

if __name__ == '__main__':
    main()