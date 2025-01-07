import rclpy
import time
from rclpy.node import Node
from my_robot_interfaces.action import CountUntil
from rclpy.action import ActionClient
from action_msgs.msg import GoalStatus

class CountUntilActionClient(Node):

    def __init__(self):
        super().__init__('count_until_client')
        self._action_client = ActionClient(self, CountUntil, 'count_until')
        self._sent_goals = 0
        self._received_results = 0
        self._goal_handles = {}

    def send_goal(self, target_number, period, cancel=False):
        goal_msg = CountUntil.Goal()
        goal_msg.target_number = target_number
        goal_msg.period = period

        self._action_client.wait_for_server()
        goal_id = self._sent_goals
        self._sent_goals += 1
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(lambda future: self.goal_response_callback(future, goal_id))

        # Send a cancel request after 2 seconds
        if cancel:
            self._timer = self.create_timer(2.0, lambda: self.send_cancel_request(goal_id))
        
        
    def send_cancel_request(self, goal_id):
        if not self._goal_handles[goal_id] is None and self._goal_handles[goal_id].status == GoalStatus.STATUS_EXECUTING:
            self.get_logger().info(f'Sending cancel request with argument goal_id={goal_id}')
            self._goal_handles[goal_id].cancel_goal_async() # Cancel the goal

        self._timer.cancel() # Cancel the timer
        
    def goal_finished(self):
        self.get_logger().info('Goal finished!')
        self._received_results += 1
        if self._sent_goals == self._received_results:
            self.get_logger().info('All goals are done!')
            rclpy.shutdown()


    def goal_response_callback(self, future, goal_id):
        self._goal_handles[goal_id] = future.result()
        if not self._goal_handles[goal_id].accepted:
            self.get_logger().warn('Goal rejected :(')
            self.goal_finished()
            return

        self.get_logger().info('Goal accepted :)')
        self._get_result_future = self._goal_handles[goal_id].get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status

        if status == GoalStatus.STATUS_SUCCEEDED:
            self.get_logger().info('Goal succeeded!')
        elif status == GoalStatus.STATUS_CANCELED:
            self.get_logger().warn('Goal was canceled')
        elif status == GoalStatus.STATUS_ABORTED:
            self.get_logger().error('Goal was aborted')
        else:
            self.get_logger().warn('Goal failed with status: {0}'.format(status))
        
        self.get_logger().info('Result: {0}'.format(result.reached_number))
        
        self.goal_finished()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.current_number))

def main(args=None):
    rclpy.init(args=args)
    action_client = CountUntilActionClient()
    action_client.send_goal(6, 1.0, True)
    action_client.send_goal(10, 0.5)
    rclpy.spin(action_client)

if __name__ == '__main__':
    main()