import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from functools import partial
from my_robot_interfaces.action import Test

class TestActionClient(Node):

    def __init__(self):
        super().__init__('test_action_client')
        
        self.declare_parameter("secs", 2)
        self.secs_ = self.get_parameter("secs").value
        
        self.get_logger().info('Action Client and ready to request moving robots for {} secs'.format(self.secs_))

        self.action_client_ = None
        self.call_action_server()
    
    def destroy(self):
        self.action_client_.destroy()
        super().destroy_node()

    def call_action_server(self):
        self.action_client_ = ActionClient(self, Test, 'my_action_move')

        while(not self.action_client_.wait_for_server(1.0)):
            self.get_logger().warn("Waiting for server my_action_move to be up")

        goal_request = Test.Goal()
        goal_request.secs = self.secs_

        future = self.action_client_.send_goal_async(goal_request, feedback_callback=self.feedback_callback)

        future.add_done_callback(partial(self.callback_action_performed, secs=goal_request.secs))


    def feedback_callback(self, feedback_msg):
        self.get_logger().info("Feedback received")
        self.get_logger().info("Feedback is '{0}' for action \"my_action_move".format(feedback_msg.feedback))

    def callback_action_performed(self, future, secs):
        try:
            goal_handle = future.result()
            if not goal_handle.accepted:
                self.get_logger().info('Goal rejected :(')
                return

            self.get_logger().info('Goal accepted :)')

            self._get_result_future = goal_handle.get_result_async()
            self._get_result_future.add_done_callback(partial(self.get_result_callback, secs=secs))

        except Exception as e:
            self.get_logger().error("Action call failed %r" % (e,))

    def get_result_callback(self, future, secs):
        try:
            status = future.result().result.status
            self.get_logger().info("Final response is {0} for action \"my_action_move\ with secs={1}".format(status, secs))

            self.destroy()
            # Shutdown after receiving a result
            rclpy.shutdown()
            
        
        except Exception as e:
            self.get_logger().error("Action call response reading failed %r" % (e,))

def main(args=None):
    rclpy.init(args=args)
    test_action_client = TestActionClient()
    rclpy.spin(test_action_client)
    rclpy.shutdown() # last line of any ROS2 .py node

if __name__ == '__main__':
    main()