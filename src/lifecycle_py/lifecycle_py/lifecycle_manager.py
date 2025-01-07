#!/usr/bin/env python3
import time
import rclpy
from rclpy.node import Node
from lifecycle_msgs.srv import ChangeState
from lifecycle_msgs.msg import Transition
import rclpy.parameter
class LifecycleManager(Node):
    def __init__(self):
        super().__init__('lifecycle_manager')
        self.declare_parameter('managed_node_name', rclpy.Parameter.Type.STRING)
        node_name = self.get_parameter('managed_node_name').value
        service_change_state_name = "/" + node_name + "/change_state"
        self.client = self.create_client(ChangeState, service_change_state_name)

        self.get_logger().info('Lifecycle manager node for {0} has been started.'.format(node_name))

    def change_state(self, transition):
        self.client.wait_for_service()
        request = ChangeState.Request()
        request.transition = transition
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        if future.result() is not None:
            self.get_logger().info('Transition successful')
        else:
            self.get_logger().info('Transition failed')
    
    def initialization_sequence(self):
        # Unconfigured to inactive
        self.get_logger().info('Trying to switching to configuring')
        transition = Transition()
        transition.id = Transition.TRANSITION_CONFIGURE
        transition.label = 'configure'
        self.change_state(transition)
        self.get_logger().info('Configuring OK, now inactive')

        # sleep just for example purposes
        time.sleep(3)

        # Inactive to active
        self.get_logger().info('Trying to switching to activating')
        transition.id = Transition.TRANSITION_ACTIVATE
        transition.label = 'activate'
        self.change_state(transition)
        self.get_logger().info('Activating OK')


def main(args=None):
    rclpy.init(args=args)
    lifecycle_manager = LifecycleManager()
    lifecycle_manager.initialization_sequence()
    # rclpy.spin(lifecycle_manager)
    lifecycle_manager.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()