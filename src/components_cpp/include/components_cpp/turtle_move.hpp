#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/turtle_move.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "turtlesim/msg/pose.hpp"

using TurtleMove = my_robot_interfaces::action::TurtleMove;
using TurtleMoveGoalHandle = rclcpp_action::ServerGoalHandle<TurtleMove>;
using namespace std::placeholders;

namespace my_namespace
{
    class TurtleMoveServer : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        TurtleMoveServer(const rclcpp::NodeOptions& options);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State &);

    private:
        rclcpp_action::GoalResponse goal_callback(
            const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const TurtleMove::Goal> goal);

        rclcpp_action::CancelResponse cancel_callback(
            const std::shared_ptr<TurtleMoveGoalHandle> goal_handle);

        void handle_accepted_callback(
            const std::shared_ptr<TurtleMoveGoalHandle> goal_handle);

        void execute_goal(
            const std::shared_ptr<TurtleMoveGoalHandle> goal_handle);

        rclcpp_action::Server<TurtleMove>::SharedPtr move_turtle_server_;
        rclcpp::CallbackGroup::SharedPtr cb_group_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
        geometry_msgs::msg::Pose2D current_pose_;
        bool activated_;
        std::mutex mutex_;
        std::shared_ptr<TurtleMoveGoalHandle> goal_handle_;
        rclcpp_action::GoalUUID preempted_goal_id_;
    };

} // namespace my_namespace