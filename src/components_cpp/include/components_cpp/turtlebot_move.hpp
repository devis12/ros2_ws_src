#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/turtle_move.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose2_d.hpp"
#include "turtlesim/msg/pose.hpp"

using TurtleBotMove = my_robot_interfaces::action::TurtleMove;
using TurtleBotMoveGoalHandle = rclcpp_action::ServerGoalHandle<TurtleBotMove>;
using namespace std::placeholders;

namespace my_namespace
{
    class TurtleBotMoveServer : public rclcpp_lifecycle::LifecycleNode
    {
    public:
        TurtleBotMoveServer(const rclcpp::NodeOptions& options);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &);

        rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State &);

    private:
        rclcpp_action::GoalResponse goal_callback(
            const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const TurtleBotMove::Goal> goal);

        rclcpp_action::CancelResponse cancel_callback(
            const std::shared_ptr<TurtleBotMoveGoalHandle> goal_handle);

        void handle_accepted_callback(
            const std::shared_ptr<TurtleBotMoveGoalHandle> goal_handle);

        void execute_goal(
            const std::shared_ptr<TurtleBotMoveGoalHandle> goal_handle);

        rclcpp_action::Server<TurtleBotMove>::SharedPtr move_turtle_server_;
        rclcpp::CallbackGroup::SharedPtr cb_group_;
        rclcpp_lifecycle::LifecyclePublisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;
        rclcpp::Subscription<turtlesim::msg::Pose>::SharedPtr pose_subscriber_;
        geometry_msgs::msg::Pose2D current_pose_;
        bool activated_;
        std::mutex mutex_;
        std::shared_ptr<TurtleBotMoveGoalHandle> goal_handle_;
        rclcpp_action::GoalUUID preempted_goal_id_;
    };

} // namespace my_namespace