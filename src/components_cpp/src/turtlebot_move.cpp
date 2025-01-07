#include "components_cpp/turtlebot_move.hpp"

#include "std_srvs/srv/empty.hpp"

using geometry_msgs::msg::Twist;
using geometry_msgs::msg::Pose2D;
using turtlesimPose = turtlesim::msg::Pose;

namespace my_namespace
{

static inline Pose2D initPose()
{
    auto pose = Pose2D{};
    pose.x = 5.544445;
    pose.y = 5.544445;
    pose.theta = 0.0;
    return pose;
}


TurtleBotMoveServer::TurtleBotMoveServer(const rclcpp::NodeOptions& options) : LifecycleNode("move_turtle_server", options)
{
    current_pose_ = initPose();
    activated_ = false;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleBotMoveServer::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "IN on_configure");

    // Spawn new turtle
    current_pose_ = initPose();

    // create cmd_vel publisher
    cmd_vel_publisher_ = this->create_publisher<Twist>("/cmd_vel", 10);

    // create pose subscriber
    // pose_subscriber_ = this->create_subscription<turtlesimPose>(
    //     "/pose", 10,
    //     [this](const turtlesimPose::SharedPtr msg) 
    //     {
    //         current_pose_.x = msg->x;
    //         current_pose_.y = msg->y;
    //         current_pose_.theta = msg->theta;
    //     }
    // );

    // Turn on the action server
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    move_turtle_server_ = rclcpp_action::create_server<TurtleBotMove>(
        this,
        "move_turtle",
        std::bind(&TurtleBotMoveServer::goal_callback, this, _1, _2),
        std::bind(&TurtleBotMoveServer::cancel_callback, this, _1),
        std::bind(&TurtleBotMoveServer::handle_accepted_callback, this, _1),
        rcl_action_server_get_default_options(),
        cb_group_
    );

    // Server still unactivated
    activated_ = false;

    RCLCPP_INFO(this->get_logger(), "Action server has been started but it is not activated yet");
    RCLCPP_INFO(this->get_logger(), "TurtleBot position: %.2f %.2f %.2f", current_pose_.x, current_pose_.y, current_pose_.theta);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleBotMoveServer::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "IN on_cleanup");

    // Cancel subscription
    pose_subscriber_.reset();

    // Cancel publisher
    cmd_vel_publisher_.reset();

    // Deactivate the server
    activated_ = false;

    // Reset action server
    cb_group_.reset();
    move_turtle_server_.reset();
    RCLCPP_INFO(this->get_logger(), "Action server has been turned off");
    RCLCPP_INFO(this->get_logger(), "TurtleBot position: %.2f %.2f %.2f", current_pose_.x, current_pose_.y, current_pose_.theta);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleBotMoveServer::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(this->get_logger(), "IN on_activate");
    activated_ = true;
    RCLCPP_INFO(this->get_logger(), "Action server has been activated");
    RCLCPP_INFO(this->get_logger(), "TurtleBot position: %.2f %.2f %.2f", current_pose_.x, current_pose_.y, current_pose_.theta);
    rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleBotMoveServer::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(this->get_logger(), "IN on_deactivate");
    activated_ = false;
    RCLCPP_INFO(this->get_logger(), "Action server has been deactivated");
    RCLCPP_INFO(this->get_logger(), "TurtleBot position: %.2f %.2f %.2f", current_pose_.x, current_pose_.y, current_pose_.theta);
    rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleBotMoveServer::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "IN on_shutdown");

    // Cancel subscription
    pose_subscriber_.reset();

    // Cancel publisher
    cmd_vel_publisher_.reset();

    // Deactivate the server
    activated_ = false;

    // Reset action server
    cb_group_.reset();
    move_turtle_server_.reset();
    RCLCPP_INFO(this->get_logger(), "Action server has been turned off");
    RCLCPP_INFO(this->get_logger(), "TurtleBot position: %.2f %.2f %.2f", current_pose_.x, current_pose_.y, current_pose_.theta);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleBotMoveServer::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "IN on_error");

    // Cancel subscription
    pose_subscriber_.reset();

    // Cancel publisher
    cmd_vel_publisher_.reset();

    // Deactivate the server
    activated_ = false;

    // Reset action server
    cb_group_.reset();
    move_turtle_server_.reset();
    RCLCPP_INFO(this->get_logger(), "Action server has been turned off");
    RCLCPP_INFO(this->get_logger(), "TurtleBot position: %.2f %.2f %.2f", current_pose_.x, current_pose_.y, current_pose_.theta);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse TurtleBotMoveServer::goal_callback(
    const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const TurtleBotMove::Goal> goal)
{
    (void)uuid;
    RCLCPP_INFO(this->get_logger(), "Received a new goal");

    if(!activated_)
    {
        RCLCPP_INFO(this->get_logger(), "Server is not activated yet, reject goal");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // Validate new goal
    if (goal->linear_vel_x == 0 && goal->angular_vel_z == 0)
    {
        RCLCPP_INFO(this->get_logger(), "Invalid velocities, reject goal");
        return rclcpp_action::GoalResponse::REJECT;
    }
    if(goal->duration <= 0)
    {
        RCLCPP_INFO(this->get_logger(), "Invalid duration, reject goal");
        return rclcpp_action::GoalResponse::REJECT;
    }

    // New goal is valid, preempt previous goal
    {
        std::lock_guard<std::mutex> lock(mutex_);
        if (goal_handle_) {
            if (goal_handle_->is_active()) {
                preempted_goal_id_ = goal_handle_->get_goal_id();
            }
        }
    }

    // Accept goal
    RCLCPP_INFO(this->get_logger(), "Accept goal");
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse TurtleBotMoveServer::cancel_callback(
    const std::shared_ptr<TurtleBotMoveGoalHandle> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TurtleBotMoveServer::handle_accepted_callback(
    const std::shared_ptr<TurtleBotMoveGoalHandle> goal_handle)
{
    execute_goal(goal_handle);
}

void TurtleBotMoveServer::execute_goal(
    const std::shared_ptr<TurtleBotMoveGoalHandle> goal_handle)
{
    {
        std::lock_guard<std::mutex> lock(mutex_);
        this->goal_handle_ = goal_handle;
    }

    double duration = goal_handle->get_goal()->duration;
    Twist move_twist;
    move_twist.linear.x = goal_handle->get_goal()->linear_vel_x; 
    move_twist.angular.z = goal_handle->get_goal()->angular_vel_z;
    Twist stop_twist;
    stop_twist.linear.x = 0.0;
    stop_twist.angular.z = 0.0;

    auto result = std::make_shared<TurtleBotMove::Result>();
    auto feedback = std::make_shared<TurtleBotMove::Feedback>();
    rclcpp::Rate loop_rate(4.0);
    rclcpp::Time start_time = this->now();
    
    RCLCPP_INFO(this->get_logger(), "Execute goal");
    while (rclcpp::ok()) {
        // Check if needs to preempt goal
        {
            std::lock_guard<std::mutex> lock(mutex_);
            if (goal_handle->get_goal_id() == preempted_goal_id_) {
                result->position = current_pose_;
                result->message = "Preempted by another goal";
                result->time_passed = (this->now() - start_time).seconds();
                cmd_vel_publisher_->publish(stop_twist);
                goal_handle->abort(result);
                return;
            }
        }

        if(!activated_)
        {
            RCLCPP_INFO(this->get_logger(), "Server has been deactivated, cancel goal");
            result->position = current_pose_;
            result->message = "Server has been deactivated";
            result->time_passed = (this->now() - start_time).seconds();
            cmd_vel_publisher_->publish(stop_twist);
            goal_handle->abort(result);
            return;
        }
        
        // Check if cancel request
        if (goal_handle->is_canceling()) 
        {
            result->position = current_pose_;
            result->time_passed = (this->now() - start_time).seconds();
            cmd_vel_publisher_->publish(stop_twist);
            if (result->time_passed >= duration) 
            {
                result->message = "Success";
                goal_handle->succeed(result);
            }
            else 
            {
                result->message = "Canceled";
                goal_handle->canceled(result);
            }
            return;
        }

        const auto time_passed = (this->now() - start_time).seconds();
        if (time_passed >= duration) 
        {
            result->position = current_pose_;
            result->message = "Success";
            result->time_passed = time_passed;
            cmd_vel_publisher_->publish(stop_twist);
            goal_handle->succeed(result);
            return;
        }

        // Keep publishing cmd_vel
        cmd_vel_publisher_->publish(move_twist);

        RCLCPP_INFO(this->get_logger(), "TurtleBot position: %.2f %.2f %.2f", current_pose_.x, current_pose_.y, current_pose_.theta);
        feedback->current_position = current_pose_;
        feedback->time_passed = time_passed;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
    }
}

}; // namespace my_namespace

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_namespace::TurtleBotMoveServer)

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<TurtleBotMoveServer>();
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node->get_node_base_interface());
//     executor.spin();
//     //rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }