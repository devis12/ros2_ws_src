#include "components_cpp/turtle_move.hpp"

#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "std_srvs/srv/empty.hpp"

#define ROBOT_NAME_PARAMETER "turtle_name"

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

bool spawnTurtle(const std::string& name, const Pose2D& pose)
{
    auto client_node = rclcpp::Node::make_shared("spawn_" + name + "_client");
    auto service = client_node->create_client<turtlesim::srv::Spawn>("spawn");
    auto request = std::make_shared<turtlesim::srv::Spawn::Request>();
    request->name = name;
    request->x = pose.x;
    request->y = pose.y;
    request->theta = pose.theta;
    auto result = service->async_send_request(request);
    if (rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS) 
    {
        return true;
    }
    return false;
}

bool killTurtle(const std::string& name, bool clear = false)
{
    auto client_node = rclcpp::Node::make_shared("kill_" + name + "_client");
    auto service = client_node->create_client<turtlesim::srv::Kill>("kill");
    auto request = std::make_shared<turtlesim::srv::Kill::Request>();
    request->name = name;
    auto result = service->async_send_request(request);
    if (rclcpp::spin_until_future_complete(client_node, result) == rclcpp::FutureReturnCode::SUCCESS) 
    {
        if(clear)
        {
            auto clear_service = client_node->create_client<std_srvs::srv::Empty>("clear");
            auto clear_request = std::make_shared<std_srvs::srv::Empty::Request>();
            auto clear_result = clear_service->async_send_request(clear_request);
            rclcpp::spin_until_future_complete(client_node, clear_result); // don't care about the result of this
        }
        return true;
    }
    return false;
}

TurtleMoveServer::TurtleMoveServer(const rclcpp::NodeOptions& options) : LifecycleNode("move_turtle_server", options)
{
    current_pose_ = initPose();
    activated_ = false;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleMoveServer::on_configure(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "IN on_configure");

    // Declare the action name parameter
    this->declare_parameter<std::string>(ROBOT_NAME_PARAMETER, "turtle1");
    // Get the action name parameter
    std::string turtle_name;
    this->get_parameter(ROBOT_NAME_PARAMETER, turtle_name);

    // Spawn new turtle
    current_pose_ = initPose();
    if(!spawnTurtle(turtle_name, current_pose_))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to spawn %s", turtle_name.c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    // create cmd_vel publisher
    cmd_vel_publisher_ = this->create_publisher<Twist>(turtle_name + "/cmd_vel", 10);

    // create pose subscriber
    pose_subscriber_ = this->create_subscription<turtlesimPose>(
        turtle_name + "/pose", 10,
        [this](const turtlesimPose::SharedPtr msg) 
        {
            current_pose_.x = msg->x;
            current_pose_.y = msg->y;
            current_pose_.theta = msg->theta;
        }
    );

    // Turn on the action server
    cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
    move_turtle_server_ = rclcpp_action::create_server<TurtleMove>(
        this,
        "move_turtle" + (turtle_name.length() > 0 ? "_" + turtle_name : ""),
        std::bind(&TurtleMoveServer::goal_callback, this, _1, _2),
        std::bind(&TurtleMoveServer::cancel_callback, this, _1),
        std::bind(&TurtleMoveServer::handle_accepted_callback, this, _1),
        rcl_action_server_get_default_options(),
        cb_group_
    );

    // Server still unactivated
    activated_ = false;

    RCLCPP_INFO(this->get_logger(), "Action server has been started but it is not activated yet");
    RCLCPP_INFO(this->get_logger(), "Turtle position: %.2f %.2f %.2f", current_pose_.x, current_pose_.y, current_pose_.theta);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleMoveServer::on_cleanup(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "IN on_cleanup");
    // Get the action name parameter
    std::string turtle_name;
    this->get_parameter(ROBOT_NAME_PARAMETER, turtle_name);

    // Despawn the turtle
    if(!killTurtle(turtle_name))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to kill %s", turtle_name.c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // Cancel subscription
    pose_subscriber_.reset();

    // Cancel publisher
    cmd_vel_publisher_.reset();

    // Undeclare parameter
    if(this->has_parameter(ROBOT_NAME_PARAMETER)) this->undeclare_parameter(ROBOT_NAME_PARAMETER);

    // Deactivate the server
    activated_ = false;

    // Reset action server
    cb_group_.reset();
    move_turtle_server_.reset();
    RCLCPP_INFO(this->get_logger(), "Action server has been turned off");
    RCLCPP_INFO(this->get_logger(), "Turtle position: %.2f %.2f %.2f", current_pose_.x, current_pose_.y, current_pose_.theta);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleMoveServer::on_activate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(this->get_logger(), "IN on_activate");
    activated_ = true;
    RCLCPP_INFO(this->get_logger(), "Action server has been activated");
    RCLCPP_INFO(this->get_logger(), "Turtle position: %.2f %.2f %.2f", current_pose_.x, current_pose_.y, current_pose_.theta);
    rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleMoveServer::on_deactivate(const rclcpp_lifecycle::State& previous_state)
{
    RCLCPP_INFO(this->get_logger(), "IN on_deactivate");
    activated_ = false;
    RCLCPP_INFO(this->get_logger(), "Action server has been deactivated");
    RCLCPP_INFO(this->get_logger(), "Turtle position: %.2f %.2f %.2f", current_pose_.x, current_pose_.y, current_pose_.theta);
    rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleMoveServer::on_shutdown(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "IN on_shutdown");
    // Get the action name parameter
    std::string turtle_name;
    this->get_parameter(ROBOT_NAME_PARAMETER, turtle_name);

    // Despawn the turtle
    if(!killTurtle(turtle_name))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to kill %s", turtle_name.c_str());
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::ERROR;
    }

    // Cancel subscription
    pose_subscriber_.reset();

    // Cancel publisher
    cmd_vel_publisher_.reset();

    // Undeclare parameter
    if(this->has_parameter(ROBOT_NAME_PARAMETER)) this->undeclare_parameter(ROBOT_NAME_PARAMETER);

    // Deactivate the server
    activated_ = false;

    // Reset action server
    cb_group_.reset();
    move_turtle_server_.reset();
    RCLCPP_INFO(this->get_logger(), "Action server has been turned off");
    RCLCPP_INFO(this->get_logger(), "Turtle position: %.2f %.2f %.2f", current_pose_.x, current_pose_.y, current_pose_.theta);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn TurtleMoveServer::on_error(const rclcpp_lifecycle::State &)
{
    RCLCPP_INFO(this->get_logger(), "IN on_error");
    // Get the action name parameter
    std::string turtle_name;
    this->get_parameter(ROBOT_NAME_PARAMETER, turtle_name);

    // Despawn the turtle
    if(!killTurtle(turtle_name))
    {
        RCLCPP_ERROR(this->get_logger(), "Failed to kill %s", turtle_name.c_str());
    }

    // Cancel subscription
    pose_subscriber_.reset();

    // Cancel publisher
    cmd_vel_publisher_.reset();

    // Undeclare parameter
    if(this->has_parameter(ROBOT_NAME_PARAMETER)) this->undeclare_parameter(ROBOT_NAME_PARAMETER);

    // Deactivate the server
    activated_ = false;

    // Reset action server
    cb_group_.reset();
    move_turtle_server_.reset();
    RCLCPP_INFO(this->get_logger(), "Action server has been turned off");
    RCLCPP_INFO(this->get_logger(), "Turtle position: %.2f %.2f %.2f", current_pose_.x, current_pose_.y, current_pose_.theta);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_action::GoalResponse TurtleMoveServer::goal_callback(
    const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const TurtleMove::Goal> goal)
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

rclcpp_action::CancelResponse TurtleMoveServer::cancel_callback(
    const std::shared_ptr<TurtleMoveGoalHandle> goal_handle)
{
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received cancel request");
    return rclcpp_action::CancelResponse::ACCEPT;
}

void TurtleMoveServer::handle_accepted_callback(
    const std::shared_ptr<TurtleMoveGoalHandle> goal_handle)
{
    execute_goal(goal_handle);
}

void TurtleMoveServer::execute_goal(
    const std::shared_ptr<TurtleMoveGoalHandle> goal_handle)
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

    auto result = std::make_shared<TurtleMove::Result>();
    auto feedback = std::make_shared<TurtleMove::Feedback>();
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

        RCLCPP_INFO(this->get_logger(), "Turtle position: %.2f %.2f %.2f", current_pose_.x, current_pose_.y, current_pose_.theta);
        feedback->current_position = current_pose_;
        feedback->time_passed = time_passed;
        goal_handle->publish_feedback(feedback);

        loop_rate.sleep();
    }
}

}; // namespace my_namespace

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(my_namespace::TurtleMoveServer)

// int main(int argc, char **argv)
// {
//     rclcpp::init(argc, argv);
//     auto node = std::make_shared<TurtleMoveServer>();
//     rclcpp::executors::MultiThreadedExecutor executor;
//     executor.add_node(node->get_node_base_interface());
//     executor.spin();
//     //rclcpp::spin(node);
//     rclcpp::shutdown();
//     return 0;
// }