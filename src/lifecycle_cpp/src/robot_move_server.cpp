#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/robot_move.hpp"

using RobotMove = my_robot_interfaces::action::RobotMove;
using RobotMoveGoalHandle = rclcpp_action::ServerGoalHandle<RobotMove>;
using namespace std::placeholders;

class RobotMoveServerNode : public rclcpp_lifecycle::LifecycleNode
{
public:
    RobotMoveServerNode() : LifecycleNode("move_robot_server")
    {
        robot_position_ = 50;
        activated_ = false;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_configure(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "IN on_configure");

        // Declare the action name parameter
        this->declare_parameter<std::string>("robot_name", "");
        // Get the action name parameter
        std::string action_name_prefix;
        this->get_parameter("robot_name", action_name_prefix);

        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        move_robot_server_ = rclcpp_action::create_server<RobotMove>(
            this,
            action_name_prefix + "/move_robot",
            std::bind(&RobotMoveServerNode::goal_callback, this, _1, _2),
            std::bind(&RobotMoveServerNode::cancel_callback, this, _1),
            std::bind(&RobotMoveServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_
        );
        activated_ = false;
        RCLCPP_INFO(this->get_logger(), "Action server has been started but it is not activated yet");
        RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "IN on_cleanup");
        this->undeclare_parameter("robot_name");
        cb_group_.reset();
        move_robot_server_.reset();
        activated_ = false;
        RCLCPP_INFO(this->get_logger(), "Action server has been turned off");
        RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "IN on_activate");
        activated_ = true;
        RCLCPP_INFO(this->get_logger(), "Action server has been activated");
        RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);
        rclcpp_lifecycle::LifecycleNode::on_activate(previous_state);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state)
    {
        RCLCPP_INFO(this->get_logger(), "IN on_deactivate");
        activated_ = false;
        RCLCPP_INFO(this->get_logger(), "Action server has been deactivated");
        RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);
        rclcpp_lifecycle::LifecycleNode::on_deactivate(previous_state);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "IN on_shutdown");
        this->undeclare_parameter("robot_name");
        cb_group_.reset();
        move_robot_server_.reset();
        activated_ = false;
        RCLCPP_INFO(this->get_logger(), "Action server has been completely shutdown");
        RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn on_error(const rclcpp_lifecycle::State &)
    {
        RCLCPP_INFO(this->get_logger(), "IN on_error");
        cb_group_.reset();
        move_robot_server_.reset();
        activated_ = false;
        RCLCPP_INFO(this->get_logger(), "Action server encountered an error");
        RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
    }

private:
    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const RobotMove::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received a new goal");

        if(!activated_)
        {
            RCLCPP_INFO(this->get_logger(), "Server is not activated yet, reject goal");
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Validate new goal
        if ((goal->position < 0) || (goal->position > 100) || (goal->velocity <= 0)) {
            RCLCPP_INFO(this->get_logger(), "Invalid position/velocity, reject goal");
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

    rclcpp_action::CancelResponse cancel_callback(
        const std::shared_ptr<RobotMoveGoalHandle> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received cancel request");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted_callback(
        const std::shared_ptr<RobotMoveGoalHandle> goal_handle)
    {
        execute_goal(goal_handle);
    }

    void execute_goal(
        const std::shared_ptr<RobotMoveGoalHandle> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            this->goal_handle_ = goal_handle;
        }

        int goal_position = goal_handle->get_goal()->position;
        int velocity = goal_handle->get_goal()->velocity;

        auto result = std::make_shared<RobotMove::Result>();
        auto feedback = std::make_shared<RobotMove::Feedback>();
        rclcpp::Rate loop_rate(1.0);

        RCLCPP_INFO(this->get_logger(), "Execute goal");
        while (rclcpp::ok()) {
            // Check if needs to preempt goal
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if (goal_handle->get_goal_id() == preempted_goal_id_) {
                    result->position = robot_position_;
                    result->message = "Preempted by another goal";
                    goal_handle->abort(result);
                    return;
                }
            }

            if(!activated_)
            {
                RCLCPP_INFO(this->get_logger(), "Server has been deactivated, cancel goal");
                result->position = robot_position_;
                result->message = "Server has been deactivated";
                goal_handle->abort(result);
                return;
            }

            // Check if cancel request
            if (goal_handle->is_canceling()) 
            {
                result->position = robot_position_;
                if (goal_position == robot_position_) 
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

            int diff = goal_position - robot_position_;

            if (diff == 0) 
            {
                result->position = robot_position_;
                result->message = "Success";
                goal_handle->succeed(result);
                return;
            }
            else if (diff > 0) 
            {
                if (diff < velocity) {
                    robot_position_ += diff;
                }
                else {
                    robot_position_ += velocity;
                }
            }
            else if (diff < 0) 
            {
                if (abs(diff) < velocity) 
                {
                    robot_position_ -= abs(diff);
                }
                else 
                {
                    robot_position_ -= velocity;
                }
            }

            RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);
            feedback->current_position = robot_position_;
            goal_handle->publish_feedback(feedback);

            loop_rate.sleep();
        }
    }

    rclcpp_action::Server<RobotMove>::SharedPtr move_robot_server_;
    rclcpp::CallbackGroup::SharedPtr cb_group_;
    int robot_position_;
    bool activated_;
    std::mutex mutex_;
    std::shared_ptr<RobotMoveGoalHandle> goal_handle_;
    rclcpp_action::GoalUUID preempted_goal_id_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotMoveServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();
    //rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}