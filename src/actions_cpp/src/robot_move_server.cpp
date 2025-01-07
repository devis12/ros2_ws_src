#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/robot_move.hpp"

using RobotMove = my_robot_interfaces::action::RobotMove;
using RobotMoveGoalHandle = rclcpp_action::ServerGoalHandle<RobotMove>;
using namespace std::placeholders;

class RobotMoveServerNode : public rclcpp::Node
{
public:
    RobotMoveServerNode() : Node("move_robot_server")
    {
        robot_position_ = 50;
        cb_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);
        move_robot_server_ = rclcpp_action::create_server<RobotMove>(
            this,
            "move_robot",
            std::bind(&RobotMoveServerNode::goal_callback, this, _1, _2),
            std::bind(&RobotMoveServerNode::cancel_callback, this, _1),
            std::bind(&RobotMoveServerNode::handle_accepted_callback, this, _1),
            rcl_action_server_get_default_options(),
            cb_group_
            );
        RCLCPP_INFO(this->get_logger(), "Action server has been started");
        RCLCPP_INFO(this->get_logger(), "Robot position: %d", robot_position_);
    }

private:
    rclcpp_action::GoalResponse goal_callback(
        const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const RobotMove::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(), "Received a new goal");

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

            // Check if cancel request
            if (goal_handle->is_canceling()) {
                result->position = robot_position_;
                if (goal_position == robot_position_) {
                   result->message = "Success";
                   goal_handle->succeed(result); 
                }
                else {
                    result->message = "Canceled";
                    goal_handle->canceled(result);
                }
                return;
            }

            int diff = goal_position - robot_position_;

            if (diff == 0) {
                result->position = robot_position_;
                result->message = "Success";
                goal_handle->succeed(result);
                return;
            }
            else if (diff > 0) {
                if (diff < velocity) {
                    robot_position_ += diff;
                }
                else {
                    robot_position_ += velocity;
                }
            }
            else if (diff < 0) {
                if (abs(diff) < velocity) {
                    robot_position_ -= abs(diff);
                }
                else {
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
    std::mutex mutex_;
    std::shared_ptr<RobotMoveGoalHandle> goal_handle_;
    rclcpp_action::GoalUUID preempted_goal_id_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotMoveServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    //rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}