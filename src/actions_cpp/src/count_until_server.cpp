#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

class CountUntilServerNode : public rclcpp::Node
{
public:
    using CountUntil = my_robot_interfaces::action::CountUntil;
    using GoalHandleCountUntil = rclcpp_action::ServerGoalHandle<CountUntil>;

    CountUntilServerNode() : Node("count_until_server")
    {
        this->goal_handle_ = nullptr;
        this->callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        this->action_server_ = rclcpp_action::create_server<CountUntil>(
            this,
            "count_until",
            std::bind(&CountUntilServerNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CountUntilServerNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&CountUntilServerNode::handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), 
            callback_group_);
        
        RCLCPP_INFO(this->get_logger(), "Count until server has been started");
    }

private:
    rclcpp_action::Server<CountUntil>::SharedPtr action_server_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    std::shared_ptr<GoalHandleCountUntil> goal_handle_;
    std::mutex mutex_;
    rclcpp_action::GoalUUID preempting_goal_uuid_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CountUntil::Goal> goal)
    {
        (void)uuid;

        if (goal->target_number < 0 || goal->period < 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Rejected goal request with negative target_number %ld or period %.2f", goal->target_number, goal->period);
            return rclcpp_action::GoalResponse::REJECT;
        }

        // Policy: Refuse new goal if current goal is active
        // {
        //     std::lock_guard<std::mutex> lock(mutex_);
        //     if (goal_handle_ && goal_handle_->is_active())
        //     {
        //         RCLCPP_WARN(this->get_logger(), "Rejected new goal request, current goal is active");
        //         return rclcpp_action::GoalResponse::REJECT;
        //     }
        // }

        // Policy: preempt the currently active goal when receiving a new valid one
        {
            std::lock_guard<std::mutex> lock(mutex_);   
            if (goal_handle_ && goal_handle_->is_active())
            {
                preempting_goal_uuid_ = goal_handle_->get_goal_id();
            }
        }

        RCLCPP_INFO(this->get_logger(), "Received goal request with target_number %ld, period %.2f", goal->target_number, goal->period);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
        (void)goal_handle;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
    {
        using namespace std::placeholders;
        execute_goal(goal_handle);
        // std::thread{std::bind(&CountUntilServerNode::execute, this, _1), goal_handle}.detach();
    }

    void execute_goal(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
    {
        {
            std::lock_guard<std::mutex> lock(mutex_);
            goal_handle_ = goal_handle;
        }

        RCLCPP_INFO(this->get_logger(), "Executing goal for target_number %ld, period %.2f", goal_handle->get_goal()->target_number, goal_handle->get_goal()->period);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<CountUntil::Feedback>();
        auto &progress = feedback->current_number;
        auto result = std::make_shared<CountUntil::Result>();

        rclcpp::Rate loop_rate(1.0 / goal->period);

        for (int i = 1; i <= goal->target_number; ++i)
        {
            if(preempting_goal_uuid_ == goal_handle->get_goal_id())
            {
                result->reached_number = progress;
                goal_handle->abort(result);
                RCLCPP_INFO(this->get_logger(), "Preempted goal for target_number %ld, period %.2f", goal->target_number, goal->period);
                return;
            }
            if (goal_handle->is_canceling())
            {
                result->reached_number = progress;
                goal_handle->canceled(result);
                RCLCPP_INFO(this->get_logger(), "Goal canceled for target_number %ld, period %.2f", goal->target_number, goal->period);
                return;
            }

            progress = i;
            goal_handle->publish_feedback(feedback);
            RCLCPP_INFO(this->get_logger(), "Published feedback: %ld", progress);
            loop_rate.sleep();
        }

        if (rclcpp::ok())
        {
            result->reached_number = progress;
            goal_handle->succeed(result);
            RCLCPP_INFO(this->get_logger(), "Goal succeeded for target_number %ld, period %.2f", goal->target_number, goal->period);
        }
    }
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilServerNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
