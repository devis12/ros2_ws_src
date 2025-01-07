#include <queue>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "my_robot_interfaces/action/count_until.hpp"

class CountUntilServerQueueGoalsNode : public rclcpp::Node
{
public:
    using CountUntil = my_robot_interfaces::action::CountUntil;
    using GoalHandleCountUntil = rclcpp_action::ServerGoalHandle<CountUntil>;

    CountUntilServerQueueGoalsNode() : Node("count_until_server")
    {
        this->callback_group_ = this->create_callback_group(rclcpp::CallbackGroupType::Reentrant);

        this->action_server_ = rclcpp_action::create_server<CountUntil>(
            this,
            "count_until",
            std::bind(&CountUntilServerQueueGoalsNode::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&CountUntilServerQueueGoalsNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&CountUntilServerQueueGoalsNode::handle_accepted, this, std::placeholders::_1),
            rcl_action_server_get_default_options(), 
            callback_group_);
        
        RCLCPP_INFO(this->get_logger(), "Count until server has been started");

        goal_queue_thread_ = std::thread(&CountUntilServerQueueGoalsNode::run_goal_thread, this);
    }

    ~CountUntilServerQueueGoalsNode()
    {
        goal_queue_thread_.join();
    }

private:
    rclcpp_action::Server<CountUntil>::SharedPtr action_server_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    std::queue<std::shared_ptr<GoalHandleCountUntil>> goal_queue_;
    std::mutex mutex_;
    std::thread goal_queue_thread_;

    rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid, std::shared_ptr<const CountUntil::Goal> goal)
    {
        (void)uuid;

        if (goal->target_number < 0 || goal->period < 0.0)
        {
            RCLCPP_WARN(this->get_logger(), "Rejected goal request with negative target_number %ld or period %.2f", goal->target_number, goal->period);
            return rclcpp_action::GoalResponse::REJECT;
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
        {
            std::lock_guard<std::mutex> lock(mutex_);
            goal_queue_.push(goal_handle);
            RCLCPP_INFO(this->get_logger(), "Added goal into queue request with target_number %ld, period %.2f", goal_handle->get_goal()->target_number, goal_handle->get_goal()->period);
            RCLCPP_INFO(this->get_logger(), "Queue size: %ld", goal_queue_.size());
        }
        // execute_goal(goal_handle);
        // std::thread{std::bind(&CountUntilServerQueueGoalsNode::execute, this, _1), goal_handle}.detach();
    }

    void run_goal_thread()
    {
        rclcpp::Rate loop_rate(1000.0);
        while(rclcpp::ok())
        {
            std::shared_ptr<GoalHandleCountUntil> goal_handle;
            {
                std::lock_guard<std::mutex> lock(mutex_);
                if(!goal_queue_.empty())
                {
                    goal_handle = goal_queue_.front();
                    goal_queue_.pop();
                }
            }

            if(goal_handle)
            {
                execute_goal(goal_handle);
            }
            else
            {
                loop_rate.sleep();
            }
        }
    }

    void execute_goal(const std::shared_ptr<GoalHandleCountUntil> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing goal for target_number %ld, period %.2f", goal_handle->get_goal()->target_number, goal_handle->get_goal()->period);
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<CountUntil::Feedback>();
        auto &progress = feedback->current_number;
        auto result = std::make_shared<CountUntil::Result>();

        rclcpp::Rate loop_rate(1.0 / goal->period);

        for (int i = 1; i <= goal->target_number; ++i)
        {
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
    auto node = std::make_shared<CountUntilServerQueueGoalsNode>();
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();
    rclcpp::shutdown();
    return 0;
}
