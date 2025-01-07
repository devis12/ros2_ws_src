#include <chrono>
#include <map>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include "my_robot_interfaces/action/count_until.hpp"

using namespace std::chrono_literals;

class CountUntilClient : public rclcpp::Node
{
public:
    using CountUntil = my_robot_interfaces::action::CountUntil;
    using GoalHandleCountUntil = rclcpp_action::ClientGoalHandle<CountUntil>;

    explicit CountUntilClient(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("count_until_client", options)
    {
        this->client_ptr_ = rclcpp_action::create_client<CountUntil>(this, "count_until");
    }

    void send_goal(int target_number, float period, bool cancel = false)
    {
        using namespace std::placeholders;

        // Wait for action server to be up and active
        if (!this->client_ptr_->wait_for_action_server()) {
            RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
            rclcpp::shutdown();
            return;
        }

        // Create a goal message
        auto goal_msg = CountUntil::Goal();
        goal_msg.target_number = target_number;
        goal_msg.period = period;

        // Send the goal
        RCLCPP_INFO(this->get_logger(), "Sending goal with target_number %d and period %.2f", target_number, period);

        int goal_id = sent_goals_num_;
        auto send_goal_options = rclcpp_action::Client<CountUntil>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&CountUntilClient::goal_response_callback, this, _1, goal_id);
        send_goal_options.feedback_callback =
            std::bind(&CountUntilClient::feedback_callback, this, _1, _2);
        send_goal_options.result_callback =
            std::bind(&CountUntilClient::result_callback, this, _1);

        this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
        sent_goals_num_++;
        if(cancel)
        {
            timer_ = this->create_wall_timer(2s, [this, target_number, period, goal_id]() 
            {
                if(goal_handles_.find(goal_id) != goal_handles_.end() && goal_handles_[goal_id] && goal_handles_[goal_id]->get_status() == action_msgs::msg::GoalStatus::STATUS_EXECUTING)
                {
                    RCLCPP_INFO(this->get_logger(), "Cancelling goal with target_number %d and period %.2f", target_number, period);
                    this->client_ptr_->async_cancel_goal(goal_handles_[goal_id]);
                }
                timer_->cancel();
            });
        }
    }

private:
    rclcpp_action::Client<CountUntil>::SharedPtr client_ptr_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::unordered_map<int, GoalHandleCountUntil::SharedPtr> goal_handles_;

    int received_goals_num_ = 0;
    int sent_goals_num_ = 0;

    void goal_finished_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Goal finished");
        received_goals_num_++;

        // Shutdown the node after receiving the result
        if(sent_goals_num_ == received_goals_num_)
            rclcpp::shutdown();
    }   

    // Callback to receive the goal response, i.e. goal was accepted or rejected
    void goal_response_callback(const GoalHandleCountUntil::SharedPtr& goal_handle, const int goal_id)
    {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal %d was rejected by server", goal_id);
            goal_finished_callback();
        } else {
            goal_handles_[goal_id] = goal_handle;
            RCLCPP_INFO(this->get_logger(), "Goal %d accepted by server, waiting for result", goal_id);
        }
    }

    // Callback to receive feedback
    void feedback_callback(
        GoalHandleCountUntil::SharedPtr,
        const std::shared_ptr<const CountUntil::Feedback> feedback)
    {
        RCLCPP_INFO(this->get_logger(), "Got feedback: %ld", feedback->current_number);
    }

    // Callback to receive the result once the goal is finished
    void result_callback(const GoalHandleCountUntil::WrappedResult & result)
    {
        switch (result.code) 
        {
            case rclcpp_action::ResultCode::SUCCEEDED:
                RCLCPP_INFO(this->get_logger(), "Result received: %ld", result.result->reached_number);
                break;
            case rclcpp_action::ResultCode::ABORTED:
                RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
                break;
            default:
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }
        goal_finished_callback();
    }
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CountUntilClient>();
    node->send_goal(6, 1.0, false);
    node->send_goal(-2, 0.6);
    node->send_goal(20, 0.4);
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}