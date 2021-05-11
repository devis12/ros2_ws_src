#include <memory>
#include <string>
#include <sstream>

#include "my_robot_interfaces/action/test.hpp"
#include "rclcpp/rclcpp.hpp"

#include "rclcpp_action/rclcpp_action.hpp"

class TestActionClient : public rclcpp::Node
{
public:
  using Test = my_robot_interfaces::action::Test;
  using GoalHandleTest = rclcpp_action::ClientGoalHandle<Test>;

  explicit TestActionClient(const rclcpp::NodeOptions & node_options = rclcpp::NodeOptions())
  : Node("test_action_client", node_options)
  {
    this->client_ptr_ = rclcpp_action::create_client<Test>(
      this->get_node_base_interface(),
      this->get_node_graph_interface(),
      this->get_node_logging_interface(),
      this->get_node_waitables_interface(),
      "my_action_move");

    this->timer_ = this->create_wall_timer(
      std::chrono::milliseconds(500),
      std::bind(&TestActionClient::send_goal, this));
  }

  void send_goal()
  {
    using namespace std::placeholders;

    this->timer_->cancel();

    if (!this->client_ptr_->wait_for_action_server(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
      rclcpp::shutdown();
      return;
    }

    auto goal_msg = Test::Goal();
    goal_msg.secs = 10;

    RCLCPP_INFO(this->get_logger(), "Sending goal");

    auto send_goal_options = rclcpp_action::Client<Test>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(&TestActionClient::goal_response_callback, this, _1);
    send_goal_options.feedback_callback =
      std::bind(&TestActionClient::feedback_callback, this, _1, _2);
    send_goal_options.result_callback =
      std::bind(&TestActionClient::result_callback, this, _1);
    this->client_ptr_->async_send_goal(goal_msg, send_goal_options);
  }

private:
  rclcpp_action::Client<Test>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(std::shared_future<GoalHandleTest::SharedPtr> future)
  {
    auto goal_handle = future.get();
    if (!goal_handle) {
      RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
    }
  }

  void feedback_callback(
    GoalHandleTest::SharedPtr,
    const std::shared_ptr<const Test::Feedback> feedback_msg)
  {
    std::stringstream ss;
    ss << "Next number in sequence received: ";
    for (auto number : feedback_msg->feedback) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
  }

  void result_callback(const GoalHandleTest::WrappedResult & result)
  {
    switch (result.code) {
      case rclcpp_action::ResultCode::SUCCEEDED:
        break;
      case rclcpp_action::ResultCode::ABORTED:
        RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
        return;
      case rclcpp_action::ResultCode::CANCELED:
        RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
        return;
      default:
        RCLCPP_ERROR(this->get_logger(), "Unknown result code");
        return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    for (auto number : result.result->status) {
      ss << number << " ";
    }
    RCLCPP_INFO(this->get_logger(), ss.str().c_str());
    rclcpp::shutdown();
  }
};  // class TestActionClient

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<TestActionClient>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}
