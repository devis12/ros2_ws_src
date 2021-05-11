#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "my_robot_interfaces/action/test.hpp"

#include "geometry_msgs/msg/twist.hpp"


class TestActionServer : public rclcpp::Node
{
public:
  using Test = my_robot_interfaces::action::Test;
  using GoalHandleTest = rclcpp_action::ServerGoalHandle<Test>;

  explicit TestActionServer(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
  : Node("test_action_server", options)
  {
    using namespace std::placeholders;

    this->action_server_ = rclcpp_action::create_server<Test>(
      this,
      "my_action_move",
      std::bind(&TestActionServer::handle_goal, this, _1, _2),
      std::bind(&TestActionServer::handle_cancel, this, _1),
      std::bind(&TestActionServer::handle_accepted, this, _1));

    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

  }

private:
  rclcpp_action::Server<Test>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const Test::Goal> goal)
  {
    RCLCPP_INFO(this->get_logger(), "Received goal request with secs %d", goal->secs);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleTest> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleTest> goal_handle)
  {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up a new thread
    std::thread{std::bind(&TestActionServer::execute, this, _1), goal_handle}.detach();
  }

  void execute(const std::shared_ptr<GoalHandleTest> goal_handle)
  {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<Test::Feedback>();
    auto & message = feedback->feedback;
    message = "Starting movement...";
    auto result = std::make_shared<Test::Result>();
    auto move = geometry_msgs::msg::Twist();

    for (int i = 1; (i < goal->secs) && rclcpp::ok(); ++i) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        result->status = message;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }
      // Update feedback
      message = "Moving forward...";
      move.linear.x = 0.3;
      publisher_->publish(move);
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->status = "Finished action server. Robot moved during 5 seconds";
      move.linear.x = 0.0;
      publisher_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }
};  // class TestActionServer

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<TestActionServer>();

  rclcpp::spin(action_server);

  rclcpp::shutdown();
  return 0;
}