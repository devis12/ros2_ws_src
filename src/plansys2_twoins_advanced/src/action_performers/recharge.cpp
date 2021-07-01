#include <memory>
#include <algorithm>
#include <vector>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

using std::vector;
using std::string;

class Recharge : public plansys2::ActionExecutorClient
{
public:
  Recharge()
  : plansys2::ActionExecutorClient("recharge", 250ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    vector<string> args = get_arguments();
    if (progress_ < 1.0) {
      progress_ += 0.0625;
      send_feedback(progress_, args[0] + " recharging in " + args[1] + " running");
    } else {
      finish(true, 1.0, args[0] + " recharging in " + args[1] + " completed");

      progress_ = 0.0;
    }

    float progress_100 = ((progress_ * 100.0) < 100.0)? (progress_ * 100.0) : 100.0; 
    RCLCPP_INFO(this->get_logger(), 
      "[" + args[0] + " recharging in " + args[1] + "] "+ 
      "progress: %.1f%%", progress_100);
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Recharge>();
  node->declare_parameter("agent_name", "agent0");
  string agent_name = node->get_parameter("agent_name").as_string();
  vector<string> specialized_arguments = vector<string>();
  specialized_arguments.push_back(agent_name);

  node->set_parameter(rclcpp::Parameter("action_name", "recharge"));
  node->set_parameter(rclcpp::Parameter("specialized_arguments", specialized_arguments));
  //action node, once created, must pass to inactive state to be ready to execute. 
  // ActionExecutorClient is a managed node (https://design.ros2.org/articles/node_lifecycle.html)
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
