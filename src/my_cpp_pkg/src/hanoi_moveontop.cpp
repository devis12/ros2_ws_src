#include <memory>
#include <algorithm>
#include <vector>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

using std::vector;
using std::string;

class HanoiMoveOnTop : public plansys2::ActionExecutorClient
{
public:
  HanoiMoveOnTop()
  : plansys2::ActionExecutorClient("moveontop", 250ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    vector<string> args = get_arguments();
    if (progress_ < 1.0) {
      progress_ += 0.10;
      send_feedback(progress_, "Move on top running");
    } else {
      finish(true, 1.0, "Move on top completed");

      progress_ = 0.0;
      //std::cout << std::endl;
    }

    float progress_100 = ((progress_ * 100.0) < 100.0)? (progress_ * 100.0) : 100.0; 
    RCLCPP_INFO(this->get_logger(), 
      "[move " + args[0] + " on top of " +  args[2] + "] "+ 
      "progress: %.1f%%", progress_100);
    //std::cout << "\r\e[K" << std::flush;
    //std::cout << "[move " << args[0] << " on top of " << args[2] << "] progress: " << ::flush;
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<HanoiMoveOnTop>();

  node->set_parameter(rclcpp::Parameter("action_name", "moveontop"));

  //action node, once created, must pass to inactive state to be ready to execute. 
  // ActionExecutorClient is a managed node (https://design.ros2.org/articles/node_lifecycle.html)
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
