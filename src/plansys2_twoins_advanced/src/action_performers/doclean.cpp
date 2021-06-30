#include <memory>
#include <algorithm>
#include <vector>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

using namespace std::chrono_literals;

using std::vector;
using std::string;

class DoClean : public plansys2::ActionExecutorClient
{
public:
  DoClean()
  : plansys2::ActionExecutorClient("doclean", 250ms)
  {
    progress_ = 0.0;
  }

private:
  void do_work()
  {
    vector<string> args = get_arguments();
    if (progress_ < 1.0) {
      progress_ += 0.0625;
      send_feedback(progress_, args[0] + "cleaning " + args[1] + " running");
    } else {
      finish(true, 1.0, args[0] + " cleaning " + args[1] + " completed");

      progress_ = 0.0;
    }

    float progress_100 = ((progress_ * 100.0) < 100.0)? (progress_ * 100.0) : 100.0; 
    RCLCPP_INFO(this->get_logger(), 
      "[" + args[0] + " cleaning " +  args[1] + "] "+ 
      "progress: %.1f%%", progress_100);
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DoClean>();
  node->declare_parameter("action_suffix", "");
  string action_suffix = node->get_parameter("action_suffix").as_string();
  node->set_parameter(rclcpp::Parameter("action_name", "doclean"+action_suffix));

  //action node, once created, must pass to inactive state to be ready to execute. 
  // ActionExecutorClient is a managed node (https://design.ros2.org/articles/node_lifecycle.html)
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
