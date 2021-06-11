#include <memory>
#include <algorithm>
#include <vector>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "turtlesim/srv/kill.hpp"

using namespace std::chrono_literals;

using std::vector;
using std::string;
using std::shared_ptr;
using std::thread;
using std::bind;

using turtlesim::srv::Kill;

class Eating : public plansys2::ActionExecutorClient
{
public:
  Eating()
  : plansys2::ActionExecutorClient("eating", 250ms)
  {
    progress_ = 0.0;
  }

private:
  void killTurtle(string turtlename)
  {
    rclcpp::Client<Kill>::SharedPtr client_ = this->create_client<Kill>("kill");
      while(!client_->wait_for_service(std::chrono::seconds(1)))
          RCLCPP_WARN(this->get_logger(), "Waiting for /kill to be up");

      auto request = std::make_shared<Kill::Request>();
      request->name = turtlename;
      auto future = client_->async_send_request(request);

      try{
          auto response = future.get();
          finish(true, 1.0, "Eating completed");
          progress_ = 0.0;
      }catch(const std::exception &e){
          RCLCPP_ERROR(this->get_logger(), "Response error in /kill turtle '" + turtlename + "'");
      }
  }

  void do_work()
  {
    vector<string> args = get_arguments();
    if (progress_ < 1.0) {
      progress_ += 0.125;
      send_feedback(progress_, "Eating running");
    } else {
      shared_ptr<thread> thr = 
            std::make_shared<thread>(bind(&Eating::killTurtle, this, args[1]));
        
      thr->detach();
    }

    float progress_100 = ((progress_ * 100.0) < 100.0)? (progress_ * 100.0) : 100.0; 
    RCLCPP_INFO(this->get_logger(), 
      "["+ args[0] + " eating " +  args[1] + "] "+ 
      "progress: %.1f%%", progress_100);
  }

  float progress_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Eating>();

  node->set_parameter(rclcpp::Parameter("action_name", "eating"));

  //action node, once created, must pass to inactive state to be ready to execute. 
  // ActionExecutorClient is a managed node (https://design.ros2.org/articles/node_lifecycle.html)
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
