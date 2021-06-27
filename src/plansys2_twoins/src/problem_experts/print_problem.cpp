#include <cstdlib>
#include <ctime>
#include <memory>
#include <vector>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"

#include "example_interfaces/msg/string.hpp"

using std::string;
using std::vector;
using std::shared_ptr;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;

using plansys2::ProblemExpertClient;
using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Function;

using example_interfaces::msg::String;

class PrintProblem : public rclcpp::Node
{
public:
  PrintProblem()
  : rclcpp::Node("print_problem")
  {
    this->declare_parameter("agent_id", "agent0");
  }

  void init()
  { 
    problem_expert_up_ = false;
    problem_ = "";
    problem_expert_ = std::make_shared<ProblemExpertClient>();

    print_problem_timer_ = this->create_wall_timer(
        milliseconds((int)(1000.0 / 0.5)),
        bind(&PrintProblem::printProblem, this));

    publish_problem_timer_ = this->create_wall_timer(
        milliseconds((int)(1000.0 / 0.25)),
        bind(&PrintProblem::publishProblem, this));

    
    problem_publisher_ = this->create_publisher<String>("my_problem", 10);


    auto agent_id = this->get_parameter("agent_id").as_string();
    
    if(agent_id == "agent1")
        listen_agent_id_ = "agent2";
    else if(agent_id == "agent2")
        listen_agent_id_ = "agent1";

    other_problem_subscriber_ = this->create_subscription<String>(
                "/"+listen_agent_id_+"/my_problem", 10,
                bind(&PrintProblem::printProblemOther, this, _1));

    RCLCPP_INFO(this->get_logger(), "Print problem node initialized");
  }

private:

    bool isProblemExpertActive()
    {
        bool isUp = problem_expert_->addInstance(Instance{"turtlepp0", "turtle"});
        if(isUp)
          problem_expert_->removeInstance(Instance{"turtlepp0", "turtle"});
        return isUp;
    }

    void printProblem()
    {
        if(problem_expert_up_ || isProblemExpertActive())
        {
            problem_expert_up_ = true;
            problem_ = problem_expert_->getProblem();
            RCLCPP_INFO(this->get_logger(), problem_);
        }
        else
        {
            problem_expert_up_ = false;
            problem_ = "";
        }
    }

    void publishProblem()
    {
        RCLCPP_INFO(this->get_logger(), "PUBLISHER CALLBACK");
        if(problem_expert_up_ && problem_ != "")
        {
            RCLCPP_INFO(this->get_logger(), "I'm going to publish my problem");
            auto msg = String();
            msg.data = problem_;
            problem_publisher_->publish(msg);
        }
    }

    void printProblemOther(const String::SharedPtr msg)
    {
        RCLCPP_WARN(this->get_logger(), "" + this->listen_agent_id_  + " PROBLEM:\n\n" + msg->data);
    }

    bool problem_expert_up_;
    string listen_agent_id_;
    std::shared_ptr<ProblemExpertClient> problem_expert_;

    string problem_;

    rclcpp::TimerBase::SharedPtr print_problem_timer_;
    rclcpp::TimerBase::SharedPtr publish_problem_timer_;

    rclcpp::Publisher<String>::SharedPtr problem_publisher_;

    rclcpp::Subscription<String>::SharedPtr other_problem_subscriber_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PrintProblem>();

  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
