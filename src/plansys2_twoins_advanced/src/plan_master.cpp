#include <memory>
#include <vector>
#include <optional>

#include "rclcpp/rclcpp.hpp"

#include "plansys2_msgs/msg/action_execution_info.hpp"
#include "plansys2_msgs/msg/plan.hpp"

#include "plansys2_pddl_parser/Utils.h"

#include "plansys2_domain_expert/DomainExpertClient.hpp"
#include "plansys2_executor/ExecutorClient.hpp"
#include "plansys2_planner/PlannerClient.hpp"
#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "plansys2_msgs/srv/affect_param.hpp"

#include "example_interfaces/msg/string.hpp"

using std::string;
using std::vector;
using std::shared_ptr;
using std::bind;
using std::placeholders::_1;
using std::chrono::milliseconds;

using plansys2::ProblemExpertClient;
using plansys2::DomainExpertClient;
using plansys2::PlannerClient;
using plansys2::ExecutorClient;

using plansys2_msgs::msg::Plan;
using plansys2_msgs::msg::PlanItem;

using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Goal;

using plansys2_msgs::srv::AffectParam;

using example_interfaces::msg::String;

typedef enum {NO_GOAL, COMPUTE_PLAN, EXECUTING_PLAN, OFF} StateType;

class PlanMaster : public rclcpp::Node
{
public:
  PlanMaster()
  : rclcpp::Node("plan_master"), state_(NO_GOAL)
  {
    this->declare_parameter("agent_id", "agent0");
    this->declare_parameter("cancel_prob", 0.0);
  }

  void init()
  { 
    RCLCPP_INFO(this->get_logger(), "Plan master a");
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    RCLCPP_INFO(this->get_logger(), "Plan master b");
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    RCLCPP_INFO(this->get_logger(), "Plan master c");
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    RCLCPP_INFO(this->get_logger(), "Plan master d");
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    RCLCPP_INFO(this->get_logger(), "Plan master e");
    
    srand (static_cast <unsigned> (time(0)));//for random generation

    problem_expert_up_ = false;
    comm_errors_ = 0;

    
    agent_id_ = this->get_parameter("agent_id").as_string();
    
    if(agent_id_ == "agent1")
        other_agent_id_ = "agent2";
    else if(agent_id_ == "agent2")
        other_agent_id_ = "agent1";

    mygoal_publisher_ = this->create_publisher<String>("/"+other_agent_id_+"/goal_notifications", 10);

    other_goal_subscriber_ = this->create_subscription<String>(
                "goal_notifications", 10,
                bind(&PlanMaster::goalOthersNotification, this, _1));


    goal_string_ = getGoalString();

    do_work_timer_ = this->create_wall_timer(
    milliseconds(500),
    bind(&PlanMaster::step, this));

    RCLCPP_INFO(this->get_logger(), "Plan master node initialized");
  }

  void step()
  {
    switch (state_) {
      case NO_GOAL:
      {
            // check problem expert alive to init knowledge before spawning
            if(problem_expert_up_ || isProblemExpertActive())
            {   
                problem_expert_up_ = true;
                comm_errors_ = 0;
                if(parser::pddl::toString(problem_expert_->getGoal()) == goal_string_){
                    setState(COMPUTE_PLAN);
                    auto msg = String();
                    msg.data = " I am committed to fulfill goal: " + goal_string_;
                    mygoal_publisher_->publish(msg);
                }

            }else{
                problem_expert_up_ = false;
                RCLCPP_ERROR(this->get_logger(), "Impossible to communicate with problem expert");
                comm_errors_++;
            }

            if(comm_errors_ > 16)
                rclcpp::shutdown();

            break;
      }
      case COMPUTE_PLAN:
      {   
            if(parser::pddl::toString(problem_expert_->getGoal()) == goal_string_)
            {
                string domain = domain_expert_->getDomain();
                string problem = problem_expert_->getProblem();
                std::optional<Plan> plan = planner_client_->getPlan(domain, problem);
                if(!plan.has_value())
                {
                    RCLCPP_ERROR(this->get_logger(), "Could not find plan to reach goal " +
                        parser::pddl::toString(problem_expert_->getGoal())  + "\n");
                }else{
                    auto msg = String();
                    msg.data = " I am going to execute the following plan:\n\n";
                    for(PlanItem pi : plan.value().items)
                       msg.data += pi.action + "\n";
                    mygoal_publisher_->publish(msg);

                    RCLCPP_INFO(this->get_logger(), "Start plan execution");
                    
                    if(executor_client_->start_plan_execution(plan.value()))
                        setState(EXECUTING_PLAN);
                    else
                    {
                        RCLCPP_ERROR(this->get_logger(), "Cannot start plan execution ");
                        goal_string_ = "";
                        setState(NO_GOAL);
                    }
                }

                break;
            }
            else
            {
                goal_string_ = getGoalString();
                setState(NO_GOAL);
            }
                
            break;
          
      }
      case EXECUTING_PLAN:
      {   
            double cancel_plan_prob = this->get_parameter("cancel_prob").as_double();
            int gen = rand() % 101;
            bool cancel = (cancel_plan_prob >= 1) || 
                (cancel_plan_prob > 0 && (gen) <= (cancel_plan_prob*100));
            //RCLCPP_WARN(this->get_logger(), "Cancel plan prob %.2f%\t generated: %d",(cancel_plan_prob*100), gen);
            if(cancel)
            {
                executor_client_->cancel_plan_execution();
                auto msg = String();
                msg.data = " I am going to cancel the execution of the plan because Devis has ordered me to";
                mygoal_publisher_->publish(msg);

                RCLCPP_WARN(this->get_logger(), "Cancel plan execution because Devis has ordered me to");
                setState(OFF);
            }
            else
            {
                auto feedback = executor_client_->getFeedBack();
                for (const auto & action_feedback : feedback.action_execution_status) {           
                    if(action_feedback.EXECUTING && action_feedback.completion > 0.0 && action_feedback.completion < 1.0)
                        RCLCPP_INFO(this->get_logger(), "[ " + action_feedback.action_full_name + "  %.2f %]\n", 
                            action_feedback.completion * 100.0);
                }

                if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) 
                {
                    if (executor_client_->getResult().value().success)
                    {
                        RCLCPP_INFO(this->get_logger(),"Successful finished \n");

                        auto msg = String();
                        msg.data = " I have completed the plan to fulfill the goal: " + goal_string_;
                        mygoal_publisher_->publish(msg);

                        goal_string_ = "";
                        setState(NO_GOAL);
                    }        
                }   
            }
            break;
          
      }
      case OFF:
      {
          rclcpp::shutdown();
          break;
      }
      default:
        break;
    }
  }

private:
    
    bool isProblemExpertActive()
    {                   
        rclcpp::Client<AffectParam>::SharedPtr client_ = this->create_client<AffectParam>("problem_expert/add_problem_instance");
        while(!client_->wait_for_service(std::chrono::seconds(1))){
            RCLCPP_WARN(this->get_logger(), "Waiting for problem_expert/add_problem_instance server to be up");
            return false;
        }

        auto test_name = "plm" + agent_id_ + "_wyp";
        RCLCPP_INFO(this->get_logger(), "Ready to define test instance " + test_name);
        bool isUp = problem_expert_->addInstance(Instance{test_name, "waypoint"});
        if(isUp)
        {
            if(problem_expert_->removeInstance(Instance{test_name, "waypoint"}))
                RCLCPP_INFO(this->get_logger(), "Defined and removed test instance " + test_name);
            else
            {
                RCLCPP_INFO(this->get_logger(), "Impossible to remove test instance " + test_name);
                isUp = false;
            }
        }
        return isUp;
    }

    void setState(StateType state)
    {
        state_ = state;
    }

    string getGoalString()
    {
        int id = std::stoi(agent_id_.substr(5));

        string goal_string = "(and ";
        
        goal_string += "(cleaned bathroom"+std::to_string(id) + ")";
        goal_string += "(cleaned bedroom"+std::to_string(id) + ")";
        goal_string += "(cleaned kitchen"+std::to_string(id) + ")";
        
        goal_string += ")";

        return goal_string;
    }

    void goalOthersNotification(const String::SharedPtr msg)
    {
        RCLCPP_WARN(this->get_logger(), "" + other_agent_id_  + ": " + msg->data);
    }

    StateType state_;
    int comm_errors_;
    bool problem_expert_up_;
    string goal_string_;
    string agent_id_;
    string other_agent_id_;
    std::shared_ptr<ProblemExpertClient> problem_expert_;
    std::shared_ptr<DomainExpertClient> domain_expert_;
    std::shared_ptr<ExecutorClient> executor_client_;
    std::shared_ptr<PlannerClient> planner_client_;

    rclcpp::Publisher<String>::SharedPtr mygoal_publisher_;
    rclcpp::Subscription<String>::SharedPtr other_goal_subscriber_;

    rclcpp::TimerBase::SharedPtr do_work_timer_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanMaster>();

  node->init();
  
  rclcpp::spin(node);

  rclcpp::shutdown();

  return 0;
}
