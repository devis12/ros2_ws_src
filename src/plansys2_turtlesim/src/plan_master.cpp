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

using std::string;
using std::vector;
using std::shared_ptr;
using std::bind;

using plansys2::ProblemExpertClient;
using plansys2::DomainExpertClient;
using plansys2::PlannerClient;
using plansys2::ExecutorClient;

using plansys2_msgs::msg::Plan;
using plansys2_msgs::msg::PlanItem;

using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Goal;

typedef enum {NO_GOAL, NEW_GOAL, COMPUTE_PLAN, EXECUTING_PLAN} StateType;

class PlanMaster : public rclcpp::Node
{
public:
  PlanMaster()
  : rclcpp::Node("plan_master"), state_(NO_GOAL)
  {
    comm_errors_ = 0;
  }

  void init()
  { 
   
    domain_expert_ = std::make_shared<plansys2::DomainExpertClient>();
    planner_client_ = std::make_shared<plansys2::PlannerClient>();
    problem_expert_ = std::make_shared<plansys2::ProblemExpertClient>();
    executor_client_ = std::make_shared<plansys2::ExecutorClient>();
    goal_string_ = "";
    RCLCPP_INFO(this->get_logger(), "Plan master node initialized");
  }

  void step()
  {
    
    switch (state_) {
      case NO_GOAL:
      {
            // check problem expert alive to init knowledge before spawning
            if(isProblemExpertActive())
            {   
                if(countAliveTurtles() >= 4)
                    setState(NEW_GOAL);
            }else{
                RCLCPP_ERROR(this->get_logger(), "Impossible to communicate with problem expert");
                comm_errors_++;
            }

            if(comm_errors_ > 4)
                rclcpp::shutdown();

            break;
      }
      case NEW_GOAL:
      {     
            vector<string> turtles = getAliveTurtles();
            if(turtles.size() < 4)
                setState(NO_GOAL);
            else if(goal_string_ == "")
            {
                if(setNewEatingGoal(turtles))
                {
                    RCLCPP_INFO(this->get_logger(), "Set new goal: " + goal_string_);
                    setState(COMPUTE_PLAN);
                }else
                    setState(NO_GOAL);
                
            }
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
                    RCLCPP_INFO(this->get_logger(), "Found plan for goal: " + goal_string_
                        +"\n\n\n PLAN:\n\n");
                    for(PlanItem pi : plan.value().items)
                        RCLCPP_INFO(this->get_logger(), "%.2f\t" + pi.action + "\t%.2f",
                                                        pi.time, pi.duration);

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
                goal_string_ = "";
                setState(NO_GOAL);
            }
                
            break;
          
      }
      case EXECUTING_PLAN:
      {   
            auto feedback = executor_client_->getFeedBack();

            for (const auto & action_feedback : feedback.action_execution_status) {
                RCLCPP_INFO(this->get_logger(), "[" + action_feedback.action + " " +
                std::to_string(action_feedback.completion * 100.0) + "%]" + "\n");
            }

            if (!executor_client_->execute_and_check_plan() && executor_client_->getResult()) 
            {
                if (executor_client_->getResult().value().success)
                {
                    RCLCPP_INFO(this->get_logger(),"Successful finished \n");
                    goal_string_ = "";
                    setState(NO_GOAL);
                }        
            }   

            break;
          
      }
      default:
        break;
    }
  }

private:
    void setState(StateType state)
    {
        state_ = state;
    }

    bool isProblemExpertActive()
    {
        bool isUp = problem_expert_->addInstance(Instance{"turtlepm0", "turtle"});
        problem_expert_->removeInstance(Instance{"turtlepm0", "turtle"});
        return isUp;
    }

    int countAliveTurtles(string excludedTurtle="turtle1")
    {
        int alive_counter = 0;
        for(Instance ins : problem_expert_->getInstances())
        {
            if(ins.type == "turtle" && ins.name != excludedTurtle &&
                problem_expert_->existPredicate(Predicate{"(alive " + ins.name + ")"}) && 
                !problem_expert_->existPredicate(Predicate{"(eaten " + ins.name + ")"}))
                    alive_counter++;
        }
        return alive_counter;
    }

    vector<string> getAliveTurtles(string excludedTurtle="turtle1")
    {
        vector<string> turtles = vector<string>();
        for(Instance ins : problem_expert_->getInstances())
        {
            if(ins.type == "turtle" && ins.name != excludedTurtle &&
                problem_expert_->existPredicate(Predicate{"(alive " + ins.name + ")"}) && 
                !problem_expert_->existPredicate(Predicate{"(eaten " + ins.name + ")"}))
                    turtles.push_back(ins.name);
        }
        return turtles;
    }

    bool setNewEatingGoal(vector<string> turtles_names)
    {
        goal_string_ = "(and ";
        
        for(string t_name : turtles_names)
            goal_string_ += "(eaten " + t_name + ")";
        
        goal_string_ += ")";

        return problem_expert_->setGoal(Goal{goal_string_});
    }


    StateType state_;
    int comm_errors_;
    string goal_string_;
    std::shared_ptr<ProblemExpertClient> problem_expert_;
    std::shared_ptr<DomainExpertClient> domain_expert_;
    std::shared_ptr<ExecutorClient> executor_client_;
    std::shared_ptr<PlannerClient> planner_client_;

};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<PlanMaster>();

  node->init();

  rclcpp::Rate rate(0.5);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
