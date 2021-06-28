#include <cstdlib>
#include <ctime>
#include <memory>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "plansys2_pddl_parser/Utils.h"

#include "rclcpp/rclcpp.hpp"

using std::string;
using std::shared_ptr;
using std::chrono::seconds;
using std::bind;

using plansys2::ProblemExpertClient;
using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Function;
using plansys2::Goal;

typedef enum {STARTING, DEFINE, PAUSE} StateType;

class DefineProblem : public rclcpp::Node
{
public:
  DefineProblem()
  : rclcpp::Node("define_problem"), state_(STARTING)
  {
    comm_errors_ = 0;
    this->declare_parameter("agent_id", "agent0");
    problem_expert_up_ = false;
    //goal_set_ = false;
  }

  void init()
  { 
    agent_id_ = this->get_parameter("agent_id").as_string();
    problem_expert_ = std::make_shared<ProblemExpertClient>();

    do_work_timer_ = this->create_wall_timer(
        seconds(4),
        bind(&DefineProblem::step, this));

    RCLCPP_INFO(this->get_logger(), "Problem definer node initialized");
  }

  void step()
  {
    // check problem expert alive to init knowledge before spawning
    if(problem_expert_up_ || isProblemExpertActive()){
        problem_expert_up_ = true;

        switch (state_) {
            case STARTING:
            {
                    
                    if(initKnowledge())
                        setState(DEFINE);
                    else{
                        problem_expert_up_ = false;
                        RCLCPP_ERROR(this->get_logger(), "Impossible to communicate with problem expert");
                        comm_errors_++;
                    }

                    break;
            }
            case DEFINE:
            {    
                setNewGoalString();
                Goal current_goal = problem_expert_->getGoal();

                if(parser::pddl::toString(current_goal) != goal_string_)
                        setProblemGoal();
                    else
                        setState(PAUSE);
                    break;
            }
            case PAUSE:
            {   
                //RCLCPP_INFO(this->get_logger(), "Not the moment to ask for new cleaning tasks yet");    
                break;
            }
            default:
                break;
        }
    }else{
        comm_errors_++;

        if(comm_errors_ > 4)
            rclcpp::shutdown();
    }
  }

private:
    void setState(StateType state)
    {
        state_ = state;
    }

    bool isProblemExpertActive()
    {   
        auto test_name = "pd" + agent_id_ + "_wyp";
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
    
    bool initKnowledge()
    {
        if(agent_id_.length() < 6 || agent_id_.substr(0,5) != "agent")//correct name has to be agentX, where X is a number
            return false;

        int id = std::stoi(agent_id_.substr(5));

        bool done = true;
        
        done = done && problem_expert_->addInstance(Instance{agent_id_, "robot"});
        if(done)
            RCLCPP_INFO(this->get_logger(), "Defined instance " + agent_id_);
        
        done = done && problem_expert_->addInstance(Instance{"bathroom"+std::to_string(id), "waypoint"});
        if(done)
            RCLCPP_INFO(this->get_logger(), "Defined instance bathroom"+std::to_string(id));
        done = done && problem_expert_->addInstance(Instance{"bedroom"+std::to_string(id), "waypoint"});
        if(done)
            RCLCPP_INFO(this->get_logger(), "Defined instance bedroom"+std::to_string(id));
        done = done && problem_expert_->addInstance(Instance{"kitchen"+std::to_string(id), "waypoint"});
        if(done)
            RCLCPP_INFO(this->get_logger(), "Defined instance kitchen"+std::to_string(id));
        done = done && problem_expert_->addInstance(Instance{"dock"+std::to_string(id), "waypoint"});
        if(done)
            RCLCPP_INFO(this->get_logger(), "Defined instance dock"+std::to_string(id));

        done = done && problem_expert_->addPredicate(Predicate{"(workfree " + agent_id_ + ")"});
        if(done)
            RCLCPP_INFO(this->get_logger(), "Defined predicate workfree " + agent_id_);
        done = done && problem_expert_->addPredicate(Predicate{"(recharging_station dock" +std::to_string(id) + ")"});
        if(done)
            RCLCPP_INFO(this->get_logger(), "Defined predicate recharging_station dock" +std::to_string(id));

        if(agent_id_ == "agent1")
        {
            done = done && problem_expert_->addPredicate(Predicate{"(in " + agent_id_ + " dock" +std::to_string(id) + ")"});
            if(done)
                RCLCPP_INFO(this->get_logger(), "Defined predicate in " + agent_id_ + " dock" +std::to_string(id));
            done = done && problem_expert_->addFunction(Function{"(battery_charge 90)"});
            if(done)
                RCLCPP_INFO(this->get_logger(), "Defined function battery_charge 90");
        }
        else if(agent_id_ == "agent2")
        {
            done = done && problem_expert_->addPredicate(Predicate{"(in " + agent_id_ + " bathroom" +std::to_string(id) + ")"});
            if(done)
                RCLCPP_INFO(this->get_logger(), "Defined predicate in " + agent_id_ + " bathroom" +std::to_string(id));
            done = done && problem_expert_->addFunction(Function{"(battery_charge 60)"});
            if(done)
                RCLCPP_INFO(this->get_logger(), "Defined function battery_charge 60");
        }

        return done;
    }

    void setNewGoalString()
    {
        int id = std::stoi(agent_id_.substr(5));

        goal_string_ = "(and ";
        
        goal_string_ += "(cleaned bathroom"+std::to_string(id) + ")";
        goal_string_ += "(cleaned bedroom"+std::to_string(id) + ")";
        goal_string_ += "(cleaned kitchen"+std::to_string(id) + ")";
        
        goal_string_ += ")";
    }

    bool setProblemGoal()
    {
        if(goal_string_ == "")
            setNewGoalString(); 

        bool result = problem_expert_->setGoal(Goal{goal_string_});
        if(result)
            RCLCPP_INFO(this->get_logger(), "Defined new goal: " + goal_string_);    
        else
            RCLCPP_ERROR(this->get_logger(), "Impossible to define new goal: " + goal_string_);    
        return result;
    }

    StateType state_;
    int comm_errors_;
    std::shared_ptr<ProblemExpertClient> problem_expert_;

    string agent_id_;

    bool problem_expert_up_;
    string goal_string_;
    rclcpp::TimerBase::SharedPtr do_work_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<DefineProblem>();

  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
