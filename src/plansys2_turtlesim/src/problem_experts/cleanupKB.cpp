#include <memory>
#include <vector>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"
#include "plansys2_msgs/msg/param.hpp"

#include "rclcpp/rclcpp.hpp"

using std::string;
using std::vector;
using std::shared_ptr;

using plansys2::ProblemExpertClient;
using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Function;

using plansys2_msgs::msg::Param;

typedef enum {STARTING, COMPUTING, PAUSE} StateType;

class CleanupKB : public rclcpp::Node
{
public:
  CleanupKB()
  : rclcpp::Node("cleanupKB"), state_(STARTING)
  {
    comm_errors_ = 0;
  }

  void init()
  {
    problem_expert_ = std::make_shared<ProblemExpertClient>();
    RCLCPP_INFO(this->get_logger(), "Cleanup node initialized");
  }

  void step()
  {
    
    switch (state_) {
      case STARTING:
      {
            // check problem expert alive to init knowledge before spawning
            if(isProblemExpertActive())
                    setState(COMPUTING);
            else{
                RCLCPP_ERROR(this->get_logger(), "Impossible to communicate with problem expert");
                comm_errors_++;
                if(comm_errors_ > 4)//give up and fail
                  rclcpp::shutdown();
            }
            

            break;
      }
      case COMPUTING:
      {     
            checkForEatenTurtles();
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
        bool isUp = problem_expert_->addInstance(Instance{"turtleckb0", "turtle"});
        problem_expert_->removeInstance(Instance{"turtleckb0", "turtle"});
        return isUp;
    }

    void cleanupTurtleInformation(string turtlename)
    {   
        vector<Function> functions = problem_expert_->getFunctions();
        vector<Predicate> predicates = problem_expert_->getPredicates();

        string posx_del_name = "";
        string posy_del_name = "";
        bool allow_remove_posx = true;
        bool allow_remove_posy = true;

        if(problem_expert_->existPredicate(Predicate{"(eaten " + turtlename  + ")"}))
        {
          for(Predicate p : predicates)
          {
            if(p.parameters[0].name == turtlename)
            {
              if(p.name == "posx")
                posx_del_name = p.parameters[1].name;
              else if(p.name == "posy")
                posy_del_name = p.parameters[1].name;

              problem_expert_->removePredicate(p);
            }
          }

          for(Predicate p : predicates)
          {
            if(p.name == "posx" && p.parameters[1].name == posx_del_name)
              allow_remove_posx = false;
            if(p.name == "posy" && p.parameters[1].name == posx_del_name)
              allow_remove_posy = false;
          }

          for(Function f : functions)
          {
            if(f.parameters[0].name == turtlename ||
                (f.parameters.size() > 1 && f.parameters[1].name == turtlename))
              problem_expert_->removeFunction(f);

          }

          problem_expert_->removeInstance(Instance{turtlename, "turtle"});
          if(allow_remove_posx)
            problem_expert_->removeInstance(Instance{posx_del_name, "pos"});
          if(allow_remove_posy)
            problem_expert_->removeInstance(Instance{posy_del_name, "pos"});
          
          RCLCPP_INFO(this->get_logger(), "Cleanup performed for turtle " + turtlename);
        }

    }

    void checkForEatenTurtles()
    {
        vector<Instance> instances = problem_expert_->getInstances();
        
        for(Instance turtle : instances)
        {
          if(problem_expert_->existPredicate(Predicate{"(eaten " + turtle.name + ")"}))
            cleanupTurtleInformation(turtle.name);//turtle is dead -> remove its info from KB
        }

        
    }

    StateType state_;
    int comm_errors_;
    std::shared_ptr<ProblemExpertClient> problem_expert_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<CleanupKB>();

  node->init();

  rclcpp::Rate rate(0.125);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
