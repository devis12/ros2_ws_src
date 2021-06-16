#include <memory>
#include <vector>
#include <optional>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"

#include "turtlesim/msg/pose.hpp"
#include "turtlesim_final_interfaces/msg/turtle_spawn.hpp"
#include "turtlesim_final_interfaces/msg/turtle_array.hpp"

using std::string;
using std::vector;
using std::shared_ptr;
using std::thread;
using std::bind;
using std::optional;
using std::placeholders::_1;

using plansys2::ProblemExpertClient;
using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Function;

using turtlesim::msg::Pose;
using turtlesim_final_interfaces::msg::TurtleArray;
using turtlesim_final_interfaces::msg::TurtleSpawn;

typedef enum {STARTING, COMPUTING, PAUSE} StateType;

class ComputeDistances : public rclcpp::Node
{
public:
  ComputeDistances()
  : rclcpp::Node("computedistances"), state_(STARTING)
  {
    comm_errors_ = 0;
  }

  void init()
  {
    controlled_name_ = "turtle1";
    try{
        controlled_name_ = this->get_parameter("controlled_name").as_double();
    }catch(std::exception&){
        RCLCPP_ERROR(this->get_logger(), "controlled_name must be a string and will be automatically set to  \"turtle1\"");
        controlled_name_ = "turtle1";
    }

    controlled_pose_subscriber_ = this->create_subscription<Pose>(
                controlled_name_+"/pose", 10,
                bind(&ComputeDistances::callbackControlledPose, this, _1));
    
    alive_turtles_subscriber_ = this->create_subscription<TurtleArray>(
                "alive_turtles", 10,
                bind(&ComputeDistances::callbackAliveTurtles, this, _1));

    problem_expert_ = std::make_shared<ProblemExpertClient>(shared_from_this());

    RCLCPP_INFO(this->get_logger(), "Compute distances node initialized");
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
            computeDistances();
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
        bool isUp = problem_expert_->addInstance(Instance{"turtled0", "turtle"});
        problem_expert_->removeInstance(Instance{"turtled0", "turtle"});
        return isUp;
    }

    void callbackControlledPose(const Pose::SharedPtr msg)
    {
      controlled_turtle_ = *msg;
      //RCLCPP_INFO(this->get_logger(), "Updated current position to %.2f, %.2f, %.2f", msg->x, msg->y, msg->theta);
    }

    void callbackAliveTurtles(const TurtleArray::SharedPtr msg)
    {
      alive_turtles_ = msg->turtles;
      //RCLCPP_INFO(this->get_logger(), "Updated alives turtle array");
    }

    TurtleSpawn findTurtle(string turtlename)
    {
      for(TurtleSpawn t : alive_turtles_)
      {
          if(t.name == turtlename)
            return t;
          
      }
      auto no_found = TurtleSpawn();
      return no_found;//not found
    }

    double computeDistanceFromTurtle(TurtleSpawn t2)
    {
        double dist_x = controlled_turtle_.x - t2.x;
        double dist_y = controlled_turtle_.y - t2.y;
        return sqrt(dist_x * dist_x + dist_y * dist_y);
    }

    void computeDistances()
    {
        vector<Instance> instances = problem_expert_->getInstances();
        
        for(Instance turtle : instances)
        {
          if(turtle.name == controlled_name_)
            continue;

          TurtleSpawn turtle_alive = findTurtle(turtle.name); 
        
          if(  turtle_alive.name == turtle.name &&
               problem_expert_->existPredicate(Predicate{"(alive " + turtle.name + ")"}) && 
              !problem_expert_->existPredicate(Predicate{"(eaten " + turtle.name + ")"}))
          {
            double d = computeDistanceFromTurtle(turtle_alive);
            if(problem_expert_ -> updateFunction(Function{"(distance " + controlled_name_ + " " + turtle.name + " " + std::to_string(d) + ")"}))
            {  
              //NOTE: getFunction return std::optional<Function>
              //Function f = (problem_expert_->getFunction("(distance " + controlled_name_ + " " + turtle.name + ")")).value();
              //double f_value = f.value;
              
              //RCLCPP_INFO(this->get_logger(),  "[computedistances]: recomputing done for " + turtle.name + " equals to " + std::to_string(d));
            }
          }
        }

        
    }

    vector<TurtleSpawn> alive_turtles_;
    Pose controlled_turtle_;
    string controlled_name_;

    rclcpp::Subscription<Pose>::SharedPtr controlled_pose_subscriber_;
    rclcpp::Subscription<TurtleArray>::SharedPtr alive_turtles_subscriber_;

    StateType state_;
    int comm_errors_;
    std::shared_ptr<ProblemExpertClient> problem_expert_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<ComputeDistances>();

  node->init();

  rclcpp::Rate rate(8);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
