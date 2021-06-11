#include <memory>
#include <vector>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"

#include "turtlesim/srv/spawn.hpp"

using std::string;
using std::vector;
using std::shared_ptr;
using std::thread;
using std::bind;

using plansys2::ProblemExpertClient;
using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Function;

using turtlesim::srv::Spawn;

typedef enum {STARTING, SPAWNING, PAUSE} StateType;

class Spawner : public rclcpp::Node
{
public:
  Spawner()
  : rclcpp::Node("spawner_controller"), state_(STARTING)
  {
      comm_errors_ = 0;
  }

  void init()
  {
    problem_expert_ = std::make_shared<ProblemExpertClient>(shared_from_this());
  }

  void step()
  {
    
    switch (state_) {
      case STARTING:
      {
            // check problem expert alive to init knowledge before spawning
            if(isProblemExpertActive()){
                if(initKnowledge())
                    setState(SPAWNING);
                else{
                    RCLCPP_ERROR(this->get_logger(), "Impossible to communicate with problem expert");
                    comm_errors_++;
                }
            }else
                comm_errors_++;

            if(comm_errors_ > 4)
                rclcpp::shutdown();

            break;
      }
      case SPAWNING:
      {     if(counter_ < 6)
                spawnNewTurtle();
            else
                setState(PAUSE);
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

    float getRandomX()
    {
        
        return MIN_X + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(MAX_X-MIN_X)));
    }

    float getRandomY()
    {
        
        return MIN_Y + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(MAX_Y-MIN_Y)));
    }

    float getRandomTheta()
    {
        
        return -M_PI + static_cast <float> (rand()) /( static_cast <float> (RAND_MAX/(2*M_PI)));
    }

    bool isProblemExpertActive()
    {
        bool isUp = problem_expert_->addInstance(Instance{"turtles0", "turtle"});
        problem_expert_->removeInstance(Instance{"turtles0", "turtle"});
        return isUp;
    }
    
    bool initKnowledge()
    {
        bool done = true;
        done = done && problem_expert_->addInstance(Instance{"turtle1", "turtle"});

        this-> counter_ = 2;

        done = done && problem_expert_->addPredicate(Predicate("(control turtle1)"));
        done = done && problem_expert_->addPredicate(Predicate("(targetfree turtle1)"));
        done = done && problem_expert_->addPredicate(Predicate("(alive turtle1)"));

        done = done && problem_expert_->addFunction(Function("(turtleseaten turtle1 0)"));

        return done;
    }

    void spawnNewTurtle()
    {
        float x = getRandomX();
        float y = getRandomY();
        float theta = getRandomTheta();
        string name = "turtle" + std::to_string(counter_);

        RCLCPP_INFO(this->get_logger(),  "[spawner_controller]: Spawning " + name + " in x: %.2f, y: %.2f, theta: %.2f,", x, y, theta);
        
        shared_ptr<thread> thr = 
            std::make_shared<thread>(bind(&Spawner::callSpawnTurtleService, this, counter_, x, y, theta, name));
        
        thr->detach();

        this->counter_++;
    }

    /*
    void checkIfProblemExpertActive()
    {
        
        rclcpp::Client<GetState>::SharedPtr client_ = this->create_client<GetState>("/problem_expert/get_state");
            while(!client_->wait_for_service(std::chrono::seconds(1)))
                RCLCPP_WARN(this->get_logger(), "Waiting for /problem_expert/get_state to be up");

        auto request = std::make_shared<GetState::Request>();
    
        auto future = client_->async_send_request(request);

        try{
            auto response = future.get();

            if(response->current_state.label == "active"){
                if(initKnowledge())
                    setState(SPAWNING);
            }

        
        }catch(const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Cannot retrieve problem expert state!\n Exception: %s", e.what());
        }
    }
    */

    void callSpawnTurtleService(int count, float x, float y, float theta, string name)
    {
        
        rclcpp::Client<Spawn>::SharedPtr client_ = this->create_client<Spawn>("spawn");
        while(!client_->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for /spawn to be up");

        auto request = std::make_shared<Spawn::Request>();
        request->x = x;
        request->y = y;
        request->theta = theta;
        request->name = name;
    
        auto future = client_->async_send_request(request);

        try{
            auto response = future.get();

            if(response->name == request->name){
                problem_expert_->addInstance(Instance{response->name, "turtle"});
                problem_expert_->addPredicate(Predicate("(alive " + response->name +")"));
                problem_expert_->addFunction(Function("(distance turtle1 " + response->name + " 11)"));
            }

        }catch(const std::exception &e){
             RCLCPP_ERROR(this->get_logger(), "Cannot spawn new turtle", e.what());
        }
        
    }
    StateType state_;
    int comm_errors_;
    std::shared_ptr<ProblemExpertClient> problem_expert_;

    int counter_;

    vector<shared_ptr<thread>> spawn_turtle_threads_;

    const float MIN_X = 0.0, MAX_X = 11.0;
    const float MIN_Y = 0.0, MAX_Y = 11.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Spawner>();

  node->init();

  rclcpp::Rate rate(0.25);
  while (rclcpp::ok()) {
    node->step();

    rate.sleep();
    rclcpp::spin_some(node->get_node_base_interface());
  }

  rclcpp::shutdown();

  return 0;
}
