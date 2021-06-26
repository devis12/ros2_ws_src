#include <cstdlib>
#include <ctime>
#include <memory>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"

using std::string;
using std::shared_ptr;
using std::chrono::seconds;
using std::thread;
using std::bind;

using plansys2::ProblemExpertClient;
using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Function;

typedef enum {STARTING, SPAWNING, PAUSE_SPAWNING} StateType;

class Spawner : public rclcpp::Node
{
public:
  Spawner()
  : rclcpp::Node("spawner_controller"), state_(STARTING)
  {
    comm_errors_ = 0;
    this->declare_parameter("pub_frequency", 0.5); // 0.5Hz is default value to be published
    this->declare_parameter("agent_id", "agent0");
    problem_expert_up_ = false;
  }

  void init()
  { 
    double pub_frequency = 0.5;
    try{
        pub_frequency = this->get_parameter("pub_frequency").as_double();
    }catch(std::exception&){
        RCLCPP_ERROR(this->get_logger(), "pub_frequency must be a positive number and will be automatically set to 500mHz");
        pub_frequency = 0.5;
    }

    
    problem_expert_ = std::make_shared<ProblemExpertClient>();
    srand (static_cast <unsigned> (time(0)));//for random generation

    do_work_timer_ = this->create_wall_timer(
        seconds(4),
        bind(&Spawner::step, this));

    RCLCPP_INFO(this->get_logger(), "Spawner node initialized");
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
                        setState(SPAWNING);
                    else{
                        problem_expert_up_ = false;
                        RCLCPP_ERROR(this->get_logger(), "Impossible to communicate with problem expert");
                        comm_errors_++;
                    }

                    break;
            }
            case SPAWNING:
            {     if(counter_ < 4)
                        spawnNewTurtle();
                    else
                        setState(PAUSE_SPAWNING);
                    break;
            }
            case PAUSE_SPAWNING:
            {   
                    if(counter_ < 2)
                    {
                        setState(SPAWNING);
                        RCLCPP_INFO(this->get_logger(), "Restart spawning now!");
                    }
                    else
                        RCLCPP_INFO(this->get_logger(), "Not the moment to spawn yet... still %d turtles around", counter_);
                    
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

    string extractPosName(string prefix, double val, int num_decimal=2)
    {
        int int_val = (int)val;
        double d_dec_val = val - int_val;
        string s_dec_val = std::to_string(d_dec_val);
        s_dec_val = s_dec_val.substr(s_dec_val.find_first_of(".")+1, num_decimal);
        int dec_val = (int)d_dec_val;
        return  prefix + std::to_string(int_val) + "_" +  s_dec_val; 
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
        if(isUp)
          problem_expert_->removeInstance(Instance{"turtles0", "turtle"});
        return isUp;
    }
    
    bool initKnowledge()
    {
        bool done = true;
        done = done && problem_expert_->addInstance(Instance{"turtle1", "turtle"});

        this-> counter_ = 2;

        done = done && problem_expert_->addPredicate(Predicate{"(control turtle1)"});
        done = done && problem_expert_->addPredicate(Predicate{"(targetfree turtle1)"});
        done = done && problem_expert_->addPredicate(Predicate{"(alive turtle1)"});

        done = done && problem_expert_->addInstance(Instance{"px5_54", "pos"});
        done = done && problem_expert_->addInstance(Instance{"py5_54", "pos"});

        done = done && problem_expert_->addPredicate(Predicate{"(posx turtle1 px5_54)"});
        done = done && problem_expert_->addPredicate(Predicate{"(posy turtle1 py5_54)"});

        done = done && problem_expert_->addFunction(Function{"(turtleseaten turtle1 0)"});

        return done;
    }

    void spawnNewTurtle()
    {
        
        float x = getRandomX();
        float y = getRandomY();
        float theta = getRandomTheta();
        string name = "turtle" + std::to_string(counter_);

        auto agent_id = this->get_parameter("agent_id").as_string();
        RCLCPP_INFO(this->get_logger(),  agent_id + " spawning " + name + " in x: %.2f, y: %.2f, theta: %.2f,", x, y, theta);
        
        shared_ptr<thread> thr = 
            std::make_shared<thread>(bind(&Spawner::callSpawnTurtleService, this, counter_, x, y, theta, name));
        
        thr->detach();

        this->counter_++;
    }

    void callSpawnTurtleService(int count, float x, float y, float theta, string name)
    {
        string px = extractPosName("px", x);
        string py = extractPosName("py", y);
                
        addTurtleKB(name, px, py);
    }

    void addTurtleKB(const string name, const string px, const string py)
    {
        auto agent_id = this->get_parameter("agent_id").as_string();
        auto new_name = agent_id + "_" + name;
        problem_expert_->addInstance(Instance{new_name, "turtle"});
        problem_expert_->addPredicate(Predicate{"(alive " + new_name +")"});
        problem_expert_->addFunction(Function{"(distance turtle1 " + new_name + " 11)"});
        problem_expert_->addInstance(Instance{px, "pos"});
        problem_expert_->addInstance(Instance{py, "pos"});
        problem_expert_->addPredicate(Predicate{"(posx " + new_name + " " + px + ")"});
        problem_expert_->addPredicate(Predicate{"(posy " + new_name + " " + py + ")"});
    }

    StateType state_;
    int comm_errors_;
    std::shared_ptr<ProblemExpertClient> problem_expert_;

    bool problem_expert_up_;
    int counter_;
    rclcpp::TimerBase::SharedPtr do_work_timer_;

    const float MIN_X = 0.0, MAX_X = 11.0;
    const float MIN_Y = 0.0, MAX_Y = 11.0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<Spawner>();

  node->init();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
