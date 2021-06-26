#include <cstdlib>
#include <ctime>
#include <memory>
#include <vector>
#include <mutex>

#include "plansys2_problem_expert/ProblemExpertClient.hpp"

#include "rclcpp/rclcpp.hpp"

#include "turtlesim/srv/spawn.hpp"

#include "turtlesim_final_interfaces/msg/turtle_spawn.hpp"
#include "turtlesim_final_interfaces/msg/turtle_array.hpp"

using std::string;
using std::vector;
using std::shared_ptr;
using std::chrono::seconds;
using std::chrono::milliseconds;
using std::thread;
using std::bind;
using std::mutex;

using plansys2::ProblemExpertClient;
using plansys2::Instance;
using plansys2::Predicate;
using plansys2::Function;

using turtlesim::srv::Spawn;
using turtlesim_final_interfaces::msg::TurtleSpawn;
using turtlesim_final_interfaces::msg::TurtleArray;

typedef enum {STARTING, SPAWNING, PAUSE_SPAWNING} StateType;

class Spawner : public rclcpp::Node
{
public:
  Spawner()
  : rclcpp::Node("spawner_controller"), state_(STARTING)
  {
    comm_errors_ = 0;
    this->declare_parameter("pub_frequency", 0.5); // 0.5Hz is default value to be published
    this->declare_parameter("agent_id", "agent1");
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

    alive_turtles_publisher_ = this->create_publisher<TurtleArray>("alive_turtles", 10);

    alive_publisher_timer_ = this->create_wall_timer(
        milliseconds((int)(1000.0 / pub_frequency)),
        bind(&Spawner::publishAliveTurtles, this));

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
        if(alive_turtles_.size() > 0)
            checkForEatenTurtles();

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
        {     if(alive_turtles_.size() < 4)
                    spawnNewTurtle();
                else
                    setState(PAUSE_SPAWNING);
                break;
        }
        case PAUSE_SPAWNING:
        {   
                if(alive_turtles_.size() < 2)
                {
                    setState(SPAWNING);
                    RCLCPP_INFO(this->get_logger(), "Restart spawning now!");
                }
                else
                    RCLCPP_INFO(this->get_logger(), "Not the moment to spawn yet... still %d turtles around", alive_turtles_.size());
                
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

    void checkForEatenTurtles()
    {
        for(Instance ins : problem_expert_->getInstances())
        {
            if(ins.type == "turtle" && 
                (!problem_expert_->existPredicate(Predicate{"(alive " + ins.name + ")"}) || problem_expert_->existPredicate(Predicate{"(eaten " + ins.name + ")"}) ))
                removeTurtle(ins.name);
        }


        //DOUBLE CHECK FOR EATEN TURTLES (disappeared from KB... because of cleanup already happened)
        for(TurtleSpawn t : alive_turtles_)
        {   
            bool found = false;
            for(Instance ins : problem_expert_->getInstances())
                if(ins.name == t.name)
                {
                    found = true;
                    break;
                }

            if(!found)
                removeTurtle(t.name);
        }
    }

    void spawnNewTurtle()
    {
        //spawn_lock.lock();
        float x = getRandomX();
        float y = getRandomY();
        float theta = getRandomTheta();
        string name = "turtle" + std::to_string(counter_);

        RCLCPP_INFO(this->get_logger(),  "Spawning " + name + " in x: %.2f, y: %.2f, theta: %.2f,", x, y, theta);
        
        shared_ptr<thread> thr = 
            std::make_shared<thread>(bind(&Spawner::callSpawnTurtleService, this, counter_, x, y, theta, name));
        
        thr->detach();

        this->counter_++;
    }

    void callSpawnTurtleService(int count, float x, float y, float theta, string name)
    {
        auto agent_id = this->get_parameter("agent_id").as_string();
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
                addTurtle(name, x, y, theta);
                
                problem_expert_->addInstance(Instance{response->name, "turtle"});
                problem_expert_->addPredicate(Predicate{"(alive " + response->name +")"});
                problem_expert_->addFunction(Function{"(distance turtle1 " + response->name + " 11)"});
                
                string px = extractPosName("px", x);
                string py = extractPosName("py", y);
                problem_expert_->addInstance(Instance{px, "pos"});
                problem_expert_->addInstance(Instance{py, "pos"});
                problem_expert_->addPredicate(Predicate{"(posx " + response->name + " " + px + ")"});
                problem_expert_->addPredicate(Predicate{"(posy " + response->name + " " + py + ")"});

                //spawn_lock.unlock();
            }

        }catch(const std::exception &e){
             RCLCPP_ERROR(this->get_logger(), "Cannot spawn new turtle", e.what());
        }
        
    }

    void addTurtle(string name, float x, float y, float theta)
    {
        TurtleSpawn newTurtle = TurtleSpawn();
        newTurtle.name = name;
        newTurtle.x = x;
        newTurtle.y = y;
        newTurtle.theta = theta;
        alive_turtles_.push_back(newTurtle);
        publishAliveTurtles();
    }

     void removeTurtle(string name)
    {   
        vector<TurtleSpawn>::iterator itDel;
        bool found = false;
        for (vector<TurtleSpawn>::iterator it = alive_turtles_.begin() ; !found && it != alive_turtles_.end(); it++)
        {
            if((*it).name == name)
            {
                itDel = it; found = true;
            }
        }

        if(found)
        {
            alive_turtles_.erase(itDel);
            publishAliveTurtles();
        }
    }

    void publishAliveTurtles()
    {
        auto msg = TurtleArray();
        msg.turtles = alive_turtles_;
        alive_turtles_publisher_->publish(msg);
    }

    StateType state_;
    int comm_errors_;
    std::shared_ptr<ProblemExpertClient> problem_expert_;
    mutex spawn_lock;

    bool problem_expert_up_;
    int counter_;
    vector<TurtleSpawn> alive_turtles_;
    rclcpp::Publisher<TurtleArray>::SharedPtr alive_turtles_publisher_;
    rclcpp::TimerBase::SharedPtr alive_publisher_timer_;
    rclcpp::TimerBase::SharedPtr do_work_timer_;

    vector<shared_ptr<thread>> spawn_turtle_threads_;

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
