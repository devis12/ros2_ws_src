#include <cstdlib>
#include <ctime>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "turtlesim/srv/spawn.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim_final_interfaces/msg/turtle_array.hpp"
#include "turtlesim_final_interfaces/msg/turtle_spawn.hpp"

using turtlesim::srv::Spawn;
using turtlesim::srv::Kill;
using turtlesim_final_interfaces::msg::TurtleArray;
using turtlesim_final_interfaces::msg::TurtleSpawn;

using std::string;
using std::vector;
using std::shared_ptr;
using std::thread;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;
using std::placeholders::_2;

class TurtlesMaster : public rclcpp::Node
{
public:
    TurtlesMaster() : Node("turtles_master")
    {
        this->declare_parameter("spawning_frequency", 0.125); // 2 is default value to be published

        this->counter_ = 2;

        double spawning_frequency = 1.0;
        try{
            spawning_frequency = this->get_parameter("spawning_frequency").as_double();
        }catch(std::exception&){
            RCLCPP_ERROR(this->get_logger(), "spawning_frequency must be a positive number and will be automatically set to 1Hz");
            spawning_frequency = 1.0;
        }

        srand (static_cast <unsigned> (time(0)));//for random generation

        alive_turtles_publisher_ = this->create_publisher<TurtleArray>("alive_turtles", 10);

        spawn_turtle_timer_ = this->create_wall_timer(
            milliseconds((int)(1000.0 / spawning_frequency)),
            bind(&TurtlesMaster::spawnNewTurtle, this));

        kill_turtle_server_ = this->create_service<Kill>("master_kill", 
            bind(&TurtlesMaster::callbackMasterKill, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Spawn Turtles has been started");
        
    }

private:

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

    void spawnNewTurtle()
    {
        float x = getRandomX();
        float y = getRandomY();
        float theta = getRandomTheta();
        string name = "turtle" + std::to_string(counter_);
        this->counter_++;

        shared_ptr<thread> thr = 
            std::make_shared<thread>(bind(&TurtlesMaster::callSpawnTurtleService, this, x, y, theta, name));
        
        thr->detach();
    }

    void removeTurtle(const TurtleSpawn turtle)
    {
        bool found = false;
        vector<TurtleSpawn>::iterator itDel;
        for (auto it = alive_turtles_.begin(); !found && it != alive_turtles_.end(); ++it)
            if( (*it).name == turtle.name )
            {
                found = true;
                itDel = it;
            }
        
        if(found)
        {    
            alive_turtles_.erase(itDel);
            RCLCPP_INFO(this->get_logger(), "Killed '" + turtle.name + "'");
            publishAliveTurtles();
        }
    }       

    void callSpawnTurtleService(float x, float y, float theta, string name)
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

            if(response->name == request->name)
                spawnedNewTurtle(request->name, request->x, request->y, request->theta);
            else
                printSpawningError(false, request->name, request->x, request->y, request->theta);
        
        }catch(const std::exception &e){
            printSpawningError(true, request->name, request->x, request->y, request->theta);
        }
    }

    void printSpawningError(bool calling, const string name, const float x, const float y, const float theta){
        string err_while = (calling)? "while calling" : "in response from"; 
        RCLCPP_ERROR(this->get_logger(), "Error " + err_while + " server for spawn of '" + name + "' x: %.2f, y: %.2f, theta: %.2f", 
                x, y, theta);
    }

    void publishAliveTurtles()
    {
        auto msg = TurtleArray();
        msg.turtles = alive_turtles_;
        alive_turtles_publisher_->publish(msg);
    }

    void spawnedNewTurtle(const string name, const float x, const float y, const float theta)
    {
        auto new_turtle = TurtleSpawn();
        new_turtle.x = x;
        new_turtle.y = y;
        new_turtle.theta = theta;
        new_turtle.name = name;
        alive_turtles_.push_back(new_turtle);

        RCLCPP_INFO(this->get_logger(), "Spawned '" + name + "' in  x: %.2f, y: %.2f, theta: %.2f", 
                x, y, theta);

        publishAliveTurtles();
    }

    void callbackMasterKill(const Kill::Request::SharedPtr request,const Kill::Response::SharedPtr response)
    {   
        TurtleSpawn turtle = TurtleSpawn();
        turtle.name = request->name;
        
        shared_ptr<thread> thr = 
            std::make_shared<thread>(bind(&TurtlesMaster::callKillTurtleService, this, turtle));
        
        thr->detach();
    }

    void callKillTurtleService(const TurtleSpawn turtle)
    {
        rclcpp::Client<Kill>::SharedPtr client_ = this->create_client<Kill>("kill");
        while(!client_->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for /kill to be up");

        auto request = std::make_shared<Kill::Request>();
        request->name = turtle.name;
        auto future = client_->async_send_request(request);

        try{
            auto response = future.get();
            removeTurtle(turtle);
        }catch(const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Response error in /kill turtle '" + turtle.name + "'");
        }
    }

    rclcpp::Publisher<TurtleArray>::SharedPtr alive_turtles_publisher_;
    rclcpp::TimerBase::SharedPtr spawn_turtle_timer_;
    rclcpp::Service<Kill>::SharedPtr kill_turtle_server_;

    vector<TurtleSpawn> alive_turtles_;
    int counter_;

    vector<shared_ptr<thread>> spawn_turtle_threads_;

    const float MIN_X = 0.0, MAX_X = 11.0;
    const float MIN_Y = 0.0, MAX_Y = 11.0;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //shared pointer, you don't need to delete manually the instance when object is not referenced (like GC in Java)
    auto node = std::make_shared<TurtlesMaster>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}