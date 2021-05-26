#include <cstdlib>
#include <ctime>
#include <cmath>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim/srv/kill.hpp"
#include "turtlesim_final_interfaces/msg/turtle_array.hpp"
#include "turtlesim_final_interfaces/msg/turtle_spawn.hpp"

using geometry_msgs::msg::Twist;
using turtlesim::msg::Pose;
using turtlesim::srv::Kill;
using turtlesim_final_interfaces::msg::TurtleArray;
using turtlesim_final_interfaces::msg::TurtleSpawn;

using std::string;
using std::sqrt;
using std::atan2;
using std::vector;
using std::shared_ptr;
using std::thread;
using std::chrono::milliseconds;
using std::bind;
using std::placeholders::_1;

class TurtleController : public rclcpp::Node
{
public:
    TurtleController() : Node("controller")
    {
        this->declare_parameter("turtle_name", "turtle1"); // 'turtle1' is the default value for the turtle to be controlled
        this->declare_parameter("step", 0.08); // single step size

        try{
            name_ = this->get_parameter("turtle_name").as_string();
        }catch(std::exception&){
            RCLCPP_ERROR(this->get_logger(), "turtle_name must be of type string and will be automatically set to turtle1");
            name_ = "turtle1";
        }

        try{
            step_ = this->get_parameter("step").as_double();
        }catch(std::exception&){
            RCLCPP_ERROR(this->get_logger(), "step must be of type double and will be automatically set to 0.08");
            step_ = 0.08;
        }

        turtlesim_up_ = false;
        target_pos_ = TurtleSpawn();
        target_pos_.name = "";

        cmd_vel_publisher_ = this->create_publisher<Twist>(name_+"/cmd_vel", 10);
        control_loop_timer_ = this->create_wall_timer(milliseconds(250),
                                         bind(&TurtleController::moveTowardsTarget, this));

        position_subscriber_ = this->create_subscription<Pose>(
                name_+"/pose", 10,
                bind(&TurtleController::callbackCurrPose, this, _1));

        alive_turtles_subscriber_ = this->create_subscription<TurtleArray>(
                "alive_turtles", 10,
                bind(&TurtleController::callbackAliveTurtles, this, _1));

        RCLCPP_INFO(this->get_logger(), "Controller for " + name_ + " has been started");
        
    }

private:
    void callbackCurrPose(const Pose::SharedPtr msg)
    {
        current_pos_ = *msg;
        turtlesim_up_ = true;
        //RCLCPP_INFO(this->get_logger(), "Updated current position to %.2f, %.2f, %.2f", msg->x, msg->y, msg->theta);
    }

    void acquireNewTarget()
    {
        if(target_pos_.name == "" && alive_turtles_.size() > 0)
        {
            double min_dist = 11.0;
            TurtleSpawn target = TurtleSpawn();
            target.name = "";
            for(TurtleSpawn turtle : alive_turtles_)
            {   
                double dist = computeDistanceFromTurtle(turtle);
                if(dist < min_dist)
                {
                    min_dist =  dist;
                    target = turtle;
                }
            }
            RCLCPP_INFO(this->get_logger(), "Acquired new target in %.1f, %.1f", target_pos_.x, target_pos_.y);
            target_pos_ = target;
        }
    }

    double computeDistanceFromTurtle(const TurtleSpawn turtle)
    {
        double dist_x = turtle.x - current_pos_.x;
        double dist_y = turtle.y - current_pos_.y;
        return sqrt(dist_x * dist_x + dist_y * dist_y);
    }

    void callbackAliveTurtles(const TurtleArray::SharedPtr msg)
    {
        alive_turtles_ = msg->turtles;
        RCLCPP_INFO(this->get_logger(), "Alive turtles callback; current target name: " + target_pos_.name);
        
        if(target_pos_.name == "")
            acquireNewTarget();
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
            target_pos_.name = "";
            RCLCPP_INFO(this->get_logger(), "Eaten '" + turtle.name + "'");
        }
    }       

    void moveTowardsTarget()
    {
        if(!turtlesim_up_ || target_pos_.name == "")
            return;

        
        double distance = computeDistanceFromTurtle(target_pos_);

        Twist msg = Twist();

        if (distance > COLLIDE_DISTANCE)
        {
            // position
            msg.linear.x = 2 * distance;

            // orientation
            double steering_angle = atan2(target_pos_.y - current_pos_.y, target_pos_.x - current_pos_.x);
            double angle_diff = steering_angle - current_pos_.theta;
            if (angle_diff > M_PI)
            {
                angle_diff -= 2 * M_PI;
            }
            else if (angle_diff < -M_PI)
            {
                angle_diff += 2 * M_PI;
            }
            msg.angular.z = 6 * angle_diff;
        }
        else
        {
            // target reached!
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            
            eatTurtle(target_pos_);
        }

        cmd_vel_publisher_->publish(msg);
    }

    void eatTurtle(const TurtleSpawn turtle)
    {
        double distance = computeDistanceFromTurtle(turtle);
        if(distance < COLLIDE_DISTANCE)
        {
            removeTurtle(turtle);//remove from private alive array, otherwise needs to wait for next topic echo by spawner
            shared_ptr<thread> thr = 
                std::make_shared<thread>(bind(&TurtleController::callKillTurtleService, this, turtle));
            thr->detach();
        }
    }

    void callKillTurtleService(const TurtleSpawn turtle)
    {
        rclcpp::Client<Kill>::SharedPtr client_ = this->create_client<Kill>("master_kill");
        while(!client_->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for /master_kill to be up");

        auto request = std::make_shared<Kill::Request>();
        request->name = turtle.name;
        auto future = client_->async_send_request(request);

        try{
            auto response = future.get();
                    
        }catch(const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Response error in /master_kill turtle '" + turtle.name + "'");
        }
    }

    rclcpp::Publisher<Twist>::SharedPtr cmd_vel_publisher_;
    rclcpp::Subscription<Pose>::SharedPtr position_subscriber_;
    rclcpp::Subscription<TurtleArray>::SharedPtr alive_turtles_subscriber_;
    rclcpp::TimerBase::SharedPtr control_loop_timer_;
    Pose current_pos_;
    TurtleSpawn target_pos_;
    vector<TurtleSpawn> alive_turtles_;
    string name_;
    double step_;
    bool turtlesim_up_;

    const double COLLIDE_DISTANCE = 0.32;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //shared pointer, you don't need to delete manually the instance when object is not referenced (like GC in Java)
    auto node = std::make_shared<TurtleController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}