#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

using sensor_msgs::msg::LaserScan;
using geometry_msgs::msg::Twist;

class ObstacleAvoidance : public rclcpp::Node
{
public:
    ObstacleAvoidance() : Node("obstacle_avoidance")
    {
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        this->declare_parameter("linear_vel", 0.3);
        this->declare_parameter("angular_vel", 0.3);
        linear_vel_ = this->get_parameter("linear_vel").as_double();
        angular_vel_ = this->get_parameter("angular_vel").as_double();

        subscription_ = this->create_subscription<LaserScan>(
            "laser_scan", default_qos, std::bind(&ObstacleAvoidance::topic_callback, this, _1));

        publisher_ = this->create_publisher<Twist>("cmd_vel", 10);

        RCLCPP_INFO(this->get_logger(), "Hello my friends, obstacle_avoidance robot is up!");
    }

private:
    void topic_callback(const LaserScan::SharedPtr _msg)
    {
        /*  200 readings, from right to left, from -57 to 57 degrees
            calculate new velocity cmd
        */
       float min = 10;
       for(int i=0; i<200; i++)
       {
            float current = _msg->ranges[i];
            if(current < min)
                min = current;
       }
       auto message = this->calculateVelMsg(min);
       publisher_->publish(message);
    }

    Twist calculateVelMsg(float distance){
        auto msg = Twist();
        RCLCPP_INFO(this->get_logger(), "Calculate new vel msg based on current distance value: %.4f", distance);
        if(distance < 1){
            //turn around
            msg.linear.x = 0;
            msg.angular.z = angular_vel_;
        }else{
            msg.linear.x = linear_vel_;
            msg.angular.z = 0;
        }
        return msg;
    }

    rclcpp::Subscription<LaserScan>::SharedPtr subscription_;
    rclcpp::Publisher<Twist>::SharedPtr publisher_;
    double linear_vel_;
    double angular_vel_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //shared pointer, you don't need to delete manually the instance when object is not referenced (like GC in Java)
    auto node = std::make_shared<ObstacleAvoidance>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}