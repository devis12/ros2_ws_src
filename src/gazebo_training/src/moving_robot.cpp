#include <chrono>
#include <functional>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using std::placeholders::_1;

using geometry_msgs::msg::Twist;

using namespace std::chrono_literals;

class MovingRobot : public rclcpp::Node
{
public:
    MovingRobot() : Node("moving_robot")
    {
        publisher_ = this->create_publisher<Twist>("/dolly/cmd_vel", 10);
        timer_ = this->create_wall_timer(500ms, std::bind(&MovingRobot::timer_callback, this));
        RCLCPP_INFO(this->get_logger(), "Hello my friends, moving_robot is arrived!");
    }

private:
    void timer_callback()
    {
        auto message = Twist();
        message.linear.x = 0.5;
        message.angular.z = 0.3;
        RCLCPP_INFO(this->get_logger(), "Publishing: {linear.x: '%.2f', angular.z: %.2f}", message.linear.x, message.angular.z);
        publisher_->publish(message);
    }

    rclcpp::Publisher<Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //shared pointer, you don't need to delete manually the instance when object is not referenced (like GC in Java)
    auto node = std::make_shared<MovingRobot>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}