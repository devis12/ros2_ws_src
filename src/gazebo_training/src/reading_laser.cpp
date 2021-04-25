#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"

using std::placeholders::_1;

using sensor_msgs::msg::LaserScan;

class ReadingLaser : public rclcpp::Node
{
public:
    ReadingLaser() : Node("reading_laser")
    {
        auto default_qos = rclcpp::QoS(rclcpp::SystemDefaultsQoS());

        subscription_ = this->create_subscription<LaserScan>(
            "laser_scan", default_qos, std::bind(&ReadingLaser::topic_callback, this, _1));

        RCLCPP_INFO(this->get_logger(), "Hello my friends, reading_laser robot is up!");
    }

private:
    void topic_callback(const LaserScan::SharedPtr _msg)
    {
        RCLCPP_INFO(this->get_logger(), "I heard: '%f', '%f'", _msg->ranges[0], _msg->ranges[100]);
    }

    rclcpp::Subscription<LaserScan>::SharedPtr subscription_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //shared pointer, you don't need to delete manually the instance when object is not referenced (like GC in Java)
    auto node = std::make_shared<ReadingLaser>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}