#include "rclcpp/rclcpp.hpp"
#include "turtlesim/msg/pose.hpp"

using turtlesim::msg::Pose;

using std::string;
using std::bind;
using std::placeholders::_1;

class TurtlePoseEcho : public rclcpp::Node
{
public:
    TurtlePoseEcho() : Node("controller")
    {
        this->declare_parameter("turtle_name", "turtle1"); // 'turtle1' is the default value for the turtle to be controlled

        try{
            controlled_name_ = this->get_parameter("turtle_name").as_string();
        }catch(std::exception&){
            RCLCPP_ERROR(this->get_logger(), "turtle_name must be of type string and will be automatically set to turtle1");
            controlled_name_ = "turtle1";
        }

        position_subscriber_ = this->create_subscription<Pose>(
                controlled_name_+"/pose", 10,
                bind(&TurtlePoseEcho::callbackCurrPose, this, _1));
        
    }

private:
    void callbackCurrPose(const Pose::SharedPtr msg)
    {
        
        RCLCPP_INFO(this->get_logger(), "Updated current position to %.2f, %.2f, %.2f", msg->x, msg->y, msg->theta);
    }

    string controlled_name_;
  
    rclcpp::Subscription<Pose>::SharedPtr position_subscriber_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //shared pointer, you don't need to delete manually the instance when object is not referenced (like GC in Java)
    auto node = std::make_shared<TurtlePoseEcho>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}