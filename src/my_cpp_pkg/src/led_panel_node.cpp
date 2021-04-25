#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/msg/led_states.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using std::placeholders::_1;
using std::placeholders::_2;
using std::vector;
using std::chrono::seconds;

using my_robot_interfaces::msg::LedStates;
using my_robot_interfaces::srv::SetLed;

class LedPanelNode : public rclcpp::Node
{
public:
    LedPanelNode() : Node("led_panel")
    {
        this->declare_parameter("led_states", vector<int64_t> (3, 0));
        this->led_states_ = this->get_parameter("led_states").as_integer_array();
        led_states_publisher_ = this->create_publisher<LedStates>("led_states", 10);
        timer_ = this->create_wall_timer(seconds(1),
                                         bind(&LedPanelNode::publishLedStates, this));

        set_led_server_ = this->create_service<SetLed>("set_led", std::bind(&LedPanelNode::callbackSetLed, this, _1, _2));

        RCLCPP_INFO(this->get_logger(), "Led Panel Node has been started");
    }

private:
    void publishLedStates()
    {
        auto msg = LedStates();
        msg.led_states = this->led_states_;
        led_states_publisher_->publish(msg);
    }

    void callbackSetLed(const SetLed::Request::SharedPtr request,const SetLed::Response::SharedPtr response)
    {
        if(request->led_num >= 0 && request->led_num <= (int)this->led_states_.size()){
            this->led_states_[request->led_num] = request->state;
            response->success = true;
        }else
            response->success = false;
    }

    rclcpp::Publisher<LedStates>::SharedPtr led_states_publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Service<SetLed>::SharedPtr set_led_server_;
    vector<int64_t> led_states_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //shared pointer, you don't need to delete manually the instance when object is not referenced (like GC in Java)
    auto node = std::make_shared<LedPanelNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}