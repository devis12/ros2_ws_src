#include "rclcpp/rclcpp.hpp"
#include "my_robot_interfaces/srv/set_led.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using std::vector;
using std::chrono::seconds;
using std::thread;
using std::bind;
using std::string;

using my_robot_interfaces::srv::SetLed;

class BatteryNode : public rclcpp::Node
{
public:
    BatteryNode() : Node("battery"), charge_(6.0), recharge_(false)
    {
        timer_ = this->create_wall_timer(seconds(1),
                                         bind(&BatteryNode::chargeCycle, this));
    }

private:
    void chargeCycle()
    {
        charge_ += (recharge_)? 1 : -1.5;

        RCLCPP_INFO(this->get_logger(), "Battery status %.1f", charge_);
        
        if(charge_ == 6.0){
            recharge_ = false;
            RCLCPP_INFO(this->get_logger(), ("Battery fully charge: request for turning off led"));
            threads.push_back(thread(bind(&BatteryNode::callSetLedServer, this, 2, false)));
        }else if(charge_ == 0.0){
            recharge_ = true;
            RCLCPP_INFO(this->get_logger(), ("Battery recharging: request for turning on led"));
            threads.push_back(thread(bind(&BatteryNode::callSetLedServer, this, 2, true)));
        }
            
    }

    void callSetLedServer(int led_num, bool led_state)
    {
        rclcpp::Client<SetLed>::SharedPtr client_ = this->create_client<SetLed>("set_led");
        while(!client_->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for Set Led server to be up");

        auto request = std::make_shared<SetLed::Request>();
        request->led_num = led_num;
        request->state = led_state;

        auto future = client_->async_send_request(request);

        string state_s = (request->state)? "true" : "false";
        try{
            auto response = future.get();
            string success_s = (response->success)? "true" : "false";
            RCLCPP_INFO(this->get_logger(), "Response is " + success_s + " for led_num: %d, state: " + state_s, request->led_num);

        }catch(const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Error while calling server for led_num: %d, state: " +state_s, request->led_num);
        }
    }

    float charge_;
    bool recharge_;
    rclcpp::TimerBase::SharedPtr timer_;
    vector<thread> threads;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //shared pointer, you don't need to delete manually the instance when object is not referenced (like GC in Java)
    auto node = std::make_shared<BatteryNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}