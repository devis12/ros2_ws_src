#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/msg/int64.hpp"

class NumberPublisher : public rclcpp::Node
{
public:
    NumberPublisher() : Node("number_publisher")
    {
        this->declare_parameter("number_to_publish", 2); // 2 is default value to be published
        this->declare_parameter("publish_frequency", 1.0); // 1Hz is the default frequency
        
        try{
            this->number_ = this->get_parameter("number_to_publish").as_int();
        }catch(const std::exception&){
            RCLCPP_ERROR(this->get_logger(), "number_to_publish must be an integer and will be automatically set to 2");
            this->number_ = 2;
        }

        double publish_frequency = 1.0;
        try{
            publish_frequency = this->get_parameter("publish_frequency").as_double();
        }catch(std::exception&){
            RCLCPP_ERROR(this->get_logger(), "publish_frequency must be a positive number and will be automatically set to 1Hz");
            publish_frequency = 1.0;
        }

        publisher_ = this->create_publisher<example_interfaces::msg::Int64>("number", 10);
        timer_ = this->create_wall_timer(std::chrono::milliseconds((int) (1000.0 / publish_frequency)),
                                         std::bind(&NumberPublisher::publishNumber, this));
        RCLCPP_INFO(this->get_logger(), "Number Publisher has been started");
    }

private:
    void publishNumber()
    {
        auto msg = example_interfaces::msg::Int64();
        msg.data = this->number_;
        publisher_->publish(msg);
    }

    rclcpp::Publisher<example_interfaces::msg::Int64>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    int number_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //shared pointer, you don't need to delete manually the instance when object is not referenced (like GC in Java)
    auto node = std::make_shared<NumberPublisher>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}