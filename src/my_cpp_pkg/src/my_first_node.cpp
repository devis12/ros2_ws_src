#include "rclcpp/rclcpp.hpp"

using std::string;
using namespace std::chrono;

class MyNode: public rclcpp::Node
{
    public:
        MyNode(): Node("cpp_test")
        {
            RCLCPP_INFO(this->get_logger(), "Hello Cpp Node!!");
            this->counter_ = 0;
            start_ms_ = system_clock::now();
            timer_ = this->create_wall_timer(seconds(1), 
                                             std::bind(&MyNode::timerCallback, this));
        }
    
    private:

        void timerCallback()
        {
            this->counter_++;
            milliseconds ms_passed = duration_cast<milliseconds>(system_clock::now() - this->start_ms_);

            /*
            char buffer[100];
            snprintf(buffer, 
                     strlen(" Hello,  ms from the start") + std::to_string(this->counter_).length() + + std::to_string(ms_passed.count()).length(),
                     "%u Hello, %lu ms from the start", this->counter_, ms_passed.count());*/
            RCLCPP_INFO(this->get_logger(), "%u Hello, %lu ms from the start", this->counter_, ms_passed.count());
        }

        rclcpp::TimerBase::SharedPtr timer_;
        int counter_;
        std::chrono::system_clock::time_point start_ms_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    //shared pointer, you don't need to delete manually the instance when object is not referenced (like GC in Java)
    auto node = std::make_shared<MyNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}