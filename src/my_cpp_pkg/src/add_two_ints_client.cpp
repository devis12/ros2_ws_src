#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using std::vector;
using std::thread;
using std::bind;

using example_interfaces::srv::AddTwoInts;

class AddTwoIntsClientNode : public rclcpp::Node
{
public:
    AddTwoIntsClientNode() : Node("add_two_ints_client")
    {
        threads.push_back(thread(bind(&AddTwoIntsClientNode::callAddTwoInts, this)));
    }

private:
    void callAddTwoInts()
    {
        int a, b;

        std::cout << "Insert a: "; std::cin >> a;
        std::cout << "Insert b: "; std::cin >> b;

        rclcpp::Client<AddTwoInts>::SharedPtr client_ = this->create_client<AddTwoInts>("add_two_ints");
        while(!client_->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for Two Ints server to be up");

        auto request = std::make_shared<AddTwoInts::Request>();
        request->a = a;
        request->b = b;

        auto future = client_->async_send_request(request);

        try{
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Response is %d for sum %d + %d", response->sum, request->a, request->b);

            threads.push_back(thread(bind(&AddTwoIntsClientNode::callAddTwoInts, this)));
        }catch(const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Error while calling server for sum %d + %d", request->a, request->b);
        }
    }

    vector<thread> threads;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //shared pointer, you don't need to delete manually the instance when object is not referenced (like GC in Java)
    auto node = std::make_shared<AddTwoIntsClientNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}