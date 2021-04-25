#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using std::bind;

using example_interfaces::srv::AddTwoInts;

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<rclcpp::Node>("add_two_ints_client_no_oop");
    auto client = node->create_client<AddTwoInts>("add_two_ints");
    while(!client->wait_for_service(std::chrono::seconds(1)))
        RCLCPP_WARN(node->get_logger(), "Waiting for Two Ints server to be up");

    auto request = std::make_shared<AddTwoInts::Request>();
    request->a = 3;
    request->b = 8;

    auto future = client->async_send_request(request);
    if(rclcpp::spin_until_future_complete(node, future) == rclcpp::executor::FutureReturnCode::SUCCESS)
        RCLCPP_INFO(node->get_logger(), "Response is %d for sum %d + %d", future.get()->sum, request->a, request->b);
    else
        RCLCPP_ERROR(node->get_logger(), "Error while calling server for sum %d + %d", request->a, request->b);

    rclcpp::shutdown();
    return 0;
}