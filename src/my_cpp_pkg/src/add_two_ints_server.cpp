#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

using std::placeholders::_1;
using std::placeholders::_2;

using std::bind;

using example_interfaces::srv::AddTwoInts;

class AddTwoIntsServerNode : public rclcpp::Node
{
public:
    AddTwoIntsServerNode() : Node("add_two_ints_server")
    {
        server_ = this->create_service<AddTwoInts>("add_two_ints", bind(&AddTwoIntsServerNode::callbackTwoInts, this, _1, _2));
        RCLCPP_INFO(this->get_logger(), "Add Two Ints server has been started");
    }

private:
    void callbackTwoInts(const AddTwoInts::Request::SharedPtr request,const AddTwoInts::Response::SharedPtr response)
    {
        response->sum = request->a + request->b;
        RCLCPP_INFO(this->get_logger(), "%d + %d = %d", request->a, request->b, response->sum);
    }

    rclcpp::Service<AddTwoInts>::SharedPtr server_;

};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    //shared pointer, you don't need to delete manually the instance when object is not referenced (like GC in Java)
    auto node = std::make_shared<AddTwoIntsServerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}