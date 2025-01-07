#include <chrono>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "lifecycle_msgs/srv/change_state.hpp"
#include "lifecycle_msgs/msg/transition.hpp"

using namespace std::chrono_literals;

class LifecycleRobotMoveManager : public rclcpp::Node
{
public:
    LifecycleRobotMoveManager()
    : Node("lifecycle_robot_move_manager")
    {
        this->declare_parameter<std::vector<std::string>>("managed_node_names", std::vector<std::string>());
        this->get_parameter("managed_node_names", node_names_);

        for (const auto & node_name : node_names_) {
            std::string service_change_state_name = "/" + node_name + "/change_state";
            clients_.emplace_back(this->create_client<lifecycle_msgs::srv::ChangeState>(service_change_state_name));
        }

        RCLCPP_INFO(this->get_logger(), "Lifecycle manager node has been started.");
    }

    void change_state(const std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>> & client, uint8_t transition_id, const std::string & label)
    {
        client->wait_for_service();
        auto request = std::make_shared<lifecycle_msgs::srv::ChangeState::Request>();
        request->transition.id = transition_id;
        request->transition.label = label;

        auto future = client->async_send_request(request);
        if (rclcpp::spin_until_future_complete(this->get_node_base_interface(), future) == rclcpp::FutureReturnCode::SUCCESS) {
            RCLCPP_INFO(this->get_logger(), "Transition %s successful", label.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "Transition %s failed", label.c_str());
        }
    }

    void initialization_sequence()
    {
        for (const auto & client : clients_) 
        {
            // Unconfigured to inactive
            RCLCPP_INFO(this->get_logger(), "Trying to switch to configuring");
            change_state(client, lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE, "configure");
            RCLCPP_INFO(this->get_logger(), "Configuring OK, now inactive");
        }

        // Sleep just for example purposes
        std::this_thread::sleep_for(3s);

        for (const auto & client : clients_)
        {
            // Inactive to active
            RCLCPP_INFO(this->get_logger(), "Trying to switch to activating");
            change_state(client, lifecycle_msgs::msg::Transition::TRANSITION_ACTIVATE, "activate");
            RCLCPP_INFO(this->get_logger(), "Activating OK");
        }
    }

private:
    std::vector<std::string> node_names_;
    std::vector<std::shared_ptr<rclcpp::Client<lifecycle_msgs::srv::ChangeState>>> clients_;
};

int main(int argc, char ** argv)
{
    rclcpp::init(argc, argv);
    auto lifecycle_manager = std::make_shared<LifecycleRobotMoveManager>();
    lifecycle_manager->initialization_sequence();
    rclcpp::shutdown();
    return 0;
}