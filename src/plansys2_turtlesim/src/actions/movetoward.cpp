#include <memory>
#include <thread>
#include <cmath>
#include <vector>

#include "plansys2_executor/ActionExecutorClient.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "turtlesim/msg/pose.hpp"
#include "turtlesim_final_interfaces/srv/turtle_pose.hpp"

using namespace std::chrono_literals;

using std::vector;
using std::string;
using std::thread;
using std::bind;
using std::placeholders::_1;

using geometry_msgs::msg::Twist;
using turtlesim::msg::Pose;
using turtlesim_final_interfaces::srv::TurtlePose;

typedef struct{
  string name;
  float posx;
  float posy;
}Turtle;

class MoveToward : public plansys2::ActionExecutorClient
{
public:
  MoveToward()
  : plansys2::ActionExecutorClient("movetoward", 250ms)
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
            bind(&MoveToward::callbackCurrPose, this, _1));

    progress_ = 0.0;
    starting_distance_ = -1.0;
    turtlesim_up_ = false;
    current_target_= {.name = "", .posx = -1.0, .posy = -1.0};
    consultingKB_ = false;
  }

   rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State & previous_state)
  {

    cmd_vel_pub_ = this->create_publisher<Twist>(controlled_name_ + "/cmd_vel", 10);
    cmd_vel_pub_->on_activate();

    return ActionExecutorClient::on_activate(previous_state);
  }

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State & previous_state)
  {
    cmd_vel_pub_->on_deactivate();

    return ActionExecutorClient::on_deactivate(previous_state);
  }

private:

    void callbackCurrPose(const Pose::SharedPtr msg)
    {
        controlled_pos_ = *msg;
        turtlesim_up_ = true;
        RCLCPP_INFO(this->get_logger(), "turtle1 pose -> x: %.1f, y: %.1f", 
                current_target_.posx, current_target_.posy);
    }
    
    double distanceFromTarget()
    {
        double dist_x = controlled_pos_.x - current_target_.posx;
        double dist_y = controlled_pos_.y - current_target_.posy;
        return sqrt(dist_x * dist_x + dist_y * dist_y);
    }

    void callGetPoseNewTarget(const string turtlename)
    {
        rclcpp::Client<TurtlePose>::SharedPtr client_ = this->create_client<TurtlePose>("get_pose");
        while(!client_->wait_for_service(std::chrono::seconds(1)))
            RCLCPP_WARN(this->get_logger(), "Waiting for /get_pose to be up");

        auto request = std::make_shared<TurtlePose::Request>();
        request->name = turtlename;
        auto future = client_->async_send_request(request);

        try{
            auto response = future.get();
            current_target_ = {.name = turtlename, .posx = response->posx, .posy = response->posy};
            consultingKB_ = false;
        }catch(const std::exception &e){
            RCLCPP_ERROR(this->get_logger(), "Response error in /get_pose turtle '" + turtlename + "'");
        }
    }

    void getTargetFromKB(string turtlename)
    {
      consultingKB_ = true;
      std::shared_ptr<thread> thr = 
                std::make_shared<thread>(bind(&MoveToward::callGetPoseNewTarget, this, turtlename));
      thr->detach();
    }


    void moveTowardsTarget(string turtlename)
    {
        if(current_target_.name == "" || current_target_.posx < 0 || current_target_.posy < 0)
        {
          if(!consultingKB_)
            getTargetFromKB(turtlename);
        }

        double curr_distance = distanceFromTarget();
        RCLCPP_INFO(this->get_logger(), "current target: " + turtlename + "' in x: %.1f, y: %.1f (d = %.2f)", 
                current_target_.posx, current_target_.posy, curr_distance);

        if(starting_distance_ < 0)
          starting_distance_ = curr_distance;

        Twist msg = Twist();

        if (curr_distance > COLLIDE_DISTANCE)
        {
            // position
            msg.linear.x = 2 * curr_distance;

            // orientation
            double steering_angle = atan2(current_target_.posy - controlled_pos_.y, current_target_.posx - controlled_pos_.x);
            double angle_diff = steering_angle - controlled_pos_.theta;
            if (angle_diff > M_PI)
            {
                angle_diff -= 2 * M_PI;
            }
            else if (angle_diff < -M_PI)
            {
                angle_diff += 2 * M_PI;
            }
            msg.angular.z = 6 * angle_diff;

            progress_ = (starting_distance_ - curr_distance)/starting_distance_;
            send_feedback(progress_, "Move toward running");
        }
        else
        {
            // target reached!
            msg.linear.x = 0.0;
            msg.angular.z = 0.0;
            finish(true, 1.0, "Move toward completed");

            progress_ = 0.0;
            starting_distance_ = -1.0;
            current_target_= {.name = "", .posx = -1.0, .posy = -1.0};
            consultingKB_ = false;
        }

        cmd_vel_pub_->publish(msg);

        float progress_100 = ((progress_ * 100.0) < 100.0)? (progress_ * 100.0) : 100.0; 
            RCLCPP_INFO(this->get_logger(), 
              "[" + controlled_name_ + " moving toward " +  current_target_.name + "] "+ 
              "progress: %.1f%%", progress_100);
    }

  void do_work()
  {
    vector<string> args = get_arguments();
    moveTowardsTarget(args[1]);
  }

  float progress_;
  float starting_distance_;
  bool turtlesim_up_;
  bool consultingKB_;
  string controlled_name_;
  Pose controlled_pos_;

  Turtle current_target_;

  rclcpp_lifecycle::LifecyclePublisher<Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<Pose>::SharedPtr position_subscriber_;


  const double COLLIDE_DISTANCE = 0.32;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MoveToward>();

  node->set_parameter(rclcpp::Parameter("action_name", "movetoward"));

  //action node, once created, must pass to inactive state to be ready to execute. 
  // ActionExecutorClient is a managed node (https://design.ros2.org/articles/node_lifecycle.html)
  node->trigger_transition(lifecycle_msgs::msg::Transition::TRANSITION_CONFIGURE);

  rclcpp::spin(node->get_node_base_interface());

  rclcpp::shutdown();

  return 0;
}
