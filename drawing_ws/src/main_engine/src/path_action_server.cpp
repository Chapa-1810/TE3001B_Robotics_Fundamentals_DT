#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "main_interfaces/action/follower.hpp"
// #include "geometry_msgs/msg/twist.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
//#include "geometry_msgs/msg/JointState.hpp"

using action_service = main_interfaces::action::Follower;

rclcpp::TimerBase::SharedPtr timer_;
// rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr matlab_pub_;
// rclcpp::Subscription<geometry_msgs::msg::JointNOSE>::SharedPtr matlab_sub_;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr poses_pub_;
rclcpp_action::Server<action_service>::SharedPtr action_server_;
std::shared_ptr<rclcpp::Node> node_;

bool USING_MATLAB = false;


// void matlab_callback(const geometry_msgs::msg::JointNOSE::SharedPtr msg){
//   if (USING_MATLAB) {
//     RCLCPP_INFO(node_->get_logger(), "Received pose from matlab");
//     geometry_msgs::msg::Twist twist;
//     twist.linear.x = msg->x;
//     twist.linear.y = msg->y;
//     twist.linear.z = msg->z;
//     twist.angular.x = msg->roll;
//     twist.angular.y = msg->pitch;
//     twist.angular.z = msg->yaw;
//     matlab_pub_->publish(twist);
//   }
// }

void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_service>> goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<action_service::Feedback>();
  auto & sequence = feedback->pose_index;
  sequence = 0;
  auto result = std::make_shared<action_service::Result>();

  for (long unsigned int i = 0; (i < goal->path.size) && rclcpp::ok(); ++i) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->completed = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(node_->get_logger(), "Goal canceled");
      return;
    }

    poses_pub_->publish(goal->path.poses[i]);
    sequence++;

    // Publish feedback
    goal_handle->publish_feedback(feedback);
    RCLCPP_INFO(node_->get_logger(), "Publish feedback");

    loop_rate.sleep();
  }

  // Check if goal is done
  if (rclcpp::ok()) {
    result->completed = true;
    goal_handle->succeed(result);
    RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
  }
}


rclcpp_action::GoalResponse handle_goal( const rclcpp_action::GoalUUID & uuid, std::shared_ptr<const action_service::Goal> goal)
{
  int size = goal->path.size;
  RCLCPP_INFO(node_->get_logger(), "Received goal request with %d poses", size);
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_service>> goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void handle_accepted(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_service>> goal_handle)
{
  execute(goal_handle);
}

int main( int argc, char* argv[] )
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options = rclcpp::NodeOptions();
  node_ = rclcpp::Node::make_shared("path_action_server", options);

  // matlab_pub = node_->create_publisher<geometry_msgs::msg::Twist>("matlab_pub", 10);
  // matlab_sub = node_->create_subscription<geometry_msgs::msg::JointNOSE>("matlab_sub", 10, std::bind(matlab_callback, std::placeholders::_1));
  poses_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("poses_pub", 10);

  action_server_ = rclcpp_action::create_server<action_service>(node_, "jacobian_follower", 
            std::bind(handle_goal, std::placeholders::_1, std::placeholders::_2), 
            std::bind(handle_cancel, std::placeholders::_1), 
            std::bind(handle_accepted, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "Initialized action server");

  rclcpp::spin(node_);

  rclcpp::shutdown();

  return 0;
}