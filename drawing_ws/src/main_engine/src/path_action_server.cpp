#include <functional>
#include <memory>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "main_interfaces/action/follower.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

using action_service = main_interfaces::action::Follower;

rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr poses_pub_;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr goal_sub_;
rclcpp_action::Server<action_service>::SharedPtr action_server_;
std::shared_ptr<rclcpp::Node> node_;
std_msgs::msg::Bool::SharedPtr goal_;

void goal_callback(const std_msgs::msg::Bool::SharedPtr msg)
{
  RCLCPP_INFO(node_->get_logger(), "Received subcription from goal topic");
  goal_ = msg;
}

void execute(const std::shared_ptr<rclcpp_action::ServerGoalHandle<action_service>> goal_handle)
{
  RCLCPP_INFO(node_->get_logger(), "Executing goal");
  rclcpp::Rate loop_rate(1);
  const auto goal = goal_handle->get_goal();
  auto feedback = std::make_shared<action_service::Feedback>();
  auto & sequence = feedback->pose_index;
  sequence = 1;
  auto result = std::make_shared<action_service::Result>();

  for (long unsigned int i = 0; (i < goal->path.size) && rclcpp::ok(); i++) {
    // Check if there is a cancel request
    if (goal_handle->is_canceling()) {
      result->completed = false;
      goal_handle->canceled(result);
      RCLCPP_INFO(node_->get_logger(), "Goal canceled");
      return;
    }

    poses_pub_->publish(goal->path.poses[i]);

    // Publish feedback
    goal_handle->publish_feedback(feedback);  
    RCLCPP_INFO(node_->get_logger(), "Publish feedback");

    //if (!goal_->data) continue;

    sequence++;

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
  
  //goal_->data = false;

  poses_pub_ = node_->create_publisher<geometry_msgs::msg::PoseStamped>("pose_suscriber", 10);
  //goal_sub_ = node_->create_subscription<std_msgs::msg::Bool>("goal_check", 10, std::bind(goal_callback, std::placeholders::_1));

  action_server_ = rclcpp_action::create_server<action_service>(node_, "jacobian_follower", 
            std::bind(handle_goal, std::placeholders::_1, std::placeholders::_2), 
            std::bind(handle_cancel, std::placeholders::_1), 
            std::bind(handle_accepted, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "Initialized action server");

  rclcpp::spin(node_);

  rclcpp::shutdown();

  return 0;
}