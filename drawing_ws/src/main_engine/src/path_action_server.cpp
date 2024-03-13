#include <functional>
#include <memory>
#include <thread>
#include <queue>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "main_interfaces/action/follower.hpp"
#include "main_interfaces/action/move_arm.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

using action_service = main_interfaces::action::Follower;
using arm_service = main_interfaces::action::MoveArm;

rclcpp_action::Server<action_service>::SharedPtr action_server_;
rclcpp_action::Client<arm_service>::SharedPtr arm_client_;
std::shared_ptr<rclcpp::Node> node_;
auto action_goal = arm_service::Goal();
auto send_goal_options = rclcpp_action::Client<arm_service>::SendGoalOptions();

bool arm_done = true;

void goal_response_callback(const rclcpp_action::ClientGoalHandle<arm_service>::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void feedback_callback(const rclcpp_action::ClientGoalHandle<arm_service>::SharedPtr, const std::shared_ptr<const arm_service::Feedback> feedback){
  RCLCPP_INFO(node_->get_logger(), "Received feedback");
}

void result_callback(const rclcpp_action::ClientGoalHandle<arm_service>::WrappedResult & result)
{   
    RCLCPP_ERROR(node_->get_logger(), "Result recieved");
    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            break;
        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "Goal was aborted");
            return;
        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_ERROR(node_->get_logger(), "Goal was canceled");
            return;
        default:
            RCLCPP_ERROR(node_->get_logger(), "Unknown result code");
            return;
    }
    std::stringstream ss;
    ss << "Result received: ";
    bool success = result.result->success;
    arm_done = true;
    RCLCPP_INFO(node_->get_logger(), "Success: %s", success ? "true" : "false");
}

void execute(std::queue<geometry_msgs::msg::PoseStamped>& qq)
{
  auto result = std::make_shared<action_service::Result>();
  auto feedback = std::make_shared<action_service::Feedback>();
  auto & sequence = feedback->pose_index;
  sequence = 1;


  if (!qq.empty()) {
    RCLCPP_INFO(node_->get_logger(), "Sending goal");
    action_goal.target_pose = qq.front().pose;
    auto future = arm_client_->async_send_goal(action_goal, send_goal_options);
    qq.pop();
  }




  // for (long unsigned int i = 0; (i < goal->path.size) && rclcpp::ok();) {
  //   // Check if there is a cancel request
  //   if (goal_handle->is_canceling()) {
  //     result->completed = false;
  //     goal_handle->canceled(result);
  //     RCLCPP_INFO(node_->get_logger(), "Goal canceled");
  //     return;
  //   } else if (!arm_done) {
  //     //RCLCPP_INFO(node_->get_logger(), "Arm completing action");
  //     continue;
  //   }

  //   // Publish feedback
  //   goal_handle->publish_feedback(feedback);  
  //   RCLCPP_INFO(node_->get_logger(), "Publish feedback");

  //   action_goal.target_pose = goal->path.poses[i].pose;
  
  //   // TODO: CHECK FOR GOAL OPTIONS 
  //   send_goal_options.goal_response_callback = std::bind(goal_response_callback, std::placeholders::_1);
  //   send_goal_options.feedback_callback = std::bind(feedback_callback, std::placeholders::_1, std::placeholders::_2);
  //   send_goal_options.result_callback = std::bind(result_callback, std::placeholders::_1);
// arm_done = false;
  //   arm_client_->async_send_goal(action_goal, send_goal_options);
  //   

  //   sequence++;  i++;

  //   //loop_rate.sleep();
  // }

  // Check if goal is done
  if (rclcpp::ok()) {
    result->completed = true;
    // goal_handle->succeed(result);
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
  const auto goal = goal_handle->get_goal();
  std::queue<geometry_msgs::msg::PoseStamped> qq;
  for (long unsigned int i = 0; (i < goal->path.size) && rclcpp::ok(); i++) {
    qq.push(goal->path.poses[i]);
  } 
  execute(qq);
}

int main( int argc, char* argv[] )
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options = rclcpp::NodeOptions();
  node_ = rclcpp::Node::make_shared("path_action_server", options);

  arm_client_ = rclcpp_action::create_client<arm_service>(node_, "move_arm");

  while (!arm_client_->wait_for_action_server(std::chrono::seconds(1))){
    RCLCPP_INFO(node_->get_logger(), "Waiting for arm service");  
    // std::this_thread::sleep_for(std::chrono::seconds(1));
  }

  action_server_ = rclcpp_action::create_server<action_service>(node_, "jacobian_follower", 
            std::bind(handle_goal, std::placeholders::_1, std::placeholders::_2), 
            std::bind(handle_cancel, std::placeholders::_1), 
            std::bind(handle_accepted, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "Initialized action server");

  rclcpp::spin(node_);

  rclcpp::shutdown();

  return 0;
}