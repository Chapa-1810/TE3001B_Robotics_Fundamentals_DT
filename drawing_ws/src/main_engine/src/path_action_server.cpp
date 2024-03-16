#include <functional>
#include <memory>
#include <thread>
#include <queue>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "main_interfaces/action/follower.hpp"
#include "main_interfaces/action/move_arm.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "std_msgs/msg/bool.hpp"

using action_service = main_interfaces::action::Follower;
using arm_service = main_interfaces::action::MoveArm;

rclcpp::TimerBase::SharedPtr timer_;
rclcpp_action::Server<action_service>::SharedPtr action_server_;
rclcpp_action::Client<arm_service>::SharedPtr arm_client_;
std::shared_ptr<rclcpp_action::ServerGoalHandle<action_service>> goal_handle_;
std::shared_ptr<rclcpp::Node> node_;
auto action_goal = arm_service::Goal();
auto send_goal_options = rclcpp_action::Client<arm_service>::SendGoalOptions();

bool arm_done = false;
bool finished_goals = false;

auto result = std::make_shared<action_service::Result>();
auto feedback = std::make_shared<action_service::Feedback>();
//auto & sequence = feedback->pose_index;
std::shared_ptr<const action_service::Goal>  goal_;
auto  result_ = std::make_shared<action_service::Result>();
int ind = 0;

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

void timer_callback(){
  if (!arm_done || finished_goals) return;

  goal_  = goal_handle_->get_goal();

  auto & sequence = feedback->pose_index;
  sequence = ind + 1;
  // std::shared_ptr<main_interfaces::action::Follower_Goal_<std::allocator<void> > > const std::shared_ptr<const main_interfaces::action::Follower_Goal_<std::allocator<void> > >
  if (goal_handle_->is_canceling()) {
    result->completed = false;
    goal_handle_->canceled(result);
    RCLCPP_INFO(node_->get_logger(), "Goal canceled");
    return;
  }

  // Publish feedback
  goal_handle_->publish_feedback(feedback);  
  RCLCPP_INFO(node_->get_logger(), "Publish feedback");

  action_goal.target_pose = goal_->path.poses[ind].pose;

  // TODO: CHECK FOR GOAL OPTIONS 
  send_goal_options.goal_response_callback = std::bind(goal_response_callback, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(feedback_callback, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(result_callback, std::placeholders::_1);
  arm_done = false;
  arm_client_->async_send_goal(action_goal, send_goal_options);
  
  ind++;

  RCLCPP_INFO(node_->get_logger(), "Iteration");
  if (rclcpp::ok() && ind == goal_->path.size) {
    result->completed = true;
    finished_goals = true;
    goal_handle_->succeed(result);
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
  arm_done = true;
  finished_goals = false;
  goal_handle_ = goal_handle;
  ind = 0;
}

int main( int argc, char* argv[] )
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options = rclcpp::NodeOptions();
  node_ = rclcpp::Node::make_shared("path_action_server", options);

  arm_client_ = rclcpp_action::create_client<arm_service>(node_, "move_arm");

  while (!arm_client_->wait_for_action_server(std::chrono::seconds(1))){
    RCLCPP_INFO(node_->get_logger(), "Waiting for arm service");  
  }

  action_server_ = rclcpp_action::create_server<action_service>(node_, "jacobian_follower", 
            std::bind(handle_goal, std::placeholders::_1, std::placeholders::_2), 
            std::bind(handle_cancel, std::placeholders::_1), 
            std::bind(handle_accepted, std::placeholders::_1));

  RCLCPP_INFO(node_->get_logger(), "Initialized action server");

  timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), timer_callback);


  rclcpp::spin(node_);

  rclcpp::shutdown();

  return 0;
}