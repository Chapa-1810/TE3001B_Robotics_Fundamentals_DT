#include <functional>
#include <memory>
#include <bits/stdc++.h>
#include <chrono>
#include <string>
#include <future>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "std_msgs/msg/bool.hpp"
#include "std_msgs/msg/string.hpp"
#include "main_interfaces/msg/figure.hpp"
#include "main_interfaces/msg/pose_stamped_array.hpp"
#include "main_interfaces/srv/path_generator.hpp"
#include "main_interfaces/action/follower.hpp"

using path_service = main_interfaces::srv::PathGenerator;
using action_service = main_interfaces::action::Follower;

rclcpp::TimerBase::SharedPtr timer_;
rclcpp::Publisher<std_msgs::msg::String>::SharedPtr talker_pub_;
rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr talker_sub_;
rclcpp::Subscription<main_interfaces::msg::Figure>::SharedPtr figure_sub_;
rclcpp::Client<path_service>::SharedPtr client_;
path_service::Request::SharedPtr path_request;
rclcpp_action::Client<action_service>::SharedPtr action_client_;
action_service::Goal action_goal;

std::shared_ptr<rclcpp::Node> node_;

bool service_done_ = false;
bool received_figure_ = false;

void talker_callback(const std_msgs::msg::Bool::SharedPtr msg){
  if (bool(msg->data)) {
    RCLCPP_INFO(node_->get_logger(), "Speaking");
  }
}

void figure_callback(const main_interfaces::msg::Figure::SharedPtr msg){
  if (received_figure_) return;

  path_request->figure = *msg; // maybe needs unreferencing &
  received_figure_ = true;

  RCLCPP_INFO(node_->get_logger(), "Recieved figure");
}

void goal_response_callback(const rclcpp_action::ClientGoalHandle<action_service>::SharedPtr & goal_handle){
  if (!goal_handle) {
      RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    } else {
      RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void feedback_callback(const rclcpp_action::ClientGoalHandle<action_service>::SharedPtr, const std::shared_ptr<const action_service::Feedback> feedback){
  RCLCPP_INFO(node_->get_logger(), "Current index of pose array: %d", feedback->pose_index);
}

void result_callback(const rclcpp_action::ClientGoalHandle<action_service>::WrappedResult & result){
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
    ss << "Result received: " << result.result->completed;

    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
}


void timer_callback(){
  if (!received_figure_) return;
  service_done_ = false;
  received_figure_ = false;

  auto msg = std_msgs::msg::String();
  msg.data = "Drawing figure";
  talker_pub_->publish(msg);

  RCLCPP_INFO(node_->get_logger(), "Sending request to  server");
  auto path_result = client_->async_send_request(path_request);

  if (rclcpp::spin_until_future_complete(node_, path_result) != rclcpp::FutureReturnCode::SUCCESS){
    RCLCPP_ERROR(node_->get_logger() , "Failed to call service add_two_ints");
    return;
  } 

  RCLCPP_INFO(node_->get_logger(), "Sending goal to action server");
  action_goal.path = path_result.get()->path;

  auto send_goal_options = rclcpp_action::Client<action_service>::SendGoalOptions();
  
  // TODO: CHECK FOR GOAL OPTIONS 
  send_goal_options.goal_response_callback = std::bind(goal_response_callback, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(feedback_callback, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(result_callback, std::placeholders::_1);
  
  action_client_->async_send_goal(action_goal, send_goal_options);
  
  service_done_ = true;
}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  node_ = rclcpp::Node::make_shared("main_engine");
  path_request = std::make_shared<path_service::Request>();
  action_goal = action_service::Goal();

  talker_pub_ = node_->create_publisher<std_msgs::msg::String>("/sayText", 1000);
  talker_sub_ = node_->create_subscription<std_msgs::msg::Bool>("/speaking", 10, std::bind(talker_callback, std::placeholders::_1));
  //figure_sub_ = node_->create_subscription<main_interfaces::msg::Figure>("/figure", 10, std::bind(figure_callback, std::placeholders::_1));

  client_ = node_->create_client<path_service>("path_generator");
  action_client_ = rclcpp_action::create_client<action_service>(node_, "jacobian_follower");

  while(!action_client_->wait_for_action_server() || !client_->wait_for_service()){
    if (!rclcpp::ok()){
      RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
      return 0;
    } else if (!action_client_->wait_for_action_server() ){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "action service not available, waiting again...");
    } else if (!client_->wait_for_service()){
      RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "service not available, waiting again...");
    }
  }

  timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), timer_callback);
  
  rclcpp::spin(node_);
  rclcpp::shutdown();
  
  return 0;
}