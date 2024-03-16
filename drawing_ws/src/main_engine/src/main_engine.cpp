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

bool action_service_done_ = true;
bool service_done_ = false;
bool received_figure_ = false;
bool DEBUG_POSES = false;
bool DEBUG_FIGURE = true;

void talker_callback(const std_msgs::msg::Bool::SharedPtr msg){
  if (bool(msg->data)) {
    RCLCPP_INFO(node_->get_logger(), "Speaking");
  }
}

void figure_callback(const main_interfaces::msg::Figure::SharedPtr msg){
  if (received_figure_) return;

  path_request->figure = *msg;
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
    action_service_done_ = true;

    RCLCPP_INFO(node_->get_logger(), ss.str().c_str());
}

void service_callback(const rclcpp::Client<path_service>::SharedFuture future){
  auto status = future.wait_for(std::chrono::milliseconds(100));
  if (status == std::future_status::ready) {
    RCLCPP_INFO(node_->get_logger(), "Service finished");
    action_goal.path = future.get()->path;
    service_done_ = true;
    for(auto pose : action_goal.path.poses){
      RCLCPP_INFO(node_->get_logger(), "Pose: %f %f %f", pose.pose.position.x, pose.pose.position.y, pose.pose.position.z);
    }
  } else {
    RCLCPP_INFO(node_->get_logger(), "Service In-Progress...");
    service_done_ = false;
  }
}


void timer_callback(){
  if (!received_figure_ || !action_service_done_) return;
  
  auto path_result = client_->async_send_request(path_request, std::bind(service_callback, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "Sending request to  server");

  if (!service_done_){
    RCLCPP_INFO(node_->get_logger(), "Service already done");
    return;
  }

  auto msg = std_msgs::msg::String();
  msg.data = "Attempt to draw figure";
  talker_pub_->publish(msg);


  action_service_done_ = false;
  received_figure_ = false;



  if (DEBUG_POSES){
    auto path = main_interfaces::msg::PoseStampedArray();
    for (int i = 1; i <= 10; i++){
      geometry_msgs::msg::PoseStamped pose;
      pose.pose.position.x = i * 0.1;
      pose.pose.position.y = i * 0.1;
      pose.pose.position.z = i * 0.1;
      path.poses.push_back(pose);
    }
    path.size = 10;
    action_goal.path = path;
  } 

  RCLCPP_INFO(node_->get_logger(), "Debug");

  auto send_goal_options = rclcpp_action::Client<action_service>::SendGoalOptions();
  
  // TODO: CHECK FOR GOAL OPTIONS 
  send_goal_options.goal_response_callback = std::bind(goal_response_callback, std::placeholders::_1);
  send_goal_options.feedback_callback = std::bind(feedback_callback, std::placeholders::_1, std::placeholders::_2);
  send_goal_options.result_callback = std::bind(result_callback, std::placeholders::_1);
  
  RCLCPP_INFO(node_->get_logger(), "Sending goal to action server");
  action_client_->async_send_goal(action_goal, send_goal_options);
}

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);

  node_ = rclcpp::Node::make_shared("main_engine");
  path_request = std::make_shared<path_service::Request>();
  action_goal = action_service::Goal();

  talker_pub_ = node_->create_publisher<std_msgs::msg::String>("hri/speak_text", 1000);
  //talker_sub_ = node_->create_subscription<std_msgs::msg::Bool>("hri/speaking", 10, std::bind(talker_callback, std::placeholders::_1));
  figure_sub_ = node_->create_subscription<main_interfaces::msg::Figure>("hri/figure", 10, std::bind(figure_callback, std::placeholders::_1));

  client_ = node_->create_client<path_service>("PathGenerator");
  action_client_ = rclcpp_action::create_client<action_service>(node_, "jacobian_follower");

  while(!action_client_->wait_for_action_server(std::chrono::milliseconds(500)) || !client_->wait_for_service(std::chrono::milliseconds(500))){
    if (!rclcpp::ok()){
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
      return 0;
    }
    RCLCPP_INFO(node_->get_logger(), "Waiting for service to appear...");
  }

  while(!action_client_->wait_for_action_server(std::chrono::milliseconds(500))){
    if (!rclcpp::ok()){
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
      return 0;
    }
    RCLCPP_INFO(node_->get_logger(), "Waiting for service to appear...");
  }

  RCLCPP_INFO(node_->get_logger(), "Initialized client");

  if (DEBUG_FIGURE){
    main_interfaces::msg::Figure data;
    data.figure_id = 2;
    data.length = 0.05;
    data.radius = 0.0;
    data.width = 0;
    path_request->figure = data;
    received_figure_ = true;
  }

  timer_ = node_->create_wall_timer(std::chrono::milliseconds(500), timer_callback);
  
  rclcpp::spin(node_);
  rclcpp::shutdown();
  
  return 0;
}