#include <functional>
#include <memory>
#include <future>
#include <string>
#include <sstream>

#include <iostream>


#include "xarm_as_interfaces/action/move_arm.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"

using action_service = xarm_as_interfaces::action::MoveArm;
std::shared_ptr<rclcpp::Node> node_;
rclcpp_action::Client<action_service>::SharedPtr action_client_;

void goal_response_callback(const rclcpp_action::ClientGoalHandle<action_service>::SharedPtr & goal_handle)
{
    if (!goal_handle) {
        RCLCPP_ERROR(node_->get_logger(), "Goal was rejected by server");
    } else {
        RCLCPP_INFO(node_->get_logger(), "Goal accepted by server, waiting for result");
    }
}

void feedback_callback(const rclcpp_action::ClientGoalHandle<action_service>::SharedPtr, const std::shared_ptr<const action_service::Feedback> feedback){
  RCLCPP_INFO(node_->get_logger(), "Received feedback");
}

void result_callback(const rclcpp_action::ClientGoalHandle<action_service>::WrappedResult & result)
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
    RCLCPP_INFO(node_->get_logger(), "Success: %s", success ? "true" : "false");
}


int main(int argc, char * argv[])
{
    std::cout << "Hello, world!" << std::endl;
    rclcpp::init(argc, argv);
    rclcpp::NodeOptions options;
    options.automatically_declare_parameters_from_overrides(true);
    node_ = rclcpp::Node::make_shared("xarm_action_client", options);
    RCLCPP_INFO(node_->get_logger(), "Starting MoveArmActionClient");

    action_client_ = rclcpp_action::create_client<action_service>(node_, "move_arm");

    while(!action_client_->wait_for_action_server(std::chrono::milliseconds(500))){
    if (!rclcpp::ok()){
      RCLCPP_ERROR(node_->get_logger(), "Interrupted while waiting for the service. Exiting.");
      rclcpp::shutdown();
      return 0;
    }
    RCLCPP_INFO(node_->get_logger(), "Waiting for service to appear...");
  }
    
    RCLCPP_INFO(node_->get_logger(), "Service appeared");

    auto goal_msg = action_service::Goal();
    goal_msg.target_pose.position.x = 0.3;
    goal_msg.target_pose.position.y = -0.1;
    goal_msg.target_pose.position.z = 0.4;
    goal_msg.target_pose.orientation.x = 1;
    goal_msg.target_pose.orientation.y = 0;
    goal_msg.target_pose.orientation.z = 0;
    goal_msg.target_pose.orientation.w = 0;

    auto send_goal_options = rclcpp_action::Client<action_service>::SendGoalOptions();
    send_goal_options.goal_response_callback =
      std::bind(goal_response_callback, std::placeholders::_1);
    send_goal_options.feedback_callback =
        std::bind(feedback_callback, std::placeholders::_1, std::placeholders::_2);
    send_goal_options.result_callback =
        std::bind(result_callback, std::placeholders::_1);
    RCLCPP_INFO(node_->get_logger(), "Sending goal");
    action_client_->async_send_goal(goal_msg, send_goal_options);
    rclcpp::spin(node_);
    rclcpp::shutdown();
    return 0;

}