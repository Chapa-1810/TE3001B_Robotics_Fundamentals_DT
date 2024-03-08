#include <chrono>
#include <functional>
#include "xarm_planner/xarm_planner.h"
#include "geometry_msgs/msg/pose.hpp"
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/int32.hpp"
#include "std_msgs/msg/float32.hpp" 
#include "geometry_msgs/msg/pose_stamped.hpp"
#include "main_interfaces/action/follower.hpp"
using std::placeholders::_1;
using namespace std::chrono_literals;
using action_service = main_interfaces::action::Follower;

class PoseSubscriber : public rclcpp::Node
{
public:
    PoseSubscriber()
    : Node("challenge_xarm_planner_api_pose")
    {
        rclcpp::NodeOptions node_options = rclcpp::NodeOptions();
        node_ = rclcpp::Node::make_shared("path_action_server", node_options);
        node_options.automatically_declare_parameters_from_overrides(true);
        RCLCPP_INFO(node_->get_logger(), "path_action_server start");

        action_server_ = rclcpp_action::create_server<action_service>(node_, "jacobian_follower", 
            std::bind(&PoseSubscriber::handle_goal, this, std::placeholders::_1, std::placeholders::_2), 
            std::bind(&PoseSubscriber::handle_cancel, this, std::placeholders::_1), 
            std::bind(&PoseSubscriber::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(node_->get_logger(), "Initialized action server");
        
        int dof;
        node_->get_parameter_or("dof", dof, 6);
        std::string robot_type;
        node_->get_parameter_or("robot_type", robot_type, std::string("xarm"));
        std::string group_name = robot_type;
        if (robot_type == "xarm" || robot_type == "lite")
            group_name = robot_type + std::to_string(dof);
        xarm_planner_ = new xarm_planner::XArmPlanner(node_, group_name);
    }

    std::string group_name_;
    std::shared_ptr<rclcpp::Node> node_;
    geometry_msgs::msg::PoseStamped pose_;
    xarm_planner::XArmPlanner* xarm_planner_;
    
    geometry_msgs::msg::Pose create_Pose(geometry_msgs::msg::PoseStamped pose_unf){
        geometry_msgs::msg::Pose pose_target;
        pose_target = pose_unf.pose;
        pose_target.position.x = pose_unf.pose.position.x;
        pose_target.position.y = pose_unf.pose.position.y;
        pose_target.position.z = pose_unf.pose.position.z;
        pose_target.orientation.x = 1;
        pose_target.orientation.y = 0;
        pose_target.orientation.z = 0;
        pose_target.orientation.w = 0;
        return pose_target;
    }
    
    void exit_sig_handler(int signum)
    {
        fprintf(stderr, "[test_xarm_planner_api_pose] Ctrl-C caught, exit process...\n");
        exit(-1);
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

        for (long unsigned int i = 0; (i < goal->path.size) && rclcpp::ok();) {
            // Check if there is a cancel request
            if (goal_handle->is_canceling()) {
            result->completed = false;
            goal_handle->canceled(result);
            RCLCPP_INFO(node_->get_logger(), "Goal canceled");
            return;
            }

            // Publish feedback
            goal_handle->publish_feedback(feedback);  
            RCLCPP_INFO(node_->get_logger(), "Publish feedback");

            xarm_planner_->planPoseTarget(create_Pose(goal->path.poses[i]));
            xarm_planner_->executePath(true);

            sequence++, i++;

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


    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr publisher_;
    rclcpp_action::Server<action_service>::SharedPtr action_server_;
    size_t count_;

};


int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<PoseSubscriber>());
    rclcpp::shutdown();    
    return 0;
}