#include <functional>
#include <memory>
#include <thread>

#include "xarm_as_interfaces/action/move_arm.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "rclcpp_components/register_node_macro.hpp"
#include "geometry_msgs/msg/pose.hpp"
#include "geometry_msgs/msg/pose_stamped.hpp"

//#include "xarm_as/visibility_control.h"

//moveit2
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

using MoveArm = xarm_as_interfaces::action::MoveArm;
using GoalHandleMoveArm = rclcpp_action::ServerGoalHandle<MoveArm>;

rclcpp_action::Server<MoveArm>::SharedPtr action_server_;
std::shared_ptr<rclcpp::Node> node_;
std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group;
std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface;

rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID & uuid,
        std::shared_ptr<const MoveArm::Goal> goal)
    {
        RCLCPP_INFO(node_->get_logger(), "Received goal request with target %f", goal->target_pose.position.x);
        (void)uuid;
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }
    
rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMoveArm> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
}

void execute(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
{
    RCLCPP_INFO(node_->get_logger(), "Executing goal");
    rclcpp::Rate loop_rate(1);
    const auto goal = goal_handle->get_goal();
    const auto goal_pose = goal->target_pose;
    const auto goal_state = goal->target_state;
    const auto goal_joints = goal->target_joints;
    const auto planning_time = goal->planning_time;
    const auto num_planning_attempts = goal->num_planning_attempts;
    const auto velocity = goal->velocity;
    const auto acceleration = goal->acceleration;
    const auto positionTolerance = goal->position_tolerance;
    const auto orientationTolerance = goal->orientation_tolerance;

  RCLCPP_INFO(node_->get_logger(), "Planning time: %d", planning_time);
  RCLCPP_INFO(node_->get_logger(), "Num planning attempts: %d", num_planning_attempts);
  RCLCPP_INFO(node_->get_logger(), "Max velocity scaling factor: %f", velocity);
  RCLCPP_INFO(node_->get_logger(), "Max acceleration scaling factor: %f", acceleration);
  RCLCPP_INFO(node_->get_logger(), "Goal position tolerance: %f", positionTolerance);
  RCLCPP_INFO(node_->get_logger(), "Goal orientation tolerance: %f", orientationTolerance);


    RCLCPP_INFO(node_->get_logger(), "Received goal request with target x: %f, y: %f, z: %f", goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
    RCLCPP_INFO(node_->get_logger(), "Received goal request with target state: %s", goal_state.c_str());
    std::string joint_str = "";
    for (int i = 0; i < goal_joints.size(); i++)
    {
      joint_str += std::to_string(goal_joints[i]) + " ";
    }
    RCLCPP_INFO(node_->get_logger(), "Received goal request with target joints: %s", joint_str.c_str());
    auto feedback = std::make_shared<MoveArm::Feedback>();
    auto result = std::make_shared<MoveArm::Result>();

    RCLCPP_INFO(node_->get_logger(), "Setting up MoveGroup");
    move_group->setPlanningTime(planning_time ? planning_time : 10);
    RCLCPP_INFO(node_->get_logger(), "Planning time set");
    move_group->setNumPlanningAttempts(num_planning_attempts ? num_planning_attempts : 3);
    RCLCPP_INFO(node_->get_logger(), "Num planning attempts set");
    move_group->setMaxVelocityScalingFactor(velocity ? velocity : 0.5);
    RCLCPP_INFO(node_->get_logger(), "Max velocity scaling factor set");
    move_group->setMaxAccelerationScalingFactor(acceleration ? acceleration : 0.1);
    RCLCPP_INFO(node_->get_logger(), "Max acceleration scaling factor set");
    move_group->setGoalPositionTolerance(positionTolerance ? positionTolerance : 0.01);
    RCLCPP_INFO(node_->get_logger(), "Position tolerance set");
    move_group->setGoalOrientationTolerance(orientationTolerance ? orientationTolerance : 0.01);
    RCLCPP_INFO(node_->get_logger(), "MoveGroup set up");

    if (goal_state != "")
    {
        RCLCPP_INFO(node_->get_logger(), "Setting target state");
        move_group->setNamedTarget(goal_state);
    }
    else if (goal_pose.position.x != 0.0 || goal_pose.position.y != 0.0 || goal_pose.position.z != 0.0)
    {
        RCLCPP_INFO(node_->get_logger(), "Setting target pose");
        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = goal_pose.position.x;
        target_pose.position.y = goal_pose.position.y;
        target_pose.position.z = goal_pose.position.z;
        target_pose.orientation.x = goal_pose.orientation.x;
        target_pose.orientation.y = goal_pose.orientation.y;
        target_pose.orientation.z = goal_pose.orientation.z;
        target_pose.orientation.w = goal_pose.orientation.w;
        move_group->setPoseTarget(target_pose);
    }
    else if (goal_joints.size() > 0)
    {
      RCLCPP_INFO(node_->get_logger(), "Setting target joints");
      std::vector<double> goal_joints_set;
      for (int i = 0; i < goal_joints.size(); i++)
      {
        goal_joints_set.push_back(goal_joints[i]);
      }
      move_group->setJointValueTarget(goal_joints_set);
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "No target specified");
        result->success = false;
        goal_handle->abort(result);
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "Planning...");
    moveit::planning_interface::MoveGroupInterface::Plan movePlan;
    bool success = (move_group->plan(movePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
    RCLCPP_INFO(node_->get_logger(), "Plan result: %d", success);
    if(success)
    {
        RCLCPP_INFO(node_->get_logger(), "Executing...");
        move_group->execute(movePlan);
        RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
        result->success = true;
    }
    else
    {
        RCLCPP_INFO(node_->get_logger(), "Goal aborted");
        result->success = false;
    }
    if (rclcpp::ok()) {
      goal_handle->succeed(result);
      RCLCPP_INFO(node_->get_logger(), "Goal succeeded");
    }
}

void handle_accepted(const std::shared_ptr<GoalHandleMoveArm> goal_handle)
{
    execute(goal_handle);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  
  rclcpp::NodeOptions node_options;
  node_options.automatically_declare_parameters_from_overrides(true);
  node_ = rclcpp::Node::make_shared("xarm_action_server", node_options);
  RCLCPP_INFO(node_->get_logger(), "xArm AS Node start");
  RCLCPP_INFO(node_->get_logger(), "Connecting to MoveGroup: %s", "xarm6");
  move_group = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, "xarm6");
  planning_scene_interface = std::make_shared<moveit::planning_interface::PlanningSceneInterface>();
  //signal(SIGINT, exit_sig_handler);
  RCLCPP_INFO(node_->get_logger(), "Starting AS");
  action_server_ = rclcpp_action::create_server<MoveArm>(
    node_,
    "move_arm",
    std::bind(handle_goal, std::placeholders::_1, std::placeholders::_2),
    std::bind(handle_cancel, std::placeholders::_1),
    std::bind(handle_accepted, std::placeholders::_1));
  RCLCPP_INFO(node_->get_logger(), "AS started");

  /*move_group->setPlannerId("RRTConnectkConfigDefault");
  move_group->setPlanningTime(10);
  move_group->setNumPlanningAttempts(3);
  move_group->setMaxVelocityScalingFactor(0.5);
  move_group->setMaxAccelerationScalingFactor(0.1);
  move_group->setGoalPositionTolerance(0.01);
  move_group->setGoalOrientationTolerance(0.01);

  geometry_msgs::msg::PoseStamped target_pose;
  target_pose.pose.position.x = 0.3;
  target_pose.pose.position.y = -0.1;
  target_pose.pose.position.z = 0.5;
  target_pose.pose.orientation.x = 1.0;
  target_pose.pose.orientation.y = 0.0;
  target_pose.pose.orientation.z = 0.0;
  target_pose.pose.orientation.w = 0.0;

  move_group->setPoseTarget(target_pose);
  moveit::planning_interface::MoveGroupInterface::Plan movePlan;
  RCLCPP_INFO(node->get_logger(), "Planning...");
  bool success = (move_group->plan(movePlan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(node->get_logger(), "Plan result: %d", success);
  if(success)
  {
      RCLCPP_INFO(node->get_logger(), "Executing...");
      move_group->execute(movePlan);
      RCLCPP_INFO(node->get_logger(), "Goal succeeded");
  }
  else
  {
      RCLCPP_INFO(node->get_logger(), "Goal aborted");
  }*/



  rclcpp::spin(node_);
  rclcpp::shutdown();
  return 0;
}

