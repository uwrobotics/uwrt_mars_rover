#include "uwrt_mars_rover_arm_urdf_moveit/motion_plan_server.hpp"

#include <cinttypes>
#include <iostream>
#include <memory>
#include <functional>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include <moveit/trajectory_processing/iterative_time_parameterization.h>

namespace uwrt_motion_planning
{
MotionPlanServer::MotionPlanServer(const rclcpp::NodeOptions & options) : Node("Server", options)
{
  using namespace std::placeholders;

  action_srv_ = rclcpp_action::create_server<MotionPlan>(
      this,
      "MotionPlan",
      std::bind(&MotionPlanServer::handle_goal, this, _1, _2),
      std::bind(&MotionPlanServer::handle_cancel, this, _1),
      std::bind(&MotionPlanServer::handle_accepted, this, _1));

  RCLCPP_INFO(this->get_logger(), "CREATING SERVER");
}

rclcpp_action::GoalResponse MotionPlanServer::handle_goal(const rclcpp_action::GoalUUID & uuid,
  std::shared_ptr<const MotionPlan::Goal> goal)
{
  RCLCPP_INFO(this->get_logger(), "Received goal request");
  (void)uuid;
  return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
}

rclcpp_action::CancelResponse MotionPlanServer::handle_cancel(const std::shared_ptr<GoalHandleMotionPlan> goal_handle)
{
  RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
  (void)goal_handle;
  return rclcpp_action::CancelResponse::ACCEPT;
}

void MotionPlanServer::handle_accepted(const std::shared_ptr<GoalHandleMotionPlan> goal_handle)
{
  using namespace std::placeholders;
  // this needs to return quickly to avoid blocking the executor, so spin up a new thread
  std::thread{std::bind(&MotionPlanServer::execute, this, _1), goal_handle}.detach();
}

void MotionPlanServer::execute(const std::shared_ptr<GoalHandleMotionPlan> goal_handle) 
{
  auto result = std::make_shared<uwrt_mars_rover_arm_urdf_moveit::action::MotionPlan::Result>();

  static const std::string PLANNING_GROUP = "uwrt_arm";
  std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group =
    std::make_shared<moveit::planning_interface::MoveGroupInterface>(this->shared_from_this(), PLANNING_GROUP);

  move_group->setPoseTarget(goal_handle->get_goal()->pose);
  move_group->setPlanningTime(1.0);
  move_group->setMaxVelocityScalingFactor(1.0);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // try planning to point and record success/failure
  bool success = (move_group->plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(this->get_logger(), "Visualizing plan (pose goal) %s", success ? "" : "FAILED");

  // if cannot plan to pose, then abort
  result->success = success;
  if (!success) {
    goal_handle->succeed(result);
    return;
  }
  move_group->asyncExecute(my_plan);

  auto endJointPositions = my_plan.trajectory_.joint_trajectory.points[my_plan.trajectory_.joint_trajectory.points.size()-1].positions;
  double jointTolerance = move_group->getGoalJointTolerance();
  RCLCPP_INFO(this->get_logger(), "joint tolerance %f", jointTolerance);

  while (true) {
    std::vector<double> currentJointPos = move_group->getCurrentJointValues();
    std::string str;
    bool reached = true;

    // check whether we are within an error threshold of our joint positions to check if trajectory finished
    for (int i=0; i<currentJointPos.size(); ++i) {
      if (abs(endJointPositions[i] - currentJointPos[i]) > jointTolerance) {
        reached = false;
        break;
      }
    } 
    if (reached) break;

    // if cancelled, stop trajectory execution
    if (goal_handle->is_canceling()) {
      move_group->stop();
      result->success = false;
      goal_handle->canceled(result);
      break;
    }
  }
  goal_handle->succeed(result);
}

}  // namespace uwrt_motion_planning

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uwrt_motion_planning::MotionPlanServer)
