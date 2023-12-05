#ifndef MOTION_PLAN_SERVER_H
#define MOTION_PLAN_SERVER_H

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "uwrt_mars_rover_arm_urdf_moveit/action/motion_plan.hpp"
#include "uwrt_mars_rover_arm_urdf_moveit/visibility_control.hpp"

namespace uwrt_motion_planning
{
class MotionPlanServer : public rclcpp::Node
{
  using MotionPlan = uwrt_mars_rover_arm_urdf_moveit::action::MotionPlan;
  using GoalHandleMotionPlan = rclcpp_action::ServerGoalHandle<MotionPlan>;

public:
  UWRT_MOTION_PLANNING_PUBLIC
  explicit MotionPlanServer(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr trajectory_sub_;

  rclcpp_action::Server<uwrt_mars_rover_arm_urdf_moveit::action::MotionPlan>::SharedPtr action_srv_;

  rclcpp_action::GoalResponse handle_goal(
    const rclcpp_action::GoalUUID & uuid,
    std::shared_ptr<const MotionPlan::Goal> goal);

  rclcpp_action::CancelResponse handle_cancel(
    const std::shared_ptr<GoalHandleMotionPlan> goal_handle);

  void handle_accepted(const std::shared_ptr<GoalHandleMotionPlan> goal_handle);
  void execute(const std::shared_ptr<GoalHandleMotionPlan> goal_handle);
  void trajectory_callback(const std_msgs::msg::String::SharedPtr msg);
};

}  // namespace uwrt_motion_planning

#endif
