#ifndef MOTION_PLAN_CLIENT_H
#define MOTION_PLAN_CLIENT_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "uwrt_mars_rover_arm_urdf_moveit/visibility_control.hpp"
#include "uwrt_mars_rover_arm_urdf_moveit/action/motion_plan.hpp"

namespace uwrt_motion_planning
{
class MotionPlanClient : public rclcpp::Node
{
  using MotionPlan = uwrt_mars_rover_arm_urdf_moveit::action::MotionPlan;
  using GoalHandleMotionPlan = rclcpp_action::ClientGoalHandle<uwrt_mars_rover_arm_urdf_moveit::action::MotionPlan>;

public:
  explicit MotionPlanClient(const rclcpp::NodeOptions & options);

protected:
  void execute();

private:
  rclcpp_action::Client<uwrt_mars_rover_arm_urdf_moveit::action::MotionPlan>::SharedPtr client_ptr_;
  rclcpp::TimerBase::SharedPtr timer_;

  void goal_response_callback(std::shared_future<GoalHandleMotionPlan::SharedPtr> future);
  void feedback_callback(GoalHandleMotionPlan::SharedPtr, const std::shared_ptr<const MotionPlan::Feedback> feedback);
  void result_callback(const GoalHandleMotionPlan::WrappedResult & result);
};

}  // namespace uwrt_motion_planning

#endif
