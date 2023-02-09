#ifndef MOTION_PLAN_CLIENT_H
#define MOTION_PLAN_CLIENT_H

#include "rclcpp/rclcpp.hpp"
#include "uwrt_mars_rover_arm_urdf_moveit/srv/motion_plan.hpp"
#include "uwrt_mars_rover_arm_urdf_moveit/visibility_control.hpp"

namespace uwrt_motion_planning
{
class MotionPlanClient : public rclcpp::Node
{
public:
  explicit MotionPlanClient(const rclcpp::NodeOptions & options);

protected:
  void execute();

private:
  rclcpp::Client<uwrt_mars_rover_arm_urdf_moveit::srv::MotionPlan>::SharedPtr client_;
};

}  // namespace uwrt_motion_planning

#endif
