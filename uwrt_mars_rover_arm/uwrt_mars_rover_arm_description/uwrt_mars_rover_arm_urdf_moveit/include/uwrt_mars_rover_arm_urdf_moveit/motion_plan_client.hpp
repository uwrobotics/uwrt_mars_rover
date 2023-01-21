// #include <rclcpp/rclcpp.hpp>
// #include <uwrt_mars_rover_arm_urdf_moveit/srv/motion_plan.hpp>
// #include <uwrt_mars_rover_arm_urdf_moveit/visibility_control.hpp>

// #include "example_interfaces/srv/add_two_ints.hpp"

// namespace uwrt_motion_planning {

// class MotionPlanClient : public rclcpp::Node {
// public:
//     UWRT_MOTION_PLANNING_PUBLIC
//     explicit MotionPlanClient(const rclcpp::NodeOptions& options);
// private:
//     // rclcpp::Client<uwrt_mars_rover_arm_urdf_moveit::srv::MotionPlan>::SharedPtr client;
//     rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client;
//     rclcpp::TimerBase::SharedPtr timer;

//     void on_timer();

// };
// }

// #endif

#ifndef MOTION_PLAN_CLIENT_H
#define MOTION_PLAN_CLIENT_H

#include "uwrt_mars_rover_arm_urdf_moveit/srv/motion_plan.hpp"
#include "uwrt_mars_rover_arm_urdf_moveit/visibility_control.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"
#include "rclcpp/rclcpp.hpp"

namespace uwrt_motion_planning
{

class MotionPlanClient : public rclcpp::Node
{
public:
  explicit MotionPlanClient(const rclcpp::NodeOptions & options);

protected:
  void on_timer();

private:
  rclcpp::Client<uwrt_mars_rover_arm_urdf_moveit::srv::MotionPlan>::SharedPtr client_;
  rclcpp::TimerBase::SharedPtr timer_;
};

}  // namespace uwrt_motion_planning

#endif  // COMPOSITION__CLIENT_COMPONENT_HPP_
