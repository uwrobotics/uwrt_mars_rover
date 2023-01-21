#include "uwrt_mars_rover_arm_urdf_moveit/motion_plan_server.hpp"

#include <cinttypes>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"

namespace uwrt_motion_planning {

MotionPlanServer::MotionPlanServer(const rclcpp::NodeOptions& options) : Node("Server", options) {
  srv_ = create_service<uwrt_mars_rover_arm_urdf_moveit::srv::MotionPlan>(
      "motion_plan", std::bind(&MotionPlanServer::solve_ik, this, std::placeholders::_1, std::placeholders::_2));
}

void MotionPlanServer::solve_ik(
    const std::shared_ptr<uwrt_mars_rover_arm_urdf_moveit::srv::MotionPlan::Request> request,
    std::shared_ptr<uwrt_mars_rover_arm_urdf_moveit::srv::MotionPlan::Response> response) {
  RCLCPP_INFO(this->get_logger(), "Incoming request");

  // set move_group that we are planning for
  static const std::string PLANNING_GROUP = "uwrt_arm";
  moveit::planning_interface::MoveGroupInterface move_group(this->shared_from_this(), PLANNING_GROUP);

  move_group.setPoseTarget(request->pose);

  moveit::planning_interface::MoveGroupInterface::Plan my_plan;

  // try planning to point and record success/failure
  bool success = (move_group.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);
  RCLCPP_INFO(this->get_logger(), "Visualizing plan 1 (pose goal) %s", success ? "" : "FAILED");

  // move to position if planning is successful
  response->success = success;
  if (success) move_group.move();
}

}  // namespace uwrt_motion_planning

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(uwrt_motion_planning::MotionPlanServer)
