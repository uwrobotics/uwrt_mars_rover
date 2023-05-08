#include "uwrt_mars_rover_arm_urdf_moveit/motion_plan_client.hpp"

#include <cinttypes>
#include <iostream>
#include <memory>

#include "geometry_msgs/msg/pose.hpp"
#include "rclcpp/rclcpp.hpp"

using namespace std::chrono_literals;
using namespace std::placeholders;

namespace uwrt_motion_planning
{
MotionPlanClient::MotionPlanClient(const rclcpp::NodeOptions & options) : Node("Client", options)
{
//   client_ = create_client<uwrt_mars_rover_arm_urdf_moveit::srv::MotionPlan>("motion_plan");

  this->client_ptr_ = rclcpp_action::create_client<uwrt_mars_rover_arm_urdf_moveit::action::MotionPlan>(
    this,
    "MotionPlan");

  this->timer_ = this->create_wall_timer(
    std::chrono::milliseconds(500),
    std::bind(&MotionPlanClient::execute, this));

  execute();
}

void MotionPlanClient::execute()
{
  geometry_msgs::msg::Pose target;
  target.orientation.w = 1.0;
  target.position.x = 0.28;
  target.position.y = -0.2;
  target.position.z = 0.5;

  this->timer_->cancel();

  if (!this->client_ptr_->wait_for_action_server()) {
    RCLCPP_ERROR(this->get_logger(), "Action server not available after waiting");
    rclcpp::shutdown();
  }

  auto goal_msg = uwrt_mars_rover_arm_urdf_moveit::action::MotionPlan::Goal();
  goal_msg.pose = target;

  RCLCPP_INFO(this->get_logger(), "SENDING GOAL");

  auto send_goal_options = rclcpp_action::Client<uwrt_mars_rover_arm_urdf_moveit::action::MotionPlan>::SendGoalOptions();
  send_goal_options.goal_response_callback =
    std::bind(&MotionPlanClient::goal_response_callback, this, _1);
  send_goal_options.feedback_callback =
    std::bind(&MotionPlanClient::feedback_callback, this, _1, _2);
  send_goal_options.result_callback =
    std::bind(&MotionPlanClient::result_callback, this, _1);

  this->client_ptr_->async_send_goal(goal_msg, send_goal_options);

}

void MotionPlanClient::goal_response_callback(std::shared_future<GoalHandleMotionPlan::SharedPtr> future)
{
  auto goal_handle = future.get();
  if (!goal_handle) {
    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
  } else {
    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, waiting for result");
  }
}

void MotionPlanClient::feedback_callback(
  GoalHandleMotionPlan::SharedPtr,
  const std::shared_ptr<const MotionPlan::Feedback> feedback)
{
}

void MotionPlanClient::result_callback(const GoalHandleMotionPlan::WrappedResult & result)
{
  switch (result.code) {
    case rclcpp_action::ResultCode::SUCCEEDED:
      break;
    case rclcpp_action::ResultCode::ABORTED:
      RCLCPP_ERROR(this->get_logger(), "Goal was aborted");
      return;
    case rclcpp_action::ResultCode::CANCELED:
      RCLCPP_ERROR(this->get_logger(), "Goal was canceled");
      return;
    default:
      RCLCPP_ERROR(this->get_logger(), "Unknown result code");
      return;
  }
  RCLCPP_INFO(this->get_logger(), "DONE");
  // rclcpp::shutdown();
}

}  // namespace uwrt_motion_planning

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(uwrt_motion_planning::MotionPlanClient)
