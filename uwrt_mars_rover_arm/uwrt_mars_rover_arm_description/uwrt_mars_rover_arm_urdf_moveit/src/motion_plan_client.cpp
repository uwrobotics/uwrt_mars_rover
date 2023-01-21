#include "uwrt_mars_rover_arm_urdf_moveit/motion_plan_client.hpp"

#include <cinttypes>
#include <iostream>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/pose.hpp"

using namespace std::chrono_literals;

namespace uwrt_motion_planning
{

MotionPlanClient::MotionPlanClient(const rclcpp::NodeOptions & options)
: Node("Client", options)
{
  client_ = create_client<uwrt_mars_rover_arm_urdf_moveit::srv::MotionPlan>("motion_plan");

  //create timer just for testing purposes
  // timer_ = create_wall_timer(2s, std::bind(&MotionPlanClient::on_timer, this));
  on_timer();
}

void MotionPlanClient::on_timer()
{
  if (!client_->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(
        this->get_logger(),
        "Interrupted while waiting for the service. Exiting.");
      return;
    }
    RCLCPP_INFO(this->get_logger(), "Service not available after waiting");
    return;
  }

  geometry_msgs::msg::Pose target;
  target.orientation.w = 1.0;
  target.position.x = 0.28;
  target.position.y = -0.2;
  target.position.z = 0.5;

  auto request = std::make_shared<uwrt_mars_rover_arm_urdf_moveit::srv::MotionPlan::Request>();
  request->pose = target;

  using ServiceResponseFuture =
    rclcpp::Client<uwrt_mars_rover_arm_urdf_moveit::srv::MotionPlan>::SharedFuture;
  auto response_received_callback = [this](ServiceResponseFuture future) {
      RCLCPP_INFO(this->get_logger(), "Got result: [%d]", future.get()->success);
    };
  auto future_result = client_->async_send_request(request, response_received_callback);
}

}

#include "rclcpp_components/register_node_macro.hpp"

RCLCPP_COMPONENTS_REGISTER_NODE(uwrt_motion_planning::MotionPlanClient)
