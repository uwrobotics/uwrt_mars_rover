#ifndef MOTION_PLAN_SERVER_H
#define MOTION_PLAN_SERVER_H

#include "uwrt_mars_rover_arm_urdf_moveit/visibility_control.hpp"
#include "uwrt_mars_rover_arm_urdf_moveit/srv/motion_plan.hpp"
#include "rclcpp/rclcpp.hpp"

#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/move_group_interface/move_group_interface.h>

namespace uwrt_motion_planning
{

class MotionPlanServer : public rclcpp::Node
{
public:
    UWRT_MOTION_PLANNING_PUBLIC
    explicit MotionPlanServer(const rclcpp::NodeOptions & options);

private:
    rclcpp::Service<uwrt_mars_rover_arm_urdf_moveit::srv::MotionPlan>::SharedPtr srv_;
    void solve_ik(const std::shared_ptr<uwrt_mars_rover_arm_urdf_moveit::srv::MotionPlan::Request> request,
            std::shared_ptr<uwrt_mars_rover_arm_urdf_moveit::srv::MotionPlan::Response> response);
};

}  // namespace uwrt_motion_planning

#endif

