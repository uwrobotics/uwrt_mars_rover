#pragma once

#include <memory>

#include "controller_interface/controller_interface.hpp"
#include "geometry_msgs/msg/twist_stamped.hpp"
#include "kdl/chainiksolver.hpp"
#include "kdl/chainiksolvervel_pinv.hpp"
#include "kdl/tree.hpp"
#include "rclcpp/subscription.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_buffer.h"
#include "uwrt_arm_cartesian_controller/visibility_control.hpp"

namespace uwrt_arm_cartesian_controller
{
using TwistStamped = geometry_msgs::msg::TwistStamped;
using CallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class UWRTCartesianController : public controller_interface::ControllerInterface
{
public:
  UWRTCartesianController();

  ~UWRTCartesianController();

  UWRT_ARM_CARTESIAN_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  UWRT_ARM_CARTESIAN_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  UWRT_ARM_CARTESIAN_CONTROLLER_PUBLIC
  CallbackReturn on_init() override;

  UWRT_ARM_CARTESIAN_CONTROLLER_PUBLIC
  CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  UWRT_ARM_CARTESIAN_CONTROLLER_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;

  UWRT_ARM_CARTESIAN_CONTROLLER_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;

  UWRT_ARM_CARTESIAN_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

protected:
  rclcpp::Subscription<TwistStamped>::SharedPtr m_cart_command_sub;

  std::vector<std::string> m_joint_names;
  std::string m_cart_command_topic, m_robot_description, m_root_name, m_tip_name;

private:
  bool parse_params();
  TwistStamped get_null_twist() const;

  KDL::Tree m_kdl_tree;
  KDL::Chain m_kdl_chain;
  std::unique_ptr<KDL::ChainIkSolverVel> m_ik_solver;

  realtime_tools::RealtimeBuffer<std::shared_ptr<TwistStamped>> m_rt_buffer;

  static constexpr auto COMMAND_INTERFACE = "velocity";
  static constexpr auto STATE_INTERFACE = "position";
};

}  // namespace uwrt_arm_cartesian_controller
