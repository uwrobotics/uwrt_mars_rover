#include "uwrt_arm_hw/voltage_joint_interface.h"

// Pluginlib
#include <pluginlib/class_list_macros.hpp>

// Project
#include <gripper_action_controller/gripper_action_controller.h>

namespace voltage_controllers
{
  /**
   * \brief Gripper action controller that sends
   * commands to a \b voltage interface.
   */
  typedef gripper_action_controller::GripperActionController<hardware_interface::VoltageJointInterface>
          GripperActionController;
}

PLUGINLIB_EXPORT_CLASS(voltage_controllers::GripperActionController, controller_interface::ControllerBase)
