#include "uwrt_mars_rover_arm_hw/uwrt_mars_rover_arm_hw_differential.hpp"

#include <bits/c++config.h>

#include <cstdlib>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "transmission_interface/differential_transmission.hpp"
#include "transmission_interface/handle.hpp"

using hardware_interface::CommandInterface;
using hardware_interface::HardwareInfo;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

using transmission_interface::ActuatorHandle;
using transmission_interface::DifferentialTransmission;
using transmission_interface::JointHandle;

using rclcpp_lifecycle::State;

namespace uwrt_mars_rover_arm_hw
{
void ArmDifferentialSystemInterface::configure_interfaces_and_handles()
{
  for (const auto & joint : this->info_.joints) {
    const auto & joint_name{joint.name};

    // instantiate state interfaces
    joint_states.insert_or_assign(joint_name, DifferentialTransmissionData::JointData::StateData{});
    actuator_states.insert_or_assign(
      joint_name, DifferentialTransmissionData::ActuatorData::StateData{});

    // instantiate command interfaces
    actuator_commands.insert_or_assign(
      joint_name, DifferentialTransmissionData::ActuatorData::CommandData{});
    joint_commands.insert_or_assign(
      joint_name, DifferentialTransmissionData::JointData::CommandData{});

    // instantiate state handles
    joint_position_state_handle.insert_or_assign(
      joint_name,
      JointHandle{
        joint_name, hardware_interface::HW_IF_POSITION, &joint_states.at(joint_name).position});
    joint_velocity_state_hanlde.insert_or_assign(
      joint_name,
      JointHandle{
        joint_name, hardware_interface::HW_IF_VELOCITY, &joint_states.at(joint_name).velocity});
    actuator_position_state_handle.insert_or_assign(
      joint_name,
      ActuatorHandle{
        joint_name, hardware_interface::HW_IF_POSITION, &actuator_states.at(joint_name).position});
    actuator_velocity_state_handle.insert_or_assign(
      joint_name,
      ActuatorHandle{
        joint_name, hardware_interface::HW_IF_VELOCITY, &actuator_states.at(joint_name).velocity});

    // instantiate command handles
    joint_velocity_command_handle.insert_or_assign(
      joint_name,
      JointHandle{
        joint_name, hardware_interface::HW_IF_VELOCITY, &joint_commands.at(joint_name).velocity});
    actuator_velocity_command_handle.insert_or_assign(
      joint_name, ActuatorHandle{
                    joint_name, hardware_interface::HW_IF_VELOCITY,
                    &actuator_commands.at(joint_name).velocity});
  }
}

bool ArmDifferentialSystemInterface::configure_differential_transmission()
{
  state_transmission = std::make_shared<DifferentialTransmission>(
    differential_transmission_actuator_reductions, differential_transmission_joint_reductions);
  command_transmission = std::make_shared<DifferentialTransmission>(
    differential_transmission_actuator_reductions, differential_transmission_joint_reductions);

  if (!state_transmission || !command_transmission) {
    return false;
  };

  // create vector of actuator handles and joint handles
  for (const auto & joint : this->info_.joints) {
    const auto & joint_name{joint.name};

    // joint handles for state transmission
    state_joint_handle_vector.push_back(joint_position_state_handle.at(joint_name));
    state_joint_handle_vector.push_back(joint_velocity_state_hanlde.at(joint_name));

    // actuator handles for state transmission
    state_actuator_handle_vector.push_back(actuator_position_state_handle.at(joint_name));
    state_actuator_handle_vector.push_back(actuator_velocity_state_handle.at(joint_name));

    // joint handles for command transmission
    command_joint_handle_vector.push_back(joint_velocity_command_handle.at(joint_name));

    // actuator handles for command transmission
    command_actuator_handle_vector.push_back(actuator_velocity_command_handle.at(joint_name));
  }

  // configure state & command differential transmissons
  state_transmission->configure(state_joint_handle_vector, state_actuator_handle_vector);
  command_transmission->configure(command_joint_handle_vector, command_actuator_handle_vector);

  return true;
}

std::vector<StateInterface> ArmDifferentialSystemInterface::export_state_interfaces()
{
  std::vector<StateInterface> state_interface_lists{};

  for (const auto & joint : this->info_.joints) {
    const auto & joint_name{joint.name};

    state_interface_lists.emplace_back(StateInterface(
      joint_name, hardware_interface::HW_IF_VELOCITY, &actuator_states.at(joint_name).velocity));
    state_interface_lists.emplace_back(StateInterface(
      joint_name, hardware_interface::HW_IF_POSITION, &actuator_states.at(joint_name).position));
    state_interface_lists.emplace_back(StateInterface(
      joint_name, ArmDifferentialSystemInterface::HW_IF_IQ_CURRENT,
      &actuator_states.at(joint_name).iq_current));
  }

  return state_interface_lists;
}

std::vector<CommandInterface> ArmDifferentialSystemInterface::export_command_interfaces()
{
  std::vector<CommandInterface> command_interfaces_lists{};

  for (const auto & joint : this->info_.joints) {
    const auto & joint_name{joint.name};

    command_interfaces_lists.emplace_back(CommandInterface(
      joint_name, hardware_interface::HW_IF_VELOCITY, &joint_commands.at(joint_name).velocity));
  }

  return command_interfaces_lists;
}

/*
 *
 * This will be needed when we have multiple methods of control 
 *
 */
return_type ArmDifferentialSystemInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  (void)start_interfaces;
  (void)stop_interfaces;

  return return_type::OK;
}

CallbackReturn ArmDifferentialSystemInterface::on_init(
  const hardware_interface::HardwareInfo & hardware_info)
{
  const auto & t = hardware_info;
  (void)t;
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmDifferentialSystemInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmDifferentialSystemInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmDifferentialSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmDifferentialSystemInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmDifferentialSystemInterface::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  return CallbackReturn::SUCCESS;
}

return_type ArmDifferentialSystemInterface::write() { return return_type::OK; }

return_type ArmDifferentialSystemInterface::read() { return return_type::OK; }

}  // namespace uwrt_mars_rover_arm_hw

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  uwrt_mars_rover_arm_hw::ArmDifferentialSystemInterface, hardware_interface::SystemInterface)
