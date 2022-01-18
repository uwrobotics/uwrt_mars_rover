#include "uwrt_mars_rover_arm_hw/uwrt_mars_rover_arm_hw_differential.hpp"

#include <bits/c++config.h>

#include <cmath>
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
  //TODO: get differential_transmission_reductions and differential_transmission_joint reductions from URDF
  state_transmission = std::make_shared<DifferentialTransmission>(
    differential_transmission_actuator_reductions, differential_transmission_joint_reductions);
  command_transmission = std::make_shared<DifferentialTransmission>(
    differential_transmission_actuator_reductions, differential_transmission_joint_reductions);

  if (!state_transmission || !command_transmission) {
    return false;
  };

  // instantiate maps with correct handles and interfaces
  configure_interfaces_and_handles();

  // instantiate vector of actuator handles and joint handles
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

ArmDifferentialSystemInterface::ArmDifferentialSystemInterface()
: system_logger(rclcpp::get_logger("ArmDifferentialSystemInterface"))
{
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
  if (hardware_interface::SystemInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    RCLCPP_FATAL_STREAM(system_logger, "FAILED TO PARSE ARM DIFFERENTIAL SYSTEM FROM URDF ...");
    return CallbackReturn::ERROR;
  }

  // setup logger
  system_logger = rclcpp::get_logger(this->info_.name);

  //validate ARM URDF differential joints
  if (this->info_.joints.size() != NUM_JOINTS) {
    RCLCPP_FATAL_STREAM(
      system_logger, "'" << info_.name.c_str() << "' has " << info_.joints.size()
                         << " joints. Expected: " << NUM_JOINTS << "'");
    return CallbackReturn::ERROR;
  }

  for (const auto & joint_component : this->info_.joints) {
    const auto & joint_name{joint_component.name};

    // check there are the right nubmer of state interfaces present for joint
    if (joint_component.state_interfaces.size() != NUM_STATE_INTERFACES) {
      RCLCPP_FATAL_STREAM(
        system_logger, "Joint '" << joint_name << "' has "
                                 << joint_component.state_interfaces.size()
                                 << " state interface. Expected: " << NUM_STATE_INTERFACES << "'");

      return CallbackReturn::ERROR;
    }

    // check there are the right number of command interfaces present for joint
    if (joint_component.command_interfaces.size() != NUM_COMMAND_INTERFACES) {
      RCLCPP_FATAL_STREAM(
        system_logger,
        "Joint '" << joint_name << "' has " << joint_component.command_interfaces.size()
                  << " command interface. Expected: " << NUM_COMMAND_INTERFACES << "'");

      return CallbackReturn::ERROR;
    }

    // check state interfaces for each joint
    for (const hardware_interface::InterfaceInfo & state_interfaces :
         joint_component.state_interfaces) {
      if (!(state_interfaces.name == hardware_interface::HW_IF_POSITION ||
            state_interfaces.name == hardware_interface::HW_IF_VELOCITY ||
            state_interfaces.name == ArmDifferentialSystemInterface::HW_IF_IQ_CURRENT)) {
        RCLCPP_FATAL_STREAM(
          system_logger, "Joint '" << joint_name << "'has " << state_interfaces.name.c_str()
                                   << "state interfaces. '" << hardware_interface::HW_IF_POSITION
                                   << "' or '" << hardware_interface::HW_IF_VELOCITY << "' or '"
                                   << ArmDifferentialSystemInterface::HW_IF_IQ_CURRENT
                                   << "' expected.");

        return CallbackReturn::ERROR;
      }
    }

    // check command interfaces for each joint
    for (const hardware_interface::InterfaceInfo & command_interfaces :
         joint_component.command_interfaces) {
      if (!(command_interfaces.name == hardware_interface::HW_IF_VELOCITY)) {
        RCLCPP_FATAL_STREAM(
          system_logger, "Joint '" << joint_name << "'has " << command_interfaces.name.c_str()
                                   << "command interfaces. '" << hardware_interface::HW_IF_VELOCITY
                                   << "' expected.");
        return CallbackReturn::ERROR;
      }
    }
  }

  // set intial states and command to 'Not a Number'
  for (const auto & joint_component : this->info_.joints) {
    const auto & joint_name{joint_component.name};

    joint_states.at(joint_name).velocity = std::numeric_limits<double>::quiet_NaN();
    actuator_states.at(joint_name).velocity = std::numeric_limits<double>::quiet_NaN();

    joint_states.at(joint_name).position = std::numeric_limits<double>::quiet_NaN();
    actuator_states.at(joint_name).position = std::numeric_limits<double>::quiet_NaN();

    actuator_states.at(joint_name).iq_current = std::numeric_limits<double>::quiet_NaN();

    joint_commands.at(joint_name).velocity = std::numeric_limits<double>::quiet_NaN();
    actuator_commands.at(joint_name).velocity = std::numeric_limits<double>::quiet_NaN();
  }

  //TODO: initialize Melvin CAN library

  //TODO: intialize transmission
  /***** Differential Transmission stuff
  if (!configure_differential_transmission()) {
    RCLCPP_ERROR_STREAM(
      system_logger, "Differential Transmission linked with: '"
                       << this->info_.joints.at(0).name << "' and '"
                       << this->info_.joints.at(1).name << "failed to be configured from URDF");
    return CallbackReturn::ERROR;
  }
  ******/

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmDifferentialSystemInterface::on_configure(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(system_logger, "Differential Arm System Configuring ...");

  for (const auto & joint_components : this->info_.joints) {
    const auto & joint_name{joint_components.name};
    /*
     * Set this to 0 every time configure is called regardless of its value
     * as we do not want to accidentally write a value that could be NaN or some garbage - causing unwanted movement
     * ideally we would do this to all the location of memory where the command interface resides
     */
    joint_commands.at(joint_name).velocity = 0;

    /* 
     * if the state interfaces are NaN (which may happen from jumping from different 'lifecyle states')
     * set them to 0, else just leave it - it will get correctly 
     * updated eventually in the update loop and we still keep the data from the last read this way. 
     * ideally we would so this to all the location of memory where the state interfaces resides
     */
    if (std::isnan(actuator_states.at(joint_name).velocity)) {
      actuator_states.at(joint_name).velocity = 0;
    }

    if (std::isnan(actuator_states.at(joint_name).position)) {
      actuator_states.at(joint_name).position = 0;
    }

    if (std::isnan(actuator_states.at(joint_name).iq_current)) {
      actuator_states.at(joint_name).iq_current = 0;
    }
  }

  RCLCPP_INFO(system_logger, "Differential Arm System Configured Successfully ...");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmDifferentialSystemInterface::on_activate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(system_logger, "Arm Differential Transmission System Activating");
  RCLCPP_INFO(system_logger, "Arm Differential Transmission System Activated Successfully");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmDifferentialSystemInterface::on_deactivate(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(system_logger, "Arm Differential Transmission System Deactivating");

  // for deactivation set commands to 0 to stop movement
  for (const auto & joint_component : this->info_.joints) {
    const auto & joint_name{joint_component.name};
    joint_commands.at(joint_name).velocity = 0;
  }

  RCLCPP_INFO(system_logger, "Arm Differential Transmission System Deactivating Successfully");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmDifferentialSystemInterface::on_shutdown(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(system_logger, "Arm Differential Transmission System Shutting Down");

  /*
     * Melvin CAN API Clean up
     */

  RCLCPP_INFO(system_logger, "Arm Differential Transmission System Shut Down Successfull");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmDifferentialSystemInterface::on_cleanup(
  const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;
  for (const auto & joint_component : this->info_.joints) {
    const auto & joint_name{joint_component.name};

    actuator_states.at(joint_name).velocity = std::numeric_limits<double>::quiet_NaN();
    actuator_states.at(joint_name).position = std::numeric_limits<double>::quiet_NaN();
    actuator_states.at(joint_name).iq_current = std::numeric_limits<double>::quiet_NaN();

    joint_commands.at(joint_name).velocity = std::numeric_limits<double>::quiet_NaN();
  }

  return CallbackReturn::SUCCESS;
}

return_type ArmDifferentialSystemInterface::write()
{
  RCLCPP_DEBUG(system_logger, "Arm Differential Transmission System Writing ...");

  /***
     * Would call Meshva's API function 'joint_to_actuator' 
     * to translate joint commands from ros controllers from joint space
     * into actuator space
     *
     */
  for (const auto & joint_components : this->info_.joints) {
    const auto & joint_name{joint_components.name};

    RCLCPP_INFO_STREAM(
      system_logger, "Joint: '" << joint_name << "': " << joint_commands.at(joint_name).velocity);

    /*
         * Melvin CAN API stuff
         *
         */
  }

  RCLCPP_DEBUG(system_logger, "Arm Differential Transmission System Write Succesfull ...");

  return return_type::OK;
}

return_type ArmDifferentialSystemInterface::read()
{
  RCLCPP_DEBUG(system_logger, "Arm Differential Transmission System Reading ...");
  /*
     * Would call Meshva's API function 'actuator_to_joint'
     * to translate states read from actuator into joint space.
     *
     */

  for (const auto & joint_components : this->info_.joints) {
    const auto & joint_name{joint_components.name};

    RCLCPP_INFO_STREAM(
      system_logger,
      "Joint: '" << joint_name << "': "
                 << "position (state): " << actuator_states.at(joint_name).position << " "
                 << "velocity (state): " << actuator_states.at(joint_name).velocity << " "
                 << "iq_current (state): " << actuator_states.at(joint_name).iq_current);

    /*
         * Melvin CAN API
         *
         */
  }

  RCLCPP_DEBUG(system_logger, "Arm Differential Transmission System Read Successful");

  return return_type::OK;
}

}  // namespace uwrt_mars_rover_arm_hw

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  uwrt_mars_rover_arm_hw::ArmDifferentialSystemInterface, hardware_interface::SystemInterface)
