#pragma once

#include <bits/c++config.h>

#include <cmath>
#include <cstdint>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <transmission_interface/simple_transmission.hpp>
#include <vector>
#include <uwrt_mars_rover_utils/data_utils.hpp>
#include "uwrt_mars_rover_arm_hw/visibility.hpp"

namespace uwrt_mars_rover_arm_hw
{

class ArmActuatorInterface : public hardware_interface::ActuatorInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmActuatorInterface)

  UWRT_MARS_ROVER_ARM_HW_PUBLIC
  ArmActuatorInterface();

  UWRT_MARS_ROVER_ARM_HW_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  UWRT_MARS_ROVER_ARM_HW_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  UWRT_MARS_ROVER_ARM_HW_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & /* start_interfaces */,
    const std::vector<std::string> & /* stop interfaces */) override;

  UWRT_MARS_ROVER_ARM_HW_PUBLIC
  hardware_interface::return_type read() override;

  UWRT_MARS_ROVER_ARM_HW_PUBLIC
  hardware_interface::return_type write() override;

  /*
         * Lifecycle node member functions
         *
         */
  UWRT_MARS_ROVER_ARM_HW_PUBLIC
  CallbackReturn on_init(
    const hardware_interface::HardwareInfo & hardware_info) override;  // intialize

  UWRT_MARS_ROVER_ARM_HW_PUBLIC
  CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;  // configuration

  UWRT_MARS_ROVER_ARM_HW_PUBLIC
  CallbackReturn on_activate(const rclcpp_lifecycle::State & previous_state) override;  // start

  UWRT_MARS_ROVER_ARM_HW_PUBLIC
  CallbackReturn on_deactivate(const rclcpp_lifecycle::State & previous_state) override;  // stop

  UWRT_MARS_ROVER_ARM_HW_PUBLIC
  CallbackReturn on_shutdown(const rclcpp_lifecycle::State & previous_state)
    override;  // free up resources before destruction

  UWRT_MARS_ROVER_ARM_HW_PUBLIC
  CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;  // set node unconfigured state

protected: /*idk we might extend for some reason*/
  UWRT_MARS_ROVER_ARM_HW_LOCAL
  bool configure_transmission();

  // ARM Actuator URDF defines these numbers
  inline static constexpr std::size_t NUM_JOINTS{1};
  inline static constexpr std::size_t NUM_STATE_INTERFACES{3};
  inline static constexpr std::size_t NUM_COMMAND_INTERFACES{1};

  // holds actuator reads/writes
  DifferentialTransmissionData::ActuatorData::CommandData actuator_commands{};
  DifferentialTransmissionData::ActuatorData::StateData actuator_states{};
  DifferentialTransmissionData::JointData::CommandData joint_commands{};
  DifferentialTransmissionData::JointData::StateData joint_states{};

  // transmission for state & command transmission
  std::shared_ptr<transmission_interface::SimpleTransmission> command_transmission{nullptr};
  std::shared_ptr<transmission_interface::SimpleTransmission> state_transmission{nullptr};

  // create actuator & joint handles for state transmission
  std::shared_ptr<transmission_interface::ActuatorHandle> actuator_velocity_state_handle{nullptr};
  std::shared_ptr<transmission_interface::JointHandle> joint_velocity_state_hanlde{nullptr};
  std::shared_ptr<transmission_interface::ActuatorHandle> actuator_position_state_handle{nullptr};
  std::shared_ptr<transmission_interface::JointHandle> joint_position_state_handle{nullptr};

  // create actuator & joint handles for command transmission
  std::shared_ptr<transmission_interface::ActuatorHandle> actuator_velocity_command_handle{nullptr};
  std::shared_ptr<transmission_interface::JointHandle> joint_velocity_command_handle{nullptr};

  // TODO: pull down this value from the transmission
  double joint_to_actuator_reduction{};

  rclcpp::Logger actuator_logger;

  // iq_current interface
  static constexpr char HW_IF_IQ_CURRENT[] = "iq_current";
};

}  // namespace uwrt_mars_rover_arm_hw
