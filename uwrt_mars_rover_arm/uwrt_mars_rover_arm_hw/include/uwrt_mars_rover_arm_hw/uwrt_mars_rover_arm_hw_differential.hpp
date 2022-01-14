#pragma once

#include <bits/c++config.h>

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <map>
#include <memory>
#include <rclcpp/logger.hpp>
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <transmission_interface/differential_transmission.hpp>
#include <vector>

#include "transmission_interface/handle.hpp"
#include "uwrt_mars_rover_arm_hw/uwrt_mars_rover_arm_hw_actuator.hpp"
#include "uwrt_mars_rover_arm_hw/visibility.hpp"

namespace uwrt_mars_rover_arm_hw
{
namespace DifferentialTransmissionData
{
namespace ActuatorData
{
using CommandData = struct CommandData
{
  double velocity{};
};

using StateData = struct StateData
{
  double velocity{};
  double position{};
  double iq_current{};  // not used for differential transmission
};

}  // namespace ActuatorData
namespace JointData
{
using CommandData = struct CommandData
{
  double velocity{};
};

using StateData = struct StateData
{
  double velocity{};
  double position{};
};

}  // namespace JointData

}  // namespace DifferentialTransmissionData

class ArmDifferentialSystemInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ArmDifferentialSystemInterface)

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
  void configure_interfaces_and_handles();

  UWRT_MARS_ROVER_ARM_HW_LOCAL
  bool configure_differential_transmission();

private:
  using ActuatorCommandsData_t =
    std::map<const std::string, DifferentialTransmissionData::ActuatorData::CommandData>;
  using ActuatorStatesData_t =
    std::map<const std::string, DifferentialTransmissionData::ActuatorData::StateData>;
  using JointStatesData_t =
    std::map<const std::string, DifferentialTransmissionData::JointData::StateData>;
  using JointCommandsData_t =
    std::map<const std::string, DifferentialTransmissionData::JointData::CommandData>;

protected:
  // ARM Actuator URDF defines these numbers
  inline static constexpr std::size_t NUM_JOINTS{2};
  inline static constexpr std::size_t NUM_ACTUATORS{2};
  inline static constexpr std::size_t NUM_STATE_INTERFACES{3};
  inline static constexpr std::size_t NUM_COMMAND_INTERFACES{1};

  // holds actuator reads/writes
  ActuatorCommandsData_t actuator_commands{};
  ActuatorStatesData_t actuator_states{};
  JointCommandsData_t joint_commands{};
  JointStatesData_t joint_states{};

  // transmission for state(reads) & command(writes) interfaces
  std::shared_ptr<transmission_interface::DifferentialTransmission> command_transmission{nullptr};
  std::shared_ptr<transmission_interface::DifferentialTransmission> state_transmission{nullptr};

  // create actuator & joint handles for state transmission
  std::map<const std::string, transmission_interface::ActuatorHandle>
    actuator_velocity_state_handle{};
  std::map<const std::string, transmission_interface::JointHandle> joint_velocity_state_hanlde{};
  std::map<const std::string, transmission_interface::ActuatorHandle>
    actuator_position_state_handle{};
  std::map<const std::string, transmission_interface::JointHandle> joint_position_state_handle{};

  // create actuator & joint handles for command transmission
  std::map<const std::string, transmission_interface::ActuatorHandle>
    actuator_velocity_command_handle{};
  std::map<const std::string, transmission_interface::JointHandle> joint_velocity_command_handle{};

  // vector to contain reductions of actuators to joints
  std::vector<double> differential_transmission_actuator_reductions{};
  std::vector<double> differential_transmission_joint_reductions{};

  // vector of handles for state transmission configuration
  std::vector<transmission_interface::JointHandle> state_joint_handle_vector{};
  std::vector<transmission_interface::ActuatorHandle> state_actuator_handle_vector{};

  // vector of handles for command transmission configuration
  std::vector<transmission_interface::JointHandle> command_joint_handle_vector{};
  std::vector<transmission_interface::ActuatorHandle> command_actuator_handle_vector{};

  // logger for system interface
  std::function<rclcpp::Logger(void)> system_logger = [this]() {
    return rclcpp::get_logger(this->info_.name);
  };

  // iq_current interface
  static constexpr char HW_IF_IQ_CURRENT[] = "iq_current";
};

}  // namespace uwrt_mars_rover_arm_hw
