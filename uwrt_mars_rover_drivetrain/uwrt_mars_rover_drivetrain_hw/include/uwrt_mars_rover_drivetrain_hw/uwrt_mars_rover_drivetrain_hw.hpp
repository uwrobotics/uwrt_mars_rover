#pragma once

#include <cstdint>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <memory>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <transmission_interface/simple_transmission.hpp>
#include <vector>

#include "uwrt_mars_rover_drivetrain_hw/visibility.hpp"

using hardware_interface::return_type;

namespace uwrt_mars_rover_drivetrain_hw {
class UWRTMarsRoverDrivetrainHardware
    : public hardware_interface::BaseInterface<hardware_interface::ActuatorInterface> {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UWRTMarsRoverDrivetrainHardware)

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo& actuator_info) override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  return_type prepare_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                          const std::vector<std::string>& stop_interfaces) override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  return_type start() override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  return_type stop() override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  return_type read() override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  return_type write() override;

 private:
  // Parameters for drivetrain simulation
  double drivetrain_start_sec{};
  double drivetrain_stop_sec{};
  double drivetrain_slowdown{};

 protected:
  // actuator commands for velocity and position
  std::vector<double> drivetrain_commands_velocities;

  // actuator states for velocity
  std::vector<double> drivetrain_joint_position;
  
  // keep track of which command interface
  enum command_interface : std::uint32_t {
    VELOCITY_COMMAND = 0,
  };

  // keep track of which state interface
  enum state_interface : std::uint32_t {
    VELOCITY_STATE = 0,
    POSITION_STATE = 1,
  };

  struct CommandData {
    static double joint_position;
    static double joint_velocity;
    static double actuator_velocity;
  };

  using CommandData = struct CommandData;

  std::string joint_name;

  inline std::string get_joint_name() const {return joint_name;}
  inline void set_joint_name(const std::string& name) {joint_name = std::move(name);}

  // number of command interfaces for joints - look at urdf
  static constexpr unsigned int NUM_COMMAND_INTERFACES{1};

  // number of state interfaces for joints - look at urdf
  static constexpr unsigned int NUM_STATE_INTERFACES{2};

  // transmission stuff - there should only be one actuator linked to one joint
  transmission_interface::SimpleTransmission transmission = {0.0};

  // joint & actuator handles for the transmission
  transmission_interface::JointHandle velocity_joint_handle = {"", ""};
  transmission_interface::JointHandle positon_joint_handle = {"", ""};
  transmission_interface::ActuatorHandle velocity_actuator_handle = {"", ""};

  // parse urdf and get all drivetrain joints and transmissions
  UWRT_MARS_ROVER_DRIVETRAIN_HW_LOCAL
  bool configureDrivetrainJoints(const std::vector<hardware_interface::ComponentInfo>& drivetrainJoints);

  UWRT_MARS_ROVER_DRIVETRAIN_HW_LOCAL
  bool configureDrivetrainTransmissions(const std::vector<hardware_interface::ComponentInfo>& drivetrainTransmissions);

  UWRT_MARS_ROVER_DRIVETRAIN_HW_LOCAL
  bool configureDrivetrainHardwareInfo(const hardware_interface::HardwareInfo& drivetarinHardwareInfo);
};

}  // namespace uwrt_mars_rover_drivetrain_hw
