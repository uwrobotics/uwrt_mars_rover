#pragma once

#include <cstdint>
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <memory>
#include <rclcpp/macros.hpp>
#include <string>
#include <vector>

#include "uwrt_mars_rover_drivetrain_hw/visibility.hpp"

using hardware_interface::return_type;

namespace uwrt_mars_rover_drivetrain_hw {
class UWRTMarsRoverDrivetrainHardware : public hardware_interface::BaseInterface<hardware_interface::SystemInterface> {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UWRTMarsRoverDrivetrainHardware);

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo& info) override;

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
  // drivetrain commands for velocity and position
  std::vector<double> drivetrain_commands_velocities;
  std::vector<double> drivetrain_commands_positons;

  // drivetrain states for velocity and positon
  std::vector<double> drivetrain_velocities;
  std::vector<double> drivetrain_positions;

  // a way to maintain which command interface are we using to control drivetrain - velocity commands or position
  // commands
  enum class command_interface_type : std::uint8_t { UNDEFINED = 0, VELOCITY = 1, POSITION = 2 };

  std::vector<command_interface_type> control_type;

  // parse urdf and get all drivetrain joints and transmissions
  UWRT_MARS_ROVER_DRIVETRAIN_HW_LOCAL
  bool configureDrivetrainJoints(const hardware_interface::ComponentInfo& drivetrainJoints);

  UWRT_MARS_ROVER_DRIVETRAIN_HW_LOCAL
  bool configureDrivetrainTransmissions(const hardware_interface::ComponentInfo& drivetrainTransmissions);
  
  UWRT_MARS_ROVER_DRIVETRAIN_HW_LOCAL
  bool configureDrivetrainHardwareInfo(const hardware_interface::HardwareInfo& drivetarinHardwareInfo);
};

}  // namespace uwrt_mars_rover_drivetrain_hw
