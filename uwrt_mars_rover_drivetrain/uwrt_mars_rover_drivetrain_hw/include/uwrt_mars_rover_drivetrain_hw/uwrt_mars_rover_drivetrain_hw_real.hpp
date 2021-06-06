#pragma once

#include "uwrt_mars_rover_drivetrain_hw/uwrt_mars_rover_drivetrain_hw.hpp"
#include "uwrt_mars_rover_drivetrain_hw/visibility.hpp"

using hardware_interface::return_type;

namespace uwrt_mars_rover_drivetrain_hw {

class UWRTMarsRoverDrivetrainHardwareReal : public UWRTMarsRoverDrivetrainHardware {

  RCLCPP_SHARED_PTR_DEFINITIONS(UWRTMarsRoverDrivetrainHardwareReal)

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  return_type configure(const hardware_interface::HardwareInfo& actuator_info) override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  return_type perform_command_mode_switch(const std::vector<std::string>& start_interfaces,
                                          const std::vector<std::string>& stop_interfaces) override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  return_type read() override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  return_type write() override;
};

}  // namespace uwrt_mars_rover_drivetrain_hw