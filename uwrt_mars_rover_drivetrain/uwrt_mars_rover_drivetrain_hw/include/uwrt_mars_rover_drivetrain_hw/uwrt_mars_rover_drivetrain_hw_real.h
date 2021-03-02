#pragma once

#include <uwrt_mars_rover_drivetrain_hw/uwrt_mars_rover_drivetrain_hw.h>

#include "roboteq_driver/RoboteqController.hpp"

namespace uwrt_mars_rover_drivetrain_hw {

class UWRTMarsRoverDrivetrainHWReal : public UWRTRoverHWDrivetrain {
 public:
  explicit UWRTMarsRoverDrivetrainHWReal() : UWRTRoverHWDrivetrain("UWRTRoverHWDrivetrainReal"){};

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;
  void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                const std::list<hardware_interface::ControllerInfo> &stop_list) override;

 private:
  bool loadRoboteqConfigFromParamServer(ros::NodeHandle& robot_hw_nh);

  std::unique_ptr<roboteq::RoboteqController> motor_controller_;
  uint8_t roboteq_canopen_id_{0};

  std::map<std::string, uint8_t> roboteq_actuator_index_;
  std::map<std::string, int> ticks_per_revolution_;
};

}  // namespace uwrt_mars_rover_drivetrain_hw
