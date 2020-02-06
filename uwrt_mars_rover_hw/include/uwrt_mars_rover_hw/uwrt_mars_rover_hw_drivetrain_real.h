#pragma once

#include <uwrt_mars_rover_hw/uwrt_mars_rover_hw_drivetrain.h>

#include "RoboteqController.hpp"

namespace uwrt_mars_rover_hw {

class UWRTRoverHWDrivetrainReal : public UWRTRoverHWDrivetrain {
 public:
  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;

 private:
  // TODO: load this from a yaml and/or rosparam?
  std::map<std::string, uint8_t> joint_roboteq_index_ = {{"right_drive_joint", 0}, {"left_drive_joint", 1}};
  std::unique_ptr<roboteq::RoboteqController> motor_controller_;
};

}  // namespace uwrt_mars_rover_hw
