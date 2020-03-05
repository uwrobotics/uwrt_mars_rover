#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_science_real.h"

#include <pluginlib/class_list_macros.hpp>

namespace uwrt_mars_rover_hw {
bool UWRTRoverHWScienceReal::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!UWRTRoverHWScience::init(root_nh, robot_hw_nh)) {
    return false;
  }

  // TODO (wraftus) init CAN library

  return true;
}

// TODO (wraftus) implement
void UWRTRoverHWScienceReal::read(const ros::Time& time, const ros::Duration& period) {
  for (const auto& name : joint_names_) {

  }

  for (const auto& name : indexer_names_) {
    
  }
}

// TODO (wraftus) implement
void UWRTRoverHWScienceReal::write(const ros::Time& time, const ros::Duration& period) {
  for (const auto& name : joint_names_) {

  }

  for (const auto& name : indexer_names_) {

  }
}
}  // namespace uwrt_mars_rover_hw
PLUGINLIB_EXPORT_CLASS(uwrt_mars_rover_hw::UWRTRoverHWScienceReal, hardware_interface::RobotHW)