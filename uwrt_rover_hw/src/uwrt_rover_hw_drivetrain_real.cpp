#include "uwrt_rover_hw_drivetrain_real.h"

namespace uwrt_rover_hw {
bool UWRTRoverHWDrivetrainReal::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (UWRTRoverHWDrivetrain::init(ros_nh, robot_hw_nh)) {
    return false;
  }
}

// TODO: Implement
bool UWRTRoverHWDrivetrainReal::read(const ros::Time& time, const ros::Duration& period) {
  return false;
}
// TODO: Implement
bool UWRTRoverHWDrivetrainReal::write(const ros::Time& time, const ros::Duration& period) {
  return false;
}
}  // namespace uwrt_rover_hw