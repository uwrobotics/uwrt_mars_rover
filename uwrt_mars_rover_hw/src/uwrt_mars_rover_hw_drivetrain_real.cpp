#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_drivetrain_real.h"

namespace uwrt_mars_rover_hw {
bool UWRTRoverHWDrivetrainReal::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (UWRTRoverHWDrivetrain::init(root_nh, robot_hw_nh)) {
    return false;
  }

  // TODO: roboteq driver implementation
  return true;
}

// TODO: Implement
void UWRTRoverHWDrivetrainReal::read(const ros::Time& time, const ros::Duration& period) {}
// TODO: Implement
void UWRTRoverHWDrivetrainReal::write(const ros::Time& time, const ros::Duration& period) {}
}  // namespace uwrt_mars_rover_hw
