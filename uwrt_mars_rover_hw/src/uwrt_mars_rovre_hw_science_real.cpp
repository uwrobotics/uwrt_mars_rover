#include "uwrt_mars_rover_hw_science_real.h"

namespace uwrt_mars_rover_hw {
bool UWRTRoverHWScienceReal::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!UWRTRoverHWScience::init(root_nh, robot_hw_nh)) {
    return false;
  }

  // TODO (wraftus) init socketcan
  
  return true;
}

void UWRTRoverHWDrivetrainReal::read(const ros::Time& time, const ros::Duration& period) {
  // TODO (wraftus) read joint and sensor issue from science board, and update joint_state_
}
// TODO: Implement
void UWRTRoverHWDrivetrainReal::write(const ros::Time& time, const ros::Duration& period) {
  //TODO (wraftus) send new commands from joint_cmds_ to the science board
}

} // namespace uwrt_mars_rover_hw