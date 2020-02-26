#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_science_real.h"

namespace uwrt_mars_rover_hw {
bool UWRTRoverHWScienceReal::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!UWRTRoverHWScience::init(root_nh, robot_hw_nh)) {
    return false;
  }

  // TODO (wraftus) init socketcan

  return true;
}

void UWRTRoverHWScienceReal::read(const ros::Time& time, const ros::Duration& period) {
  // TODO (wraftus) read joint and sensor issue from science board, and update joint_state_
}
// TODO: Implement
void UWRTRoverHWScienceReal::write(const ros::Time& time, const ros::Duration& period) {
  for (const auto& name : indexer_names_) {
    ROS_INFO_STREAM_THROTTLE(1, "Sending " << indexer_cmds_[name] << " to " << name);
  }
}

}  // namespace uwrt_mars_rover_hw