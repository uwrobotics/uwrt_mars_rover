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
  for (const auto& name : joint_names_) {
    switch (joint_cmds_[name].type) {
      case ScienceJointCmd::Type::NONE:
        // TODO (wraftus) send none command to science joint
        break;
      case ScienceJointCmd::Type::VEL:
        // TODO (wraftus) send velocity command to science joint
        break;
      case ScienceJointCmd::Type::POS:
        // TODO (wraftus) send pos command to science joint
        break;
      case ScienceJointCmd::Type::EFF:
        // TODO (wraftus) send effort command to science joint
        break;
      default:
        ROS_ERROR_STREAM_NAMED(name_, "Unrecognized command type: " << static_cast<int>(joint_cmds_[name].type));
    }
  }
}

}  // namespace uwrt_mars_rover_hw