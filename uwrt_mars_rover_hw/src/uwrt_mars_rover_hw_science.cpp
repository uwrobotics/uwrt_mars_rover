#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_science.h"

namespace uwrt_mars_rover_hw {

bool UWRTRoverHWScience::init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle& robot_hw_nh) {
  if (!robot_hw_nh.getParam("science_joints", joint_names_)) {
    ROS_ERROR_STREAM_NAMED(name_, "could not find science joints from parameter server");
    return false;
  }

  // connect and register the state and command interfaces
  for (const auto& name : joint_names_) {
    ROS_ERROR_STREAM_NAMED(name_, "found science joint: " << name);

    // joint state interface
    hardware_interface::JointStateHandle state_handle(name, &joint_states_[name].pos, &joint_states_[name].vel,
                                                      &joint_states_[name].pos);
    joint_state_interface_.registerHandle(state_handle);

    // joint command interfaces
    hardware_interface::JointHandle cmd_handle(joint_state_interface_.getHandle(name), &joint_cmds_[name].data);
    joint_pos_interface_.registerHandle(cmd_handle);
    joint_vel_interface_.registerHandle(cmd_handle);
    joint_eff_interface_.registerHandle(cmd_handle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&joint_pos_interface_);
  registerInterface(&joint_vel_interface_);
  registerInterface(&joint_eff_interface_);

  return true;
}

void UWRTRoverHWScience::read(const ros::Time& time, const ros::Duration& period) {
  ROS_ERROR_STREAM_NAMED(name_, "Read called from base science_hw class, please only call from real or sim classes");
}

void UWRTRoverHWScience::write(const ros::Time& time, const ros::Duration& period) {
  ROS_ERROR_STREAM_NAMED(name_, "Write called from base science_hw class, please only call from real or sim classes");
}

void UWRTRoverHWScience::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                  const std::list<hardware_interface::ControllerInfo>& stop_list) {
  // clear commands for controllers to stop
  for (const auto& controller : stop_list) {
    for (const auto& claimed : controller.claimed_resources) {
      for (const auto& resource : claimed.resources) {
        joint_cmds_[resource].type = ScienceJointCmd::Type::NONE;
        joint_cmds_[resource].data = 0.0;
      }
    }
  }

  for (const auto& controller : start_list) {
    for (const auto& claimed : controller.claimed_resources) {
      for (const auto& resource : claimed.resources) {
        if (claimed.hardware_interface == "hardware_interface::PositionJointInterface") {
          joint_cmds_[resource].type = ScienceJointCmd::Type::POS;
          joint_cmds_[resource].data = joint_states_[resource].pos;
        } else if (claimed.hardware_interface == "hardware_interface::VelocityJointInterface") {
          joint_cmds_[resource].type = ScienceJointCmd::Type::VEL;
          joint_cmds_[resource].data = 0.0;
        } else if (claimed.hardware_interface == "hardware_interface::EffortJointInterface") {
          joint_cmds_[resource].type = ScienceJointCmd::Type::EFF;
          joint_cmds_[resource].data = 0.0;
        }
      }
    }
  }
}

}  // namespace uwrt_mars_rover_hw