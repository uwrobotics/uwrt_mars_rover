#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_science.h"

namespace uwrt_mars_rover_hw {

bool UWRTRoverHWScience::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  // TODO (wraftus) load in the joints

  // connect and register the state and command interfaces
  for (const auto& name : joint_names_) {
    ScienceJointState joint_state = joint_states_[name];
    ScienceJointCmd joint_cmd = joint_cmds_[name];

    // joint state interface
    hardware_interface::JointStateHandle state_handle(name, &joint_state.pos, &joint_state.vel, &joint_state.pos);
    joint_state_interface_.registerHandle(state_handle);

    // joint position interface
    hardware_interface::JointHandle pos_handle(joint_state_interface_.getHandle(name), &joint_cmd.data);
    joint_pos_interface_.registerHandle(pos_handle);

    // joint position interface
    hardware_interface::JointHandle vel_handle(joint_state_interface_.getHandle(name), &joint_cmd.data);
    joint_vel_interface_.registerHandle(vel_handle);

    // joint effort interface
    hardware_interface::JointHandle eff_handle(joint_state_interface_.getHandle(name), &joint_cmd.data);
    joint_eff_interface_.registerHandle(eff_handle);
  }

  return true;
}

void UWRTRoverHWScience::read(const ros::Time& time, const ros::Duration& period) {
  ROS_WARN_STREAM("Read called from base science_hw class, please only call from real or sim classes");
}

void UWRTRoverHWScience::write(const ros::Time& time, const ros::Duration& period) {
  ROS_WARN_STREAM("Write called from base science_hw class, please only call from real or sim classes");
}

void UWRTRoverHWScience::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                  const std::list<hardware_interface::ControllerInfo>& stop_list) {
  // clear commands for controllers to stop
  for (const auto& controller : stop_list) {
    for (const auto& claimed : controller.claimed_resources) {
      for (const auto& resource : claimed.resources) {
        joint_cmds_[resource].type = ScienceJointCmd::ScienceCmdType::NONE;
        joint_cmds_[resource].data = 0.0;
      }
    }
  }

  for (const auto& controller : start_list) {
    for (const auto& claimed : controller.claimed_resources) {
      for (const auto& resource : claimed.resources) {
        if (claimed.hardware_interface == "hardware_interface::PositionJointInterface") {
          joint_cmds_[resource].type = ScienceJointCmd::ScienceCmdType::POS;
          joint_cmds_[resource].data = joint_states_[resource].pos;
        } else if (claimed.hardware_interface == "hardware_interface::VelocityJointInterface") {
          joint_cmds_[resource].type = ScienceJointCmd::ScienceCmdType::VEL;
          joint_cmds_[resource].data = 0.0;
        } else if (claimed.hardware_interface == "hardware_interface::EffortJointInterface") {
          joint_cmds_[resource].type = ScienceJointCmd::ScienceCmdType::EFF;
          joint_cmds_[resource].data = 0.0;
        }
      }
    }
  }
}

}  // namespace uwrt_mars_rover_hw