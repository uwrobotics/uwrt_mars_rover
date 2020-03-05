#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_science.h"

namespace uwrt_mars_rover_hw {

bool UWRTRoverHWScience::init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle& robot_hw_nh) {
  // Get list of joints and indexers
  if (!robot_hw_nh.getParam("joints", joint_names_)) {
    ROS_ERROR_STREAM_NAMED(name_, "could not find science joints form paramater server");
    return false;
  }
  if (!robot_hw_nh.getParam("indexers", indexer_names_)) {
    ROS_ERROR_STREAM_NAMED(name_, "could not find science indexers from parameter server");
    return false;
  }

  // Connect and register the joint state and command handles
  for (const auto& name : joint_names_) {
    ROS_INFO_STREAM_NAMED(name_, "found science joint: " << name);

    // Joint state interface
    hardware_interface::JointStateHandle joint_state_handle(name, &joint_states_[name].pos,
                                                                  &joint_states_[name].vel,
                                                                  &joint_states_[name].eff);
    joint_state_interface_.registerHandle(joint_state_handle);

    // Joint command interfaces
    hardware_interface::JointHandle joint_command_handle(joint_state_interface_.getHandle(name),
                                                         &joint_cmds_[name].data);
    joint_pos_interface_.registerHandle(joint_command_handle);
  }

  // Connect and register the indexer state and command handles
  for (const auto& name : indexer_names_) {
    ROS_INFO_STREAM_NAMED(name_, "found science indexer: " << name);

    // Indexer state interface
    hardware_interface::IndexerStateHandle indexer_state_handle(name, &indexer_states_[name].raw_pos);
    indexer_state_interface_.registerHandle(indexer_state_handle);

    // Indexer command interfaces
    hardware_interface::IndexerCommandHandle indexer_cmd_handle(indexer_state_interface_.getHandle(name), &indexer_cmds_[name].pos);
    indexer_cmd_interface_.registerHandle(indexer_cmd_handle);
  }

  // Register interfaces
  registerInterface(&joint_state_interface_);
  registerInterface(&joint_pos_interface_);
  registerInterface(&indexer_state_interface_);
  registerInterface(&indexer_cmd_interface_);

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
      for (const auto& name : claimed.resources) {
        if (claimed.hardware_interface == "hardware_interface::JointCommandInterface") {
          joint_cmds_[name].type = ScienceJointCommand::Type::NONE;
          joint_cmds_[name].data = 0.0;
        } else if (claimed.hardware_interface == "hardware_interface::IndexerCommandInterface") {
          indexer_cmds_[name].type = ScienceIndexerCommand::Type::NONE;
          indexer_cmds_[name].pos = 0.0;
        }
      }
    }
  }

  for (const auto& controller : start_list) {
    for (const auto& claimed : controller.claimed_resources) {
      for (const auto& name : claimed.resources) {
        if (claimed.hardware_interface == "hardware_interface::PositionJointInterface") {
          joint_cmds_[name].type = ScienceJointCommand::Type::POS;
          joint_cmds_[name].data = 0.0;
        } else if (claimed.hardware_interface == "hardware_interface::IndexerCommandInterface") {
          indexer_cmds_[name].type = ScienceIndexerCommand::Type::POS;
          indexer_cmds_[name].pos = 0.0;
        }
      }
    }
  }
}

}  // namespace uwrt_mars_rover_hw