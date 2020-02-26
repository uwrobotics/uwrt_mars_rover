#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_science.h"

namespace uwrt_mars_rover_hw {

bool UWRTRoverHWScience::init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle& robot_hw_nh) {
  if (!robot_hw_nh.getParam("indexer_joints", indexer_names_)) {
    ROS_ERROR_STREAM_NAMED(name_, "could not find science joints from parameter server");
    return false;
  }

  // connect and register the state and command indexer interfaces
  for (const auto& name : indexer_names_) {
    ROS_INFO_STREAM_NAMED(name_, "found science indexer joint: " << name);

    // joint state interface
    hardware_interface::IndexerStateHandle indexer_state_handle(name, &indexer_states_[name]);
    indexer_state_interface_.registerHandle(indexer_state_handle);

    // joint command interfaces
    hardware_interface::IndexerCommandHandle indexer_cmd_handle(indexer_state_interface_.getHandle(name), &indexer_cmds_[name]);
    indexer_cmd_interface_.registerHandle(indexer_cmd_handle);
  }

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
      for (const auto& resource : claimed.resources) {
        if (claimed.hardware_interface == "hardware_interface::IndexerCommandInterface") {

        }
      }
    }
  }

  for (const auto& controller : start_list) {
    for (const auto& claimed : controller.claimed_resources) {
      for (const auto& resource : claimed.resources) {
        if (claimed.hardware_interface == "hardware_interface::IndexerCommandInterface") {
          
        }
      }
    }
  }
}

}  // namespace uwrt_mars_rover_hw