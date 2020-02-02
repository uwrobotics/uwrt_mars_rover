#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_drivetrain.h"

namespace uwrt_mars_rover_hw {

UWRTRoverHWDrivetrain::UWRTRoverHWDrivetrain() : UWRTRoverHWDrivetrain("UWRTRoverHWDrivetrain") {}

bool UWRTRoverHWDrivetrain::init(ros::NodeHandle& /*root_nh*/, ros::NodeHandle& robot_hw_nh) {
  bool param_fetched = robot_hw_nh.getParam("joints", joint_names_);
  if (!param_fetched) {
    ROS_WARN_STREAM_NAMED(name_, robot_hw_nh.getNamespace()
                                     << "/joints could not be found and loaded from parameter server.");
  } else {
    ROS_DEBUG_STREAM_NAMED(name_, robot_hw_nh.getNamespace() << "/joints loaded from parameter server.");
  }

  for (const auto& joint_name : joint_names_) {
    hardware_interface::JointStateHandle joint_state_handle(joint_name, &joint_states[joint_name].position,
                                                            &joint_states[joint_name].velocity,
                                                            &joint_states[joint_name].effort);
    joint_state_interface_.registerHandle(joint_state_handle);

    hardware_interface::JointHandle joint_command_handle(joint_state_interface_.getHandle(joint_name),
                                                         &joint_commands_[joint_name].data);
    joint_position_interface_.registerHandle(joint_command_handle);
    joint_velocity_interface_.registerHandle(joint_command_handle);
    joint_effort_interface_.registerHandle(joint_command_handle);
  }

  registerInterface(&joint_state_interface_);
  registerInterface(&joint_position_interface_);
  registerInterface(&joint_velocity_interface_);
  registerInterface(&joint_effort_interface_);

  return true;
}

void UWRTRoverHWDrivetrain::read(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  ROS_ERROR_STREAM_NAMED(name_, "read function called from "
                                    << name_ << " class. read calls should only happen to the overloaded function.");
}
void UWRTRoverHWDrivetrain::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  ROS_ERROR_STREAM_NAMED(name_, "write function called from "
                                    << name_ << " class. write calls should only happen to the overloaded function.");
}

void UWRTRoverHWDrivetrain::doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                                     const std::list<hardware_interface::ControllerInfo>& stop_list) {
  // Reset commands for joints claimed by stopping controllers
  for (const auto& controller : stop_list) {
    for (const auto& hardware_interface_resource_list : controller.claimed_resources) {
      for (const auto& joint_name : hardware_interface_resource_list.resources) {
        joint_commands_[joint_name].type = DrivetrainJointCommand::Type::NONE;
        joint_commands_[joint_name].data = 0.0;
      }
    }
  }

  // Set command type for joints claimed by starting controllers
  for (const auto& controller : start_list) {
    for (const auto& claimed : controller.claimed_resources) {
      for (const auto& joint_name : claimed.resources) {
        if (claimed.hardware_interface == "hardware_interface::PositionJointInterface") {
          joint_commands_[joint_name].type = DrivetrainJointCommand::Type::POSITION;
          joint_commands_[joint_name].data = joint_states[joint_name].position;
        } else if (claimed.hardware_interface == "hardware_interface::VelocityJointInterface") {
          joint_commands_[joint_name].type = DrivetrainJointCommand::Type::VELOCITY;
          joint_commands_[joint_name].data = 0.0;
        } else if (claimed.hardware_interface == "hardware_interface::EffortJointInterface") {
          joint_commands_[joint_name].type = DrivetrainJointCommand::Type::EFFORT;
          joint_commands_[joint_name].data = 0.0;
        }  // TODO add an open loop interface (voltage?)
      }
    }
  }
}

}  // namespace uwrt_mars_rover_hw
