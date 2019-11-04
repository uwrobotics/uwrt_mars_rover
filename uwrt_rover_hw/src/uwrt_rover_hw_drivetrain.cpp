#include "uwrt_rover_hw_drivetrain.h"

namespace uwrt_rover_hw {

bool UWRTRoverHWDrivetrain::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  // connect and register the joint state interface
  hardware_interface::JointStateHandle left_drive_joint("LeftDriveJoint", &position_[0], &velocity_[0], &effort_[0]);
  joint_state_interface.registerHandle(left_drive_joint);
  hardware_interface::JointStateHandle right_drive_joint("RightDriveJoint", &position_[1], &velocity_[1], &effort_[1]);
  joint_state_interface.registerHandle(right_drive_joint);
  registerInterface(&joint_state_interface);

  // connect and register the joint position_ interface
  hardware_interface::JointHandle left_drive_position__handle(joint_state_interface.getHandle("LeftDriveJoint"),
                                                             &command_[0]);
  joint_position_interface_.registerHandle(left_drive_position__handle);
  hardware_interface::JointHandle right_drive_position__handle(joint_state_interface.getHandle("RightDriveJoint"),
                                                              &command_[1]);
  joint_position_interface_.registerHandle(right_drive_position__handle);
  registerInterface(&joint_position_interface_);

  // connect and register the joint velocity_ interface
  hardware_interface::JointHandle left_drive_velocity__handle(joint_state_interface.getHandle("LeftDriveJoint"),
                                                             &command_[0]);
  joint_velocity_interface_.registerHandle(left_drive_velocity__handle);
  hardware_interface::JointHandle right_drive_velocity__handle(joint_state_interface.getHandle("RightDriveJoint"),
                                                              &command_[1]);
  joint_velocity_interface_.registerHandle(right_drive_velocity__handle);
  registerInterface(&joint_velocity_interface_);

  // connect and register the joint effort_ interface
  hardware_interface::JointHandle left_drive_effort__handle(joint_state_interface.getHandle("LeftDriveJoint"),
                                                           &command_[0]);
  joint_effort_interface_.registerHandle(left_drive_effort__handle);
  hardware_interface::JointHandle right_drive_effort__handle(joint_state_interface.getHandle("RightDriveJoint"),
                                                            &command_[1]);
  joint_effort_interface_.registerHandle(right_drive_effort__handle);
  registerInterface(&joint_effort_interface_);

  return true;
}

}  // namespace uwrt_rover_hw
