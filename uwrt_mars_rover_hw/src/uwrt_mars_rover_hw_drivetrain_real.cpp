#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_drivetrain_real.h"

#include "canopen_comm.hpp"

namespace uwrt_mars_rover_hw {

bool UWRTRoverHWDrivetrainReal::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  if (!UWRTRoverHWDrivetrain::init(root_nh, robot_hw_nh)) {
    return false;
  }

  // TODO: controller to switch the roboteq communication mode
  std::unique_ptr<roboteq::i_comm> comm = std::make_unique<roboteq::canopen_comm>(0x01, "can0");
  motor_controller_ = std::make_unique<roboteq::roboteq_controller>(std::move(comm));
  return true;
}

void UWRTRoverHWDrivetrainReal::read(const ros::Time& time, const ros::Duration& period) {
  //  for (const auto& joint_name : joint_names_) {
  //    // TODO: change to use tpdos
  //    joint_states_[joint_name].position =
  //    motor_controller_->ReadAbsoluteEncoderCount(joint_roboteq_index_[joint_name]);
  //    joint_states_[joint_name].velocity = motor_controller_->ReadEncoderMotorSpeed(joint_roboteq_index_[joint_name]);
  //    joint_states_[joint_name].effort = motor_controller_->ReadMotorAmps(joint_roboteq_index_[joint_name]) / 10.0;
  //  }

  // TODO: remove
  for (const auto& joint_name : joint_names_) {
    ROS_ERROR_STREAM_NAMED(name_, std::left << "Joint: " << std::setw(20) << joint_name << "Position: " << std::setw(20)
                                            << joint_states_[joint_name].position << "Velocity: " << std::setw(20)
                                            << joint_states_[joint_name].velocity << "EFfort: " << std::setw(20)
                                            << joint_states_[joint_name].effort);
  }
}

void UWRTRoverHWDrivetrainReal::write(const ros::Time& time, const ros::Duration& period) {
  //  for (const auto& joint_name : joint_names_) {
  //    bool successful_joint_write = false;
  //    switch (joint_commands_[joint_name].type) {
  //      case UWRTRoverHWDrivetrain::DrivetrainJointCommand::Type::NONE:
  //        ROS_WARN_STREAM_NAMED(name_, joint_name << " has a " << joint_commands_[joint_name].type << " command
  //        type."); break;
  //
  //      case UWRTRoverHWDrivetrain::DrivetrainJointCommand::Type::POSITION:
  //        successful_joint_write =
  //            motor_controller_->SetPosition(joint_commands_[joint_name].data, joint_roboteq_index_[joint_name]);
  //        break;
  //
  //      case UWRTRoverHWDrivetrain::DrivetrainJointCommand::Type::VELOCITY:
  //        successful_joint_write =
  //            motor_controller_->SetVelocity(joint_commands_[joint_name].data, joint_roboteq_index_[joint_name]);
  //        break;
  //
  //      case UWRTRoverHWDrivetrain::DrivetrainJointCommand::Type::EFFORT:
  //        // TODO: look into roboteq torque control
  //        break;
  //
  //      default:
  //        ROS_ERROR_STREAM_NAMED(name_, "Could not write to joints. "
  //                                          << static_cast<int>(joint_commands_[joint_name].type)
  //                                          << " is an unknown command type.");
  //    }
  //    if (!successful_joint_write) {
  //      ROS_ERROR_STREAM_NAMED(
  //          name_, "Failed to write " << joint_commands_[joint_name].type << " command to " << joint_name << ".");
  //    }
  //  }

  // TODO: remove
  for (const auto& joint_name : joint_names_) {
    ROS_DEBUG_STREAM_NAMED(name_, std::left << "Joint: " << std::setw(20) << joint_name
                                            << "Command Type: " << std::setw(20) << joint_commands_[joint_name].type
                                            << "Command Data: " << std::setw(20) << joint_commands_[joint_name].data);
  }
}
}  // namespace uwrt_mars_rover_hw
