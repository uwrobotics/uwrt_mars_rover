#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace uwrt_rover_hw {

class UWRTRoverHWDrivetrain : public hardware_interface::RobotHW {
 public:
  explicit UWRTRoverHWDrivetrain() : UWRTRoverHWDrivetrain("uwrt_rover_hw"){}

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;

 protected:
  explicit UWRTRoverHWDrivetrain(const std::string& name) : name_(std::move(name)) {}
  const std::string name_;

 private:
  hardware_interface::JointStateInterface joint_state_interface;

  // TODO: change to vector of hardware_interface::JointCommandInterface? Maybe have init decide what interfaces are
  // available
  hardware_interface::PositionJointInterface joint_position_interface_;
  hardware_interface::VelocityJointInterface joint_velocity_interface_;
  hardware_interface::EffortJointInterface joint_effort_interface_;

  double command_[2];
  double position_[2];
  double velocity_[2];
  double effort_[2];
};

}  // namespace uwrt_rover_hw