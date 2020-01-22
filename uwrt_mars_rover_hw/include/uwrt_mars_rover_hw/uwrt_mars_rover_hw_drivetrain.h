#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace uwrt_mars_rover_hw {

class UWRTRoverHWDrivetrain : public hardware_interface::RobotHW {
 public:
  explicit UWRTRoverHWDrivetrain() : UWRTRoverHWDrivetrain("uwrt_mars_rover_hw") {}

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  inline std::string getName() const { return name_; }

 protected:
  explicit UWRTRoverHWDrivetrain(const std::string& name) : name_(std::move(name)) {}
  const std::string name_;

  hardware_interface::JointStateInterface joint_state_interface_;

  // TODO: change to vector of hardware_interface::JointCommandInterface? Maybe have init decide what interfaces are
  // available

  // Joint Command Interfaces
  hardware_interface::PositionJointInterface joint_position_interface_;
  hardware_interface::VelocityJointInterface joint_velocity_interface_;
  hardware_interface::EffortJointInterface joint_effort_interface_;

  double command_[2];
  double position_[2];
  double velocity_[2];
  double effort_[2];
};

}  // namespace uwrt_mars_rover_hw
