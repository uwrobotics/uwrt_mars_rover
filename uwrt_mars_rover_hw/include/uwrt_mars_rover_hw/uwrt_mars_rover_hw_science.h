#pragma once

#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <ros/ros.h>

namespace uwrt_mars_rover_hw {
// A generic template for interfacing with the science firmware (real or simulated)
class UWRTRoverHWScience : public hardware_interface::RobotHW {
 public:
  explicit UWRTRoverHWScience() : UWRTRoverHWScience("uwrt_mars_rover_hw_science") {}

  struct ScienceJointState {
    double pos;
    double vel;
    double eff;
  };

  struct ScienceJointCmd {
    enum class ScienceCmdType { NONE, POS, VEL, EFF };

    double data;
    Type type;
  };

  // Overrides
  virtual bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  virtual void read(const ros::Time& time, const ros::Duration& period) override;
  virtual void write(const ros::Time& time, const ros::Duration& period) override;
  virtual void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                        const std::list<hardware_interface::ControllerInfo>& stop_list) override;

  // Get the name of the controller
  inline std::string getName() const {
    return name_;
  }

 protected:
  explicit UWRTRoverHWScience(const std::string& name) : name_(std::move(name)) {}

  // Short name for this class
  const std::string name_;

  // Hardware state interfaces
  hardware_interface::JointStateInterface joint_state_interface_;

  // Hardware command interfaces
  hardware_interface::PositionJointInterface joint_pos_interface_;
  hardware_interface::VelocityJointInterface joint_vel_interface_;
  hardware_interface::EffortJointInterface joint_eff_interface_;

  // Joint names
  std::vector<std::string> joint_names_;

  // States
  std::map<std::string, ScienceJointState> joint_states_;

  // Commands
  std::map<std::string, ScienceJointCmd> joint_cmds_;
};
}  // namespace uwrt_mars_rover_hw
