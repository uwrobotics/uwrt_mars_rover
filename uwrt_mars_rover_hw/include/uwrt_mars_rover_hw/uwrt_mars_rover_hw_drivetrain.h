#pragma once

#include <hardware_interface/controller_info.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>

namespace uwrt_mars_rover_hw {

class UWRTRoverHWDrivetrain : public hardware_interface::RobotHW {
 public:
  explicit UWRTRoverHWDrivetrain() : UWRTRoverHWDrivetrain("UWRTRoverHWDrivetrain"){};

  struct DrivetrainJointState {
    double position;
    double velocity;
    double effort;
  };

  struct DrivetrainJointCommand {
    enum class Type { NONE, POSITION, VELOCITY, EFFORT };
    Type type;
    double data;
  };

  bool init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) override;
  void read(const ros::Time& time, const ros::Duration& period) override;
  void write(const ros::Time& time, const ros::Duration& period) override;
  void doSwitch(const std::list<hardware_interface::ControllerInfo>& start_list,
                const std::list<hardware_interface::ControllerInfo>& stop_list) override;

  inline std::string getName() const {
    return name_;
  }

 protected:
  explicit UWRTRoverHWDrivetrain(std::string name) : name_(std::move(name)) {}
  const std::string name_;

  std::vector<std::string> joint_names_;

  // Joint State Interface
  hardware_interface::JointStateInterface joint_state_interface_;

  // Joint Command Interfaces
  hardware_interface::PositionJointInterface joint_position_interface_;
  hardware_interface::VelocityJointInterface joint_velocity_interface_;
  hardware_interface::EffortJointInterface joint_effort_interface_;

  // Joint states and commands associated with joint names
  std::map<std::string, DrivetrainJointState> joint_states;
  std::map<std::string, DrivetrainJointCommand> joint_commands_;
};

inline std::ostream& operator<<(std::ostream& os, UWRTRoverHWDrivetrain::DrivetrainJointCommand::Type& command_type) {
  switch (command_type) {
    case UWRTRoverHWDrivetrain::DrivetrainJointCommand::Type::NONE:
      os << "None";
      break;
    case UWRTRoverHWDrivetrain::DrivetrainJointCommand::Type::POSITION:
      os << "Position";
      break;
    case UWRTRoverHWDrivetrain::DrivetrainJointCommand::Type::VELOCITY:
      os << "Velocity";
      break;
    case UWRTRoverHWDrivetrain::DrivetrainJointCommand::Type::EFFORT:
      os << "Effort";
      break;
    default:
      os.setstate(std::ios_base::failbit);
  }
  return os;
}

}  // namespace uwrt_mars_rover_hw
