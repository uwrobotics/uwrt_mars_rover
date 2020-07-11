#pragma once

#include <hardware_interface/controller_info.h>
#include <hardware_interface/joint_command_interface.h>
#include <hardware_interface/joint_state_interface.h>
#include <hardware_interface/robot_hw.h>
#include <transmission_interface/simple_transmission.h>
#include <transmission_interface/transmission_interface.h>

namespace uwrt_mars_rover_hw {

class UWRTRoverHWDrivetrain : public hardware_interface::RobotHW {
 public:
  explicit UWRTRoverHWDrivetrain() : UWRTRoverHWDrivetrain("UWRTRoverHWDrivetrain"){};

  struct DrivetrainActuatorJointState {
    double joint_position;
    double joint_velocity;
    double joint_effort;

    double actuator_position;
    double actuator_velocity;
    double actuator_effort;
  };

  struct DrivetrainActuatorJointCommand {
    enum class Type { NONE, POSITION, VELOCITY };
    Type type;
    double actuator_data;
    double joint_data;
  };

  bool init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) override;

  void read(const ros::Time &time, const ros::Duration &period) override;

  void write(const ros::Time &time, const ros::Duration &period) override;

  void doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                const std::list<hardware_interface::ControllerInfo> &stop_list) override;

  inline std::string getName() const {
    return name_;
  }

 protected:
  explicit UWRTRoverHWDrivetrain(std::string name) : name_(std::move(name)) {}

  const std::string name_;

  // Joint States and Commands mapped to Joint Names
  std::vector<std::string> joint_names_;
  std::map<std::string, DrivetrainActuatorJointState> actuator_joint_states_;
  std::map<std::string, DrivetrainActuatorJointCommand> actuator_joint_commands_;

  // Joint Transmissions and Actuator/Joint Data Wrappers
  std::map<std::string, transmission_interface::SimpleTransmission> joint_transmissions_;
  std::map<std::string, transmission_interface::ActuatorData> actuator_state_data_;
  std::map<std::string, transmission_interface::ActuatorData> actuator_command_data_;
  std::map<std::string, transmission_interface::JointData> joint_state_data_;
  std::map<std::string, transmission_interface::JointData> joint_command_data_;

  // State Interfaces
  hardware_interface::JointStateInterface joint_state_interface_;
  transmission_interface::ActuatorToJointStateInterface actuator_to_joint_state_interface_;

  // Joint Command Interfaces
  hardware_interface::PositionJointInterface joint_position_interface_;
  hardware_interface::VelocityJointInterface joint_velocity_interface_;
  hardware_interface::EffortJointInterface joint_effort_interface_;

  // Joint Command Transmission Interfaces
  transmission_interface::JointToActuatorPositionInterface joint_to_actuator_position_interface_;
  transmission_interface::JointToActuatorVelocityInterface joint_to_actuator_velocity_interface_;
  transmission_interface::JointToActuatorEffortInterface joint_to_actuator_effort_interface_;

 private:
  bool loadJointInfoFromParameterServer(ros::NodeHandle &robot_hw_nh);
  void registerStateInterfacesAndTransmissions(const std::string &joint_name);
  void registerCommandInterfacesAndTransmissions(const std::string &joint_name);
};

inline std::ostream &operator<<(std::ostream &os,
                                UWRTRoverHWDrivetrain::DrivetrainActuatorJointCommand::Type &command_type) {
  switch (command_type) {
    case UWRTRoverHWDrivetrain::DrivetrainActuatorJointCommand::Type::NONE:
      os << "None";
      break;

    case UWRTRoverHWDrivetrain::DrivetrainActuatorJointCommand::Type::POSITION:
      os << "Position";
      break;

    case UWRTRoverHWDrivetrain::DrivetrainActuatorJointCommand::Type::VELOCITY:
      os << "Velocity";
      break;

    default:
      os.setstate(std::ios_base::failbit);
  }
  return os;
}

}  // namespace uwrt_mars_rover_hw
