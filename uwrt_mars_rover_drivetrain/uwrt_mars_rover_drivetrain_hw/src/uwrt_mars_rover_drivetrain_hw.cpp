#include "uwrt_mars_rover_drivetrain_hw/uwrt_mars_rover_drivetrain_hw.hpp"

using hardware_interface::return_type;
using transmission_interface::ActuatorHandle;
using transmission_interface::JointHandle;
namespace uwrt_mars_rover_drivetrain_hw {

return_type UWRTMarsRoverDrivetrainHardware::configure(const hardware_interface::HardwareInfo& actuator_info) {
  bool configure_success{true};

  configure_success &= (configure_default(actuator_info) == return_type::OK);
  configure_success &= configureDrivetrainHardwareInfo(actuator_info);
  configure_success &= configureDrivetrainJoints(actuator_info.joints);
  configure_success &= configureDrivetrainTransmissions(actuator_info.transmissions);

  if (configure_success) {
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Hardware Configured ...");

    return return_type::OK;
  } else {
    RCLCPP_ERROR(rclcpp::get_logger(this->get_name()), "Hardware Configuration Failed ...");
    return return_type::ERROR;
  }
}

// load parameters if there are any
bool UWRTMarsRoverDrivetrainHardware::configureDrivetrainHardwareInfo(
    const hardware_interface::HardwareInfo& drivetrainHardwareInfo) {
  (void)drivetrainHardwareInfo;
  return true;
}

bool UWRTMarsRoverDrivetrainHardware::configureDrivetrainJoints(
    const std::vector<hardware_interface::ComponentInfo>& drivetrainJoints) {
  // resize command vector to the number of joints available in the urdf
  drivetrain_commands_velocities.resize(drivetrainJoints.size(), std::numeric_limits<double>::quiet_NaN());
  drivetrain_joint_position.resize(drivetrainJoints.size(), std::numeric_limits<double>::quiet_NaN());

  for (const hardware_interface::ComponentInfo& joint : drivetrainJoints) {
    // set joint names
    set_joint_name(joint.name);

    // check for number of command interfaces - there should only be one
    if (joint.command_interfaces.size() != NUM_COMMAND_INTERFACES) {
      RCLCPP_FATAL(rclcpp::get_logger(this->get_name()), "Incorrect number of command interfaces found");
      return false;
    }

    // check the command interface type - there should be one command interface
    if (joint.command_interfaces[command_interface::VELOCITY_COMMAND].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger(this->get_name()), "Velcoity Command Interface not found");
      return false;
    }

    // check for number of state interfaces - there should be two .... velocity and position
    if (joint.state_interfaces.size() != NUM_STATE_INTERFACES) {
      RCLCPP_FATAL(rclcpp::get_logger(this->get_name()), "Incorrect number of state interfaces found");
      return false;
    }

    // check for state interface type - there should be two state interfaces .... i.e. position & velocity
    if (joint.state_interfaces[state_interface::VELOCITY_STATE].name != hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(rclcpp::get_logger(this->get_name()), "Velocity State Interface not found");
      return false;
    }
    if (joint.state_interfaces[state_interface::POSITION_STATE].name != hardware_interface::HW_IF_POSITION) {
      RCLCPP_FATAL(rclcpp::get_logger(this->get_name()), "Position State Interface not found");
      return false;
    }
  }

  return true;
}

bool UWRTMarsRoverDrivetrainHardware::configureDrivetrainTransmissions(
    const std::vector<hardware_interface::ComponentInfo>& drivetrainTransmissions) {
  // there is only one transmission per joint .... dont really need a loop
  for (const hardware_interface::ComponentInfo& transmission_info : drivetrainTransmissions) {
    // get transmission reduction value from urdf
    double mechanicle_reduction{std::stod(transmission_info.parameters.at("mechanical_reduction"))};

    // instantiate transmission
    transmission = std::move(transmission_interface::SimpleTransmission(mechanicle_reduction));

    // create jointhandle
    positon_joint_handle =
        std::move(JointHandle(this->get_joint_name(), hardware_interface::HW_IF_POSITION, &CommandData::joint_position));
    velocity_joint_handle =
        std::move(JointHandle(this->get_joint_name(), hardware_interface::HW_IF_VELOCITY, &CommandData::joint_velocity));

    // create actuator handle
    velocity_actuator_handle = std::move(
        ActuatorHandle(this->get_name(), hardware_interface::HW_IF_VELOCITY, &CommandData::actuator_velocity));

    // configure transmission
    transmission.configure({positon_joint_handle, velocity_joint_handle}, {velocity_actuator_handle});
  }
  return true;
}

std::vector<hardware_interface::StateInterface> UWRTMarsRoverDrivetrainHardware::export_state_interfaces(){
    // create state interface to export
    std::vector<hardware_interface::StateInterface> state_interfaces_list;


    return state_interfaces_list;
}

std::vector<hardware_interface::CommandInterface> UWRTMarsRoverDrivetrainHardware::export_command_interfaces(){
   // create command interfaces to export
   std::vector<hardware_interface::CommandInterface> command_interfaces_list;

   return command_interfaces_list;
}

}  // namespace uwrt_mars_rover_drivetrain_hw

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(uwrt_mars_rover_drivetrain_hw::UWRTMarsRoverDrivetrainHardware,
                       hardware_interface::ActuatorInterface)