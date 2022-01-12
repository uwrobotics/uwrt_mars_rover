#include "uwrt_mars_rover_drivetrain_hw/uwrt_mars_rover_drivetrain_hw_actuator_interface.hpp"

namespace uwrt_mars_rover_drivetrain_hw {
LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_init(
    const hardware_interface::HardwareInfo& actuator_info) {
  RCLCPP_DEBUG(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"), "Drivetrain Actuator Initializing...");

  if (hardware_interface::ActuatorInterface::on_init(actuator_info) != LifecyleNodeCallbackReturn::SUCCESS) {
    return LifecyleNodeCallbackReturn::ERROR;
  }

  // Validate that parsed urdf information matches expected structure
  if (info_.joints.size() != NUM_JOINTS) {
    RCLCPP_FATAL_STREAM(
        rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"),
        "'" << info_.name.c_str() << "' has " << info_.joints.size() << " joints. " << NUM_JOINTS << " expected.");
    return LifecyleNodeCallbackReturn::ERROR;
  }
  const hardware_interface::ComponentInfo& joint = info_.joints[0];
  if (joint.state_interfaces.size() != NUM_STATE_INTERFACES) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"),
                        "Joint '" << joint.name.c_str() << "' has " << joint.state_interfaces.size()
                                  << " state interfaces. " << NUM_STATE_INTERFACES << " expected.");
    return LifecyleNodeCallbackReturn::ERROR;
  }
  for (const hardware_interface::InterfaceInfo& state_interface : joint.state_interfaces) {
    if (!(state_interface.name == hardware_interface::HW_IF_POSITION ||
          state_interface.name == hardware_interface::HW_IF_VELOCITY ||
          state_interface.name == "iq_current")) {  // TODO: add shared HW_IF_IQ_CURRENT in some shared package
      RCLCPP_FATAL_STREAM(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"),
                          "Joint '" << joint.name.c_str() << "' has " << state_interface.name.c_str()
                                    << " state interfaces. '" << hardware_interface::HW_IF_POSITION << "' or '"
                                    << hardware_interface::HW_IF_VELOCITY << "' or 'iq_current' expected.");
      return LifecyleNodeCallbackReturn::ERROR;
    }
  }
  if (joint.command_interfaces.size() != NUM_COMMAND_INTERFACES) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"),
                        "Joint '" << joint.name.c_str() << "' has " << joint.command_interfaces.size()
                                  << " command interfaces. " << NUM_COMMAND_INTERFACES << " expected.");
    return LifecyleNodeCallbackReturn::ERROR;
  }
  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
    RCLCPP_FATAL_STREAM(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"),
                        "Joint '" << joint.name.c_str() << "' has " << joint.command_interfaces[0].name.c_str()
                                  << " command interface. '" << hardware_interface::HW_IF_VELOCITY << "' expected.");
    return LifecyleNodeCallbackReturn::ERROR;
  }

  // Init states and commands to nan
  actuator_state_position_ = std::numeric_limits<double>::quiet_NaN();
  actuator_state_velocity_ = std::numeric_limits<double>::quiet_NaN();
  actuator_state_iq_current_ = std::numeric_limits<double>::quiet_NaN();
  joint_velocity_command_ = std::numeric_limits<double>::quiet_NaN();

  // TODO: init transmissions
  // TODO: init can library

  RCLCPP_DEBUG(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"),
               "Drivetrain Actuator Initialized Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_cleanup(
    const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"), "Drivetrain Actuator Cleaning Up...");

  // Reset state and command data to nan
  actuator_state_position_ = std::numeric_limits<double>::quiet_NaN();
  actuator_state_velocity_ = std::numeric_limits<double>::quiet_NaN();
  actuator_state_iq_current_ = std::numeric_limits<double>::quiet_NaN();
  joint_velocity_command_ = std::numeric_limits<double>::quiet_NaN();

  // TODO: re-init or reset can library to init state
  RCLCPP_INFO(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"),
              "Drivetrain Actuator Cleaned Up Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_configure(
    const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"), "Drivetrain Actuator Configuring ...");

  joint_velocity_command_ = 0.0;

  // Prevent diff_drive_controller from using nan values
  if (std::isnan(actuator_state_position_)) {
    actuator_state_position_ = 0.0;
  }
  if (std::isnan(actuator_state_velocity_)) {
    actuator_state_velocity_ = 0.0;
  }
  if (std::isnan(actuator_state_iq_current_)) {
    actuator_state_iq_current_ = 0.0;
  }

  // TODO: enable can library to start receiving data for state interfaces and non-movement command interfaces. consider
  // existing state data and non-movement command data as stale

  RCLCPP_INFO(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"),
              "Drivetrain Actuator Configured Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_deactivate(
    const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"), "Drivetrain Actuator Deactivating...");

  joint_velocity_command_ = 0.0;

  // TODO: enable can library to start receiving data for state interfaces and non-movement command interfaces. consider
  // existing state data and non-movement command data as stale

  RCLCPP_INFO(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"),
              "Drivetrain Actuator Deactivated Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_shutdown(
    const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"), "Drivetrain Actuator Shutting Down...");

  // TODO: Null out CAN lib object?

  RCLCPP_INFO(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"),
              "Drivetrain Actuator Shut Down Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_activate(
    const rclcpp_lifecycle::State& previous_state) {
  RCLCPP_INFO(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"), "Drivetrain Actuator Activating...");

  RCLCPP_INFO(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"),
              "Drivetrain Actuator Activated Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

hardware_interface::return_type UWRTMarsRoverDrivetrainHWActuatorInterface::read() {
  RCLCPP_DEBUG(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"), "Drivetrain Actuator Reading...");

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"),
                      "Actuator Position: " << actuator_state_position_
                                            << " Actuator Velocity: " << actuator_state_velocity_
                                            << " Actuator IQ Current: " << actuator_state_iq_current_);
  // TODO: read from CAN

  RCLCPP_DEBUG(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"),
               "Drivetrain Actuator Read Successfully...");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UWRTMarsRoverDrivetrainHWActuatorInterface::write() {
  RCLCPP_DEBUG(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"), "Drivetrain Actuator Writing...");

  RCLCPP_DEBUG_STREAM(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"),
                      "Joint Velocity: " << joint_velocity_command_);
  // TODO: write to CAN

  RCLCPP_DEBUG(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface"),
               "Drivetrain Actuator Written Successfully...");
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> UWRTMarsRoverDrivetrainHWActuatorInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces_list;
  state_interfaces_list.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &actuator_state_position_));
  state_interfaces_list.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &actuator_state_velocity_));
  state_interfaces_list.emplace_back(
      hardware_interface::StateInterface(info_.joints[0].name, "iq_current", &actuator_state_iq_current_));
  return state_interfaces_list;
}

std::vector<hardware_interface::CommandInterface>
UWRTMarsRoverDrivetrainHWActuatorInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces_list;
  command_interfaces_list.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &joint_velocity_command_));
  return command_interfaces_list;
}

}  // namespace uwrt_mars_rover_drivetrain_hw

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(uwrt_mars_rover_drivetrain_hw::UWRTMarsRoverDrivetrainHWActuatorInterface,
                       hardware_interface::ActuatorInterface)
