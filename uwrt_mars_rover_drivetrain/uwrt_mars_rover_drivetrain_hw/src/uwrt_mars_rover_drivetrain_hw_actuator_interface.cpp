#include "uwrt_mars_rover_drivetrain_hw/uwrt_mars_rover_drivetrain_hw_actuator_interface.hpp"

namespace uwrt_mars_rover_drivetrain_hw {
UWRTMarsRoverDrivetrainHWActuatorInterface::UWRTMarsRoverDrivetrainHWActuatorInterface()
    : logger_(rclcpp::get_logger("UWRTMarsRoverDrivetrainHWActuatorInterface")) {}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_init(
    const hardware_interface::HardwareInfo& actuator_info) {
  logger_ = rclcpp::get_logger(actuator_info.name);

  RCLCPP_INFO(logger_, "Drivetrain Actuator Initializing...");

  if (hardware_interface::ActuatorInterface::on_init(actuator_info) != LifecyleNodeCallbackReturn::SUCCESS) {
    return LifecyleNodeCallbackReturn::ERROR;
  }

  // Validate that parsed urdf information matches expected structure
  if (info_.joints.size() != NUM_JOINTS) {
    RCLCPP_FATAL_STREAM(logger_, "'" << info_.name.c_str() << "' has " << info_.joints.size() << " joints. "
                                     << NUM_JOINTS << " expected.");
    return LifecyleNodeCallbackReturn::ERROR;
  }
  const hardware_interface::ComponentInfo& joint = info_.joints[0];
  if (joint.state_interfaces.size() != NUM_STATE_INTERFACES) {
    RCLCPP_FATAL_STREAM(logger_, "Joint '" << joint.name.c_str() << "' has " << joint.state_interfaces.size()
                                           << " state interfaces. " << NUM_STATE_INTERFACES << " expected.");
    return LifecyleNodeCallbackReturn::ERROR;
  }
  for (const hardware_interface::InterfaceInfo& state_interface : joint.state_interfaces) {
    if (!(state_interface.name == hardware_interface::HW_IF_POSITION ||
          state_interface.name == hardware_interface::HW_IF_VELOCITY ||
          state_interface.name == "iq_current")) {  // TODO: add shared HW_IF_IQ_CURRENT in some shared package
      RCLCPP_FATAL_STREAM(logger_, "Joint '" << joint.name.c_str() << "' has " << state_interface.name.c_str()
                                             << " state interfaces. '" << hardware_interface::HW_IF_POSITION << "' or '"
                                             << hardware_interface::HW_IF_VELOCITY << "' or 'iq_current' expected.");
      return LifecyleNodeCallbackReturn::ERROR;
    }
  }
  if (joint.command_interfaces.size() != NUM_COMMAND_INTERFACES) {
    RCLCPP_FATAL_STREAM(logger_, "Joint '" << joint.name.c_str() << "' has " << joint.command_interfaces.size()
                                           << " command interfaces. " << NUM_COMMAND_INTERFACES << " expected.");
    return LifecyleNodeCallbackReturn::ERROR;
  }
  if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY) {
    RCLCPP_FATAL_STREAM(logger_, "Joint '" << joint.name.c_str() << "' has " << joint.command_interfaces[0].name.c_str()
                                           << " command interface. '" << hardware_interface::HW_IF_VELOCITY
                                           << "' expected.");
    return LifecyleNodeCallbackReturn::ERROR;
  }

  // Init states and commands to nan
  actuator_state_position_ = std::numeric_limits<double>::quiet_NaN();
  actuator_state_velocity_ = std::numeric_limits<double>::quiet_NaN();
  actuator_state_iq_current_ = std::numeric_limits<double>::quiet_NaN();
  joint_velocity_command_ = std::numeric_limits<double>::quiet_NaN();


  
  drivetrain_can_wrapper_ = uwrt_mars_rover_utilities::UWRTCANWrapper("drivetrain_can","vcan0", false); 
  

  RCLCPP_DEBUG(logger_, "Drivetrain Actuator Initialized Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Drivetrain Actuator Cleaning Up...");

  // Reset state and command data to nan
  actuator_state_position_ = std::numeric_limits<double>::quiet_NaN();
  actuator_state_velocity_ = std::numeric_limits<double>::quiet_NaN();
  actuator_state_iq_current_ = std::numeric_limits<double>::quiet_NaN();
  joint_velocity_command_ = std::numeric_limits<double>::quiet_NaN();
  

  RCLCPP_INFO(logger_, "Drivetrain Actuator Cleaned Up Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Drivetrain Actuator Configuring ...");

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

  std::vector<uint32_t> addresses {actuator_encoder_address_, actuator_state_iq_current_address_};
  drivetrain_can_wrapper_.init(addresses);


  RCLCPP_INFO(logger_, "Drivetrain Actuator Configured Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Drivetrain Actuator Deactivating...");

  joint_velocity_command_ = 0.0;

  // TODO: enable can library to start receiving data for state interfaces and non-movement command interfaces. consider
  // existing state data and non-movement command data as stale

  std::vector<uint32_t> addresses {actuator_encoder_address_, actuator_state_iq_current_address_};
  drivetrain_can_wrapper_.init(addresses);

  RCLCPP_INFO(logger_, "Drivetrain Actuator Deactivated Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Drivetrain Actuator Shutting Down...");

  // TODO: Null out CAN lib object?

  RCLCPP_INFO(logger_, "Drivetrain Actuator Shut Down Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Drivetrain Actuator Activating...");

  RCLCPP_INFO(logger_, "Drivetrain Actuator Activated Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

struct TwoFloats {
  float a, b;
};

hardware_interface::return_type UWRTMarsRoverDrivetrainHWActuatorInterface::read() {
  RCLCPP_DEBUG(logger_, "Drivetrain Actuator Reading...");

  TwoFloats encoderEstimates;
  drivetrain_can_wrapper_.getLatestFromID<TwoFloats>(encoderEstimates, actuator_encoder_address_);
  
  // Sketchy, not tested yet, copy this into two floats
  // std::memcpy(&actuator_state_position, &encoderEstimates, sizeof(actuator_state_position)); 
  // std::memcpy(&actuator_state_velocity, &encoderEstimates[4], sizeof(actuator_state_velocity)); 
  actuator_state_position_ = (double) encoderEstimates.a;
  actuator_state_velocity_ = (double) encoderEstimates.b;
  
  TwoFloats iq_current;
  drivetrain_can_wrapper_.getLatestFromID<TwoFloats>(iq_current, actuator_state_iq_current_address_);
  // TODO (by Colin) I think we want the last 4 bytes (measured IQ) on the above line, that should be read into float
  // std::memcpy(&actuator_state_iq_current, &iq_current[4], sizeof(actuator_state_iq_current));
  actuator_state_iq_current_ = (double) iq_current.b;
  
  RCLCPP_DEBUG_STREAM(logger_, "Actuator Position: " << actuator_state_position_
                                                     << " Actuator Velocity: " << actuator_state_velocity_
                                                     << " Actuator IQ Current: " << actuator_state_iq_current_);


  RCLCPP_DEBUG(logger_, "Drivetrain Actuator Read Successfully...");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UWRTMarsRoverDrivetrainHWActuatorInterface::write() {
  RCLCPP_DEBUG(logger_, "Drivetrain Actuator Writing...");

  RCLCPP_DEBUG_STREAM(logger_, "Joint Velocity: " << joint_velocity_command_);

  // Write to float because velocity should be 4 bytes
  drivetrain_can_wrapper_.writeToID<float>((float) joint_velocity_command_, write_address_);
  

  RCLCPP_DEBUG(logger_, "Drivetrain Actuator Written Successfully...");
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
