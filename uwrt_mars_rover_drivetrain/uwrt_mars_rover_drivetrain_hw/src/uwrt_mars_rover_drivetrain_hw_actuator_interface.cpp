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
  // actuator_state_iq_current_ = std::numeric_limits<double>::quiet_NaN();
  motor_velocity_ = std::numeric_limits<double>::quiet_NaN();
  
  // TODO (npalmar): ensure there are no problems with 6 can wrappers that have the same name
  drivetrain_can_wrapper_ = uwrt_mars_rover_utilities::UWRTCANWrapper("drivetrain_can", can_interface_, false); 
  
  RCLCPP_DEBUG(logger_, "Drivetrain Actuator Initialized Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_cleanup(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Drivetrain Actuator Cleaning Up...");

  // Reset state and command data to nan
  actuator_state_position_ = std::numeric_limits<double>::quiet_NaN();
  actuator_state_velocity_ = std::numeric_limits<double>::quiet_NaN();
  // actuator_state_iq_current_ = std::numeric_limits<double>::quiet_NaN();
  motor_velocity_ = std::numeric_limits<double>::quiet_NaN();

  RCLCPP_INFO(logger_, "Drivetrain Actuator Cleaned Up Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_configure(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Drivetrain Actuator Configuring ...");

  motor_velocity_ = 0.0;

  // Prevent diff_drive_controller from using nan values
  if (std::isnan(actuator_state_position_)) {
    actuator_state_position_ = 0.0;
  }
  if (std::isnan(actuator_state_velocity_)) {
    actuator_state_velocity_ = 0.0;
  }
  // if (std::isnan(actuator_state_iq_current_)) {
  //   actuator_state_iq_current_ = 0.0;
  // }

  // TODO: enable can library to start receiving data for state interfaces and non-movement command interfaces. consider
  // existing state data and non-movement command data as stale

  // get the can_id parameter
  can_id_ = std::stoi(info_.hardware_parameters["can_id"]);

  RCLCPP_INFO_STREAM(logger_, "Drivetrain Actuator got CAN ID " << can_id_ << " with CAN interface " << can_interface_);

  get_encoder_estimates_id_ = get_arbitration_id(can_id_, get_encoder_estimates_cmd_);
  set_input_vel_id_ = get_arbitration_id(can_id_, set_input_vel_cmd_);

  std::vector<uint32_t> addresses {get_encoder_estimates_id_, set_input_vel_id_};
  drivetrain_can_wrapper_.init(addresses);

  RCLCPP_INFO(logger_, "Drivetrain Actuator Configured Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_deactivate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Drivetrain Actuator Deactivating...");

  motor_velocity_ = 0.0;

  // enable can library to start receiving data for state interfaces and non-movement command interfaces. consider
  // existing state data and non-movement command data as stale

  // assume that can_id is staying the same, no need to read again
  std::vector<uint32_t> addresses {get_encoder_estimates_id_, set_input_vel_id_};
  drivetrain_can_wrapper_.init(addresses);

  RCLCPP_INFO(logger_, "Drivetrain Actuator Deactivated Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_shutdown(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Drivetrain Actuator Shutting Down...");

  // TODO: Null out CAN lib object?

  // zero out the velocities to the wheels before shutting down
  if (drivetrain_can_wrapper_.writeToID<float>(0.0, set_input_vel_id_))
  {
      RCLCPP_INFO_STREAM(logger_, "Successfully set 0 velocity for shutting down on can_id " << can_id_);
  }
  else
  {
    // TODO (npalmar): do something else in case of sending an error
    RCLCPP_INFO_STREAM(logger_, "Error: Failed to set 0 velocity when exiting on can_id " << can_id_);
  }
  

  RCLCPP_INFO(logger_, "Drivetrain Actuator Shut Down Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

LifecyleNodeCallbackReturn UWRTMarsRoverDrivetrainHWActuatorInterface::on_activate(
    const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(logger_, "Drivetrain Actuator Activating...");

  RCLCPP_INFO(logger_, "Drivetrain Actuator Activated Successfully");
  return LifecyleNodeCallbackReturn::SUCCESS;
}

hardware_interface::return_type UWRTMarsRoverDrivetrainHWActuatorInterface::read() {
  RCLCPP_DEBUG(logger_, "Drivetrain Actuator Reading...");

  TwoFloats encoder_readings;
  drivetrain_can_wrapper_.getLatestFromID<TwoFloats>(encoder_readings, get_encoder_estimates_id_);
  
  // Sketchy, not tested yet, copy this into two floats
  // std::memcpy(&actuator_state_position, &encoderEstimates, sizeof(actuator_state_position)); 
  // std::memcpy(&actuator_state_velocity, &encoderEstimates[4], sizeof(actuator_state_velocity)); 
  // TODO (npalmar): confirm the order of reading is correct (might be backwards)
  
  // ignore garbage if we get garbage (seems to work well)
  // ignore garbage if we get garbage
  if (encoder_readings.a > 0.1)
  {
    actuator_state_position_ = (double) encoder_readings.a;
  }
  if (encoder_readings.b > 0.1)
  {
    actuator_state_velocity_ = (double) encoder_readings.b;
  }
  
  // TwoFloats iq_current;
  // drivetrain_can_wrapper_.getLatestFromID<TwoFloats>(iq_current, actuator_state_iq_current_address_);
  // TODO (by Colin) I think we want the last 4 bytes (measured IQ) on the above line, that should be read into float
  // std::memcpy(&actuator_state_iq_current, &iq_current[4], sizeof(actuator_state_iq_current));
  // actuator_state_iq_current_ = (double) iq_current.b;
  
  RCLCPP_INFO_STREAM(logger_, "Actuator Position: " << actuator_state_position_
                                                     << " Actuator Velocity: " << actuator_state_velocity_);
                                                    //  << " Actuator IQ Current: " << actuator_state_iq_current_);


  RCLCPP_DEBUG(logger_, "Drivetrain Actuator Read Successfully...");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type UWRTMarsRoverDrivetrainHWActuatorInterface::write() {
  RCLCPP_DEBUG(logger_, "Drivetrain Actuator Writing...");

  // RCLCPP_INFO_STREAM(logger_, "Joint Velocity: " << motor_velocity_);

  // Write to float because velocity should be 4 bytes
  if (drivetrain_can_wrapper_.writeToID<float>((float) motor_velocity_, set_input_vel_id_))
  {
    RCLCPP_INFO_STREAM(logger_, "Successfully sent joint velocity of " << motor_velocity_ << " for CAN_id " << can_id_);
  }
  else
  {
    // TODO (npalmar): do something else in case of sending an error

    RCLCPP_ERROR_STREAM(logger_, "Failed to send joint velocity for CAN_id" << can_id_);
  }
  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> UWRTMarsRoverDrivetrainHWActuatorInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces_list;
  state_interfaces_list.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_POSITION, &actuator_state_position_));
  state_interfaces_list.emplace_back(hardware_interface::StateInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &actuator_state_velocity_));
  // state_interfaces_list.emplace_back(
  //     hardware_interface::StateInterface(info_.joints[0].name, "iq_current", &actuator_state_iq_current_));
  return state_interfaces_list;
}

std::vector<hardware_interface::CommandInterface>
UWRTMarsRoverDrivetrainHWActuatorInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces_list;
  command_interfaces_list.emplace_back(hardware_interface::CommandInterface(
      info_.joints[0].name, hardware_interface::HW_IF_VELOCITY, &motor_velocity_));
  return command_interfaces_list;
}

}  // namespace uwrt_mars_rover_drivetrain_hw

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(uwrt_mars_rover_drivetrain_hw::UWRTMarsRoverDrivetrainHWActuatorInterface,
                       hardware_interface::ActuatorInterface)
