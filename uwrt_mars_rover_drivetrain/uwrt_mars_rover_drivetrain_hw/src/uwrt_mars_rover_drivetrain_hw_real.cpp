#include "uwrt_mars_rover_drivetrain_hw/uwrt_mars_rover_drivetrain_hw_real.hpp"

using hardware_interface::return_type;
using hardware_interface::status;

namespace uwrt_mars_rover_drivetrain_hw
{
return_type UWRTMarsRoverDrivetrainHardwareReal::configure(
  const hardware_interface::HardwareInfo & actuator_info)
{
  bool configure_success{true};

  /*****************
   *
   *
   *  create CAN class
   *
   *
   ****************/

  configure_success &= (configure_default(actuator_info) == return_type::OK);
  configure_success &= configureDrivetrainHardwareInfo(actuator_info);
  configure_success &= configureDrivetrainJoints(actuator_info.joints);
  configure_success &= configureDrivetrainTransmissions(actuator_info.transmissions);

  if (configure_success) {
    RCLCPP_INFO(actuator_logger(), "Hardware Configured ...");
    status_ = status::CONFIGURED;
    return return_type::OK;
  } else {
    RCLCPP_ERROR(actuator_logger(), "Hardware Configuration Failed ...");
    status_ = status::UNKNOWN;
    return return_type::ERROR;
  }
}

// not really needed right now as we only support one command interface but will be needed when we support voltage
// commands
return_type UWRTMarsRoverDrivetrainHardwareReal::perform_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  (void)start_interfaces;
  (void)stop_interfaces;
  return return_type::OK;
}

// read position and velocity into state interfaces
return_type UWRTMarsRoverDrivetrainHardwareReal::read()
{
  // read velocity (radians/sec) from actuator
  try {
    // read into actuator velocity
    actuator.states.joint_velocity = Conversion::RPM_TO_RADIANS_PER_SECOND_FACTOR;
    // read from CAN bus

  } catch (const std::exception & ex) {
    RCLCPP_ERROR(actuator_logger(), "EXCEPTION: ", ex.what());
    return return_type::ERROR;
  }

  // bring readings from actuator space to joint space
  state_transmission->actuator_to_joint();

  return return_type::OK;
}

return_type UWRTMarsRoverDrivetrainHardwareReal::write()
{
  bool success{true};

  (void)success;
  switch (actuator.commands.actuator_mode) {
    // velocity diff drive
    case UWRTDrivetrainCommandMode::VELOCITY:

      // propogate from joint to actuator space
      command_transmission->joint_to_actuator();

      try {
        std::cout << actuator.commands.actuator_velocity_command << "Write to CAN BUS" << std::endl;
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(actuator_logger(), "EXCEPTION: %s", ex.what());
        return return_type::ERROR;
      }

      break;

    // voltage diff drive .... TODO: need to make voltage diff drive
    case UWRTDrivetrainCommandMode::VOLTAGE:

      try {
        std::cout << "Not yet supported" << actuator.commands.joint_voltage_command << std::endl;
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(actuator_logger(), "EXCEPTION: %s", ex.what());
        return return_type::ERROR;
      }

      RCLCPP_INFO(actuator_logger(), "Not yet supported");
      break;

    // undefined .... stop actuator
    case UWRTDrivetrainCommandMode::UNDEFINED:

      try {
        std::cout << "Stopping actuator" << std::endl;
      } catch (const std::exception & ex) {
        RCLCPP_ERROR(actuator_logger(), "EXCEPTION: %s", ex.what());
        return return_type::ERROR;
      }
      break;

    // unknown command type error
    default:
      RCLCPP_ERROR(actuator_logger(), "Command Type not found");
      return return_type::ERROR;
  }
  return return_type::OK;
}

return_type UWRTMarsRoverDrivetrainHardwareReal::stop()
{
  RCLCPP_INFO(actuator_logger(), "Stopping Drivetrain ...");

  try {
    std::cout << "Send CAN message to stop actuator" << std::endl;
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(actuator_logger(), "EXCEPTION %s", ex.what());
    RCLCPP_ERROR(
      actuator_logger(), "CAN Message may or may not have been sent to actuator to stop");
    status_ = status::UNKNOWN;
    return return_type::ERROR;
  }

  status_ = status::STOPPED;
  RCLCPP_INFO(actuator_logger(), "Drivetrain Stopped Successfully");
  return return_type::OK;
}

return_type UWRTMarsRoverDrivetrainHardwareReal::start()
{
  //bool can_setup_complete{};

  RCLCPP_INFO(actuator_logger(), "Starting Drivetrain ...");

  /******************
   * 
   * Call CAN init on CAN class that 
   * was configured in in configure function
   * 
   *******************/
  try {
    //can_setup_complete = (status_ == status::STARTED);
  } catch (const std::exception & ex) {
    RCLCPP_ERROR(actuator_logger(), "EXCEPTION: %s", ex.what());
    RCLCPP_ERROR(actuator_logger(), "CAN Bus was not setup properly");
    status_ = status::UNKNOWN;
    return return_type::ERROR;
  }

  RCLCPP_INFO(actuator_logger(), "CAN setup complete");

  // set default values for when starting .... set everything to zero
  // sanity check using the isnan
  if (std::isnan(actuator.commands.actuator_velocity_command)) {
    actuator.commands.actuator_velocity_command = 0.0;
  }
  if (std::isnan(actuator.states.joint_velocity)) {
    actuator.states.joint_velocity = 0.0;
  }
  if (std::isnan(actuator.states.joint_velocity)) {
    actuator.states.joint_velocity = 0.0;
  }
  
  status_ = status::STARTED;

  // set command mode to velocity command mode by default
  actuator.commands.actuator_mode = UWRTDrivetrainCommandMode::VELOCITY;

  RCLCPP_INFO(actuator_logger(), "Drivetrain Started Successfully - default Velocity Command mode");
  return return_type::OK;
}

}  // namespace uwrt_mars_rover_drivetrain_hw

#include "pluginlib/class_list_macros.hpp"

PLUGINLIB_EXPORT_CLASS(
  uwrt_mars_rover_drivetrain_hw::UWRTMarsRoverDrivetrainHardwareReal,
  hardware_interface::ActuatorInterface)