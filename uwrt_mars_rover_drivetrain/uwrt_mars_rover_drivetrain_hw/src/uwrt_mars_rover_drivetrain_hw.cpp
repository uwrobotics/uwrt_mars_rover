#include "uwrt_mars_rover_drivetrain_hw/uwrt_mars_rover_drivetrain_hw.hpp"

using hardware_interface::return_type;
using hardware_interface::status;
using transmission_interface::ActuatorHandle;
using transmission_interface::JointHandle;

namespace uwrt_mars_rover_drivetrain_hw
{
return_type UWRTMarsRoverDrivetrainHardware::configure(
  const hardware_interface::HardwareInfo & actuator_info)
{
  bool configure_success{true};

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

// load parameters if there are any
bool UWRTMarsRoverDrivetrainHardware::configureDrivetrainHardwareInfo(
  const hardware_interface::HardwareInfo & drivetrainHardwareInfo)
{

  // setup joint map between command interfaces and state interfaces
  std::cout << "hello" << std::endl;
  std::size_t state_index{0}, command_index{0};

  for(auto& joint : drivetrainHardwareInfo.joints){

    set_joint_name(joint.name);

    for(auto& state_interfaces : joint.state_interfaces){
      if(state_interfaces.name == std::string(hardware_interface::HW_IF_VELOCITY)){
        joint_state_map.insert({hardware_interface::HW_IF_VELOCITY, state_index});
      }
      state_index++;
    }

    for(auto& command_interfaces: joint.command_interfaces){
      if(command_interfaces.name == std::string(hardware_interface::HW_IF_VELOCITY)){
        joint_command_map.insert({hardware_interface::HW_IF_VELOCITY, command_index});
      }
      command_index++;
    }
  }

  return true;
}


// setup and configure joints ..... only one joint
bool UWRTMarsRoverDrivetrainHardware::configureDrivetrainJoints(
  const std::vector<hardware_interface::ComponentInfo> & drivetrainJoints)
{ 
  std::cout << "hiii" << std::endl;
  std::cout << "cmd map size: " << joint_command_map.size()<<" " << "state map size: " << joint_state_map.size() << std::endl;
  
  for(auto& i : joint_state_map){
    std::cout << i.first << i.second << std::endl;
  }
  
  for(auto& i : joint_command_map){
    std::cout << i.first << i.second << std::endl;
  }

  // there is only one joint really
  // each joint has one command interface (velocity) and two state interfacse (position and velocity)
  for (const hardware_interface::ComponentInfo & joint : drivetrainJoints) {

    // check for number of command interfaces - there should only be one
    if (joint.command_interfaces.size() != NUM_COMMAND_INTERFACES) {
      RCLCPP_FATAL(actuator_logger(), "Incorrect number of command interfaces found");
      return false;
    }

    // check the command interface type - there should be one command interface
    if (
      joint.command_interfaces[joint_command_map.at(hardware_interface::HW_IF_VELOCITY)].name !=
      hardware_interface::HW_IF_VELOCITY) {
      RCLCPP_FATAL(actuator_logger(), "Velcoity Command Interface not found");
      return false;
    }

    // check for number of state interfaces - there should be one  .... velocity
    if (joint.state_interfaces.size() != NUM_STATE_INTERFACES) {
      RCLCPP_FATAL(actuator_logger(), "Incorrect number of state interfaces found");
      return false;
    }

    // check for state interface type - there should be one state interfaces ....  velocity
    //if (
    //  joint.state_interfaces[joint_command_map.at(hardware_interface::HW_IF_POSITION)].name !=
    //  hardware_interface::HW_IF_VELOCITY) {
    //  RCLCPP_FATAL(actuator_logger(), "Velocity State Interface not found");
    //  return false;
    //}
  }

  return true;
}

// setup and configure transmission .... only one transmission
bool UWRTMarsRoverDrivetrainHardware::configureDrivetrainTransmissions(
  const std::vector<hardware_interface::ComponentInfo> & drivetrainTransmissions)
{
  // there is only one transmission per joint .... dont really need a loop
  for (const hardware_interface::ComponentInfo & transmission_info : drivetrainTransmissions) {
    // get transmission reduction value from urdf
    double mechanicle_reduction{std::stod(transmission_info.parameters.at("mechanical_reduction"))};

    // instantiate transmission
    //transmission = transmission_interface::SimpleTransmission(mechanicle_reduction);
    this->command_transmission = std::make_unique<transmission_interface::SimpleTransmission>(mechanicle_reduction);

    // create jointhandle
    *velocity_joint_handle = JointHandle(
      this->get_joint_name(), hardware_interface::HW_IF_VELOCITY,
      &actuator.commands.joint_velocity_command);

    // create actuator handle
    *actuator_command_handle = ActuatorHandle(
      this->get_name(), hardware_interface::HW_IF_VELOCITY,
      &actuator.commands.actuator_velocity_command);

    // configure transmission
    command_transmission->configure(
      {*velocity_joint_handle}, {*actuator_command_handle});
  }
  return true;
}

std::vector<hardware_interface::StateInterface>
UWRTMarsRoverDrivetrainHardware::export_state_interfaces()
{
  // create state interface to export
  std::vector<hardware_interface::StateInterface> state_interfaces_list;

  state_interfaces_list.emplace_back(hardware_interface::StateInterface(
    this->get_joint_name(), hardware_interface::HW_IF_VELOCITY,
    &actuator.states.joint_velocity));

  return state_interfaces_list;
}

std::vector<hardware_interface::CommandInterface>
UWRTMarsRoverDrivetrainHardware::export_command_interfaces()
{
  // create command interfaces to export
  std::vector<hardware_interface::CommandInterface> command_interfaces_list;

  command_interfaces_list.emplace_back(hardware_interface::CommandInterface(
    this->get_joint_name(), hardware_interface::HW_IF_VELOCITY,
    &actuator.commands.joint_velocity_command));

  return command_interfaces_list;
}

return_type UWRTMarsRoverDrivetrainHardware::start()
{
  RCLCPP_INFO(actuator_logger(), "Starting Drivetrain ...");

  // set default values for when starting .... set everything to zero
  // sanity check using the isnan
  if (std::isnan(actuator.commands.actuator_velocity_command)) {
    actuator.commands.actuator_velocity_command = 0.0;
  }
  if(std::isnan(actuator.commands.joint_velocity_command)) {
    actuator.commands.joint_velocity_command = 0.0;
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

return_type UWRTMarsRoverDrivetrainHardware::stop()
{
  RCLCPP_INFO(actuator_logger(), "Stopping Drivetrain ...");
  status_ = status::STOPPED;
  RCLCPP_INFO(actuator_logger(), "Drivetrain Stopped Successfully");
  return return_type::OK;
}

// not really needed right now as we only support commands in one mode ... but will be needed in the future to support voltage
return_type UWRTMarsRoverDrivetrainHardware::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  (void)start_interfaces;
  (void)stop_interfaces;

  return return_type::OK;
}

/**************    simulation read    *********************/
return_type UWRTMarsRoverDrivetrainHardware::read() { return return_type::OK; }

/***************** simulation write     *****************/
return_type UWRTMarsRoverDrivetrainHardware::write() { return return_type::OK; }

}  // namespace uwrt_mars_rover_drivetrain_hw

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
  uwrt_mars_rover_drivetrain_hw::UWRTMarsRoverDrivetrainHardware,
  hardware_interface::ActuatorInterface)