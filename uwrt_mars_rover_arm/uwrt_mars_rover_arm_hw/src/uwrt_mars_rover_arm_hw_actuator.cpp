#include "uwrt_mars_rover_arm_hw/uwrt_mars_rover_arm_hw_actuator.hpp"

#include <cstdlib>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <limits>
#include <rclcpp/logger.hpp>
#include <rclcpp/logging.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>
#include <transmission_interface/handle.hpp>
#include <transmission_interface/simple_transmission.hpp>
#include <transmission_interface/transmission.hpp>

using hardware_interface::CommandInterface;
using hardware_interface::HardwareInfo;
using hardware_interface::return_type;
using hardware_interface::StateInterface;

using transmission_interface::ActuatorHandle;
using transmission_interface::JointHandle;
using transmission_interface::SimpleTransmission;

using rclcpp_lifecycle::State;

namespace uwrt_mars_rover_arm_hw
{
/***
 *
 * Dont need this function rn, but it is nice to see how things would look with a transmission
 *
 ***/
bool ArmActuatorInterface::configure_transmission()
{
  // TODO: actually get real value for joint to actuator reduction
  state_transmission = std::make_shared<SimpleTransmission>(joint_to_actuator_reduction, 0.0);
  command_transmission = std::make_shared<SimpleTransmission>(joint_to_actuator_reduction, 0.0);

  if (!command_transmission || !state_transmission) {
    return false;
  }

  // only want these to get created once ... might as well make it const while we're at it
  static const std::string actuator{"_actuator"};
  static const std::string joint{"_joint"};
  static const std::string vel{"_vel_handle"};
  static const std::string pos{"_pos_handle"};

  // create actuator & joint handles for state transmission
  actuator_velocity_state_handle = std::make_shared<ActuatorHandle>(
    info_.name + actuator + pos, hardware_interface::HW_IF_VELOCITY, &actuator_states.velocity);
  joint_velocity_state_hanlde = std::make_shared<JointHandle>(
    info_.name + joint + vel, hardware_interface::HW_IF_VELOCITY, &joint_states.velocity);
  actuator_position_state_handle = std::make_shared<ActuatorHandle>(
    info_.name + actuator + pos, hardware_interface::HW_IF_POSITION, &actuator_states.position);
  joint_position_state_handle = std::make_shared<JointHandle>(
    info_.name + joint + pos, hardware_interface::HW_IF_POSITION, &joint_states.position);

  // create actuator & joint handles for command transmission
  actuator_velocity_command_handle = std::make_shared<ActuatorHandle>(
    info_.name + actuator + pos, hardware_interface::HW_IF_VELOCITY, &actuator_commands.velocity);
  joint_velocity_command_handle = std::make_shared<JointHandle>(
    info_.name + joint + vel, hardware_interface::HW_IF_VELOCITY, &joint_commands.velocity);

  if (
    !actuator_velocity_state_handle || !joint_velocity_state_hanlde ||
    !actuator_position_state_handle || !joint_position_state_handle ||
    !actuator_velocity_command_handle || !joint_velocity_command_handle) {
    return false;
  }

  // configure both state & command transmissions
  command_transmission->configure(
    {*joint_velocity_command_handle}, {*actuator_velocity_command_handle});
  state_transmission->configure(
    {*joint_velocity_state_hanlde, *joint_position_state_handle},
    {*actuator_velocity_state_handle, *actuator_position_state_handle});

  return true;
}

std::vector<StateInterface> ArmActuatorInterface::export_state_interfaces()
{
  std::vector<StateInterface> state_interfaces_list{};

  state_interfaces_list.emplace_back(StateInterface(
    info_.joints.at(0).name, hardware_interface::HW_IF_POSITION, &actuator_states.position));
  state_interfaces_list.emplace_back(StateInterface(
    info_.joints.at(0).name, hardware_interface::HW_IF_VELOCITY, &actuator_states.velocity));

  state_interfaces_list.emplace_back(StateInterface(
    info_.joints.at(0).name, ArmActuatorInterface::HW_IF_IQ_CURRENT, &actuator_states.iq_current));

  return state_interfaces_list;
}

std::vector<CommandInterface> ArmActuatorInterface::export_command_interfaces()
{
  std::vector<CommandInterface> command_interface_list{};

  command_interface_list.emplace_back(CommandInterface(
    info_.joints.at(0).name, hardware_interface::HW_IF_POSITION, &joint_commands.velocity));

  return command_interface_list;
}

// we will need this once we have multiple ways of controlling
return_type ArmActuatorInterface::prepare_command_mode_switch(
  const std::vector<std::string> & start_interfaces,
  const std::vector<std::string> & stop_interfaces)
{
  (void)start_interfaces;
  (void)stop_interfaces;

  return return_type::OK;
}

return_type ArmActuatorInterface::read()
{
  RCLCPP_DEBUG(actuator_logger(), "Arm Actuator Reading ...");

  // call actuator to joint space transmission API function here
  // aka state_transmission->actuator_to_joint();

  // dummy values - generated from 1 to 10
  static const unsigned int RANDOM{1 + 10};
  std::srand(std::time(nullptr));

  actuator_states.position = (double)(std::rand() % RANDOM);
  actuator_states.velocity = (double)(std::rand() % RANDOM);
  actuator_states.iq_current = (double)(std::rand() % RANDOM);

  RCLCPP_INFO_STREAM(
    actuator_logger(),
    "Actuator Position State: " << actuator_states.position
                                << "Actuator Velocity State: " << actuator_states.velocity
                                << "Actuator IQ Current State: " << actuator_states.iq_current);
  /*
    * Melvin CAN API Stuff
    *
    */

  RCLCPP_DEBUG(actuator_logger(), "Arm Actuator Read Successfull ...");

  return return_type::OK;
}

return_type ArmActuatorInterface::write()
{
  RCLCPP_DEBUG(actuator_logger(), "Arm Actuator Writing ...");

  // call joint space to actuator space transmission API function here
  // aka 'command_transmission->joint_to_actuator()' then actually write actuator_commands.velocity to CAN
  // not joint_commands.velocity
  actuator_commands.velocity =
    joint_to_actuator_reduction * joint_commands.velocity;  //transmission

  RCLCPP_INFO_STREAM(actuator_logger(), "Joint Velocity Command: " << actuator_commands.velocity);
  /*
    * Melvin CAN API Stuff
    *
    */

  RCLCPP_DEBUG(actuator_logger(), "Arm Actuator Write Successfull ...");

  return return_type::OK;
}

CallbackReturn ArmActuatorInterface::on_init(const HardwareInfo & hardware_info)
{
  if (hardware_interface::ActuatorInterface::on_init(hardware_info) != CallbackReturn::SUCCESS) {
    RCLCPP_FATAL_STREAM(
      rclcpp::get_logger("UWRTMarsRoverArmActuator"), "FAILED TO PARSE ARM URDF ...");
    return CallbackReturn::ERROR;
  }

  // setup logger that will be unique to each joint name
  actuator_logger = [this]() -> rclcpp::Logger {
    return rclcpp::get_logger(this->info_.joints.at(0).name);
  };

  // validate ARM URDF
  // check there are the correct number of joints, interfaces, and that they exist

  if (info_.joints.size() != NUM_JOINTS) {
    RCLCPP_FATAL_STREAM(
      actuator_logger(), "'" << info_.name.c_str() << "' has " << info_.joints.size()
                             << " joints. Expected: " << NUM_JOINTS << "'");
    return CallbackReturn::ERROR;
  }

  const hardware_interface::ComponentInfo & joint =
    info_.joints.at(0);  // alias for easy use - only one joint

  if (joint.state_interfaces.size() != NUM_STATE_INTERFACES) {
    RCLCPP_FATAL_STREAM(
      actuator_logger(), "Joint '"
                           << joint.name.c_str() << "' has " << joint.state_interfaces.size()
                           << " state interface. Expected: " << NUM_STATE_INTERFACES << "'");

    return CallbackReturn::ERROR;
  }

  if (joint.command_interfaces.size() != NUM_COMMAND_INTERFACES) {
    RCLCPP_FATAL_STREAM(
      actuator_logger(), "Joint '"
                           << joint.name.c_str() << "' has " << joint.command_interfaces.size()
                           << " command interface. Expected: " << NUM_COMMAND_INTERFACES << "'");

    return CallbackReturn::ERROR;
  }

  // lol only one joint
  for (const hardware_interface::InterfaceInfo & state_interfaces : joint.state_interfaces) {
    if (!(state_interfaces.name == hardware_interface::HW_IF_POSITION ||
          state_interfaces.name == hardware_interface::HW_IF_VELOCITY ||
          state_interfaces.name == ArmActuatorInterface::HW_IF_IQ_CURRENT)) {
      RCLCPP_FATAL_STREAM(
        actuator_logger(), "Joint '" << joint.name.c_str() << "'has "
                                     << state_interfaces.name.c_str() << "state interfaces. '"
                                     << hardware_interface::HW_IF_POSITION << "' or '"
                                     << hardware_interface::HW_IF_VELOCITY << "' or '"
                                     << ArmActuatorInterface::HW_IF_IQ_CURRENT << "' expected.");

      return CallbackReturn::ERROR;
    }
  }

  // lol literally only one interface ... could use if statement ... but for sake of scalability and coherency lets use a for loop
  for (const hardware_interface::InterfaceInfo & command_interface : joint.command_interfaces) {
    if (!(command_interface.name == hardware_interface::HW_IF_VELOCITY)) {
      RCLCPP_FATAL_STREAM(
        actuator_logger(), "Joint '" << joint.name.c_str() << "'has "
                                     << command_interface.name.c_str() << "command interfaces. '"
                                     << hardware_interface::HW_IF_VELOCITY << "' expected.");
    }

    return CallbackReturn::ERROR;
  }

  // set all intial states and command to 'Not a Number'
  joint_commands.velocity = std::numeric_limits<double>::quiet_NaN();
  actuator_commands.velocity = std::numeric_limits<double>::quiet_NaN();

  joint_states.velocity = std::numeric_limits<double>::quiet_NaN();
  actuator_states.velocity = std::numeric_limits<double>::quiet_NaN();

  joint_states.position = std::numeric_limits<double>::quiet_NaN();
  actuator_states.position = std::numeric_limits<double>::quiet_NaN();

  actuator_states.iq_current = std::numeric_limits<double>::quiet_NaN();

  // TODO: initialize CAN library

  // TODO: intialize transmission
  /**** Tranmission stuff
    if(configure_transmission()){
       RCLCPP_ERROR_STREAM(actuator_logger(), 
          "Transmission linked with: '" << info_.name << "' failed to be configured from URDF"
               );

       return CallbackReturn::ERROR;
    }
    ****/
  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmActuatorInterface::on_configure(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(actuator_logger(), "Arm Actuator Configuring");

  /*
     * Set this to 0 every time configure is called regardless of its value
     * as we do not want to accidentally write a value that could be NaN or some garbage - causing unwanted movement
     * ideally we would do this to all the location of memory where the command interface resides
     */
  joint_commands.velocity = 0;

  /* 
     * if the state interfaces are NaN (which may happen from jumping from different 'lifecyle states')
     * set them to 0, else just leave it - it will get correctly 
     * updated eventually in the update loop and we still keep the data from the last read this way. 
     * ideally we would so this to all the location of memory where the state interfaces resides
     */
  if (std::isnan(actuator_states.velocity)) {
    actuator_states.velocity = 0;
  }
  if (std::isnan(actuator_states.position)) {
    actuator_states.position = 0;
  }
  if (std::isnan(actuator_states.iq_current)) {
    actuator_states.iq_current = 0;
  }

  RCLCPP_INFO(actuator_logger(), "Arm Actuator Configured Successfully");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmActuatorInterface::on_cleanup(const rclcpp_lifecycle::State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(actuator_logger(), "Arm Actuator Cleaning Up");

  // reset values in the command & state interfaces location to NaN
  joint_commands.velocity = std::numeric_limits<double>::quiet_NaN();

  actuator_states.velocity = std::numeric_limits<double>::quiet_NaN();
  actuator_states.position = std::numeric_limits<double>::quiet_NaN();
  actuator_states.iq_current = std::numeric_limits<double>::quiet_NaN();

  /*
     * Melvin CAN API reset
     */

  RCLCPP_INFO(actuator_logger(), "Arm Actuator Clean Up Successfull");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmActuatorInterface::on_activate(const State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(actuator_logger(), "Arm Actuator Activating");
  RCLCPP_INFO(actuator_logger(), "Arm Actuator Activated Successfully");

  return CallbackReturn::SUCCESS;
}

/*
 *
 *
 *
 */
CallbackReturn ArmActuatorInterface::on_deactivate(const State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(actuator_logger(), "Arm Actuator Deactivating");

  joint_commands.velocity = 0;

  RCLCPP_INFO(actuator_logger(), "Arm Actuator Deactivating Successfully");

  return CallbackReturn::SUCCESS;
}

CallbackReturn ArmActuatorInterface::on_shutdown(const State & previous_state)
{
  (void)previous_state;

  RCLCPP_INFO(actuator_logger(), "Arm Actuator Shutting Down");

  /*
     * Melvin CAN API Clean up
     */

  RCLCPP_INFO(actuator_logger(), "Arm Actuator Shut Down Successfull");

  return CallbackReturn::SUCCESS;
}

}  // namespace uwrt_mars_rover_arm_hw

#include <pluginlib/class_list_macros.hpp>

PLUGINLIB_EXPORT_CLASS(
  uwrt_mars_rover_arm_hw::ArmActuatorInterface, hardware_interface::ActuatorInterface)
