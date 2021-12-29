#pragma once

#include <cstdint>
#include <hardware_interface/actuator_interface.hpp>
#include <hardware_interface/base_interface.hpp>
#include <hardware_interface/handle.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <hardware_interface/types/hardware_interface_status_values.hpp>
#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <memory>
#include <rclcpp/macros.hpp>
#include <rclcpp/rclcpp.hpp>
#include <string>
#include <transmission_interface/simple_transmission.hpp>
#include <vector>

#include "uwrt_mars_rover_drivetrain_hw/visibility.hpp"

namespace uwrt_mars_rover_drivetrain_hw
{
class UWRTMarsRoverDrivetrainHardware
: public hardware_interface::BaseInterface<hardware_interface::ActuatorInterface>
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UWRTMarsRoverDrivetrainHardware)
  
  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  hardware_interface::return_type configure(
    const hardware_interface::HardwareInfo & actuator_info) override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /****** for simulation  ******/
  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  hardware_interface::return_type prepare_command_mode_switch(
    const std::vector<std::string> & start_interfaces,
    const std::vector<std::string> & stop_interfaces) override;
  
  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  hardware_interface::return_type start() override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  hardware_interface::return_type stop() override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  hardware_interface::return_type read() override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  hardware_interface::return_type write() override;

private:
  // Parameters for drivetrain simulation - gazebo or something
  double drivetrain_start_sec{};
  double drivetrain_stop_sec{};
  double drivetrain_slowdown{};

protected:
  std::unordered_map<std::string, std::size_t> joint_state_map;
  std::unordered_map<std::string, std::size_t> joint_command_map;
  

  enum class UWRTDrivetrainCommandMode : std::uint32_t {
    UNDEFINED = 0,
    VELOCITY = 1,
    VOLTAGE = 2, /* TODO: make a voltage controller for voltage control mode */
  };

  struct Actuator
  {
    struct Commands
    {
      double actuator_velocity_command;  // rad/sec
      double joint_velocity_command; // needed for the transmission
      double joint_voltage_command;
      UWRTDrivetrainCommandMode actuator_mode;  // which type of commands are being sent
    };

    using Commands = struct Commands;
    Commands commands;

    struct States
    {
      double joint_velocity;  // rad/sec
      double joint_voltage;
    };

    using States = struct States;
    States states;
  };

  using Actuator = struct Actuator;
  Actuator actuator;

  struct Conversion
  {
    static constexpr double MOTOR_READING_TO_AMPS_CONVERSION_FACTOR{10.0};
    static constexpr double RPM_TO_RADIANS_PER_SECOND_FACTOR{2 * M_PI / 60};
    static constexpr double REVOLUTIONS_TO_RADIANS_FACTOR{2 * M_PI};
    static constexpr double RADIANS_PER_SECOND_TO_RPM_FACTOR{60 / M_PI / 2};
    static constexpr double REVOLUTIONS_PER_RADIAN{1 / (2 * M_PI)};
  };


  // helpful functions to access joint names
  std::string joint_name;
  inline std::string get_joint_name() const { return joint_name; }
  inline void set_joint_name(const std::string & name) { joint_name = std::move(name); }

  // logger for the actuator
  inline rclcpp::Logger actuator_logger() const { return rclcpp::get_logger(this->get_name()); }

  // number of command interfaces for joints - look at urdf
  static constexpr unsigned int NUM_COMMAND_INTERFACES{1};

  // number of state interfaces for joints - look at urdf
  static constexpr unsigned int NUM_STATE_INTERFACES{1};

  // transmission stuff - there should only be one actuator linked to one joint
  // make into a pointer 
  std::unique_ptr<transmission_interface::SimpleTransmission> command_transmission;
  std::unique_ptr<transmission_interface::SimpleTransmission> state_transmission;

  // joint & actuator handles for the transmission

  std::unique_ptr<transmission_interface::JointHandle> velocity_joint_handle;
  std::unique_ptr<transmission_interface::ActuatorHandle> actuator_command_handle;
  

  // parse urdf and get all drivetrain joints and transmissions
  UWRT_MARS_ROVER_DRIVETRAIN_HW_LOCAL
  bool configureDrivetrainJoints(
    const std::vector<hardware_interface::ComponentInfo> & drivetrainJoints);

  UWRT_MARS_ROVER_DRIVETRAIN_HW_LOCAL
  bool configureDrivetrainTransmissions(
    const std::vector<hardware_interface::ComponentInfo> & drivetrainTransmissions);

  UWRT_MARS_ROVER_DRIVETRAIN_HW_LOCAL
  bool configureDrivetrainHardwareInfo(
    const hardware_interface::HardwareInfo & drivetarinHardwareInfo);
};

}  // namespace uwrt_mars_rover_drivetrain_hw
