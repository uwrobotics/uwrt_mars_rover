#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/actuator_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "transmission_interface/simple_transmission.hpp"
#include "uwrt_mars_rover_utilities/uwrt_can.h"
#include "uwrt_mars_rover_drivetrain_hw/visibility_control.hpp"


using LifecyleNodeCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

namespace uwrt_mars_rover_drivetrain_hw {
class UWRTMarsRoverDrivetrainHWActuatorInterface : public hardware_interface::ActuatorInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(UWRTMarsRoverDrivetrainHWActuatorInterface)

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  UWRTMarsRoverDrivetrainHWActuatorInterface();

  // TODO (npalmar): implement the on_error method to turn off wheel velocities when killing node

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  LifecyleNodeCallbackReturn on_init(const hardware_interface::HardwareInfo& actuator_info) override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  LifecyleNodeCallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  LifecyleNodeCallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  LifecyleNodeCallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  LifecyleNodeCallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  LifecyleNodeCallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  hardware_interface::return_type read() override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  hardware_interface::return_type write() override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  UWRT_MARS_ROVER_DRIVETRAIN_HW_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

 protected:
  uint32_t get_arbitration_id(uint8_t can_id, uint32_t cmd_id)
  {
    return can_id << 5 | cmd_id;
  }

  struct TwoFloats {
    float a, b;
  };

  static constexpr uint32_t get_encoder_estimates_cmd_ {0x009};
  static constexpr uint32_t set_input_vel_cmd_ {0x00d};

  uint32_t can_id_; // get the can ID as a parameter in the urdf hardware description
  std::string can_interface_{ "can0" }; 

  // combine can Id and cmd in the variables below for easier access using get_arbitration_id
  uint32_t get_encoder_estimates_id_; 
  uint32_t set_input_vel_id_;

  double actuator_state_position_;
  double actuator_state_velocity_;
  // double actuator_state_iq_current_;
  double motor_velocity_;
  rclcpp::Logger logger_;

  // UWRTMarsRoverDrivetrainHWActuatorInterface defines the following structure in URDF
  static constexpr unsigned int NUM_JOINTS{1};
  static constexpr unsigned int NUM_COMMAND_INTERFACES{1};
  static constexpr unsigned int NUM_STATE_INTERFACES{2};

  // TODO (npalmar): probably remove current stuff from here 
  // static constexpr uint32_t actuator_state_iq_current_address_ {0x014};

  uwrt_mars_rover_utilities::UWRTCANWrapper drivetrain_can_wrapper_;
};

}  // namespace uwrt_mars_rover_drivetrain_hw
