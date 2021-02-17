#pragma once

#include <control_msgs/JointTrajectoryControllerState.h>
#include <controller_interface/controller.h>
#include <geometry_msgs/TwistStamped.h>
#include <hardware_interface/joint_command_interface.h>
#include <realtime_tools/realtime_buffer.h>
#include <realtime_tools/realtime_publisher.h>
#include <ros/node_handle.h>
#include <uwrt_mars_rover_drivetrain_msgs/OpenLoopTwist.h>
#include <uwrt_mars_rover_hw/voltage_joint_interface.h>

#include <string>
#include <vector>

namespace diff_drive_voltage_controller {

/**
 * DiffDriveVoltageController - A ROS controller that takes in a uwrt_mars_rover_drivetrain_msgs::OpenLoopTwist msg and
 * decomposes it into left-side and right-side motor voltage commands. This is loosely based off of
 * diff_drive_controller.
 *
 * A large difference is that this controller takes in a uwrt_mars_rover_drivetrain_msgs::OpenLoopTwist msg that has
 * linear x and angular z components that are in the range of [-1.0, 1.0]. This controller will optionally clamp the
 * open loop commands, optionally publish clamped open loop commands,
 * decompose the commands from linear & angular to left & right, and optionally publish the controller state (commands
 * sent to RobotHW)
 */
class DiffDriveVoltageController
    : public controller_interface::Controller<uwrt_hardware_interface::VoltageJointInterface> {
 public:
  DiffDriveVoltageController() = default;

  bool init(uwrt_hardware_interface::VoltageJointInterface* hw, ros::NodeHandle& controller_nh) override;
  void starting(const ros::Time& /*time*/) override;
  void stopping(const ros::Time& /*time*/) override;
  void update(const ros::Time& time, const ros::Duration& /*period*/) override;

 private:
  bool loadWheelParameters(ros::NodeHandle& controller_nh, std::vector<std::string>& left_wheel_names,
                           std::vector<std::string>& right_wheel_names);
  bool getWheelNames(ros::NodeHandle& controller_nh, const std::string& wheel_param,
                     std::vector<std::string>& wheel_names);
  void commandSubscriberCallback(const uwrt_mars_rover_drivetrain_msgs::OpenLoopTwist& command);
  void stopMotors();

 private:
  // limit defaults
  static constexpr bool DEFAULT_HAS_LINEAR_CMD_LIMITS{false};
  static constexpr double DEFAULT_MAX_LINEAR_CMD{1.0};
  static constexpr double DEFAULT_MIN_LINEAR_CMD{-1.0};
  static constexpr bool DEFAULT_HAS_ANGULAR_CMD_LIMITS{false};
  static constexpr double DEFAULT_MAX_ANGULAR_CMD{1.0};
  static constexpr double DEFAULT_MIN_ANGULAR_CMD{-1.0};

  // publisher & subscriber option defaults
  static constexpr double DEFAULT_CMD_TIMEOUT{0.5};
  static constexpr bool DEFAULT_ALLOW_MULTIPLE_CMD_PUBLISHERS{true};
  static constexpr bool DEFAULT_PUBLISH_CONTROLLER_CMD_OUTPUT{false};
  static constexpr bool DEFAULT_PUBLISH_WHEEL_JOINT_CONTROLLER_STATE{false};
  static constexpr unsigned DEFAULT_PUBLISH_QUEUE_SIZE{100};

  struct Command {
    double linear{0.0};
    double angular{0.0};
    ros::Time timestamp{0.0};
  };

  std::string name_;

  struct JointHandlePair {
    hardware_interface::JointHandle left_wheel_joint;
    hardware_interface::JointHandle right_wheel_joint;
  };
  std::vector<JointHandlePair> wheel_joint_pairs_;

  realtime_tools::RealtimeBuffer<Command> command_realtime_buffer_;
  Command command_struct_;

  ros::Subscriber command_subscriber_;

  std::shared_ptr<realtime_tools::RealtimePublisher<uwrt_mars_rover_drivetrain_msgs::OpenLoopTwist>>
      output_command_publisher_;

  // limits
  bool has_linear_cmd_limits_{false};
  double max_linear_cmd_{0.0};
  double min_linear_cmd_{0.0};
  bool has_angular_cmd_limits_{false};
  double max_angular_cmd_{0.0};
  double min_angular_cmd_{0.0};

  // publisher & subscriber options
  double cmd_timeout_{0.0};
  bool allow_multiple_cmd_publishers_{false};
  bool publish_controller_cmd_output_{false};
  bool publish_wheel_joint_controller_state_{false};
};
}  // namespace diff_drive_voltage_controller
