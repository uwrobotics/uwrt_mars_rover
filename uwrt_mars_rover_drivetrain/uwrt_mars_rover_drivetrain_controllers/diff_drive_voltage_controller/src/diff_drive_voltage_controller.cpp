#include <diff_drive_voltage_controller/diff_drive_voltage_controller.h>
#include <uwrt_mars_rover_utils/uwrt_params.h>

#include <algorithm>
#include <pluginlib/class_list_macros.hpp>

namespace diff_drive_voltage_controller {
// static constexpr class members must have definitions outside of their class to compile. This can be removed in C++17
constexpr double DiffDriveVoltageController::DEFAULT_CMD_TIMEOUT;
constexpr bool DiffDriveVoltageController::DEFAULT_ALLOW_MULTIPLE_CMD_PUBLISHERS;
constexpr bool DiffDriveVoltageController::DEFAULT_PUBLISH_CONTROLLER_CMD_OUTPUT;
constexpr bool DiffDriveVoltageController::DEFAULT_PUBLISH_WHEEL_JOINT_CONTROLLER_STATE;

bool DiffDriveVoltageController::init(hardware_interface::VelocityJointInterface* hw, ros::NodeHandle& controller_nh) {
  name_ = uwrt_mars_rover_utils::getLoggerName(controller_nh);

  std::vector<std::string> left_wheel_names, right_wheel_names;
  if (!loadWheelParameters(controller_nh, left_wheel_names, right_wheel_names)) {
    return false;
  }

  // TODO: load vel/accel limits in position mode
  has_linear_cmd_limits_ =
      uwrt_mars_rover_utils::getParam(controller_nh, name_, "linear/x/has_limits", DEFAULT_HAS_LINEAR_CMD_LIMITS);
  max_linear_cmd_ = uwrt_mars_rover_utils::getParam(controller_nh, name_, "linear/x/max_cmd", DEFAULT_MAX_LINEAR_CMD);
  min_linear_cmd_ = uwrt_mars_rover_utils::getParam(controller_nh, name_, "linear/x/min_cmd", DEFAULT_MIN_LINEAR_CMD);
  has_angular_cmd_limits_ =
      uwrt_mars_rover_utils::getParam(controller_nh, name_, "angular/z/has_limits", DEFAULT_HAS_ANGULAR_CMD_LIMITS);
  max_angular_cmd_ =
      uwrt_mars_rover_utils::getParam(controller_nh, name_, "angular/z/max_cmd", DEFAULT_MAX_ANGULAR_CMD);
  min_angular_cmd_ =
      uwrt_mars_rover_utils::getParam(controller_nh, name_, "angular/z/min_cmd", DEFAULT_MIN_ANGULAR_CMD);

  cmd_timeout_ = uwrt_mars_rover_utils::getParam(controller_nh, name_, "cmd_timeout", DEFAULT_CMD_TIMEOUT);
  allow_multiple_cmd_publishers_ =
      uwrt_mars_rover_utils::getParam(controller_nh, name_, "cmd_timeout", DEFAULT_ALLOW_MULTIPLE_CMD_PUBLISHERS);
  publish_controller_cmd_output_ = uwrt_mars_rover_utils::getParam(
      controller_nh, name_, "publish_controller_output_cmd", DEFAULT_PUBLISH_CONTROLLER_CMD_OUTPUT);

  if (publish_controller_cmd_output_) {
    output_command_publisher_.reset(
        new realtime_tools::RealtimePublisher<uwrt_mars_rover_drivetrain_msgs::OpenLoopTwist>(
            controller_nh, "cmd_open_loop_out", 100));
  }


  //  publish_wheel_joint_controller_state_ = uwrt_mars_rover_utils::getParam(
  //      controller_nh, "publish_wheel_joint_controller_state", DEFAULT_PUBLISH_WHEEL_JOINT_CONTROLLER_STATE, name_);
  //  if (publish_wheel_joint_controller_state_) {
  //    controller_state_pub_.reset(new
  //    realtime_tools::RealtimePublisher<control_msgs::JointTrajectoryControllerState>(controller_nh,
  //    "wheel_joint_controller_state", 100));
  //
  //          const size_t num_wheels = wheel_joint_pairs_.size() * 2;
  //
  //      controller_state_pub_->msg_.joint_names.resize(num_wheels);
  //
  //      controller_state_pub_->msg_.desired.positions.resize(num_wheels);
  //      controller_state_pub_->msg_.desired.velocities.resize(num_wheels);
  //      controller_state_pub_->msg_.desired.accelerations.resize(num_wheels);
  //      controller_state_pub_->msg_.desired.effort.resize(num_wheels);
  //
  //      controller_state_pub_->msg_.actual.positions.resize(num_wheels);
  //      controller_state_pub_->msg_.actual.velocities.resize(num_wheels);
  //      controller_state_pub_->msg_.actual.accelerations.resize(num_wheels);
  //      controller_state_pub_->msg_.actual.effort.resize(num_wheels);
  //
  //      controller_state_pub_->msg_.error.positions.resize(num_wheels);
  //      controller_state_pub_->msg_.error.velocities.resize(num_wheels);
  //      controller_state_pub_->msg_.error.accelerations.resize(num_wheels);
  //      controller_state_pub_->msg_.error.effort.resize(num_wheels);
  //
  //      // Convention is the list all left joints in order before all right joints in order
  //      for (size_t i = 0; i < wheel_joint_pairs_.size(); ++i) {
  //        controller_state_pub_->msg_.joint_names[i] = left_wheel_names[i];
  //        controller_state_pub_->msg_.joint_names[i + wheel_joint_pairs_.size()] = right_wheel_names[i];
  //      }
  //
  //      vel_left_previous_.resize(wheel_joint_pairs_.size(), 0.0);
  //      vel_right_previous_.resize(wheel_joint_pairs_.size(), 0.0);
  //  }

  // Get the joint object to use in the realtime loop
  for (size_t i = 0; i < wheel_joint_pairs_.size(); i++) {
    ROS_INFO_STREAM_NAMED(
        name_, "Adding left wheel with joint name: " << left_wheel_names[i]
                                                     << " and right wheel with joint name: " << right_wheel_names[i]);
    wheel_joint_pairs_[i].left_wheel_joint = hw->getHandle(left_wheel_names[i]);    // throws on failure
    wheel_joint_pairs_[i].right_wheel_joint = hw->getHandle(right_wheel_names[i]);  // throws on failure
  }

  command_subscriber_ =
      controller_nh.subscribe("cmd_open_loop", 1, &DiffDriveVoltageController::commandSubscriberCallback, this);

  return true;
}

void DiffDriveVoltageController::starting(const ros::Time& time) {
  stopMotors();
}

void DiffDriveVoltageController::stopping(const ros::Time& time) {
  stopMotors();
}

void DiffDriveVoltageController::update(const ros::Time& time, const ros::Duration& period) {
  Command current_cmd = *(command_realtime_buffer_.readFromRT());
  const double cmd_dt{(time - current_cmd.timestamp).toSec()};

  // Set command to 0 if subscribed command has timed out
  if (cmd_dt > cmd_timeout_) {
    current_cmd.linear = 0.0;
    current_cmd.angular = 0.0;
  }

  // Limit input commands
  if (has_linear_cmd_limits_) {
    current_cmd.linear = std::clamp(current_cmd.linear, min_linear_cmd_, max_linear_cmd_);
  }
  if (has_angular_cmd_limits_) {
    current_cmd.angular = std::clamp(current_cmd.angular, min_angular_cmd_, max_angular_cmd_);
  }

  // Publish clamped input commands:
  if (publish_controller_cmd_output_ && output_command_publisher_ && output_command_publisher_->trylock()) {
    output_command_publisher_->msg_.linear.x = current_cmd.linear;
    output_command_publisher_->msg_.angular.z = current_cmd.angular;
    output_command_publisher_->unlockAndPublish();
  }

  // Compute wheel commands:
  double vel_left = current_cmd.linear - current_cmd.angular;
  double vel_right = current_cmd.linear + current_cmd.angular;

  // Clamp outputs commands to be within [-1.0, 1.0]
  current_cmd.linear = std::clamp(current_cmd.linear, -1.0, 1.0);
  current_cmd.angular = std::clamp(current_cmd.angular, -1.0, 1.0);

  // Set wheels velocities:
  for (auto& joint_pair : wheel_joint_pairs_) {
    joint_pair.left_wheel_joint.setCommand(vel_left);
    joint_pair.right_wheel_joint.setCommand(vel_right);
  }
}

bool DiffDriveVoltageController::loadWheelParameters(ros::NodeHandle& controller_nh,
                                                     std::vector<std::string>& left_wheel_names,
                                                     std::vector<std::string>& right_wheel_names) {
  // Get wheel joint names from the parameter server
  if (!getWheelNames(controller_nh, "left_wheel", left_wheel_names) ||
      !getWheelNames(controller_nh, "right_wheel", right_wheel_names)) {
    return false;
  }

  // Number of Wheels on each side must be same for controller math to be valid
  if (left_wheel_names.size() != right_wheel_names.size()) {
    ROS_ERROR_STREAM_NAMED(name_, "#left wheels (" << left_wheel_names.size() << ") != "
                                                   << "#right wheels (" << right_wheel_names.size() << ").");
    return false;
  }

  wheel_joint_pairs_.resize(left_wheel_names.size());

  // TODO(azumnanji? or wmmc88): Add function here to grab wheel_separation and wheel radii from urdf. Should probably
  // issue a warning if both sources of info exist(urdf should take precedence). If only one exists, there should be a
  // console log that says where things got loaded from.

  return true;
}

bool DiffDriveVoltageController::getWheelNames(ros::NodeHandle& controller_nh, const std::string& wheel_param,
                                               std::vector<std::string>& wheel_names) {
  XmlRpc::XmlRpcValue wheel_list;
  if (!controller_nh.getParam(wheel_param, wheel_list)) {
    ROS_ERROR_STREAM_NAMED(name_, "Couldn't retrieve wheel param '" << wheel_param << "'.");
    return false;
  }

  if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeArray) {
    if (wheel_list.size() == 0) {
      ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param << "' is an empty list");
      return false;
    }

    for (int i = 0; i < wheel_list.size(); ++i) {
      if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param << "' #" << i << " isn't a string.");
        return false;
      }
    }

    wheel_names.resize(wheel_list.size());
    for (int i = 0; i < wheel_list.size(); ++i) {
      wheel_names[i] = static_cast<std::string>(wheel_list[i]);
    }
  } else if (wheel_list.getType() == XmlRpc::XmlRpcValue::TypeString) {
    wheel_names.push_back(wheel_list);
  } else {
    ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param << "' is neither a list of strings nor a string.");
    return false;
  }

  return true;
}

void DiffDriveVoltageController::commandSubscriberCallback(
    const uwrt_mars_rover_drivetrain_msgs::OpenLoopTwist& command) {
  if (isRunning()) {
    if (!allow_multiple_cmd_publishers_ && command_subscriber_.getNumPublishers() > 1) {
      ROS_ERROR_STREAM_THROTTLE_NAMED(1.0, name_,
                                      "Detected " << command_subscriber_.getNumPublishers() << " publishers on \""
                                                  << command_subscriber_.getTopic()
                                                  << "\". Only 1 publisher is allowed. Going to stop motors.");
      stopMotors();
      return;
    }

    if (!std::isfinite(command.angular.z) || !std::isfinite(command.linear.x)) {
      ROS_WARN_THROTTLE_NAMED(1.0, name_, "Received NaN in velocity command. Ignoring.");
      return;
    }

    // todo for position controller: implement sticky idea? have a reset field in the message that rezeros and doesn't
    // apply offsets until a second press? or maybe until not pressed?
    command_struct_.angular = command.angular.z;
    command_struct_.linear = command.linear.x;
    command_struct_.timestamp = ros::Time::now();
    command_realtime_buffer_.writeFromNonRT(command_struct_);
    ROS_DEBUG_STREAM_NAMED(name_, "Added values to command. "
                                      << "Ang: " << command_struct_.angular << ", "
                                      << "Lin: " << command_struct_.linear << ", "
                                      << "Stamp: " << command_struct_.timestamp);
  } else {
    ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
  }
}

void DiffDriveVoltageController::stopMotors() {
  constexpr double STOP_COMMAND_VALUE{0.0};
  for (auto& joint_pair : wheel_joint_pairs_) {
    joint_pair.left_wheel_joint.setCommand(STOP_COMMAND_VALUE);
    joint_pair.right_wheel_joint.setCommand(STOP_COMMAND_VALUE);
  }
}
}  // namespace diff_drive_voltage_controller
PLUGINLIB_EXPORT_CLASS(diff_drive_voltage_controller::DiffDriveVoltageController, controller_interface::ControllerBase)
