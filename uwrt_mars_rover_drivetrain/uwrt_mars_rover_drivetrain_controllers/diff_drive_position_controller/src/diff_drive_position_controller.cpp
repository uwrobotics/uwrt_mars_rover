#include <diff_drive_position_controller/diff_drive_position_controller.h>
#include <uwrt_mars_rover_utils/uwrt_params.h>

#include <algorithm>
#include <pluginlib/class_list_macros.hpp>

namespace diff_drive_position_controller {
// static constexpr class members must have definitions outside of their class to compile. This can be removed in C++17
constexpr double DiffDrivePositionController::DEFAULT_CMD_TIMEOUT;
constexpr bool DiffDrivePositionController::DEFAULT_ALLOW_MULTIPLE_CMD_PUBLISHERS;
constexpr bool DiffDrivePositionController::DEFAULT_PUBLISH_CONTROLLER_CMD_OUTPUT;
constexpr bool DiffDrivePositionController::DEFAULT_PUBLISH_WHEEL_JOINT_CONTROLLER_STATE;

bool DiffDrivePositionController::init(hardware_interface::PositionJointInterface* hw, ros::NodeHandle& controller_nh) {
  name_ = uwrt_mars_rover_utils::getLoggerName(controller_nh);

  std::vector<std::string> left_wheel_names, right_wheel_names;
  if (!loadWheelParameters(controller_nh, left_wheel_names, right_wheel_names)) {
    return false;
  }

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
  allow_multiple_cmd_publishers_ = uwrt_mars_rover_utils::getParam(
      controller_nh, name_, "allow_multiple_cmd_publishers", DEFAULT_ALLOW_MULTIPLE_CMD_PUBLISHERS);
  publish_controller_cmd_output_ = uwrt_mars_rover_utils::getParam(
      controller_nh, name_, "publish_controller_output_cmd", DEFAULT_PUBLISH_CONTROLLER_CMD_OUTPUT);

  if (publish_controller_cmd_output_) {
    output_command_publisher_.reset(
        new realtime_tools::RealtimePublisher<uwrt_mars_rover_drivetrain_msgs::PositionTwist>(
            controller_nh, "cmd_open_loop_out", 100));
  }

  publish_wheel_joint_controller_state_ = uwrt_mars_rover_utils::getParam(
      controller_nh, name_, "publish_wheel_joint_controller_state", DEFAULT_PUBLISH_WHEEL_JOINT_CONTROLLER_STATE);
  if (publish_wheel_joint_controller_state_) {
    // TODO: Add controller_state_publisher_.reset here in #121 (controller_state_publisher_.reset(new
    // realtime_tools::RealtimePublisher<uwrt_mars_rover_msgs::JointTrajectoryControllerState>(controller_nh,
    // "wheel_joint_controller_state", 100)))
  }

  // Get the joint object to use in the realtime loop
  for (size_t i = 0; i < wheel_joint_pairs_.size(); i++) {
    ROS_INFO_STREAM_NAMED(
        name_, "Adding left wheel with joint name: " << left_wheel_names[i]
                                                     << " and right wheel with joint name: " << right_wheel_names[i]);
    wheel_joint_pairs_[i].joint_handles.left_wheel_joint = hw->getHandle(left_wheel_names[i]);    // throws on failure
    wheel_joint_pairs_[i].joint_handles.right_wheel_joint = hw->getHandle(right_wheel_names[i]);  // throws on failure
  }

  waiting_for_setpoint_reset_ = false;
  accepting_offset_commands_ = false;

  command_subscriber_ =
      controller_nh.subscribe("cmd_open_loop", 1, &DiffDrivePositionController::commandSubscriberCallback, this);

  reset_setpoint_service_server_ = controller_nh.advertiseService(
      "reset_position_setpoint", &DiffDrivePositionController::resetPositionSetpointCallback, this);

  return true;
}

void DiffDrivePositionController::starting(const ros::Time& time) {
  stopMotors();
}

void DiffDrivePositionController::stopping(const ros::Time& time) {
  stopMotors();
}

void DiffDrivePositionController::update(const ros::Time& time, const ros::Duration& period) {
  // Re-zero the position setpoint if request was received by service server. Signals success to use in service reply.
  bool needs_setpoint_reset = waiting_for_setpoint_reset_.exchange(false);
  if (needs_setpoint_reset) {
    resetPositionSetpoints();
    accepting_offset_commands_ = false;
  }

  // Start accepting offset commands again if the offset commands have reset to 0
  Command current_cmd = *(command_offset_realtime_buffer_.readFromRT());
  if (!accepting_offset_commands_) {
    static constexpr double LINEAR_CLOSE_THRESHOLD{0.001};   // 1 mm
    static constexpr double ANGULAR_CLOSE_THRESHOLD{0.002};  // ~0.11 degrees
    if (is_close(current_cmd.linear, 0, LINEAR_CLOSE_THRESHOLD) &&
        is_close(current_cmd.angular, 0, ANGULAR_CLOSE_THRESHOLD)) {
      accepting_offset_commands_ = true;
    }
  }

  // Set command to 0 if subscribed command has timed out or if waiting for offset command to reset to 0
  const double cmd_dt{(time - current_cmd.timestamp).toSec()};
  if (cmd_dt > cmd_timeout_ || !accepting_offset_commands_) {
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

  // Compute wheel position offsets:
  const double pos_offset_left = (current_cmd.linear - current_cmd.angular * wheel_separation_ / 2.0) / wheel_radius_;
  const double pos_offset_right = (current_cmd.linear + current_cmd.angular * wheel_separation_ / 2.0) / wheel_radius_;

  // Set wheel positions:
  for (auto& joint_pair : wheel_joint_pairs_) {
    double left_command = joint_pair.joint_setpoints.left_wheel_setpoint + pos_offset_left;
    double right_command = joint_pair.joint_setpoints.right_wheel_setpoint + pos_offset_right;

    joint_pair.joint_handles.left_wheel_joint.setCommand(left_command);
    joint_pair.joint_handles.right_wheel_joint.setCommand(right_command);
  }

  // Publish Controller State
  // TODO in #121:
  // if publish_wheel_joint_controller_state_ && controller_state_publisher_->trylock(), publish controller state
}

bool DiffDrivePositionController::loadWheelParameters(ros::NodeHandle& controller_nh,
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

  if (!controller_nh.getParam("wheel_separation", wheel_separation_)) {
    return false;
  }

  if (!controller_nh.getParam("wheel_radius", wheel_radius_)) {
    return false;
  }

  // TODO(azumnanji? or wmmc88): Add function here to grab wheel params from urdf. Should probably
  // issue a warning if both sources of info exist(urdf should take precedence). If only one exists, there should be a
  // console log that says where things got loaded from.

  return true;
}

bool DiffDrivePositionController::getWheelNames(ros::NodeHandle& controller_nh, const std::string& wheel_param,
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

    for (int i = 0; i < wheel_list.size(); i++) {
      if (wheel_list[i].getType() != XmlRpc::XmlRpcValue::TypeString) {
        ROS_ERROR_STREAM_NAMED(name_, "Wheel param '" << wheel_param << "' #" << i << " isn't a string.");
        return false;
      }
    }

    wheel_names.resize(wheel_list.size());
    for (int i = 0; i < wheel_list.size(); i++) {
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

void DiffDrivePositionController::commandSubscriberCallback(
    const uwrt_mars_rover_drivetrain_msgs::PositionTwist& command) {
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
      ROS_WARN_THROTTLE_NAMED(1.0, name_, "Received NaN in open loop command. Ignoring.");
      return;
    }

    command_struct_.angular = command.angular.z;
    command_struct_.linear = command.linear.x;
    command_struct_.timestamp = ros::Time::now();
    command_offset_realtime_buffer_.writeFromNonRT(command_struct_);
    ROS_DEBUG_STREAM_NAMED(name_, "Added values to command. "
                                      << "Ang: " << command_struct_.angular << ", "
                                      << "Lin: " << command_struct_.linear << ", "
                                      << "Stamp: " << command_struct_.timestamp);
  } else {
    ROS_ERROR_NAMED(name_, "Can't accept new commands. Controller is not running.");
  }
}

bool DiffDrivePositionController::resetPositionSetpointCallback(std_srvs::Trigger::Request& request,
                                                                std_srvs::Trigger::Response& response) {
  static const ros::Duration RESET_SETPOINT_TIMEOUT{0.2};

  waiting_for_setpoint_reset_ = true;

  ros::Time current_time;
  ros::ros_steadytime(current_time.sec, current_time.nsec);

  ros::Time timeout_time = current_time + RESET_SETPOINT_TIMEOUT;

  response.success = false;
  while (current_time < timeout_time) {
    if (!waiting_for_setpoint_reset_) {
      // Wait for controller manager thread to reset setpoint
      response.success = true;
      break;
    }

    // update current time
    ros::ros_steadytime(current_time.sec, current_time.nsec);
  }

  if (!response.success) {
    std::stringstream ss;
    ss << "Request Timed Out! Waited for " << RESET_SETPOINT_TIMEOUT.toSec() << " seconds before terminating.";
    response.message = ss.str();
  }

  return true;
}

void DiffDrivePositionController::resetPositionSetpoints() {
  for (auto& joint_pair : wheel_joint_pairs_) {
    joint_pair.joint_setpoints.left_wheel_setpoint = joint_pair.joint_handles.left_wheel_joint.getPosition();
    joint_pair.joint_setpoints.right_wheel_setpoint = joint_pair.joint_handles.right_wheel_joint.getPosition();
  }
}

void DiffDrivePositionController::stopMotors() {
  resetPositionSetpoints();
  for (auto& joint_pair : wheel_joint_pairs_) {
    joint_pair.joint_handles.left_wheel_joint.setCommand(joint_pair.joint_setpoints.left_wheel_setpoint);
    joint_pair.joint_handles.right_wheel_joint.setCommand(joint_pair.joint_setpoints.right_wheel_setpoint);
  }
}

bool DiffDrivePositionController::is_close(double value_1, double value_2, double threshold) {
  return std::abs(value_1 - value_2) < threshold;
}
}  // namespace diff_drive_position_controller
PLUGINLIB_EXPORT_CLASS(diff_drive_position_controller::DiffDrivePositionController,
                       controller_interface::ControllerBase)
