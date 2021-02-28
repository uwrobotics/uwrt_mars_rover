#include <uwrt_mars_rover_drivetrain_hw/uwrt_mars_rover_drivetrain_hw_real.h>
#include <uwrt_mars_rover_utils/uwrt_params.h>

#include <pluginlib/class_list_macros.hpp>

namespace uwrt_mars_rover_drivetrain_hw {

bool UWRTMarsRoverDrivetrainHWReal::init(ros::NodeHandle &root_nh, ros::NodeHandle &robot_hw_nh) {
  if (!UWRTRoverHWDrivetrain::init(root_nh, robot_hw_nh)) {
    return false;
  }

  if (!loadRoboteqConfigFromParamServer(robot_hw_nh)) {
    return false;
  }

  std::unique_ptr<roboteq::CanopenInterface> comm = std::make_unique<roboteq::CanopenInterface>(
      roboteq_canopen_id_, uwrt_mars_rover_utils::getParam<std::string>(root_nh, name_, "can_interface_name", "can0"));
  motor_controller_ = std::make_unique<roboteq::RoboteqController>(std::move(comm));
  return true;
}

void UWRTMarsRoverDrivetrainHWReal::read(const ros::Time & /*time*/, const ros::Duration & /*period*/) {
  static constexpr double MOTOR_READING_TO_AMPS_CONVERSION_FACTOR{10.0};
  static constexpr double RPM_TO_RADIANS_PER_SECOND_FACTOR{2 * M_PI / 60};
  static constexpr double REVOLUTIONS_TO_RADIANS_FACTOR{2 * M_PI};

  // TODO: change to use tpdos instead of queries
  for (const auto &joint_name : joint_names_) {
    // position measured in Radians from starting position
    try {
      actuator_joint_states_[joint_name].actuator_position =
          REVOLUTIONS_TO_RADIANS_FACTOR *
          motor_controller_->readAbsoluteEncoderCount(roboteq_actuator_index_[joint_name]) /
          ticks_per_revolution_[joint_name];
    } catch (const std::exception &e) {
      // TODO: replace roboteq exceptions with ros diagnostics!! #106
      ROS_ERROR_STREAM_NAMED(name_, "CAUGHT EXCEPTION: " << e.what());
    }

    // velocity in Radians/sec
    try {
      actuator_joint_states_[joint_name].actuator_velocity =
          motor_controller_->readEncoderMotorSpeed(roboteq_actuator_index_[joint_name]) *
          RPM_TO_RADIANS_PER_SECOND_FACTOR;
    } catch (const std::exception &e) {
      // TODO: replace roboteq exceptions with ros diagnostics!! #106
      ROS_ERROR_STREAM_NAMED(name_, "CAUGHT EXCEPTION: " << e.what());
    }

    // TODO #121: populate actuator current and voltage to new state interface
    // motor_controller_->readMotorAmps(roboteq_actuator_index_[joint_name]) * MOTOR_READING_TO_AMPS_CONVERSION_FACTOR;

    //  TODO: #121 maybe? add interface for motor fault flags
    // motor_controller_->readMotorStatusFlags(roboteq_actuator_index_[joint_name]);
  }
  actuator_to_joint_state_interface_.propagate();

  //  TODO: #121 maybe? add interface for motor fault flags
  //  motor_controller_->readFaultFlags();
}

void UWRTMarsRoverDrivetrainHWReal::write(const ros::Time & /*time*/, const ros::Duration & /*period*/) {
  static constexpr double RADIANS_PER_SECOND_TO_RPM_FACTOR{60 / M_PI / 2};
  static constexpr double REVOLUTIONS_PER_RADIAN{1 / (2 * M_PI)};

  for (const auto &joint_name : joint_names_) {
    bool successful_joint_write = false;
    switch (actuator_joint_commands_[joint_name].type) {
      case UWRTRoverHWDrivetrain::DrivetrainActuatorJointCommand::Type::POSITION:
        joint_to_actuator_position_interface_.propagate();

        // FIxME: This will cause catastrophic behaviour when the counter rolls over. At the rollover point, our
        // diff_drive_position_controller will cause the rover to suddenly drive full speed in the opposite direction it
        // was already driving for very very long time... (almost 23km). The good news is that this only happens after
        // the rover drives continuously in one direction for ~23km in the current configuration. 🧐💩💩
        successful_joint_write = motor_controller_->setPosition(
            static_cast<int32_t>(actuator_joint_commands_[joint_name].actuator_data * REVOLUTIONS_PER_RADIAN *
                                 ticks_per_revolution_[joint_name]),
            roboteq_actuator_index_[joint_name]);
        break;

      case UWRTRoverHWDrivetrain::DrivetrainActuatorJointCommand::Type::VELOCITY:
        joint_to_actuator_velocity_interface_.propagate();
        try {
          successful_joint_write =
              motor_controller_->setVelocity(static_cast<int32_t>(actuator_joint_commands_[joint_name].actuator_data *
                                                                  RADIANS_PER_SECOND_TO_RPM_FACTOR),
                                             roboteq_actuator_index_[joint_name]);
        } catch (const std::exception &e) {
          // TODO: replace roboteq exceptions with ros diagnostics!! #106
          ROS_ERROR_STREAM_NAMED(name_, "CAUGHT EXCEPTION: " << e.what());
        }

        break;

      case UWRTRoverHWDrivetrain::DrivetrainActuatorJointCommand::Type::VOLTAGE:
        // No need for joint_to_actuator propagation for voltage commands. Data is already in actuator space.
        actuator_joint_commands_[joint_name].actuator_data = actuator_joint_commands_[joint_name].joint_data;

        successful_joint_write =
            motor_controller_->setMotorCommand(static_cast<int32_t>(actuator_joint_commands_[joint_name].actuator_data),
                                               roboteq_actuator_index_[joint_name]);
        break;

      case UWRTRoverHWDrivetrain::DrivetrainActuatorJointCommand::Type::NONE:
        ROS_DEBUG_STREAM_NAMED(name_, joint_name << " has a " << actuator_joint_commands_[joint_name].type
                                                 << " command type. Sending Stop Command to Roboteq Controller!");
        successful_joint_write = motor_controller_->stopInAllModes(roboteq_actuator_index_[joint_name]);
        break;

      default:
        ROS_ERROR_STREAM_NAMED(
            name_, joint_name << " has a joint command with index "
                              << static_cast<int>(actuator_joint_commands_[joint_name].type)
                              << ",which is an unknown command type. Sending Stop Command to Roboteq Controller!");
        successful_joint_write = motor_controller_->stopInAllModes(roboteq_actuator_index_[joint_name]);
    }
    if (!successful_joint_write && actuator_joint_commands_[joint_name].type !=
                                       UWRTRoverHWDrivetrain::DrivetrainActuatorJointCommand::Type::NONE) {
      ROS_ERROR_STREAM_NAMED(name_, "Failed to write " << actuator_joint_commands_[joint_name].type << " command to "
                                                       << joint_name << ".");
    }
  }
}

void UWRTMarsRoverDrivetrainHWReal::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
              const std::list<hardware_interface::ControllerInfo> &stop_list) {
  uint8_t MAX_WHILE {100};
  uint8_t count_while {0};
  uint8_t target_mode {0};
  for (const auto &controller : start_list) {
    for (const auto &claimed : controller.claimed_resources) {
      for (const auto &joint_name : claimed.resources) {
        if (claimed.hardware_interface == "hardware_interface::PositionJointInterface") {
          target_mode = 3;
        } else if (claimed.hardware_interface == "hardware_interface::VelocityJointInterface") {
          target_mode = 1;
        } else if (claimed.hardware_interface == "uwrt_hardware_interface::VoltageJointInterface") {
          target_mode = 2;
        }
      }
    }
  }
  uint8_t received_msg1 {0};
  uint8_t received_msg2 {0};

  if (target_mode != 0){
    do{
      motor_controller_->setUserIntVariable(target_mode, 9);
      received_msg1 = motor_controller_->readUserIntegerVariable(1);
      received_msg2 = motor_controller_->readUserIntegerVariable(2);
      count_while++;
    }while((received_msg1 != target_mode || received_msg2 != target_mode) && count_while < MAX_WHILE);
  }

  UWRTRoverHWDrivetrain::doSwitch(start_list, stop_list);
}

bool UWRTMarsRoverDrivetrainHWReal::loadRoboteqConfigFromParamServer(ros::NodeHandle &robot_hw_nh) {
  roboteq_canopen_id_ = uwrt_mars_rover_utils::getParam<int>(robot_hw_nh, name_, "roboteq_canopen_id", 0x01);

  // Get joint list info
  XmlRpc::XmlRpcValue joints_list;
  bool param_fetched = robot_hw_nh.getParam("joints", joints_list);
  if (!param_fetched) {
    ROS_WARN_STREAM_NAMED(name_, robot_hw_nh.getNamespace() << "/joints could not be loaded from parameter server.");
    return false;
  }
  ROS_DEBUG_STREAM_NAMED(name_, robot_hw_nh.getNamespace() << "/joints loaded from parameter server.");
  ROS_ASSERT(joints_list.getType() == XmlRpc::XmlRpcValue::TypeArray);
  // NOLINTNEXTLINE(modernize-loop-convert): iterator only valid for XmlRpcValue::TypeStruct
  for (size_t joint_index = 0; joint_index < joints_list.size(); joint_index++) {
    ROS_ASSERT(joints_list[joint_index].getType() == XmlRpc::XmlRpcValue::TypeStruct);
    ROS_ASSERT(joints_list[joint_index].hasMember("name"));
    ROS_ASSERT(joints_list[joint_index]["name"].getType() == XmlRpc::XmlRpcValue::TypeString);
    std::string joint_name = joints_list[joint_index]["name"];

    ROS_ASSERT(joints_list[joint_index].hasMember("roboteq_index"));
    ROS_ASSERT(joints_list[joint_index]["roboteq_index"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    roboteq_actuator_index_[joint_name] = static_cast<int>(joints_list[joint_index]["roboteq_index"]);

    ROS_ASSERT(joints_list[joint_index].hasMember("encoder_ticks_per_revolution"));
    ROS_ASSERT(joints_list[joint_index]["encoder_ticks_per_revolution"].getType() == XmlRpc::XmlRpcValue::TypeInt);
    ticks_per_revolution_[joint_name] = static_cast<int>(joints_list[joint_index]["encoder_ticks_per_revolution"]);
  }
  return true;
}
}  // namespace uwrt_mars_rover_drivetrain_hw
PLUGINLIB_EXPORT_CLASS(uwrt_mars_rover_drivetrain_hw::UWRTMarsRoverDrivetrainHWReal, hardware_interface::RobotHW)
