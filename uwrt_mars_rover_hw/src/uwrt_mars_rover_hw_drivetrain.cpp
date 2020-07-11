#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_drivetrain.h"

namespace uwrt_mars_rover_hw {

bool UWRTRoverHWDrivetrain::init(ros::NodeHandle & /*root_nh*/, ros::NodeHandle &robot_hw_nh) {
  if (!loadJointInfoFromParameterServer(robot_hw_nh)) {
    return false;
  }

  for (const auto &joint_name : joint_names_) {
    registerStateInterfacesAndTransmissions(joint_name);
    registerCommandInterfacesAndTransmissions(joint_name);
  }

  this->registerInterface(&joint_state_interface_);
  this->registerInterface(&joint_position_interface_);
  this->registerInterface(&joint_velocity_interface_);

  return true;
}

void UWRTRoverHWDrivetrain::read(const ros::Time & /*time*/, const ros::Duration & /*period*/) {
  ROS_ERROR_STREAM_NAMED(name_, "read function called from "
                                    << name_ << " class. read calls should only happen to the overloaded function.");
}

void UWRTRoverHWDrivetrain::write(const ros::Time & /*time*/, const ros::Duration & /*period*/) {
  ROS_ERROR_STREAM_NAMED(name_, "write function called from "
                                    << name_ << " class. write calls should only happen to the overloaded function.");
}

void UWRTRoverHWDrivetrain::doSwitch(const std::list<hardware_interface::ControllerInfo> &start_list,
                                     const std::list<hardware_interface::ControllerInfo> &stop_list) {
  // Reset commands for joints claimed by stopping controllers
  for (const auto &controller : stop_list) {
    for (const auto &hardware_interface_resource_list : controller.claimed_resources) {
      for (const auto &joint_name : hardware_interface_resource_list.resources) {
        actuator_joint_commands_[joint_name].type = DrivetrainActuatorJointCommand::Type::NONE;
        actuator_joint_commands_[joint_name].joint_data = 0.0;
      }
    }
  }

  // Set command type for joints claimed by starting controllers
  for (const auto &controller : start_list) {
    for (const auto &claimed : controller.claimed_resources) {
      for (const auto &joint_name : claimed.resources) {
        if (claimed.hardware_interface == "hardware_interface::PositionJointInterface") {
          actuator_joint_commands_[joint_name].type = DrivetrainActuatorJointCommand::Type::POSITION;
          actuator_joint_commands_[joint_name].actuator_data = actuator_joint_states_[joint_name].actuator_position;
        } else if (claimed.hardware_interface == "hardware_interface::VelocityJointInterface") {
          actuator_joint_commands_[joint_name].type = DrivetrainActuatorJointCommand::Type::VELOCITY;
          actuator_joint_commands_[joint_name].actuator_data = 0.0;
        }  // TODO(wmmc88): add an open loop interface (voltage?)
      }
    }
  }
}

bool UWRTRoverHWDrivetrain::loadJointInfoFromParameterServer(ros::NodeHandle &robot_hw_nh) {
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
    joint_names_.push_back(joint_name);
    ROS_ASSERT(joints_list[joint_index].hasMember("transmission_reduction"));
    ROS_ASSERT(joints_list[joint_index]["transmission_reduction"].getType() == XmlRpc::XmlRpcValue::TypeDouble);
    joint_transmissions_.emplace(joint_name, joints_list[joint_index]["transmission_reduction"]);
  }
  return true;
}

void UWRTRoverHWDrivetrain::registerStateInterfacesAndTransmissions(const std::string &joint_name) {
  // Register JointStateHandle to the JointStateInterface
  hardware_interface::JointStateHandle joint_state_handle(
      joint_name, &actuator_joint_states_[joint_name].joint_position,
      &actuator_joint_states_[joint_name].joint_velocity, &actuator_joint_states_[joint_name].joint_effort);
  joint_state_interface_.registerHandle(joint_state_handle);

  // Wrap Actuators States
  actuator_state_data_[joint_name].position.push_back(&actuator_joint_states_[joint_name].actuator_position);
  actuator_state_data_[joint_name].velocity.push_back(&actuator_joint_states_[joint_name].actuator_velocity);
  actuator_state_data_[joint_name].effort.push_back(&actuator_joint_states_[joint_name].actuator_effort);

  // Wrap Joint States
  joint_state_data_[joint_name].position.push_back(&actuator_joint_states_[joint_name].joint_position);
  joint_state_data_[joint_name].velocity.push_back(&actuator_joint_states_[joint_name].joint_velocity);
  joint_state_data_[joint_name].effort.push_back(&actuator_joint_states_[joint_name].joint_effort);

  // Register ActuatorToJointStateHandle with wrapped state data to a ActuatorToJointStateInterface
  transmission_interface::ActuatorToJointStateHandle actuator_to_joint_state_handle(
      joint_name, &joint_transmissions_.find(joint_name)->second, actuator_state_data_[joint_name],
      joint_state_data_[joint_name]);
  actuator_to_joint_state_interface_.registerHandle(actuator_to_joint_state_handle);
}

void UWRTRoverHWDrivetrain::registerCommandInterfacesAndTransmissions(const std::string &joint_name) {
  // Register JointHandle associated with a JointStateHandle to command interfaces
  hardware_interface::JointHandle joint_command_handle(joint_state_interface_.getHandle(joint_name),
                                                       &actuator_joint_commands_[joint_name].joint_data);
  joint_position_interface_.registerHandle(joint_command_handle);
  joint_velocity_interface_.registerHandle(joint_command_handle);

  // Wrap Actuator Commands
  actuator_command_data_[joint_name].position.push_back(&actuator_joint_commands_[joint_name].actuator_data);
  actuator_command_data_[joint_name].velocity.push_back(&actuator_joint_commands_[joint_name].actuator_data);

  // Wrap Joint Commands
  joint_command_data_[joint_name].position.push_back(&actuator_joint_commands_[joint_name].joint_data);
  joint_command_data_[joint_name].velocity.push_back(&actuator_joint_commands_[joint_name].joint_data);

  // Register JointToActuatorPositionHandle with wrapped command data to a JointToActuatorPositionInterface
  transmission_interface::JointToActuatorPositionHandle joint_to_actuator_position_handle(
      joint_name, &joint_transmissions_.find(joint_name)->second, actuator_command_data_[joint_name],
      joint_command_data_[joint_name]);
  joint_to_actuator_position_interface_.registerHandle(joint_to_actuator_position_handle);

  // Register JointToActuatorVelocityHandle with wrapped command data to a JointToActuatorVelocityInterface
  transmission_interface::JointToActuatorVelocityHandle joint_to_actuator_velocity_handle(
      joint_name, &joint_transmissions_.find(joint_name)->second, actuator_command_data_[joint_name],
      joint_command_data_[joint_name]);
  joint_to_actuator_velocity_interface_.registerHandle(joint_to_actuator_velocity_handle);
}

}  // namespace uwrt_mars_rover_hw
