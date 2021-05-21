#include <cstdint>
#include <memory>
#include <pluginlib/class_list_macros.hpp>

#include "uwrt_mars_rover_hw/uwrt_mars_rover_hw_science_real.h"
#include "uwrt_mars_rover_utils/hw_bridge.h"
#include "uwrt_mars_rover_utils/uwrt_can.h"
#include "uwrt_mars_rover_utils/uwrt_params.h"

namespace uwrt_mars_rover_hw {

UWRTRoverHWScienceReal::~UWRTRoverHWScienceReal() {
  delete comm;
  comm = nullptr;
}

bool UWRTRoverHWScienceReal::init(ros::NodeHandle& root_nh, ros::NodeHandle& robot_hw_nh) {
  // initialize real class by making call to UWRTRoverHWScience
  if (!UWRTRoverHWScience::init(root_nh, robot_hw_nh)) {
    return false;
  }

  // load configs from parameter server
  if (!loadScienceFromParamServer(robot_hw_nh)) {
    return false;
  }
  // initialize CAN interface with melvin's library
  uwrt_mars_rover_utils::UWRTCANWrapper* comm = new uwrt_mars_rover_utils::UWRTCANWrapper(
      getName(),
      uwrt_mars_rover_utils::getParam<std::string>(robot_hw_nh, "science_logger", "can_interface_name", "can0"), true);

  return true;
}

// TODO (wraftus) implement
void UWRTRoverHWScienceReal::read(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  // read states from all joints and set them to the correct place in memory

  // structs that includes angle and speed
  MotionReport report;

  for (const auto& name : joint_names_) {
    if (comm->getLatestFromID<UWRTRoverHWScienceReal::MotionReport>(report, CANID::REPORT_ELEVATOR_HEIGHT)) {
      ROS_ERROR_STREAM_NAMED(_name, "Failed to read Science Joint State Member: pos");
    }
    // copy the data from CAN messaged into joint states
    memcpy(&joint_states_[joint_names_].pos, static_cast<double*> & report.position, sizeof(report.position));
    memcpy(&joint_states_[joint_names_].vel, static_cast<double*> & report.velocity, sizeof(report.position));
    // ask melvin how to calculate effort
    float effort{0};
    effort = report.positon * report.velocity;
    memcpy(&joint_states_[joint_names_].eff, static_cast<double*> & effort, sizeof(effort));
    // reset report variable
    memset(&report, 0, sizeof(report));
  }

  // read states from all indices and set them to the correct place in memory
  for (const auto& name : indexer_names_) {
    if (comm->getLatestFromID<UWRTRoverHWScienceReal::MotionReport>(report, CANID::REPORT_GENEVA_INDEX)) {
      ROS_ERROR_STREAM_NAMED(_name, "Failed to read Science Index Joint State Member: raw_pos");
    }
    memcpy(&indexer_states_[indexer_names_].raw_pos, &report.angle, sizeof(report.angle));
    memcpy();
  }
}

// TODO (wraftus) implement
void UWRTRoverHWScienceReal::write(const ros::Time& /*time*/, const ros::Duration& /*period*/) {
  for (const auto& name : joint_names_) {
    bool successful_joint_write{false};
    switch (joint_cmds[joint_names_].type) {
      case UWRTRoverHWScience::ScienceJointCommand::Type::POS:
        // set the elevator height
        successful_joint_write =
            comm->writeToID<double>(joint_cmds_[joint_names_].data, CANID::SET_ELEVATOR_HEIGHT) break;
      case UWRTRoverHWScience::ScienceJointCommand::Type::NONE:
        // stop the elevator - do not move it
        successful_joint_write = comm->writeToID<double>(0.0, CANID::SET_ELEVATOR_HEIGHT) break;
      default:
        ROS_ERROR_STREAM_NAMED(_name, "Invalid Joint Command for Joint: " << joint_names_);
    }
  }

  // did write to science joint fail
  if (!successful_joint_write) {
    ROS_ERROR_STREAM_NAMED(_name, "Write to Science Joint Failed");
  }

  for (const auto& name : indexer_names_) {
    bool successful_index_write{false};
    switch (indexer_cmds_[indexer_names_].type) {
      case UWRTRoverHWScience::ScienceIndexerCommand::Type::NONE:
        // stop indexer - do not move it
        successful_index_write = comm->writeToID<float>(0.0, CANID::SET_GENEVA_INDEX) break;
      case UWRTRoverHWScience::ScienceIndexerCommand::Type::POS:
        // set indexer
        successful_index_write =
            comm->writeToID<float>(indexer_cmds_[indexer_names_].data, CANID::SET_GENEVA_INDEX) break;
      default:
        ROS_ERROR_STREAM_NAMED(_name, "Invalid Indexer Command for Indexer: " << indexer_names_);
        return;
    }
  }

  // did write to science indexer fail
  if (!successful_index_write) {
    ROS_ERROR_STREAM_NAMED(_name, "Write to Science Indexer Failed");
  }
}

/****
 *
 * Apparently I pulled down the wrong config file for science app
 * so since the function to load things from the param server
 * is based on the config file this is the working version of it
 *
 *****/

bool UWRTRoverHWScienceReal::loadScienceFromParamServer(ros::NodeHandle& robot_hw_nh) {
  // get joints from param server - config file
  XmlRpc::XmlRpcValue joint_config bool param_fetched_joint = robot_hw_nh.getParam("joints", joint_config);
  if (!param_fetched) {
    ROS_ERROR_STREAM_NAMED(_name, robot_hw_nh.getNamespace() << "/joint could not be loaded parameter server");
    return false;
  }

  // check the value pulled from param server is valid
  ROS_ASSERT(joint_config.getType == XmlRpc::XmlRpcValue::TypeArray);

  for (size_t joint_index{0}; joint_index < joint_config.size(); joint_index++) {
    // make sure the joint is there
    ROS_ASSERT(joint_config[joint_index].hasMember("science_elevator_joint")	
    joint_names_[joint_index] = static_cast<std::string>(joint_config[joint_index]);
  }
}

}  // namespace uwrt_mars_rover_hw
PLUGINLIB_EXPORT_CLASS(uwrt_mars_rover_hw::UWRTRoverHWScienceReal, hardware_interface::RobotHW)
