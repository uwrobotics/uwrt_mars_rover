#include "uwrt_mars_rover_pid_tuning_api.h"
#include "uwrt_mars_rover_hw_bridge/hw_bridge.h"
#include <algorithm>
#include <iterator> 
#include <string>
#include <vector>
#include <ros/ros.h>
#include <stdio.h>
#include <ctype.h>
#include <boost/algorithm/string.hpp>

pidTuningApi::pidTuningApi(ros::NodeHandle &nh, const uint8_t loop_rate, const std::string &log_filter,
                           const std::string &can_interface)
    : _nh(nh),
      _loop_rate(loop_rate),
      _log_filter(log_filter),
      CANMsg(uwrt_utils::UWRTCANWrapper("pid_tuning_api_can", can_interface, true)) {
    CANMsg.init(std::vector<uint32_t>{HWBRIDGE::CANID::SET_JOINT_PID_P, HWBRIDGE::CANID::SET_JOINT_PID_I,
                                    HWBRIDGE::CANID::SET_JOINT_PID_D, HWBRIDGE::CANID::SET_PID_DEADZONE,
                                    HWBRIDGE::CANID::SET_JOINT_PID_BIAS});
}

bool pidTuningApi::updatePIDParam(uwrt_mars_rover_msgs::pid_tuning_api_srv::Request& req, 
uwrt_mars_rover_msgs::pid_tuning_api_srv::Response& res) {
  ROS_INFO_NAMED(_log_filter, "Updating PID gains");
  if (!isValidPayload(req.pid_tuning_api_data)) {
    ROS_WARN_NAMED(_log_filter, "Invalid PID payload. Service call failed. Message not sent");
    return false;
  }
  HWBRIDGE::ARM::PID::tuningApiPayload payload{.value = req.pid_tuning_api_data.value,
                                               .velocity = req.pid_tuning_api_data.isVelocityPID,
                                               .actuatorID = req.pid_tuning_api_data.actuatorID};
  int32_t can_id;
  std::string lower_case_parameter = boost::to_lower_copy(req.pid_tuning_api_data.parameter);
  if (lower_case_parameter == "deadzone") 
      can_id = 1873;
  else if (lower_case_parameter == "p")
      can_id = 1874;
  else if (lower_case_parameter == "i")
      can_id = 1875;
  else if (lower_case_parameter == "d")    
      can_id = 1876;
  else
      can_id = 1877;
  return CANMsg.writeToID<HWBRIDGE::ARM::PID::tuningApiPayload>(payload, can_id);
}

bool pidTuningApi::isValidPayload(uwrt_mars_rover_msgs::pid_tuning_api_msg & payload_data) {
  std::string parameters[5] {"p", "i", "d", "bias", "deadzone"};
  bool exists = std::find(std::begin(parameters), std::end(parameters), boost::to_lower_copy(payload_data.parameter)) != std::end(parameters);
  return exists && payload_data.actuatorID >= 0 && payload_data.actuatorID <= 5;
}

void pidTuningApi::run() {
  ros::ServiceServer _ss = _nh.advertiseService("pid_tuning_api", &pidTuningApi::updatePIDParam, this);
  ROS_INFO_NAMED(_log_filter, "Ready to update PID gain paramaters.");
  ros::spin();
}