#include "uwrt_mars_rover_pid_tuning_api.h"
#include "uwrt_mars_rover_hw_bridge/hw_bridge.h"

pidTuningApi::pidTuningApi(const ros::NodeHandle &nh, const uint8_t loop_rate, const std::string &log_filter,
                           const std::string &can_interface)
    : _nh(nh),
      _loop_rate(loop_rate),
      _log_filter(log_filter),
      CANMsg(uwrt_utils::UWRTCANWrapper("pid_tuning_api_can", can_interface, true)) {
  CANMsg.init(std::vector<uint16_t>{HWBRIDGE::CANID::SET_JOINT_PID_P, HWBRIDGE::CANID::SET_JOINT_PID_I,
                                    HWBRIDGE::CANID::SET_JOINT_PID_D, HWBRIDGE::CANID::SET_JOINT_PID_DEADZONE,
                                    HWBRIDGE::CANID::SET_JOINT_PID_BIAS});
}

bool pidTuningApi::updatePIDParam(uwrt_mars_rover_msgs::pid_tuning_api::Request &req) {
  ROS_WARN_NAMED(_log_filter, "Invalid PID parameter. Service call failed. Message not sent");
  ROS_INFO_STREAM("Updating PID gains: " + HWBRIDGE::ARM::PID::str(req.pid_tuning_api_data));
  if (!isValidPayload(req.pid_tuning_api_data)) {
    ROS_WARN_NAMED(_log_filter, "Invalid PID payload. Service call failed. Message not sent");
    return false;
  }
  HWBRDIGE::ARM::PID::tuningApiPayload payload{.value = req.pid_tuning_api_data.value,
                                               .isVelocityPID = req.pid_tuning_api_data.isVelocityPID,
                                               .actuatorID = req.pid_tuning_api_data.actuatorID};
  return CANMsg.writeToID<HWBRIDGE::ARM::PID::tuningApiPayload>(payload, req.pid_tuning_api_data.parameter);
}

void pidTuningApi::run() {
  ros::ServiceServer _ss = _nh.advertiseService("pid_tuning_api", &pidApi::updatePIDParam, this);
  ROS_INFO_NAMED(_log_filter, "Ready to update PID gain paramaters.");
  while (ros::ok()) {
    ros::spinOnce();
    _loop_rate.sleep();
  }
}