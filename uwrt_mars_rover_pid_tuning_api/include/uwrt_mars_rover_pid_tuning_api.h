#pragma once
#include "ros/ros.h"
#include "uwrt_mars_rover_msgs/PidTuningApi.h"
#include "uwrt_mars_rover_msgs/pid_tuning_api.h"
#include "uwrt_utils/uwrt_can.h"

class PidTuningApi {
 private:
  ros::NodeHandle& _nh;
  const ros::Rate _loop_rate;
  const std::string _log_filter;
  uwrt_utils::UWRTCANWrapper CANMsg;
  // public:
  bool updatePIDParam(uwrt_mars_rover_msgs::pid_tuning_api::Request& req,
                      uwrt_mars_rover_msgs::pid_tuning_api::Response& res);
  bool isValidPayload(uwrt_mars_rover_msgs::PidTuningApi& payload_data);

 public:
  PidTuningApi(ros::NodeHandle& nh, uint8_t loop_rate, std::string log_filter = "pidApiServiceServer",
               std::string can_interface = "can0");
  void run();
};