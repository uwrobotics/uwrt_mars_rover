#pragma once
#include "ros/ros.h"
// do not push, do i need this #include "uwrt_mars_rover_msgs/pid_tuning_api_message.h"
// do not push do i need this #include "uwrt_mars_rover_msgs/pid_tuning_api_service.h"
#include "uwrt_utils/uwrt_can.h"

class pidTuningApi {
 private:
  ros::NodeHandle& _nh;
  const ros::Rate _loop_rate;
  const std::string _log_filter;
  uwrt_utils::UWRTCANWrapper CANMsg;
  // public:
  bool updatePIDParam(uwrt_mars_rover_msgs::pid_tuning_api::Request& req);

 public:
  pidApi(const ros::NodeHandle& nh, const uint8_t loop_rate, const std::string& log_filter = "pidApiServiceServer",
         const std::string& can_interface = "can0");
  void run();
};