#pragma once
#include "ros/ros.h"
#include "uwrt_mars_rover_msgs/pid_tuning_api_msg.h"
#include "uwrt_mars_rover_msgs/pid_tuning_api_srv.h"
#include "uwrt_utils/uwrt_can.h"

class pidTuningApi {
 private:
  ros::NodeHandle& _nh;
  const ros::Rate _loop_rate;
  const std::string _log_filter;
  uwrt_utils::UWRTCANWrapper CANMsg;
  // public:
  bool updatePIDParam(uwrt_mars_rover_msgs::pid_tuning_api_srv::Request& req, uwrt_mars_rover_msgs::pid_tuning_api_srv::Response& res);
  bool isValidPayload(uwrt_mars_rover_msgs::pid_tuning_api_msg & payload_data);
 public:
  pidTuningApi(ros::NodeHandle& nh, const uint8_t loop_rate, const std::string& log_filter = "pidApiServiceServer",
         const std::string& can_interface = "can0");
  void run();
};