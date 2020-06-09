#pragma once
#include "uwrt_utils/uwrt_can.h"
#include "uwrt_mars_rover_msgs/pid_api_message.h"
#include "uwrt_mars_rover_msgs/pid_api_service.h"
#include "ros/ros.h"

#define PID_API_TESTING

class pidApi {
    private:
    ros::NodeHandle _nh;
    ros::Rate _loop_rate;
    std::string _log_filter;
    uwrt_utils::UWRTCANWrapper CANMsg;
    //public:
    bool updatePIDParam(uwrt_mars_rover_msgs::pid_api_service::Request &req, uwrt_mars_rover_msgs::pid_api_service::Response &res);
    public:
    pidApi(const ros::NodeHandle &nh, uint8_t loop_rate, std::string log_filter = "pidApiServiceServer");
    void run();
};