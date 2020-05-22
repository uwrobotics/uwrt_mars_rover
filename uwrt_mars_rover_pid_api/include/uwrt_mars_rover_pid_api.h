#pragma once
#include "uwrt_can.h"
#include "ros/ros.h"

#define PID_API_TESTING

class pidApi {
    private:
    ros::ServiceServer _ss;
    ros::NodeHandle _nh;
    ros::Rate _loop_rate;
    uwrt_utils::UWRTCANWrapper CANMsg;
    void pidApi::updatePIDGain(uwrt_mars_rover_msgs::set_state::Request &req, uwrt_mars_rover_msgs::set_state::Response &res);
    public:
    pidApi(const ros::NodeHandle &nh, float loop_rate);
    void run();
};