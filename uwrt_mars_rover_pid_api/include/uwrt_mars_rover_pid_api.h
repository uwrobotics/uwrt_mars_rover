#pragma once
#include <string>
#include "ros/ros.h"
using namespace std::literals::chrono_literals;

class pidApi {
    private:
    ros::ServiceServer _ss;
    ros::NodeHandle _nh;
    ros::Rate _loop_rate;
    public:
    pidApi(const ros::NodeHandle &nh, auto loop_rate);
    void run();
};