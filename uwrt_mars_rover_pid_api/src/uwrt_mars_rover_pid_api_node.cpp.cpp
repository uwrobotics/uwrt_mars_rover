#include "uwrt_mars_rover_pid_api/uwrt_mars_rover_pid_api.h"

pidApi::pidApi(const ros::NodeHandle &nh, auto loop_rate) : _nh(nh), _loop_rate(loop_rate) {
    // init
}

void pidApi::run() {
    // while ros::ok
}