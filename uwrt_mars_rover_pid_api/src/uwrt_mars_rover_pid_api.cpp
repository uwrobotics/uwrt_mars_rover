#include "uwrt_mars_rover_pid_api.h"
#define LOG_FILTER "pidApi"
#ifdef PID_API_TESTING
#define CAN_INTERFACE "vcan0"
#else
#define CAN_INTERFACE "can0"
#endif

pidApi::pidApi(const ros::NodeHandle &nh, float loop_rate) : _nh(nh), _loop_rate(loop_rate) {
    CANMsg = uwrt_utils::UWRTCANWrapper("pid_api_can", CAN_INTERFACE, true);
    CANMsg.init(NULL);
}

void pidApi::updatePIDGain(const uwrt_mars_rover_msgs::pid_api::Request &req, uwrt_mars_rover_msgs::pid_api::Response &res) {
    // use req to send a canmsg
}

void pidApi::run() {
  _ss = _nh.advertiseService("neopixel_set", &Neopixel::setState, this);
  ROS_INFO_NAMED(LOG_FILTER, "Ready to update PID gain paramaters.");
  while (ros::ok()) {
    ros::spinOnce();
    _loop_rate.sleep();
  }
}