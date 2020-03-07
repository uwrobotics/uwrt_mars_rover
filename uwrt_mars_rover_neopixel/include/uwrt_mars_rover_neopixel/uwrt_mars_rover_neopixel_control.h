#pragma once
#include "uwrt_mars_rover_msgs/NeopixelArrayMode.h"
#include "uwrt_mars_rover_msgs/set_state.h"
#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_can.h"

class Neopixel {
 private:
  // State of neopixels {0:solid_red, 1:solid_blue, 2:flashing_green}
  volatile uint8_t _state{3};
  // Create service
  ros::ServiceServer _ss;
  // ROS Node handle
  ros::NodeHandle _nh;
  // CAN interface
  std::string _can_interface;
  // Filter for log messages
  std::string _log_filter;
  // Outgoing CAN_ID
  uint16_t _neopixel_can_id_outgoing;
  // Rate to run the while loop at (Hz)
  ros::Rate _loop_rate;
  // Create a CAN object to send the can message
  NeopixelCan _neopixel_can_msg;

 public:
  // neopixel constructor
  Neopixel(ros::NodeHandle &nh, uint8_t loop_rate, const std::string &can_interface, const std::string &log_filter,
           uint16_t neopixel_can_id_outgoing);
  // Call back function
  bool setState(uwrt_mars_rover_msgs::set_state::Request &req, uwrt_mars_rover_msgs::set_state::Response &res);
  // Runs the while loop
  void run();
};
