#pragma once
#include "uwrt_mars_rover_msgs/NeopixelArrayMode.h"
#include "uwrt_mars_rover_msgs/set_state.h"
#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_can.h"

class Neopixel {
 private:
  // State of neopixels {0:solid_red, 1:solid_blue, 2:flashing_green}
  volatile uint8_t _state{3};
  // Create service
  ros::ServiceServer _neopixel_service;
  // CLI arg count needed for intiliaze_neopixels
  uint8_t _arg_count;
  // CLI arg list needed for intiliaze_neopixels
  char **_arg_list;
  // CAN interface
  char *_can_interface;
  // Create a CAN object to send the can message
  NeopixelCan _neopixel_can_msg;

 public:
  // neopixel constructor
  Neopixel(int argc, char **argv, char *can_interface);
  // Call back function
  bool setState(uwrt_mars_rover_msgs::set_state::Request &req, uwrt_mars_rover_msgs::set_state::Response &res);
  // Runs the while loop
  void run();
};
