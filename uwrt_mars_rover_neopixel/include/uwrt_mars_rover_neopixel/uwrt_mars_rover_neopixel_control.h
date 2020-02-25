#pragma once
#include "uwrt_mars_rover_msgs/set_state.h"
#include "uwrt_mars_rover_msgs/NeopixelArrayMode.h"
#include "uwrt_mars_rover_neopixel/uwrt_mars_rover_neopixel_can.h"

class neopixel{
private:
    // State of neopixels {0:solid_red, 1:solid_blue, 2:flashing_green}
    volatile uint8_t _state{3};
    // Create service
    ros::ServiceServer _neopixel_service;
    // CLI arg count needed for intiliaze_neopixels
    uint8_t _arg_count;
    // CLI arg list needed for intiliaze_neopixels
    char **_arg_list;
    // Create a CAN object to send the can message
    neopixelCan _neopixel_can_msg;
public:
    // neopixel constructor
    neopixel(int argc, char **argv);
    // Sets the state var
    bool setState(uwrt_mars_rover_msgs::set_state::Request &req,
                   uwrt_mars_rover_msgs::set_state::Response &res);
    // Initializes the neopixels
    //void initializeNeopixels();
    // Runs the while loop
    void run();
};